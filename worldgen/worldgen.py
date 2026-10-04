#!/usr/bin/env python3
"""
================================================================================
 worldgen.py  --  Real Earth -> FPGA voxel world generator
================================================================================

 Takes a place name (or lat/lon), downloads real satellite + elevation + weather
 data for that spot, and turns it into a 128x128 block world that a Cyclone V
 FPGA can render in first person.

 Pipeline:

     "Mount Baker"
          |
          v  Open-Meteo Geocoding            -> lat / lon
          v  Copernicus DEM GLO-30 (AWS)     -> real elevation, 30 m per pixel
          v  ESA WorldCover 10m (AWS)        -> what is actually on the ground
          v  Sentinel-2 L2A (Earth Search)   -> NDVI / NDWI / NDSI refinement
          v  Open-Meteo Forecast             -> live weather right now
          v  NASA FIRMS (optional)           -> active wildfire hotspots
          |
          v
     128 x 128 grid, one byte per cell:  [ height 5 bits | block type 3 bits ]
          |
          +--> world.bin   raw 16384 bytes            (UART / SD card)
          +--> world.mif   Quartus memory init file   (synthesise into M10K)
          +--> world.hex   $readmemh file             (simulation / Verilog)
          +--> world.json  spawn point + weather + stats
          +--> preview.txt / preview.png              (so you can SEE it worked)

 Everything degrades gracefully. No network? It uses the cache. No cache? It
 generates a believable procedural world so your demo never shows a stack trace
 in front of a judge.

--------------------------------------------------------------------------------
 INSTALL
--------------------------------------------------------------------------------

     python3 -m venv venv
     source venv/bin/activate          # Windows: venv\\Scripts\\activate

     # Required (small, fast, always works):
     pip install numpy requests

     # Strongly recommended (real elevation + real land cover):
     pip install rasterio

     # Optional (Sentinel-2 vegetation/snow/water refinement):
     pip install pystac-client

     # Optional extras:
     pip install pillow        # PNG preview image for the website / pitch
     pip install pyserial      # send the world to the FPGA over UART

 If `rasterio` will not install, the program automatically falls back to AWS
 Terrain Tiles (PNG elevation tiles, needs only requests + pillow) and then to
 the Open-Meteo elevation API. You will still get a real world.

--------------------------------------------------------------------------------
 USAGE
--------------------------------------------------------------------------------

     # The basic thing
     python worldgen.py "Mount Baker"

     # Pick scale: 30 m per block = 3.8 km across. 120 m = 15 km across.
     python worldgen.py "Vancouver" --scale 120

     # Make the terrain more dramatic for first-person view
     python worldgen.py "Grouse Mountain" --exaggeration 1.8

     # Disaster mode: raise the water by 8 real metres
     python worldgen.py "Richmond, BC" --flood-meters 8

     # Wildfire mode (needs a free FIRMS map key)
     python worldgen.py "Kelowna" --fires --firms-key YOUR_KEY

     # Turn on the Sentinel-2 layer
     python worldgen.py "Okanagan Lake" --sentinel

     # Send straight to the board
     python worldgen.py "Squamish" --uart /dev/ttyUSB0          # send to the board
  python worldgen.py --serve --uart /dev/ttyUSB0              # UI picks -> board
  python worldgen.py --uart /dev/ttyUSB0 --board-ping

     # Bake the demo locations into the cache BEFORE you trust venue wifi
     python worldgen.py --bake-demos

     # Run as a tiny web service for the website teammate
     python worldgen.py --serve --port 8080
     #   then: GET http://localhost:8080/generate?place=Banff&scale=60&flood=0

--------------------------------------------------------------------------------
 Written for StormHacks 2026. Data credits live in world.json and README.md.
================================================================================
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import shutil
import struct
import sys
import time
import urllib.parse
from dataclasses import dataclass, field, asdict
from typing import Optional, Tuple, List, Dict, Any

import numpy as np
import requests


# ==============================================================================
# SECTION 1 -- CONFIGURATION
# Everything you might want to tweak during the hackathon lives up here.
# ==============================================================================

GRID = 128                 # world is GRID x GRID cells. Must stay 128 for the FPGA.
HEIGHT_LEVELS = 32         # heights are 0..31, which is exactly 5 bits.
MAX_HEIGHT = HEIGHT_LEVELS - 1

# --- Block types. Exactly 8 of them, which is exactly 3 bits. ------------------
# These numbers are the contract with the FPGA. Do not renumber them without
# telling your teammates, because they are baked into the Verilog colour table.
BLOCK_WATER  = 0
BLOCK_GRASS  = 1
BLOCK_SAND   = 2
BLOCK_STONE  = 3
BLOCK_SNOW   = 4
BLOCK_FOREST = 5
BLOCK_CITY   = 6
BLOCK_DIRT   = 7   # also used for burn scars in wildfire mode

BLOCK_NAMES = {
    BLOCK_WATER: "water",  BLOCK_GRASS: "grass", BLOCK_SAND: "sand",
    BLOCK_STONE: "stone",  BLOCK_SNOW: "snow",   BLOCK_FOREST: "forest",
    BLOCK_CITY: "city",    BLOCK_DIRT: "dirt",
}

# RGB colours used for the preview image and the terminal preview.
# Hand these to the FPGA person as the starting point for their colour LUT:
# they are already picked to look good on a VGA monitor.
BLOCK_RGB = {
    BLOCK_WATER:  (38, 92, 160),
    BLOCK_GRASS:  (96, 150, 62),
    BLOCK_SAND:   (214, 196, 134),
    BLOCK_STONE:  (124, 124, 128),
    BLOCK_SNOW:   (238, 243, 248),
    BLOCK_FOREST: (42, 94, 54),
    BLOCK_CITY:   (156, 134, 128),
    BLOCK_DIRT:   (124, 92, 62),
}

# Single characters for the plain-ASCII preview.
BLOCK_CHAR = {
    BLOCK_WATER: "~", BLOCK_GRASS: '"', BLOCK_SAND: ".", BLOCK_STONE: "^",
    BLOCK_SNOW: "*",  BLOCK_FOREST: "#", BLOCK_CITY: "M", BLOCK_DIRT: ":",
}

# --- ESA WorldCover class codes -> our block types ----------------------------
# WorldCover is the "what is actually there" layer. These are its official codes.
WORLDCOVER_TO_BLOCK = {
    10:  BLOCK_FOREST,  # tree cover
    20:  BLOCK_GRASS,   # shrubland
    30:  BLOCK_GRASS,   # grassland
    40:  BLOCK_DIRT,    # cropland  -> farmland
    50:  BLOCK_CITY,    # built-up
    60:  BLOCK_STONE,   # bare / sparse vegetation
    70:  BLOCK_SNOW,    # snow and ice
    80:  BLOCK_WATER,   # permanent water bodies
    90:  BLOCK_WATER,   # herbaceous wetland
    95:  BLOCK_FOREST,  # mangroves
    100: BLOCK_GRASS,   # moss and lichen
}

# --- Remote data endpoints ----------------------------------------------------
GEOCODE_URL   = "https://geocoding-api.open-meteo.com/v1/search"
WEATHER_URL   = "https://api.open-meteo.com/v1/forecast"
ELEV_API_URL  = "https://api.open-meteo.com/v1/elevation"

# Copernicus DEM GLO-30, 1 degree tiles, free, no sign-in.
COP_DEM_BASE  = "https://copernicus-dem-30m.s3.amazonaws.com"

# ESA WorldCover 10 m, 3 degree tiles. Two host spellings; we try both because
# the bucket is region-pinned and plain s3.amazonaws.com sometimes 301s.
WORLDCOVER_HOSTS = [
    "https://esa-worldcover.s3.eu-central-1.amazonaws.com",
    "https://esa-worldcover.s3.amazonaws.com",
]
WORLDCOVER_PREFIX = "v200/2021/map"

# AWS Terrain Tiles - backup elevation that needs no rasterio at all.
TERRARIUM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"

# Earth Search STAC catalog for Sentinel-2.
STAC_URL = "https://earth-search.aws.element84.com/v1"
S2_COLLECTIONS = ["sentinel-2-c1-l2a", "sentinel-2-l2a"]

# NASA FIRMS active fire CSV API. Free map key from firms.modaps.eosdis.nasa.gov
FIRMS_URL = "https://firms.modaps.eosdis.nasa.gov/api/area/csv/{key}/{src}/{bbox}/{days}"
FIRMS_SOURCE = "VIIRS_SNPP_NRT"

HTTP_TIMEOUT = 25          # seconds before we give up on any single request
USER_AGENT = "StormHacks2026-WorldGen/1.0 (student hackathon project)"

# --- Demo locations. Bake these before the venue wifi betrays you. ------------
DEMO_LOCATIONS = [
    # (label,            place query,            scale m/block, notes)
    ("vancouver",        "Vancouver, Canada",            60,  "city + ocean + mountains"),
    ("grouse",           "Grouse Mountain, Canada",      30,  "steep alpine relief"),
    ("fraser-delta",     "Richmond, British Columbia",   60,  "flat delta, flood demo"),
    ("okanagan",         "Kelowna, Canada",              90,  "wildfire-prone valley"),
    ("grand-canyon",     "Grand Canyon Village",         90,  "extreme terrain, crowd pleaser"),
    ("venice",           "Venice, Italy",                30,  "water + city, flood demo"),
]


# ==============================================================================
# SECTION 2 -- SMALL HELPERS
# ==============================================================================

VERBOSE = True
_NET_STATE: Optional[bool] = None


def net_available(force_recheck: bool = False) -> bool:
    """
    Is there actually internet right now? Checked once and remembered.

    This matters more than it looks. The ocean has no elevation tiles at all,
    so 'the tile 404ed' legitimately means 'this is the sea'. But a dead wifi
    connection produces the same symptom, and you do not want your mountain
    demo to silently become an empty ocean in front of a judge. So before we
    ever conclude 'this is ocean', we check whether the network itself is up.
    """
    global _NET_STATE
    if _NET_STATE is not None and not force_recheck:
        return _NET_STATE
    for url in ("https://copernicus-dem-30m.s3.amazonaws.com/readme.html",
                "https://api.open-meteo.com/v1/elevation?latitude=0&longitude=0"):
        try:
            r = session().get(url, timeout=8)
            if r.status_code < 500:
                _NET_STATE = True
                return True
        except Exception:
            continue
    _NET_STATE = False
    log("no internet connection detected", "warn")
    return False


def log(msg: str, level: str = "info") -> None:
    """Print a progress line. Quiet mode silences everything but errors."""
    if not VERBOSE and level == "info":
        return
    prefix = {"info": "  ", "ok": "[ok]   ", "warn": "[warn] ", "err": "[FAIL] ",
              "step": "\n>> "}[level]
    print(prefix + msg, file=sys.stderr if level in ("warn", "err") else sys.stdout,
          flush=True)


def short(exc: Exception, limit: int = 130) -> str:
    """
    One-line error text. Requests likes to embed the entire request URL in its
    exceptions, and a 4000-character wall of latitudes buries the actual
    message. This keeps the log readable at 3am.
    """
    text = " ".join(str(exc).split())
    return text if len(text) <= limit else text[:limit] + " ..."


def session() -> requests.Session:
    """One requests Session so connections get reused (noticeably faster)."""
    global _SESSION
    try:
        return _SESSION
    except NameError:
        pass
    s = requests.Session()
    s.headers.update({"User-Agent": USER_AGENT})
    _SESSION = s
    return s


def slugify(text: str) -> str:
    """'Mount Baker, WA' -> 'mount-baker-wa'. Used for cache + output filenames."""
    out = []
    for ch in text.lower():
        if ch.isalnum():
            out.append(ch)
        elif out and out[-1] != "-":
            out.append("-")
    return "".join(out).strip("-") or "world"


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """
    CRC-16/CCITT-FALSE. Polynomial 0x1021, init 0xFFFF, no reflection.
    Chosen because it is about 12 lines of Verilog on the FPGA side.
    """
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def resample_2d(arr: np.ndarray, out_h: int, out_w: int,
                method: str = "bilinear") -> np.ndarray:
    """
    Resize a 2D numpy array without needing scipy or opencv.

    method="bilinear" for continuous data like elevation (smooth).
    method="nearest"  for category data like land cover (must not invent
                      a class that is halfway between 'water' and 'city').
    """
    h, w = arr.shape
    if (h, w) == (out_h, out_w):
        return arr.astype(np.float32) if method == "bilinear" else arr

    # Sample at pixel centres so we do not drift half a pixel.
    ys = (np.arange(out_h) + 0.5) * h / out_h - 0.5
    xs = (np.arange(out_w) + 0.5) * w / out_w - 0.5

    if method == "nearest":
        yi = np.clip(np.round(ys).astype(int), 0, h - 1)
        xi = np.clip(np.round(xs).astype(int), 0, w - 1)
        return arr[yi][:, xi]

    a = arr.astype(np.float32)
    y0 = np.clip(np.floor(ys).astype(int), 0, h - 1)
    x0 = np.clip(np.floor(xs).astype(int), 0, w - 1)
    y1 = np.clip(y0 + 1, 0, h - 1)
    x1 = np.clip(x0 + 1, 0, w - 1)
    wy = np.clip(ys - y0, 0, 1)[:, None]
    wx = np.clip(xs - x0, 0, 1)[None, :]

    top = a[y0][:, x0] * (1 - wx) + a[y0][:, x1] * wx
    bot = a[y1][:, x0] * (1 - wx) + a[y1][:, x1] * wx
    return top * (1 - wy) + bot * wy


def box_blur(arr: np.ndarray, passes: int = 1) -> np.ndarray:
    """
    Tiny 3x3 averaging blur, repeated `passes` times. This is what removes the
    speckly single-pixel noise that makes a voxel world look like static.
    Edges are handled by repeating the border pixel ('edge' padding).
    """
    a = arr.astype(np.float32)
    for _ in range(max(0, passes)):
        p = np.pad(a, 1, mode="edge")
        a = (p[0:-2, 0:-2] + p[0:-2, 1:-1] + p[0:-2, 2:] +
             p[1:-1, 0:-2] + p[1:-1, 1:-1] + p[1:-1, 2:] +
             p[2:,   0:-2] + p[2:,   1:-1] + p[2:,   2:]) / 9.0
    return a


def bbox_from_center(lat: float, lon: float, meters_per_block: float,
                     grid: int = GRID) -> Tuple[float, float, float, float]:
    """
    Turn a centre point + a scale into a (west, south, east, north) box in
    degrees. One degree of latitude is ~111320 m anywhere. One degree of
    longitude shrinks as you move away from the equator, hence the cos().
    """
    span_m = meters_per_block * grid
    dlat = (span_m / 2.0) / 111320.0
    dlon = (span_m / 2.0) / (111320.0 * max(0.15, math.cos(math.radians(lat))))
    return (lon - dlon, lat - dlat, lon + dlon, lat + dlat)


# ==============================================================================
# SECTION 3 -- DATA CLASSES
# ==============================================================================

@dataclass
class Location:
    name: str
    lat: float
    lon: float
    country: str = ""
    admin1: str = ""

    @property
    def label(self) -> str:
        bits = [self.name] + [b for b in (self.admin1, self.country) if b]
        return ", ".join(bits)


@dataclass
class Weather:
    temperature_c: float = 12.0
    rain_mm_h: float = 0.0
    snowfall_cm_h: float = 0.0
    cloud_cover_pct: float = 40.0
    wind_speed_kmh: float = 8.0
    wind_direction_deg: float = 180.0
    is_day: int = 1
    weather_code: int = 0
    local_hour: float = 12.0
    source: str = "default"


@dataclass
class World:
    """Everything the FPGA and the website need, in one object."""
    location: Location
    weather: Weather
    heights: np.ndarray              # (128,128) uint8, 0..31
    blocks: np.ndarray               # (128,128) uint8, 0..7
    spawn: Tuple[int, int, int, int] # x, y, z, yaw(0..255)
    meters_per_block: float
    meters_per_level: float
    elev_min_m: float
    elev_max_m: float
    water_level: int = 0
    flood_level: int = 0
    flood_mask: Optional[np.ndarray] = None   # cells that were land before the flood
    fires: List[Tuple[int, int]] = field(default_factory=list)
    sources: Dict[str, str] = field(default_factory=dict)
    stats: Dict[str, Any] = field(default_factory=dict)
    # Kept for the voxel-game terrain (terrain.bin), which needs real metres
    # rather than the 5-bit levels above.
    elev_m: Optional[np.ndarray] = None       # (128,128) float, smoothed metres
    bbox: Optional[Tuple[float, float, float, float]] = None   # W, S, E, N
    exaggeration: float = 1.0
    terrain_blob: Optional[bytes] = None      # packed terrain.bin, set by write_terrain


# ==============================================================================
# SECTION 4 -- GEOCODING  (place name -> latitude / longitude)
# ==============================================================================

def geocode(place: str) -> Optional[Location]:
    """
    Ask Open-Meteo's free geocoder where a place is. No API key needed.
    Returns None if the network is down or nothing matched.
    """
    log(f"geocoding {place!r}", "step")
    try:
        r = session().get(GEOCODE_URL,
                          params={"name": place, "count": 1, "language": "en",
                                  "format": "json"},
                          timeout=HTTP_TIMEOUT)
        r.raise_for_status()
        results = r.json().get("results") or []
        if not results:
            log(f"no match for {place!r}", "warn")
            return None
        g = results[0]
        loc = Location(name=g.get("name", place), lat=float(g["latitude"]),
                       lon=float(g["longitude"]), country=g.get("country", ""),
                       admin1=g.get("admin1", ""))
        log(f"{loc.label}  ->  {loc.lat:.4f}, {loc.lon:.4f}", "ok")
        return loc
    except Exception as exc:
        log(f"geocoding failed: {short(exc)}", "warn")
        return None


# ==============================================================================
# SECTION 5 -- WEATHER  (live conditions right now, at that spot)
# ==============================================================================

def fetch_weather(lat: float, lon: float) -> Weather:
    """
    Open-Meteo current conditions. Free, no key, no rate limit worth worrying
    about. If it fails we return pleasant defaults so the demo still runs.
    """
    log("fetching live weather", "step")
    fields = ("temperature_2m,precipitation,rain,snowfall,cloud_cover,"
              "wind_speed_10m,wind_direction_10m,is_day,weather_code")
    try:
        r = session().get(WEATHER_URL,
                          params={"latitude": lat, "longitude": lon,
                                  "current": fields, "timezone": "auto"},
                          timeout=HTTP_TIMEOUT)
        r.raise_for_status()
        data = r.json()
        c = data.get("current", {})

        # Work out the local hour so the FPGA can do a day/night sky.
        local_hour = 12.0
        iso = c.get("time")
        if iso and "T" in iso:
            hh, mm = iso.split("T")[1].split(":")[:2]
            local_hour = int(hh) + int(mm) / 60.0

        w = Weather(
            temperature_c=float(c.get("temperature_2m", 12.0)),
            rain_mm_h=float(c.get("rain", c.get("precipitation", 0.0)) or 0.0),
            snowfall_cm_h=float(c.get("snowfall", 0.0) or 0.0),
            cloud_cover_pct=float(c.get("cloud_cover", 40.0) or 0.0),
            wind_speed_kmh=float(c.get("wind_speed_10m", 8.0) or 0.0),
            wind_direction_deg=float(c.get("wind_direction_10m", 180.0) or 0.0),
            is_day=int(c.get("is_day", 1) or 0),
            weather_code=int(c.get("weather_code", 0) or 0),
            local_hour=local_hour,
            source="open-meteo",
        )
        log(f"{w.temperature_c:.1f} C, rain {w.rain_mm_h:.1f} mm/h, "
            f"snow {w.snowfall_cm_h:.1f} cm/h, cloud {w.cloud_cover_pct:.0f}%, "
            f"{'day' if w.is_day else 'night'}", "ok")
        return w
    except Exception as exc:
        log(f"weather failed ({short(exc)}); using defaults", "warn")
        return Weather()


# ==============================================================================
# SECTION 6 -- ELEVATION
# Three independent sources, tried in order. You will almost always get #1.
# ==============================================================================

def _rasterio_env():
    """GDAL settings that make reading remote COGs fast and anonymous."""
    import rasterio
    return rasterio.Env(
        AWS_NO_SIGN_REQUEST="YES",
        GDAL_DISABLE_READDIR_ON_OPEN="EMPTY_DIR",   # do not list the bucket
        CPL_VSIL_CURL_ALLOWED_EXTENSIONS=".tif,.TIF",
        GDAL_HTTP_MULTIPLEX="YES",
        VSI_CACHE="TRUE",
        VSI_CACHE_SIZE="64000000",
    )


def _cop_dem_tile_url(lat_floor: int, lon_floor: int) -> str:
    """
    Build the Copernicus GLO-30 filename for a 1x1 degree tile.
    Example: N49, W123  ->  Copernicus_DSM_COG_10_N49_00_W123_00_DEM
    The '10' means GLO-30 (10 arc-seconds naming); GLO-90 would be '30'.
    """
    ns = "N" if lat_floor >= 0 else "S"
    ew = "E" if lon_floor >= 0 else "W"
    name = (f"Copernicus_DSM_COG_10_{ns}{abs(lat_floor):02d}_00_"
            f"{ew}{abs(lon_floor):03d}_00_DEM")
    return f"{COP_DEM_BASE}/{name}/{name}.tif"


def fetch_elevation_copernicus(bbox) -> Optional[np.ndarray]:
    """
    Primary elevation source: Copernicus DEM GLO-30, 30 m per pixel, worldwide.
    Returns a (128,128) float array of metres above sea level, or None.

    Note: the ocean has no tiles at all. A missing tile means 'sea level', which
    is exactly right, so a missing tile is not an error.
    """
    try:
        import rasterio
        from rasterio.merge import merge
        from rasterio.enums import Resampling
    except ImportError:
        log("rasterio not installed, skipping Copernicus DEM", "warn")
        return None

    w, s, e, n = bbox
    urls = []
    for la in range(math.floor(s), math.floor(n) + 1):
        for lo in range(math.floor(w), math.floor(e) + 1):
            urls.append(_cop_dem_tile_url(la, lo))

    log(f"Copernicus DEM GLO-30: {len(urls)} tile(s)")
    datasets = []
    try:
        with _rasterio_env():
            for url in urls:
                try:
                    datasets.append(rasterio.open("/vsicurl/" + url))
                except Exception:
                    log(f"  tile missing (ocean or unreleased): {url.split('/')[-1]}")
            if not datasets:
                # No tiles. Either this really is open ocean, or the network
                # is down. Those look identical from here, so go and check.
                if net_available():
                    log("no DEM tiles and network is fine -> open ocean", "ok")
                    return np.zeros((GRID, GRID), dtype=np.float32)
                log("no DEM tiles and no network -> trying other sources", "warn")
                return None

            res_x = (e - w) / GRID
            res_y = (n - s) / GRID
            mosaic, _ = merge(datasets, bounds=(w, s, e, n), res=(res_x, res_y),
                              resampling=Resampling.bilinear, nodata=0)
            arr = np.asarray(mosaic[0], dtype=np.float32)
    finally:
        for d in datasets:
            try:
                d.close()
            except Exception:
                pass

    # Copernicus uses a large negative value for 'no data'; clamp it to sea level.
    arr = np.where(arr < -500, 0.0, arr)
    arr = resample_2d(arr, GRID, GRID, "bilinear")
    log(f"elevation {arr.min():.0f} m to {arr.max():.0f} m", "ok")
    return arr


def fetch_elevation_terrarium(bbox, lat: float) -> Optional[np.ndarray]:
    """
    Backup elevation: AWS Terrain Tiles in 'terrarium' PNG encoding.
    Needs only requests + pillow, so this works even when rasterio will not
    install on a locked-down laptop at 2am.

    Decoding rule:  elevation_m = (R * 256 + G + B / 256) - 32768
    """
    try:
        from PIL import Image
    except ImportError:
        return None

    w, s, e, n = bbox
    # Pick a zoom level whose pixels are about as fine as our blocks.
    span_m = (e - w) * 111320.0 * math.cos(math.radians(lat))
    target_px = span_m / GRID
    z = int(round(math.log2(156543.03 * math.cos(math.radians(lat)) / max(1.0, target_px))))
    z = max(1, min(14, z))

    def lon2x(lo, zz):
        return (lo + 180.0) / 360.0 * (2 ** zz)

    def lat2y(la, zz):
        r = math.radians(max(-85.05, min(85.05, la)))
        return (1.0 - math.log(math.tan(r) + 1 / math.cos(r)) / math.pi) / 2.0 * (2 ** zz)

    x0f, x1f = lon2x(w, z), lon2x(e, z)
    y0f, y1f = lat2y(n, z), lat2y(s, z)       # note: y grows southward
    x0, x1 = int(math.floor(x0f)), int(math.floor(x1f))
    y0, y1 = int(math.floor(y0f)), int(math.floor(y1f))

    tiles_w, tiles_h = x1 - x0 + 1, y1 - y0 + 1
    if tiles_w * tiles_h > 36:
        log("terrain tile span too large at this zoom, skipping", "warn")
        return None

    log(f"AWS Terrain Tiles z={z}, {tiles_w}x{tiles_h} tiles")
    canvas = np.zeros((tiles_h * 256, tiles_w * 256), dtype=np.float32)
    got_any = False
    for ty in range(y0, y1 + 1):
        for tx in range(x0, x1 + 1):
            url = TERRARIUM_URL.format(z=z, x=tx, y=ty)
            try:
                r = session().get(url, timeout=HTTP_TIMEOUT)
                if r.status_code != 200:
                    continue
                im = Image.open(io.BytesIO(r.content)).convert("RGB")
                px = np.asarray(im, dtype=np.float32)
                elev = px[:, :, 0] * 256.0 + px[:, :, 1] + px[:, :, 2] / 256.0 - 32768.0
                canvas[(ty - y0) * 256:(ty - y0 + 1) * 256,
                       (tx - x0) * 256:(tx - x0 + 1) * 256] = elev
                got_any = True
            except Exception:
                continue
    if not got_any:
        return None

    # Crop the mosaic down to exactly our bounding box, then resample to 128.
    cx0 = int((x0f - x0) * 256)
    cx1 = int((x1f - x0) * 256)
    cy0 = int((y0f - y0) * 256)
    cy1 = int((y1f - y0) * 256)
    cx1 = max(cx1, cx0 + 2)
    cy1 = max(cy1, cy0 + 2)
    crop = canvas[cy0:cy1, cx0:cx1]
    if crop.size == 0:
        return None
    arr = resample_2d(crop, GRID, GRID, "bilinear")
    log(f"elevation {arr.min():.0f} m to {arr.max():.0f} m (terrain tiles)", "ok")
    return arr


def fetch_elevation_openmeteo(bbox) -> Optional[np.ndarray]:
    """
    Emergency elevation: Open-Meteo's elevation endpoint, 100 points per call.
    We sample a coarse 16x16 lattice and smoothly upscale it. The result is
    blurry but it is real terrain and it keeps the demo alive.
    """
    w, s, e, n = bbox
    coarse = 16
    lats = np.linspace(s, n, coarse)
    lons = np.linspace(w, e, coarse)
    grid_lat, grid_lon = np.meshgrid(lats, lons, indexing="ij")
    flat_lat, flat_lon = grid_lat.ravel(), grid_lon.ravel()

    values: List[float] = []
    try:
        for i in range(0, flat_lat.size, 100):
            chunk_lat = ",".join(f"{v:.5f}" for v in flat_lat[i:i + 100])
            chunk_lon = ",".join(f"{v:.5f}" for v in flat_lon[i:i + 100])
            r = session().get(ELEV_API_URL,
                              params={"latitude": chunk_lat, "longitude": chunk_lon},
                              timeout=HTTP_TIMEOUT)
            r.raise_for_status()
            values.extend(r.json()["elevation"])
    except Exception as exc:
        log(f"elevation API failed: {short(exc)}", "warn")
        return None

    arr = np.asarray(values, dtype=np.float32).reshape(coarse, coarse)
    arr = np.flipud(arr)                 # our grids are north-up
    arr = resample_2d(arr, GRID, GRID, "bilinear")
    log(f"elevation {arr.min():.0f} m to {arr.max():.0f} m (coarse API)", "ok")
    return arr


def synthetic_elevation(seed: int) -> np.ndarray:
    """
    Last resort: fake but convincing terrain from layered value noise
    (the classic 'sum octaves of random noise at doubling frequency' trick).

    Used only when every network source failed AND there is no cache. The world
    keeps a coastline and real relief so the demo still looks like somewhere,
    and world.json records source='synthetic' so you never accidentally claim
    fake terrain is real in front of a judge.
    """
    rng = np.random.default_rng(seed)
    field_ = np.zeros((GRID, GRID), dtype=np.float32)
    amplitude = 1.0
    size = 4
    while size <= GRID:
        layer = rng.random((size, size)).astype(np.float32)
        field_ += resample_2d(layer, GRID, GRID, "bilinear") * amplitude
        amplitude *= 0.5
        size *= 2
    field_ = box_blur(field_, 2)
    field_ -= field_.min()
    field_ /= max(1e-6, field_.max())

    # Tilt the whole thing so one corner is below sea level: that gives us a
    # coastline, which is far more interesting to walk around than a blob.
    yy, xx = np.mgrid[0:GRID, 0:GRID].astype(np.float32)
    tilt = (xx / GRID) * 0.45 + (yy / GRID) * 0.25
    field_ = field_ * 0.8 + tilt * 0.5
    field_ -= np.percentile(field_, 22)      # ~22% of the map ends up as sea
    field_ *= 1400.0
    # Real DEMs report the ocean as a flat 0 m rather than as bathymetry, so
    # clamp here too. Otherwise the deep water would stretch the height scale
    # and squash all the land into the top few levels.
    return np.maximum(field_, 0.0)


def synthetic_landcover(elev_m: np.ndarray, seed: int) -> np.ndarray:
    """
    Matching fake land cover for the synthetic world, in real ESA WorldCover
    class codes so it flows through exactly the same mapping path as real data.
    Beach at the waterline, forest on the lower slopes, rock up high, snow on
    the peaks, and a couple of built-up patches so cities are represented.
    """
    rng = np.random.default_rng(seed + 7)
    lo, hi = float(np.percentile(elev_m, 1)), float(np.percentile(elev_m, 99))
    rel = np.clip((elev_m - lo) / max(1e-6, hi - lo), 0, 1)

    cover = np.full((GRID, GRID), 30, dtype=np.uint8)        # grassland
    cover[rel > 0.30] = 10                                   # tree cover
    cover[rel > 0.68] = 60                                   # bare / rock
    cover[rel > 0.88] = 70                                   # snow and ice
    cover[elev_m <= 0.0] = 80                                # water

    # Two or three towns on gentle ground near the water.
    buildable = (elev_m > 0) & (rel < 0.28)
    ys, xs = np.nonzero(buildable)
    if ys.size:
        for _ in range(3):
            i = int(rng.integers(0, ys.size))
            cy, cx = int(ys[i]), int(xs[i])
            r = int(rng.integers(5, 12))
            yy, xx = np.mgrid[0:GRID, 0:GRID]
            town = ((yy - cy) ** 2 + (xx - cx) ** 2) < r * r
            cover[town & buildable] = 50                     # built-up
    # Fields around the towns. Smoothed noise gives coherent patches instead
    # of per-pixel confetti, which looks like static once it is 3D.
    patch = resample_2d(rng.random((16, 16)).astype(np.float32), GRID, GRID, "bilinear")
    patch = box_blur(patch, 1)
    cover[(cover == 30) & (rel < 0.26) & (patch > 0.55)] = 40
    return cover


def fetch_elevation(bbox, lat: float, prefer: str = "auto") -> Tuple[np.ndarray, str]:
    """Try each elevation source in turn. Returns (array_in_metres, source_name)."""
    log("fetching elevation", "step")
    order = (["copernicus", "terrarium", "openmeteo"] if prefer == "auto"
             else [prefer, "copernicus", "terrarium", "openmeteo"])
    seen = set()
    for src in order:
        if src in seen:
            continue
        seen.add(src)
        try:
            if src == "copernicus":
                arr = fetch_elevation_copernicus(bbox)
            elif src == "terrarium":
                arr = fetch_elevation_terrarium(bbox, lat)
            else:
                arr = fetch_elevation_openmeteo(bbox)
        except Exception as exc:
            log(f"{src} raised {short(exc)}", "warn")
            arr = None
        if arr is not None:
            return arr, src
    log("every elevation source failed -> synthetic terrain", "warn")
    return synthetic_elevation(int(abs(lat) * 1000)), "synthetic"


# ==============================================================================
# SECTION 7 -- LAND COVER  (ESA WorldCover 10 m)
# ==============================================================================

def _worldcover_tile_name(lat_floor3: int, lon_floor3: int) -> str:
    ns = "N" if lat_floor3 >= 0 else "S"
    ew = "E" if lon_floor3 >= 0 else "W"
    return (f"ESA_WorldCover_10m_2021_v200_"
            f"{ns}{abs(lat_floor3):02d}{ew}{abs(lon_floor3):03d}_Map.tif")


def fetch_landcover(bbox) -> Optional[np.ndarray]:
    """
    ESA WorldCover: 10 m global land cover, 11 classes, free, no key.
    Tiles are 3x3 degrees, named by their lower-left corner.

    Returns (128,128) uint8 of WorldCover class codes, or None.
    Resampling is NEAREST on purpose: averaging 'water' and 'built-up' would
    produce a class that does not exist.
    """
    try:
        import rasterio
        from rasterio.merge import merge
        from rasterio.enums import Resampling
    except ImportError:
        log("rasterio not installed, skipping land cover", "warn")
        return None

    log("fetching ESA WorldCover land cover", "step")
    w, s, e, n = bbox

    def floor3(v):
        return int(math.floor(v / 3.0) * 3)

    names = []
    for la in range(floor3(s), floor3(n) + 1, 3):
        for lo in range(floor3(w), floor3(e) + 1, 3):
            names.append(_worldcover_tile_name(la, lo))

    datasets = []
    try:
        with _rasterio_env():
            for name in names:
                opened = False
                for host in WORLDCOVER_HOSTS:
                    url = f"/vsicurl/{host}/{WORLDCOVER_PREFIX}/{name}"
                    try:
                        datasets.append(rasterio.open(url))
                        opened = True
                        break
                    except Exception:
                        continue
                if not opened:
                    log(f"  land cover tile unavailable: {name}")
            if not datasets:
                log("no land cover tiles available", "warn")
                return None

            res_x = (e - w) / GRID
            res_y = (n - s) / GRID
            mosaic, _ = merge(datasets, bounds=(w, s, e, n), res=(res_x, res_y),
                              resampling=Resampling.nearest, nodata=0)
            arr = np.asarray(mosaic[0], dtype=np.uint8)
    finally:
        for d in datasets:
            try:
                d.close()
            except Exception:
                pass

    arr = resample_2d(arr, GRID, GRID, "nearest").astype(np.uint8)
    present = sorted(set(int(v) for v in np.unique(arr)))
    log(f"land cover classes present: {present}", "ok")
    return arr


# ==============================================================================
# SECTION 8 -- SENTINEL-2  (the layer that makes the satellite track real)
# ==============================================================================

def fetch_sentinel2(bbox, max_cloud: int = 20,
                    days_back: int = 150) -> Optional[Dict[str, np.ndarray]]:
    """
    Find the most recent low-cloud Sentinel-2 scene over this box and compute
    three spectral indices from it:

        NDVI = (NIR - Red)   / (NIR + Red)     how green / alive is it
        NDWI = (Green - NIR) / (Green + NIR)   is this open water
        NDSI = (Green - SWIR)/ (Green + SWIR)  is this snow or ice

    These are not decoration. They override the 2021 WorldCover map with what
    the satellite saw a few days ago, which is how a lake that dried up or a
    snowline that moved actually shows up in the game.

    Returns {'ndvi':..., 'ndwi':..., 'ndsi':..., 'date':..., 'id':...} or None.
    """
    try:
        from pystac_client import Client
        import rasterio
        from rasterio.vrt import WarpedVRT
        from rasterio.windows import from_bounds
        from rasterio.enums import Resampling
    except ImportError:
        log("pystac-client or rasterio missing, skipping Sentinel-2", "warn")
        return None

    log("searching Sentinel-2 archive", "step")
    import datetime as _dt
    end = _dt.datetime.now(_dt.timezone.utc)
    start = end - _dt.timedelta(days=days_back)
    window = f"{start.date().isoformat()}/{end.date().isoformat()}"

    item = None
    try:
        client = Client.open(STAC_URL)
        # Somewhere like coastal BC in October can genuinely have no clear
        # scene for months, so relax the cloud limit rather than give up. A
        # partly cloudy scene still gives usable indices over most of the box.
        for cloud_limit in (max_cloud, 45, 80):
            for coll in S2_COLLECTIONS:
                try:
                    search = client.search(
                        collections=[coll], bbox=list(bbox), datetime=window,
                        query={"eo:cloud_cover": {"lt": cloud_limit}}, max_items=20)
                    items = list(search.items())
                except Exception:
                    items = []
                if items:
                    # Prefer the clearest scene, and among equals the newest.
                    items.sort(key=lambda it: (it.properties.get("eo:cloud_cover", 100),
                                               -it.datetime.timestamp()))
                    item = items[0]
                    break
            if item is not None:
                break
            if cloud_limit != 80:
                log(f"nothing under {cloud_limit}% cloud, relaxing the limit")
    except Exception as exc:
        log(f"STAC search failed: {short(exc)}", "warn")
        return None

    if item is None:
        log(f"no Sentinel-2 scene at all in the last {days_back} days", "warn")
        return None

    cloud = item.properties.get("eo:cloud_cover", -1)
    log(f"scene {item.id}  {item.datetime.date()}  cloud {cloud:.1f}%", "ok")

    def read_band(key: str) -> Optional[np.ndarray]:
        """Read one band, reprojected to lat/lon, cropped to our box, at 128x128."""
        asset = item.assets.get(key)
        if asset is None:
            return None
        try:
            with _rasterio_env():
                with rasterio.open(asset.href) as src:
                    # Sentinel-2 lives in UTM; our bbox is in degrees. WarpedVRT
                    # does the reprojection on the fly without downloading all of it.
                    with WarpedVRT(src, crs="EPSG:4326",
                                   resampling=Resampling.bilinear) as vrt:
                        win = from_bounds(*bbox, transform=vrt.transform)
                        band = vrt.read(1, window=win, out_shape=(GRID, GRID),
                                        boundless=True, fill_value=0,
                                        resampling=Resampling.bilinear)
            return band.astype(np.float32)
        except Exception as exc:
            log(f"  band {key} failed: {short(exc)}", "warn")
            return None

    red = read_band("red")
    green = read_band("green")
    nir = read_band("nir")
    swir = read_band("swir16")
    if red is None or green is None or nir is None:
        log("essential Sentinel-2 bands missing", "warn")
        return None

    def ratio(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        denom = a + b
        out = np.zeros_like(a, dtype=np.float32)
        ok = denom > 1e-6
        out[ok] = (a[ok] - b[ok]) / denom[ok]
        return np.clip(out, -1.0, 1.0)

    result = {
        "ndvi": ratio(nir, red),
        "ndwi": ratio(green, nir),
        "ndsi": ratio(green, swir) if swir is not None else np.zeros((GRID, GRID),
                                                                     dtype=np.float32),
        "scene_id": item.id,
        "scene_date": str(item.datetime.date()),
        "cloud_cover": float(cloud),
    }
    log(f"NDVI {result['ndvi'].mean():+.2f} avg, "
        f"NDWI {result['ndwi'].mean():+.2f} avg, "
        f"NDSI {result['ndsi'].mean():+.2f} avg", "ok")
    return result


# ==============================================================================
# SECTION 9 -- WILDFIRE HOTSPOTS  (NASA FIRMS, optional free key)
# ==============================================================================

def fetch_fires(bbox, map_key: str, days: int = 2) -> List[Tuple[float, float]]:
    """
    NASA FIRMS active fire detections from VIIRS, last `days` days.
    Get a free map key at firms.modaps.eosdis.nasa.gov/api/map_key/
    Returns a list of (lat, lon).
    """
    if not map_key:
        return []
    log("fetching NASA FIRMS active fires", "step")
    w, s, e, n = bbox
    url = FIRMS_URL.format(key=map_key, src=FIRMS_SOURCE,
                           bbox=f"{w:.4f},{s:.4f},{e:.4f},{n:.4f}",
                           days=max(1, min(10, days)))
    try:
        r = session().get(url, timeout=HTTP_TIMEOUT)
        r.raise_for_status()
        lines = r.text.strip().splitlines()
        if len(lines) < 2:
            log("no active fires in this box", "ok")
            return []
        header = [h.strip().lower() for h in lines[0].split(",")]
        li, loi = header.index("latitude"), header.index("longitude")
        pts = []
        for row in lines[1:]:
            cells = row.split(",")
            try:
                pts.append((float(cells[li]), float(cells[loi])))
            except (ValueError, IndexError):
                continue
        log(f"{len(pts)} active fire detections", "ok")
        return pts
    except Exception as exc:
        log(f"FIRMS failed: {short(exc)}", "warn")
        return []


# ==============================================================================
# SECTION 10 -- BUILDING THE WORLD
# This is the heart of the program: raw metres and class codes become
# 5-bit heights and 3-bit block types.
# ==============================================================================

def classify_blocks(elev_m: np.ndarray,
                    landcover: Optional[np.ndarray],
                    indices: Optional[Dict[str, np.ndarray]],
                    weather: Weather) -> Tuple[np.ndarray, Dict[str, Any]]:
    """
    Decide what each of the 16384 cells is made of.

    Priority, lowest to highest:
      1. a plausible guess from elevation alone  (always available)
      2. ESA WorldCover 2021                     (what was there last year)
      3. Sentinel-2 indices from the last weeks  (what is there NOW)
      4. live weather                            (fresh snow turns it white)
    """
    notes: Dict[str, Any] = {}

    # ---- Layer 1: elevation-only guess -------------------------------------
    # Relative height within this scene, 0 = lowest land, 1 = highest.
    # Negative values are below sea level, and a deep trench offshore would
    # otherwise stretch this scale and push the whole landmass into the top
    # few percent, so clamp the sea to 0 first the way real DEMs do.
    land_elev = np.maximum(elev_m, 0.0)
    lo = float(np.percentile(land_elev, 1))
    hi = float(np.percentile(land_elev, 99))
    rel = np.clip((land_elev - lo) / max(1e-6, hi - lo), 0, 1)

    blocks = np.full((GRID, GRID), BLOCK_GRASS, dtype=np.uint8)
    blocks[rel > 0.72] = BLOCK_STONE
    blocks[rel < 0.06] = BLOCK_SAND
    blocks[elev_m <= 0.5] = BLOCK_WATER        # at or below sea level
    notes["base"] = "elevation heuristic"

    # ---- Layer 2: ESA WorldCover -------------------------------------------
    if landcover is not None:
        mapped = np.zeros((GRID, GRID), dtype=np.uint8)
        known = np.zeros((GRID, GRID), dtype=bool)
        for code, block in WORLDCOVER_TO_BLOCK.items():
            m = landcover == code
            mapped[m] = block
            known |= m
        blocks = np.where(known, mapped, blocks).astype(np.uint8)
        notes["landcover"] = f"ESA WorldCover 2021 applied to {known.mean()*100:.0f}% of cells"

    # ---- Layer 3: Sentinel-2 live refinement -------------------------------
    if indices is not None:
        ndvi, ndwi, ndsi = indices["ndvi"], indices["ndwi"], indices["ndsi"]

        # Open water: strongly positive NDWI and not green.
        water_now = (ndwi > 0.15) & (ndvi < 0.25)
        blocks[water_now] = BLOCK_WATER

        # Dense vegetation that WorldCover may have missed or that grew back.
        dense = (ndvi > 0.55) & ~water_now
        blocks[dense] = BLOCK_FOREST

        # Thin vegetation on what we thought was bare rock -> grass.
        thin = (ndvi > 0.28) & (ndvi <= 0.55) & (blocks == BLOCK_STONE)
        blocks[thin] = BLOCK_GRASS

        # Bare ground: low NDVI, low NDWI, and up high -> rock, down low -> sand.
        bare = (ndvi < 0.12) & (ndwi < 0.0)
        blocks[bare & (rel > 0.45)] = BLOCK_STONE
        blocks[bare & (rel <= 0.12)] = BLOCK_SAND

        # Snow and ice: high NDSI with low vegetation. This is the real snowline.
        snow_now = (ndsi > 0.42) & (ndvi < 0.2)
        blocks[snow_now] = BLOCK_SNOW

        notes["sentinel2"] = (f"scene {indices['scene_id']} ({indices['scene_date']}); "
                              f"water {water_now.mean()*100:.1f}%, "
                              f"forest {dense.mean()*100:.1f}%, "
                              f"snow {snow_now.mean()*100:.1f}% of cells")

    # ---- Layer 4: live weather ---------------------------------------------
    # Below freezing, or snow actively falling: whiten the high ground.
    if weather.temperature_c < 0 or weather.snowfall_cm_h > 0.05:
        # The colder it is, the further down the mountain the snow reaches.
        snow_from = 0.80 if weather.temperature_c > -3 else (
                    0.55 if weather.temperature_c > -8 else 0.25)
        if weather.snowfall_cm_h > 0.3:
            snow_from = max(0.05, snow_from - 0.25)
        fresh = (rel >= snow_from) & (blocks != BLOCK_WATER) & (blocks != BLOCK_CITY)
        blocks[fresh] = BLOCK_SNOW
        notes["weather_snow"] = (f"{weather.temperature_c:.1f} C -> snow above "
                                 f"{snow_from*100:.0f}% relative height "
                                 f"({fresh.mean()*100:.1f}% of cells)")

    return blocks, notes


# ==============================================================================
# SECTION 10b -- POST-PROCESSING  (make obvious places look like themselves)
#
# The raw layers are right on average but wrong in ways anyone notices:
# ESA WorldCover calls both the Sahara and the beach at Copacabana "bare /
# sparse vegetation", which became grey stone; elevation tiles have holes
# (zeros) at their seams that punch trenches into the world; and nothing
# knew where mountains get snow or trees stop growing. These rules fix that
# before anything is drawn or sent, so the preview and the game agree.
# ==============================================================================

def fill_dem_voids(elev: np.ndarray, landcover: Optional[np.ndarray]
                   ) -> Tuple[np.ndarray, int]:
    """
    Cells at ~0 m that are not water but sit among higher ground are missing
    data (tile seams, radar shadows), not sea. Fill them from their
    neighbours, outside in. Returns (elevation, cells filled).
    """
    low = elev <= 0.5
    if landcover is not None:
        low &= ~np.isin(landcover, (80, 90))           # real water stays
    if not low.any():
        return elev, 0
    p = np.pad(np.where(low, np.nan, elev), 2, mode="edge")
    stack = [p[dy:dy + GRID, dx:dx + GRID] for dy in range(5) for dx in range(5)]
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            around = np.nanmedian(np.stack(stack), axis=0)
    void = low & (np.nan_to_num(around, nan=0.0) > 20.0)
    if not void.any():
        return elev, 0

    out = elev.astype(np.float64).copy()
    known = ~void
    for _ in range(GRID):
        if known.all():
            break
        vals = np.pad(np.where(known, out, 0.0), 1)
        cnt = np.pad(known.astype(np.float64), 1)
        s = sum(vals[dy:dy + GRID, dx:dx + GRID] for dy in range(3) for dx in range(3))
        c = sum(cnt[dy:dy + GRID, dx:dx + GRID] for dy in range(3) for dx in range(3))
        grow = ~known & (c > 0)
        out[grow] = s[grow] / c[grow]
        known |= grow
    return out.astype(elev.dtype), int(void.sum())


def _dilate(mask: np.ndarray, steps: int) -> np.ndarray:
    m = mask.copy()
    for _ in range(steps):
        p = np.pad(m, 1)
        m = (p[1:-1, 1:-1] | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:] |
             p[:-2, :-2] | p[:-2, 2:] | p[2:, :-2] | p[2:, 2:])
    return m


def snowline_m(lat: float) -> float:
    """Rough permanent snowline: ~5200 m at the equator, ~2500 m at 49 degrees."""
    return max(300.0, 5200.0 - 55.0 * abs(lat))


def refine_classes(blocks: np.ndarray, elev: np.ndarray, lat: float,
                   meters_per_block: float) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Desert sand, beaches, snowline and treeline. Returns (blocks, notes)."""
    b = blocks.copy()
    notes: Dict[str, Any] = {}
    water = b == BLOCK_WATER
    land = ~water

    gy, gx = np.gradient(elev.astype(np.float64), meters_per_block)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))

    # --- deserts: bare ground with almost no plant life around ---------------
    veg = np.isin(b, (BLOCK_GRASS, BLOCK_FOREST, BLOCK_DIRT)) & land
    veg_frac = float(veg.sum()) / max(1, int(land.sum()))
    if land.any() and veg_frac < 0.2 and abs(lat) < 50:
        dunes = land & (b == BLOCK_STONE) & (slope < 15) & (elev < 3000)
        b[dunes] = BLOCK_SAND
        notes["desert"] = (f"{veg_frac * 100:.0f}% vegetation: bare ground on gentle "
                           f"slopes -> sand ({dunes.mean() * 100:.0f}% of cells)")

    # --- beaches: low, gentle land right next to the water -------------------
    if water.any() and land.any():
        reach = max(1, int(round(60.0 / meters_per_block)))
        near = _dilate(water, reach) & land
        # height of the water nearby: spread the water's elevation outwards
        wlev = np.where(water, elev, -np.inf)
        for _ in range(reach):
            p = np.pad(wlev, 1, constant_values=-np.inf)
            wlev = np.maximum.reduce([p[1:-1, 1:-1], p[:-2, 1:-1], p[2:, 1:-1],
                                      p[1:-1, :-2], p[1:-1, 2:]])
        beach = (near & (elev - wlev <= 6.0) & (slope < 12) &
                 np.isin(b, (BLOCK_STONE, BLOCK_GRASS, BLOCK_DIRT, BLOCK_SAND)))
        b[beach] = BLOCK_SAND
        if beach.any():
            notes["beach"] = f"{int(beach.sum())} shoreline cells -> sand"

    # --- snowline and treeline ------------------------------------------------
    line = snowline_m(lat)
    snow = land & (((elev > line) & (slope < 45)) | (elev > line + 400))
    alpine = land & (b == BLOCK_FOREST) & (elev > line - 600) & ~snow
    b[alpine & (slope >= 30)] = BLOCK_STONE
    b[alpine & (slope < 30)] = BLOCK_GRASS
    b[snow & (b != BLOCK_CITY)] = BLOCK_SNOW
    if snow.any() or alpine.any():
        notes["mountains"] = (f"snowline {line:.0f} m: {int(snow.sum())} cells snow, "
                              f"{int(alpine.sum())} above the treeline")
    return b, notes


def quantize_heights(elev_m: np.ndarray, exaggeration: float, smooth: int
                     ) -> Tuple[np.ndarray, float, float, float]:
    """
    Turn metres above sea level into 0..31 block heights.

    Why percentiles and not min/max: one weird spike (a radio tower in the DSM,
    a data glitch) would otherwise squash the entire rest of the terrain into
    two or three levels. The 2nd and 98th percentile ignore those outliers.

    `exaggeration` stretches the relief before clipping. First person views make
    real terrain look disappointingly flat, so 1.3 to 2.0 usually looks better
    than 1.0. Everything that clips ends up at level 31, which reads as a plateau.

    Returns (heights 0..31, metres_per_level, elev_lo, elev_hi).
    """
    smoothed = box_blur(elev_m, smooth)
    lo = float(np.percentile(smoothed, 2))
    hi = float(np.percentile(smoothed, 98))

    if hi - lo < 1.0:
        # Genuinely flat place (open ocean, salt flat, prairie). Give it a
        # little relief so it is not a featureless plane, but stay honest.
        lo, hi = float(smoothed.min()), float(smoothed.min()) + 1.0

    norm = (smoothed - lo) / (hi - lo)
    norm = np.clip(norm * float(exaggeration), 0.0, 1.0)
    heights = np.round(norm * MAX_HEIGHT).astype(np.uint8)

    metres_per_level = (hi - lo) / (MAX_HEIGHT * max(1e-6, exaggeration))
    return heights, metres_per_level, lo, hi


def flatten_water(heights: np.ndarray, blocks: np.ndarray) -> Tuple[np.ndarray, int]:
    """
    Real lakes are flat. After quantizing, water cells have slightly different
    heights from DEM noise, which looks terrible in first person (a rippled,
    staircase lake). Snap every water cell to one level.

    Returns (heights, water_level).
    """
    water = blocks == BLOCK_WATER
    if not water.any():
        return heights, 0
    level = int(np.median(heights[water]))
    heights = heights.copy()
    heights[water] = level
    return heights, level


def apply_flood(heights: np.ndarray, blocks: np.ndarray, flood_level: int
                ) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any], Optional[np.ndarray]]:
    """
    Disaster mode. Everything at or below `flood_level` becomes water at exactly
    that height. Returns the new grids plus statistics you can say out loud in
    the pitch ("at plus 6 metres, 34 percent of the built-up area is underwater").
    """
    if flood_level <= 0:
        return heights, blocks, {}, None

    was_land = blocks != BLOCK_WATER
    was_city = blocks == BLOCK_CITY
    was_forest = blocks == BLOCK_FOREST
    drowned = was_land & (heights <= flood_level)

    heights = heights.copy()
    blocks = blocks.copy()
    submerged = heights <= flood_level
    heights[submerged] = flood_level
    blocks[submerged] = BLOCK_WATER

    total_land = max(1, int(was_land.sum()))
    stats = {
        "flood_level": int(flood_level),
        "land_cells_flooded": int(drowned.sum()),
        "land_pct_flooded": round(100.0 * drowned.sum() / total_land, 1),
        "city_pct_flooded": round(100.0 * (drowned & was_city).sum() /
                                  max(1, int(was_city.sum())), 1),
        "forest_pct_flooded": round(100.0 * (drowned & was_forest).sum() /
                                    max(1, int(was_forest.sum())), 1),
        "water_pct_total": round(100.0 * (blocks == BLOCK_WATER).mean(), 1),
    }
    # `drowned` is the useful one for display: land that the flood took, as
    # opposed to lakes and ocean that were already there.
    return heights, blocks, stats, drowned


def apply_fires(blocks: np.ndarray, fire_points: List[Tuple[float, float]],
                bbox) -> Tuple[np.ndarray, List[Tuple[int, int]]]:
    """
    Convert FIRMS lat/lon detections into grid cells. Each hotspot scorches a
    small radius into dirt (burn scar) and is reported separately so the FPGA
    can draw an animated flame on top of exactly those cells.
    """
    if not fire_points:
        return blocks, []

    w, s, e, n = bbox
    blocks = blocks.copy()
    cells: List[Tuple[int, int]] = []
    seen = set()

    for lat, lon in fire_points:
        if not (w <= lon <= e and s <= lat <= n):
            continue
        x = int((lon - w) / (e - w) * (GRID - 1))
        y = int((n - lat) / (n - s) * (GRID - 1))   # row 0 is north
        x, y = max(0, min(GRID - 1, x)), max(0, min(GRID - 1, y))
        if (x, y) in seen:
            continue
        seen.add((x, y))
        cells.append((x, y))
        # Scorch a 5x5 patch, but never set fire to a lake.
        y0, y1 = max(0, y - 2), min(GRID, y + 3)
        x0, x1 = max(0, x - 2), min(GRID, x + 3)
        patch = blocks[y0:y1, x0:x1]
        patch[patch != BLOCK_WATER] = BLOCK_DIRT
        blocks[y0:y1, x0:x1] = patch

    return blocks, cells[:255]        # header stores the count in one byte


def find_spawn(heights: np.ndarray, blocks: np.ndarray) -> Tuple[int, int, int, int]:
    """
    Find somewhere safe and interesting to drop the player.

    Rules, in order:
      - must be on land, not in water
      - must not be on a cliff edge: no neighbour differs by more than 2 levels
      - must be at least 4 cells from the border so the camera is not clipping
      - among everything that qualifies, pick the one closest to the centre

    Then pick a facing direction: look toward the highest ground within 48
    cells, so the very first frame the judges see has a mountain in it.

    Returns (x, y, z, yaw) where yaw is 0..255 (0 = +X / east, counter-clockwise).
    """
    h = heights.astype(np.int16)

    # Largest height difference against the four neighbours, per cell.
    pad = np.pad(h, 1, mode="edge")
    slope = np.maximum.reduce([
        np.abs(pad[1:-1, 1:-1] - pad[0:-2, 1:-1]),
        np.abs(pad[1:-1, 1:-1] - pad[2:,   1:-1]),
        np.abs(pad[1:-1, 1:-1] - pad[1:-1, 0:-2]),
        np.abs(pad[1:-1, 1:-1] - pad[1:-1, 2:]),
    ])

    border = np.zeros((GRID, GRID), dtype=bool)
    border[4:-4, 4:-4] = True
    land = blocks != BLOCK_WATER

    yy, xx = np.mgrid[0:GRID, 0:GRID]
    centre_dist = np.hypot(xx - GRID / 2.0, yy - GRID / 2.0)

    for max_slope in (2, 3, 5, 31):            # relax the rules if we must
        ok = land & border & (slope <= max_slope)
        if ok.any():
            masked = np.where(ok, centre_dist, np.inf)
            idx = int(np.argmin(masked))
            y, x = divmod(idx, GRID)
            break
    else:
        x = y = GRID // 2                      # should never happen

    z = int(heights[y, x])

    # Face the best view: tallest cell within 48 blocks.
    radius = 48
    y0, y1 = max(0, y - radius), min(GRID, y + radius + 1)
    x0, x1 = max(0, x - radius), min(GRID, x + radius + 1)
    region = heights[y0:y1, x0:x1]
    ry, rx = np.unravel_index(int(np.argmax(region)), region.shape)
    ty, tx = y0 + ry, x0 + rx
    dx, dy = tx - x, y - ty                    # screen y grows south, flip it
    angle = math.degrees(math.atan2(dy, dx)) % 360.0
    yaw = int(round(angle / 360.0 * 256)) & 0xFF

    return int(x), int(y), z, yaw


def build_world(loc: Location, meters_per_block: float, *,
                exaggeration: float = 1.0, smooth: int = 1,
                flood_level: int = 0, flood_meters: Optional[float] = None,
                use_sentinel: bool = False, firms_key: str = "",
                use_fires: bool = False, offline: bool = False,
                cache_dir: str = "cache") -> World:
    """
    The whole pipeline, start to finish. Everything above gets called from here.
    """
    bbox = bbox_from_center(loc.lat, loc.lon, meters_per_block)
    span_km = meters_per_block * GRID / 1000.0
    log(f"world: {span_km:.2f} km x {span_km:.2f} km at {meters_per_block:.0f} m/block",
        "step")
    log(f"bbox W{bbox[0]:.4f} S{bbox[1]:.4f} E{bbox[2]:.4f} N{bbox[3]:.4f}")

    cache_key = f"{slugify(loc.label)}_{int(meters_per_block)}m"
    sources: Dict[str, str] = {}

    # ---- Gather raw layers --------------------------------------------------
    # We always peek at the cache: offline mode needs it, and online mode uses
    # it as a safety net if the live fetch falls all the way through.
    cached = load_cache(cache_dir, cache_key)
    if offline and cached is None:
        log("offline and nothing cached for this place "
            "(run --bake-demos while you have wifi)", "warn")

    if offline and cached is not None:
        elev = cached["elev"]
        landcover = cached.get("landcover")
        indices = cached.get("indices")
        weather = Weather(**cached["weather"]) if cached.get("weather") else Weather()
        elev_source = cached.get("elev_source", "cache")
        fire_points = []
        log("using cached layers (offline mode)", "ok")
    else:
        elev, elev_source = fetch_elevation(bbox, loc.lat)
        landcover = fetch_landcover(bbox)
        indices = fetch_sentinel2(bbox) if use_sentinel else None
        weather = fetch_weather(loc.lat, loc.lon)
        fire_points = fetch_fires(bbox, firms_key) if (use_fires and firms_key) else []

        # If live fetching mostly failed but we have a cache, prefer the cache.
        if elev_source == "synthetic" and cached is not None:
            log("live elevation failed, falling back to cached layers", "warn")
            elev = cached["elev"]
            landcover = cached.get("landcover", landcover)
            indices = cached.get("indices", indices)
            elev_source = cached.get("elev_source", "cache")
        else:
            save_cache(cache_dir, cache_key, elev, landcover, indices, weather,
                       elev_source)

    # If we ended up on synthetic terrain, give it synthetic land cover too,
    # otherwise the fallback world is one flat colour and looks broken.
    if elev_source == "synthetic" and landcover is None:
        landcover = synthetic_landcover(elev, int(abs(loc.lat * 1000)))

    sources["elevation"] = {
        "copernicus": "Copernicus DEM GLO-30 (ESA/AWS Open Data), 30 m",
        "terrarium":  "AWS Terrain Tiles (Mapzen/Terrarium)",
        "openmeteo":  "Open-Meteo Elevation API (coarse)",
        "synthetic":  "PROCEDURAL FALLBACK - not real data",
        "cache":      "locally cached Copernicus DEM",
    }.get(elev_source, elev_source)
    sources["landcover"] = ("ESA WorldCover 10 m v200 (2021)" if landcover is not None
                            else "unavailable - used elevation heuristic")
    sources["weather"] = f"Open-Meteo current conditions ({weather.source})"
    if indices is not None:
        sources["sentinel2"] = (f"Sentinel-2 L2A via Earth Search STAC, scene "
                                f"{indices['scene_id']} on {indices['scene_date']}")

    # ---- Repair elevation holes before anything uses it ----------------------
    elev, filled = fill_dem_voids(elev, landcover)
    if filled:
        log(f"filled {filled} cells of missing elevation data", "ok")

    # ---- Heights ------------------------------------------------------------
    log("building the block world", "step")
    heights, m_per_level, elev_lo, elev_hi = quantize_heights(elev, exaggeration, smooth)
    log(f"elevation {elev_lo:.0f}-{elev_hi:.0f} m mapped to levels 0-31 "
        f"({m_per_level:.1f} m per level, exaggeration {exaggeration}x)")

    # ---- Block types --------------------------------------------------------
    blocks, class_notes = classify_blocks(elev, landcover, indices, weather)
    blocks, refine_notes = refine_classes(blocks, box_blur(elev, smooth), loc.lat,
                                          meters_per_block)
    if filled:
        refine_notes["dem_voids"] = f"{filled} cells of missing elevation filled"
    if refine_notes:
        class_notes["postprocess"] = refine_notes
        for k, v in refine_notes.items():
            log(f"postprocess {k}: {v}")
    heights, water_level = flatten_water(heights, blocks)

    # ---- Flood mode ---------------------------------------------------------
    if flood_meters is not None and flood_meters > 0:
        # Convert real metres of sea level rise into block levels.
        flood_level = int(round(water_level + flood_meters / max(0.01, m_per_level)))
        flood_level = max(0, min(MAX_HEIGHT, flood_level))
        log(f"flood of +{flood_meters:.1f} m = level {flood_level} "
            f"(water sits at level {water_level})")
    heights, blocks, flood_stats, flood_mask = apply_flood(heights, blocks, flood_level)

    # ---- Wildfire mode ------------------------------------------------------
    blocks, fire_cells = apply_fires(blocks, fire_points, bbox)
    if fire_cells:
        sources["fires"] = f"NASA FIRMS {FIRMS_SOURCE}, {len(fire_cells)} hotspots"

    # ---- Spawn --------------------------------------------------------------
    spawn = find_spawn(heights, blocks)
    log(f"spawn at ({spawn[0]}, {spawn[1]}) height {spawn[2]}, facing yaw {spawn[3]}",
        "ok")

    # ---- Stats --------------------------------------------------------------
    counts = {BLOCK_NAMES[b]: int((blocks == b).sum()) for b in range(8)}
    stats: Dict[str, Any] = {
        "span_km": round(span_km, 2),
        "block_counts": counts,
        "block_percent": {k: round(100.0 * v / (GRID * GRID), 1)
                          for k, v in counts.items()},
        "relief_m": round(elev_hi - elev_lo, 1),
        "classification": class_notes,
    }
    if flood_stats:
        stats["flood"] = flood_stats

    return World(location=loc, weather=weather, heights=heights, blocks=blocks,
                 spawn=spawn, meters_per_block=meters_per_block,
                 meters_per_level=m_per_level, elev_min_m=elev_lo,
                 elev_max_m=elev_hi, water_level=water_level,
                 flood_level=flood_level, flood_mask=flood_mask,
                 fires=fire_cells, sources=sources, stats=stats,
                 elev_m=box_blur(elev, smooth), bbox=bbox,
                 exaggeration=float(exaggeration))


# ==============================================================================
# SECTION 11 -- CACHING  (so venue wifi cannot ruin your demo)
# ==============================================================================

def load_cache(cache_dir: str, key: str) -> Optional[Dict[str, Any]]:
    path = os.path.join(cache_dir, key + ".npz")
    if not os.path.exists(path):
        return None
    try:
        z = np.load(path, allow_pickle=True)
        out: Dict[str, Any] = {"elev": z["elev"]}
        if "landcover" in z.files and z["landcover"].size:
            out["landcover"] = z["landcover"]
        if "ndvi" in z.files and z["ndvi"].size:
            out["indices"] = {
                "ndvi": z["ndvi"], "ndwi": z["ndwi"], "ndsi": z["ndsi"],
                "scene_id": str(z["scene_id"]), "scene_date": str(z["scene_date"]),
                "cloud_cover": float(z["cloud_cover"]),
            }
        if "weather" in z.files:
            out["weather"] = json.loads(str(z["weather"]))
        out["elev_source"] = str(z["elev_source"]) if "elev_source" in z.files else "cache"
        return out
    except Exception as exc:
        log(f"cache read failed: {short(exc)}", "warn")
        return None


def save_cache(cache_dir: str, key: str, elev, landcover, indices, weather,
               elev_source: str) -> None:
    if elev_source == "synthetic":
        return                         # never cache fake terrain
    os.makedirs(cache_dir, exist_ok=True)
    payload: Dict[str, Any] = {
        "elev": elev,
        "landcover": landcover if landcover is not None else np.array([]),
        "elev_source": np.array(elev_source),
        "weather": np.array(json.dumps(asdict(weather))),
    }
    if indices is not None:
        payload.update({
            "ndvi": indices["ndvi"], "ndwi": indices["ndwi"], "ndsi": indices["ndsi"],
            "scene_id": np.array(indices["scene_id"]),
            "scene_date": np.array(indices["scene_date"]),
            "cloud_cover": np.array(indices["cloud_cover"]),
        })
    try:
        np.savez_compressed(os.path.join(cache_dir, key + ".npz"), **payload)
        log(f"cached layers to {cache_dir}/{key}.npz", "ok")
    except Exception as exc:
        log(f"cache write failed: {short(exc)}", "warn")


# ==============================================================================
# SECTION 12 -- PACKING  (the exact bytes the FPGA receives)
# ==============================================================================

MAGIC = b"WGEN"
FORMAT_VERSION = 1
HEADER_SIZE = 32


def pack_payload(world: World) -> bytes:
    """
    The 16384-byte terrain block. One byte per cell, row-major, row 0 = NORTH,
    column 0 = WEST.

        bit 7 6 5 4 3 | 2 1 0
            height    | block type
            (0..31)   | (0..7)

        byte = (height << 3) | block_type

    On the FPGA:  height = data[7:3];  block = data[2:0];
    """
    packed = (world.heights.astype(np.uint8) << 3) | (world.blocks.astype(np.uint8) & 0x07)
    return packed.tobytes()


def pack_header(world: World) -> bytes:
    """
    32-byte little-endian header. See FPGA_INTERFACE.md for the field table.
    Keep this in sync with the Verilog struct or everything will look like noise.
    """
    w = world.weather
    sx, sy, sz, yaw = world.spawn

    flags = 0
    if "sentinel2" in world.sources:
        flags |= 1 << 0
    if world.sources.get("landcover", "").startswith("ESA"):
        flags |= 1 << 1
    if world.flood_level > 0:
        flags |= 1 << 2
    if world.fires:
        flags |= 1 << 3

    def u8(v, lo=0, hi=255):
        return int(max(lo, min(hi, round(v))))

    return struct.pack(
        "<4sBBBBBBhBBBBBBBBBBBBhhBBH",
        MAGIC,                                  # 0   4s  "WGEN"
        FORMAT_VERSION,                         # 4   B   format version
        GRID,                                   # 5   B   grid size (128)
        u8(sx),                                 # 6   B   spawn x
        u8(sy),                                 # 7   B   spawn y
        u8(sz),                                 # 8   B   spawn height
        u8(yaw),                                # 9   B   spawn yaw 0..255
        int(max(-3000, min(3000, round(w.temperature_c * 10)))),   # 10 h  temp x10
        u8(w.rain_mm_h * 10),                   # 12  B   rain mm/h x10
        u8(w.snowfall_cm_h * 10),               # 13  B   snow cm/h x10
        u8(w.cloud_cover_pct),                  # 14  B   cloud %
        u8(w.wind_speed_kmh),                   # 15  B   wind km/h
        u8(w.wind_direction_deg / 2.0, 0, 179), # 16  B   wind dir / 2
        u8(w.is_day, 0, 1),                     # 17  B   1 = daytime
        u8(w.weather_code),                     # 18  B   WMO weather code
        u8(world.water_level, 0, 31),           # 19  B   water surface level
        u8(world.flood_level, 0, 31),           # 20  B   flood level (0 = off)
        u8(len(world.fires)),                   # 21  B   fire cell count
        u8(world.meters_per_block),             # 22  B   metres per block
        u8(world.meters_per_level),             # 23  B   metres per height level
        int(max(-9000, min(9000, round(world.location.lat * 100)))),  # 24 h lat x100
        int(max(-18000, min(18000, round(world.location.lon * 100)))),# 26 h lon x100
        u8(w.local_hour / 24.0 * 255),          # 28  B   local time of day
        flags,                                  # 29  B   feature flags
        0,                                      # 30  H   reserved
    )


def pack_frame(world: World) -> bytes:
    """
    The complete wire frame for UART:

        0xA5 0x5A          sync word, 2 bytes
        header             32 bytes
        terrain            16384 bytes
        fire cells         2 bytes each (x, y), header says how many
        crc16              2 bytes, CCITT-FALSE over header+terrain+fires

    Total with no fires: 2 + 32 + 16384 + 2 = 16420 bytes.
    At 115200 baud that is about 1.4 seconds. At 921600 it is 0.18 s.
    """
    header = pack_header(world)
    payload = pack_payload(world)
    fires = b"".join(struct.pack("<BB", x, y) for x, y in world.fires)
    body = header + payload + fires
    return b"\xA5\x5A" + body + struct.pack("<H", crc16_ccitt(body))


# ==============================================================================
# SECTION 12b -- VOXEL GAME TERRAIN  (terrain.bin for the DE1-SoC game)
#
# The game on the board (hackathon/minecraft) does not read the 128x128 byte
# grid above. It reads terrain.bin: a 4 KB header, then 16x16 tiles of 8-byte
# column records (ground height, water surface, top block, tree, weather).
# Its exact layout is the C struct in hackathon/minecraft/terrain.h; this
# section turns our real elevation + land cover + live weather into it.
# ==============================================================================

MC_HEIGHT_LIMIT = 128      # world height of the game
MC_MAX_GROUND = 118        # the game clamps ground and water above this
MC_BASE = 32               # block height of the lowest ground / sea surface
MC_MIN_GROUND = 4

# The game's block ids (minecraft/mc.h). Only these make sense as a surface.
MC_GRASS, MC_DIRT, MC_STONE, MC_SAND, MC_SNOW, MC_COBBLE = 1, 2, 3, 4, 5, 10

# The game's weather ids (minecraft/terrain.h).
WX_SUNNY, WX_CLOUDY, WX_NIGHT, WX_SNOW, WX_RAIN = 0, 1, 2, 3, 4
WX_NAMES = ["sunny", "cloudy", "night", "snow", "rain"]

TERRAIN_MAGIC = b"MCTERR1\0"
TERRAIN_HEADER_SIZE = 4096
TERRAIN_TILE = 16
TERRAIN_RECORD = np.dtype([("height", "<u2"), ("water", "<u2"), ("surface", "u1"),
                           ("feature", "u1"), ("weather", "u1"), ("reserved", "u1")])

# Our land classes -> the block the game puts on top of the column.
CLASS_TO_SURFACE = {
    BLOCK_GRASS: MC_GRASS, BLOCK_SAND: MC_SAND, BLOCK_STONE: MC_STONE,
    BLOCK_SNOW: MC_SNOW, BLOCK_FOREST: MC_GRASS, BLOCK_CITY: MC_COBBLE,
    BLOCK_DIRT: MC_DIRT, BLOCK_WATER: MC_SAND,     # lake / sea bed
}

# WMO weather codes (Open-Meteo "weather_code")
WMO_SNOW = {71, 73, 75, 77, 85, 86}
WMO_RAIN = {51, 53, 55, 56, 57, 61, 63, 65, 66, 67, 80, 81, 82, 95, 96, 99}
WMO_OVERCAST = {3, 45, 48}

LAPSE_C_PER_M = 0.0065     # air cools ~6.5 C per km of altitude


def classify_weather(w: Weather, temp_c: np.ndarray,
                     snow_cover: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Live conditions -> the game's five weathers, per column.

    temp_c is the air temperature at each column (the reported temperature
    corrected for altitude), so rain at the town can be snow on the peaks.
      - snow falling, or rain/drizzle where it is at or below freezing -> SNOW
      - rain, drizzle, showers, thunderstorms                         -> RAIN
      - 70%+ cloud, overcast or fog                                   -> CLOUDY
      - otherwise                                                     -> SUNNY
    After dark, clear and cloudy skies become NIGHT; falling rain or snow
    stays rain or snow (that is what you would notice). Snow-covered ground
    under a mostly cloudy sky also gets SNOW, so glaciers look the part.
    """
    code = int(w.weather_code)
    snowing = w.snowfall_cm_h > 0.05 or code in WMO_SNOW
    raining = w.rain_mm_h > 0.1 or code in WMO_RAIN
    cloudy = w.cloud_cover_pct >= 70 or code in WMO_OVERCAST

    out = np.full(temp_c.shape, WX_CLOUDY if cloudy else WX_SUNNY, dtype=np.uint8)
    if raining:
        out[:] = WX_RAIN
    if snowing:
        out[:] = WX_SNOW
    elif raining:
        out[temp_c <= 0.5] = WX_SNOW
    if snow_cover is not None and w.cloud_cover_pct >= 50:
        out[snow_cover] = WX_SNOW
    if not w.is_day:
        out[(out == WX_SUNNY) | (out == WX_CLOUDY)] = WX_NIGHT
    return out


def _label_regions(mask: np.ndarray) -> Tuple[np.ndarray, int]:
    """4-connected components of a boolean grid (no scipy). Labels 1..n."""
    from collections import deque
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    n = 0
    for y0, x0 in zip(*np.nonzero(mask)):
        if labels[y0, x0]:
            continue
        n += 1
        labels[y0, x0] = n
        q = deque([(y0, x0)])
        while q:
            y, x = q.popleft()
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not labels[ny, nx]:
                    labels[ny, nx] = n
                    q.append((ny, nx))
    return labels, n


def _grow(a: np.ndarray, fn) -> np.ndarray:
    """Combine each cell with its 8 neighbours using fn (np.maximum / np.minimum)."""
    p = np.pad(a, 1, mode="edge")
    out = a.copy()
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            out = fn(out, p[dy:dy + a.shape[0], dx:dx + a.shape[1]])
    return out


def build_terrain_columns(world: World, blocks_per_cell: int = 2) -> Dict[str, Any]:
    """
    Turn the world into the game's column grid, N x N with N = 128 *
    blocks_per_cell (each of our cells becomes blocks_per_cell^2 columns,
    elevation interpolated smoothly between them).

    Heights are true to scale where they fit: one block of height is as many
    metres as one block of width (times the exaggeration). Only terrain too
    tall for the 128-block world is squeezed. Every lake and the sea gets one
    flat surface (the median elevation of its cells) and a bed that deepens
    away from the shore. Forest becomes trees, built-up areas cobblestone.
    The spawn point is moved to column (0, 0), where the game starts you.
    """
    if world.elev_m is None:
        raise ValueError("world has no elevation in metres (built by an old version?)")
    up = max(1, int(blocks_per_cell))
    n = GRID * up
    elev = world.elev_m.astype(np.float64)
    blocks = world.blocks
    water = blocks == BLOCK_WATER

    # ---- one flat surface per body of water (on the 128 grid) --------------
    labels, nregions = _label_regions(water)
    surf_m = np.zeros_like(elev)
    flood_m = None
    if world.flood_level > 0:
        flood_m = world.elev_min_m + world.flood_level * world.meters_per_level
    for r in range(1, nregions + 1):
        cells = labels == r
        level = float(np.median(elev[cells]))
        if flood_m is not None:
            level = max(level, flood_m)
        surf_m[cells] = level

    # ---- upsample: smooth elevation, blocky classes -------------------------
    elev_u = resample_2d(elev, n, n, "bilinear").astype(np.float64)
    cls_u = np.repeat(np.repeat(blocks, up, axis=0), up, axis=1)
    surf_u = np.repeat(np.repeat(surf_m, up, axis=0), up, axis=1)
    water_u = cls_u == BLOCK_WATER

    # ---- metres -> blocks ---------------------------------------------------
    m_per_block = world.meters_per_block / up
    land = ~water_u
    lows = [float(np.percentile(elev_u[land], 1))] if land.any() else []
    highs = [float(np.percentile(elev_u[land], 99.5))] if land.any() else []
    if water_u.any():
        lows.append(float(surf_u[water_u].min()))
        highs.append(float(surf_u[water_u].max()))
    e_lo, e_hi = min(lows), max(highs)
    m_per_level = m_per_block / max(0.05, world.exaggeration)
    room = MC_MAX_GROUND - MC_BASE - 4
    if (e_hi - e_lo) / m_per_level > room:
        m_per_level = (e_hi - e_lo) / room           # too tall: squeeze to fit

    def to_blocks(m):
        return np.round(MC_BASE + (m - e_lo) / m_per_level)

    height = np.clip(to_blocks(elev_u), MC_MIN_GROUND, MC_MAX_GROUND).astype(np.int32)
    water_top = np.where(water_u, np.clip(to_blocks(surf_u), MC_MIN_GROUND + 1,
                                          MC_MAX_GROUND), 0).astype(np.int32)

    # Water beds deepen with distance from the shore (1 block at the edge).
    dist = np.zeros((n, n), dtype=np.int32)
    inner = water_u.copy()
    for d in range(1, 9):
        inner = inner & (_grow(inner.astype(np.int8), np.minimum) > 0)
        dist[inner] = d
    depth = np.clip(1 + dist // max(1, up), 1, 6)
    height = np.where(water_u, np.minimum(height, water_top - depth), height)
    height = np.maximum(height, MC_MIN_GROUND)

    # Dry land never sits below the water right next to it.
    near_water = _grow(water_top, np.maximum)
    height = np.where(land, np.maximum(height, near_water), height)

    # ---- what is on top -----------------------------------------------------
    surface = np.full((n, n), MC_GRASS, dtype=np.uint8)
    for cls, blk in CLASS_TO_SURFACE.items():
        surface[cls_u == cls] = blk
    surface[water_u & (depth > 2)] = MC_DIRT                     # deep beds
    slope = np.maximum(_grow(height, np.maximum) - height, height - _grow(height, np.minimum))
    greenish = land & ((cls_u == BLOCK_GRASS) | (cls_u == BLOCK_FOREST))
    surface[greenish & (slope >= 3)] = MC_STONE                  # cliffs
    # the waterline itself: a sandy strip two columns wide wherever it is low
    shore = land & (_grow(water_u.astype(np.int8), np.maximum) > 0)
    shore2 = land & (_grow(_grow(water_u.astype(np.int8), np.maximum), np.maximum) > 0)
    soft = np.isin(cls_u, (BLOCK_GRASS, BLOCK_STONE, BLOCK_DIRT, BLOCK_SAND))
    surface[shore2 & soft & (slope < 3) & (height <= near_water + 1)] = MC_SAND

    # farmland: patchwork fields (8x8 columns), mostly green, some ploughed
    fy, fx = np.mgrid[0:n, 0:n] // 8
    field = (fx * 0x9E3779B1 + fy * 0x85EBCA77 + 12345) & 0xFFFFFFFF
    field = ((field ^ (field >> 15)) * 0x2C1B3C6D & 0xFFFFFFFF) >> 28
    farm = land & (cls_u == BLOCK_DIRT)
    surface[farm] = np.where(field[farm] < 6, MC_DIRT, MC_GRASS)

    # ---- trees: one candidate per 4x4 block cell, kept by land class -------
    rng = np.random.default_rng(int(abs(world.location.lat * 1e4) + abs(world.location.lon * 1e4)))
    feature = np.zeros((n, n), dtype=np.uint8)
    cell = 4
    for cy in range(0, n, cell):
        for cx in range(0, n, cell):
            y, x = cy + int(rng.integers(cell)), cx + int(rng.integers(cell))
            if y >= n or x >= n or not land[y, x] or surface[y, x] != MC_GRASS:
                continue
            p = 0.4 if cls_u[y, x] == BLOCK_FOREST else 0.03
            if slope[y, x] <= 2 and not shore[y, x] and rng.random() < p:
                feature[y, x] = 1

    # ---- weather, per column -----------------------------------------------
    centre_m = float(elev_u[n // 2, n // 2])
    temp = world.weather.temperature_c - LAPSE_C_PER_M * (elev_u - centre_m)
    weather = classify_weather(world.weather, temp, cls_u == BLOCK_SNOW)

    # ---- put the spawn at (0, 0) -------------------------------------------
    sx, sy = world.spawn[0] * up + up // 2, world.spawn[1] * up + up // 2
    arrays = dict(height=height, water=water_top, surface=surface,
                  feature=feature, weather=weather)
    for k in arrays:
        arrays[k] = np.roll(arrays[k], (-sy, -sx), axis=(0, 1))
    for dy in range(-3, 4):                      # keep the spawn clear of trees
        for dx in range(-3, 4):
            arrays["feature"][dy % n, dx % n] = 0

    w_, s_, e_, n_ = world.bbox
    origin_lon = w_ + (sx + 0.5) / n * (e_ - w_)
    origin_lat = n_ - (sy + 0.5) / n * (n_ - s_)
    sea = int(water_top[water_u].min()) if water_u.any() else MC_BASE

    return dict(size=n, sea_level=sea, origin_lat=origin_lat, origin_lon=origin_lon,
                meters_per_block=m_per_block, meters_per_level=m_per_level,
                source=f"worldgen: {world.location.label}", **arrays)


def pack_terrain_header(cols: Dict[str, Any]) -> bytes:
    """The 128 meaningful bytes of terrain.bin's header (rest is zero)."""
    n = cols["size"]
    return struct.pack("<8s8I3d64s", TERRAIN_MAGIC, 1, TERRAIN_HEADER_SIZE, n, n,
                       TERRAIN_TILE, TERRAIN_RECORD.itemsize, MC_HEIGHT_LIMIT,
                       int(cols["sea_level"]), float(cols["origin_lat"]),
                       float(cols["origin_lon"]), float(cols["meters_per_block"]),
                       cols["source"].encode("utf-8", "replace")[:63])


def terrain_records(cols: Dict[str, Any]) -> np.ndarray:
    """(N, N) array of column records, row 0 = north, column 0 = west."""
    n = cols["size"]
    rec = np.zeros((n, n), dtype=TERRAIN_RECORD)
    for k in ("height", "water", "surface", "feature", "weather"):
        rec[k] = cols[k]
    return rec


def pack_terrain_bin(cols: Dict[str, Any]) -> bytes:
    """The complete terrain.bin: header, then 16x16 tiles of records."""
    n = cols["size"]
    t = TERRAIN_TILE
    tiles = terrain_records(cols).reshape(n // t, t, n // t, t).transpose(0, 2, 1, 3)
    return pack_terrain_header(cols).ljust(TERRAIN_HEADER_SIZE, b"\0") + \
        np.ascontiguousarray(tiles).tobytes()


# ---- compact transfer encoding ("TRZ1") --------------------------------------
#
# terrain.bin is mostly smooth or constant, so for the UART it is sent as 8
# byte-planes (byte k of every record, in north-to-south, west-to-east order),
# each delta coded (byte minus the previous byte, mod 256) and then run-length
# coded with PackBits. A 256x256 world is 512 KB raw and usually a few tens of
# KB like this. The board rebuilds terrain.bin and checks its CRC-32.
#
#   "TRZ1" | u32 width | u32 depth | 128-byte terrain header
#   8 x ( u32 length | PackBits bytes )
#   u32 CRC-32 (zlib) of the complete terrain.bin

def packbits(data: np.ndarray) -> bytes:
    """
    PackBits: control byte c, then
      c in 0..127    -> c + 1 literal bytes follow
      c in 129..255  -> the next byte repeats 257 - c times (2..128)
    """
    a = np.asarray(data, dtype=np.uint8)
    out = bytearray()
    lit = bytearray()

    def flush():
        for i in range(0, len(lit), 128):
            chunk = lit[i:i + 128]
            out.append(len(chunk) - 1)
            out.extend(chunk)
        lit.clear()

    if a.size == 0:
        return b""
    edges = np.flatnonzero(np.diff(a)) + 1
    starts = np.concatenate(([0], edges))
    ends = np.concatenate((edges, [a.size]))
    for s, e in zip(starts.tolist(), ends.tolist()):
        run, v = e - s, int(a[s])
        if run < 3:
            lit.extend(bytes((v,)) * run)
            continue
        flush()
        while run >= 2:
            k = min(run, 128)
            out.append(257 - k)
            out.append(v)
            run -= k
        if run:
            lit.append(v)
    flush()
    return bytes(out)


def unpackbits(data: bytes, size: int) -> np.ndarray:
    """Inverse of packbits (used by the self test; the board has a C copy)."""
    out = bytearray()
    i = 0
    while i < len(data) and len(out) < size:
        c = data[i]
        i += 1
        if c < 128:
            out.extend(data[i:i + c + 1])
            i += c + 1
        elif c > 128:
            out.extend(bytes((data[i],)) * (257 - c))
            i += 1
    return np.frombuffer(bytes(out[:size]), dtype=np.uint8)


def pack_terrain_transfer(cols: Dict[str, Any], terrain_bin: bytes) -> bytes:
    import zlib
    n = cols["size"]
    planes = terrain_records(cols).view(np.uint8).reshape(n * n, TERRAIN_RECORD.itemsize)
    parts = [b"TRZ1", struct.pack("<II", n, n), pack_terrain_header(cols)]
    for k in range(TERRAIN_RECORD.itemsize):
        p = planes[:, k]
        delta = np.diff(p, prepend=np.uint8(0)).astype(np.uint8)   # mod 256
        enc = packbits(delta)
        parts += [struct.pack("<I", len(enc)), enc]
    parts.append(struct.pack("<I", zlib.crc32(terrain_bin) & 0xFFFFFFFF))
    return b"".join(parts)


def unpack_terrain_transfer(blob: bytes) -> bytes:
    """Rebuild terrain.bin from a TRZ1 blob, as the board does (for the self test)."""
    import zlib
    assert blob[:4] == b"TRZ1"
    w, d = struct.unpack_from("<II", blob, 4)
    hdr = blob[12:140]
    off = 140
    planes = np.zeros((w * d, TERRAIN_RECORD.itemsize), dtype=np.uint8)
    for k in range(TERRAIN_RECORD.itemsize):
        (ln,) = struct.unpack_from("<I", blob, off)
        off += 4
        delta = unpackbits(blob[off:off + ln], w * d)
        off += ln
        planes[:, k] = np.cumsum(delta, dtype=np.uint64).astype(np.uint8)
    (crc,) = struct.unpack_from("<I", blob, off)
    t = TERRAIN_TILE
    rec = planes.reshape(d, w, TERRAIN_RECORD.itemsize)
    tiles = rec.reshape(d // t, t, w // t, t, -1).transpose(0, 2, 1, 3, 4)
    out = hdr.ljust(TERRAIN_HEADER_SIZE, b"\0") + np.ascontiguousarray(tiles).tobytes()
    if zlib.crc32(out) & 0xFFFFFFFF != crc:
        raise ValueError("TRZ1 CRC mismatch")
    return out


def describe_terrain(cols: Dict[str, Any]) -> str:
    wx = np.bincount(cols["weather"].ravel(), minlength=5)
    tot = cols["size"] ** 2
    wtxt = ", ".join(f"{WX_NAMES[i]} {100 * c / tot:.0f}%" for i, c in enumerate(wx) if c)
    return (f"{cols['size']}x{cols['size']} columns, ground {int(cols['height'].min())}-"
            f"{int(cols['height'].max())}, sea {cols['sea_level']}, "
            f"{int(cols['feature'].sum())} trees, {cols['meters_per_block']:.0f} m/block, "
            f"{cols['meters_per_level']:.1f} m per level; weather {wtxt}")


# ==============================================================================
# SECTION 13 -- OUTPUT FILES
# ==============================================================================

def write_bin(world: World, path: str) -> None:
    with open(path, "wb") as f:
        f.write(pack_payload(world))
    log(f"wrote {path} ({GRID*GRID} bytes)", "ok")


def write_frame(world: World, path: str) -> None:
    frame = pack_frame(world)
    with open(path, "wb") as f:
        f.write(frame)
    log(f"wrote {path} ({len(frame)} bytes, UART-ready with header + CRC)", "ok")


def write_mif(world: World, path: str) -> None:
    """
    Quartus Memory Initialization File. Point an M10K / altsyncram at this and
    the world is baked into the bitstream: no UART needed for a static demo.
    Consecutive identical bytes are collapsed into ranges to keep it small.
    """
    data = pack_payload(world)
    lines = [f"-- Generated by worldgen.py for {world.location.label}",
             f"-- {GRID}x{GRID} world, byte = (height << 3) | block_type",
             "DEPTH = %d;" % len(data),
             "WIDTH = 8;",
             "ADDRESS_RADIX = HEX;",
             "DATA_RADIX = HEX;",
             "CONTENT BEGIN"]
    i = 0
    while i < len(data):
        j = i
        while j + 1 < len(data) and data[j + 1] == data[i]:
            j += 1
        if j > i:
            lines.append(f"    [{i:04X}..{j:04X}] : {data[i]:02X};")
        else:
            lines.append(f"    {i:04X} : {data[i]:02X};")
        i = j + 1
    lines.append("END;")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    log(f"wrote {path} (Quartus MIF, {len(lines)} lines)", "ok")


def write_hex(world: World, path: str) -> None:
    """Plain hex, one byte per line: what $readmemh wants in simulation."""
    data = pack_payload(world)
    with open(path, "w") as f:
        f.write("\n".join(f"{b:02X}" for b in data) + "\n")
    log(f"wrote {path} ($readmemh format)", "ok")


def world_to_dict(world: World) -> Dict[str, Any]:
    sx, sy, sz, yaw = world.spawn
    return {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "format_version": FORMAT_VERSION,
        "location": {
            "name": world.location.name, "label": world.location.label,
            "latitude": round(world.location.lat, 6),
            "longitude": round(world.location.lon, 6),
            "country": world.location.country,
        },
        "grid": {
            "size": GRID,
            "meters_per_block": world.meters_per_block,
            "span_km": world.stats["span_km"],
            "height_levels": HEIGHT_LEVELS,
            "meters_per_level": round(world.meters_per_level, 2),
            "elevation_min_m": round(world.elev_min_m, 1),
            "elevation_max_m": round(world.elev_max_m, 1),
            "byte_layout": "bits[7:3] = height 0-31, bits[2:0] = block type 0-7",
        },
        "spawn": {"x": sx, "y": sy, "z": sz, "yaw": yaw,
                  "yaw_degrees": round(yaw / 256.0 * 360.0, 1)},
        "weather": asdict(world.weather),
        "water_level": world.water_level,
        "flood_level": world.flood_level,
        "fires": [{"x": x, "y": y} for x, y in world.fires],
        "block_types": {str(k): v for k, v in BLOCK_NAMES.items()},
        "block_colors_rgb": {BLOCK_NAMES[k]: list(v) for k, v in BLOCK_RGB.items()},
        "stats": world.stats,
        "data_sources": world.sources,
        "attribution": [
            "Elevation: Copernicus DEM GLO-30, (c) DLR/ESA, via AWS Open Data",
            "Land cover: (c) ESA WorldCover project 2021 / Contains modified "
            "Copernicus Sentinel data (2021) processed by ESA WorldCover consortium",
            "Imagery: Contains modified Copernicus Sentinel data, via Element 84 "
            "Earth Search on AWS",
            "Weather: Open-Meteo.com (CC BY 4.0)",
            "Active fires: NASA FIRMS (LANCE/EOSDIS)",
        ],
    }


def write_json(world: World, path: str) -> None:
    with open(path, "w") as f:
        json.dump(world_to_dict(world), f, indent=2)
    log(f"wrote {path}", "ok")


# ==============================================================================
# SECTION 14 -- PREVIEWS  (you must be able to see that it worked)
# ==============================================================================

def ascii_preview(world: World, color: bool = True, step: int = 2) -> str:
    """
    Draw the world in the terminal. `step` of 2 means 64x64 characters, which
    fits a normal window. Colour mode uses 24-bit ANSI and looks genuinely good
    on a projector; set color=False for a plain-text version you can paste
    anywhere.
    """
    h, b = world.heights, world.blocks
    sx, sy, _, _ = world.spawn
    fire_set = set(world.fires)
    rows = []

    for y in range(0, GRID, step):
        line = []
        for x in range(0, GRID, step):
            bt = int(b[y, x])
            ht = int(h[y, x])
            is_flood = (world.flood_mask is not None
                        and bool(world.flood_mask[y, x]))
            near_spawn = abs(x - sx) < step and abs(y - sy) < step
            is_fire = any((fx, fy) in fire_set
                          for fx in range(x, min(GRID, x + step))
                          for fy in range(y, min(GRID, y + step)))
            if color:
                r, g, bl = BLOCK_RGB[bt]
                # Shade by height so relief is visible: 55% dark to 115% bright.
                shade = 0.55 + 0.60 * (ht / MAX_HEIGHT)
                r, g, bl = (min(255, int(c * shade)) for c in (r, g, bl))
                ch = "██"
                if near_spawn:
                    line.append(f"\033[48;2;{r};{g};{bl}m\033[38;2;255;255;0m[]\033[0m")
                elif is_fire:
                    line.append(f"\033[48;2;{r};{g};{bl}m\033[38;2;255;80;0m><\033[0m")
                else:
                    line.append(f"\033[38;2;{r};{g};{bl}m{ch}\033[0m")
            else:
                if near_spawn:
                    line.append("P")
                elif is_fire:
                    line.append("!")
                elif is_flood:
                    line.append("w")          # newly flooded land
                else:
                    line.append(BLOCK_CHAR[bt])
        rows.append("".join(line))

    out = ["", f"  {world.location.label}   "
               f"{world.stats['span_km']:.1f} km across   "
               f"{world.elev_min_m:.0f}-{world.elev_max_m:.0f} m elevation", ""]
    out.extend("  " + r for r in rows)
    out.append("")

    legend = "  ".join(
        (f"\033[38;2;{BLOCK_RGB[b_][0]};{BLOCK_RGB[b_][1]};{BLOCK_RGB[b_][2]}m"
         f"██\033[0m {BLOCK_NAMES[b_]}" if color
         else f"{BLOCK_CHAR[b_]} {BLOCK_NAMES[b_]}")
        for b_ in range(8))
    out.append("  " + legend)

    pct = world.stats["block_percent"]
    out.append("  " + "  ".join(f"{k} {v}%" for k, v in pct.items() if v > 0))
    w = world.weather
    out.append(f"  weather: {w.temperature_c:.1f} C, rain {w.rain_mm_h:.1f} mm/h, "
               f"snow {w.snowfall_cm_h:.1f} cm/h, cloud {w.cloud_cover_pct:.0f}%, "
               f"wind {w.wind_speed_kmh:.0f} km/h, "
               f"{'day' if w.is_day else 'night'}")
    sx, sy, sz, yaw = world.spawn
    out.append(f"  spawn: x={sx} y={sy} z={sz} facing {yaw/256*360:.0f} deg  "
               f"[P] marks the spot")
    if world.flood_level:
        fs = world.stats.get("flood", {})
        out.append(f"  FLOOD: level {world.flood_level} submerges "
                   f"{fs.get('land_pct_flooded', 0)}% of land, "
                   f"{fs.get('city_pct_flooded', 0)}% of built-up area")
    if world.fires:
        out.append(f"  FIRES: {len(world.fires)} active hotspots from NASA FIRMS")
    out.append("")
    return "\n".join(out)


def write_png(world: World, path: str, scale: int = 5) -> bool:
    """
    A top-down map with hillshading. This is the image the website shows and
    the one that goes in the pitch deck, so it is worth doing properly:
    colours by block type, brightness by a real lighting calculation.
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        log("pillow not installed, skipping PNG preview", "warn")
        return False

    h = world.heights.astype(np.float32)
    rgb = np.zeros((GRID, GRID, 3), dtype=np.float32)
    for bt, col in BLOCK_RGB.items():
        m = world.blocks == bt
        rgb[m] = col

    # Hillshade: light from the north-west, the cartographic convention.
    # Gradients here are in height levels per cell, typically 0 to 3, so the
    # multiplier is small. Turn it up and every slope blows out to black/white.
    gy, gx = np.gradient(box_blur(h, 1))
    shade = np.clip(0.88 + 0.22 * (-gx - gy), 0.45, 1.35)
    # Height also brightens things slightly so peaks read as peaks.
    lift = 0.88 + 0.20 * (h / MAX_HEIGHT)
    rgb = np.clip(rgb * shade[:, :, None] * lift[:, :, None], 0, 255)

    # Water gets a flat specular sheen instead of hillshade noise.
    water = world.blocks == BLOCK_WATER
    rgb[water] = np.array(BLOCK_RGB[BLOCK_WATER], dtype=np.float32) * 1.05
    if world.flood_mask is not None and world.flood_mask.any():
        # Floodwater gets its own paler, slightly sickly tint so a judge can
        # see at a glance exactly which land the water took.
        rgb[world.flood_mask] = np.array([108, 170, 206], dtype=np.float32)

    img = Image.fromarray(rgb.astype(np.uint8), "RGB")
    img = img.resize((GRID * scale, GRID * scale), Image.NEAREST)
    draw = ImageDraw.Draw(img)

    # Fires
    for x, y in world.fires:
        cx, cy = x * scale + scale // 2, y * scale + scale // 2
        draw.ellipse([cx - scale, cy - scale, cx + scale, cy + scale],
                     fill=(255, 110, 20), outline=(255, 220, 90))

    # Spawn marker: a yellow ring with a facing tick.
    sx, sy, _, yaw = world.spawn
    cx, cy = sx * scale + scale // 2, sy * scale + scale // 2
    r = scale * 2
    draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(255, 232, 60), width=2)
    ang = math.radians(yaw / 256.0 * 360.0)
    draw.line([cx, cy, cx + math.cos(ang) * r * 2.2, cy - math.sin(ang) * r * 2.2],
              fill=(255, 232, 60), width=2)

    img.save(path)
    log(f"wrote {path} ({GRID*scale}x{GRID*scale} px)", "ok")
    return True


# ==============================================================================
# SECTION 15 -- SENDING TO THE BOARD OVER UART
# ==============================================================================

# The board runs `terrad` (hackathon/minecraft/terrad.c), which owns the
# board's console UART, runs the game, and swaps in each new world:
#
#   PC -> board   "TRRA" | u8 type | u8 seq | u16 0 | u32 length
#                 | u32 CRC-32(payload) | u32 CRC-32(previous 16 bytes) | payload
#                 type 1 = WORLD (payload: TRZ1 blob), 2 = PING, 3 = CONSOLE
#   board -> PC   text lines "@TERRAD READY | RECV s n | OK s | ERR s why | PONG s"
#
# Kernel messages share the same UART, so replies are found by their prefix.

FRAME_WORLD, FRAME_PING, FRAME_CONSOLE = 1, 2, 3
_board_seq = 0


def pack_board_frame(ftype: int, seq: int, payload: bytes = b"") -> bytes:
    import zlib
    head = struct.pack("<4sBBHII", b"TRRA", ftype, seq & 0xFF, 0, len(payload),
                       zlib.crc32(payload) & 0xFFFFFFFF)
    return head + struct.pack("<I", zlib.crc32(head) & 0xFFFFFFFF) + payload


def board_send(port: str, baud: int, ftype: int, payload: bytes = b"",
               expect: Optional[str] = "OK", progress=None) -> Tuple[bool, str]:
    """
    Send one frame to terrad and wait for its answer. Returns (ok, reply).
    expect is the reply word that means success (OK / PONG / CONSOLE), or
    None to not wait at all.
    """
    global _board_seq
    try:
        import serial
    except ImportError:
        return False, "pyserial not installed: pip install pyserial"

    _board_seq = (_board_seq + 1) & 0xFF
    seq = _board_seq
    frame = pack_board_frame(ftype, seq, payload)
    # the line takes ~10 bits per byte; allow for decoding and writing on the board
    deadline_s = len(frame) * 10.0 / baud * 1.5 + 15.0
    try:
        with serial.Serial(port, baud, timeout=0.2, write_timeout=deadline_s) as ser:
            ser.reset_input_buffer()
            t0 = time.time()
            for i in range(0, len(frame), 1024):
                ser.write(frame[i:i + 1024])
                if progress:
                    progress(min(len(frame), i + 1024), len(frame))
            ser.flush()
            if expect is None:
                return True, "sent"
            buf = b""
            while time.time() - t0 < deadline_s:
                buf += ser.read(256)
                for raw in buf.split(b"\n"):
                    line = raw.decode("ascii", "replace").strip()
                    if not line.startswith("@TERRAD "):
                        continue
                    words = line.split()
                    if len(words) >= 3 and words[1] in (expect, "ERR") and words[2] == str(seq):
                        return words[1] == expect, line[len("@TERRAD "):]
                    if expect == "CONSOLE" and words[1:2] == ["CONSOLE"]:
                        return True, "CONSOLE"
            return False, "no answer from the board (is terrad running?)"
    except Exception as exc:
        return False, f"serial error: {short(exc)}"


def send_uart(world: World, port: str, baud: int = 115200,
              blocks_per_cell: int = 2) -> bool:
    """Send the world to the game on the board (via terrad). Needs pyserial."""
    cols = build_terrain_columns(world, blocks_per_cell)
    tb = pack_terrain_bin(cols)
    blob = pack_terrain_transfer(cols, tb)
    return send_terrain_blob(blob, port, baud, world.location.label)


def send_terrain_blob(blob: bytes, port: str, baud: int, label: str = "",
                      progress=None) -> bool:
    log(f"sending {label or 'world'} to the board: {len(blob)} bytes on {port} at "
        f"{baud} baud (~{len(blob) * 10 / baud:.0f} s)", "step")
    t0 = time.time()
    for attempt in (1, 2):
        ok, msg = board_send(port, baud, FRAME_WORLD, blob, progress=progress)
        if ok:
            log(f"board has the new world ({time.time() - t0:.1f} s), game restarting", "ok")
            return True
        log(f"board transfer failed: {msg}" + ("; retrying" if attempt == 1 else ""), "warn")
    return False


class BoardPusher:
    """
    Background sender for --serve: each new destination is queued, and only
    the newest one waiting is sent once the current transfer finishes (picking
    five places quickly sends the first and the last, not all five).
    """

    def __init__(self, port: str, baud: int):
        import itertools
        import threading
        self.port, self.baud = port, baud
        self.cond = threading.Condition()
        self.pending: Optional[Tuple[bytes, str, Any]] = None
        self.ticket = itertools.count(1)     # order in which requests arrived
        self.newest = 0                      # newest request already queued/sent
        self.state = {"state": "idle", "place": "", "detail": "", "upload_id": None,
                      "progress": 0, "time": time.time()}
        threading.Thread(target=self._run, daemon=True).start()

    def take_ticket(self) -> int:
        """Call when a request arrives; pass the number to push()."""
        with self.cond:
            return next(self.ticket)

    def push(self, blob: bytes, label: str, ticket: int, upload_id: Any = None) -> None:
        """Queue a world, unless a request that arrived later got there first
        (picks are slow to build, so they can finish out of order)."""
        with self.cond:
            if ticket < self.newest:
                log(f"not sending {label}: a newer destination was picked", "info")
                return
            self.newest = ticket
            self.pending = (blob, label, upload_id)
            self.state.update(state="queued", place=label, upload_id=upload_id,
                              progress=0, detail="", time=time.time())
            self.cond.notify()

    def status(self) -> Dict[str, Any]:
        with self.cond:
            return dict(self.state, queued=self.pending is not None)

    def _set(self, **kw) -> None:
        with self.cond:
            self.state.update(kw, time=time.time())

    def _run(self) -> None:
        while True:
            with self.cond:
                while self.pending is None:
                    self.cond.wait()
                blob, label, uid = self.pending
                self.pending = None
            self._set(state="sending", place=label, upload_id=uid, progress=0,
                      detail=f"{len(blob)} bytes")

            def progress(done, total):
                self._set(progress=int(100 * done / max(1, total)))

            ok = send_terrain_blob(blob, self.port, self.baud, label, progress)
            with self.cond:
                superseded = self.pending is not None
            self._set(state="done" if ok else "failed", place=label, upload_id=uid,
                      progress=100 if ok else self.state["progress"],
                      detail="game restarted on the new world" if ok else
                      "no answer from the board (is it on, and terrad installed?)")
            if superseded:
                log("another upload is waiting; sending it next", "info")


# ==============================================================================
# SECTION 16 -- OPTIONAL AI NARRATION  (Gemini + ElevenLabs tracks)
# ==============================================================================

def narrate_with_gemini(world: World, api_key: str,
                        model: str = "gemini-2.5-flash") -> Optional[str]:
    """
    Ask Gemini to describe the place a player just landed in, using the real
    numbers we measured rather than whatever it remembers about the city.
    That grounding is the interesting part: the model is reading our satellite
    derived statistics, not hallucinating a travel brochure.
    """
    if not api_key:
        return None
    log("writing narration with Gemini", "step")
    s = world.stats
    w = world.weather
    facts = (
        f"Place: {world.location.label} ({world.location.lat:.3f}, {world.location.lon:.3f}).\n"
        f"Area shown: {s['span_km']} km square.\n"
        f"Elevation range in view: {world.elev_min_m:.0f} m to {world.elev_max_m:.0f} m.\n"
        f"Terrain composition measured from satellite: "
        + ", ".join(f"{k} {v}%" for k, v in s['block_percent'].items() if v > 0) + ".\n"
        f"Live weather: {w.temperature_c:.1f} C, rain {w.rain_mm_h:.1f} mm/h, "
        f"snowfall {w.snowfall_cm_h:.1f} cm/h, cloud cover {w.cloud_cover_pct:.0f}%, "
        f"wind {w.wind_speed_kmh:.0f} km/h, {'daytime' if w.is_day else 'night'}.\n"
    )
    if world.flood_level and "flood" in s:
        facts += (f"Flood simulation active: {s['flood']['land_pct_flooded']}% of land "
                  f"and {s['flood']['city_pct_flooded']}% of built-up area submerged.\n")
    if world.fires:
        facts += f"{len(world.fires)} active wildfire hotspots detected by NASA FIRMS.\n"

    prompt = (
        "You are the narrator of an exploration game. The player has just landed "
        "in a procedurally reconstructed version of a real place, built from "
        "satellite data. Write 3 to 4 sentences, under 80 words, spoken aloud to "
        "the player as they arrive. Be vivid and specific. Use the measurements "
        "below as the source of truth about what they can see; do not invent "
        "landmarks. Mention the weather because they are standing in it.\n\n"
        + facts)

    try:
        r = session().post(
            f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent",
            headers={"x-goog-api-key": api_key, "Content-Type": "application/json"},
            json={"contents": [{"parts": [{"text": prompt}]}],
                  "generationConfig": {"temperature": 0.9, "maxOutputTokens": 300}},
            timeout=HTTP_TIMEOUT)
        r.raise_for_status()
        parts = r.json()["candidates"][0]["content"]["parts"]
        text = "".join(p.get("text", "") for p in parts).strip()
        log(text, "ok")
        return text
    except Exception as exc:
        log(f"Gemini failed: {short(exc)}", "warn")
        return None


def speak_with_elevenlabs(text: str, api_key: str, path: str,
                          voice_id: str = "21m00Tcm4TlvDq8ikWAM") -> bool:
    """Turn the narration into an mp3. Voice IDs come from your ElevenLabs page."""
    if not (api_key and text):
        return False
    log("synthesising narration with ElevenLabs", "step")
    try:
        r = session().post(
            f"https://api.elevenlabs.io/v1/text-to-speech/{voice_id}",
            headers={"xi-api-key": api_key, "Content-Type": "application/json",
                     "Accept": "audio/mpeg"},
            json={"text": text, "model_id": "eleven_multilingual_v2",
                  "voice_settings": {"stability": 0.45, "similarity_boost": 0.75}},
            timeout=60)
        r.raise_for_status()
        with open(path, "wb") as f:
            f.write(r.content)
        log(f"wrote {path} ({len(r.content)//1024} KB)", "ok")
        return True
    except Exception as exc:
        log(f"ElevenLabs failed: {short(exc)}", "warn")
        return False


# ==============================================================================
# SECTION 17 -- OUTPUT ORCHESTRATION
# ==============================================================================

def write_terrain(world: World, out_dir: str, blocks_per_cell: int = 2) -> Dict[str, str]:
    """terrain.bin for the voxel game, and terrain.trz (the same, packed for UART)."""
    cols = build_terrain_columns(world, blocks_per_cell)
    tb = pack_terrain_bin(cols)
    blob = pack_terrain_transfer(cols, tb)
    paths = {"terrain_bin": os.path.join(out_dir, "terrain.bin"),
             "terrain_trz": os.path.join(out_dir, "terrain.trz")}
    with open(paths["terrain_bin"], "wb") as f:
        f.write(tb)
    with open(paths["terrain_trz"], "wb") as f:
        f.write(blob)
    world.terrain_blob = blob       # in memory: the server must not re-read a shared file
    log(f"wrote {paths['terrain_bin']} ({describe_terrain(cols)})", "ok")
    log(f"wrote {paths['terrain_trz']} ({len(blob)} bytes for the UART)", "ok")
    world.stats["terrain"] = {"columns": cols["size"], "trees": int(cols["feature"].sum()),
                              "transfer_bytes": len(blob),
                              "weather": WX_NAMES[int(np.bincount(cols["weather"].ravel()).argmax())]}
    return paths


def emit_all(world: World, out_dir: str, *, png_scale: int = 5,
             gemini_key: str = "", eleven_key: str = "",
             gemini_model: str = "gemini-2.5-flash",
             voice_id: str = "21m00Tcm4TlvDq8ikWAM",
             blocks_per_cell: int = 2) -> Dict[str, str]:
    """Write every output file. Returns a map of name -> path."""
    os.makedirs(out_dir, exist_ok=True)
    paths: Dict[str, str] = {}

    log("writing output files", "step")
    p = os.path.join(out_dir, "world.bin");    write_bin(world, p);   paths["bin"] = p
    p = os.path.join(out_dir, "world.frame");  write_frame(world, p); paths["frame"] = p
    p = os.path.join(out_dir, "world.mif");    write_mif(world, p);   paths["mif"] = p
    p = os.path.join(out_dir, "world.hex");    write_hex(world, p);   paths["hex"] = p
    paths.update(write_terrain(world, out_dir, blocks_per_cell))
    p = os.path.join(out_dir, "world.json");   write_json(world, p);  paths["json"] = p

    p = os.path.join(out_dir, "preview.txt")
    with open(p, "w") as f:
        f.write(ascii_preview(world, color=False, step=1))
    paths["preview_txt"] = p
    log(f"wrote {p}", "ok")

    p = os.path.join(out_dir, "preview.png")
    if write_png(world, p, png_scale):
        paths["preview_png"] = p

    narration = None
    if gemini_key:
        narration = narrate_with_gemini(world, gemini_key, gemini_model)
        if narration:
            p = os.path.join(out_dir, "narration.txt")
            with open(p, "w") as f:
                f.write(narration + "\n")
            paths["narration_txt"] = p
            # Fold it into world.json too so the website gets it in one request.
            data = world_to_dict(world)
            data["narration"] = narration
            with open(paths["json"], "w") as f:
                json.dump(data, f, indent=2)
    if eleven_key and narration:
        p = os.path.join(out_dir, "narration.mp3")
        if speak_with_elevenlabs(narration, eleven_key, p, voice_id):
            paths["narration_mp3"] = p

    return paths


# ==============================================================================
# SECTION 18 -- HTTP SERVER  (so the website teammate can just call a URL)
# ==============================================================================

def serve(host: str, port: int, out_dir: str, cache_dir: str, defaults) -> None:
    """
    A tiny no-dependency web service.

        GET /                      human readable help
        GET /demos                 the baked demo locations
        GET /generate?place=Banff&scale=60&exaggeration=1.4&flood_meters=0
                                   -> JSON metadata, writes all output files
        GET /files/world.bin       the raw 16 KB world
        GET /files/preview.png     the map image
        GET /files/<name>          anything else in the output directory

    CORS is wide open because this only ever runs on localhost during a demo.
    """
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    import itertools
    import threading
    pusher = BoardPusher(defaults.uart, defaults.baud) if defaults.uart else None
    if pusher:
        log(f"Upload sends worlds to the board on {defaults.uart}", "ok")
    # Worlds built by /generate, kept in memory until someone uploads them
    # (every request writes the same out/ files, so those can't be trusted).
    built: Dict[str, Tuple[bytes, str]] = {}
    built_order: List[str] = []
    built_lock = threading.Lock()
    build_ids = itertools.count(1)

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt, *args):
            log(f"http {fmt % args}")

        def do_OPTIONS(self):
            self._send(204, b"")

        def do_GET(self):
            parsed = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(parsed.query)
            route = parsed.path.rstrip("/") or "/"

            def arg(name, default, cast=str):
                try:
                    return cast(q[name][0])
                except (KeyError, ValueError, IndexError):
                    return default

            if route == "/":
                help_text = {
                    "service": "StormHacks world generator",
                    "endpoints": {
                        "/generate": "?place=<name>&scale=<m/block>&exaggeration=<f>"
                                     "&flood_meters=<m>&sentinel=0|1&fires=0|1",
                        "/demos": "list of pre-cached demo locations",
                        "/upload": "?id=<upload_id from /generate>: send that world to the board",
                        "/board": "state of the transfer to the board (--uart)",
                        "/files/<name>": "download generated files",
                    },
                    "block_types": {str(k): v for k, v in BLOCK_NAMES.items()},
                }
                return self._send(200, json.dumps(help_text, indent=2).encode())

            if route == "/board":
                body = pusher.status() if pusher else {"state": "disabled",
                                                       "detail": "start with --uart PORT"}
                return self._send(200, json.dumps(body).encode())

            if route == "/upload":
                if not pusher:
                    return self._send(503, json.dumps({
                        "error": "no board connected: start the server with --uart PORT "
                                 "(./run.sh does that)"}).encode())
                uid = arg("id", "")
                with built_lock:
                    entry = built.get(uid)
                if entry is None:
                    return self._send(404, json.dumps({
                        "error": "that world is gone; press Generate again"}).encode())
                pusher.push(entry[0], entry[1], pusher.take_ticket(), uid)
                return self._send(200, json.dumps({"state": "queued", "upload_id": uid,
                                                   "place": entry[1]}).encode())

            if route == "/demos":
                demos = [{"key": k, "place": p, "scale": s, "note": note}
                         for k, p, s, note in DEMO_LOCATIONS]
                return self._send(200, json.dumps(demos, indent=2).encode())

            if route.startswith("/files/"):
                name = os.path.basename(parsed.path[len("/files/"):])
                fpath = os.path.join(out_dir, name)
                if not os.path.isfile(fpath):
                    return self._send(404, json.dumps({"error": "not found"}).encode())
                ctype = {".png": "image/png", ".json": "application/json",
                         ".txt": "text/plain; charset=utf-8", ".mp3": "audio/mpeg",
                         ".mif": "text/plain", ".hex": "text/plain"}.get(
                             os.path.splitext(name)[1], "application/octet-stream")
                with open(fpath, "rb") as f:
                    return self._send(200, f.read(), ctype)

            if route == "/generate":
                place = arg("place", "")
                lat = arg("lat", None, float)
                lon = arg("lon", None, float)
                if not place and lat is None:
                    return self._send(400, json.dumps(
                        {"error": "need ?place= or ?lat=&lon="}).encode())
                try:
                    if lat is not None and lon is not None:
                        loc = Location(name=place or f"{lat:.3f},{lon:.3f}",
                                       lat=lat, lon=lon)
                    else:
                        loc = geocode(place)
                        if loc is None:
                            return self._send(404, json.dumps(
                                {"error": f"could not find {place!r}"}).encode())

                    world = build_world(
                        loc,
                        arg("scale", defaults.scale, float),
                        exaggeration=arg("exaggeration", defaults.exaggeration, float),
                        smooth=arg("smooth", defaults.smooth, int),
                        flood_level=arg("flood", 0, int),
                        flood_meters=arg("flood_meters", None, float),
                        use_sentinel=bool(arg("sentinel", int(defaults.sentinel), int)),
                        firms_key=defaults.firms_key,
                        use_fires=bool(arg("fires", int(defaults.fires), int)),
                        offline=bool(arg("offline", int(defaults.offline), int)),
                        cache_dir=cache_dir)

                    paths = emit_all(world, out_dir, png_scale=defaults.png_scale,
                                     gemini_key=defaults.gemini_key,
                                     eleven_key=defaults.eleven_key,
                                     gemini_model=defaults.gemini_model,
                                     voice_id=defaults.voice,
                                     blocks_per_cell=defaults.blocks_per_cell)

                    # Keep the world for a later /upload (the newest 16).
                    upload_id = None
                    if world.terrain_blob:
                        upload_id = str(next(build_ids))
                        with built_lock:
                            built[upload_id] = (world.terrain_blob, world.location.label)
                            built_order.append(upload_id)
                            while len(built_order) > 16:
                                built.pop(built_order.pop(0), None)

                    body = world_to_dict(world)
                    body["files"] = {k: f"/files/{os.path.basename(v)}"
                                     for k, v in paths.items()}
                    # Handy for the website: the whole world as base64, no second fetch.
                    body["world_base64"] = base64.b64encode(pack_payload(world)).decode()
                    body["upload_id"] = upload_id
                    body["board"] = bool(pusher)
                    if "narration_txt" in paths:
                        with open(paths["narration_txt"]) as f:
                            body["narration"] = f.read().strip()
                    return self._send(200, json.dumps(body).encode())
                except Exception as exc:
                    import traceback
                    traceback.print_exc()
                    return self._send(500, json.dumps({"error": str(exc)}).encode())

            self._send(404, json.dumps({"error": "unknown route"}).encode())

    os.makedirs(out_dir, exist_ok=True)
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"\n  World generator listening on http://{host}:{port}")
    print(f"  Try:  http://{host}:{port}/generate?place=Vancouver&scale=60\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n  stopped")


# ==============================================================================
# SECTION 19 -- DEMO BAKING
# ==============================================================================

def bake_demos(args) -> None:
    """
    Run every demo location once, with good wifi, so the layers land in the
    cache. After this you can run with --offline and nothing touches the network.
    DO THIS BEFORE YOU PRESENT.
    """
    log("baking demo locations into the cache", "step")
    ok, failed = [], []
    for key, place, scale, note in DEMO_LOCATIONS:
        print(f"\n{'='*70}\n  {key}: {place} @ {scale} m/block  ({note})\n{'='*70}")
        loc = geocode(place)
        if loc is None:
            failed.append(key)
            continue
        try:
            world = build_world(loc, scale, exaggeration=args.exaggeration,
                                smooth=args.smooth, use_sentinel=args.sentinel,
                                firms_key=args.firms_key, use_fires=args.fires,
                                cache_dir=args.cache_dir)
            demo_out = os.path.join(args.out, "demos", key)
            emit_all(world, demo_out, png_scale=args.png_scale)
            print(ascii_preview(world, color=args.color, step=2))
            ok.append(key)
        except Exception as exc:
            log(f"{key} failed: {short(exc)}", "err")
            failed.append(key)

    print(f"\n{'='*70}")
    log(f"baked {len(ok)}/{len(DEMO_LOCATIONS)}: {', '.join(ok)}", "ok")
    if failed:
        log(f"failed: {', '.join(failed)}", "warn")
    log(f"demo outputs are in {os.path.join(args.out, 'demos')}")
    log("you can now run with --offline and no network at all")


# ==============================================================================
# SECTION 19b -- SELF TEST
# Run `python worldgen.py --self-test` any time you change the format. It needs
# no network and it checks the exact things that would silently break the FPGA.
# ==============================================================================

def self_test() -> int:
    """Verify the byte contract end to end. Returns 0 if everything passes."""
    print("\n" + "=" * 70)
    print("  SELF TEST  (no network needed)")
    print("=" * 70)
    failures: List[str] = []

    def check(name: str, condition: bool, detail: str = "") -> None:
        status = "PASS" if condition else "FAIL"
        print(f"  [{status}] {name}" + (f"   {detail}" if detail else ""))
        if not condition:
            failures.append(name)

    # Build a world with no network at all.
    loc = Location(name="Self Test", lat=49.25, lon=-123.0, country="Canada")
    elev = synthetic_elevation(42)
    cover = synthetic_landcover(elev, 42)
    weather = Weather(temperature_c=-4.0, rain_mm_h=0.0, snowfall_cm_h=0.8,
                      cloud_cover_pct=88, wind_speed_kmh=23,
                      wind_direction_deg=240, is_day=1, weather_code=73,
                      local_hour=15.5, source="self-test")

    heights, mpl, lo, hi = quantize_heights(elev, 1.5, 1)
    blocks, _ = classify_blocks(elev, cover, None, weather)
    heights, water_level = flatten_water(heights, blocks)
    spawn = find_spawn(heights, blocks)
    world = World(location=loc, weather=weather, heights=heights, blocks=blocks,
                  spawn=spawn, meters_per_block=30.0, meters_per_level=mpl,
                  elev_min_m=lo, elev_max_m=hi, water_level=water_level,
                  elev_m=box_blur(elev, 1), bbox=bbox_from_center(loc.lat, loc.lon, 30.0),
                  exaggeration=1.5,
                  stats={"span_km": 3.84,
                         "block_percent": {BLOCK_NAMES[b]:
                                           round(100.0 * (blocks == b).mean(), 1)
                                           for b in range(8)}})

    # --- ranges -------------------------------------------------------------
    check("heights stay inside 0..31",
          heights.min() >= 0 and heights.max() <= MAX_HEIGHT,
          f"min {heights.min()} max {heights.max()}")
    check("block types stay inside 0..7",
          blocks.min() >= 0 and blocks.max() <= 7,
          f"min {blocks.min()} max {blocks.max()}")
    check("grid is exactly 128x128", heights.shape == (GRID, GRID))

    # --- packing round trip -------------------------------------------------
    payload = pack_payload(world)
    check("payload is 16384 bytes", len(payload) == GRID * GRID, f"{len(payload)}")
    unpacked = np.frombuffer(payload, dtype=np.uint8).reshape(GRID, GRID)
    check("height survives the round trip",
          np.array_equal(unpacked >> 3, heights))
    check("block type survives the round trip",
          np.array_equal(unpacked & 0x07, blocks))

    # --- header -------------------------------------------------------------
    header = pack_header(world)
    check("header is exactly 32 bytes", len(header) == HEADER_SIZE, f"{len(header)}")
    check("header starts with the WGEN magic", header[:4] == MAGIC)
    f = struct.unpack("<4sBBBBBBhBBBBBBBBBBBBhhBBH", header)
    # f[0]=magic f[1]=version f[2]=grid f[3..6]=spawn x,y,z,yaw f[7]=temp x10
    check("header version field", f[1] == FORMAT_VERSION)
    check("header grid field says 128", f[2] == 128, f"{f[2]}")
    check("header spawn matches the computed spawn",
          (f[3], f[4], f[5], f[6]) == spawn,
          f"{(f[3], f[4], f[5], f[6])} vs {spawn}")
    check("header temperature decodes correctly",
          abs(f[7] / 10.0 - weather.temperature_c) < 0.05, f"{f[7]/10.0} C")
    # f[15]=water_level f[16]=flood_level f[17]=fire_count f[18]=m/block
    check("header water level matches", f[15] == water_level, f"{f[15]}")
    check("header metres-per-block matches", f[18] == 30, f"{f[18]}")
    check("header fire count is zero here", f[17] == 0)

    # --- frame and CRC ------------------------------------------------------
    frame = pack_frame(world)
    check("frame starts with the A5 5A sync word", frame[:2] == b"\xA5\x5A")
    check("frame length is 2 + 32 + 16384 + 2",
          len(frame) == 2 + HEADER_SIZE + GRID * GRID + 2, f"{len(frame)}")
    body, crc_sent = frame[2:-2], struct.unpack("<H", frame[-2:])[0]
    check("CRC16 verifies", crc16_ccitt(body) == crc_sent,
          f"0x{crc_sent:04X}")
    corrupted = bytearray(body)
    corrupted[1000] ^= 0xFF
    check("CRC16 actually catches a flipped byte",
          crc16_ccitt(bytes(corrupted)) != crc_sent)

    # --- spawn sanity -------------------------------------------------------
    sx, sy, sz, yaw = spawn
    check("spawn is on land", blocks[sy, sx] != BLOCK_WATER,
          BLOCK_NAMES[int(blocks[sy, sx])])
    check("spawn height matches the grid", sz == int(heights[sy, sx]))
    check("spawn is away from the border",
          4 <= sx < GRID - 4 and 4 <= sy < GRID - 4, f"({sx},{sy})")
    neigh = [int(heights[max(0, sy-1), sx]), int(heights[min(GRID-1, sy+1), sx]),
             int(heights[sy, max(0, sx-1)]), int(heights[sy, min(GRID-1, sx+1)])]
    check("spawn is not on a cliff edge",
          max(abs(n - sz) for n in neigh) <= 3, f"neighbours {neigh}")
    check("spawn yaw fits in a byte", 0 <= yaw <= 255)

    # --- water --------------------------------------------------------------
    water = blocks == BLOCK_WATER
    if water.any():
        check("every water cell sits at one flat level",
              len(np.unique(heights[water])) == 1,
              f"level {int(heights[water][0])}")

    # --- flood --------------------------------------------------------------
    fh, fb, fstats, fmask = apply_flood(heights, blocks, water_level + 4)
    check("flooding only ever adds water",
          int((fb == BLOCK_WATER).sum()) >= int(water.sum()),
          f"{int(water.sum())} -> {int((fb == BLOCK_WATER).sum())} cells")
    check("flood reports a sane land percentage",
          0 <= fstats.get("land_pct_flooded", -1) <= 100,
          f"{fstats.get('land_pct_flooded')}% of land")
    check("nothing is left sitting below the flood line",
          bool((fh >= min(water_level + 4, int(heights.max()))).all()),
          f"lowest cell is now level {int(fh.min())}")
    check("flooding actually submerged some land",
          fstats.get("land_cells_flooded", 0) > 0,
          f"{fstats.get('land_cells_flooded')} cells")
    check("the flood mask marks only former land",
          fmask is not None and not (fmask & (blocks == BLOCK_WATER)).any())

    # --- fires --------------------------------------------------------------
    bbox = bbox_from_center(loc.lat, loc.lon, 30.0)
    fake_fires = [(loc.lat, loc.lon), (loc.lat + 0.004, loc.lon + 0.004)]
    nb, cells = apply_fires(blocks, fake_fires, bbox)
    check("fire hotspots land inside the grid",
          all(0 <= x < GRID and 0 <= y < GRID for x, y in cells),
          f"{len(cells)} cells")
    check("fire scorching never sets a lake on fire",
          not ((blocks == BLOCK_WATER) & (nb != BLOCK_WATER)).any())

    # --- block type coverage ------------------------------------------------
    present = {BLOCK_NAMES[b] for b in range(8) if (blocks == b).any()}
    check("the fallback world is not one flat colour", len(present) >= 4,
          ", ".join(sorted(present)))

    # --- exports ------------------------------------------------------------
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        global VERBOSE
        was = VERBOSE
        VERBOSE = False
        write_mif(world, os.path.join(tmp, "w.mif"))
        write_hex(world, os.path.join(tmp, "w.hex"))
        write_json(world, os.path.join(tmp, "w.json"))
        VERBOSE = was
        with open(os.path.join(tmp, "w.hex")) as f:
            hex_lines = [l.strip() for l in f if l.strip()]
        check("hex file has one line per cell", len(hex_lines) == GRID * GRID,
              f"{len(hex_lines)}")
        check("hex bytes match the payload",
              all(int(hex_lines[i], 16) == payload[i] for i in range(0, 16384, 97)))
        with open(os.path.join(tmp, "w.mif")) as f:
            mif = f.read()
        check("mif declares the right depth and width",
              "DEPTH = 16384;" in mif and "WIDTH = 8;" in mif)
        with open(os.path.join(tmp, "w.json")) as f:
            meta = json.load(f)
        check("json round trips", meta["grid"]["size"] == 128 and
              meta["spawn"]["x"] == sx)

    # --- voxel-game terrain (terrain.bin + UART blob) -----------------------
    import zlib
    cols = build_terrain_columns(world, 2)
    tb = pack_terrain_bin(cols)
    n = cols["size"]
    check("terrain.bin has the right size", len(tb) == 4096 + n * n * 8, f"{len(tb)} bytes")
    check("terrain header magic and size",
          tb[:8] == TERRAIN_MAGIC and struct.unpack_from("<II", tb, 16) == (n, n))
    rec = np.frombuffer(tb, dtype=TERRAIN_RECORD, offset=4096)
    # column (x=17, z=5) lives in tile 1 of tile row 0, row 5, column 1
    k = (0 * (n // 16) + 1) * 256 + 5 * 16 + 1
    check("tile layout puts (17, 5) where the game reads it",
          int(rec["height"][k]) == int(cols["height"][5, 17]))
    check("heights within the game's limits",
          int(cols["height"].min()) >= MC_MIN_GROUND and int(cols["height"].max()) <= MC_MAX_GROUND)
    land = cols["water"] <= cols["height"]
    check("spawn column (0, 0) is dry land", bool(land[0, 0]))
    check("snowing weather -> snow everywhere it is falling",
          bool((cols["weather"] == WX_SNOW).all()))
    blob = pack_terrain_transfer(cols, tb)
    check("TRZ1 blob unpacks to the identical terrain.bin",
          unpack_terrain_transfer(blob) == tb, f"{len(blob)} of {len(tb)} bytes")
    frame = pack_board_frame(FRAME_WORLD, 7, blob)
    check("board frame header CRC and payload CRC",
          zlib.crc32(frame[:16]) & 0xFFFFFFFF == struct.unpack_from("<I", frame, 16)[0] and
          zlib.crc32(frame[20:]) & 0xFFFFFFFF == struct.unpack_from("<I", frame, 12)[0])
    for wx, expect in ((Weather(is_day=1, cloud_cover_pct=10), WX_SUNNY),
                       (Weather(is_day=1, cloud_cover_pct=90), WX_CLOUDY),
                       (Weather(is_day=0, cloud_cover_pct=10), WX_NIGHT),
                       (Weather(is_day=1, rain_mm_h=2.0), WX_RAIN),
                       (Weather(is_day=0, rain_mm_h=2.0), WX_RAIN)):
        got = int(classify_weather(wx, np.array([10.0]))[0])
        check(f"weather -> {WX_NAMES[expect]}", got == expect, WX_NAMES[got])
    cold = classify_weather(Weather(is_day=1, rain_mm_h=2.0), np.array([8.0, -3.0]))
    check("rain falls as snow on cold high ground", list(cold) == [WX_RAIN, WX_SNOW])

    print("=" * 70)
    if failures:
        print(f"  {len(failures)} CHECK(S) FAILED: {', '.join(failures)}")
        print("=" * 70 + "\n")
        return 1
    print("  ALL CHECKS PASSED - the byte format is safe to send to the board")
    print("=" * 70 + "\n")
    return 0


# ==============================================================================
# SECTION 20 -- COMMAND LINE
# ==============================================================================

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="worldgen.py",
        description="Turn any place on Earth into a 128x128 block world for an FPGA.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  python worldgen.py "Mount Baker"
  python worldgen.py "Vancouver" --scale 120 --exaggeration 1.5
  python worldgen.py --lat 49.28 --lon -123.12 --scale 60
  python worldgen.py "Richmond BC" --flood-meters 6
  python worldgen.py "Kelowna" --fires --firms-key KEY --sentinel
  python worldgen.py "Squamish" --uart /dev/ttyUSB0          # send to the board
  python worldgen.py --serve --uart /dev/ttyUSB0              # UI picks -> board
  python worldgen.py --uart /dev/ttyUSB0 --board-ping
  python worldgen.py --bake-demos
  python worldgen.py --serve --port 8080
""")

    p.add_argument("place", nargs="?", help="place name, e.g. \"Mount Baker\"")
    p.add_argument("--lat", type=float, help="latitude (skips geocoding)")
    p.add_argument("--lon", type=float, help="longitude (skips geocoding)")

    g = p.add_argument_group("world shape")
    g.add_argument("--scale", type=float, default=30.0,
                   help="metres per block (default 30, so 3.84 km across)")
    g.add_argument("--exaggeration", type=float, default=1.0,
                   help="vertical exaggeration; 1.3-2.0 looks better in first "
                        "person (default 1.0)")
    g.add_argument("--smooth", type=int, default=1,
                   help="blur passes over the heightmap (default 1, try 2-3 for "
                        "noisy cities)")

    g = p.add_argument_group("scenarios")
    g.add_argument("--flood", type=int, default=0, metavar="LEVEL",
                   help="flood everything at or below this block level (0-31)")
    g.add_argument("--flood-meters", type=float, default=None, metavar="M",
                   help="flood by this many REAL metres above the water line")
    g.add_argument("--fires", action="store_true",
                   help="fetch NASA FIRMS active fire hotspots")
    g.add_argument("--firms-key", default=os.environ.get("FIRMS_MAP_KEY", ""),
                   help="FIRMS map key (or set FIRMS_MAP_KEY)")
    g.add_argument("--sentinel", action="store_true",
                   help="add the Sentinel-2 NDVI/NDWI/NDSI refinement layer")

    g = p.add_argument_group("output")
    g.add_argument("--out", default="out", help="output directory (default out/)")
    g.add_argument("--cache-dir", default="cache", help="cache directory")
    g.add_argument("--png-scale", type=int, default=5,
                   help="pixels per block in preview.png (default 5 -> 640px)")
    g.add_argument("--no-color", dest="color", action="store_false",
                   help="plain ASCII preview instead of colour blocks")
    g.add_argument("--preview-step", type=int, default=2,
                   help="1 = full 128 wide preview, 2 = 64 wide (default 2)")
    g.add_argument("--quiet", action="store_true", help="less chatter")

    g = p.add_argument_group("hardware (the DE1-SoC voxel game, via terrad)")
    g.add_argument("--uart", metavar="PORT",
                   help="board's serial port, e.g. /dev/ttyUSB0 or COM3: send the "
                        "world to the game (with --serve: every generated world)")
    g.add_argument("--baud", type=int, default=115200, help="baud rate (default 115200)")
    g.add_argument("--blocks-per-cell", type=int, default=2,
                   help="game columns per 128-grid cell along each side: 1 = 128x128, "
                        "2 = 256x256 (default), 4 = 512x512 (slower to send)")
    g.add_argument("--board-ping", action="store_true",
                   help="check that terrad answers on --uart, then exit")
    g.add_argument("--board-console", action="store_true",
                   help="tell terrad to give the UART back to a login shell, then exit")

    g = p.add_argument_group("ai extras (optional tracks)")
    g.add_argument("--gemini-key", default=os.environ.get("GEMINI_API_KEY", ""),
                   help="Gemini API key (or set GEMINI_API_KEY) to write narration")
    g.add_argument("--gemini-model", default="gemini-2.5-flash")
    g.add_argument("--eleven-key", default=os.environ.get("ELEVENLABS_API_KEY", ""),
                   help="ElevenLabs key (or set ELEVENLABS_API_KEY) to voice it")
    g.add_argument("--voice", default="21m00Tcm4TlvDq8ikWAM", help="ElevenLabs voice id")

    g = p.add_argument_group("modes")
    g.add_argument("--serve", action="store_true", help="run as an HTTP service")
    g.add_argument("--host", default="127.0.0.1")
    g.add_argument("--port", type=int, default=8080)
    g.add_argument("--bake-demos", action="store_true",
                   help="pre-fetch every demo location into the cache")
    g.add_argument("--offline", action="store_true",
                   help="use only cached data, never touch the network")
    g.add_argument("--self-test", action="store_true",
                   help="verify the byte format and spawn logic, no network needed")

    return p


def main(argv: Optional[List[str]] = None) -> int:
    global VERBOSE
    args = build_parser().parse_args(argv)
    VERBOSE = not args.quiet

    if args.self_test:
        return self_test()

    if args.board_ping or args.board_console:
        if not args.uart:
            log("--board-ping / --board-console need --uart PORT", "err")
            return 2
        if args.board_ping:
            ok, msg = board_send(args.uart, args.baud, FRAME_PING, expect="PONG")
        else:
            ok, msg = board_send(args.uart, args.baud, FRAME_CONSOLE, expect="CONSOLE")
        log(msg, "ok" if ok else "err")
        return 0 if ok else 1

    if args.bake_demos:
        bake_demos(args)
        return 0

    if args.serve:
        serve(args.host, args.port, args.out, args.cache_dir, args)
        return 0

    # Work out where we are generating.
    if args.lat is not None and args.lon is not None:
        loc = Location(name=args.place or f"{args.lat:.3f}, {args.lon:.3f}",
                       lat=args.lat, lon=args.lon)
        log(f"using given coordinates {loc.lat:.4f}, {loc.lon:.4f}", "ok")
    elif args.place:
        loc = geocode(args.place)
        if loc is None:
            if args.offline:
                log("cannot geocode while offline; pass --lat and --lon", "err")
                return 2
            log("could not find that place; try adding a country, "
                "or pass --lat and --lon", "err")
            return 2
    else:
        build_parser().print_help()
        return 1

    world = build_world(
        loc, args.scale,
        exaggeration=args.exaggeration, smooth=args.smooth,
        flood_level=args.flood, flood_meters=args.flood_meters,
        use_sentinel=args.sentinel, firms_key=args.firms_key,
        use_fires=args.fires, offline=args.offline, cache_dir=args.cache_dir)

    print(ascii_preview(world, color=args.color, step=args.preview_step))

    emit_all(world, args.out, png_scale=args.png_scale,
             gemini_key=args.gemini_key, eleven_key=args.eleven_key,
             gemini_model=args.gemini_model, voice_id=args.voice,
             blocks_per_cell=args.blocks_per_cell)

    if args.uart and not send_terrain_blob(world.terrain_blob, args.uart, args.baud, loc.label):
        return 1

    log(f"done. files are in {os.path.abspath(args.out)}", "ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
