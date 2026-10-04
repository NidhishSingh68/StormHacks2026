#!/bin/sh
# run.sh - start the world server for the UI and the DE1-SoC game
#
#   ./run.sh                 find the board's serial port, start the server on
#                            port 8080 and open the UI (http://localhost:8080/ui)
#   ./run.sh /dev/ttyUSB1    use this serial port
#   ./run.sh COM5            Windows port name (WSL 1 only; WSL 2: see below)
#   ./run.sh none            no board: UI only
#
# Every place picked in the UI is turned into a world and sent to the board,
# where terrad restarts the game on it (install terrad on the board once,
# see ../de1soc/README.md).
#
# Linux and WSL (Windows Subsystem for Linux) both work. Under WSL 2 the
# board's USB serial adapter must be passed through from Windows with
# usbipd-win (https://github.com/dorssel/usbipd-win): install it once, and
# once, in an administrator PowerShell, share the adapter:
#     usbipd list                     (the board: 0403:6001 USB Serial Converter)
#     usbipd bind --busid <BUSID>
# This script then attaches it to WSL by itself every time it runs. The UI
# opens in the Windows browser.
cd "$(dirname "$0")" || exit 1

is_wsl=0
grep -qi microsoft /proc/version 2>/dev/null && is_wsl=1

# ---- Python environment ----------------------------------------------------

PY=./venv/bin/python
if [ ! -x "$PY" ]; then
    echo "setting up venv (first run only)..."
    if ! python3 -m venv --system-site-packages venv; then
        rm -rf venv
        echo "python3 -m venv failed; install it first:"
        echo "  sudo apt install python3-venv python3-pip"
        exit 1
    fi
    ./venv/bin/pip install -q numpy requests pillow pyserial rasterio || exit 1
fi

# ---- the board's serial port -----------------------------------------------

# WSL 2: attach the board's FTDI adapter (USB 0403:6001) from Windows
wsl_attach_board() {
    command -v usbipd.exe >/dev/null 2>&1 || {
        echo "WSL: the board's USB serial adapter is not visible here. Install"
        echo "  usbipd-win on Windows (winget install usbipd), then see run.sh."
        return 1
    }
    line=$(usbipd.exe list 2>/dev/null | tr -d '\r' | grep -i "0403:6001" | head -n 1)
    [ -n "$line" ] || {
        echo "WSL: no FTDI adapter (0403:6001) on Windows' USB: is the board's"
        echo "  UART cable plugged in?"
        return 1
    }
    busid=$(echo "$line" | awk '{print $1}')
    case "$line" in
        *Attached*)
            ;;
        *"Not shared"*)
            echo "WSL: share the board's adapter once, in an administrator PowerShell:"
            echo "  usbipd bind --busid $busid"
            echo "  and run this again."
            return 1 ;;
        *)
            echo "WSL: attaching the board's USB serial adapter ($busid) ..."
            usbipd.exe attach --wsl --busid "$busid" >/dev/null 2>&1 || {
                echo "  usbipd attach failed; try in PowerShell: usbipd attach --wsl --busid $busid"
                return 1
            } ;;
    esac
    # the device node appears a moment later
    i=0
    while [ $i -lt 20 ] && ! ls /dev/ttyUSB* >/dev/null 2>&1; do
        sleep 0.5
        i=$((i + 1))
    done
    ls /dev/ttyUSB* >/dev/null 2>&1 || {
        echo "  attached, but no /dev/ttyUSB* appeared in WSL"
        return 1
    }
}

find_port() {
    # the board's USB-UART is an FTDI FT232R
    for dev in /dev/serial/by-id/*FTDI* /dev/ttyUSB*; do
        [ -e "$dev" ] && { echo "$dev"; return 0; }
    done
    return 1
}

PORT="$1"
case "$PORT" in
    COM[0-9]*|com[0-9]*)
        # WSL 1 maps COMn to /dev/ttySn; WSL 2 has no Windows COM ports
        n=${PORT#???}
        PORT="/dev/ttyS$n"
        [ -e "$PORT" ] || { echo "$1 is not visible from here (WSL 2?): run without"
                            echo "an argument to attach the board with usbipd"; exit 1; } ;;
esac
if [ -z "$PORT" ]; then
    PORT=$(find_port)
    if [ -z "$PORT" ] && [ $is_wsl -eq 1 ]; then
        wsl_attach_board && PORT=$(find_port)
    fi
fi

UART_ARGS=""
if [ -n "$PORT" ] && [ "$PORT" != "none" ]; then
    if [ ! -w "$PORT" ]; then
        echo "no permission to use $PORT; either run once:"
        echo "  sudo usermod -aG dialout $USER     (then log in again / wsl --shutdown)"
        echo "or for this session only:"
        echo "  sudo chmod a+rw $PORT"
        exit 1
    fi
    if command -v fuser >/dev/null && fuser "$PORT" >/dev/null 2>&1; then
        echo "$PORT is in use (minicom?). Close it and run again."
        exit 1
    fi
    echo "board on $PORT:"
    if "$PY" worldgen.py --uart "$PORT" --board-ping --quiet; then
        echo "  terrad answered, worlds will be sent to the game"
    else
        echo "  terrad did not answer (board off, still booting, or terrad not"
        echo "  installed). Starting anyway; worlds are sent once it answers."
    fi
    UART_ARGS="--uart $PORT"
else
    echo "no board serial port found: UI only (plug the board in and run again)"
fi

# ---- the UI ----------------------------------------------------------------

# served by worldgen.py itself, so it works from a Windows browser too
URL="http://localhost:8080/ui"
( sleep 2
  if [ $is_wsl -eq 1 ]; then
      if command -v wslview >/dev/null; then wslview "$URL"
      else cmd.exe /c start "" "$URL" >/dev/null 2>&1 || echo "open $URL in a browser"; fi
  elif command -v xdg-open >/dev/null; then xdg-open "$URL" >/dev/null 2>&1
  elif command -v open >/dev/null; then open "$URL"
  else echo "open $URL in a browser"; fi ) &

echo "server on http://localhost:8080 (UI: $URL, Ctrl-C to stop)"
exec "$PY" worldgen.py --serve --port 8080 $UART_ARGS
