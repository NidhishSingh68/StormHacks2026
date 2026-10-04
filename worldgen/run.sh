#!/bin/sh
# run.sh - start the world server for the UI and the DE1-SoC game
#
#   ./run.sh                 find the board's serial port, start the server on
#                            port 8080 (where the UI looks) and open the UI
#   ./run.sh /dev/ttyUSB1    use this serial port
#   ./run.sh none            no board: UI only
#
# Every place picked in the UI is turned into a world and sent to the board,
# where terrad restarts the game on it (install terrad on the board once,
# see ../de1soc/README.md).
cd "$(dirname "$0")" || exit 1

PY=./venv/bin/python
if [ ! -x "$PY" ]; then
    echo "setting up venv (first run only)..."
    python3 -m venv --system-site-packages venv &&
        ./venv/bin/pip install -q numpy requests pillow pyserial rasterio || exit 1
fi

PORT="$1"
if [ -z "$PORT" ]; then
    # the board's USB-UART is an FTDI FT232R
    for dev in /dev/serial/by-id/*FTDI* /dev/ttyUSB*; do
        [ -e "$dev" ] && { PORT="$dev"; break; }
    done
fi

UART_ARGS=""
if [ -n "$PORT" ] && [ "$PORT" != "none" ]; then
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

# open the UI once the server is up
( sleep 2
  URL="file://$(pwd)/indexnew.html"
  if command -v xdg-open >/dev/null; then xdg-open "$URL" >/dev/null 2>&1
  elif command -v open >/dev/null; then open "$URL"
  else echo "open $URL in a browser"; fi ) &

echo "server on http://127.0.0.1:8080 (Ctrl-C to stop)"
exec "$PY" worldgen.py --serve --port 8080 $UART_ARGS
