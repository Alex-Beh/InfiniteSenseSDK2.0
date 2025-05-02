#!/bin/bash

SERIAL_PORT="/dev/ttyACM0"
BAUD_RATE="921600"
CFG_FILE="cfg.json"

if [ ! -f "$CFG_FILE" ]; then
    echo "Error: $CFG_FILE not found!"
    exit 1
fi

echo "Sending cfg to $SERIAL_PORT at $BAUD_RATE baud..."

# Configure the serial port
stty -F "$SERIAL_PORT" raw speed "$BAUD_RATE" cs8 -cstopb -parenb

# Send the content + newline
cat "$CFG_FILE" | sed 's/$/\n/' > "$SERIAL_PORT"

echo "Done."
