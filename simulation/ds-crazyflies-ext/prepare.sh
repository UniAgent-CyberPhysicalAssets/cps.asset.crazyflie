#!/bin/bash

echo "Current working directory: $(pwd)"
# Define the source and destination directories
SRC_DIR="./docker/"
DEST_DIR="./simulation.ds-crazyflies"

echo "Expected source directory: $SRC_DIR"
# Check if the source directory exists
if [ ! -d "$SRC_DIR" ]; then
  echo "Source directory $SRC_DIR does not exist. Exiting."
  exit 1
fi

# Create the destination directory if it does not exist
mkdir -p "$DEST_DIR"

git clone --recurse https://github.com/DynamicSwarms/ds-crazyflies.git simulation.ds-crazyflies

# Copy the source directory to the destination
cp -r "$SRC_DIR" "$DEST_DIR"
cp -r crazyflie.wbt "$DEST_DIR"

# Notify the user of the successful copy
echo "Copied $SRC_DIR to $DEST_DIR successfully."