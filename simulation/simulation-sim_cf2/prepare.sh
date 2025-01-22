#!/bin/bash

echo "Current working directory: $(pwd)"
# Define the source and destination directories
SRC_DIR="../../controller/cf.PyControl"
DEST_DIR="."

echo "Expected source directory: $SRC_DIR"
# Check if the source directory exists
if [ ! -d "$SRC_DIR" ]; then
  echo "Source directory $SRC_DIR does not exist. Exiting."
  exit 1
fi

# Create the destination directory if it does not exist
mkdir -p "$DEST_DIR"

# Copy the source directory to the destination
cp -r "$SRC_DIR" "$DEST_DIR"

# Notify the user of the successful copy
echo "Copied $SRC_DIR to $DEST_DIR successfully."