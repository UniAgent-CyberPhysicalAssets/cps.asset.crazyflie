#!/bin/bash

echo "Current working directory: $(pwd)"

# Define directories
SRC_DIR="./docker/"
DEST_DIR="./simulation.ds-crazyflies"

COPY_DOCKER=false

# Parse arguments
for arg in "$@"; do
  case $arg in
    --linux)
      COPY_DOCKER=true
      shift
      ;;
    *)
      ;;
  esac
done

# Clone repository if destination does not exist
if [ ! -d "$DEST_DIR/.git" ]; then
  git clone --recurse https://github.com/DynamicSwarms/ds-crazyflies.git "$DEST_DIR"
  cd "$DEST_DIR"
  git checkout 9728f025382f1e8bb4a9b670c4d53999f3801224
  git submodule update --init --recursive
  cd ..
else
  echo "Repository already exists at $DEST_DIR, skipping clone."
fi

if [ "$COPY_DOCKER" = true ]; then
  echo "Linux mode enabled: copying Docker files."

  if [ ! -d "$SRC_DIR" ]; then
    echo "Source directory $SRC_DIR does not exist. Exiting."
    exit 1
  fi

  cp -r "$SRC_DIR" "$DEST_DIR"
else
  echo "Linux mode disabled: skipping Docker files copy."
fi

# Copy Webots world
cp -r crazyflie.wbt "$DEST_DIR"

echo "Setup completed successfully."
