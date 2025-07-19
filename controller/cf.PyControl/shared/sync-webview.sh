#!/bin/bash

# Default values
VERBOSE=0
SRC_PATH=/home/user/dev_ws/libs/crazyflie-controller/src/webview  # Inside container
DEST_PATH=./  # On host

# Function to print usage
usage() {
  echo "Usage: $0 <container_id_or_name> [--verbose]"
  echo ""
  echo "Arguments:"
  echo "  <container_id_or_name>   Name or ID of the Docker container"
  echo "  --verbose                Enable verbose output (shows docker cp output)"
  exit 1
}

# Argument parsing
if [ $# -lt 1 ]; then
  usage
fi

CONTAINER_NAME=""
for arg in "$@"; do
  case $arg in
    --verbose)
      VERBOSE=1
      ;;
    -*)
      echo "Unknown option: $arg"
      usage
      ;;
    *)
      CONTAINER_NAME="$arg"
      ;;
  esac
done

if [ -z "$CONTAINER_NAME" ]; then
  usage
fi

DEST_PATH=$DEST_PATH/$CONTAINER_NAME
mkdir -p "$DEST_PATH"
sudo chown -R $USER:$USER $DEST_PATH

echo "Syncing from container: $CONTAINER_NAME"
echo "Source (in container): $SRC_PATH"
echo "Destination (on host): $DEST_PATH"
echo "Verbose mode: $([ $VERBOSE -eq 1 ] && echo enabled || echo disabled)"

# Sync loop
while true; do
  if [ $VERBOSE -eq 1 ]; then
    sudo docker cp "$CONTAINER_NAME":"$SRC_PATH" "$DEST_PATH"
  else
    sudo docker cp "$CONTAINER_NAME":"$SRC_PATH" "$DEST_PATH" 2>/dev/null
  fi
  sudo chown -R $USER:$USER $DEST_PATH
  sleep 2
done
