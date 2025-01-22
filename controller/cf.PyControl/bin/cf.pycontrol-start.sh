#!/bin/bash

# Default arguments for the Python application
URI="radio://0/80/2M/E7E7E7E7E1"
PORT="5000"

cd ../src/
# Run the Python application with default arguments and any extra arguments passed to the script
python3 cf-ctrl-service.py --uri "$URI" --port "$PORT" "$@"