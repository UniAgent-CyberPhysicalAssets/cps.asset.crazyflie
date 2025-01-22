#!/bin/bash
set -e

# not really necessary
echo "export DISPLAY=unix:1" >> /etc/bash.bashrc

exec "$@"