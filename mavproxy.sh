#!/bin/bash
# MAVProxy launcher for the eventide module runtime.
#
# pip installs MAVProxy's console script into the module venv, but the exact
# location depends on the Python build's install scheme:
#   <venv>/local/bin/mavproxy.py   (Debian "local" scheme)
#   <venv>/bin/mavproxy.py         (standard scheme)
# Run it with the venv's python so pymavlink & co. resolve inside the venv.
#
# Usage: mavproxy.sh <venv_dir> [mavproxy args...]

set -u
VENV_DIR="$1"; shift

for cand in "$VENV_DIR/local/bin/mavproxy.py" "$VENV_DIR/bin/mavproxy.py"; do
    if [ -f "$cand" ]; then
        exec "$VENV_DIR/bin/python3" "$cand" "$@"
    fi
done

echo "mavproxy.py not found under $VENV_DIR (looked in local/bin and bin)" >&2
exit 1
