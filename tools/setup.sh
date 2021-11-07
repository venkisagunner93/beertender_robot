#!/bin/sh

TOOLS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

function beertender () {
    python3 $TOOLS_DIR/beertender_main.py $@
}