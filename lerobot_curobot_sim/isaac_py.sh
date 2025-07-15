#!/bin/bash

# Set possible Isaac Sim paths
ISAAC_SIM_PATHS=("$HOME/isaacsim" "/isaac-sim")

# Get absolute path of the current script directory
CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Handle arguments properly - first argument should be the script, rest are arguments
if [ $# -eq 0 ]; then
    echo "Usage: $0 <script> [arguments...]"
    exit 1
fi

# First argument is the script to run
SCRIPT_NAME="$1"
shift  # Remove the first argument

# Prepend the script directory only to the script name
SCRIPT_PATH="${CUR_SCRIPT_DIR}/${SCRIPT_NAME}"

# Check if the script exists
if [ ! -f "$SCRIPT_PATH" ]; then
    echo "Error: Script not found: $SCRIPT_PATH"
    exit 1
fi

# Build the argument list with the script path and remaining arguments
NEW_ARGS="${SCRIPT_PATH}"
for arg in "$@"; do
    NEW_ARGS="${NEW_ARGS} ${arg}"
done

# Look for a valid Isaac Sim installation
for ISAAC_SIM_PATH in "${ISAAC_SIM_PATHS[@]}"; do
    if [[ -f "$ISAAC_SIM_PATH/python.sh" ]]; then
        "$ISAAC_SIM_PATH/python.sh" $NEW_ARGS
        exit 0
    fi
done

echo "No valid Isaac Sim installation found under home or root directory."
exit 1

