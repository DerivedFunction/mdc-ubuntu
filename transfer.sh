#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <send/receive> <source_path> <destination_path>"
    echo "capstone@mdc-nx.local:/home/capstone/"
    exit 1
fi

# Assign arguments to variables
ACTION=$1
SOURCE_PATH=$2
DESTINATION_PATH=$3

# Perform action based on the first argument
if [ "$ACTION" == "send" ]; then
    scp "$SOURCE_PATH" capstone@mdc-nx.local:/home/capstone/"$DESTINATION_PATH"
elif [ "$ACTION" == "receive" ]; then
    scp capstone@mdc-nx.local:/home/capstone/"$SOURCE_PATH" "$DESTINATION_PATH"
else
    echo "Invalid action. Use 'send' or 'receive'."
    exit 1
fi