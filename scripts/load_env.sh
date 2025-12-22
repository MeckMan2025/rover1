#!/bin/bash
# Automatically load environment variables from .env file

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ENV_FILE="$PROJECT_DIR/.env"

if [ -f "$ENV_FILE" ]; then
    echo "Loading environment variables from .env..."
    export $(grep -v '^#' "$ENV_FILE" | grep -v '^$' | xargs)
    echo "Environment variables loaded successfully!"
else
    echo "Warning: .env file not found at $ENV_FILE"
    echo "Please create it from .env.example"
fi