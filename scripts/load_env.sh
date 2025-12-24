#!/bin/bash
# Automatically load environment variables from .env file

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ENV_FILE="$PROJECT_DIR/.env"

if [ -f "$ENV_FILE" ]; then
    while IFS= read -r line || [[ -n "$line" ]]; do
        # Trim leading and trailing whitespace
        trimmed=$(echo "$line" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
        
        # Skip empty lines and lines starting with #
        if [[ -n "$trimmed" && ! "$trimmed" =~ ^# ]]; then
            # Remove inline comments (part after #) and export
            clean_line=$(echo "$trimmed" | cut -d'#' -f1 | sed 's/[[:space:]]*$//')
            if [[ -n "$clean_line" ]]; then
                export "$clean_line"
            fi
        fi
    done < "$ENV_FILE"
    echo "Environment variables loaded successfully!"
else
    echo "Warning: .env file not found at $ENV_FILE"
    echo "Please create it from .env.example"
fi