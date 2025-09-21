#!/bin/bash

# Script to create and deploy solution with base64 encoded stack data
# Usage: ./define-solution.sh <json-file>

# Check if JSON file argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <json-file>"
    echo "Example: $0 talker-listener-stack.json"
    exit 1
fi

JSON_FILE="$1"

# Check if file exists
if [ ! -f "$JSON_FILE" ]; then
    echo "Error: File '$JSON_FILE' not found!"
    exit 1
fi

# Extract solution name from filename (remove path and .json extension)
ROOT_NAME=$(basename "$JSON_FILE" .json)
SOLUTION_NAME="${ROOT_NAME}-v-1"

export SYMPHONY_API_URL=http://192.168.0.47:8082/v1alpha2/

# Get authentication token
TOKEN=$(curl -X POST -H "Content-Type: application/json" -d '{"username":"admin","password":""}' "${SYMPHONY_API_URL}users/auth" | jq -r '.accessToken')

# Base64 encode the contents of the JSON file
STACK_DATA_BASE64=$(base64 -w 0 "$JSON_FILE")

# Create the solution JSON in a variable
SOLUTION_DATA=$(cat << EOF
{
    "metadata": {
        "namespace": "default",
        "name": "$SOLUTION_NAME"
    },
    "spec": {
        "displayName": "$SOLUTION_NAME",
        "rootResource": "$ROOT_NAME",
        "version": "1",
        "components": [
            {
                "name": "$SOLUTION_NAME",
                "type": "muto-agent",
                "properties": {
                    "type": "stack", 
                    "content-type": "application/json", 
                    "data": "$STACK_DATA_BASE64",
                    "foo": "bar",
                    "number": 123
                }
            }
        ]
    }
}
EOF
)

echo "Created solution JSON with base64 encoded stack data"
echo "Base64 encoded data length: ${#STACK_DATA_BASE64} characters"

# Show current solutions
# echo "Current solutions:"
# curl -s -X GET -H "Content-Type: application/json" -H "Authorization: Bearer $TOKEN" "${SYMPHONY_API_URL}solutions" | jq .

echo "Posting $SOLUTION_NAME solution to Symphony..."

# Try to delete existing solution first (ignore errors if it doesn't exist)
# curl -s -X DELETE -H "Authorization: Bearer $TOKEN" "${SYMPHONY_API_URL}solutions/$SOLUTION_NAME" > /dev/null 2>&1

# Post the new solution
curl -s -v -X POST -H "Content-Type: application/json" -H "Authorization: Bearer $TOKEN" -d "$SOLUTION_DATA" "${SYMPHONY_API_URL}solutions/$SOLUTION_NAME"
