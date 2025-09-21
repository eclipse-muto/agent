#!/bin/bash

export SYMPHONY_API_URL=http://192.168.0.47:8082/v1alpha2/

TOKEN=$(curl -X POST -H "Content-Type: application/json" -d '{"username":"admin","password":""}' "${SYMPHONY_API_URL}users/auth" | jq -r '.accessToken')


# Prompt user to press Enter to continue after the target has been registered

curl -v -s -X GET  -H "Content-Type: application/json"  -H "Authorization: Bearer $TOKEN"  "${SYMPHONY_API_URL}instances"

# Read the content of solution.json file and send it as data in the POST request
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOLUTION_DATA=$(cat "$SCRIPT_DIR/instance.json")

curl -v -s -X POST  -H "Content-Type: application/json"  -H "Authorization: Bearer $TOKEN" -d "$SOLUTION_DATA" "${SYMPHONY_API_URL}instances/test-robot-debug-instance"

