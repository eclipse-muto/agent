#!/bin/bash

export SYMPHONY_API_URL=http://192.168.0.47:8082/v1alpha2/

TOKEN=$(curl -X POST -H "Content-Type: application/json" -d '{"username":"admin","password":""}' "${SYMPHONY_API_URL}users/auth" | jq -r '.accessToken')
# Prompt user to press Enter to continue after the target has been registered

curl -v -s -X DELETE -H "Authorization: Bearer $TOKEN" "${SYMPHONY_API_URL}targets/registry/test-robot-debug"
