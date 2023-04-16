#!/bin/bash

# Parse command line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --build) build="true";;
        *) echo "Unknown parameter passed: $1"; exit 1;;
    esac
    shift
done

# Run docker-compose command with or without --build flag
if [[ $build == "true" ]]; then
    docker compose up --build -d roscore dev-runner rviz bridge
else
    docker compose up -d roscore dev-runner rviz bridge
fi
