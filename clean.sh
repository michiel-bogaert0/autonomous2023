#!/bin/bash

PREFIX=$1
shift
if [ -z "$PREFIX" ]
  then
    echo "Please provide a prefix."
    exit 1
fi

docker ps -q --filter name=$PREFIX* | xargs docker rm -f