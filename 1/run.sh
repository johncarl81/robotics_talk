#!/bin/bash
docker run \
  --mount type=bind,source="$(pwd)"/workspace,target=/workspace \
  -it talk_one:latest
