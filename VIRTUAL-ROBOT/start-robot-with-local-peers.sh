#!/usr/bin/env bash

if [[ -e "./start-docker.sh" ]]; then
  ./start-docker.sh --with-local-peers
fi
