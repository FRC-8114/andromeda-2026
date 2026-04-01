#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

FIELD_HEIGHT="8.0772"

input="src/main/deploy/choreo/Trench2xOutpost.traj"
rename="Trench2xDepot"

jq -f "${SCRIPT_DIR}/pathflipper.jq" \
  --argjson FIELD_HEIGHT "$FIELD_HEIGHT" \
  --arg NAME "$rename" \
  "$input" > "$(dirname "$input")/${rename}.traj"
