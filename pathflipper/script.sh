#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

FIELD_HEIGHT="8.0772"

dir="${SCRIPT_DIR}/src/main/deploy/choreo"
input="PPOutpostTrench"
rename="PPDepotTrench"

jq -f "${SCRIPT_DIR}/pathflipper.jq" \
  --argjson FIELD_HEIGHT "$FIELD_HEIGHT" \
  --arg NAME "$rename" \
  "${dir}/${input}.traj" > "${dir}/${rename}.traj"
