#!/usr/bin/env bash
set -eou pipefail

SCRIPT_DIR=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
CHOREO_DIR=$(realpath "${SCRIPT_DIR}/../src/main/deploy/choreo")

FIELD_HEIGHT="8.0772"

flip_path() {
    local input="$1"
    local rename="$2"

    local from="${CHOREO_DIR}/${input}.traj"
    local to="${CHOREO_DIR}/${rename}.traj"

    printf "Flipping %s -> %s\n" "$input" "$rename"
    test -f "$to" || touch "$to"

    test -f "$from" || {
        printf "Can't find path %s" "$from"
        exit 1
    }
    
    jq -f "${SCRIPT_DIR}/pathflipper.jq" \
      --argjson FIELD_HEIGHT "$FIELD_HEIGHT" \
      --arg NAME "$rename" \
      "$from" > "$to"
}
