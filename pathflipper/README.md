# `pathflipper`
While Choreo natively supports flipping paths from red to blue side (flipping along the y-axis), it has no built-in tooling (at least that I'm aware of) for flipping paths along the x-axis.
This [jq](https://jqlang.org) script implements this feature, allowing us to flip paths along the x-axis on demand.

While this tool is mainly build for FRC team 8114, it could be used by any team with minimal modification.

## Setup
- Copy this directory to your own codebase
- Install `jq` with the system package manager (ex. `sudo apt install jq` or `nix-shell -p jq`)
- update `$input` and `$rename` variables to match the file path to the Choreo path (.traj file) that will be flipped and the name of the new path, respectively

## Running
- Run `script.sh`. This will write the flipped path to the specified location
- Open Choreo and enter the newly-generated path
- Gemerate the Choreo path. This will populate the rest of the json fields that `pathflipper` does not touch