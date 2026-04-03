$JQ = Join-Path $PSScriptRoot "../bin/jq.exe"
$jqScriptPath = Join-Path $PSScriptRoot "pathflipper.jq"

$FIELDHEIGHT = "8.0772"

$input = "src/main/deploy/choreo/Trench2xOutpost.traj"
$rename = "Trench2xDepot"

$outputDir = Split-Path $input
$outputFile = Join-Path $outputDir "$rename.traj"

& $JQ -f $jqScriptPath `
  --argjson FIELD_HEIGHT $FIELDHEIGHT `
  --arg NAME $rename `
  $input > $outputFile