.name = $NAME
| .trajectory.samples = [] # delete generated path

| .snapshot.waypoints |= map(
    .y = $FIELD_HEIGHT - .y
    | .heading = - .heading
  )
| .params.waypoints |= map(
    .y.val = $FIELD_HEIGHT - .y.val
    | .y.exp = "\(.y.val) m"
    | .heading.val = - .heading.val
    | .heading.exp = "\(.heading.val) rad"
  )

| .snapshot.constraints |= map(
    if .data.type == "KeepOutCircle" then
      .data.props.y |= ($FIELD_HEIGHT - .)
    elif .data.type == "KeepInRectangle" then
      .data.props |= (
        .y = $FIELD_HEIGHT - .y - .h
      )
    else . end
  )
| .params.constraints |= map(
    if .data.type == "KeepOutCircle" then
      .data.props.y.val |= ($FIELD_HEIGHT - .)
      | .data.props.y.exp = "\(.data.props.y.val) m"
    elif .data.type == "KeepInRectangle" then
      .data.props.h.val as $h
      | .data.props.y.val |= $FIELD_HEIGHT - . - $h
      | .data.props.y.exp = "\(.data.props.y.val) m"
    else . end
  )
