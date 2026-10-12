# R1 Pro Web Control Runtime Data

This directory is a versioned snapshot of the current Web UI configuration:

- `actions.json`: 74 atomic actions, including navigation goals, code-workflow entries,
  and editable EE axial-motion examples (forward and reverse).
- `sequences.json`: 5 saved multi-step workflows.

The running server reads these files from `~/.ros/r1pro_web_control/`. To restore
the snapshot, stop the Web control server, back up the current files, copy these
two JSON files into that directory, and then restart the server. Preserve the
filenames exactly.

Before using the snapshot on a different robot or map, review all joint targets,
end-effector poses, chassis velocities, navigation coordinates, delays, and
workflow parameters. Validate motion in a safe environment before execution.

The runtime authentication token, logs, perception captures, registration
bundles, temporary files, and migration backups are intentionally excluded.
