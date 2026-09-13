# Optional WBC assets

The official alpha policies are installed from the Hub into the policy store; see [policy commands](../docs/robot/cheatsheet.md) and `scripts/seed-policies.sh`. Only the local WBC network and reference CSVs remain bundled with this fork.

### Optional WBC skill assets

`wbc_v1.onnx` is the shipped 72-input controller. Four headerless 24-column references run at
50 Hz: `wbc_happy.csv` (the default deployed 989-frame motion), `wbc_curious.csv`,
`wbc_happy_bob.csv`, and `wbc_wiggle.csv`. Set `[wbc] reference` to one of these filenames;
a relative path resolves inside the current release's `policies/` directory.

The CSV already contains the training-side linear/angular velocities and is parsed once before
the realtime loop. Observation order is reference (24), gyro (3), projected gravity (3),
`q - HOME` (14), joint velocity (14), previous action (14). Output is a residual added to the
reference joint pose; the mouth is excluded. The final row returns through HOME to alpha.

WBC is disabled by default. Invalid ONNX shape or CSV contents disable only this mode and surface
the reason through `robotctl health` and `robotctl monitor`.

The additional CSVs are exported from `wbc-mjlab` holdout motions and share the deployed ABI,
but are not claimed as hardware-qualified. Exercise a new reference in simulation and then under
a support frame before floor use.
