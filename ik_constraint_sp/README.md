# ik_constraint_sp

`SPConstraint` constrains the world-XY projection of the mass-weighted combined
COM of the models registered with `setModels()` to the convex hull of all support
polygon vertices projected into world XY. This is a geometric static-balance
constraint, not a friction/wrench feasibility test (SCFR).

Each `SupportPolygon` specifies a const model shared pointer, `jointId`,
`localPose` in that joint, and clockwise `Eigen::Vector2d` vertices in the local
pose's XY plane. A null model with joint ID zero specifies a fixed world pose.
Support-only models are included in the inherited model/data mapping but do not
contribute mass. `getCOMModels()` returns the mass-contributing models.

Call `setModels()` and `setSupportPolygons()` before solving. The solver must
update joint placements and `Data.com[0]` before `updateBounds()`, as it does for
`COMConstraint`. `updateBounds()` precedes `updateJacobian()`.
Support poses are evaluated at the current state, but their motion is deliberately
ignored in the Jacobian; fix supporting feet/joints using position constraints.
Degenerate projected hulls throw an exception rather than silently disabling balance.

`setTolerance()` specifies an inward edge margin in metres (default zero).
`setPrecision()` is the allowed norm of margin violations (default 0.0001 m).
`setMaxError()` limits the restoring step (default 0.02 m).
`clone()` shares const models and copies per-constraint computation state.

`drawOn(viewer)` draws green support polygons, the cyan combined convex hull,
and red spheres at the combined COM and its projection. The projection is drawn
at the hull's average height, for visualization only; feasibility uses XY alone.

The `sp` sample fixes both G1 feet, includes an 8 kg box in the COM calculation,
holds the box with both hands, and moves it forward and up at lowest priority:

```sh
ros2 run prioritized_inverse_kinematics_solver_sample sp
ros2 run prioritized_inverse_kinematics_solver_sample sp --headless
```

The sample enforces foot poses, support margin and joint limits first, rigid
hand/box grasps second, and the box target third. It does not perform collision
avoidance or a contact-force feasibility check.
