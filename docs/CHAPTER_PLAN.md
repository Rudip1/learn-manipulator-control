# Chapter plan — learn-manipulator-control   (`<pkg>` = `manipulator_control`)

Kinematic control of manipulators, from one Jacobian to many prioritized tasks. Existing material to mine:
resolved-rate control, task-priority control with equality and inequality tasks, joint limits, mobile manipulator.
Validate kinematics against Pinocchio in tests.

1. Rigid-body transforms: rotations, homogeneous transforms, twists (brief, notation for the module).
2. Forward kinematics: DH convention and product of exponentials; the same arm both ways.
3. The Jacobian: geometric vs analytic, numerical check by finite differences.
4. Resolved-rate motion control: pseudoinverse, transpose, convergence and gains.
5. Singularities: manipulability, damped least squares, variable damping.
6. Redundancy and null space: secondary objectives, projection.
7. Task-priority control: two tasks, recursive formulation for N tasks, algorithmic singularities.
8. Inequality tasks: joint limits, obstacle avoidance, set-based activation (and its chattering — break it).
9. Mobile manipulators: base + arm as one kinematic system, weighting base vs arm motion.

Used in: [vehicle-manipulator-task-priority](https://github.com/Rudip1/vehicle-manipulator-task-priority).
Acknowledgement: chapters 8–9 grew out of joint work with Gebrecherkos Gebreslassie.
