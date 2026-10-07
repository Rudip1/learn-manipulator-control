# Notation

One notation is used through the whole module. Each symbol is defined here once; the chapters refer back to
this table. Vectors are columns. Angles are in radians.

## Frames, rotations, transforms (chapter 1)

| Symbol | Meaning |
|---|---|
| $\{0\}$, $\{i\}$, $\{e\}$ | base (world) frame, frame attached to link $i$, end-effector frame |
| ${}^{a}R_{b} \in SO(3)$ | orientation of frame $\{b\}$ expressed in frame $\{a\}$; its columns are the axes of $\{b\}$ written in $\{a\}$ |
| ${}^{a}p_{b} \in \mathbb{R}^3$ | origin of $\{b\}$ expressed in $\{a\}$ |
| ${}^{a}T_{b} \in SE(3)$ | homogeneous transform $\begin{bmatrix} {}^{a}R_{b} & {}^{a}p_{b} \\ 0 & 1 \end{bmatrix}$; pose of $\{b\}$ in $\{a\}$. Frame indices are dropped when the base frame is meant: $T_i = {}^{0}T_{i}$ |
| $R_x(\cdot), R_y(\cdot), R_z(\cdot)$ | elementary rotations about the coordinate axes |
| $[\omega]_\times$ | $3\times 3$ skew-symmetric matrix with $[\omega]_\times u = \omega \times u$ |
| $\hat\omega$, $\theta$ | unit rotation axis and rotation angle; $\omega\,\theta$ are the exponential coordinates of a rotation |
| $\phi, \vartheta, \psi$ | roll, pitch, yaw (ZYX Euler angles): $R = R_z(\psi) R_y(\vartheta) R_x(\phi)$ |
| $\mathcal{V} = (v, \omega) \in \mathbb{R}^6$ | twist: linear part first, angular part second (same order as the Jacobian rows) |
| $[\mathcal{V}] \in se(3)$ | $4\times4$ matrix $\begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix}$ |
| $\mathcal{S}$ | screw axis, a twist with $\lVert\omega\rVert = 1$ (or $\omega = 0$, $\lVert v \rVert = 1$) |
| $\mathrm{Ad}_T$ | $6\times 6$ adjoint of $T$, maps twists between frames |
| $\exp$, $\log$ | matrix exponential and logarithm on $SO(3)$ and $SE(3)$ |

## Kinematics and control (chapters 2–9)

| Symbol | Meaning |
|---|---|
| $n$ | number of joints (degrees of freedom of the arm) |
| $q \in \mathbb{R}^n$, $\dot q$ | joint positions and joint velocities |
| $a_i, \alpha_i, d_i, \theta_i$ | Denavit–Hartenberg parameters of link $i$ |
| $M$ | end-effector pose at $q = 0$ (product of exponentials) |
| $J(q) \in \mathbb{R}^{6\times n}$ | geometric Jacobian of a frame, rows $(v, \omega)$ in the base frame |
| $J_P$, $J_O$ | position (top three rows) and orientation (bottom three rows) parts of $J$ |
| $J_A(q)$ | analytic Jacobian, derivative of a minimal pose parametrisation |
| $\sigma \in \mathbb{R}^m$ | task variable (end-effector position, a joint angle, a distance, …) |
| $\sigma_d$, $\dot\sigma_d$ | desired task value and its feedforward velocity |
| $\tilde\sigma = \sigma_d - \sigma$ | task error |
| $K$ | positive definite gain matrix |
| $s_1 \ge \dots \ge s_m \ge 0$ | singular values of a Jacobian ($\sigma$ is reserved for task variables) |
| $J^\dagger$ | Moore–Penrose pseudoinverse |
| $J^\dagger_\lambda$ | damped least-squares inverse $J^T (J J^T + \lambda^2 I)^{-1}$ with damping $\lambda$ |
| $W$ | positive definite weighting matrix on joint velocities |
| $w(q)$ | manipulability measure $\sqrt{\det(J J^T)}$ |
| $N = I - J^\dagger J$ | null-space projector of $J$ |
| $J_i$, $\bar J_i = J_i N_{i-1}$ | Jacobian of the task with priority $i$ and its augmented (projected) Jacobian |
| $\Delta t$, $k$ | control period and discrete time index |
| $\eta = (x, y, \psi)$ | pose of a mobile base in the plane |
| $\zeta$ | quasi-velocities of a mobile manipulator: base angular and forward speed, then $\dot q$ |
