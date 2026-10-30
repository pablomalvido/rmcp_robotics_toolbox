
import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb


# ============================================================
# Configuration
# ============================================================

robot = rtb.models.DH.Puma560()

N = 200
T_END = 5.0
t = np.linspace(0, T_END, N)

print(robot)


# ============================================================
# Helper functions
# ============================================================

def forward_kinematics_trajectory(q_traj):
    """Calculate end-effector poses for a sequence of joint vectors."""
    return [robot.fkine(q) for q in q_traj]


def pose_arrays(Ts):
    """Extract Cartesian position and RPY orientation from poses."""
    xyz = np.array([T.t for T in Ts])
    rpy = np.array([T.rpy() for T in Ts])
    return xyz, rpy


def inverse_kinematics_trajectory(Ts, q_initial):
    """
    Solve IK sequentially, using the previous solution as the
    initial guess for the next pose.
    """
    q_traj = []
    q_seed = np.asarray(q_initial, dtype=float).copy()

    for i, T in enumerate(Ts):
        sol = robot.ikine_LM(T, q0=q_seed)

        if not sol.success:
            raise RuntimeError(
                f"IK failed at waypoint {i}/{len(Ts) - 1}. "
                "Try different endpoints or a different initial guess."
            )

        q_seed = sol.q.copy()
        q_traj.append(q_seed)

    return np.asarray(q_traj)


def plot_motion(q_traj, title):
    """Plot joint values, Cartesian position, x-y path and orientation."""
    Ts = forward_kinematics_trajectory(q_traj)
    xyz, rpy = pose_arrays(Ts)

    fig, axes = plt.subplots(3, 1, figsize=(10, 11))

    # Joint values versus time
    for j in range(robot.n):
        axes[0].plot(t, np.rad2deg(q_traj[:, j]),
                     label=f"q{j + 1}")
    axes[0].set_title(f"{title}: joint values")
    axes[0].set_ylabel("Joint angle (deg)")
    axes[0].set_xlabel("Time (s)")
    axes[0].grid(True)
    axes[0].legend(ncol=3)

    # Cartesian position versus time
    for i, label in enumerate(["x", "y", "z"]):
        axes[1].plot(t, xyz[:, i], label=label)
    axes[1].set_title("Cartesian position versus time")
    axes[1].set_ylabel("Position (m)")
    axes[1].set_xlabel("Time (s)")
    axes[1].grid(True)
    axes[1].legend()

    # Orientation versus time
    for i, label in enumerate(["Roll", "Pitch", "Yaw"]):
        axes[2].plot(t, np.rad2deg(rpy[:, i]), label=label)
    axes[2].set_title("End-effector orientation versus time")
    axes[2].set_ylabel("Angle (deg)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True)
    axes[2].legend()

    fig.tight_layout()

    # Separate x-y trajectory plot
    plt.figure(figsize=(7, 6))
    plt.plot(xyz[:, 0], xyz[:, 1], linewidth=2)
    plt.scatter(xyz[0, 0], xyz[0, 1], label="Start")
    plt.scatter(xyz[-1, 0], xyz[-1, 1], label="End")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title(f"{title}: x-y trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    return xyz, rpy


# ============================================================
# PART A: Joint-space and Cartesian-space motion
# ============================================================

# Choose two joint configurations within the robot's workspace.
# Angles are specified in radians.
q_start = np.deg2rad([0, -30, 60, 0, 45, 0])
q_goal  = np.deg2rad([45, -20, 40, 30, 60, -20])


# ------------------------------------------------------------
# A1. Joint-space motion
# ------------------------------------------------------------

# Quintic joint trajectory between the two configurations.
traj_joint = rtb.jtraj(q_start, q_goal, t)
q_joint = traj_joint.q

xyz_joint, rpy_joint = plot_motion(
    q_joint, "Joint-space motion"
)


# ------------------------------------------------------------
# A2. Cartesian-space motion
# ------------------------------------------------------------

# Obtain the start and goal end-effector poses.
T_start = robot.fkine(q_start)
T_goal = robot.fkine(q_goal)

# Interpolate a straight Cartesian path with smooth orientation
# interpolation between the two end-effector poses.
T_cartesian = rtb.ctraj(T_start, T_goal, N)

# Solve inverse kinematics for each Cartesian waypoint.
q_cartesian = inverse_kinematics_trajectory(
    T_cartesian, q_start
)

xyz_cart, rpy_cart = plot_motion(
    q_cartesian, "Cartesian-space motion"
)

plt.show()


# ============================================================
# PART B: Motion through a wrist singularity
# ============================================================

# For the standard Puma 560 DH model, q5 = 0 is a wrist
# singularity: the wrist axes associated with q4 and q6 align.
#
# Keep the other joints fixed and move q5 through zero.

q_sing_start = np.deg2rad([195, 110, -18, -86, 17, -97])
q_sing_end   = np.deg2rad([132, 110, -18, 90, 51, 92])

# Joint-space trajectory: q5 crosses zero.
traj_sing = rtb.jtraj(q_sing_start, q_sing_end, t)
q_sing_joint = traj_sing.q

xyz_sing_joint, rpy_sing_joint = plot_motion(
    q_sing_joint, "Joint-space motion through wrist singularity"
)

# Cartesian-space trajectory between the same endpoint poses.
T_sing_start = robot.fkine(q_sing_start)
T_sing_end = robot.fkine(q_sing_end)

T_sing_cart = rtb.ctraj(T_sing_start, T_sing_end, N)

# Sequential IK can become ill-conditioned near the singularity.
q_sing_cart = inverse_kinematics_trajectory(
    T_sing_cart, q_sing_start
)

xyz_sing_cart, rpy_sing_cart = plot_motion(
    q_sing_cart, "Cartesian-space motion near wrist singularity"
)


# ------------------------------------------------------------
# Singularity diagnostics
# ------------------------------------------------------------

def calculate_jacobian_metrics(q_traj):
    """
    Compute the smallest singular value of the geometric Jacobian.
    A value close to zero indicates proximity to a singularity.
    """
    sigma_min = []
    condition_number = []
    jac_det = []

    for q in q_traj:
        J = robot.jacob0(q)
        singular_values = np.linalg.svd(J, compute_uv=False)

        sigma_min.append(singular_values[-1])

        if singular_values[-1] < 1e-12:
            condition_number.append(np.inf)
        else:
            condition_number.append(
                singular_values[0] / singular_values[-1]
            )

        jac_det.append(np.linalg.det(J))
        
    return np.asarray(sigma_min), np.asarray(condition_number), np.asarray(jac_det)


sigma_joint, cond_joint, det_joint = calculate_jacobian_metrics(q_sing_joint)
sigma_cart, cond_cart, det_cart = calculate_jacobian_metrics(q_sing_cart)

fig, axes = plt.subplots(3, 1, figsize=(10, 12))

axes[0].plot(t, sigma_joint, label="Joint-space")
axes[0].plot(t, sigma_cart, "--", label="Cartesian-space")
axes[0].set_ylabel("Smallest singular value")
axes[0].set_title("Jacobian singularity indicator")
axes[0].grid(True)
axes[0].legend()

# Clip very large condition numbers for a readable plot.
axes[1].plot(t, np.minimum(cond_joint, 1e6), label="Joint-space")
axes[1].plot(t, np.minimum(cond_cart, 1e6), "--",
             label="Cartesian-space")
axes[1].set_ylabel("Jacobian condition number")
axes[1].set_xlabel("Time (s)")
axes[1].set_title("Robot Jacobian conditioning")
axes[1].grid(True)
axes[1].legend()

# Clip very large condition numbers for a readable plot.
axes[2].plot(t, det_joint, label="Joint-space")
axes[2].plot(t, det_cart, "--", label="Cartesian-space")
axes[2].set_ylabel("Jacobian determinant")
axes[2].set_xlabel("Time (s)")
axes[2].set_title("Robot Jacobian determinant")
axes[2].grid(True)
axes[2].legend()

fig.tight_layout()
plt.show()

print("Joint-space q5 range (deg):",
      np.rad2deg([q_sing_joint[:, 4].min(),
                  q_sing_joint[:, 4].max()]))

print("Minimum Jacobian singular value:")
print(f"  Joint-space:     {sigma_joint.min():.6e}")
print(f"  Cartesian-space: {sigma_cart.min():.6e}")
