from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def diag(values):
    """Row-major flattening of a diagonal matrix."""
    n = len(values)
    return [values[i] if i == j else 0.0 for i in range(n) for j in range(n)]


# A: original model (minimal fixes only).
#    z = [x, y, theta_odom, theta_imu, w_imu, a_x, a_y]
# B: world-frame constant acceleration + body-frame observations, with odom pose.
#    z = [x, y, theta_odom, theta_imu, w_imu, v_fwd, v_lat, a_fwd, a_lat]
# C: like B but without the odom pose (wheel velocity + IMU only).
#    z = [theta_imu, w_imu, v_fwd, v_lat, a_fwd, a_lat]
# D: IMU-driven prediction (gyro + accel are the control input), odom twist is the only measurement.
#    state = [x, y, theta, v_x, v_y, b_w, b_af, b_al],  z = [v_fwd, v_lat]
#    Q is a continuous-time density (per second); the IMU noise enters through imu_noise_density.
#
# Q order (A-C): [x, y, theta, v_x, v_y, w, a_x, a_y]
VARIANTS = {
    "A": dict(model_fixes=False, use_odom_pose=True,
              Q=[0.1] * 8,
              R=[0.05] * 7),
    "B": dict(model_fixes=True, use_odom_pose=True,
              Q=[0.01, 0.01, 1e-4, 0.05, 0.05, 0.01, 0.1, 0.1],
              R=[0.5, 0.5, 0.01, 0.001, 0.001, 0.01, 0.01, 0.5, 0.5]),
    "C": dict(model_fixes=True, use_odom_pose=False,
              Q=[0.01, 0.01, 1e-4, 0.05, 0.05, 0.01, 0.1, 0.1],
              R=[0.001, 0.001, 0.01, 0.01, 0.5, 0.5]),
    "D": dict(imu_propagation=True,
              imu_noise_density=[0.005, 0.5],
              Q=[0.0, 0.0, 0.0, 0.1, 0.1, 1e-10, 1e-3, 1e-3],
              R=[0.002, 0.002],
              Cov_init=[1e-4, 1e-4, 1e-4, 4.0, 4.0, 4e-6, 0.1, 0.1]),
}


def make_node(context):
    v = VARIANTS[LaunchConfiguration("variant").perform(context)]
    q, r = v["Q"], v["R"]
    return [Node(
        package="kalman_filter_app",
        executable="kalman_filter_app",
        name=LaunchConfiguration("node_name"),
        output='screen',
        remappings=[
            ("/odom", LaunchConfiguration("odom_topic")),
            ("/imu", LaunchConfiguration("imu_topic")),
            ("/ekf/odom", LaunchConfiguration("output_topic")),
            ("/ekf/nis", LaunchConfiguration("nis_topic")),
        ],
        parameters=[
            {"model_fixes": v.get("model_fixes", False)},
            {"use_odom_pose": v.get("use_odom_pose", False)},
            {"imu_propagation": v.get("imu_propagation", False)},
            {"imu_noise_density": v.get("imu_noise_density", [0.0, 0.0])},
            {"Q_process_noise": diag(q)},
            {"Q_rows": len(q)},
            {"Q_cols": len(q)},
            {"R_observation_noise": diag(r)},
            {"R_rows": len(r)},
            {"R_cols": len(r)},
            {"X_init": [0.0] * 8},
            {"X_init_size": 8},
            {"Cov_init": diag(v.get("Cov_init", [1.0] * 8))},
            {"Cov_init_rows": 8},
            {"Cov_init_cols": 8},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("variant", default_value="C", description="A, B, C or D (see VARIANTS)"),
        DeclareLaunchArgument("odom_topic", default_value="/wheel_odom"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data_raw"),
        DeclareLaunchArgument("output_topic", default_value="/ekf/odom"),
        DeclareLaunchArgument("nis_topic", default_value="/ekf/nis"),
        DeclareLaunchArgument("node_name", default_value="IMUODOMNODE"),
        OpaqueFunction(function=make_node),
    ])
