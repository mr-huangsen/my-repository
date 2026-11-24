import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


DOUBLE_PARAMS = {
    "L", "l", "w", "h", "distance_forward", "distance_back", "control_cycle",
    "MAX_SPEED", "MAX_SPEED_LAT", "MAX_ACCEL", "MAX_CURVATURE", "MAX_ROAD_WIDTH",
    "D_ROAD_W", "DT", "MAXT", "MINT", "DTIME", "TARGET_SPEED", "D_T_S",
    "SMALL_SPEED", "DS", "MAXS", "MINS", "KJ", "KT", "KJD", "KD", "KLAT",
    "KLON", "KCAC", "KEFF", "delta_t", "w_pos", "w_vel", "w_yaw", "w_acc",
    "w_stl", "obstacle_exp_q1", "obstacle_exp_q2", "state_exp_q1",
    "state_exp_q2", "alm_rho_init", "alm_gamma", "max_rho", "max_mu",
    "init_lamb", "lamb_decay", "lamb_amplify", "max_lamb",
    "convergence_threshold", "accept_step_threshold", "wheelbase", "width",
    "length", "velo_max", "velo_min", "yaw_lim", "acc_max", "acc_min",
    "stl_lim", "d_safe", "target_velocity",
}


def _load_params(package_share, file_name):
    path = os.path.join(package_share, "config", file_name)
    with open(path, "r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream) or {}
    normalized = {}
    for key, value in params.items():
        if not isinstance(value, (bool, int, float, str)):
            continue
        normalized[key] = float(value) if key in DOUBLE_PARAMS else value
    return normalized


def generate_launch_description():
    package_share = get_package_share_directory("plan")
    params = {}
    params.update(_load_params(package_share, "plan.yaml"))
    params.update(_load_params(package_share, "ilqr.yaml"))

    return LaunchDescription([
        Node(
            package="plan",
            executable="plan_node",
            name="plan",
            output="screen",
            parameters=[params],
        )
    ])
