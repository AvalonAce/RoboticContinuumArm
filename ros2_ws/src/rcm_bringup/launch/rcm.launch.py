"""Full RCM stack.   ros2 launch rcm_bringup rcm.launch.py [sim:=true] [arm:=false] [sensors:=false] [port:=...]"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory("rcm_bringup"), "config", "arm.yaml")
    root = os.environ.get("RCM_ROOT")
    if root and os.path.isfile(os.path.join(root, "ros2_ws/src/rcm_bringup/config/arm.yaml")):
        default_cfg = os.path.join(root, "ros2_ws/src/rcm_bringup/config/arm.yaml")   # edit without rebuild
    cfg = LaunchConfiguration("config")
    sim = LaunchConfiguration("sim")
    port = LaunchConfiguration("port")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_cfg),
        DeclareLaunchArgument("sim", default_value="false", description="simulated board, no hardware"),
        DeclareLaunchArgument("port", default_value="", description="serial port override"),
        DeclareLaunchArgument("dashboard", default_value="true"),
        DeclareLaunchArgument("arm", default_value="true", description="servo bridge (false = sensors/dashboard only)"),
        DeclareLaunchArgument("sensors", default_value="true", description="tip IMU bridge (tolerates board absent)"),
        DeclareLaunchArgument("sensor_port", default_value="", description="IMU board port override"),
        DeclareLaunchArgument("flex_port", default_value="", description="flex board port override"),
        Node(package="rcm_hw", executable="hw_bridge", name="hw_bridge", output="screen",
             parameters=[cfg, {"sim": sim, "port": port}], condition=IfCondition(LaunchConfiguration("arm"))),
        Node(package="rcm_sensors", executable="sensor_bridge", name="sensor_bridge", output="screen",
             parameters=[cfg, {"sim": sim, "port": LaunchConfiguration("sensor_port")}],
             condition=IfCondition(LaunchConfiguration("sensors"))),
        Node(package="rcm_sensors", executable="flex_bridge", name="flex_bridge", output="screen",
             parameters=[cfg, {"sim": sim, "port": LaunchConfiguration("flex_port")}],
             condition=IfCondition(LaunchConfiguration("sensors"))),
        Node(package="rcm_kinematics", executable="kinematics", name="kinematics", output="screen",
             parameters=[cfg]),
        Node(package="rcm_experiments", executable="test_runner", name="test_runner", output="screen",
             parameters=[cfg]),
        Node(package="rcm_dashboard", executable="dashboard", name="dashboard", output="screen",
             parameters=[cfg], condition=IfCondition(LaunchConfiguration("dashboard"))),
    ])
