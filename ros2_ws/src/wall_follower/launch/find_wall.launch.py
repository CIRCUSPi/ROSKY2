import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import ThisLaunchFile, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.launch_description_source import LaunchDescriptionSource

from pathlib import Path

def generate_launch_description():
    ld = LaunchDescription()

    # get environment variable
    self_name_space = os.environ["VEHICLE_NAME"]
    self_package = "wall_follower"
    self_node_name = "find_wall_server"

    # get parameter file path
    example = Path(
        get_package_share_directory(self_package),
        "pid_controller.yaml"
    )

    # remapping topic 
    self_scan_to_scan = ("~/scan", "/scan")
    self_cmd_to_cmd = ("~/cmd_vel", "/cmd_vel")

    # remapping service
    self_stop_record_to_stop_record = (f"~/stop_record", f"/{self_name_space}/record_odometry_action_server/stop_record")
    self_ominibotcar_driver_parameter_to_ominibotcar_driver_parameter = (f"~/ominibot_car_driver/get_parameters", f"/{self_name_space}/ominibot_car_driver/get_parameters")

    find_wall_server = Node(
        package=self_package,
        executable=self_node_name,
        namespace=self_name_space,
        output="log",
        #parameters=[],
        remappings=[
            self_scan_to_scan, 
            self_cmd_to_cmd, 
            self_stop_record_to_stop_record,
            self_ominibotcar_driver_parameter_to_ominibotcar_driver_parameter,
            ],

    )
    ld.add_action(find_wall_server)
    return ld