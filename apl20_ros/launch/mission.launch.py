"""apl20 mission commander -- the planner half of the controller/commander split.
Builds a waypoint pattern in absolute local NED and publishes it ONCE as a
latched nav_msgs/Path on the autopilot's setpoint_path/local; the autopilot then
sequences + tracks it. Run alongside autopilot.launch.py (order does not matter,
the Path is latched):

    ros2 launch apl20_ros autopilot.launch.py
    ros2 launch apl20_ros mission.launch.py pattern:=square square.side:=6
    ros2 launch apl20_ros mission.launch.py pattern:=lawnmower \\
        mow.length:=12 mow.width:=8

The pattern is centred on the target pose (x north, y east, altitude up, yaw
heading). Tracking-log recording lives on the autopilot now (track_log there),
since it owns the shaped reference -- the commander just hands over the path and
can be killed once it is delivered.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "pattern",
            default_value="hover",
            description="Mission pattern: hover | square | lawnmower.",
        ),
        DeclareLaunchArgument(
            "x", default_value="0.0", description="Pattern centre, NED north [m]."
        ),
        DeclareLaunchArgument(
            "y", default_value="0.0", description="Pattern centre, NED east [m]."
        ),
        DeclareLaunchArgument(
            "altitude",
            default_value="2.0",
            description="Survey altitude above NED origin [m].",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Heading held over the pattern [rad].",
        ),
        DeclareLaunchArgument(
            "square.side", default_value="4.0", description="Square pattern side [m]."
        ),
        DeclareLaunchArgument(
            "mow.length", default_value="10.0", description="Lawnmower lane length [m]."
        ),
        DeclareLaunchArgument(
            "mow.width",
            default_value="6.0",
            description="Lawnmower coverage width [m].",
        ),
        DeclareLaunchArgument(
            "mow.spacing",
            default_value="3.0",
            description="Lawnmower lane spacing [m].",
        ),
    ]

    mission = Node(
        package="apl20_ros",
        executable="mission_player",
        name="mission_player",
        output="screen",
        parameters=[
            {
                "pattern": LaunchConfiguration("pattern"),
                "target.x": LaunchConfiguration("x"),
                "target.y": LaunchConfiguration("y"),
                "target.altitude": LaunchConfiguration("altitude"),
                "target.yaw": LaunchConfiguration("yaw"),
                "square.side": LaunchConfiguration("square.side"),
                "mow.length": LaunchConfiguration("mow.length"),
                "mow.width": LaunchConfiguration("mow.width"),
                "mow.spacing": LaunchConfiguration("mow.spacing"),
            }
        ],
        remappings=[
            ("setpoint_path/local", "/autopilot/setpoint_path/local"),
        ],
    )

    return LaunchDescription([*args, mission])
