"""apl20 autopilot -- the cascade controller. Tracks MAVROS-style setpoints; it
does not plan (run mission.launch.py for the commander).

Optionally brings up the Micro-XRCE-DDS agent (start_agent:=false if you manage
it yourself, e.g. it is already running). Hover is just a single setpoint:

    ros2 launch apl20_ros autopilot.launch.py
    ros2 topic pub -r 10 /autopilot/setpoint_position/local \\
        geometry_msgs/PoseStamped \\
        '{header: {frame_id: map}, pose: {position: {z: -2.0}, \\
          orientation: {w: 1.0}}}'

For a survey, run the commander separately -- it publishes a latched Path the
autopilot sequences + tracks (set track_log here, on the controller, to record):

    ros2 launch apl20_ros autopilot.launch.py track_log:=/tmp/track.csv
    ros2 launch apl20_ros mission.launch.py pattern:=square square.side:=6

PX4 SITL is started separately (it builds/runs Gazebo):

    cd ~/src/PX4-Autopilot && make px4_sitl gz_x500
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "start_agent",
            default_value="true",
            description="Whether to launch the MicroXRCEAgent (false if you run "
            "it yourself).",
        ),
        DeclareLaunchArgument(
            "agent",
            default_value="MicroXRCEAgent",
            description="Path to the MicroXRCEAgent binary.",
        ),
        DeclareLaunchArgument(
            "hover_thrust",
            default_value="0.70",
            description="Normalized hover-thrust operating point [0,1].",
        ),
        DeclareLaunchArgument(
            "accept_radius",
            default_value="0.4",
            description="Path waypoint acceptance radius [m].",
        ),
        DeclareLaunchArgument(
            "settle_speed",
            default_value="0.4",
            description="Max speed for a path waypoint to count as settled [m/s].",
        ),
        DeclareLaunchArgument(
            "track_log",
            default_value="",
            description="CSV path for the shaped-reference-vs-measured tracking "
            "log ('' disables).",
        ),
        DeclareLaunchArgument(
            "auto_engage",
            default_value="false",
            description="Self-command offboard + arm once setpoints stream "
            "(headless SITL only). Off by default: the operator/GCS arms.",
        ),
    ]

    agent = ExecuteProcess(
        cmd=[LaunchConfiguration("agent"), "udp4", "-p", "8888"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_agent")),
    )

    autopilot = Node(
        package="apl20_ros",
        executable="autopilot_node",
        name="autopilot",
        output="screen",
        parameters=[
            {
                "hover_thrust": LaunchConfiguration("hover_thrust"),
                "accept_radius": LaunchConfiguration("accept_radius"),
                "settle_speed": LaunchConfiguration("settle_speed"),
                "track_log": LaunchConfiguration("track_log"),
                "auto_engage": LaunchConfiguration("auto_engage"),
            }
        ],
    )

    return LaunchDescription([*args, agent, autopilot])
