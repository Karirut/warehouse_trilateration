import launch
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription()

    k4a_node = Node(
        package="azure_kinect_ros2_driver",
        executable="azure_kinect_node",
        name="k4a_ros2_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"recording_file": "/home/kermit/retofinal_ws/video/-prueba1.mkv"}
        ]
    )

    k4a_test_node = Node(
        package="azure_kinect_ros2_driver",
        executable="test_azure_kinect_subscriber.py",
        output="screen",
        emulate_tty=True
    )

    req_exit  = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=k4a_node,
            on_exit=[
                launch.actions.LogInfo(
                    msg="azure_kinect_node finished; shutting down all nodes"),
                launch.actions.EmitEvent(
                    event=launch.events.Shutdown())]))


    ld.add_action(k4a_node)
    ld.add_action(k4a_test_node)
    ld.add_action(req_exit)

    return ld
