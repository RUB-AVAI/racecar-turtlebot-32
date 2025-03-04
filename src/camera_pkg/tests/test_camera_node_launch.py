import os
import sys
import launch
import launch_ros
import launch_testing.actions
import pytest

@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)
    camera_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "camera_node.py")],
        additional_env={"PYTHONUNBUFFERED": "1"},
    )
    return (
        launch.LaunchDescription([
            camera_node,
            launch_testing.actions.ReadyToTest()
        ]),
        {"camera": camera_node}
    )

