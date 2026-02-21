import os
import unittest
import launch
import launch_testing
import pytest
from ament_index_python.packages import get_package_share_directory


# Define the launch file to test
@pytest.mark.rostest
def generate_test_description():
    # Get the path to the INSTALLED package share directory
    pkg_share = get_package_share_directory('skratch_sensor')

    # Construct the path reliably
    launch_file_path_1 = os.path.join(pkg_share, 'launch', 'dual_laser_merger.launch.py')
    launch_file_path_2 = os.path.join(pkg_share, 'launch', 'laser_relay.launch.py')

    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path_1)
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path_2)
        ),
        launch_testing.actions.ReadyToTest()
    ])


# Define the test assertion
class TestLaunch(unittest.TestCase):
    def test_readiness(self, proc_output):
        pass
