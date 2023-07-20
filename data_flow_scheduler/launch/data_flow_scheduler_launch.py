from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import stat

def generate_launch_description():
    # Get the absolute path to the directory of the launch file
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    # Construct the absolute path to the bash file
    bash_file_path = os.path.join(launch_file_dir, 'data_flow_scheduler.sh')
    # Set the executable permission for the bash file
    os.chmod(bash_file_path, stat.S_IRWXU)

    node = ExecuteProcess(
        cmd=['taskset', '-c', '0', bash_file_path, '24'],
        output='screen'
    )
    
    # Create the launch description and add the actions
    ld = LaunchDescription()
    ld.add_action(node)
    return ld