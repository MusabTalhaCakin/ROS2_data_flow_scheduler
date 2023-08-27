from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import stat

def generate_launch_description():
    # Get the absolute path to the directory of the launch file
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    # Construct the absolute path to the bash file
    bash_file_path = os.path.join(launch_file_dir, 'singlethreaded_exec.sh')
    # Set the executable permission for the bash file
    os.chmod(bash_file_path, stat.S_IRWXU)


    #multifusion
    mf_node1 = ExecuteProcess(
        cmd=['taskset', '-c', '3', bash_file_path, '7', str(1024), 'BehaviorPlanner',
        'ObjectCollisionEstimator', 'NDTLocalizer',
        'Lanelet2GlobalPlanner', 'Lanelet2MapLoader',
        'ParkingPlanner', 'LanePlanner', 'BehaviorPlanner'],
        output='screen'
        )
    cy_node1 = ExecuteProcess(
        cmd=['taskset', '-c', '3', bash_file_path, '4', str(1024), 'BehaviorPlanner', 
        'ObjectCollisionEstimator', 'NDTLocalizer',
        'Lanelet2GlobalPlanner', 'Lanelet2MapLoader',
        'ParkingPlanner', 'LanePlanner'],
        output='screen'
        )    
    i_node1 = ExecuteProcess(
        cmd=['taskset', '-c', '3', bash_file_path, '5', str(1024), 'EuclideanClusterDetector', 
        'RayGroundFilter', 'EuclideanClusterDetector', 
        'EuclideanClusterSettings', 'EuclideanIntersection'],
        output='screen'
        )

    # Create the launch description and add the actions
    ld = LaunchDescription()
    ld.add_action(mf_node1)
    return ld
