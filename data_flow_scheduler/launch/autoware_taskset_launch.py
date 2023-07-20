from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import stat

def generate_launch_description():
    # Get the absolute path to the directory of the launch file
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    # Construct the absolute path to the bash file
    bash_file_path = os.path.join(launch_file_dir, 'singlethreaded_ex.sh')
    # Set the executable permission for the bash file
    os.chmod(bash_file_path, stat.S_IRWXU)

    # Sensor
    s_node1 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), 
        'FrontLidarDriver', 'FrontLidarDriver'],
        output='screen'
    )
    s_node2 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), 
        'RearLidarDriver', 'RearLidarDriver'],
        output='screen'
    )
    s_node3 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), 
        'PointCloudMap', 'PointCloudMap'],
        output='screen'
    )
    s_node4 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), 
        'Visualizer', 'Visualizer'],
        output='screen'
    )
    s_node5 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), 
        'Lanelet2Map', 'Lanelet2Map'],
        output='screen'
    )
    s_node6 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), 
        'EuclideanClusterSettings', 'EuclideanClusterSettings'],
        output='screen'
    )

    #Transform
    t_node1 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(1024), 'PointsTransformerFront', 
        'FrontLidarDriver', 'PointsTransformerFront'],
        output='screen'
    )
    t_node2 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(1024), 'PointsTransformerRear', 
        'RearLidarDriver', 'PointsTransformerRear'],
        output='screen'
    )
    t_node3 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(1024), 'VoxelGridDownsampler', 
        'PointCloudFusion', 'VoxelGridDownsampler'],
        output='screen'
    )
    t_node4 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(1024), 'PointCloudMapLoader', 
        'PointCloudMap', 'PointCloudMapLoader'],
        output='screen'
    )
    t_node5 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(1024), 'RayGroundFilter', 
        'PointCloudFusion', 'RayGroundFilter'],
        output='screen'
    )
    t_node6 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(1024), 'ObjectCollisionEstimator', 
        'EuclideanClusterDetector', 'ObjectCollisionEstimator'],
        output='screen'
    )
    t_node7 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(1024), 'MPCController', 
        'BehaviorPlanner', 'MPCController'],
        output='screen'
    )
    t_node8 = ExecuteProcess( 
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(1024), 'ParkingPlanner', 
        'Lanelet2MapLoader', 'ParkingPlanner'],
        output='screen'
    )
    t_node9 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(1024), 'LanePlanner', 
        'Lanelet2MapLoader', 'LanePlanner'],
        output='screen'
    )

    #fusion
    f_node1 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '3', str(1024), 'PointCloudFusion', 
        'PointsTransformerFront', 'PointsTransformerRear', "PointCloudFusion"],
        output='screen'
    )
    f_node2 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '3', str(1024), 'NDTLocalizer', 
        'VoxelGridDownsampler', 'PointCloudMapLoader', 'NDTLocalizer'],
        output='screen'
    )
    f_node3 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '3', str(1024), 'VehicleInterface', 
        'MPCController', 'BehaviorPlanner', 'VehicleInterface'],
        output='screen'
    )
    f_node4 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '3', str(1024), 'Lanelet2GlobalPlanner', 
        'Visualizer', 'NDTLocalizer', 'Lanelet2GlobalPlanner'],
        output='screen'
    )
    f_node5 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '3', str(1024), 'Lanelet2MapLoader', 
        'Lanelet2Map', 'Lanelet2GlobalPlanner', 'Lanelet2MapLoader'],
        output='screen'
    )

    #cycle
    cy_node1 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '4', str(1024), 'BehaviorPlanner', 
        'ObjectCollisionEstimator', 'NDTLocalizer',
        'Lanelet2GlobalPlanner', 'Lanelet2MapLoader',
        'ParkingPlanner', 'LanePlanner'],
        output='screen'
    )

    #intersection
    i_node1 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '5', str(1024), 'EuclideanClusterDetector', 
        'RayGroundFilter', 'EuclideanClusterDetector', 
        'EuclideanClusterSettings', 'EuclideanIntersection'],
        output='screen'
    )

    #Command
    co_node1 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '6', str(0), 'VehicleDBWSystem', 
        'VehicleInterface'],
        output='screen'
    )
    co_node2 = ExecuteProcess(
        cmd=['taskset', '-c', '7', bash_file_path, '6', str(0), 'IntersectionOutput', 
        'EuclideanIntersection'],
        output='screen'
    )
    
    # Create the launch description and add the actions
    ld = LaunchDescription()
    ld.add_action(s_node1)
    ld.add_action(s_node2)
    ld.add_action(s_node3)
    ld.add_action(s_node4)
    ld.add_action(s_node5)
    ld.add_action(s_node6)
    ld.add_action(t_node1)
    ld.add_action(t_node2)
    ld.add_action(t_node3)
    ld.add_action(t_node4)
    ld.add_action(t_node5)
    ld.add_action(t_node6)
    ld.add_action(t_node7)
    ld.add_action(t_node8)
    ld.add_action(t_node9)
    ld.add_action(f_node1)
    ld.add_action(f_node2)
    ld.add_action(f_node3)
    ld.add_action(f_node4)
    ld.add_action(f_node5)
    ld.add_action(cy_node1)
    ld.add_action(i_node1)
    ld.add_action(co_node1)
    ld.add_action(co_node2)
    return ld
