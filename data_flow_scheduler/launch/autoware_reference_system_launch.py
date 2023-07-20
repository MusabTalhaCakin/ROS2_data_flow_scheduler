from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import stat

def generate_launch_description():
    # Get the absolute path to the directory of the launch file
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    # Construct the absolute path to the bash file
    bash_file_path = os.path.join(launch_file_dir, 'node_bridge_autoware.sh')
    # Set the executable permission for the bash file
    os.chmod(bash_file_path, stat.S_IRWXU)

    # Sensor
    s_node1 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '1', str(100000), str(0), 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), str(0), 
>>>>>>> Stashed changes
        'FrontLidarDriver', 'FrontLidarDriver'],
        output='screen'
    )
    s_node2 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '1', str(100000), str(0), 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), str(0), 
>>>>>>> Stashed changes
        'RearLidarDriver', 'RearLidarDriver'],
        output='screen'
    )
    s_node3 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '1', str(100000), str(0), 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), str(0), 
>>>>>>> Stashed changes
        'PointCloudMap', 'PointCloudMap'],
        output='screen'
    )
    s_node4 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '1', str(100000), str(0), 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), str(0), 
>>>>>>> Stashed changes
        'Visualizer', 'Visualizer'],
        output='screen'
    )
    s_node5 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '1', str(100000), str(0), 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), str(0), 
>>>>>>> Stashed changes
        'Lanelet2Map', 'Lanelet2Map'],
        output='screen'
    )
    s_node6 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '1', str(100000), str(0), 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '1', str(100000), str(0), 
>>>>>>> Stashed changes
        'EuclideanClusterSettings', 'EuclideanClusterSettings'],
        output='screen'
    )

    #Transform
    t_node1 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '2', str(100000), str(2048), 'PointsTransformerFront', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '2', str(100000), str(1024), 'PointsTransformerFront', 
>>>>>>> Stashed changes
        'FrontLidarDriver', 'PointsTransformerFront'],
        output='screen'
    )
    t_node2 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '2',str(100000), str(2048), 'PointsTransformerRear', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '2',str(100000), str(1024), 'PointsTransformerRear', 
>>>>>>> Stashed changes
        'RearLidarDriver', 'PointsTransformerRear'],
        output='screen'
    )
    t_node3 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '2',str(100000), str(2048), 'VoxelGridDownsampler', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '2',str(100000), str(1024), 'VoxelGridDownsampler', 
>>>>>>> Stashed changes
       'PointCloudFusion', 'VoxelGridDownsampler'],
        output='screen'
    )
    t_node4 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '2',str(100000), str(2048), 'PointCloudMapLoader', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '2',str(100000), str(1024), 'PointCloudMapLoader', 
>>>>>>> Stashed changes
        'PointCloudMap', 'PointCloudMapLoader'],
        output='screen'
    )
    t_node5 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '2',str(100000), str(2048), 'RayGroundFilter', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '2',str(100000), str(1024), 'RayGroundFilter', 
>>>>>>> Stashed changes
        'PointCloudFusion', 'RayGroundFilter'],
        output='screen'
    )
    t_node6 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '2',str(100000), str(2048), 'ObjectCollisionEstimator', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '2',str(100000), str(1024), 'ObjectCollisionEstimator', 
>>>>>>> Stashed changes
        'EuclideanClusterDetector', 'ObjectCollisionEstimator'],
        output='screen'
    )
    t_node7 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '2',str(100000), str(2048), 'MPCController', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '2',str(100000), str(1024), 'MPCController', 
>>>>>>> Stashed changes
        'BehaviorPlanner', 'MPCController'],
        output='screen'
    )
    t_node8 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '2',str(100000), str(2048), 'ParkingPlanner', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '2',str(100000), str(1024), 'ParkingPlanner', 
>>>>>>> Stashed changes
        'Lanelet2MapLoader', 'ParkingPlanner'],
        output='screen'
    )
    t_node9 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '2',str(100000), str(2048), 'LanePlanner', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '2',str(100000), str(1024), 'LanePlanner', 
>>>>>>> Stashed changes
        'Lanelet2MapLoader', 'LanePlanner'],
        output='screen'
    )

    #fusion
    f_node1 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '3',str(200000), str(4096), 'PointCloudFusion', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '3',str(200000), str(1024), 'PointCloudFusion', 
>>>>>>> Stashed changes
        'PointsTransformerFront', 'PointsTransformerRear', "PointCloudFusion"],
        output='screen'
    )
    f_node2 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '3',str(200000), str(4096), 'NDTLocalizer', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '3',str(200000), str(1024), 'NDTLocalizer', 
>>>>>>> Stashed changes
        'VoxelGridDownsampler', 'PointCloudMapLoader', 'NDTLocalizer'],
        output='screen'
    )
    f_node3 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '3',str(200000), str(4096), 'VehicleInterface', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '3',str(200000), str(1024), 'VehicleInterface', 
>>>>>>> Stashed changes
        'MPCController', 'BehaviorPlanner', 'VehicleInterface'],
        output='screen'
    )
    f_node4 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '3',str(200000), str(4096), 'Lanelet2GlobalPlanner', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '3',str(200000), str(1024), 'Lanelet2GlobalPlanner', 
>>>>>>> Stashed changes
        'Visualizer', 'NDTLocalizer', 'Lanelet2GlobalPlanner'],
        output='screen'
    )
    f_node5 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '3',str(200000), str(4096), 'Lanelet2MapLoader', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '3',str(200000), str(1024), 'Lanelet2MapLoader', 
>>>>>>> Stashed changes
        'Lanelet2Map', 'Lanelet2GlobalPlanner', 'Lanelet2MapLoader'],
        output='screen'
    )

    #cycle
    cy_node1 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '4', str(500000), str(8192), 'BehaviorPlanner', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '4', str(500000), str(1024), 'BehaviorPlanner', 
>>>>>>> Stashed changes
        'ObjectCollisionEstimator', 'NDTLocalizer',
        'Lanelet2GlobalPlanner', 'Lanelet2MapLoader',
        'ParkingPlanner', 'LanePlanner'],
        output='screen'
    )

    #intersection
    i_node1 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '5',str(100000), str(2048), 'EuclideanClusterDetector', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '5',str(100000), str(1024), 'EuclideanClusterDetector', 
>>>>>>> Stashed changes
        'RayGroundFilter', 'EuclideanClusterDetector', 
        'EuclideanClusterSettings', 'EuclideanIntersection'],
        output='screen'
    )

    #Command
    co_node1 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '6', str(100000), str(0), 'VehicleDBWSystem', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '6', str(100000), str(0), 'VehicleDBWSystem', 
>>>>>>> Stashed changes
        'VehicleInterface'],
        output='screen'
    )
    co_node2 = ExecuteProcess(
<<<<<<< Updated upstream
        cmd=['taskset', '-c', '0', bash_file_path, '6', str(100000), str(0), 'IntersectionOutput', 
=======
        cmd=['taskset', '-c', '7', bash_file_path, '6', str(100000), str(0), 'IntersectionOutput', 
>>>>>>> Stashed changes
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