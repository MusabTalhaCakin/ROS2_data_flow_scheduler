from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import stat

def generate_launch_description():
    # Get the absolute path to the directory of the launch file
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    # Construct the absolute path to the bash file
    bash_file_path = os.path.join(launch_file_dir, 'node_bridge_demo_graph.sh')
    # Set the executable permission for the bash file
    os.chmod(bash_file_path, stat.S_IRWXU)

    node1 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(0), str(100000), 'Node1', 'topic1'],
        output='screen',
        cwd=launch_file_dir
    )
    node2 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node2', 'topic2', 'topic1'],
        output='screen',
        cwd=launch_file_dir
    )
    node3 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node3', 'topic3', 'topic1'],
        output='screen',
        cwd=launch_file_dir
    )
    node4 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node4', 'topic8', 'topic3'],
        output='screen',
        cwd=launch_file_dir
    )
    node5 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node5', 'topic5', 'topic1'],
        output='screen',
        cwd=launch_file_dir
    )
    node6 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node6', 'topic6', 'topic1'],
        output='screen',
        cwd=launch_file_dir
    )
    node7 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node7', 'topic7', 'topic3'],
        output='screen',
        cwd=launch_file_dir
    )
    node8 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node8', 'topic8', 'topic1'],
        output='screen',
        cwd=launch_file_dir
    )
    node9 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node9', 'topic3', 'topic3'],
        output='screen',
        cwd=launch_file_dir
    )
    node10 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node10', 'topic7', 'topic8'],
        output='screen',
        cwd=launch_file_dir
    )
    node11 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node11', 'topic11', 'topic3'],
        output='screen',
        cwd=launch_file_dir
    )
    node12 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node12', 'topic12', 'topic5'],
        output='screen',
        cwd=launch_file_dir
    )
    node13 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node13', 'topic13', 'topic2'],
        output='screen',
        cwd=launch_file_dir
    )
    node14 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node14', 'topic14', 'topic7'],
        output='screen',
        cwd=launch_file_dir
    )
    node15 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node15', 'topic15', 'topic12'],
        output='screen',
        cwd=launch_file_dir
    )
    node16 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node16', 'topic16', 'topic15'],
        output='screen',
        cwd=launch_file_dir
    )
    node17 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node17', 'topic17', 'topic14'],
        output='screen',
        cwd=launch_file_dir
    )
    node18 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node18', 'topic11', 'topic16'],
        output='screen',
        cwd=launch_file_dir
    )
    node19 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node19', 'topic19', 'topic8'],
        output='screen',
        cwd=launch_file_dir
    )
    node20 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node20', 'topic20', 'topic11'],
        output='screen',
        cwd=launch_file_dir
    )
    node21 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node21', 'topic17', 'topic20'],
        output='screen',
        cwd=launch_file_dir
    )
    node22 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node22', 'topic17', 'topic15'],
        output='screen',
        cwd=launch_file_dir
    )
    node23 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node23', 'topic23', 'topic1'],
        output='screen',
        cwd=launch_file_dir
    )
    node24 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node24', 'topic17', 'topic23'],
        output='screen',
        cwd=launch_file_dir
    )
    node25 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node25', 'topic17', 'topic6'],
        output='screen',
        cwd=launch_file_dir
    )
    node26 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node26', 'topic17', 'topic13'],
        output='screen',
        cwd=launch_file_dir
    )
    node27 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node27', 'topic17', 'topic20'],
        output='screen',
        cwd=launch_file_dir
    )
    node28 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(1), str(100000), 'Node28', 'topic17', 'topic19'],
        output='screen',
        cwd=launch_file_dir
    )
    node29 = ExecuteProcess(
        cmd=['taskset', '-c', '0-7', bash_file_path, str(0), str(100000), 'Node29', 'topic29', 'topic17'],
        output='screen',
        cwd=launch_file_dir
    )

    # Create the launch description and add the actions
    ld = LaunchDescription()
    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)
    ld.add_action(node4)
    ld.add_action(node5)
    ld.add_action(node6)
    ld.add_action(node7)
    ld.add_action(node8)
    ld.add_action(node9)
    ld.add_action(node10)
    ld.add_action(node11)
    ld.add_action(node12)
    ld.add_action(node13)
    ld.add_action(node14)
    ld.add_action(node15)
    ld.add_action(node16)
    ld.add_action(node17)
    ld.add_action(node18)
    ld.add_action(node19)
    ld.add_action(node20)
    ld.add_action(node21)
    ld.add_action(node22)
    ld.add_action(node23)
    ld.add_action(node24)
    ld.add_action(node25)
    ld.add_action(node26)
    ld.add_action(node27)
    ld.add_action(node28)
    ld.add_action(node29)
    return ld