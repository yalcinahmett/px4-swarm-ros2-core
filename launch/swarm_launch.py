from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    swarm_size = 3 

    for i in range(swarm_size):  # Changed from range(1, swarm_size + 1)
        drone_id = i  # Now starts from 0
        
        node = Node(
            package='px4-swarm-ros2-core', 
            executable='swarm',
            name=f'swarm_{drone_id}',
            namespace=f'px4_{drone_id}' if drone_id > 0 else '',  # No namespace for drone 0
            output='screen',
            parameters=[
                {'drone_id': drone_id} 
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        )
        ld.add_action(node)

    return ld