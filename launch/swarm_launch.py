from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    swarm_size = 3 

    for i in range(1, swarm_size + 1):
        drone_id = i
        
        node = Node(
            package='px4-swarm-ros2-core', 
            executable='swarm',
            name=f'swarm_{drone_id}',
            namespace=f'px4_{drone_id}',
            output='screen',
            parameters=[
                {'drone_id': drone_id} 
            ],
            arguments=['--ros-args', '--log-level', 'info'] 
        )
        ld.add_action(node)

    return ld