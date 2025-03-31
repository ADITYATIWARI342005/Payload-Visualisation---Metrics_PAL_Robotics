import os  
from ament_index_python.packages import get_package_share_directory  
from launch import LaunchDescription  
from launch.actions import DeclareLaunchArgument  
from launch.substitutions import LaunchConfiguration, Command  
from launch_ros.actions import Node  

def generate_launch_description():  
    # Get the path to the package directory for the 'ros2_pinocchio_node'  
    pkg_share = get_package_share_directory('ros2_pinocchio_node')  
    
    # Construct the path to the URDF file within the package  
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')  
    
    # Launch configuration for using simulation time  
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')  
    
    # Load the robot description from the URDF file using xacro  
    robot_description = Command(['xacro ', urdf_path])  
    
    # Declare launch argument for simulation time  
    declare_use_sim_time = DeclareLaunchArgument(  
        'use_sim_time',  
        default_value='false',  
        description='Use simulation clock if true'  
    )  
    
    # Node to handle the robot model  
    robot_model_node = Node(  
        package='ros2_pinocchio_node',  
        executable='robot_model_node',  
        name='robot_model_node',  
        output='screen',  
        parameters=[  
            {'robot_description': robot_description},  # Robot model description  
            {'use_sim_time': use_sim_time},            # Simulation time setting  
            {'base_link': 'base_link'},                # Base link for the robot  
            {'end_effector_link': 'tool0'},            # End effector link  
            {'collision_link1': 'link1'},              # First collision link  
            {'collision_link2': 'link6'}               # Second collision link  
        ],  
    )  
    
    # Node to publish the robot's state  
    robot_state_publisher_node = Node(  
        package='robot_state_publisher',  
        executable='robot_state_publisher',  
        name='robot_state_publisher',  
        output='screen',  
        parameters=[  
            {'robot_description': robot_description},  # Robot model description  
            {'use_sim_time': use_sim_time}              # Simulation time setting  
        ]  
    )  
    
    # Create and return the launch description with all defined nodes and arguments  
    return LaunchDescription([  
        declare_use_sim_time,                     # Adding simulation time argument  
        robot_state_publisher_node,               # Robot state publisher node  
        robot_model_node,                         # Robot model node  
    ])  