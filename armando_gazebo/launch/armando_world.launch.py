from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
 
 
def generate_launch_description():

    # definizione argomenti GUI
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )
    # definizione argomenti pacchetto e modello URDF
    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='armando_gazebo')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='urdf/arm.urdf.xacro')
   
    # Percorso file RViz armando_display.rviz
    pkg_description_path = get_package_share_directory('armando_gazebo')
    rviz_config = os.path.join(pkg_description_path, "config", "rviz", "armando_display.rviz")
    
 
#~~~~~~~~~~~~~~~~~~~
    #2.a 
    # LaunchDescription per il mondo vuoto
    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'gz_args': ['-r ', 'empty.sdf'],
        }.items(),
    )
    # Nodo: spawn_node
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'armando', '-z', '0.5', '-unpause'],
    )
#~~~~~~~~~~~~~~~~~~~

    # Include per il file description.launch.py che genera l'URDF da xacro
    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),            # gli passo il pacchetto dell'urdf
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()  # gli passo la sottocartella
    )

 
#~~~~~~~~~~~~~~~~~~~
    #2.c
    #Nodo: spawn_joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    #Nodo: spawn_position_controller
    position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )
    controller_type = LaunchConfiguration('controller_type')
    # Nodo: spawn_controller
    spawn_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller_type, '--controller-manager', '/controller_manager'],
    )
#~~~~~~~~~~~~~~~~~~~
    #Nodo: spawn_trajectory_controller
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    # Definizione argomento controller_type
    controller_type_arg = DeclareLaunchArgument(
        name='controller_type',
        default_value='position_controller',
        description='Type of controller to use: position or trajectory',
    )



#~~~~~~~~~~~~~~~~~~~
    #3.c
    # Nodo: bridge_camera
    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args',
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )
#~~~~~~~~~~~~~~~~~~~
 
    # Avvia solo il controller selezionato tramite controller_type
    spawn_on_gen = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_node,
            on_exit=[spawn_controller],
        )
    )
 
    # Avvia i controller dopo lo spawn del robot
    spawn_controllers_on_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_node,
            on_exit=[spawn_joint_state_broadcaster],
        )
    )
    # Nodo: rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        gui_arg,
        package_arg,
        model_arg,
        controller_type_arg,
        empty_world_launch,
        description_launch_py,
        spawn_node,
        spawn_on_gen,
        spawn_controllers_on_spawn,
        rviz_node,
        bridge_camera
    ])