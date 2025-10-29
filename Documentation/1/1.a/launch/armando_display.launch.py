from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    
    #Trova il percorso del pacchetto
    pkg_description_path = get_package_share_directory('armando_description')
    
    #Definisce i percorsi dei file
    urdf_path = os.path.join(pkg_description_path, "urdf", "arm.urdf") 
    #1.b Percorso file RViz armando_display.rviz
    rviz_config = os.path.join(pkg_description_path, "config", "rviz", "armando_display.rviz")

    #Caricamento URDF (come stringa XML)
    try:
        with open(urdf_path, 'r') as inf:
            robot_description_content = inf.read()
    except FileNotFoundError:
        print(f"ERRORE GRAVE: File URDF non trovato nel percorso: {urdf_path}")
        return LaunchDescription([]) # Debug

    robot_description_param = {'robot_description': robot_description_content}
    
#~~~~~~~~~~~~~~~~~~~~
    #1.a
    # Nodo: robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # Nodo: joint_state_publisher 
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    #Nodo: rviz2 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config], 
    )
#~~~~~~~~~~~~~~~~~~~~
    #Avvio dei nodi
    return LaunchDescription([
        robot_state_publisher_node,  
        joint_state_publisher_node,
        rviz_node
    ])
