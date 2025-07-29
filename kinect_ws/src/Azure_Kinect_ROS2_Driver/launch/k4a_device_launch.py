#Karina Ruiz 
#Aylín Millan 
#Olivia Navarrete

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Obtener el directorio de share del paquete azure_kinect_ros2_driver
    k4a_driver_share_dir = get_package_share_directory('azure_kinect_ros2_driver')
    
    # Ruta al archivo URDF/Xacro de la Azure Kinect
    # Hemos confirmado que se llama 'azure_kinect.urdf.xacro'
    k4a_urdf_path = os.path.join(k4a_driver_share_dir, 'urdf', 'azure_kinect.urdf.xacro') 

    # Cargar el contenido del Xacro
    robot_description_content = Command(['xacro ', k4a_urdf_path])

    # Parámetros para el nodo del driver de Azure Kinect
    k4a_params = [
        {'point_cloud': True},
        {'rgb_point_cloud': False}, # Te recomiendo dejarlo en False para reducir carga de CPU
        {'depth_mode': 'NFOV_UNBINNED'}, # O 'NFOV_BINNED' si persisten los problemas de rendimiento
        {'color_resolution': '720P'}, # O '1080P' o '540P'
        {'fps': 15}, # O 15 si persisten los problemas de rendimiento
        {'imu_sync_depth': False}, # Sincroniza IMU con profundidad
        {'enable_rgb_to_depth_transform': True},
        {'qos_rgb_image': 2},
        {'qos_depth_image': 2},
        {'qos_imu': 2},
        {'qos_point_cloud': 2},
        {'sensor_sn': ""}, 
        {'tf_prefix': ""}, # Asegúrate de que no haya prefijo que cambie los nombres de los frames
        {'base_frame': 'camera_base'}, # Este es el frame base de tu URDF
    ]

    # Nodo del driver de Azure Kinect
    k4a_node = Node(
        package="azure_kinect_ros2_driver",
        executable="azure_kinect_node",
        name="k4a_ros2_node",
        output="screen",
        emulate_tty=True,
        parameters=k4a_params
    )

    # Nodo Robot State Publisher para publicar las TFs estáticas desde el URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    return [
        k4a_node,
        robot_state_publisher_node
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
