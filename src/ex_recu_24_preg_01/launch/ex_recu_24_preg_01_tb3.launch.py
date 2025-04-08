import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# Obtener el modelo desde la variable de entorno
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    # Ruta al paquete personalizado
    pkg_share = FindPackageShare(package='ex_recu_24_preg_01').find('ex_recu_24_preg_01')

    # Rutas a los archivos del mundo y el URDF
    world_file_name = 'world/burger_office.world'
    urdf_file_name = f'urdf/turtlebot3_{TURTLEBOT3_MODEL}.urdf'

    # Ruta completa al archivo del mundo y al URDF
    world = os.path.join(pkg_share, world_file_name)
    urdf = os.path.join(pkg_share, urdf_file_name)

    # Configuración de tiempo simulado
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Añadir ruta del modelo personalizado al path de modelos de Gazebo
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Rutas a paquetes estándar
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Comandos de lanzamiento
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Si quisieras añadir un spawn del robot personalizado, lo harías aquí.
    # Ejemplo comentado:
    # spawn_robot_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'turtlebot3', '-file', urdf, '-x', '0', '-y', '0'],
    #     output='screen'
    # )

    # Crear LaunchDescription y añadir acciones
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(spawn_robot_cmd)

    return ld
