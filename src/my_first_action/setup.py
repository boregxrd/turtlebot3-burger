from setuptools import setup
import os
from glob import glob

package_name = 'my_first_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')) #incluir
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manu',
    maintainer_email='manuelborregales12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_server = my_first_action.action_server:main', #incluir
            'action_client = my_first_action.action_client:main'   # incluir
        ],
    },
)


# source install/setup.bash
# export TURTLEBOT3_MODEL=burger
# ros2 launch turtlebot3_gazebo empty_world.launch.py


# source install/setup.bash
# ros2 launch my_first_action action_server.launch.py

# source install/setup.bash
# ros2 launch my_first_action action_client.launch.py

# ros2 run rqt_gui rqt_gui