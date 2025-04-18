from setuptools import setup
import os
from glob import glob

package_name = 'my_first_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
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
            'movement_server = my_first_service.movement_server:main',
            'movement_client = my_first_service.movement_client:main',
            'circle_server = my_first_service.circle_server:main',
            'circle_movement = my_first_service.circle_movement:main',
        ],
    },
)
