from setuptools import setup
import os #incluir
from glob import glob #incluir

package_name = 'my_first_subscriber'

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
    maintainer='manu', #modificar
    maintainer_email='manu@upv.edu.es', #modificar
    description='TODO: Creando un nodo que se subscribe al topic /odom', #modificar
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_subscriber_mod = my_first_subscriber.simple_subscriber_mod:main' #incluir
        ],
    },
)
