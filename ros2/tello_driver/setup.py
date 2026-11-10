from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tello_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instalar archivos launch (acepta cualquier script .py en la carpeta launch)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Instalar archivos de configuración config
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Lascurain',
    maintainer_email='glascurain4@users.noreply.github.com',
    description='ROS 2 Humble driver, vision and control nodes for color tracking with a DJI Tello.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Driver y teleop (base del paquete)
            'tello_driver_node = tello_driver.tello_driver_node:main',
            'tello_teleop_key  = tello_driver.tello_teleop_key:main',
            
            # NUEVOS NODOS (Visión y Control)
            'tello_vision_node = tello_driver.tello_vision_node:main',
            'tello_control_node = tello_driver.tello_control_node:main',
        ],
    },
)
