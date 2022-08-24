from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Put executables here (ex missions)
            'simple_mission = drone_control.example_missions.simple_mission:main',
            'simple_duo_mission = drone_control.example_missions.simple_duo_mission:main',
            'camera_test_mission = drone_control.example_missions.camera_test_mission:main',
            'sitl = drone_control.sitl:main',
        ],
    },
)
