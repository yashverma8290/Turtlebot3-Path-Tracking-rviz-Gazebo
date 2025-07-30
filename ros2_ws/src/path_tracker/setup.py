from setuptools import setup
import os
from glob import glob

package_name = 'path_tracker'



setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'matplotlib'],
    zip_safe=True,
    maintainer='yashverma',
    maintainer_email='yashverma@todo.todo',
    description='Path smoothing and trajectory tracking for TurtleBot3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main = path_tracker.main:main',
            'controller_node = path_tracker.ros_controller:main',

        ],
    },
)
