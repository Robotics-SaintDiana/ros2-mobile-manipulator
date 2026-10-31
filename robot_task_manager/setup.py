from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'tasks'), glob('tasks/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anemone_akiyui',
    maintainer_email='anemone_akiyui@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager = robot_task_manager.task_manager_node:main',
        ],
    },
)
