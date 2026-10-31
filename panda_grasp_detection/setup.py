import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'panda_grasp_detection'

package_data_files = []

algorithms_dir = os.path.join(package_name, 'algorithms')

for root, dirs, files in os.walk(algorithms_dir):
    for file in files:
        full_path = os.path.join(root, file)
        relative_path = os.path.relpath(full_path, package_name)
        package_data_files.append(relative_path)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: package_data_files,
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'graspnet_executor_node = panda_grasp_detection.graspnet_executor:main',
        ],
    },
)
