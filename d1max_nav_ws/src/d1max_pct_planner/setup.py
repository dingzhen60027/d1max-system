from glob import glob
from setuptools import find_packages, setup

package_name = 'd1max_pct_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='dndx',
    maintainer_email='dndx@localhost.localdomain',
    description='ROS 2 and CPU tomography integration for PCT Planner on D1 Max.',
    license='GPL-2.0-only',
    entry_points={
        'console_scripts': [
            'pct_build_tomogram = d1max_pct_planner.cpu_tomography:main',
            'pct_build_floor_map = d1max_pct_planner.floor_traversability:main',
            'pct_plan_offline = d1max_pct_planner.plan_offline:main',
            'pct_visualize = d1max_pct_planner.visualize:main',
        ],
    },
)
