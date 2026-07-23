from glob import glob
from setuptools import setup

package_name = 'swarm_sim'
setup(
    name=package_name, version='0.2.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Ali', maintainer_email='you@example.com',
    description='Multi-robot Gazebo bringup with per-milestone coordinators',
    license='MIT',
    entry_points={'console_scripts': [
        # M1: each robot drives to its own fixed goal, no inter-robot coupling
        'independent_goals = swarm_sim.independent_goals:main',
        # M2: leader walks a waypoint route, followers hold body-frame offsets
        'coordinator = swarm_sim.coordinator:main',
    ]},
)
