from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tbot3_nav_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('../../worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('../../maps/*')),
    ],
    install_requires=['setuptools', 'flask', 'flask-cors', 'pandas', 'numpy'],
    zip_safe=True,
    maintainer='Coena',
    maintainer_email='coenadas.nr@gmail.com',
    description='TurtleBot3 navigation performance monitor with adaptive behavior.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'metrics_collector  = tbot3_nav_monitor.metrics_collector:main',
            'adaptive_behavior  = tbot3_nav_monitor.adaptive_behavior:main',
            'data_logger        = tbot3_nav_monitor.data_logger:main',
            'web_dashboard      = tbot3_nav_monitor.web_dashboard:main',
            'waypoint_patrol    = tbot3_nav_monitor.waypoint_patrol:main',
        ],
    },
)
