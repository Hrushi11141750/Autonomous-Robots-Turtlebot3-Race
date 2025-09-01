from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autorace_real'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # You can include other directories like config/ if needed
        # (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hrushikesh',
    maintainer_email='hj-235786@rwu.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_signal_real = autorace_real.start_signal_real:main',
            'follow_wall_real = autorace_real.follow_wall_real:main',
            'follow_road_real = autorace_real.follow_road_real:main',
            'enter_tunnel_real = autorace_real.enter_tunnel_real:main',
            'autorace_real = autorace_real.autorace_real:main',
            'avoid_obstacle_real_action = autorace_real.avoid_obstacle_real_action:main',
        ],
    },
)
