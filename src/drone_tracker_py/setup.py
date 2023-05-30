from setuptools import setup
import os
from glob import glob

package_name = 'drone_tracker_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='judocero',
    maintainer_email='morazzo.davide@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_tracker_py = drone_tracker_py.tracker:main',
            'mocap_gazebo = drone_tracker_py.mocap_node_classic:main',
            'camera_sim = drone_tracker_py.CameraSim:main'
        ],
    },
)
