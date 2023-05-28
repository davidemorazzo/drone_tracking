from setuptools import setup

package_name = 'drone_tracker_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'mocap_gazebo = mocap_gazebo.mocap_node_classic:main.'
        ],
    },
)