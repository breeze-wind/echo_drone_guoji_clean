from setuptools import find_packages, setup

package_name = 'flight_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/mavros_adapter.yaml']),
        ('share/' + package_name + '/launch',
            [
                'launch/mavros_adapter.launch.py',
                'launch/mavros_state.launch.py',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sfx',
    maintainer_email='sfx@example.com',
    description='MAVROS adapter for Echo Drone flight-control interfaces.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mavros_adapter_node = flight_control.mavros_adapter_node:main',
        ],
    },
)
