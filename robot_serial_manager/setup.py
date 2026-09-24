from setuptools import find_packages, setup

package_name = 'robot_serial_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sfx',
    maintainer_email='sfx@example.com',
    description='Serial device inventory and health reporting.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_serial_manager_node = '
            'robot_serial_manager.robot_serial_manager_node:main',
        ],
    },
)
