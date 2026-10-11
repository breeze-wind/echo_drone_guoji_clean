from glob import glob
from setuptools import setup

package_name = 'electromagnet_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='echo',
    maintainer_email='echo@example.com',
    description='Serial ROS node for the electromagnet cargo release module.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'electromagnet_node = electromagnet_node.node:main',
    ]},
)
