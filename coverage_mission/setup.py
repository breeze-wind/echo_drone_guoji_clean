from glob import glob
from setuptools import setup

package_name = 'coverage_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='echo',
    maintainer_email='echo@example.com',
    description='Standalone serpentine target-search mission.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coverage_mission_node = coverage_mission.coverage_mission_node:main',
        ],
    },
)
