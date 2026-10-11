from setuptools import setup

package_name = 'coverage_search'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='echo',
    maintainer_email='echo@example.com',
    description='Generate and preview lawnmower coverage waypoints.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coverage_waypoint_node = coverage_search.coverage_waypoint_node:main',
        ],
    },
)
