from setuptools import setup

package_name = 'target_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/target_localization.launch.py']),
        ('share/' + package_name + '/config',
         ['config/target_perception.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='echo',
    maintainer_email='echo@example.com',
    description='YOLO target detection and pose-synchronized map localization.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'target_camera_node = '
            'target_perception.camera_driver_node:main',
            'target_localization_node = '
            'target_perception.target_localization_node:main',
            'map_target_localizer_node = '
            'target_perception.map_target_localizer_node:main',
        ],
    },
)
