from setuptools import setup
from glob import glob
import os

package_name = 'jetson_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.motorDrivers', package_name + '.Driver'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = jetson_camera.publisher_node:main',
            'subscriber_node = jetson_camera.subscriber_node:main',
            'processing_node = jetson_camera.processing_node:main',
            'QR_node = jetson_camera.QR_node:main',
            'main_qr = jetson_camera.main_qr:main',
            'application_node = jetson_camera.application_node:main',
            'face_tracking_sub = jetson_camera.face_tracking_sub:main',
            'face_detection_recognition = jetson_camera.face_detection_recognition:main',
            'ToF_sensor = jetson_camera.ToF_sensor:main',
            'tracking = jetson_camera.tracking:main',
            'object_detection = jetson_camera.object_detection:main',
            'encoder_node = jetson_camera.encoder_node:main',
            'route_plotter = jetson_camera.plot_route:main',
            'movement_node = jetson_camera.movement_node:main',
            'imu_node = jetson_camera.imu_node:main',
            'square_driver_node = jetson_camera.square_driver_node:main',
            'algorithm = jetson_camera.algorithm_node:main',
            'grid_mapping_node = jetson_camera.grid_mapping_node:main',
            'algoritm_node = jetson_camera.algorithm_node:main',
        ],
    },
)
