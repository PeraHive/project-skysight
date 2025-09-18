from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skysight_360'

setup(
    name=package_name,  # ✅ must match everywhere, no dashes
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jazz',
    maintainer_email='34100523+janithcyapa@users.noreply.github.com',
    description='YOLO-based vision module for Skysight 360',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = skysight_360.yolo_node:main',
            'preprocess_node = skysight_360.preprocess_node:main',

        ],
    },
)
