from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'skratch_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thebrobot',
    maintainer_email='bhaveshgandhi843@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_metrics_logger = skratch_navigation.nav_metrics_logger:main',
            'nav_metrics_logger_standalone = '
            'skratch_navigation.nav_metrics_logger_standalone:main',
            'save_poses = skratch_navigation.save_poses:main',
            'navigate = skratch_navigation.navigate_to_pose:main',
            'save_poser_tf = skratch_navigation.tf_static_saver:main',
            'barrier_tape_mock = '
            'skratch_navigation.barrier_tape_mock_node:main',
        ],
    },
)
