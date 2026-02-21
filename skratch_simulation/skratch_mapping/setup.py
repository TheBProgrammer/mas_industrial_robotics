from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'skratch_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/maps/rc_arena_sim', glob('maps/rc_arena_sim/*')),
        ('share/' + package_name + '/maps/eval_arena', glob('maps/eval_arena/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anudeep',
    maintainer_email='anoodeep07@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
