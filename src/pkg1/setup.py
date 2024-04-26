from setuptools import find_packages, setup
from glob import glob
import os
 
 
package_name = 'pkg1'
 
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haocheng',
    maintainer_email='s3789513@student.rmit.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_mover = pkg1.test:main',
            'exploration_manager = pkg1.ExplorationManager:main',
            f'goinitial = {package_name}.goinitial:main',
            f'explore = {package_name}.explore_controller:main'
        ],
    },
)