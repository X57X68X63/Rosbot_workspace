from setuptools import find_packages, setup

package_name = 'bug2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            f'ph_controller:{package_name}.placeholder_controller:main',
            f'ph_estimator:{package_name}.placegolder_estimator:main'
        ],
    },
)
