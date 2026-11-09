from glob import glob

from setuptools import find_packages, setup

package_name = 'terrain_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Yamin',
    maintainer_email='myamin.bcss19@iba-suk.edu.pk',
    description='IMU roughness (fast/slow band ratio of vertical acceleration) on /terrain/imu_roughness.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_roughness_node = terrain_imu.imu_roughness_node:main',
        ],
    },
)
