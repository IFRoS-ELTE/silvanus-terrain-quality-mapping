from glob import glob

from setuptools import find_packages, setup

package_name = 'rgb_terrain'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Yamin',
    maintainer_email='myamin.bcss19@iba-suk.edu.pk',
    description='RGB terrain classifier publishing the surface type on /ground/rgb_terrain.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'terrain_node = rgb_terrain.terrain_node:main',
        ],
    },
)
