from glob import glob

from setuptools import find_packages, setup

package_name = 'crack_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools', 'numpy', 'onnxruntime==1.31.0'],
    zip_safe=True,
    maintainer='Aritra Das',
    maintainer_email='aritradas080@gmail.com',
    description='U-Net crack segmentation with RGB boxes and depth projection for RViz.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'crack_bbox_node = crack_detection.crack_bbox_node:main',
        ],
    },
)
