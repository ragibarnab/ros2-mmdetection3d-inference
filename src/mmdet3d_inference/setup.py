from setuptools import find_packages, setup

package_name = 'mmdet3d_inference'

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
    maintainer='Ragib Arnab',
    maintainer_email='rae3840924@gmail.com',
    description='MMDetection3D Inference on ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_inferencer_node = mmdet3d_inference.lidar_inferencer_node:main'
        ],
    },
)
