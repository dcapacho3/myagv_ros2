import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'myagv_ros2_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
   (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sara',
    maintainer_email='sara@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'myagv_odom = myagv_ros2_odometry.myagvodom:main',
        'myagv_encoder = myagv_ros2_odometry.myagvencoderpublish:main',
        'myagv_bringup = myagv_ros2_odometry.myagv_bringup:main',
        'myagv_odometry = myagv_ros2_odometry.myagvodomfix:main',
        'myagv_imu = myagv_ros2_odometry.myagvimupublish:main',


        
        ],
    },
)
