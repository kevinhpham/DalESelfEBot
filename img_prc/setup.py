from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'img_prc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    setup_requires=['setuptools'],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs'],
    zip_safe=True,
    maintainer='dingus',
    maintainer_email='dingus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'image_processor = img_prc.image_processor:main',
        	'image_sub = img_prc.image_sub:main',
            'image_client = img_prc.image_client:main',
                    ],
    },
)
