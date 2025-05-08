from setuptools import setup
from glob import glob
import os

package_name = 'dale_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('dale_integration/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jarred',
    maintainer_email='jarred.g.deluca@student.uts.edu.au',
    description='Launch file to integrate multiple components for DalESelfieBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
