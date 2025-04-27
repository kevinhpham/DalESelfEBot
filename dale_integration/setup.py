from setuptools import setup

package_name = 'dale_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dale_launch.py']),  # we’ll create this next
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
