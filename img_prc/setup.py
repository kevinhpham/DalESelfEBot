from setuptools import find_packages, setup
from setuptools.command.install import install
import subprocess
import time

class PostInstallCommand(install):
    """Post-installation for installation mode."""
    def run(self):
        install.run(self)
        for _ in range(5):  # Retry up to 5 times
            try:
                subprocess.check_call(['pip', 'install', 'rembg', 'onnxruntime','opencv-python'])
                break
            except subprocess.CalledProcessError:
                time.sleep(5)  # Wait for 5 seconds before retrying

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
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'opencv-python',
        'numpy',
        'Pillow',
        'onnxruntime',
    ],
    cmdclass={
        'install': PostInstallCommand,
    },
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
