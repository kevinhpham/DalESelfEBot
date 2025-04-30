from setuptools import find_packages, setup
# Removed: from setuptools.command.install import install
# Removed: import subprocess
# Removed: import time
import os # Keep os if needed for data_files path construction

# Removed the entire PostInstallCommand class definition
# class PostInstallCommand(install):
#     """Post-installation for installation mode."""
#     def run(self):
#         install.run(self)
#         # ... (removed pip install logic) ...

package_name = 'img_prc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Make sure the path construction is correct for your setup
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add other data files if needed (e.g., launch files)
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # setup_requires=['setuptools'], # Often not needed with modern pip/setuptools
    install_requires=[
        'setuptools', # Keep setuptools requirement
        'rclpy',      # ROS 2 Python library
        'sensor_msgs',# For Image messages
        # 'opencv-python', # REMOVED - Rely on ROS environment's OpenCV
        'numpy<2.0',  # Keep numpy, specify compatible version if needed
        'Pillow',     # Image library used by rembg/cv2 interactions
        'onnxruntime',# Dependency for rembg
        'rembg'       # Background removal library (will be installed via pip in venv)
        # Add other direct Python dependencies of your nodes here
    ],
    # Removed cmdclass argument:
    # cmdclass={
    #     'install': PostInstallCommand,
    # },
    zip_safe=True,
    maintainer='dingus', # TODO: Update maintainer info
    maintainer_email='dingus@todo.todo', # TODO: Update email
    description='TODO: Package description', # TODO: Update description
    license='TODO: License declaration', # TODO: Update license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processor = img_prc.image_processor:main',
            'image_sub = img_prc.image_sub:main',
            'image_client = img_prc.image_client:main',
        ],
    },
)