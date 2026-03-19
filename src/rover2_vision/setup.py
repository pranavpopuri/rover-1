from setuptools import setup
import os
from glob import glob

package_name = 'rover2_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranavpopuri',
    maintainer_email='hipranav7@gmail.com',
    description='Object detection and navigation for Viam Rover 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector_node = rover2_vision.detector_node:main',
            'follower_node = rover2_vision.follower_node:main',
            'dashboard_node = rover2_vision.dashboard_node:main',
        ],
    },
)
