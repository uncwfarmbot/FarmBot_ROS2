"""Setup configuration for the farmbot_agronomy package."""
import glob
import os

from setuptools import find_packages, setup
package_name = 'farmbot_agronomy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Caddell',
    maintainer_email='jpc5858@uncw.edu',
    description='Automated per-plant water and nitrogen controllers for the ROS2 Farmbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crop_controller = farmbot_agronomy.crop_controller:main',
        ],
    },
)
