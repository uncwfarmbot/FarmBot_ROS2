"""Setup configuration for the farmbot_npk package."""
import glob
import os

from setuptools import find_packages, setup
package_name = 'farmbot_npk'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
        (os.path.join('share', package_name, 'etc'), glob.glob('etc/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Caddell',
    maintainer_email='jpc5858@uncw.edu',
    description='Driver and node for the JXCT 7-in-1 NPK soil probe on the UTM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'npk_node = farmbot_npk.npk_node:main',
            'probe_cli = farmbot_npk.probe_cli:main',
        ],
    },
)
