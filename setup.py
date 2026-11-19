from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'unicore_rtk_driver'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # install launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mingxi',
    maintainer_email='mzhou@uri.edu',
    description='ROS 2 driver for Unicore UM98x GNSS receivers (RTK rover)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unicore_rtk_driver_node = unicore_rtk_driver.driver_node:main',
            'unicore_rtk_probe = unicore_rtk_driver.tools.probe:main',
            'unicore_rtk_rover = unicore_rtk_driver.tools.rover:main',
        ],
    },
)