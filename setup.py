from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'respeaker_ros'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)],
        ),
        (os.path.join('share', package_name), ['package.xml', 'requirements.txt']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuki Furuta',
    maintainer_email='furushchev@jsk.imi.i.u-tokyo.ac.jp',
    description='ROS2 package for Respeaker Mic Array',
    license='Apache',
    entry_points={
        'console_scripts': [
            'respeaker_node = respeaker_ros.respeaker_node:main',
            'speech_to_text = respeaker_ros.speech_to_text:main',
            'respeaker_gencfg = respeaker_ros.respeaker_gencfg:main_cli',
        ],
    },
)
