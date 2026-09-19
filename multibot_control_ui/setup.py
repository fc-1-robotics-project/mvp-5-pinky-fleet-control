from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'multibot_control_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'docs'),
         glob(os.path.join('docs', '*.md'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inyup',
    maintainer_email='inyupi@naver.com',
    description=(
        'ROS 2 fleet UI for two bridged robots with Nav2 and bottleneck '
        'coordination.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_ui = multibot_control_ui.control_ui:main',
        ],
    },
)
