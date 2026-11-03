from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mppi_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch 파일 설치
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # yaml 설정 파일 설치
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # RViz layout
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),

        # MPPI team interface document
        (os.path.join('share', package_name),
            ['README_COSTMAP_INTERFACE.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoon',
    maintainer_email='answeet@naver.com',
    description='Structural map, static/local Nav2 costmaps and TF interface for the MPPI team',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'interface_report = mppi_nav.interface_report:main',
            'clear_on_relocalize = mppi_nav.clear_on_relocalize:main',
            'self_filter = mppi_nav.self_filter:main',
        ],
    },
)
