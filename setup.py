from setuptools import find_packages, setup
import glob 
import os 

package_name = 'dynamic_obstacle_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='ronen',
    maintainer_email='raniti@umd.edu',
    description='Dynamic obstacle detection with dense optical flow',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = dynamic_obstacle_detector.main_node:main'
        ],
    },
)
