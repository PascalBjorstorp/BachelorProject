from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_stack'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Onboard drivers for VESC and sensors for F1TENTH vehicles.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'throttle_interpolator = f1tenth_stack.throttle_interpolator:main',
            'tf_publisher = f1tenth_stack.tf_publisher:main'
        ],
    },
)
