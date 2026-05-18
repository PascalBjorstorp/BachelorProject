from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'f1tenth_gym_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=[
        package_name,
        'f1tenth_gym',
        'f1tenth_gym.*',
    ]),
    package_data={
        package_name: [
            '../maps/*.yaml',
            '../maps/*.png',
            '../maps/*.pgm',
        ],
        'f1tenth_gym': [
            'envs/rendering/rendering.yaml',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (f'share/{package_name}/maps', [
            os.path.join('maps', f)
            for f in os.listdir('maps')
            if f.endswith('.yaml') or f.endswith('.png') or f.endswith('.pgm')
        ]),
    ],
    install_requires=[
        'setuptools',
        'transforms3d',
        'gymnasium',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Billy Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='Bridge for using f1tenth_gym in ROS2 Jazzy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gym_bridge = f1tenth_gym_ros.gym_bridge:main'
        ],
    },
    python_requires='>=3.10',
)
