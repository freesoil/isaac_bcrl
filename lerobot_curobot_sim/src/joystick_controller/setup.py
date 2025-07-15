from setuptools import setup
import os
from glob import glob

package_name = 'joystick_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Joystick controller for SO-ARM 100 robot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_controller_node = joystick_controller.joystick_controller:main',
            'keyboard_joystick = joystick_controller.keyboard_joystick:main',
        ],
    },
) 