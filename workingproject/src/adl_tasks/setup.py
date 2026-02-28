import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'adl_tasks'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'srv'),
            glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oliver-Gaston',
    maintainer_email='gaston13@usf.edu',
    description='ADL Task Planning for Kinova Gen3 (CSE Project)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ex_move = adl_tasks.ex_move:main', # example file: spawn obj
            # Layer 1: Scene
            'scene_static = adl_tasks.scene_static:main', 
            'scene_from_vision = adl_tasks.scene_from_vision:main', # real vision to MoveIt
            # Layer 2: Vision
            'vision_stub = adl_tasks.vision_stub:main',           # temp fake vision
            'vision_apriltag = adl_tasks.vision_apriltag:main',   # real cam + OpenCV + detection
            # Layer 3: ADL Tasks
            'pick_dropped_bottle = adl_tasks.pick_dropped_bottle:main',
            'flip_light_switch = adl_tasks.flip_light_switch:main',
            'clear_table = adl_tasks.clear_table:main',
            'give_medication = adl_tasks.give_medication:main',
            # layer 4: UI
            'adl_ui = adl_tasks.adl_ui:main',
        ],
    },
)
