from setuptools import find_packages, setup

package_name = 'scout_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Milad',
    maintainer_email='milad@ufscar.br',
    description='Scout Mini control nodes for AMR course.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_square   = scout_control.move_square:main',
            'move_circle   = scout_control.move_circle:main',
            'sensor_reader = scout_control.sensor_reader:main',
            'obstacle_stop = scout_control.obstacle_stop:main',
            'challenge2_navigator = scout_control.challenge2_navigator:main',
            'maze_obstacle_avoidance = scout_control.maze_obstacle_avoidance:main',
            'obstacle_avoidance = scout_control.obstacle_avoidance:main',
        ],
    },
)
