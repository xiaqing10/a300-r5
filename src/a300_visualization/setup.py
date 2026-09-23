from setuptools import find_packages, setup

package_name = 'a300_visualization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/status_light.launch.py']),
        ('lib/' + package_name, ['scripts/status_light']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='A300 Development',
    maintainer_email='dev@a300.local',
    description='Gazebo 3D visualization for the a300 safety status.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_light = a300_visualization.status_light:main',
        ],
    },
)
