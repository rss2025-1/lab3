from setuptools import setup

package_name = 'safety_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['../launch/wall_follower.launch.py']),
        ('share/' + package_name + '/config', ['../config/wall_follower_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='skrem@mit.edu',
    description='Safety controller for RSS racecar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_controller = safety_controller.safety_controller:main',
        ],
    },
)
