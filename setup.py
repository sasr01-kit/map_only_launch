from setuptools import find_packages, setup

package_name = 'map_only_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register the package with ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install the launch folder
        ('share/' + package_name + '/launch', ['launch/map_server_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saadhvi',
    maintainer_email='uswup@student.kit.edu',
    description='Minimal package to launch a static map server for subscribing to /map',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_publisher = map_only_launch.pose_publisher:main',

        ],
    },
)
