from setuptools import find_packages, setup

package_name = 'roboracer_drive_to_cmd_vel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarunbhyri9',
    maintainer_email='saitarunbhyri9@gmail.com',
    description='Simulation only: converts /drive (AckermannDriveStamped) to /cmd_vel (Twist) for Gazebo',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_to_cmd_vel = roboracer_drive_to_cmd_vel.drive_to_cmd_vel:main',
        ],
    },
)
