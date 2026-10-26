from glob import glob
from setuptools import find_packages, setup
package_name='roboracer_lidar_centerline'
setup(name=package_name,version='0.2.0',packages=find_packages(exclude=['test']),
 data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),('share/'+package_name+'/launch',glob('launch/*.launch.py')),('share/'+package_name+'/config',glob('config/*.yaml'))],
 install_requires=['setuptools','numpy'],zip_safe=True,maintainer='Dhanush Dommadi',maintainer_email='dhanush@example.com',description='Separated wall/cone centerline extraction and Pure Pursuit with visualization.',license='Apache-2.0',
 entry_points={'console_scripts':['wall_centerline = roboracer_lidar_centerline.wall_centerline_node:main','pure_pursuit = roboracer_lidar_centerline.pure_pursuit_node:main']})
