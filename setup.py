from setuptools import find_packages, setup

package_name = 'raspimouse_drone_ros2_aruco'

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
    maintainer='nakahira',
    maintainer_email='ba2vwbus13wind@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'controller = raspimouse_drone_ros2_aruco.controller:main',
             'controller_aruco = raspimouse_drone_ros2_aruco.controller_aruco:main',
             
        ],
    },
)
