from setuptools import find_packages, setup
from glob import glob
package_name = 'interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name+ '/config', glob('config/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymavlink', 'serial'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ivanioshpa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_serial = interfaces.arduino_serial_both_node:main',
            'mamba_serial = interfaces.mamba_serial_node:main',
            'mamba_mavlink = interfaces.mamba_mavlink_node:main',
            'rosbag_recorder_node = interfaces.rosbag_recorder_node:main',
        ],
    },
)
