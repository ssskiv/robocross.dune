from setuptools import find_packages, setup
from glob import glob
package_name = 'core'

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
    install_requires=['setuptools', 'serial'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ivanioshpa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'indicator_node = core.indicator_node:main',
            'goal_checker_node = core.goal_checker_node:main',
            'goal_sender_node = core.goal_sender_node:main',
        ],
    },
)
