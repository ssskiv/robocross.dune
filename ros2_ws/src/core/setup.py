from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'core'

def generate_data_files(share_path, dir):
    data_files = []
    for path, _, files in os.walk(dir):
        list_entry = (os.path.dirname(os.path.dirname(share_path)) + '/' + path,
                      [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files

data_files = [] 
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files += generate_data_files('share/' + package_name + '/static/', 'map-server/dist/')
data_files += generate_data_files('share/' + package_name + '/launch/', 'launch/')
data_files += generate_data_files('share/' + package_name + '/worlds/', 'worlds/')
data_files += generate_data_files('share/' + package_name + '/config/', 'config/')
data_files += generate_data_files('share/' + package_name + '/resource/', 'resource/')
data_files += generate_data_files('share/' + package_name + '/train/', 'train/')


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
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
            'yolo_detect = core.yolo_detect:main',
            'goal_checker_node = core.goal_checker_node:main',
            'goal_sender_node = core.goal_sender_node:main',
        ],
    },
)
