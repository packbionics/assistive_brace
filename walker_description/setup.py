import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'walker_description'

def glob_recursive(data_files, directory):
    files = glob(directory+'*.*')
    data_files.append((os.path.join('share', package_name, directory), files))
    subdirectories = glob(directory+'*/')
    if (subdirectories == []):
        return data_files
    else:
        for dir in subdirectories:
            glob_recursive(data_files, dir)
        return data_files


data_directories = ['urdf', 'launch', 'config']
extended_data_files = []
for directory in data_directories:
    glob_recursive(extended_data_files, directory)

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + extended_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anthony Brown',
    maintainer_email='anthonybrown0528@protonmail.com',
    description='Repository for URDF descriptions of a simple biped walking robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
