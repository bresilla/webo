from glob import glob
from setuptools import setup
import os

package_name = 'webo'

data_files = []
#resources
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/resource', glob('resource/*')))
data_files.append(('share/' + package_name, ['package.xml']))

#launch files
data_files.append((os.path.join('share', package_name), glob('launch/*.py')))
#proto/world files
data_files.append(('share/' + package_name + '/protos', glob('protos/*.proto')))
data_files.append(('share/' + package_name + '/protos', glob('protos/*.stl')))
data_files.append(('share/' + package_name + '/worlds', glob('worlds/*')))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bresilla',
    maintainer_email='trim.bresilla@gmail.com',
    escription='TODO: Package description',
    license='TODO: License declaration',
    ests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webo_driver = webo.webo_driver:main',
        ],
        'launch.frontend.launch_extension': [
            'launch_ros = launch_ros'
        ]
    },
)
