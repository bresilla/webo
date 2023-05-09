from glob import glob
from setuptools import setup
import os

package_name = 'webots_ros2_pioneer3at'
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
#params
data_files.append(('share/' + package_name + '/params', glob('params/*')))
#maps
data_files.append(('share/' + package_name + '/maps', glob('maps/*')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        "webots_ros2_pioneer3at/canbus",
        "webots_ros2_pioneer3at/path_server",
        "webots_ros2_pioneer3at/weeding",
        "webots_ros2_pioneer3at/utils",
        "webots_ros2_pioneer3at/detect",
        "webots_ros2_pioneer3at/track",
    ],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bresilla',
    maintainer_email='trim.bresilla@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webots_ros2_pioneer3at = webots_ros2_pioneer3at.webots_ros2_pioneer3at:main',
            'topic_remapper = webots_ros2_pioneer3at.topic_remapper:main',
            'navfix_can = webots_ros2_pioneer3at.canbus.navfix_can:main',
            'pathserver_utm = webots_ros2_pioneer3at.path_server.utm:main',
            'pathserver_utm2 = webots_ros2_pioneer3at.path_server.utm2:main',

            'instruct = webots_ros2_pioneer3at.path_server.instruct:main',


            'save_images = webots_ros2_pioneer3at.utils.save_images:main',
            'image_player = webots_ros2_pioneer3at.utils.image_player:main',

            'detect_pixel = webots_ros2_pioneer3at.detect.detect_pixel:main',
            'detect_blob = webots_ros2_pioneer3at.detect.detect_blob:main',
            'detect_yolo = webots_ros2_pioneer3at.detect.detect_yolo:main',

            'track_nofair = webots_ros2_pioneer3at.track.track_nofair:main',
            'track_deepsort = webots_ros2_pioneer3at.track.track_deepsort:main',
            'track_nofair_old = webots_ros2_pioneer3at.track.track_nofair_old:main',
            'track_deepsort_old = webots_ros2_pioneer3at.track.track_deepsort_old:main',

            'detract_blob = webots_ros2_pioneer3at.weeding.detract_blob:main',
            'detract_yolo = webots_ros2_pioneer3at.weeding.detract_yolo:main',
            'detract_blob_all = webots_ros2_pioneer3at.weeding.detract_blob_all:main',
            'detract_yolo_all = webots_ros2_pioneer3at.weeding.detract_yolo_all:main',
        ],
        'launch.frontend.launch_extension': [
            'launch_ros = launch_ros'
        ]
    },
)
