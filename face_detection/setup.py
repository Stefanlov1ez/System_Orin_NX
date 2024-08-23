from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'face_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsy',
    maintainer_email='lsy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'yunet_detection = face_detection.Yunet_Detection:main',
            'facefollow_ctrl = face_detection.facefollow_ctrl:main',
        ],
    },
)
