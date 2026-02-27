from setuptools import setup
from glob import glob
import os

package_name = 'lesson_urdf'
mesh_files = [f for f in glob('meshes/**/*', recursive=True) if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),

        ('share/' + package_name + '/config', glob('config/*.yaml')),

        ('share/' + package_name + '/meshes', mesh_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='olmer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_gui = lesson_urdf.joint_gui:main',
        ],
    },
)