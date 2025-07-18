from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'py_socket'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giuliobesi',
    maintainer_email='giulio.besi@unimore.it',
    description='(Python) Package for socket data receiver and reader',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reader = py_socket.read_data_node:main',
        ],
    },
)
