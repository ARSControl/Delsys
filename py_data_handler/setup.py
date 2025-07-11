from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'py_data_handler'

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
    description='(Python) Data handler package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handler = py_data_handler.data_handler_node:main',
        ],
    },
)
