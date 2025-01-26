from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'first_order_boustrophedon_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdas, Dennis Liu', 
    maintainer_email='jnaneshwar.das@gmail.com, dennis48161025@gmail.com',
    description='First-order boustrophedon navigation with PD control',
    license='CC0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boustrophedon_controller = first_order_boustrophedon_navigator.boustrophedon_controller:main'
        ],
    },
) 