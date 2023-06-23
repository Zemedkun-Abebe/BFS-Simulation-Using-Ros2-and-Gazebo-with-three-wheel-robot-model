from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('lib', package_name), ['scripts/path_planner.py']),
        (os.path.join('lib', package_name), ['scripts/test.py']),
        (os.path.join('lib', package_name), ['scripts/Print_name.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zemedkun',
    maintainer_email='zemedkunabebe4@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
