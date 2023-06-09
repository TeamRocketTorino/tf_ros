import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

from setuptools import setup

package_name = 'rocket_visual'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='franz',
    maintainer_email='francescopajero@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_broad = rocket_visual.static_broad:main',
            'broad_serial = rocket_visual.broad_serial:main',
            'broad_file = rocket_visual.broad_file:main'
        ],
    },
)
