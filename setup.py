from setuptools import setup
from glob import glob
import os

package_name = 'tether_manager'
submodules = 'scripts'
setup(
    name=package_name,
    version='0.0.0',
    packages=[submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('lib/systemd/system', glob('systemd/*.service')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='estellearrc',
    maintainer_email='estelle.arricau@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tether_turn_counter = scripts.tether_turn_counter:main'
        ],
    },
)
