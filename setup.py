import os
from glob import glob
from setuptools import setup

package_name = 'adcs_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alfredo',
    maintainer_email='amamanisai@unsa.edu.pe',
    description='Package to ADCS algorithm design',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'controller = adcs_control.control_node:main',
        	'pwm_publisher = adcs_control.pwm_pub:main',
        	'gui = adcs_control.gui_node:main',
        ],
    },
)
