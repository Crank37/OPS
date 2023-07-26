#!/usr/bin/env python3

from setuptools import setup

package_name = 'rqt_joy'
setup(
    name=package_name,
    version='1.0.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/VirtualJoy.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('lib/' + package_name, ['scripts/rqt_joy'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Patrick Wiesen',
    maintainer='Patrick Wiesen',
    maintainer_email='wiesen@fh-aachen.de',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'rqt_joy provides a GUI plugin for a virtual joypad to control a robot using Joy messages.'
    ),
    license='BSD',
    scripts=['scripts/rqt_joy'],
)