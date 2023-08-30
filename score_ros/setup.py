from setuptools import setup
import os
from glob import glob

 
package_name = 'score_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luigi',
    maintainer_email='lf4943s@fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'score_ros = score_ros.score_ros:main',
            'april_tag_scorer = score_ros.april_tag_scorer:main',
        ],
    },
)
