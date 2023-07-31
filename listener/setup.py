from setuptools import setup

package_name = 'listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fatih',
    maintainer_email='fatih@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frame_creator = listener.frame_creator:main',
            'navigate_to_tag = listener.navigate_to_tag:main',
            'listener = listener.listener:main',
        ],
    },
)
