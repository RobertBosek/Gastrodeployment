from setuptools import setup

package_name = 'cube_tracker_node'

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
    maintainer='Robert Bosek',
    maintainer_email='robert.bosek@stud.uni-regensburg.de',
    description='TODO: Package description',
    license='CC BY-SA 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_tracker_node = cube_tracker_node.cube_tracker_node:main'
        ],
    },
)
