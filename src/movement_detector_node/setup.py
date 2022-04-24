from setuptools import setup

package_name = 'movement_detector_node'

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
    maintainer='vigitia',
    maintainer_email='robert.bosek@stud.uni-regensburg.de',
    description='TODO: Package description',
    license='CC BY-SA 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_detector_node_on = movement_detector_node.movement_detector_node:main',
            'movement_detector_node_off = movement_detector_node.movement_detector_node_off:main',
        ],
    },
)
