from setuptools import setup

package_name = 'sensor_switch_node'

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
            'sensor_switch_node = sensor_switch_node.sensor_switch_node:main'
        ],
    },
)
