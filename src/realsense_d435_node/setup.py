from setuptools import setup

package_name = 'realsense_d435_node'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_d435_node = realsense_d435_node.realsense_d435_node:main'
        ],
    },
)
