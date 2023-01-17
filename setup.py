from setuptools import setup

package_name = 'robot_chase'

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
    maintainer='aydenwang',
    maintainer_email='zw420@cam.ac.uk',
    description='University of Cambridge L310 Final Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "start_baddie = robot_chase.baddie:main",
            "start_police_simple = robot_chase.police_simple:main",
            "start_police_hungarian = robot_chase.police_hungarian:main",
            "start_stat = robot_chase.statistics:main",
        ],
    },
)
