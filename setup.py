from setuptools import find_packages, setup

package_name = 'robot_waiter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_waiter']),
        ('share/robot_waiter', ['package.xml']),
        ('share/robot_waiter/launch', ['launch/robot_waiter_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='myros',
    maintainer_email='myros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
               'robot_waiter_node = robot_waiter.robot_waiter_node:main',
        ],
    },
)
