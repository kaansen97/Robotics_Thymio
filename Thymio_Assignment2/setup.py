from setuptools import setup
from glob import glob
package_name = 'Assignment2'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',glob('launch/*.launch.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics23',
    maintainer_email='robotics23@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assignment2_task1 = Assignment2.assignment2_task1:main',
            'assignment2_task3 = Assignment2.assignment2_task3:main',
            'assignment2_task5 = Assignment2.assignment2_task5:main'
        ],
    },
)

