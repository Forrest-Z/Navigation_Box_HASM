from setuptools import setup

package_name = 'trajectory_error_calculator'

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
    maintainer='omax',
    maintainer_email='m.m.m.m.abdelhady@tue.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_error_calculator_node = trajectory_error_calculator.trajectory_error_calculator_node:main'
        ],
    },
)
