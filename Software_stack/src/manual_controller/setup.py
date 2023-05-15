# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import os
from setuptools import setup

package_name = 'manual_controller'

setup(
    name=package_name,
    version='2.0.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
    ],
    # This is important as well
    install_requires=['setuptools',
                      'pygame>=2.0.1'
                      ],
    zip_safe=True,
    author='AIIM',
    author_email='info@aiim.ai',
    maintainer='AIIM',
    maintainer_email='info@aiim.ai',
    keywords=['manual', 'control'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: NavInfo Europe Confidential',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Manual controller input.',
    license='NavInfo Europe Confidential',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'keyboard_publisher = manual_controller.keyboard_publisher:main',
            'joystick_publisher = manual_controller.joystick_publisher:main',
        ],
    },
)
