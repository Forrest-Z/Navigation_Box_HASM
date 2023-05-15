"""
Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
Information classification: Confidential
This content is protected by international copyright laws.
Reproduction and distribution is prohibited without written permission.
"""

from setuptools import setup

PACKAGE_NAME = 'aiim_rospy'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='AIIM',
    author_email='info@aiim.ai',
    maintainer='AIIM',
    maintainer_email='info@aiim.ai',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: NavInfo Europe Confidential',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='AIIM common ROS functions.',
    license='NavInfo Europe Confidential',
    entry_points={
        'console_scripts': [
        ],
    },
)
