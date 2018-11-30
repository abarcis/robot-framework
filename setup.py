#! /usr/bin/env python

from setuptools import setup
from setuptools import find_packages

package_name = 'robot_framework'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pyquaternion',
    ],
    zip_safe=True,
    author='Agata Gniewek',
    author_email='agata.gniewek@aau.at',
    maintainer='Agata Gniewek',
    maintainer_email='agata.gniewek@aau.at',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description="A framework for simulation and basic visualization of robots",
    license='TODO',
    tests_require=['pytest'],
)
