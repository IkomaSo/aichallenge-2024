from setuptools import find_packages, setup

import os 
from glob import glob

package_name = 'sanae_planner'

data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))

def package_files(directory, data_files):
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            data_files.append(("share/" + package_name + "/" + path, glob(path + "/**/*.*", recursive=True)))
    return data_files

data_files = package_files('trajectory/', data_files)


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ikoma',
    maintainer_email='ikoma-so@g.ecc.u-tokyo.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_optimizer = sanae_planner.trajectory_optimizer:main',
            'clothoid_traj_publisher = sanae_planner.clothoid_traj_publisher:main',
        ],
    },
)
