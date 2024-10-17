from setuptools import find_packages, setup

package_name = 'sys_id'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ikoma',
    maintainer_email='ikoma@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'longitude_test = sys_id.longitude_test:main',
            'slip_detector = sys_id.slip_detector:main'
        ],
    },
)
