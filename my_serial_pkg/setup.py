from setuptools import find_packages, setup

package_name = 'my_serial_pkg'

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
    maintainer='makarand',
    maintainer_email='makarand.ericrobotics@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = my_serial_pkg.serial_node:main',
	    'serial_test = serial_test1.serial_test:main',
        ],
    },
)
