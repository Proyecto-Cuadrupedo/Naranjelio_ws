from setuptools import find_packages, setup

package_name = 'comms'

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
    maintainer='nicolukas',
    maintainer_email='correaln47@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_movements = comms.base_movements:main',
            'imu_plotter = comms.imu_plotter:main'
        ],
    },
)
