from setuptools import setup

package_name = 'rvr_pi_4'

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
    maintainer='BoofRemaster',
    maintainer_email='as.spackman25@gmail.com',
    description='A package to control an RVR and Raspberry Pi 4 over ROS2 Framework',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'echo = rvr_pi_4.echo_rvr:main',
            'drive = rvr_pi_4.drive_rvr:main'
        ],
    },
)
