from setuptools import find_packages, setup

package_name = 'spr_remote_control'

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
    maintainer='b',
    maintainer_email='2786573564@qq.com',
    description='Remote control package for the sentry robot: interchangeable input sources (keyboard / DR16 serial DBUS) publishing unified cmd_vel and gimbal_cmd',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'keyboard_remote = spr_remote_control.keyboard_remote:main',
            'rc_serial_remote = spr_remote_control.rc_serial_remote:main',
        ],
    },
)
