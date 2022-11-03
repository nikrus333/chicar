from setuptools import setup

package_name = 'chicar'

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
    maintainer='nik',
    maintainer_email='nikrus333@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_serial = chicar.cmd_serial:main',
            'teleop_twist_keyboard = teleop_twist_keyboard:main',
            'wheel_odom = chicar.wheel_odom:main'
        ],
    },
)
