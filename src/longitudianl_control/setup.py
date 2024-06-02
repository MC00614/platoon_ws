from setuptools import setup

package_name = 'longitudianl_control'

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
    maintainer='MC00614',
    maintainer_email='12190588@inha.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'distance_control = longitudianl_control.main_distance:main',
            'velocity_control = longitudianl_control.main_velocity:main'
        ],
    },
)
