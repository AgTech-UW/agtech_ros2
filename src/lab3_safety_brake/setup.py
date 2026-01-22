from setuptools import find_packages, setup

package_name = 'lab3_safety_brake'

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
    maintainer='root',
    maintainer_email='ge.dyck@uwinnnipeg.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'safety_stop = lab3_safety_brake.safety_stop:main',
            'safety_turn_key = lab3_safety_brake.safety_turn_key:main',
        ],
    },
)
