from setuptools import find_packages, setup

package_name = 'ackermann_control'

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
    maintainer='pietro',
    maintainer_email='gelmini.pietro@gmail.com',
    description='Track following controllers',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = ackermann_control.pure_pursuit_node:main',
        ],
    },
)
