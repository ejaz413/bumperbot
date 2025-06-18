from setuptools import find_packages, setup

package_name = 'bumperbot_py'

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
    maintainer='ejaz',
    maintainer_email='ejaz@gnu.ac.krm',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = bumperbot_py.simple_publisher:main',
            'simple_subscriber = bumperbot_py.simple_subscriber:main',
            'simple_lifecycle_node = bumperbot_py.simple_lifecycle_node:main',
            'simple_qos_publisher = bumperbot_py.simple_qos_publisher:main',
            'simple_qos_subscriber = bumperbot_py.simple_qos_subscriber:main',
        ],
    },
)
