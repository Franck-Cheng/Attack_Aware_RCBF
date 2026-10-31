from setuptools import find_packages, setup

package_name = 'scan_range_limiter'

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
    maintainer='feifei',
    maintainer_email='feifei@todo.todo',
    description='Protocol V2 scan range limiter (defender perception 1.5 m)',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'scan_range_limiter_node = scan_range_limiter.scan_range_limiter_node:main',
        ],
    },
)
