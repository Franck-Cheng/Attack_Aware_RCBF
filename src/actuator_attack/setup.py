from setuptools import setup

package_name = 'actuator_attack'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Franck Cheng',
    maintainer_email='franck.cheng@example.com',
    description='Post-filter actuator-command injection node (demo attacker).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'actuator_attack_node = actuator_attack.actuator_attack_node:main',
        ],
    },
)
