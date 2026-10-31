from setuptools import setup

package_name = 'safety_filter'

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
    description='Attack-aware robust CBF safety filter (Proposed: STRICT/SOFT/MIN_RISK).',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Public entry point: the Proposed RCBF filter only.
            # The safety_filter.cbf_filter_node module is shipped because
            # ProposedV2Node subclasses it (perception / TF / publishing base);
            # no Standard-CBF or baseline console script is exposed here.
            'proposed_v2_filter_node = safety_filter.proposed_v2_filter_node:main',
        ],
    },
)
