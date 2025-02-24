from setuptools import setup

package_name = 'commander_pkg'

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
    maintainer='mzs',
    maintainer_email='mehmetzekisildir@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_commander_node=commander_pkg.simple_commander:main',
            'multiwp_commander_node=commander_pkg.multiwp_commander:main',
            'main_workflow_node=commander_pkg.main_workflow:main'
        ],
    },
)
