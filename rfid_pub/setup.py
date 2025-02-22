from setuptools import setup
import os
from glob import glob

package_name = 'rfid_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='ali@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rfid_reader_pub = rfid_pub.rfid_reader_pub:main',
            'rfid_reader_string_pub = rfid_pub.rfid_reader_string_pub:main',
            'inventory_bag_recorder = rfid_pub.inventory_bag_recorder:main',
            'bag2csv_node = rfid_pub.bag2csv:main',
            'bag2deneme_node = rfid_pub.bag_deneme:main',
            'test = rfid_pub.test:main'
        ],
    },
)
