from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'datagates'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pa99',
    maintainer_email='patrice.wenzig@stud.hs-kempten.de',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'setup_datagate_pub = datagates.cam_data_pub_Node:main',
            'setup_datagate_sub = datagates.cam_subpub_Node:main'
        ],
    },
)
