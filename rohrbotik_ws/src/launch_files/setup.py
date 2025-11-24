from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'launch_files'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
# ROS2 Imports (standard Ressource)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
# package.xml Datei
        ('share/' + package_name, ['package.xml']),
       
       
 # Eintrag zum Finden der Packages, die gelauncht werden sollen   
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pa99',
    maintainer_email='patrice.wenzig@stud.hs-kempten.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
