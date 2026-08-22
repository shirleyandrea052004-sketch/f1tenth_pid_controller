import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'f1tenth_global_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TU_NOMBRE',
    maintainer_email='tu_correo@ejemplo.com',
    description='Global path planning (Dijkstra) and path smoothing for F1TENTH in AutoDRIVE',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'global_planner_node = f1tenth_global_planner.global_planner_node:main',
        'path_smoother_node = f1tenth_global_planner.path_smoother_node:main',
        'sim_tf_broadcaster_node = f1tenth_global_planner.sim_tf_broadcaster_node:main',
    ],
},
)