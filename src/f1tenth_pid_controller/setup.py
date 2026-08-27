import os
from glob import glob
from setuptools import setup

package_name = 'f1tenth_pid_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO: tu-nombre',
    maintainer_email='TODO: tu-correo@example.com',
    description='Controlador PID de seguimiento de trayectoria para F1TENTH en AutoDRIVE (Parte 2 del proyecto).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller_node = f1tenth_pid_controller.pid_controller_node:main',
        ],
    },
)
