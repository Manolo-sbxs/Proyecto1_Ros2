from setuptools import setup
from glob import glob
import os

package_name = 'triciclo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='manuel.inostroza1801@alumnos.buibio.cl',
    description='Robot tipo triciclo en ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'mover_robot_cmdvel_rosgz = triciclo_py.mover_robot_cmdvel_rosgz:main',
         'demo_movimiento = triciclo_py.demo_movimiento:main',
        ],
    },
)
