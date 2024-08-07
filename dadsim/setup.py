from setuptools import setup
import os
from glob import glob

package_name = 'dadsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        f"{package_name}/api",
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml")))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ustc',
    maintainer_email='rl_wang@mail.ustc.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "manual = dadsim.manual:main",
            "ackermann_teleop = dadsim.ackermann_teleop:main",
        ],
    },
)
