from glob import glob
import os
from setuptools import setup

package_name = 'booblik'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),
         glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='bogdanov_am@spbstu.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
                'console_scripts': [
                    'motors = booblik.motors:main',
                    'gpsimu = booblik.gpsimu:main',
                    'ping = booblik.ping:main',
                    'ws_m181 = booblik.ws_m181:main',
                ],
    },
)
