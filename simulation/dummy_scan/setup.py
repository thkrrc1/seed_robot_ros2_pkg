from setuptools import find_packages
from setuptools import setup

package_name = 'dummy_scan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'dummy_scan = scripts.dummy_scan:main',
        ],
    },
)
