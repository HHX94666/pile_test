'''
Date: 2022-03-10 16:26:34
LastEditors: houhuixiang
'''
from setuptools import setup

package_name = 'pile_test'

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
    maintainer='droid',
    maintainer_email='houhuixiang@cvte.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = pile_test.pile_test:main',
            'service = pile_test.service:main',
        ],
    },
)
