from setuptools import setup

package_name = 'robotiq_2f85_urcap_adapter'

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
    maintainer='scherzin',
    maintainer_email='scherzin@fzi.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'urcap_adapter = robotiq_2f85_urcap_adapter.urcap_adapter:main'
        ],
    },
)
