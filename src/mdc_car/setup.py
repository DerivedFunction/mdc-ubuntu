from setuptools import find_packages, setup

package_name = 'mdc_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thong',
    maintainer_email='tdv225@lehigh.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_control = mdc_car.main_control:main'
        ],
    },
)
