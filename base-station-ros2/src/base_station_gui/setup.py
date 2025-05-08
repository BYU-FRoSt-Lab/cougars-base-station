from setuptools import find_packages, setup

package_name = 'base_station_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='tristanhodgins',
    maintainer_email='tah88@byu.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    install_requires=[
        "setuptools",
        "PyQt6",
        "graphviz",
    ],
    entry_points={
        'console_scripts': [
            'main = base_station_gui.main:main'
        ],
    },
)
