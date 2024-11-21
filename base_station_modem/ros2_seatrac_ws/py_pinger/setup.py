from setuptools import find_packages, setup

package_name = 'py_pinger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='claytonsmith',
    maintainer_email='cas314@byu.edu',
    description='Simple seatrac pinger node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_pinger = py_pinger.py_pinger:main'
        ],
    },
)