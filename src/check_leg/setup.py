from setuptools import find_packages, setup

package_name = 'check_leg'

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
    maintainer='amr2',
    maintainer_email='amr2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_node = check_leg.check_node:main',
            'check_node_slam = check_leg.check_node_slam:main',
            'check_node_lqr = check_leg.check_node_lqr:main'
        ],
    },
)
