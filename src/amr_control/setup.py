from setuptools import find_packages, setup

package_name = 'amr_control'

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
            'shipment_node = amr_control.shipment:main',
            'shipment2_node = amr_control.shipment2:main',
            'test_node = amr_control.test:main',
            'task_scheduler_node = amr_control.task_scheduler:main',
            'section_checker = amr_control.section_checker:main',
            
            
        ],
    },
)
