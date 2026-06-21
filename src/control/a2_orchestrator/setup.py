from setuptools import find_packages, setup

package_name = 'a2_orchestrator'

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
    maintainer='Dino',
    maintainer_email='dino@todo.todo',
    description='Orchestration Behavior Tree package for A2 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_publisher = a2_orchestrator.velocity_publisher:main',
        ],
    },
)
