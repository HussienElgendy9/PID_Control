from setuptools import find_packages, setup

package_name = 'lab3_pkg'

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
    maintainer='hussien',
    maintainer_email='hussien@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "distFinder_Node = lab3_pkg.dist_finder:main",
            "control_Node = lab3_pkg.control:main",
        ],
    },
)
