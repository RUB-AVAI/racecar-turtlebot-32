from setuptools import find_packages, setup

package_name = 'occupancy_map_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("lib/python3.10/site-packages/"+package_name+"/tf_transform.py", [package_name+"/tf_transform.py"]),
    ],
    install_requires=['setuptools', 'tf-transformations'],
    zip_safe=True,
    maintainer='robin',
    maintainer_email='robin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "occupancy_node = occupancy_map_pkg.occupancy_node:main"
        ],
    },
)
