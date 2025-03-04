from setuptools import find_packages, setup

package_name = 'camera_pkg'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='robin',
    maintainer_email='robin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'camera_node = camera_pkg.camera_node:main',
            'camera_topic = camera_pkg.camera_topic:main',
        ],
    },
)
