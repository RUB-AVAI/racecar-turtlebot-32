from setuptools import find_packages, setup

package_name = 'cone_detection_pkg'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #("lib/python3.10/site-packages/"+package_name+"/ml_model", [package_name+"/ml_model/best.pt"]),
        ("lib/python3.10/site-packages/"+package_name+"/ml_model", [package_name+"/ml_model/best_l.pt"]),
        #("lib/python3.10/site-packages/"+package_name+"/ml_model", [package_name+"/ml_model/best_m.pt"]),
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
            'cone_detection_node = cone_detection_pkg.cone_detection_node:main',
        ],
    },
)
