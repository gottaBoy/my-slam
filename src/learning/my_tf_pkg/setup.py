from setuptools import find_packages, setup

package_name = 'my_tf_pkg'

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
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster=my_tf_pkg.static_tf_broadcaster:main',
            'dynamic_tf_broadcaster=my_tf_pkg.dynamic_tf_broadcaster:main',
            'tf_listener=my_tf_pkg.tf_listener:main'
        ],
    },
)
