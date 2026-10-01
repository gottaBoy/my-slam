from setuptools import find_packages, setup

package_name = 'mybot_application'

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
    maintainer='fishros',
    maintainer_email='87068644+fishros@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        # 原书这里只注册了 init_robot_pose，但包里有 4 个可执行例子，
        # 导致 `ros2 run mybot_application nav_to_pose` 等三条命令找不到程序。
        # 本仓库把 4 个都注册上（4 个都已实测跑通）。
        'console_scripts': [
            'init_robot_pose=mybot_application.init_robot_pose:main',
            'nav_to_pose=mybot_application.nav_to_pose:main',
            'waypoint_follower=mybot_application.waypoint_follower:main',
            'get_robot_pose=mybot_application.get_robot_pose:main',
        ],
    },
)
