from setuptools import find_packages, setup

package_name = 'd1h_global_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/global_localization.launch.py',
                                              'launch/real_gnss.launch.py',
                                              'launch/cloud_localization.launch.py']),
        ('share/' + package_name + '/config', [
            'config/ekf_local.yaml', 'config/ekf_global.yaml', 'config/navsat.yaml',
            'config/real_lc29h_gnss.yaml',
            'config/cloud_localization.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nav_n',
    maintainer_email='dev@example.com',
    description='D1H global localization helpers.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'real_gnss = d1h_global_localization.real_gnss:main',
            'imu_relay = d1h_global_localization.imu_relay:main',
            'gazebo_localization = d1h_global_localization.gazebo_localization:main',
            'cloud_localization = d1h_global_localization.cloud_localization:main',
            'icp_navigation_bridge = d1h_global_localization.icp_navigation_bridge:main',
        ],
    },
)
