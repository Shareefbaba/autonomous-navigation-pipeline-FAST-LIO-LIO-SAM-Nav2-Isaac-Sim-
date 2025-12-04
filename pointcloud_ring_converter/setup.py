from setuptools import setup

package_name = 'pointcloud_ring_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shareef',
    maintainer_email='',
    description='PointCloud2 ring field converter for LIO-SAM',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # name in ros2 run     # module.path:function
            'ring_converter_node = pointcloud_ring_converter.ring_converter_node:main',
            'fake_imu = pointcloud_ring_converter.my_fake_imu:main',
            'imu_upsampling = pointcloud_ring_converter.imu_upsampling:main',
        ],
    },
)
