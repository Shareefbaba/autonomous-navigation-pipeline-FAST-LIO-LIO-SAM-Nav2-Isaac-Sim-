from setuptools import setup

package_name = 'fast_lio_nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch file
        ('share/' + package_name + '/launch', ['launch/nav2_fastlio.launch.py', 'launch/nav2_liosam.launch.py']),

        # Install config file
        ('share/' + package_name + '/config', ['config/nav2_fst.yaml', 'config/simple_nav.xml', 'config/nav2_liosam.yaml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shareef',
    maintainer_email='your-email@example.com',
    description='Nav2 + FAST-LIO integration package',
    license='MIT',
    entry_points={
        'console_scripts': [
            # no python nodes now
        ],
    },
)
