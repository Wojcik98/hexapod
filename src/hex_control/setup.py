from setuptools import setup

package_name = 'hex_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    author='Michał Wójcik',
    author_email="wojcikmichal98@gmail.com",
    maintainer='Michał Wójcik',
    maintainer_email="wojcikmichal98@gmail.com",
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Hexapod control.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'executor = {package_name}.executor:main',
            f'spi_bridge = {package_name}.spi_bridge:main',
            f'dummy_path_publisher = {package_name}.dummy_path_publisher:main',
            f'dummy_joint_state_publisher = {package_name}.dummy_joint_state_publisher:main',
        ],
    },
)
