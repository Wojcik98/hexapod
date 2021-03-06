from setuptools import setup

package_name = 'line_detection'

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
    description='Line detection.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'line_detector = {package_name}.line_detector:main',
            f'sparser = {package_name}.sparser:main',
            f'camera_calibration = {package_name}.camera_calibration:main',
        ],
    },
)
