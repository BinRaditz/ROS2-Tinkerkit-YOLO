from setuptools import find_packages, setup

package_name = 'tinkerkit_detection'

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
    maintainer='binn',
    maintainer_email='binlearning44@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_publisher = tinkerkit_detection.cam_pub:main',
        'camera_subscriber = tinkerkit_detection.cam_sub:main',
        'detect_node = tinkerkit_detection.detect:main',
        'position_node = tinkerkit_detection.position:main',
        'object_tf2_publisher = tinkerkit_detection.object_tf2_publisher:main',
        'fake_object_publisher = tinkerkit_detection.fake_object_publisher:main',
        'test_code = tinkerkit_detection.testcode:main',
        ],
    },
)