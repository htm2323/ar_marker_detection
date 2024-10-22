from setuptools import setup

package_name = 'ar_detection'

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
    maintainer='knakamura',
    maintainer_email='xxxx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_tf_publisher = ar_detection.cameratf:main',
            'hl2_transform_listener = ar_detection.node_hl2_transform_listener:main'
        ],
    },
)
