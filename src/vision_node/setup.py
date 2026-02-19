from setuptools import find_packages, setup

package_name = 'vision_node'

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
    maintainer='fins',
    maintainer_email='wangjinghui@sjtu.edu.cn',
    description='vision node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_server = vision_node.qr_server:main',
            'qr_client = vision_node.qr_client:main',
            'circle_server = vision_node.circle_server:main',
            'circle_client = vision_node.circle_client:main',
        ],
    },
)
