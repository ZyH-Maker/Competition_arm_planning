from setuptools import setup

package_name = "bt_test_nodes"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Test publishers for main_bt signals",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "start_publisher = bt_test_nodes.start_publisher:main",
            "success_publisher = bt_test_nodes.success_publisher:main",
            "qrcode_publisher = bt_test_nodes.qrcode_publisher:main",
            "circles_service = bt_test_nodes.circles_service:main",
        ],
    },
)
