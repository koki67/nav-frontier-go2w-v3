from glob import glob

from setuptools import find_packages, setup

package_name = "nav_frontier_go2w_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Koki Tanaka",
    maintainer_email="67k.tanaka@gmail.com",
    description="Twist-to-Sport-API velocity bridge for the Unitree Go2W.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "velocity_bridge = nav_frontier_go2w_bridge.velocity_bridge_node:main",
        ],
    },
)
