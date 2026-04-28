from glob import glob

from setuptools import find_packages, setup

package_name = "nav_frontier_go2w_frontier"

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
    description="Frontier detection and goal selection for the Go2W (BFS clusters, info-gain scoring).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "frontier_selector = nav_frontier_go2w_frontier.frontier_selector_node:main",
        ],
    },
)
