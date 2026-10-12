from glob import glob
from setuptools import setup


package_name = "r1pro_aruco_fine_approach"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="motrix",
    maintainer_email="motrix@todo.todo",
    description="Closed-loop chassis fine approach using ArUco pose in base_link.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "aruco_fine_approach = r1pro_aruco_fine_approach.fine_approach_node:main",
        ],
    },
)
