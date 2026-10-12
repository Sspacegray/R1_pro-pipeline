from glob import glob
from setuptools import setup


package_name = "r1pro_web_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        package_name,
        f"{package_name}.calibration",
        f"{package_name}.code_workflows",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/web", glob("web/*.*")),
        (f"share/{package_name}/web/map_editor", glob("web/map_editor/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="motrix",
    maintainer_email="motrix@todo.todo",
    description="LAN web control panel for R1 Pro manipulation and perception.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "r1pro_web_server = r1pro_web_control.server_node:main",
            "r1pro_web_vision_worker = r1pro_web_control.vision_worker:main",
            "r1pro_web_head_depth_worker = r1pro_web_control.head_depth_worker:main",
            "r1pro_hook_effort_calibrate = r1pro_web_control.calibration.hook_effort_calibration:main",
            "r1pro_hook_transport_calibrate = r1pro_web_control.calibration.hook_transport_calibration:main",
            "r1pro_map_editor = r1pro_web_control.map_editor:main",
        ],
    },
)
