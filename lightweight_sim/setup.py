from setuptools import setup


package_name = "lightweight_sim"

packages = [
    package_name,
    f"{package_name}.simulator",
    f"{package_name}.algorithms",
    f"{package_name}.algorithms.controller",
    f"{package_name}.algorithms.planner",
    f"{package_name}.algorithms.utils",
    f"{package_name}.analysis",
    f"{package_name}.visualization",
    f"{package_name}.ros_nodes",
]

package_dir = {
    package_name: ".",
    f"{package_name}.simulator": "simulator",
    f"{package_name}.algorithms": "algorithms",
    f"{package_name}.algorithms.controller": "algorithms/controller",
    f"{package_name}.algorithms.planner": "algorithms/planner",
    f"{package_name}.algorithms.utils": "algorithms/utils",
    f"{package_name}.analysis": "analysis",
    f"{package_name}.visualization": "visualization",
    f"{package_name}.ros_nodes": "ros_nodes",
}

setup(
    name=package_name,
    version="0.1.0",
    packages=packages,
    package_dir=package_dir,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/lightweight_sim.launch.py"]),
        (f"share/{package_name}/config", ["config/default.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "simulator_node = lightweight_sim.ros_nodes.simulator_node:main",
            "planner_node = lightweight_sim.ros_nodes.planner_node:main",
            "controller_node = lightweight_sim.ros_nodes.controller_node:main",
        ],
    },
)
