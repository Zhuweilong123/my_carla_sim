from setuptools import setup


package_name = "lightweight_sim"

packages = [
    package_name,
    f"{package_name}.engine",
    f"{package_name}.engine.simulator",
    f"{package_name}.engine.algorithms",
    f"{package_name}.engine.algorithms.controller",
    f"{package_name}.engine.algorithms.planner",
    f"{package_name}.engine.algorithms.utils",
    f"{package_name}.engine.analysis",
    f"{package_name}.visualization",
    f"{package_name}.engine.ros_nodes",
]

package_dir = {
    package_name: ".",
    f"{package_name}.engine": "engine",
    f"{package_name}.engine.simulator": "engine/simulator",
    f"{package_name}.engine.algorithms": "engine/algorithms",
    f"{package_name}.engine.algorithms.controller": "engine/algorithms/controller",
    f"{package_name}.engine.algorithms.planner": "engine/algorithms/planner",
    f"{package_name}.engine.algorithms.utils": "engine/algorithms/utils",
    f"{package_name}.engine.analysis": "engine/analysis",
    f"{package_name}.visualization": "visualization",
    f"{package_name}.engine.ros_nodes": "engine/ros_nodes",
}

setup(
    name=package_name,
    version="0.1.0",
    packages=packages,
    package_dir=package_dir,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["config/launch/lightweight_sim.launch.py"]),
        (f"share/{package_name}/config", ["config/default.yaml"]),
    ],
    install_requires=["setuptools", "numpy", "pygame-ce>=2.5"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "simulator_node = lightweight_sim.engine.ros_nodes.simulator_node:main",
            "planner_node = lightweight_sim.engine.ros_nodes.planner_node:main",
            "controller_node = lightweight_sim.engine.ros_nodes.controller_node:main",
            "controller_manager_node = lightweight_sim.engine.ros_nodes.controller_manager:main",
            "gui_node = lightweight_sim.engine.ros_nodes.gui_node:main",
        ],
    },
)
