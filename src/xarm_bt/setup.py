from setuptools import setup

package_name = "xarm_bt"

setup(
    name=package_name,
    version="0.0.0",
    packages=[
        package_name,
        f"{package_name}.behaviors",
        f"{package_name}.trees",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/grasping_tree.launch.py",
                "launch/grasping_tree_runner.launch.py",
                "launch/set_pose_example.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="oscar",
    maintainer_email="ofc1227@tec.mx",
    description="Python behavior-tree leaves for controlling xArm with MoveItPy.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "set_pose_example = xarm_bt.set_pose_example_node:main",
            "grasping_tree = xarm_bt.grasping_tree_node:main",
        ],
    },
)
