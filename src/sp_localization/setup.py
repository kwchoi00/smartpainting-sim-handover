from glob import glob
from setuptools import setup

package_name = "sp_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kwchoi00",
    maintainer_email="kwchoi4249@gmail.com",
    description="IMU + 휠 엔코더 ESKF 자기위치 추정",
    license="MIT",
    entry_points={
        "console_scripts": [
            "eskf = sp_localization.eskf_node:main",
            "eval = sp_localization.eval_node:main",
            "wheel_odom = sp_localization.wheel_odom:main",
            "slip_injector = sp_localization.slip_injector:main",
            "plot = sp_localization.plot_paths:main",
            "monitor = sp_localization.live_monitor:main",
            "lap_eval = sp_localization.lap_eval:main",
        ],
    },
)
