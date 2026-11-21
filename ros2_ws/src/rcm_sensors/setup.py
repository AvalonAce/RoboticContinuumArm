from setuptools import setup
setup(
    name="rcm_sensors", version="0.1.0", packages=["rcm_sensors"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/rcm_sensors"]),
                ("share/rcm_sensors", ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True,
    entry_points={"console_scripts": ["sensor_bridge = rcm_sensors.sensor_bridge:main",
                                  "flex_bridge = rcm_sensors.flex_bridge:main"]},
)
