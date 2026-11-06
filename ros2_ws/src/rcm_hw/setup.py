from setuptools import setup
setup(
    name="rcm_hw", version="0.1.0", packages=["rcm_hw"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/rcm_hw"]),
                ("share/rcm_hw", ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True,
    entry_points={"console_scripts": ["hw_bridge = rcm_hw.hw_bridge:main",
                                      "fakeboard = rcm_hw.sim:main"]},
)
