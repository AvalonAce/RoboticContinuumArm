from setuptools import setup
setup(
    name="rcm_kinematics", version="0.1.0", packages=["rcm_kinematics"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/rcm_kinematics"]),
                ("share/rcm_kinematics", ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True,
    entry_points={"console_scripts": ["kinematics = rcm_kinematics.kinematics_node:main"]},
)
