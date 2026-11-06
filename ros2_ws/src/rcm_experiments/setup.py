from glob import glob
from setuptools import setup
setup(
    name="rcm_experiments", version="0.1.0", packages=["rcm_experiments"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/rcm_experiments"]),
                ("share/rcm_experiments", ["package.xml"]),
                ("share/rcm_experiments/protocols", glob("protocols/*.yaml"))],
    install_requires=["setuptools"], zip_safe=True,
    entry_points={"console_scripts": ["test_runner = rcm_experiments.runner_node:main"]},
)
