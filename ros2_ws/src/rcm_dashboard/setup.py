from glob import glob
from setuptools import setup
setup(
    name="rcm_dashboard", version="0.1.0", packages=["rcm_dashboard"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/rcm_dashboard"]),
                ("share/rcm_dashboard", ["package.xml"]),
                ("share/rcm_dashboard/web", glob("web/*"))],
    install_requires=["setuptools"], zip_safe=True,
    entry_points={"console_scripts": ["dashboard = rcm_dashboard.dashboard_node:main"]},
)
