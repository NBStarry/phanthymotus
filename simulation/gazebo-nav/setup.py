from glob import glob
from setuptools import find_packages, setup

package_name = "phanthymotus_sim_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/maps", glob("maps/*")),
        (f"share/{package_name}/worlds", glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["navigation_node = phanthymotus_sim_nav.navigation_node:main"]},
)
