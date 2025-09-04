from setuptools import find_packages, setup

package_name = 'laptop'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="thong",
    maintainer_email="tdv225@lehigh.edu",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "input_node = laptop.input:main",
            "listener = laptop.listener:main",
            "velocity_to_controller = laptop.velocity_to_controller:main",
        ],
    },
)
