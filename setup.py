#!/usr/bin/env python

import glob
import sys
from os import path, walk
from setuptools import setup, find_packages


def print_error(*args, **kwargs):
    """ Print in stderr. """
    print(*args, file=sys.stderr, **kwargs)


def find_resource_files():
    """ Find the files under the resource folder. """
    resource_list = []
    for (root, _, files) in walk("resource"):
        for afile in files:
            if afile != package_name:
                src = path.join(root, afile)
                dst = path.join(
                    "share", package_name, path.relpath(root, "resource")
                )
                resource_list.append((dst, [src]))
    return resource_list


# Package name.
package_name = "dg_demos"

package_names_auto = find_packages(where="src")

# Long description from the readme.
with open(
    path.join(path.dirname(path.realpath(__file__)), "readme.md"), "r"
) as fh:
    long_description = fh.read()

# Install the resource files.
data_files_to_install = find_resource_files()

# Install the amend_index files.
data_files_to_install += [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
]
# Install the package.xml.
data_files_to_install += [(path.join("share", package_name), ["package.xml"])]

# Include all launch files.
data_files_to_install += [
    (
        path.join("share", package_name, "launch"),
        glob.glob(path.join("launch", "*.launch.py")),
    )
]

# Install demo files.
scripts_list = []
for (root, _, files) in walk(path.join("demos")):
    for demo_file in files:
        scripts_list.append(path.join(root, demo_file))

# Install scripts.
scripts_list = []
for (root, _, files) in walk(path.join("scripts")):
    for demo_file in files:
        scripts_list.append(path.join(root, demo_file))

# Final setup.
setup(
    name=package_name,
    version="1.0.0",
    package_dir={package_name: path.join("src", package_name)},
    packages=package_names_auto,
    data_files=data_files_to_install,
    scripts=scripts_list,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maximilien Naveau",
    maintainer_email="mnaveau@tuebingen.mpg.de",
    long_description=long_description,
    long_description_content_type="text/markdown",
    description="Collection of real robot demos.",
    license="BSD-3-clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD-3-clause",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
