"""Build the Angstrong vendor SDK Python extension (app/_ascam_ext).

Run from the repo root with the project venv active:

    pip install pybind11 numpy
    python setup.py build_ext --inplace

Output: app/_ascam_ext.<cpython-tag>-aarch64-linux-gnu.so (next to app/ascam.py).

The compiled extension is gitignored — every machine builds its own from
the tracked .cpp + vendor SDK headers and .so files.
"""

from __future__ import annotations

import os

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

SDK_DIR = os.path.join("Camera_Specs", "ascam_ros2_ws", "src", "ascamera")
SDK_INCLUDE = os.path.join(SDK_DIR, "libs", "include")
SDK_LIB_AARCH64 = os.path.join(SDK_DIR, "libs", "lib", "aarch64-linux-gnu")

# rpath is relative to the location of the compiled extension at runtime.
# Extension lives at  <repo>/app/_ascam_ext.<...>.so
# Vendor libs live at <repo>/Camera_Specs/ascam_ros2_ws/src/ascamera/libs/lib/aarch64-linux-gnu/
# So from the extension's directory:  ../Camera_Specs/.../aarch64-linux-gnu
RPATH = "$ORIGIN/../" + SDK_LIB_AARCH64

ext_modules = [
    Pybind11Extension(
        "app._ascam_ext",
        ["app/_ascam_ext.cpp"],
        include_dirs=[SDK_INCLUDE],
        library_dirs=[SDK_LIB_AARCH64],
        libraries=["AngstrongCameraSdk", "asusb", "asuvc"],
        cxx_std=17,
        extra_link_args=[f"-Wl,-rpath,{RPATH}"],
    ),
]

setup(
    name="ascam_ext",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
