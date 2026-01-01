Installation on Ubuntu
There are several ways to install HailoRT on Ubuntu:

As part of the Hailo Software Suite – Please refer to the suite installation instructions inside the Hailo Dataflow Compiler User Guide.

Using a Docker container (without the full suite) – Please refer to the Docker instructions.

Using the Ubuntu installer.

Note

Supported versions - Ubuntu 22.04/24.04, x86_64/aarch64.

The rest of this section focuses on the Ubuntu installer.

Ubuntu Installer Requirements
Note

These software requirements are additionally required system requirements.

build-essential package (needed for compiling the PCIe driver)

(Optional) bison, flex, libelf-dev and dkms packages (needed to register the PCIe driver using DKMS)

(Optional) curl (needed to download the firmware when installing without the PCIe driver)

(Optional) cmake (needed for compiling the HailoRT examples)

(Optional) pip and virtualenv (needed for pyhailort)

(Optional) systemd (needed for Multi-Process service)

Installing HailoRT on Ubuntu
The HailoRT Ubuntu offers several files, select the files according to your requirements:

HailoRT PCIe driver and FW - hailort-pcie-driver_<version>_all.deb

HailoRT for the platform architecture - one of the following:

hailort_<version>_amd64.deb – HailoRT for amd64

hailort_<version>_arm64.deb – HailoRT for arm64

hailort_<version>_armel.deb – HailoRT for armel

hailort_<version>_armv4.deb – HailoRT for armv4

PyHailoRT for the the platform architecture and installed Python version - one of the following:

hailort-<version>-cp310-cp310-linux_x86_64.whl – PyHailoRT for python3.10, x86_64

hailort-<version>-cp311-cp311-linux_x86_64.whl – PyHailoRT for python3.11, x86_64

hailort-<version>-cp312-cp312-linux_x86_64.whl – PyHailoRT for python3.12, x86_64

hailort-<version>-cp310-cp310-linux_aarch64.whl – PyHailoRT for python3.10, aarch64

hailort-<version>-cp311-cp311-linux_aarch64.whl – PyHailoRT for python3.11, aarch64

hailort-<version>-cp312-cp312-linux_aarch64.whl – PyHailoRT for python3.12, aarch64

Note

PyHailoRT is optional and is needed for using the Python API

Note

PyHailoRT supports numpy 1.x versions

Download
Download the relevant files for your environment from our Developer Zone.

To install HailoRT hailort_<version>_<arch>.deb is required. Information about machine’s architecture can be found using:

# .deb architecture
dpkg --print-architecture
To install the PCIe driver hailort-pcie-driver_<version>_all.deb is required.

Optional - To install PyHailoRT, you’ll also need: hailort-<version>-<python-tag>-<abi-tag>-<platform-tag>.whl.

You can find your python version and architecture using:

# Python version major and minor digits
python -V | cut -d. -f1,2 | tr -dc '0-9.' | xargs
# .whl architecture
uname -m
Installation of the PCIe Driver Only
Run the following command to install only the PCIe driver:

sudo dpkg --install hailort-pcie-driver_<version>_all.deb
Note

A prompt regarding using DKMS (Dynamic Kernel Module Support) is expected and it is recommended to approve it.

Note

PC restart is required after driver installation.

Installation with the PCIe Driver
Run the following command to install HailoRT including the PCIe driver:

sudo dpkg --install hailort_<version>_$(dpkg --print-architecture).deb hailort-pcie-driver_<version>_all.deb
This command will install HailoRT. It will also compile the PCIe driver for the machine’s kernel version and install the driver. The command needs root permissions (sudo).

Note

PC restart is required after driver installation.

After boot, you can use the hailortcli tool and run hailortcli scan to validate that the device is identified:

hailortcli scan
The scan command should find the device.

Note

The PCIe driver is not signed. In some systems it means that secure boot has to be disabled to load the driver. Users who wish to use secure boot while HailoRT PCIe driver is not signed, can use MOK (Machine-Owner Key), which can be used for (locally) signing third-party drivers. Please notice this is an advanced feature and is recommended for users who are familiar with the process of locally-signing drivers.

Identifying Device’s Serial Number
Run the following command to obtain the serial number from the device:

hailortcli fw-control identify
Installation of pyHailoRT into a New Environment
Run the following command to install PyHailoRT into a new virtual environment:

virtualenv -p python<python_version> hailo_platform_venv && . hailo_platform_venv/bin/activate && pip install ./hailort-<version>-<python_tag>-<abi_tag>-<platform_tag>.whl
For example, for installing PyHailoRT v4.17.0 to a Python-3.10 environment on a x86_64 platform, the command would be:

virtualenv -p python3.10 hailo_platform_venv && . hailo_platform_venv/bin/activate && pip install ./hailort-4.17.0-cp310-cp310-linux_x86_64.whl
Installation of pyHailoRT into an Existing Environment
Run the following command to install PyHailoRT into existing virtual environment:

source <virtualenv>/bin/activate && pip install ./hailort-<version>-<python_tag>-<abi_tag>-<platform_tag>.whl
Uninstalling HailoRT
Remove the PCIe driver:

sudo dpkg --purge hailort-pcie-driver
Remove HailoRT (excluding pyhailort):

sudo dpkg --purge hailort
Remove pyhailort (if installed):

source hailo_platform_venv/bin/activate
pip uninstall hailort
Installation on Yocto-based Linux Distribution
See the Yocto page for details about Yocto integration.

Compiling from Sources
Compiling Hailo PCIe Driver from Sources
See the PCIe driver page for details about compiling the PCIe driver from sources.

Compiling HailoRT from Sources
Using HailoRT with other Linux distributions is possible via source compilation. On Ubuntu, it is even sometimes useful to compile from sources, for example in order to keep ABI integrity. HailoRT sources can be cloned from GitHub using:

git clone https://github.com/hailo-ai/hailort.git
Compiling the sources is done with the following command:

cmake -S. -Bbuild -DCMAKE_BUILD_TYPE=Release && cmake --build build --config release
The compilation will create two artifacts:

Binary called hailortcli located in build/hailort/hailortcli/

Library called libhailort.so.<version> located in build/hailort/libhailort/src/

Note

By adding --target install to the CMake command, HailoRT artifacts will be installed on the machine.

Linux:

cmake -S. -Bbuild -DCMAKE_BUILD_TYPE=Release && sudo cmake --build build --config release --target install
after installation (either by installer or CMake install) one can link with libhailort by using CMake’s find_package(). See ‘hailort/libhailort/examples/CMakeLists.txt’ for reference.

Note

By adding -DHAILO_BUILD_EXAMPLES=1 to the CMake command, examples targets will be added to the project (useful for debug build of C++ examples, instead of linking them with pre-installed HailoRT).

Note

When building from sources, some additional tools must be installed. For example, gcc for arm or python-dev if building pyhailort.

You can now run it with:

build/hailort/hailortcli/hailortcli
Note

To compile sources on Windows please see Windows compile from sources

Compiling Specific HailoRT Targets
Compiling a specific target is supported using CMake’s API --target

Supported targets are: libhailort, hailortcli

cmake -S. -Bbuild -DCMAKE_BUILD_TYPE=Release && cmake --build build --config release --target libhailort
Some HailoRT targets require additional cmake flags to be built.

Compilation Of Hailort-Python-Binding (i.e. hailo_platform wheel):

# Run from hailort/libhailort/bindings/python/platform dir
python setup.py bdist_wheel
Note

The wheel depends on libhailort. Make sure libhailort is installed before building the (.whl) file.

Note

To change the build type (Release, Debug, etc.) of the pyhailort library, set the environment variable CMAKE_BUILD_TYPE. Default is Release.

Cross-Compilation Of Hailort-Python-Binding (i.e. hailo_platform wheel):

For cross-compilation, set the following environment variables to setup.py bdist_wheel:
CMAKE_TOOLCHAIN_FILE – path to the toolchain file (defines the compiler and linker for cross–compilation)

HAILORT_INCLUDE_DIR – path to the include directory of the HailoRT library

LIBHAILORT_PATH – path to the HailoRT library

PYTHON_EXECUTABLE – path to the python executable. See <https://pybind11.readthedocs.io/en/stable/compiling.html#findpython-mode>

PYBIND11_FINDPYTHON – see pybind documentation <https://pybind11.readthedocs.io/en/stable/compiling.html#findpython-mode>

Note

Since the wheel architecture is set by default to the native architecture, the user is required to pass to bdist_wheel the argument --plat-name - which indicates the target architecture (e.g. linux_aarch64).

# Run from hailort/libhailort/bindings/python/platform dir
LIBHAILORT_PATH=<LIB_PATH> HAILORT_INCLUDE_DIR=<INC_PATH> CMAKE_TOOLCHAIN_FILE=<TOOLCHAIN_PATH> python setup.py bdist_wheel --plat-name=linux_aarch64
Compilation of hailort-gstreamer-binding:

cmake -S. -Bbuild -DCMAKE_BUILD_TYPE=Release -DHAILO_BUILD_GSTREAMER=1 && cmake --build build --config release --target gsthailo
Note

On Windows, by default, the gstreamer files are expected in the following path: C:/gstreamer/1.0/msvc_x86_64. This path can be changed by setting the CMake variable GSTREAMER_ROOT_DIR.

Compilation of hailort-examples:

cmake -S. -Bbuild -DCMAKE_BUILD_TYPE=Release -HAILO_BUILD_EXAMPLES=1 && cmake --build build --config release --target hailort_examples
Linux Installation Troubleshooting
This section contains common possible issues that you may encounter after connecting Hailo-8 and installing the driver and library.

Common Errors
Improper PCIe device enumeration
How to verify?
From the terminal, run

lspci | grep hailo
The device should be listed in the terminal output, see driver validation

Possible root cause
Improper mechanical installation

Possible solution
Verify the module is properly attached and secured into the M.2 slot

Possible root cause
Slot is not functional

Possible solution
Verify the slot in use is a valid M.2 slot. Check to see if the slot is disabled in the platform BIOS

Device driver is not properly installed
How to verify?
From the terminal, run

lsmod | grep hailo_pci
The device should be listed in the terminal output

Possible root cause
Driver not installed

Possible solution
Re-install the driver, see driver installation only

Device firmware not loaded
How to verify?
From the terminal, run

dmesg | grep hailo
The firmware load process and events appear there - and if either the load process has ended with success or failure

Possible root cause
Firmware not loaded

Possible solution
If the firmware load has failed (during boot) - the reason may be specified in the log. Re-install the driver, see driver installation only

Module not identified by HailoRT
How to verify?
From the terminal, run

hailortcli scan
The module should be listed in the terminal output

Possible root cause
HailoRT library not installed correctly

Possible solution
Re-install the library, see HailoRT installation

Installation on Windows
Windows Requirements
Note

These software installations are additionally required: system requirements.

Windows 10/11 64-bit

(Optional) CMake and Visual Studio Build Tools – in order to compile applications that use HailoRT

Note

Validated on Visual Studio Build Tools 2019.

Windows Installation Instructions
Connect the Hailo device to the host.

Download the installer file hailort_<version>_windows_installer.msi from Hailo’s website.

Note

It is recommended to remove old HailoRT versions before installing a new version.

Run the installer and follow the instructions. Check the pyHailoRT or multi-process service boxes if you wish to install these features.

Reboot the host.

After boot, you can use the hailortcli tool and run hailortcli scan to validate that the device is identified.

If pyHailoRT - preview was checked, the Python wheel will be found in the directory C:\Program Files\HailoRT\python. Refer to section Installation of pyHailoRT into a new environment to finish the installation of pyHailoRT.

Note

The .whl cannot be installed (via pip) from the Program Files folder.

Note

Gstreamer Support: In order for the hailonet to be detected as part of gstreamer plugins, both gsthailo.lib and gsthailo.dll must be manually copied from C:\Program Files\HailoRT\HailoNet to C:\gstreamer\1.0\msvc_x86_64\lib\gstreamer-1.0

Compiling HailoRT from Sources on Windows
Note

Compilation of HailoRT from sources is recommended, for example, in order to keep ABI integrity. See clone HailoRT and compile HailoRT. Notice that when compiling the HailoRT library from sources, the library will not be signed.

To compile HailoRT on Windows, run:

cmake -S. -Bbuild -A=x64 -DCMAKE_BUILD_TYPE=Release && cmake --build build --config release --target install
Compilation Notes:

This section refers only to HailoRT library itself and not the PCIe driver. The PCIe driver should be installed using the .msi installation file (can be downloaded from our Developer Zone).

When compiling for Windows, add the define NOMINMAX to prevent collisions.

The compilation will create three artifacts:

Binary called hailortcli.exe located in build\hailort\hailortcli\Release\

Library called libhailort.lib located in build\hailort\libhailort\src\Release\

Library(DLL) called libhailort.dll located in build\hailort\libhailort\src\Release\

You can now run it with:

hailortcli.exe
Note

Windows Python API support is still in preview stage. Python applications are supported only through the infer() API. See the Python API section and the InferVStreams API reference for more information.