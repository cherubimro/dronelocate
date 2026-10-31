# Building UHD's Python bindings for python3.11 (openSUSE laptop only)

**You almost certainly do not need this.** On the Debian NUC — the deploy
target — `python3` is already 3.11, so `apt install uhd-host python3-uhd`
puts the bindings on the interpreter that runs the project and you are done.

This page exists for the openSUSE Leap 15.6 dev laptop, where the split is:

| | has `uhd` | has `zenoh` + `cbor2` |
|---|---|---|
| `python3` (3.6.15) | yes, from `python3-uhd` | no |
| `python3.11` | no | yes |

Neither interpreter can run a UHD sensor node. `node.py --source uhd` needs
both halves in one process.

## Why the installed libuhd cannot just be reused

The tempting shortcut is to keep the distro's `libuhd4_5_0` and rebuild only
the Python module. That does not work: `libpyuhd` is a pybind11 extension
compiled against a specific CPython ABI. A `.so` built for 3.6 will never
load into 3.11, and the module is not separable from the library build in
UHD's CMake. So libuhd gets built too, which is where the time goes.

The build goes into a **private prefix** and touches nothing the distro owns.
The python3.6 UHD install keeps working; `uhd_find_devices` and friends still
come from the package.

## Prerequisites

```bash
sudo zypper install python311-devel python311-Mako python311-setuptools \
  libusb-1_0-devel \
  libboost_headers1_66_0-devel libboost_atomic1_66_0-devel \
  libboost_chrono1_66_0-devel libboost_date_time1_66_0-devel \
  libboost_filesystem1_66_0-devel libboost_test1_66_0-devel \
  libboost_program_options1_66_0-devel libboost_regex1_66_0-devel \
  libboost_serialization1_66_0-devel libboost_system1_66_0-devel \
  libboost_thread1_66_0-devel
```

Boost **1.66** to match what `libuhd4_5_0` already links. `libusb-1_0-devel`
is the one that actually matters for us — without it UHD silently builds
without B200/B210 support, and you get a working `import uhd` that cannot see
the radio.

No pybind11 package is needed: UHD 4.5.0.0 vendors pybind11 2.10.0 in
`host/lib/deps/pybind11`, and 2.10.0 is the first release with Python 3.11
support. Convenient, and worth checking if you ever move to a different UHD
tag — an older vendored pybind11 will not compile against 3.11.

## Build

Source must be the release tarball for the version you are matching:

```bash
curl -sSL -o uhd-4.5.0.0.tar.gz \
  https://github.com/EttusResearch/uhd/archive/refs/tags/v4.5.0.0.tar.gz
tar xzf uhd-4.5.0.0.tar.gz
mkdir uhd-build && cd uhd-build

cmake ../uhd-4.5.0.0/host \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/opt/uhd-py311" \
  -DPYTHON_EXECUTABLE=/usr/bin/python3.11 \
  -DRUNTIME_PYTHON_EXECUTABLE=/usr/bin/python3.11 \
  -DENABLE_PYTHON_API=ON -DENABLE_LIBUHD=ON \
  -DENABLE_B200=ON -DENABLE_USB=ON \
  -DENABLE_EXAMPLES=OFF -DENABLE_UTILS=OFF -DENABLE_TESTS=OFF \
  -DENABLE_C_API=OFF -DENABLE_MANUAL=OFF -DENABLE_DOXYGEN=OFF \
  -DENABLE_MAN_PAGES=OFF \
  -DENABLE_X300=OFF -DENABLE_MPMD=OFF -DENABLE_N300=OFF -DENABLE_N320=OFF \
  -DENABLE_E320=OFF -DENABLE_E300=OFF -DENABLE_USRP1=OFF \
  -DENABLE_USRP2=OFF -DENABLE_X400=OFF -DENABLE_OCTOCLOCK=OFF

make -j"$(nproc)" && make install
```

Confirm CMake locked onto the right interpreter before letting it compile —
this is the whole point of the exercise, and it fails quietly if `python3`
wins instead:

```bash
grep -E "^PYTHON_(EXECUTABLE|INCLUDE_DIR|LIBRARY):" CMakeCache.txt
# PYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3.11
# PYTHON_INCLUDE_DIR:PATH=/usr/include/python3.11
# PYTHON_LIBRARY:FILEPATH=/usr/lib64/libpython3.11.so
```

The configure summary should list **LibUHD, LibUHD - Python API, USB, B200**
under enabled components.

## Using it

Three environment variables. The third is easy to forget and produces a
confusing failure — the board enumerates and then cannot load its FPGA:

```bash
source ./env-uhd-py311.sh
```

which is just these three, with the paths as CMake actually laid them out —
note the `local/` component in the Python path, and that libuhd lands in
`lib`, not `lib64`:

```bash
export PYTHONPATH=$HOME/opt/uhd-py311/local/lib64/python3.11/site-packages
export LD_LIBRARY_PATH=$HOME/opt/uhd-py311/lib
export UHD_IMAGES_DIR=/usr/share/uhd/images   # firmware from the distro package
```

`LD_LIBRARY_PATH` is not optional. CMake strips the RPATH on install
(`Set runtime path ... to ""`), so without it the extension resolves against
`/usr/lib64/libuhd.so.4.5.0` — the distro copy, which has the python3.6
bindings compiled in and drags `libpython3.6m` into a python3.11 process.
It appears to work. Point it at our `lib` and the dependency is clean:

```bash
ldd .../uhd/libpyuhd.cpython-311-x86_64-linux-gnu.so | grep uhd
#  libuhd.so.4.5.0 => /home/bu/opt/uhd-py311/lib/libuhd.so.4.5.0
```

`UHD_IMAGES_DIR` points at the distro's `uhd-firmware`, because the private
prefix ships no images. `usrp_b210_fpga.bin` is the one the B210 needs.

Then:

```bash
python3.11 -c "import uhd; print(uhd.__file__)"
python3.11 node.py --node n05 --source uhd --config config/site-b210.json
```

## Before plugging the board in

`uhd-udev` installs `/usr/lib/udev/rules.d/10-usrp-uhd.rules`, which hands the
device to group **`usrp`** at mode 0660. The group is created empty, so add
yourself or every capture runs as root:

```bash
sudo usermod -aG usrp "$USER"   # then log out and back in, or: newgrp usrp
```

The rules match Ettus/NI USB IDs (`2500:0020-0022`, `3923:7813-7814`). The
TinyB210 is a **clone** — if `uhd_find_devices` sees nothing, check `lsusb`
for its actual VID:PID and add a matching line before assuming the board is
dead.

## Notes

- The bindings expose **no version attribute** — `uhd.__version__` does not
  exist in 4.5.0. Use `uhd_config_info --version` (from the distro package).
  `hw_selftest.py` already falls back to it.
- `uhd.find('')` with no board attached returns an empty list rather than
  raising; that is the expected result of a bench run with nothing plugged in.
- Nothing here changes the TDOA story. Getting `import uhd` onto the right
  interpreter makes the node *runnable*; whether the TinyB210's PPS path is
  good enough is still `hw_selftest.py`'s question, and still needs hardware.
