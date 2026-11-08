# koala2-5-manager

![Koala 2.5](image/README/1690364539971.jpg)

**Koala 2.5 Manager** is a pair of Bash scripts that create and build C and C++ projects for the Koala 2.5 robot, with the Koala library they link. `k_create.sh` lays out a new project with its sources and a `Makefile`; `k_init.sh` builds the library and installs it on the robot's Linux.

## Contents

- `k_create.sh`: creates a C or C++ project that links the Koala library.
- `k_init.sh`: builds the library from `src/` and installs it.
- `projects/k_us_sensor_off.sh`: switches the ultrasonic sensors off by sending `C,0,0` to `/dev/ttyS1`; `projects/README/k_us_sensor_off.md` describes it.
- `src/`: K-Team's Koala library, which talks to the robot over the serial port `/dev/ttyS1`, its `Makefile`, its Doxygen settings, and `koala.py`, a `ctypes` wrapper over `libkoala.so`.

## Requirements

- Bash
- GCC for C projects, G++ for C++ projects
- The Koala 2.5 robot, or a Linux with its serial port, for anything that talks to it

## Usage

### k_create.sh

![k_create.sh](image/README/1690364226924.png)

```bash
Usage: k_create.sh <language> <project_name>

Parameters:
  <language>     : Use 'c' for C projects or 'cpp' for C++ projects.
  <project_name> : The name of the new project.
```

`./k_create.sh c my_c_project` creates a C project named `my_c_project` with its folders, sources and a `Makefile`. `./k_create.sh cpp my_cpp_project` does the same for C++. To build and run it:

```bash
cd my_cpp_project
make
make run
```

### k_init.sh

![k_init.sh](image/README/1690364214275.png)

`k_init.sh` takes no argument. It runs `src/Makefile` with `sudo` for `TARGET_SYSTEM=x86`: it uninstalls an old copy, builds the C, Python and C++ variants of the library, then asks once whether to install the C variant under `/usr/lib` and `/usr/include`.

### The project it creates

```text
project_name/
├── src/
│   ├── main.c (or main.cpp for C++ projects)
│   ├── project_name.c (or project_name.cpp)
│   └── project_name.h
├── include/
│   └── project_name.h
├── lib/
│   ├── libkoala.so
│   ├── libkoala.a
│   └── libkoala.so.1.0
└── Makefile
```

## Build and test

Almost all of this repository runs only on the robot. The one part that needs no robot is `process_param` in `src/koala_parse.c`, which splits an answer such as `120,-45` into integers. `.\recipe.ps1 test` builds it with CMake and Ninja, runs its GoogleTest cases under `ctest`, runs `clang-tidy` and `clang-format` over it, and searches `src/` for locks and for `throw`, `try` and `catch`. The rest of `src/` includes `termio.h`, so it builds with `src/Makefile` on the robot and nowhere else. `docs/install.md` has the details, and `docs/manual.md` covers the scripts and the parser.

## Licence

The Koala library in `src/` is `Copyright (C) Julien Tharin, K-Team S.A. 2013`, under the GNU Lesser General Public License 2.1 or later, as each of its files states. There is no `LICENSE` file at the root: which licence the scripts take is still the owner's decision.

## Lab requirements

This repository complies with [revision 4.0.0](http://127.0.0.1:3000/igors-lab/software-requirements/src/tag/v4.0.0/BEST_REQUIREMENTS.md) of the lab requirements, except R-02 (#8), R-09 (#9), R-12 (#10), R-13 (#11), R-13 (#12), R-14 (#13), R-18 (#14), R-37 (#15), R-41 (#16), R-42 (#17).
