# Install

## On the robot

1. Copy this repository to the Koala 2.5 robot's Linux, or to a Linux machine with its serial port.
2. Make the scripts runnable: `chmod +x k_create.sh k_init.sh`.
3. Run `./k_init.sh`. It builds the library from `src/` with `src/Makefile` into `build-x86-gcc/` and `build-x86-g++/`, and on `y` copies `libkoala.so`, `libkoala.a` and the headers under `/usr`. It calls `sudo` itself.
4. Create a project with `./k_create.sh c <name>` or `./k_create.sh cpp <name>`.

## Here, for the tests

1. Install mingw gcc and g++ 13 or newer, CMake 3.28 or newer, Ninja, LLVM for `clang-tidy` and `clang-format`, and Python with pytest.
2. Clone this repository.
3. Run `.\recipe.ps1 test`. The first run downloads GoogleTest, whose archive `CMakeLists.txt` pins by hash.

`recipe.ps1` looks for gcc and g++ in `C:\ProgramData\mingw64\mingw64\bin` and for LLVM in `C:\Program Files\LLVM\bin`, and adds those folders to its own PATH.

`.\recipe.ps1 build` compiles `src/koala_parse.c` and the tests into `build\`. It builds nothing else of the library, which needs the robot's `termio.h`. `.\recipe.ps1 install` and `.\recipe.ps1 run` do nothing here: there is no installer, and the scripts run on the robot.
