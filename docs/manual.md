# Manual

## The scripts

`k_create.sh <language> <project_name>` creates a project: `c` for C, `cpp` for C++. It writes `src/main.c` or `src/main.cpp`, a source and a header named after the project, copies the headers and `libkoala` from `build-x86/` into the project, and writes a `Makefile` with `make` and `make run`. The folder it creates can be moved anywhere.

`k_init.sh` runs `src/Makefile` with `sudo`: it uninstalls an old copy, builds the C, Python and C++ variants of the library, and asks once whether to install the C variant under `/usr`.

`projects/k_us_sensor_off.sh` sets `/dev/ttyS1` to 115200 baud and sends `C,0,0` twice, which switches the ultrasonic sensors off. `projects/README/k_us_sensor_off.md` describes it.

## The library

`src/koala.h` includes the rest. `koala_init` opens the serial port `/dev/ttyS1` at 115200 baud and must be called before any other function; `koala_exit` closes it. `koala_robot.h` declares the commands: motor speed and position, the PID and speed profile, the ultrasonic sensors, the accelerometer, gyroscope and magnetometer, GPS, I2C, the I/O and the battery. Each sends one line to the robot and parses its answer. `make docs` in `src/` builds K-Team's Doxygen pages.

## The parser

`process_param` is the part that needs no robot, and the one the tests cover.

```c
#include "koala_parse.h"

char answer[] = "120,-45\r\n";
int values[2];
int rc = process_param(answer, values, 2);
// rc is 0, values is {120, -45}; answer now holds "120" and "-45" split by a terminator.
```

- It reads at most `nb_arg` fields, split at `KOALA_DELIM`, a comma. A run of commas counts as one.
- It returns -1 at the first field that does not start with a number, and keeps what it read before.
- A field such as `12abc` reads as 12, and leading spaces are skipped, as `sscanf` does.
- An answer with fewer fields than asked still returns 0, and the rest of `destination` stays as it was.
- `strtok` writes into `source`, so pass a copy you can lose, and do not call it from two threads at once.
- `nb_arg` must be at least 1: with 0 nothing bounds the writes into `destination`.
