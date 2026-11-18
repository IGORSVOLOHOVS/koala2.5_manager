# koala2-5-manager: software requirements specification

Reverse-engineered from the code at commit `00fee5b` (tag `v1.0.0`) on 2026-11-18. It
describes what the code does, not what it was meant to do. Where the two differ, the
difference is listed in [3.10](#310-declared-but-not-implemented) or in
[docs/README.md, known limitations](docs/README.md#known-limitations-and-problems).
The documentation hub is [docs/README.md](docs/README.md).

<!-- jira-sync:start -->
Jira: KAN-81

## Tasks

This list is one ticket in Jira project KAN, [KAN-81](https://igorsvolohovs.atlassian.net/browse/KAN-81). Each line is a checkbox there: tick it in Jira and the sync ticks it here; add a checkbox there and it is appended here. Done items are the features described in section 3; open items are what the project declares but the code does not do yet.

- [x] Project generator (`k_create.sh`) (section 3.1) <!-- spec-project-generator-kcreate-sh -->
- [x] Library build and install (`k_init.sh`, `src/Makefile`) (section 3.2) <!-- spec-library-build-and-install-kinit-sh-src-makefile -->
- [x] Library lifecycle and serial link (section 3.3) <!-- spec-library-lifecycle-and-serial-link -->
- [x] Robot command set (section 3.4) <!-- spec-robot-command-set -->
- [x] Answer parser (`process_param`) (section 3.5) <!-- spec-answer-parser-processparam -->
- [x] Terminal helpers (`koala_utils`) (section 3.6) <!-- spec-terminal-helpers-koalautils -->
- [x] Python wrapper (`src/koala.py`) (section 3.7) <!-- spec-python-wrapper-src-koala-py -->
- [x] Ultrasonic sensors off (`projects/k_us_sensor_off.sh`) (section 3.8) <!-- spec-ultrasonic-sensors-off-projects-kussensoroff-sh -->
- [x] Desktop build and lab tooling (section 3.9) <!-- spec-desktop-build-and-lab-tooling -->
- [ ] The created project "links the Koala library" (declared, not implemented) <!-- spec-the-created-project-links-the-koala-library -->
- [ ] `k_create.sh` uses what `k_init.sh` builds (declared, not implemented) <!-- spec-kcreate-sh-uses-what-kinit-sh-builds -->
- [ ] Project tree with `src/project_name.h` (declared, not implemented) <!-- spec-project-tree-with-src-projectname-h -->
- [ ] Doxygen input `../examples` (declared, not implemented) <!-- spec-doxygen-input-examples -->
- [ ] `KOALA_FLAGS="-lm"` (declared, not implemented) <!-- spec-koalaflags-lm -->
- [ ] `KOALA_RS232_MESSAGE_ERROR_CHAR '#'` (declared, not implemented) <!-- spec-koalars232messageerrorchar -->
- [ ] Motor speed, IR proximity, buzzer, LEDs and TV remote over I2C registers (declared, not implemented) <!-- spec-motor-speed-ir-proximity-buzzer-leds-and-tv-remote -->
- [ ] Licence of the scripts (declared, not implemented) <!-- spec-licence-of-the-scripts -->

<!-- jira-sync:end -->

## 1. Purpose and scope

The system prepares a development environment for a K-Team Koala 2.5 robot on the
robot's Linux, and gives programs a C and Python API to the robot's controller over a
serial port (`README.md:5`, `docs/install.md:5`).

In scope:

- `k_create.sh`: project generator.
- `k_init.sh` and `src/Makefile`: library build and install.
- The Koala library in `src/*.c`, `src/*.h`: serial layer, command set, answer parser,
  terminal helpers.
- `src/koala.py`: Python wrapper.
- `projects/k_us_sensor_off.sh`: ultrasonic sensors off.
- The desktop build and tests of the parser: `CMakeLists.txt`, `tests/koala_parse_test.cpp`.

Out of scope:

- The robot's firmware and the meaning of its replies beyond what the library parses.
- `src/robot`, a C fragment that is not compiled (`src/Makefile:1` builds `*.c` only).
- The lab tooling (`recipe.ps1`, `.claude/`, `.gitea/`, `tests/manual/`, `tools/wiki.py`),
  apart from FR-40.

## 2. Context

| Actor or system | Role | Source |
| --- | --- | --- |
| Developer on the robot | Runs `k_init.sh` and `k_create.sh`, writes programs against `libkoala` | `docs/install.md:3-8` |
| Program using the library | Calls `koala_init`, then the commands | `docs/manual.md:13` |
| Koala 2.5 controller | Answers one-line ASCII commands on `/dev/ttyS1` at 115200 baud | `src/koala_robot.h:45-46` |
| Devices behind the controller | ultrasonic sensors, motors and encoders, accelerometer, gyroscope, magnetometer, GPS, external I2C bus, four I/O ports, four power outputs, two analog inputs, battery | `src/koala_robot.h:52-158`, `src/koala_robot.c` |
| `sudo`, `make`, `gcc`, `g++`, `ar` | Build and install tools called by the scripts | `k_init.sh:23-40`, `src/Makefile:37,39` |
| Windows desktop with mingw, CMake, Ninja, LLVM, Python | Builds and tests `koala_parse` only | `docs/install.md:12`, `CMakeLists.txt:22-26` |

Runtime environment: a Linux with `termio.h` and the device `/dev/ttyS1`
(`src/koala_serial.h:37`, `CMakeLists.txt:22-23`). `k_init.sh` builds with
`TARGET_SYSTEM="x86"` (`k_init.sh:4`). The robot's CPU architecture and Linux
distribution are not stated (see section 9).

## 3. Functional requirements

### 3.1 Project generator (`k_create.sh`)

- **FR-1.** The system shall print `Usage: <script> <language> <project_name>` and exit with status 1 unless exactly two arguments are given. Source: `k_create.sh:3-6`. Verification: no test (observed in a manual run, 2026-11-18).
- **FR-2.** The system shall accept only `c` or `cpp` as the language, and otherwise print `Invalid language. ...` and exit with status 1. Source: `k_create.sh:11-14`. Verification: no test (observed in a manual run, 2026-11-18).
- **FR-3.** The system shall create `<name>/src`, `<name>/include` and `<name>/lib` in the current directory. Source: `k_create.sh:17`. Verification: no test (observed in a manual run, 2026-11-18).
- **FR-4.** The system shall write a `Hello, world!` program to `src/main.c` (C) or `src/main.cpp` (C++), and a stub `src/<name>.c` or `src/<name>.cpp` that includes `../include/<name>.h`. Source: `k_create.sh:20-26`. Verification: no test (observed in a manual run, 2026-11-18).
- **FR-5.** The system shall write `include/<name>.h` with the include guard `<name>_H`. Source: `k_create.sh:28`. Verification: no test.
- **FR-6.** The system shall write a `Makefile` with the targets `all`, `clean`, `depend` and `run`. `DEBUG=1` selects `-g`, otherwise `-O3`. The C++ variant uses `g++` and `-std=c++17`. Source: `k_create.sh:31-35`. Verification: no test.
- **FR-7.** The system shall copy `build-x86/include` and `build-x86/lib/libkoala.so`, `libkoala.a` and `libkoala.so.1.0` from the current directory into the project. Source: `k_create.sh:40-43`. Verification: no test (observed with a placeholder `build-x86/`, 2026-11-18).
- **FR-8.** The system shall end by printing `Project '<name>' created successfully!` with exit status 0. It does not check the result of the copy step. Source: `k_create.sh:45`. Verification: no test (observed: printed after four failed copies, 2026-11-18).

### 3.2 Library build and install (`k_init.sh`, `src/Makefile`)

- **FR-9.** The system shall, from `src/`, run `make uninstall` with `sudo` for `gcc` and `g++`. This removes `/usr/lib/libkoala.so.1.0`, `/usr/lib/libkoala.so`, `/usr/lib/libkoala.a` and `/usr/include/koala`. Source: `k_init.sh:20-24`, `src/Makefile:61-70`. Verification: no test.
- **FR-10.** The system shall compile every `src/*.c` with `$(KOALA_LANGUAGE) -c -fPIC` plus `-O3`, or `-g` when `DEBUG=1`. It links `<LIBNAME>.so.<LIBVER>` with `-shared -lm` and archives `<LIBNAME>.a` into `../build-<TARGET_SYSTEM>-<KOALA_LANGUAGE>/lib`. It copies the headers to `.../include/koala` and creates `<LIBNAME>.so` beside the versioned file with `cp -s`. Source: `src/Makefile:1-44,85-87`. Verification: no test.
- **FR-11.** The system shall build the C variant (`gcc`), then the Python variant, then the C++ variant (`g++`), running `make clean` before each compiled variant. Source: `k_init.sh:27-33`. Verification: no test.
- **FR-12.** The Python variant shall link `koala.so` with `gcc` from the objects already built and copy `src/*.py` to `../build-x86-python`. Source: `src/Makefile:72-77`, `k_init.sh:30`. Verification: no test.
- **FR-13.** The system shall ask once, and only on `y` or `Y` install the C variant. It copies `libkoala.so.1.0` to `/usr/lib`, the headers to `/usr/include/koala`, and creates `/usr/lib/libkoala.so`. Any other answer prints `Installation skipped.` Source: `k_init.sh:35-43`, `src/Makefile:50-59`. Verification: no test.
- **FR-14.** `make docs` (or `make doc`) in `src/` shall run `doxygen` with `src/Doxyfile`, which writes HTML and LaTeX output to `../doc`. Source: `src/Makefile:23-25`, `src/Doxyfile:41,774,1024`. Verification: no test.

### 3.3 Library lifecycle and serial link

- **FR-15.** `koala_init(argc, argv)` shall, on its first call, read the options `--version`/`-v` and `--debug`/`-d`. `-v` prints `libkoala v1.1 [<build date>]`. `-d` has no effect. Source: `src/koala_init.c:47-54,72-93`, `src/koala.h:38-39`. Verification: no test.
- **FR-16.** `koala_init` shall open the robot port through `koala_robot_init` and return -1 if it fails. On success it registers `koala_exit` with `atexit` and returns 0. Later calls return 1. Source: `src/koala_init.c:98-111`. Verification: no test.
- **FR-17.** `koala_robot_init` shall open `/dev/ttyS1` read-write, without becoming its controlling terminal. It sets 115200 baud, 8 data bits, `CLOCAL | CREAD`, `IGNPAR`, no output or local processing, `VMIN = 1`, `VTIME = 1`, and flushes pending input. Source: `src/koala_robot.c:68-81`, `src/koala_serial.c:64-98`. Verification: no test.
- **FR-18.** `koala_exit` shall close the port once and clear the initialised flag. Source: `src/koala_init.c:121-130`, `src/koala_robot.c:94-98`. Verification: no test.
- **FR-19.** `koala_rs232_readLine` shall wait up to 500 ms for input, then read byte by byte until CR, LF or 255 characters, and NUL-terminate the text. It returns the number of characters, -1 without a device, -2 on timeout or `select` error, or -3 if the first read returns nothing. `koala_rs232_readLine_nowait` does the same with a zero wait. Source: `src/koala_serial.c:50,153-199,211-255`. Verification: no test.

### 3.4 Robot command set

- **FR-20.** Each command function shall send one ASCII line made of the command character, comma-separated decimal arguments (text for `X`) and `\r`. It returns -1 when the first character of the reply is not the expected one (table in 4.2), except `koala_get_firmware_version_revision` (-3), `koala_write_i2c` and `koala_send_gps_cmd` (-2). Source: `src/koala_robot.c:202-1122` (exceptions at 245-246, 967-968, 995-996). Verification: no test.
- **FR-21.** The fixed-field reads shall parse the reply with `sscanf` and return -2 when the number of fields read differs: motor speed `E` (2), motor current `O` (2), motor position `P` (2), I/O state `T` (2), analog inputs `U` (2), battery `V` (4), motor status `Y` (4). Source: `src/koala_robot.c:353,677,706,865,894,926,1038`. Verification: no test.
- **FR-22.** The array reads shall parse the reply from offset 2 with `process_param` and return -2 if it fails: ultrasonic `G` (9 values), accelerometer `M` (30), gyroscope `N` (30), magnetometer `&` (3). Source: `src/koala_robot.c:413,445,619,648`, `src/koala_robot.h:82,113,120,125`. Verification: no test.
- **FR-23.** `koala_gps_data` shall send `Q` and parse 14 fields: validity, satellites, latitude and hemisphere, longitude and hemisphere, `hh:mm:ss`, `dd.mm.yy`, speed in knots, altitude. It returns -2 unless all 14 are read. Source: `src/koala_robot.c:730-751`. Verification: no test.
- **FR-24.** `koala_get_firmware_version_revision` shall send `B`, ask for 7 bytes in one `read`, and take the version from byte 2 and the revision from `atoi` at byte 4. It returns -1 or -2 for a NULL argument, -3 for a wrong reply letter, and -4 for a length other than 7. Source: `src/koala_robot.c:228-256`. Verification: no test.
- **FR-25.** `koala_read_i2c` shall send `R,<address>,<register>,<count>` and return -3 unless exactly `<count>` values come back. It stops at 256 (`KOALA_MAX_I2C_DATA`). Source: `src/koala_robot.c:765-801`. Verification: no test.
- **FR-26.** `koala_write_i2c` shall refuse more than `(KOALA_MAX_BUFFER-10)/4` (253) values with -1, and otherwise send `W,<address>,<register>,<v1>,...\r`. Source: `src/koala_robot.c:943-972`. Verification: no test.
- **FR-27.** `koala_send_gps_cmd` shall refuse a command longer than `KOALA_MAX_BUFFER-4` (1020) characters with -1, and otherwise send `X,<command>\r`. Source: `src/koala_robot.c:982-999`. Verification: no test.
- **FR-28.** `koala_scan_i2c` shall send `?`, read the device count and up to 128 addresses, and return -5 if the count of addresses differs from the announced count. Source: `src/koala_robot.c:1075-1122`. Verification: no test.
- **FR-29.** `koala_get_from_auto_mode` shall read one line without waiting and return -1 if none is there. It stores the first character in `data->mode` and fills the matching fields for `e`, `g`, `m`, `n`, `o`, `p`, `q`, `$` (raw GPS text) and `@` (magnetometer). It returns -10 for any other character. Source: `src/koala_robot.c:1134-1193`. Verification: no test.

### 3.5 Answer parser (`process_param`)

- **FR-30.** `process_param(source, destination, nb_arg)` shall split `source` at commas, count a run of commas as one, and store at most `nb_arg` integers. Source: `src/koala_parse.c:42-59`. Verification: `ProcessParam.ReadsEveryFieldAsked`, `ProcessParam.StopsAtTheNumberOfFieldsAsked`, `ProcessParam.ReadsRunsOfDelimitersAsOne`, `ProcessParam.ReadsNineUltrasonicSensors`.
- **FR-31.** The system shall return -1 at the first field that does not start with a number, keeping the values already stored. Source: `src/koala_parse.c:48-49`. Verification: `ProcessParam.FailsOnAFieldThatIsNoNumber`.
- **FR-32.** The system shall read a field as `sscanf("%d")` does. It skips leading spaces, accepts a sign, takes the leading number of `12abc`, and stops at CR/LF. Source: `src/koala_parse.c:48`. Verification: `ProcessParam.ReadsNegativeValuesAndSkipsLeadingSpaces`, `ProcessParam.TakesTheLeadingNumberOfAMixedField`, `ProcessParam.ReadsALineAsTheSerialPortHandsItOver`.
- **FR-33.** The system shall return 0 when the answer has fewer fields than `nb_arg`, or none, and leave the rest of `destination` unchanged. Source: `src/koala_parse.c:45-58`. Verification: `ProcessParam.ReportsNoErrorOnAShortAnswer`, `ProcessParam.WritesNothingForAnEmptyAnswer`.
- **FR-34.** The system shall write a terminator over each delimiter it passes in `source`. Source: `src/koala_parse.c:46,51` (`strtok`). Verification: `ProcessParam.SplitsTheSourceInPlace`.

### 3.6 Terminal helpers (`koala_utils`)

- **FR-35.** `koala_kbhit` shall return the number of characters waiting on standard input, or -1 on error. `koala_change_term_mode(1)` turns off canonical mode and echo, and `koala_change_term_mode(0)` restores the saved mode if it was changed. Source: `src/koala_utils.c:52-104`. Verification: no test.
- **FR-36.** `koala_clrscr`, `koala_move_cursor`, `koala_move_cursor_column`, `koala_move_cursor_line` and `koala_erase_line` shall print ANSI escape sequences, with the origin at (1,1). `koala_timeval_diff` shall return the difference in microseconds and normalise negative microseconds. Source: `src/koala_utils.c:111-198`. Verification: no test.

### 3.7 Python wrapper (`src/koala.py`)

- **FR-37.** On import, the module shall load `/lib/libkoala.so` with `ctypes.CDLL`, and define the library's constants and the structures `gps_data_t`, `koala_auto_data_t` and `koala_rs232_t`. Source: `src/koala.py:8-166`. Verification: no test.
- **FR-38.** The module shall expose one function per public C function except `process_param`, under the same name. The battery, auto-mode, motor speed, current, position and status, and GPS reads return a tuple `(result, values...)`. `koala_get_firmware_version_revision` returns `(version, revision)`, or `("", "")` on error. The array, I/O, A/D and I2C wrappers take ctypes pointers from the caller and return the C result. Source: `src/koala.py:168-382`. Verification: no test.

### 3.8 Ultrasonic sensors off (`projects/k_us_sensor_off.sh`)

- **FR-39.** The script shall set `/dev/ttyS1` to 115200 baud with `sudo stty`, and write the line `C,0,0\r` (with `echo` and no `-e`) to `/dev/ttyS1` twice. Source: `projects/k_us_sensor_off.sh:5,9,13`. Verification: no test.

### 3.9 Desktop build and lab tooling

- **FR-40.** The repository shall carry the lab tooling of
  [repo-quality-template](https://github.com/IGORSVOLOHOVS/repo-quality-template), at
  lab requirements revision 4.0.0 with the exceptions in `README.md:75`. Within it,
  `CMakeLists.txt` builds only `koala_parse` as a C11 static library with
  `-Wall -Wextra -Wpedantic -Werror`. It builds `koala_parse_tests` in C++17 against
  GoogleTest v1.13.0, which is fetched by URL and SHA-256. It registers the CTest tests
  `koala_parse_tests` and `manual_list`. Source: `CMakeLists.txt:1-54`,
  `recipe.ps1:110-127`. Verification: `.\recipe.ps1 test`, not run for this document.

### 3.10 Declared but not implemented

| Claim | Where | What the code does |
| --- | --- | --- |
| The created project "links the Koala library" | `README.md:5,9` | The generated link line has no `-L` or `-l` (`k_create.sh:32,34`). The C include path `./build-x86/include` does not exist in the project, and the C++ `Makefile` leaves `INCS` undefined |
| `k_create.sh` uses what `k_init.sh` builds | `docs/install.md:7-8` (steps in sequence) | `k_init.sh` produces `build-x86-gcc/`, `build-x86-g++/` and `build-x86-python/` (`src/Makefile:7`). `k_create.sh` reads `build-x86/` (`k_create.sh:40-43`). Until `20c3033` (2023-08-01) `src/Makefile` built into `../build-${TARGET_SYSTEM}`, that is `build-x86/`; that commit added the language suffix and left `k_create.sh` unchanged |
| Project tree with `src/project_name.h` | `README.md:55` | Only `include/<name>.h` is written (`k_create.sh:28`) |
| Doxygen input `../examples` | `src/Doxyfile:577-578` | The directory does not exist. It was deleted in `20c3033` (2023-08-01) |
| `KOALA_FLAGS="-lm"` | `k_init.sh:10` | Never passed to `make` |
| `KOALA_RS232_MESSAGE_ERROR_CHAR '#'` | `src/koala_robot.h:164` | Used nowhere |
| Motor speed, IR proximity, buzzer, LEDs and TV remote over I2C registers | `src/robot` | Not compiled. Uses identifiers (`koala_I2C_REG_ADDRESS_*`, `koala_NUMBER_OF_IR_SENSORS`) that are defined nowhere |
| Licence of the scripts | `README.md:71` | Undecided; no `LICENSE` file |

## 4. Interfaces

### 4.1 Command line

| Command | Arguments | Exit status | Source |
| --- | --- | --- | --- |
| `k_create.sh` | `<language>` = `c` or `cpp`, `<project_name>` | 1 on wrong argument count or language, otherwise 0 | `k_create.sh:3-14,45` |
| `k_init.sh` | none; reads one line from stdin (`y`/`Y` installs) | status of the last `echo` | `k_init.sh:35-48` |
| `projects/k_us_sensor_off.sh` | none | status of the last `echo` | `projects/k_us_sensor_off.sh` |
| `make` in `src/` | `all` (default), `python`, `install`, `uninstall`, `clean`, `depend`, `docs`/`doc`; variables in [docs/README.md, configuration](docs/README.md#configuration) | make's | `src/Makefile:21-89` |
| generated `Makefile` | `all`, `clean`, `depend`, `run`; `DEBUG=1` | make's | `k_create.sh:32,34` |
| program linked with `libkoala` | `--version`/`-v`, `--debug`/`-d` (read by `koala_init`) | n/a | `src/koala_init.c:47-54` |

### 4.2 Serial protocol on `/dev/ttyS1`

Every request ends with `\r`. Reply fields start at offset 2, after the reply character
and a comma.

| Request | Reply | Function | Reply fields parsed | Source (`src/koala_robot.c`) |
| --- | --- | --- | --- | --- |
| `A,<bits>` | `a` | `koala_configure_auto_monitoring_mode` | none | 207-216 |
| `B` | `b` | `koala_get_firmware_version_revision` | 7 raw bytes | 233-253 |
| `C,<us_mask>,<io_dir>` | `c` | `koala_configure_us_io` | none | 291-300 |
| `D,<l>,<r>` | `d` | `koala_set_motor_speed` | none | 318-327 |
| `E` | `e` | `koala_read_motor_speed` | 2 ints | 344-356 |
| `F,<l>,<r>` | `f` | `koala_set_motor_target_position` | none | 373-382 |
| `G` | `g` | `koala_read_us_sensors` | 9 ints | 404-416 |
| `&` | `@` | `koala_read_magnetometer` | 3 ints | 436-448 |
| `H,<kp>,<ki>,<kd>` | `h` | `koala_configure_pid` | none | 466-472 |
| `I,<l>,<r>` | `i` | `koala_set_position_encoders` | none | 496-502 |
| `J,<acc_inc>,<acc_div>,<min_speed>,<cst_speed>,<pos_margin>,<max_current>` | `j` | `koala_set_speed_profile` | none | 528-534 |
| `K,<l>,<r>` | `k` | `koala_set_motor_speed_accel` | none | 557-563 |
| `L,<l>,<r>` | `l` | `koala_set_motor_speed_open_loop` | none | 585-591 |
| `M` | `m` | `koala_read_accelerometer` | 30 ints | 610-620 |
| `N` | `n` | `koala_read_gyroscope` | 30 ints | 639-649 |
| `O` | `o` | `koala_read_motor_current` | 2 ints | 668-678 |
| `P` | `p` | `koala_read_motor_position` | 2 ints | 697-707 |
| `Q` | `q` | `koala_gps_data` | 14 fields | 737-747 |
| `R,<addr>,<reg>,<n>` | `r` | `koala_read_i2c` | n ints | 772-798 |
| `S,<pwr>,<io0>,<io1>,<io2>,<io3>` | `s` | `koala_set_pwr_io_output` | none | 826-833 |
| `T` | `t` | `koala_read_io` | 2 ints | 856-866 |
| `U` | `u` | `koala_read_ad` | 2 ints | 885-895 |
| `V` | `v` | `koala_get_battery_data` | 4 ints | 917-927 |
| `W,<addr>,<reg>,<v>...` | `w` | `koala_write_i2c` | none | 953-968 |
| `X,<gps command>` | `x` | `koala_send_gps_cmd` | none | 989-996 |
| `Y` | `y` | `koala_get_motor_status` | 4 ints | 1029-1039 |
| `Z` | `z` | `koala_reset_microcontroller` | none | 1055-1062 |
| `?` | `!` | `koala_scan_i2c` | count, then addresses | 1085-1119 |

Units and codes given in the comments: motor speed in pulses per 10 ms; ultrasonic
values of 0 (under 25 cm), 25 to 250 cm, 1000 (no object), 2000 (disabled); motor
current in 0.1 A; battery voltage in 0.1 V; battery type 0 Li-ion, 1 NiMH, 2 not
initialised; motor status 0 to 6 (idle to error). Sources: `src/koala_robot.c:304-305,385-392,657,903-906,1006-1014`,
`src/koala_robot.h:142-144`.

### 4.3 C API

The public C API is declared in `src/koala_init.h:41-42`, `src/koala_serial.h:51-64`,
`src/koala_robot.h:211-258`, `src/koala_parse.h:54` and `src/koala_utils.h:45-63`.
Programs include `koala.h` (`src/koala.h:42-48`). `koala_parse.h` wraps its declaration
in `extern "C"` for C++ callers (`src/koala_parse.h:37-57`). The other headers do not.

## 5. Data

### 5.1 In-memory types

| Type | Fields | Source |
| --- | --- | --- |
| `koala_rs232_t` | `fd`, `struct termios tios` | `src/koala_serial.h:39-48` |
| `gps_data_t` | `valid_sat`, `sat_nb`, `lat_val`, `lat_car`, `long_val`, `long_car`, `struct tm date_time`, `speed`, `altitude` | `src/koala_robot.h:172-183` |
| `koala_auto_data_t` | speeds, positions, currents (left and right); `us[9]`, `accel[30]`, `gyro[30]`, `magne[3]`, `gps_raw[1024]`, `mode`, `gps` | `src/koala_robot.h:186-202` |
| Python mirrors | `gps_data_t` (with `struct tm` as `c_int * 9`), `koala_auto_data_t`, `koala_rs232_t` (with `tios` as `c_void_p`) | `src/koala.py:130-166` |

Conversion constants: `KOALA_PULSE_TO_MM = π·82.5/23400`; `KOALA_SPEED_TO_MM_S`
uses a 10 ms period; `KOALA_ACCEL_G = 1/16384`; `KOALA_GYRO_DEG_S = 0.066`;
`KOALA_AD_TO_V = 3.3/1024`. Default motor parameters: P 10, I 3, D 1, and others.
Source: `src/koala_robot.h:112-158`.

### 5.2 Files produced

| Path | Produced by | Source |
| --- | --- | --- |
| `build-x86-gcc/lib/libkoala.so.1.0`, `libkoala.a`, `libkoala.so`; `build-x86-gcc/include/koala/*.h` | `k_init.sh` → `make` (C) | `src/Makefile:7,35-44` |
| `build-x86-g++/...` (same layout) | `k_init.sh` → `make` (C++) | `k_init.sh:32-33` |
| `build-x86-python/lib/koala.so`, `build-x86-python/koala.py` | `k_init.sh` → `make python` | `src/Makefile:72-77` |
| `/usr/lib/libkoala.so.1.0`, `/usr/lib/libkoala.so`, `/usr/include/koala/*.h` | `make install` | `src/Makefile:50-59` |
| `../doc/` (HTML, LaTeX) | `make docs` | `src/Doxyfile:41` |
| `<name>/{src,include,lib}`, `<name>/Makefile` | `k_create.sh` | `k_create.sh:17-43` |
| `build/` | `recipe.ps1 build` or `test` (CMake) | `recipe.ps1:53` |

No database or persistent configuration file exists. `tests/manual/tests.json` (version
1, empty list) belongs to the lab tooling.

## 6. Non-functional requirements

- **NFR-1 (portability).** Everything except `koala_parse` requires a POSIX system with
  `termio.h`. Only `koala_parse` builds elsewhere. Source: `CMakeLists.txt:22-25`,
  `src/koala_serial.h:37`.
- **NFR-2 (timing).** A blocking line read gives up if no byte arrives within 500 ms.
  Auto-mode reads do not wait. Source: `src/koala_serial.c:168-173,226-231`.
- **NFR-3 (buffer bounds).** A reply line is at most 255 characters. Command buffers are
  `KOALA_MAX_BUFFER` = 1024 bytes. Source: `src/koala_serial.c:50,175`,
  `src/koala_robot.h:48`.
- **NFR-4 (concurrency).** One process-wide port handle (`koala_rs232`) with no locking,
  and `strtok` in the parser. Concurrent use from several threads is not supported.
  Source: `src/koala_robot.c:45`, `docs/manual.md:32`. The test verb fails if lock words
  appear in `src/` (`recipe.ps1:124`).
- **NFR-5 (no C++ exceptions).** `src/` contains no `throw`, `try` or `catch`, and the
  test verb enforces this. Source: `recipe.ps1:126`.
- **NFR-6 (warnings).** `koala_parse` and its tests compile with warnings as errors on
  non-MSVC compilers. `clang-tidy` runs `bugprone-*`, `performance-*`, `portability-*`
  and `clang-analyzer-*` as errors. Source: `CMakeLists.txt:16-20`, `.clang-tidy`.
- **NFR-7 (reproducible fetch).** GoogleTest is fetched as a fixed archive checked by
  SHA-256. Source: `CMakeLists.txt:33-38`.
- **NFR-8 (privileges).** Building and installing the library runs as root through
  `sudo`. Source: `k_init.sh:23-40`.

## 7. Constraints and dependencies

| Dependency | Version | Source |
| --- | --- | --- |
| Bash | not pinned | `k_create.sh:1`, `k_init.sh:1` |
| gcc, g++ (robot) | not pinned | `k_init.sh:7-8`, `README.md:17` |
| make, ar, sudo; doxygen and makedepend for optional targets | not pinned | `src/Makefile` |
| Python with `ctypes` (wrapper) | not pinned. Type hints only; no version-specific syntax checked | `src/koala.py` |
| CMake (desktop) | 3.28 or newer | `CMakeLists.txt:1`, `docs/install.md:12` |
| mingw gcc and g++ (desktop) | 13 or newer | `docs/install.md:12` |
| Ninja, LLVM (`clang-tidy`, `clang-format`), Python with pytest | not pinned | `docs/install.md:12` |
| GoogleTest | v1.13.0, SHA-256 pinned, gmock off | `CMakeLists.txt:33-41` |
| C / C++ standard | C11 for `koala_parse`; C++17 for tests and generated C++ projects | `CMakeLists.txt:4-10`, `k_create.sh:34` |
| Licence of `src/*.c`, `src/*.h` | LGPL-2.1 or later, K-Team S.A. 2013. `src/koala.py`, `src/Makefile` and `src/robot` carry no licence header | `src/koala.h:4-17`, `README.md:71` |

No lock file exists. Nothing on the robot side is pinned.

## 8. Acceptance

1. Desktop: `.\recipe.ps1 test` exits 0. This covers FR-30 to FR-34 and FR-40, and
   NFR-4 to NFR-7. Before relying on it, check that the installed `clang-format`
   accepts `src/koala_parse.c` (see docs/README.md, limitation 10).
2. Generator, on any Bash: in an empty directory, `k_create.sh` with no arguments and
   with language `x` exits 1 (FR-1, FR-2). `k_create.sh c demo` creates the layout of
   FR-3 to FR-6. With a `build-x86/` present, the files of FR-7 are copied.
3. Robot: `./k_init.sh` leaves the three `build-x86-*` directories, and after `y` the
   files of 5.2 under `/usr` (FR-9 to FR-13).
4. Robot: a program that calls `koala_init` returns 0 with the robot connected and -1
   with `/dev/ttyS1` absent (FR-16). Each command in 4.2 returns 0 against the robot
   (FR-20 to FR-29). No test or script for this exists in the repository.
5. Robot: after `projects/k_us_sensor_off.sh`, `koala_read_us_sensors` reports 2000
   (disabled) for every sensor (FR-39; meaning of 2000 from `src/koala_robot.h:142`).
   This is not verified anywhere.

## 9. Open questions

1. Who uses this repository: a course, a lab, or one robot? Who are the users? Not stated.
2. Should `k_create.sh` read `build-x86-gcc/` (or `build-x86-g++/`), or is a manual
   copy to `build-x86/` expected? Nothing documents the step between the two scripts.
   Partly answered by the history: `build-x86/` was the library's output directory until
   `20c3033` (2023-08-01) renamed it per language without changing `k_create.sh`
   (section 3.10). Which directory the C and C++ variants should take is still unknown.
3. Should the generated `Makefile` link `libkoala` (`-L./lib -lkoala`, include
   `./include`)? `README.md:9` says it links the library, and the code does not.
4. What CPU and Linux does the Koala 2.5 run? Is `/lib` the same as `/usr/lib` there? This
   decides whether `koala.py` finds the installed library, and whether
   `TARGET_SYSTEM=x86` is accurate.
5. Does the firmware accept `C,0,0\r` sent as the characters `\` `r` plus LF, as
   `k_us_sensor_off.sh` writes it? Why is the command sent twice
   (`projects/k_us_sensor_off.sh:11` only says it was assumed necessary)?
6. Where does `src/robot` come from, and is it meant to be built? Its I2C-register API does
   not match the serial command set. It arrived with the library in the first commit
   `1c458bd` (2023-07-26); its origin before that is not stated.
7. Which firmware version does the command table in 4.2 correspond to? The repository
   holds no protocol reference beyond the code comments.
8. Which licence applies to the scripts (`README.md:71`)?
9. Which LLVM version does the CI runner use? `clang-format` 20.1.0 rejects
   `src/koala_parse.c` under the current `.clang-format`.
10. Target performance and timing (control loop rate, acceptable latency) are not stated
    anywhere beyond the 500 ms read timeout.
