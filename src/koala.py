import ctypes
import math
import os
from ctypes import Structure, c_char, c_int, c_double, c_void_p, POINTER, byref
import time

# Загрузка библиотеки koala.so
koala_lib = ctypes.CDLL("/lib/libkoala.so")

# Константы для порта и скорости для последовательной связи
KOALA_SERIAL_PORT_NAME = ctypes.c_char_p(b"/dev/ttyS1")
KOALA_SERIAL_PORT_BAUDRATE = ctypes.c_int(115200)

# Максимальный размер буфера
KOALA_MAX_BUFFER = ctypes.c_int(1024)

# Разделитель для параметров
KOALA_DELIM = ctypes.c_char_p(b",")

# Константы для автоматического режима мониторинга
KOALA_AUTOM_US_SENSOR_BIT = ctypes.c_int(1 << 0)
KOALA_AUTOM_MOTOR_SPEED = ctypes.c_int(1 << 1)
KOALA_AUTOM_MOTOR_POSITION = ctypes.c_int(1 << 2)
KOALA_AUTOM_MOTOR_CURRENT = ctypes.c_int(1 << 3)
KOALA_AUTOM_ACCEL_VALUE = ctypes.c_int(1 << 4)
KOALA_AUTOM_GYRO_VALUE = ctypes.c_int(1 << 5)
KOALA_AUTOM_GPS_DATA = ctypes.c_int(1 << 6)
KOALA_AUTOM_GPS_NEMA = ctypes.c_int(1 << 7)
KOALA_AUTOM_MAGNE_VALUE = ctypes.c_int(1 << 8)
KOALA_AUTOM_ALL = ctypes.c_int(0x01FF)
KOALA_AUTOM_NONE = ctypes.c_int(0x0000)

# Константы для маски датчиков ультразвукового дальномера (US)
KOALA_US_LEFT_REAR = ctypes.c_int(1 << 0)      # Left rear
KOALA_US_LEFT_FRONT = ctypes.c_int(1 << 1)     # Left front
KOALA_US_FRONT_LEFT = ctypes.c_int(1 << 2)     # Front left
KOALA_US_FRONT = ctypes.c_int(1 << 3)          # Front
KOALA_US_FRONT_RIGHT = ctypes.c_int(1 << 4)    # Front right
KOALA_US_RIGHT_FRONT = ctypes.c_int(1 << 5)    # Right front
KOALA_US_RIGHT_REAR = ctypes.c_int(1 << 6)     # Right rear
KOALA_US_BACK_RIGHT = ctypes.c_int(1 << 7)     # Back right
KOALA_US_BACK_LEFT = ctypes.c_int(1 << 8)      # Back left
KOALA_US_ALL = ctypes.c_int(511)               # All US activated
KOALA_US_NONE = ctypes.c_int(0)                # All US deactivated

# Массив имен датчиков ультразвукового дальномера
KOALA_US_SENSOR_NAMES = [
    "Left rear", "Left front", "Front left", "Front", "Front right",
    "Right front", "Right rear", "Back right", "Back left"
]

# Количество датчиков ультразвукового дальномера
KOALA_US_SENSORS_NUMBER = ctypes.c_int(9)

# Константы для маски входов/выходов портов
KOALA_IO_0_OUTPUT = ctypes.c_int(0b00000000)    # Port 0 in output
KOALA_IO_0_INPUT = ctypes.c_int(0b00000001)     # Port 0 in input
KOALA_IO_0_PWMS = ctypes.c_int(0b00000010)      # Port 0 in PWM servo
KOALA_IO_1_OUTPUT = ctypes.c_int(0b00000000)    # Port 1 in output
KOALA_IO_1_INPUT = ctypes.c_int(0b00000100)     # Port 1 in input
KOALA_IO_1_PWMS = ctypes.c_int(0b00001000)      # Port 1 in PWM servo
KOALA_IO_2_OUTPUT = ctypes.c_int(0b00000000)    # Port 2 in output
KOALA_IO_2_INPUT = ctypes.c_int(0b00010000)     # Port 2 in input
KOALA_IO_2_PWMS = ctypes.c_int(0b00100000)      # Port 2 in PWM servo
KOALA_IO_3_OUTPUT = ctypes.c_int(0b00000000)    # Port 3 in output
KOALA_IO_3_INPUT = ctypes.c_int(0b01000000)     # Port 3 in input
KOALA_IO_3_PWMS = ctypes.c_int(0b10000000)      # Port 3 in PWM servo
KOALA_IO_ALL_OUTPUT = ctypes.c_int(0b00000000)  # All IO ports in output

# Константы для маски входов/выходов питания
KOALA_PWR_IO_0_0 = ctypes.c_int(0b00000000)   # Power Port 0 to 0
KOALA_PWR_IO_0_1 = ctypes.c_int(0b00000001)   # Power Port 0 to 1
KOALA_PWR_IO_1_0 = ctypes.c_int(0b00000000)   # Power Port 1 to 0
KOALA_PWR_IO_1_1 = ctypes.c_int(0b00000010)   # Power Port 1 to 1
KOALA_PWR_IO_2_0 = ctypes.c_int(0b00000000)   # Power Port 2 to 0
KOALA_PWR_IO_2_1 = ctypes.c_int(0b00000100)   # Power Port 2 to 1
KOALA_PWR_IO_3_0 = ctypes.c_int(0b00000000)   # Power Port 3 to 0
KOALA_PWR_IO_3_1 = ctypes.c_int(0b00001000)   # Power Port 3 to 1

# Константы для ускорения (accelerometer)
KOALA_ACCEL_G = ctypes.c_double(1.0/16384.0)    # convert to [g]
KOALA_ACCEL_VALUES_NUMBER = ctypes.c_int(30)   # number of values from the accelerometer: 3 axes * 10 values
KOALA_NEW_ACCEL_X = ctypes.c_int(0)            # position of newest X acceleration in the buffer
KOALA_NEW_ACCEL_Y = ctypes.c_int(1)            # position of newest Y acceleration in the buffer
KOALA_NEW_ACCEL_Z = ctypes.c_int(2)            # position of newest Z acceleration in the buffer

# Константы для гироскопа (gyroscope)
KOALA_GYRO_DEG_S = ctypes.c_double(66.0/1000.0)   # convert to [deg/s]
KOALA_GYRO_VALUES_NUMBER = ctypes.c_int(30)       # number of values from the gyrometer: 3 axes * 10 values
KOALA_NEW_GYRO_X = ctypes.c_int(0)                # position of newest X speed in the buffer
KOALA_NEW_GYRO_Y = ctypes.c_int(1)                # position of newest Y speed in the buffer
KOALA_NEW_GYRO_Z = ctypes.c_int(2)                # position of newest Z speed in the buffer

# Константы для магнетометра (magnetometer)
KOALA_MAGNE_VALUES_NUMBER = ctypes.c_int(3)   # x, y ,z

# Максимальное количество данных для I2C, которые могут быть отправлены/получены за один раз
KOALA_MAX_I2C_DATA = ctypes.c_int(256)

# Константы аппаратного обеспечения робота
KOALA_WHEELS_DISTANCE = ctypes.c_double(250.0)   # расстояние между колесами, для вычисления поворота [мм]
KOALA_WHEEL_DIAM = 82.5        # диаметр колеса [мм] (реальный 85; без нагрузки 82.5, с нагрузкой 3 кг: 80.5)

KOALA_PULSE_TO_MM = math.pi * KOALA_WHEEL_DIAM / 23400.0    # коэффициент преобразования положения мотора из импульсов в мм
KOALA_TIME_BTWN = 10             # [мс] время для вычисления скорости
KOALA_SPEED_TO_MM_S = ctypes.c_double(KOALA_PULSE_TO_MM / (KOALA_TIME_BTWN / 1000.0))   # коэффициент преобразования скорости мотора из единиц скорости в мм/сек

KOALA_US_DISABLED_SENSOR = ctypes.c_int(2000)   # выключенный датчик
KOALA_US_NO_OBJECT_IN_RANGE = ctypes.c_int(1000)   # нет объекта в диапазоне 25..250 см
KOALA_US_OBJECT_NEAR = ctypes.c_int(0)           # объект на расстоянии менее 25 см

# Параметры мотора по умолчанию
KOALA_MOTOR_P = ctypes.c_int(10)
KOALA_MOTOR_I = ctypes.c_int(3)
KOALA_MOTOR_D = ctypes.c_int(1)
KOALA_MOTOR_ACC_INC = ctypes.c_int(5)
KOALA_MOTOR_ACC_DIV = ctypes.c_int(1)
KOALA_MOTOR_MIN_SPACC = ctypes.c_int(10)
KOALA_MOTOR_CST_SPEED = ctypes.c_int(200)
KOALA_MOTOR_POS_MARGIN = ctypes.c_int(10)
KOALA_MOTOR_MAX_CURRENT = ctypes.c_int(10)

# Константа для преобразования значения АЦП в вольты
KOALA_AD_TO_V = ctypes.c_double(3.3 / 1024)

# Константа для символа ошибки сообщения RS232
KOALA_RS232_MESSAGE_ERROR_CHAR = ctypes.c_char(b'#')

# Тип данных для gps_data_t
class gps_data_t(ctypes.Structure):
    _fields_ = [
        ('valid_sat', ctypes.c_char),
        ('sat_nb', ctypes.c_int),
        ('lat_val', ctypes.c_double),
        ('lat_car', ctypes.c_char),
        ('long_val', ctypes.c_double),
        ('long_car', ctypes.c_char),
        ('date_time', ctypes.c_int * 9),  # Using int array to represent struct tm fields
        ('speed', ctypes.c_double),
        ('altitude', ctypes.c_int),
    ]

# Тип данных для koala_auto_data_t
class koala_auto_data_t(ctypes.Structure):
    _fields_ = [
        ('left_speed', ctypes.c_int),
        ('right_speed', ctypes.c_int),
        ('left_position', ctypes.c_int),
        ('right_position', ctypes.c_int),
        ('left_current', ctypes.c_int),
        ('right_current', ctypes.c_int),
        ('us', ctypes.c_int * 9),
        ('accel', ctypes.c_int * 30),
        ('gyro', ctypes.c_int * 30),
        ('magne', ctypes.c_int * 3),
        ('gps_raw', ctypes.c_char * 1024),
        ('mode', ctypes.c_char),
        ('gps', gps_data_t),
    ]

# Define the koala_rs232_t struct in Python using ctypes
class koala_rs232_t(ctypes.Structure):
    _fields_ = [
        ('fd', ctypes.c_int),
        ('tios', ctypes.c_void_p),  # Adjust the data type of 'tios' according to its actual type
    ]

def koala_robot_init() -> int:
    koala_robot_init = koala_lib.koala_robot_init
    koala_robot_init.restype = ctypes.c_int
    return koala_robot_init()

def koala_robot_release() -> int:
    koala_robot_release = koala_lib.koala_robot_release
    koala_robot_release.restype = ctypes.c_int
    return koala_robot_release()

def koala_sendcommand(command: bytes, write_len: int) -> int:
    koala_sendcommand = koala_lib.koala_sendcommand
    koala_sendcommand.argtypes = [ctypes.c_char_p, ctypes.c_int]
    koala_sendcommand.restype = ctypes.c_int
    return koala_sendcommand(command, write_len)

def koala_getcommand(command: bytes, read_len: int) -> int:
    koala_getcommand = koala_lib.koala_getcommand
    koala_getcommand.argtypes = [ctypes.c_char_p, ctypes.c_int]
    koala_getcommand.restype = ctypes.c_int
    return koala_getcommand(command, read_len)

def koala_getcommand_line(command: bytes) -> int:
    koala_getcommand_line = koala_lib.koala_getcommand_line
    koala_getcommand_line.argtypes = [ctypes.c_char_p]
    koala_getcommand_line.restype = ctypes.c_int
    return koala_getcommand_line(command)

def koala_get_firmware_version_revision() -> tuple:
    koala_get_firmware_version_revision = koala_lib.koala_get_firmware_version_revision
    koala_get_firmware_version_revision.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    koala_get_firmware_version_revision.restype = ctypes.c_int

    version = ctypes.create_string_buffer(256)
    revision = ctypes.create_string_buffer(256)
    result = koala_get_firmware_version_revision(version, revision)

    if result == 0:
        return version.value.decode(), revision.value.decode()
    else:
        return "", ""

# Прототипы функций
def koala_get_battery_data():
    bat_type = ctypes.c_int()
    bat_voltage = ctypes.c_int()
    bat_current = ctypes.c_int()
    chrg_current = ctypes.c_int()
    result = koala_lib.koala_get_battery_data(ctypes.byref(bat_type), ctypes.byref(bat_voltage), ctypes.byref(bat_current), ctypes.byref(chrg_current))
    return result, bat_type.value, bat_voltage.value, bat_current.value, chrg_current.value

def koala_configure_auto_monitoring_mode(bit_config: int) -> int:
    return koala_lib.koala_configure_auto_monitoring_mode(ctypes.c_uint(bit_config))

def koala_get_from_auto_mode() -> tuple:
    data = koala_auto_data_t()
    result = koala_lib.koala_get_from_auto_mode(ctypes.byref(data))
    return result, data

def koala_configure_us_io(us_mask: int, io_dir: int) -> int:
    return koala_lib.koala_configure_us_io(ctypes.c_int(us_mask), ctypes.c_ubyte(io_dir))

def koala_configure_pid(kp: int, ki: int, kd: int) -> int:
    return koala_lib.koala_configure_pid(ctypes.c_int(kp), ctypes.c_int(ki), ctypes.c_int(kd))

def koala_set_speed_profile(acc_inc: int, acc_div: int, min_speed: int, cst_speed: int, pos_margin: int, max_current: int) -> int:
    return koala_lib.koala_set_speed_profile(ctypes.c_int(acc_inc), ctypes.c_int(acc_div), ctypes.c_int(min_speed), ctypes.c_int(cst_speed), ctypes.c_int(pos_margin), ctypes.c_int(max_current))

def koala_set_position_encoders(left: int, right: int) -> int:
    return koala_lib.koala_set_position_encoders(ctypes.c_int(left), ctypes.c_int(right))

def koala_set_motor_speed(left: int, right: int) -> int:
    return koala_lib.koala_set_motor_speed(ctypes.c_int(left), ctypes.c_int(right))

def koala_set_motor_speed_accel(left: int, right: int) -> int:
    return koala_lib.koala_set_motor_speed_accel(ctypes.c_int(left), ctypes.c_int(right))

def koala_set_motor_speed_open_loop(left: int, right: int) -> int:
    return koala_lib.koala_set_motor_speed_open_loop(ctypes.c_int(left), ctypes.c_int(right))

def koala_read_motor_speed() -> tuple:
    left = ctypes.c_int()
    right = ctypes.c_int()
    result = koala_lib.koala_read_motor_speed(ctypes.byref(left), ctypes.byref(right))
    return result, left.value, right.value

def koala_set_motor_target_position(left: int, right: int) -> int:
    return koala_lib.koala_set_motor_target_position(ctypes.c_int(left), ctypes.c_int(right))

def koala_read_motor_current() -> tuple:
    left = ctypes.c_int()
    right = ctypes.c_int()
    result = koala_lib.koala_read_motor_current(ctypes.byref(left), ctypes.byref(right))
    return result, left.value, right.value

def koala_read_motor_position() -> tuple:
    left = ctypes.c_int()
    right = ctypes.c_int()
    result = koala_lib.koala_read_motor_position(ctypes.byref(left), ctypes.byref(right))
    return result, left.value, right.value

def koala_get_motor_status() -> tuple:
    left_status = ctypes.c_int()
    right_status = ctypes.c_int()
    left_pos = ctypes.c_int()
    right_pos = ctypes.c_int()
    result = koala_lib.koala_get_motor_status(ctypes.byref(left_status), ctypes.byref(right_status), ctypes.byref(left_pos), ctypes.byref(right_pos))
    return result, left_status.value, right_status.value, left_pos.value, right_pos.value

def koala_read_us_sensors(values_array: ctypes.POINTER(ctypes.c_int)) -> int:
    return koala_lib.koala_read_us_sensors(values_array)

def koala_read_accelerometer(values_array: ctypes.POINTER(ctypes.c_int)) -> int:
    return koala_lib.koala_read_accelerometer(values_array)

def koala_read_gyroscope(values_array: ctypes.POINTER(ctypes.c_int)) -> int:
    return koala_lib.koala_read_gyroscope(values_array)

def koala_read_magnetometer(values_array: ctypes.POINTER(ctypes.c_int)) -> int:
    return koala_lib.koala_read_magnetometer(values_array)

def koala_gps_data() -> tuple:
    valid_sat = ctypes.c_char()
    sat_nb = ctypes.c_int()
    lat_val = ctypes.c_double()
    lat_car = ctypes.c_char()
    long_val = ctypes.c_double()
    long_car = ctypes.c_char()
    date_time = ctypes.c_void_p()  # Assuming the struct tm pointer will be returned
    speed = ctypes.c_double()
    altitude = ctypes.c_int()
    result = koala_lib.koala_gps_data(ctypes.byref(valid_sat), ctypes.byref(sat_nb), ctypes.byref(lat_val),
                                      ctypes.byref(lat_car), ctypes.byref(long_val), ctypes.byref(long_car),
                                      date_time, ctypes.byref(speed), ctypes.byref(altitude))
    # Assuming the struct tm pointer will be handled separately in Python code
    return result, valid_sat.value, sat_nb.value, lat_val.value, lat_car.value, long_val.value, long_car.value, speed.value, altitude.value

def koala_send_gps_cmd(gps_cmd: ctypes.c_char_p) -> int:
    return koala_lib.koala_send_gps_cmd(gps_cmd)

def koala_read_i2c(i2c_add: int, i2c_reg: int, nb_read: int, data: ctypes.POINTER(ctypes.c_int)) -> int:
    return koala_lib.koala_read_i2c(ctypes.c_int(i2c_add), ctypes.c_int(i2c_reg), ctypes.c_int(nb_read), data)

def koala_write_i2c(i2c_add: int, i2c_reg: int, nb_data: int, data: ctypes.POINTER(ctypes.c_int)) -> int:
    return koala_lib.koala_write_i2c(ctypes.c_int(i2c_add), ctypes.c_int(i2c_reg), ctypes.c_int(nb_data), data)

def koala_scan_i2c(nb_devices: ctypes.POINTER(ctypes.c_int), address: ctypes.POINTER(ctypes.c_int)) -> int:
    return koala_lib.koala_scan_i2c(nb_devices, address)

def koala_set_pwr_io_output(power_out: int, IO0: int, IO1: int, IO2: int, IO3: int) -> int:
    return koala_lib.koala_set_pwr_io_output(ctypes.c_int(power_out), ctypes.c_int(IO0), ctypes.c_int(IO1),
                                             ctypes.c_int(IO2), ctypes.c_int(IO3))

def koala_read_io(io_state: ctypes.POINTER(ctypes.c_int), in_state: ctypes.POINTER(ctypes.c_int)) -> int:
    return koala_lib.koala_read_io(io_state, in_state)

def koala_read_ad(ad_0: ctypes.POINTER(ctypes.c_int), ad_1: ctypes.POINTER(ctypes.c_int)) -> int:
    return koala_lib.koala_read_ad(ad_0, ad_1)

def koala_reset_microcontroller() -> int:
    return koala_lib.koala_reset_microcontroller()

def koala_change_term_mode(dir: int) -> None:
    koala_lib.koala_change_term_mode(ctypes.c_int(dir))

def koala_kbhit() -> int:
    return koala_lib.koala_kbhit()

def koala_clrscr() -> None:
    koala_lib.koala_clrscr()

def koala_move_cursor(c: int, l: int) -> None:
    koala_lib.koala_move_cursor(ctypes.c_int(c), ctypes.c_int(l))

def koala_move_cursor_column(c: int) -> None:
    koala_lib.koala_move_cursor_column(ctypes.c_int(c))

def koala_move_cursor_line(l: int) -> None:
    koala_lib.koala_move_cursor_line(ctypes.c_int(l))

def koala_erase_line(line: int) -> None:
    koala_lib.koala_erase_line(ctypes.c_int(line))

def koala_timeval_diff(difference, end_time, start_time) -> int:
    return koala_lib.koala_timeval_diff(difference, end_time, start_time)

def koala_init(argc: int, argv: ctypes.POINTER(ctypes.c_char_p)) -> int:
    return koala_lib.koala_init(ctypes.c_int(argc), argv)

def koala_exit() -> None:
    koala_lib.koala_exit()

def koala_rs232_open(name: str, baudrate: int) -> ctypes.POINTER(koala_rs232_t):
    koala_lib.koala_rs232_open.restype = ctypes.POINTER(koala_rs232_t)
    koala_lib.koala_rs232_open.argtypes = [ctypes.c_char_p, ctypes.c_int]
    return koala_lib.koala_rs232_open(ctypes.c_char_p(name.encode('utf-8')), ctypes.c_int(baudrate))

def koala_rs232_close(rs232: ctypes.POINTER(koala_rs232_t)) -> None:
    koala_lib.koala_rs232_close.argtypes = [ctypes.POINTER(koala_rs232_t)]
    koala_lib.koala_rs232_close(rs232)

def koala_rs232_read(rs232: ctypes.POINTER(koala_rs232_t), buf: bytes, len: int) -> int:
    koala_lib.koala_rs232_read.argtypes = [ctypes.POINTER(koala_rs232_t), ctypes.c_char_p, ctypes.c_uint]
    return koala_lib.koala_rs232_read(rs232, buf, ctypes.c_uint(len))

def koala_rs232_readLine_nowait(rs232: ctypes.POINTER(koala_rs232_t), buffer: bytes) -> int:
    koala_lib.koala_rs232_readLine_nowait.argtypes = [ctypes.POINTER(koala_rs232_t), ctypes.c_char_p]
    return koala_lib.koala_rs232_readLine_nowait(rs232, buffer)

def koala_rs232_readLine(rs232: ctypes.POINTER(koala_rs232_t), buffer: bytes) -> int:
    koala_lib.koala_rs232_readLine.argtypes = [ctypes.POINTER(koala_rs232_t), ctypes.c_char_p]
    return koala_lib.koala_rs232_readLine(rs232, buffer)

def koala_rs232_write(rs232: ctypes.POINTER(koala_rs232_t), buf: bytes, len: int) -> int:
    koala_lib.koala_rs232_write.argtypes = [ctypes.POINTER(koala_rs232_t), ctypes.c_char_p, ctypes.c_uint]
    return koala_lib.koala_rs232_write(rs232, buf, ctypes.c_uint(len))