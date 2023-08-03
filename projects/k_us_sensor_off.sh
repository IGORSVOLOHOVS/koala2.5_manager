#!/bin/bash

# Шаг 1: Установка скорости передачи данных на порту /dev/ttyS1 (115200 bps)
echo "Setting serial port speed to 115200 bps..."
sudo stty -F /dev/ttyS1 115200

# Шаг 2: Отправка команды C,0,0 на порт /dev/ttyS1
echo "Sending command: C,0,0 to /dev/ttyS1..."
sudo echo "C,0,0\r" > /dev/ttyS1

# Шаг 3: Повторная отправка команды C,0,0 на порт /dev/ttyS1 (предполагая, что это было необходимо)
echo "Sending command: C,0,0 to /dev/ttyS1 again..."
sudo echo "C,0,0\r" > /dev/ttyS1

echo "k_us_sensor_off.sh execution completed."
