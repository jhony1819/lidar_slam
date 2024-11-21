#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import smbus2
import time

# Dirección I2C del MPU-9250
MPU9250_ADDRESS = 0x68
MAG_ADDRESS = 0x0C  # Dirección del magnetómetro (AK8963)

# Registros del MPU-9250
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
MAG_XOUT_L = 0x03  # Registro de salida del magnetómetro

# Inicializa el bus I2C
bus = smbus2.SMBus(1)

# Función para leer un word (2 bytes) del I2C
def read_word(bus, address, reg):
    high = bus.read_byte_data(address, reg)
    low = bus.read_byte_data(address, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:  # Convertir a valor negativo si es necesario
        value = -((65535 - value) + 1)
    return value

# Inicialización del IMU y configuración del magnetómetro
def initialize_mpu9250():
    # Despertar el MPU-9250 (estaba en modo reposo)
    bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0)
    rospy.loginfo("MPU9250 despertado.")

    # Configurar el acelerómetro y giroscopio en sus rangos predeterminados
    # Acelerómetro (±2g, ±4g, ±8g o ±16g)
    # Giroscopio (±250, ±500, ±1000, ±2000 dps)
    bus.write_byte_data(MPU9250_ADDRESS, 0x1C, 0x00)  # Acelerómetro a ±2g
    bus.write_byte_data(MPU9250_ADDRESS, 0x1B, 0x00)  # Giroscopio a ±250 dps

    # Configurar el magnetómetro
    bus.write_byte_data(MAG_ADDRESS, 0x0A, 0x16)  # Configurar el AK8963 para 16-bit resolution, 100Hz
    rospy.loginfo("Magnetómetro configurado.")

# Función para leer los datos del IMU (acelerómetro, giroscopio y magnetómetro)
def read_imu():
    # Leer aceleración (en g)
    accel_x = read_word(bus, MPU9250_ADDRESS, ACCEL_XOUT_H) / 16384.0
    accel_y = read_word(bus, MPU9250_ADDRESS, ACCEL_XOUT_H + 2) / 16384.0
    accel_z = read_word(bus, MPU9250_ADDRESS, ACCEL_XOUT_H + 4) / 16384.0

    # Leer giroscopio (en grados por segundo)
    gyro_x = read_word(bus, MPU9250_ADDRESS, GYRO_XOUT_H) / 131.0
    gyro_y = read_word(bus, MPU9250_ADDRESS, GYRO_XOUT_H + 2) / 131.0
    gyro_z = read_word(bus, MPU9250_ADDRESS, GYRO_XOUT_H + 4) / 131.0

    # Leer magnetómetro (en microteslas)
    mag_x = read_word(bus, MAG_ADDRESS, MAG_XOUT_L)  # Usar lectura del magnetómetro
    mag_y = read_word(bus, MAG_ADDRESS, MAG_XOUT_L + 2)
    mag_z = read_word(bus, MAG_ADDRESS, MAG_XOUT_L + 4)

    return (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), (mag_x, mag_y, mag_z)

# Nodo de publicación de datos IMU en ROS
def imu_publisher():
    # Inicializa el nodo de ROS
    rospy.init_node('imu_publisher', anonymous=True)

    # Crea el publicador para el tópico 'imu/data'
    pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

    # Define la tasa de publicación (10 Hz)
    rate = rospy.Rate(10)

    # Crea un mensaje vacío de tipo Imu
    imu_data = Imu()

    # Inicializar el MPU9250
    initialize_mpu9250()

    # Bucle de publicación
    while not rospy.is_shutdown():
        # Obtener los datos del IMU
        accel, gyro, mag = read_imu()

        # Llenar el mensaje IMU con los datos leídos
        imu_data.linear_acceleration.x = accel[0]
        imu_data.linear_acceleration.y = accel[1]
        imu_data.linear_acceleration.z = accel[2]

        imu_data.angular_velocity.x = gyro[0]
        imu_data.angular_velocity.y = gyro[1]
        imu_data.angular_velocity.z = gyro[2]

        # Enviar los datos al tópico /imu/data
        pub.publish(imu_data)

        # Log de los datos para depuración
        rospy.loginfo("Aceleración: X={:.2f}, Y={:.2f}, Z={:.2f}".format(accel[0], accel[1], accel[2]))
        rospy.loginfo("Giroscopio: X={:.2f}, Y={:.2f}, Z={:.2f}".format(gyro[0], gyro[1], gyro[2]))
        rospy.loginfo("Magnetómetro: X={:.2f}, Y={:.2f}, Z={:.2f}".format(mag[0], mag[1], mag[2]))

        # Espera para mantener la tasa de publicación
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass

