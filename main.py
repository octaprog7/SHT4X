import time

from sht4xmod import SHT4xSensirion
from machine import I2C
from sensor_pack_2.bus_service import I2cAdapter


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) # для примера
    # bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)   # на esp8266    !
    # Внимание!!!
    # Замените id=1 на id=0, если пользуетесь первым портом I2C !!!
    # Warning!!!
    # Replace id=1 with id=0 if you are using the first I2C port !!!
    i2c = I2C(id=1, freq=400_000)  # on Arduino Nano RP2040 Connect tested
    adaptor = I2cAdapter(i2c)
    # sensor
    sen = SHT4xSensirion(adaptor, address=0x44, check_crc=True)
    sid = sen.get_id()
    # sen.soft_reset()
    # time.sleep_ms(100)
    repeats = 3_000
    print(f"Sensor id: 0x{sid[0]:x}\t0x{sid[1]:x}")
    #
    print("работа с встроенным в датчик нагревателем")
    sen.start_measurement(with_heater=True, value=2, long_pulse=False)
    wt = sen.get_conversion_cycle_time()
    time.sleep_us(wt)
    results = sen.get_measurement_value()
    print("Результаты после прогрева!")
    print(f"T: {results[0]}; RH: {results[1]}")
    #
    print("Результаты без прогрева!")
    for _ in range(repeats):
        sen.start_measurement(with_heater=False, value=0, long_pulse=False)
        wt = sen.get_conversion_cycle_time()
        time.sleep_us(wt)
        results = sen.get_measurement_value()
        print(f"T: {results[0]}; RH: {results[1]}")
        time.sleep_ms(100)	# чтобы не зависла IDE
