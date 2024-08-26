"""SHT4x Sensirion module"""
import time

from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import BaseSensorEx, IBaseSensorEx, check_value
from sensor_pack_2.crc_mod import crc8

def _calc_crc(sequence) -> int:
    """Обертка для короткого вызова."""
    return crc8(sequence, polynomial=0x31, init_value=0xFF)

class SHT4xSensirion(BaseSensorEx, IBaseSensorEx):
    """Class for work with Sensirion SHT4x sensor"""
    cmd_get_id = 0x89
    cmd_soft_reset = 0x94
    magic = -1 + 2 ** 16

    def __init__(self, adapter: bus_service.BusAdapter, address=0x44, check_crc: bool = True):
        """Если check_crc в Истина, то каждый, принятый от датчика пакет данных, проверяется на правильность путем
        расчета контрольной суммы."""
        check_value(address, range(0x44, 0x47), f"Неверный адрес устройства: {address}")
        super().__init__(adapter, address, True)
        self._check_crc = check_crc
        self._last_cmd_code = None
        #
        self._with_heater = None
        self._value = None
        self._long_pulse = None
        #
        self._buf_1 = bytearray(1)
        self._buf_6 = bytearray(6)

    #@staticmethod
    #def get_answer_len(command_code: int) -> int:
    #    """Возвращает количество байт в ответе датчика"""
    #    if SHT4xSensirion.cmd_soft_reset == command_code:
    #        return 0
    #    return 6

    def get_last_cmd_code(self) -> int:
        """Возвращает последний код команды, переданный по шине данных в датчик"""
        return self._last_cmd_code

    def _send_command(self, command_code: int):
        """Передает команду датчику по шине"""
        check_value(command_code, range(0x100), f"Неверный код команды: {command_code}")
        _local = self._buf_1
        _local[0] = command_code
        self.write(_local)
        self._last_cmd_code = command_code

    def _read_answer(self) -> [bytes, None]:
        """Читает ответ на команду, переданную методом _send_command.
        Возвращает ссылку на буфер с принятыми данными.
        Проверяет CRC"""
        _cmd = self.get_last_cmd_code()
        if SHT4xSensirion.cmd_soft_reset == _cmd:
            return None
        _buf = self._buf_6
        self.read_to_buf(_buf)
        # ответ считан
        if self._check_crc:
            crc_from_buf = [_buf[i] for i in (2, 5)]  # список со значениями CRC
            calculated_crc = [_calc_crc(_buf[rng.start:rng.stop]) for rng in (range(2), range(3, 5))]
            if crc_from_buf != calculated_crc:
                raise ValueError(f"Неверная CRC! Вычислено: {calculated_crc}. Из буфера: {crc_from_buf};")
        return _buf

    def get_id(self) -> tuple[int, int]:
        _cmd = SHT4xSensirion.cmd_get_id
        self._send_command(_cmd)
        # этот 'чудо-датчик' не может сразу отдать прошитый в нем номер! Приходится вызывать sleep_us!
        time.sleep_us(110)
        _buf = self._read_answer()
        t = self.unpack("HBH", _buf)
        # отбрасываю CRC
        return t[0], t[2]

    def soft_reset(self):
        """Программный сброс датчика. После сброса датчик переходит в состояние простоя, idle state!"""
        self._send_command(SHT4xSensirion.cmd_soft_reset)

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время в мкс(!) преобразования сигнала в цифровой код и готовности его для чтения по шине!
        Для текущих настроек датчика. При изменении настроек следует заново вызвать этот метод!"""
        if not self._with_heater:   # работа без нагрева!
            _val = self._value  # 0..2; 0 - низкая, 1 - средняя, 2 - высокая повторяемость/точность
            _ms = 1_600, 4_500, 8_300
            return _ms[_val]

        if self._long_pulse:    # работа с нагревом!
            return 1_100_000
        # короткий импульс нагрева
        return 110_000

    def start_measurement(self, with_heater: bool = False, value: int = 0, long_pulse: bool = False):
        """Настраивает параметры датчика и запускает процесс измерения.
        with_heater - если Истина, то измерение будет проходить c нагревом (НЕ обычный режим)
        value - повторяемость, если with_heater is False:
            0       - низкая (самая низкая точность)
            1       - средняя (средняя точность)
            2       - высокая (высокая точность)
        value - мощность нагрева, если with_heater is True:
            0       -   20 мВт
            1       -   110 мВт
            2       -   200 мВт
        long_pulse - продолжительность нагрева, используется если with_heater is True:
            False   -   0.1 сек
            True    -   1.0 сек"""
        check_value(value, range(3), f"Неверное значение value: {value}")

        _cmd = None
        if not with_heater:
            # нагреватель НЕ используется! 0xE0, 0xF6, 0xFD
            _t = 0xE0, 0xF6, 0xFD
            _cmd = _t[value]

        if with_heater:
            # используется нагреватель! (0x15, 0x1E), (0x24, 0x2F), (0x32, 0x39)
            _t = (0x15, 0x1E), (0x24, 0x2F), (0x32, 0x39)
            _cmd = _t[value][long_pulse]

        # передаю в датчик код команды
        self._send_command(_cmd)
        # запоминаю для get_conversion_cycle_time
        self._with_heater = with_heater
        self._value = value
        self._long_pulse = long_pulse

    def get_measurement_value(self) -> [None, tuple[float, float]]:
        """Возвращает измеренное датчиком значение/значения"""
        _cmd = self.get_last_cmd_code()
        if SHT4xSensirion.cmd_get_id == _cmd:
            return
        _buf = self._read_answer()
        _t = self.unpack("HBH", _buf)
        t = 175.0 * _t[0] / SHT4xSensirion.magic - 45.0    # температура в градусах Цельсия!
        rh = 125.0 * _t[2] / SHT4xSensirion.magic - 6.0    # относительная влажность в процентах!
        return t, rh

    def is_single_shot_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме однократных измерений,
        каждое из которых запускается методом start_measurement"""
        return True

    def is_continuously_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме многократных измерений,
        производимых автоматически. Процесс запускается методом start_measurement"""
        return False

    # Iterator
    # Поскольку разработчики не завезли автоматический режим работы датчика, у меня нет для вас итератора :-)