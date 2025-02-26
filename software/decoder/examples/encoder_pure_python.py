import re
import time
import typing

import serial
import serial.tools.list_ports

RE_STATUS_BYTE_MASK = re.compile(r"ERROR_BYTE_MASK_ADS127=0x(\w+)")


class Adc:
    PRINTF_INTERVAL_S = 2.0
    VID = 0x2E8A
    PID = 0x4242
    MEASURMENT_BYTES = 3
    COMMAND_START = b"s"
    COMMAND_STOP = b"p"

    def __init__(self) -> None:
        self.serial = self._open_serial()

    def _open_serial(self) -> serial.Serial:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == self.VID:
                if port.pid == self.PID:
                    return serial.Serial(port=port.device, timeout=0.5)

        raise ValueError(
            f"No board with VID=0x{self.VID:02X} and PID=0x{self.PID:02X} found!"
        )

    def drain(self) -> None:
        while True:
            line = self.serial.read()
            if len(line) == 0:
                return
            
    def read_status(self) -> bool:
        ''' 
        return True on success
        '''
        while True:
            line = self.serial.readline()
            if len(line) == 0:
                return False
            line = line.strip().decode('ascii')
            print(f'  status: {line}')
            if line == "END=1":
                return True

    def iter_measurements(self) -> typing.Iterable[float]:
        counter = 0
        counter_separator = 0
        begin_s = time.monotonic()
        running_crc = 0
        STATUS_BYTE = False
        while True:
            measurement = self.serial.read(size=self.MEASURMENT_BYTES)
            if len(measurement) != self.MEASURMENT_BYTES:
                if len(measurement) == 0:
                    return
                raise ValueError(f"Wrong size {measurement}!")

            # TODO: Sync if not aligned!

            measurement_raw_unsigned = 0
            for idx in (0, 1, 2):
                measurement_raw_unsigned <<= 8
                running_crc ^= measurement[idx]
                measurement_raw_unsigned += measurement[idx]

            if STATUS_BYTE:
                byte_status, byte_crc, byte_reserve = measurement
                show = (byte_status != 0) or (running_crc != 0)
                if show:
                    print(
                        f"{counter_separator=} status=0x{byte_status:02X} crc=0x{byte_crc:02X} (0x{running_crc:02X}) reserve=0x{byte_reserve:02X}"
                    )
                counter_separator = 0
                running_crc = 0
                STATUS_BYTE = False
                continue

            if measurement_raw_unsigned == 0:
                STATUS_BYTE = True
                continue

            def signExtend(measurement_raw_unsigned) -> int:
                """
                See: https://github.com/TexasInstruments/precision-adc-examples/blob/42e54e2d3afda165bd265020bac97c8aedf1f135/devices/ads127l21/ads127l21.c#L571-L578
                """
                if measurement_raw_unsigned & 0x80_00_00:
                    measurement_raw_unsigned -= 0x1_00_00_00

                return measurement_raw_unsigned

            def get_adc_value_V(measurement_raw_signed: int) -> float:
                """
                See https://www.ti.com/lit/ds/symlink/ads127l21.pdf
                page 72, 7.5.1.8.1 Conversion Data
                """
                REF_V = 5.0
                GAIN = 5.0  # 1.0, 2.0, 5.0, 10.0

                return measurement_raw_signed / (2**23) * REF_V / GAIN

            measurement_raw_signed = signExtend(measurement_raw_unsigned)
            adc_value_V = get_adc_value_V(measurement_raw_signed)

            counter_separator += 1
            counter += 1
            duration_s = time.monotonic() - begin_s
            if duration_s > self.PRINTF_INTERVAL_S:
                duration_s = time.monotonic() - begin_s
                print(
                    f"{adc_value_V=:2.6f} ({measurement_raw_signed}) {counter/duration_s:0.1f} SPS"
                )
                begin_s = time.monotonic()
                counter = 0

            yield adc_value_V


def main():
    print("Started")
    adc = Adc()
    adc.serial.write(Adc.COMMAND_STOP)
    print("drain()")
    adc.drain()
    adc.serial.write(Adc.COMMAND_STOP)
    print("status()")
    adc.read_status()
    adc.serial.write(Adc.COMMAND_START)
    print("iter_measurements()")
    for adc_value_V in adc.iter_measurements():
        print(adc_value_V)
        pass
    return
    while True:
        line = pcb_ad.serial_control.readline()
        line = line.strip().decode("ascii")
        if len(line) > 0:  # TODO: Why
            match_status_byte = RE_STATUS_BYTE_MASK.match(line)
            if match_status_byte:
                status_byte_mask = int(match_status_byte.group(1), base=16)
                assert adc.ERROR_BYTE_MASK_ADS127 == status_byte_mask, (
                    adc.ERROR_BYTE_MASK_ADS127,
                    status_byte_mask,
                )
            print(line)


if __name__ == "__main__":
    main()
