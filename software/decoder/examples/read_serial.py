import time

import serial
import serial.tools.list_ports

VID = 0x2E8A
PID = 0x4242


def open_serial_port(vid: int, pid: int) -> serial.Serial:
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == vid and port.pid == pid:
            try:
                return serial.Serial(port=port.device, baudrate=4000000, timeout=1.0)
            except serial.SerialException as e:
                raise ValueError(f"Failed to open serial port: {e}") from e
    raise ValueError("No matching serial port found.")


BUFFER_SIZE = 1_024 * 1_024


def convert():
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

    measurement_raw_unsigned = (((0x3F << 8) + 0x30) << 8) + 0x06
    measurement_raw_signed = signExtend(measurement_raw_unsigned)
    adc_value_V = get_adc_value_V(measurement_raw_signed)
    print(f"{adc_value_V=}")


def main():
    port = open_serial_port(VID, PID)

    port.write(b"s")
    # time.sleep(0.1)
    begin_ns = time.monotonic_ns()
    counter = 0
    while True:
        measurements = port.read(size=BUFFER_SIZE)
        if counter == 0:
            # print(repr(measurements[:1024]))
            for i in range(10):
                print(" ".join([f"0x{measurements[3*i+j]:02X}" for j in range(3)]))

            # 0.49362707138061523
            # 0.49361908435821533
            # 0.4936424493789673

            # 0x3F 0x30 0x06
            # 0x3F 0x2F 0x34
            # 0x3F 0x2F 0x3C
            # 0x3F 0x2F 0x8E
            # 0x3F 0x2F 0x27
            # 0x3F 0x2F 0x24
            # 0x3F 0x2F 0x3C

        counter += len(measurements) // 3
        duration_ns = time.monotonic_ns() - begin_ns
        print(f"{len(measurements)=}, duration_s={duration_ns/1e9}")
        print(f"{1e9*counter/duration_ns:0.1f} SPS")


if __name__ == "__main__":
    convert()
    main()
