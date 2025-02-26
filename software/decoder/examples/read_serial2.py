import itertools
import pathlib
import time

import ad_low_noise_float_2023_decoder
import serial
from read_serial import BUFFER_SIZE, PID, VID, open_serial_port


class OutOfSyncException(Exception):
    pass


SEQUENCE_LEN_MAX = 32_000
BYTES_PER_MEASUREMENT = 3
DECODER_OVERFLOW_SIZE = 2 * BYTES_PER_MEASUREMENT * SEQUENCE_LEN_MAX  # 2: spare


def read_synchronous(port: serial.Serial) -> None:
    def flush_input():
     for i in range(10):
        print(f"stop and flush remaining input {i}")
        _measurements = port.read(size=BUFFER_SIZE)
        if len(_measurements) == 0:
            break
        print(_measurements)

    # Flush input
    port.write(b"\np\n")
    flush_input()

    command = b"\nr-03-00\n"
    command = b"\nr\n"
    print(f"reset adc: {command.decode()}")
    flush_input()

    print("start measurement")
    port.write(b"\ns\n")
    time.sleep(0.1)
    begin_ns = time.monotonic_ns()
    counter = 0
    counter_after_decoder = 0
    decoder = ad_low_noise_float_2023_decoder.Decoder()
    DUMP = False
    if DUMP:
        f = pathlib.Path(__file__).with_suffix(".txt").open("w")
    for i in itertools.count():
        measurements = port.read(size=BUFFER_SIZE)
        if DUMP:
            f.write(f"READ BEGIN {i} {len(measurements)}\n")
            for measurement in measurements:
                f.write(f" {measurement:02X}")
            f.write(f"\nREAD END {i} \n\n")
        counter += len(measurements) // 3
        duration_ns = time.monotonic_ns() - begin_ns
        msg = f"{1e9*counter/duration_ns:0.1f} SPS {len(measurements)=}, duration_s={duration_ns/1e9}"
        print(msg)
        if DUMP:
            f.write(msg)
            f.write("\n")
            f.flush()

        decoder.push_bytes(measurements)
        while True:
            numpy_array = decoder.get_numpy_array()
            if DUMP:
                if numpy_array is not None:
                    f.write(f"{len(numpy_array)=}\n")
                f.write(f"{decoder.get_crc()=}\n")
                f.write(f"{decoder.get_errors()=}\n")
                f.write(f"{decoder.size()=}\n")
            if numpy_array is None:
                print(
                    f"   size={decoder.size()}, counter != counter_after_decoder: {counter} != {counter_after_decoder} + {counter-counter_after_decoder}"
                )
                if decoder.size() > DECODER_OVERFLOW_SIZE:
                    msg = f"Segment overflow {decoder.size()=}!"
                    print(msg)
                    raise OutOfSyncException(msg)
                break
            if decoder.get_crc() != 0:
                msg = f"ERROR crc={decoder.get_crc()}"
                print(msg)
                raise OutOfSyncException(msg)
            if decoder.get_errors() != 0:
                print(f"ERROR errors={decoder.get_errors()}")
            # print(decoder.get_errors())
            counter_after_decoder += len(numpy_array)
            for measurement_signed in numpy_array:
                REF_V = 5.0
                GAIN = 5.0  # 1.0, 2.0, 5.0, 10.0
                measurement_V = measurement_signed / (2**23) * REF_V / GAIN
                print(f"{measurement_signed:-8d}, {measurement_V:9.6f}V")
                break


def main():
    port = open_serial_port(VID, PID)

    while True:
        try:
            read_synchronous(port)
        except OutOfSyncException as e:
            print(f"{e!r}")


if __name__ == "__main__":
    main()
