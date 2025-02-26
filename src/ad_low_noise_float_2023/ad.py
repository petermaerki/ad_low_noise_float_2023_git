# https://github.com/petermaerki/ad_low_noise_float_2023_git/blob/hmaerki/evaluation_software/evaluation_software/cpp_cdc/2025-03-30a_ads127L21/src/reader.py
import dataclasses
import logging
import re
import time
import typing

import ad_low_noise_float_2023_decoder
import numpy as np
import serial
import serial.tools.list_ports

from .constants import AD_FS_V, PcbParams

LOGGER_NAME = "ad_low_noise_float_2023"

logger = logging.getLogger(LOGGER_NAME)


RE_STATUS_BYTE_MASK = re.compile(r"STATUS_BYTE_MASK=0x(\w+)")


class OutOfSyncException(Exception):
    pass


@dataclasses.dataclass(frozen=True)
class MeasurementSequence:
    errors: int
    adc_value_V: np.ndarray
    IN_disable: typing.Optional[np.ndarray]
    IN_t: typing.Optional[np.ndarray]


@dataclasses.dataclass
class BcbStatus:
    """
    status: BEGIN=1
    status: PROGRAM=ad_low_noise_float_2023(0.3.3)
    status: REGISTER_FILTER1=0x02
    status: REGISTER_MUX=0x00
    status: SEQUENCE_LEN_MIN=1000
    status: SEQUENCE_LEN_MAX=32000
    status: ERROR_MOCKED=1
    status: ERROR_MOCKED=1
    status: ERROR_ADS127_MOD=2
    status: ERROR_ADS127_ADC=4
    status: ERROR_FIFO=8
    status: ERROR_ADS127_SPI=16
    status: ERROR_ADS127_POR=32
    status: ERROR_ADS127_ALV=64
    status: ERROR_OVLD=128
    status: ERROR_STATUS_J42=256
    status: ERROR_STATUS_J43=512
    status: ERROR_STATUS_J44=1024
    status: ERROR_STATUS_J45=2048
    status: ERROR_STATUS_J46=4096
    status: END=1
    """

    settings: typing.Dict[str, str] = dataclasses.field(default_factory=dict)
    error_codes: typing.Dict[int, str] = dataclasses.field(default_factory=dict)

    def add(self, line: str) -> None:
        key, value = line.split("=", 1)
        self.add_setting(key.strip(), value.strip())

    def add_setting(self, key: str, value: str) -> None:
        assert isinstance(key, str)
        assert isinstance(value, str)
        self.settings[key] = value

        if key.startswith("ERROR_"):
            try:
                value_int = int(value, 0)
                bit_position = 0
                while value_int > 1:
                    value_int >>= 1
                    bit_position += 1
                self.error_codes[bit_position] = key
            except ValueError:
                logger.warning(f"Invalid error code: {key}={value}")

    def validate(self) -> None:
        assert self.settings["BEGIN"] == "1"
        assert self.settings["END"] == "1"

    def list_errors(self, error_code: int, inclusive_status: bool) -> typing.List[str]:
        """
        Returns a list of error messages for the given error code.
        """
        assert isinstance(error_code, int)
        # return a list of bit positions which are set in error_code
        error_bits = [i for i in range(32) if (error_code & (1 << i)) != 0]

        error_strings = [
            self.error_codes.get(bit_position, f"{bit_position}?")
            for bit_position in error_bits
        ]
        if not inclusive_status:
            error_strings = [
                x for x in error_strings if not x.startswith("ERROR_STATUS_")
            ]
        return error_strings

    @property
    def gain_from_jumpers(self) -> float:
        status_J42_J46 = int(self.settings["STATUS_J42_J46"], 0)
        status_J42_J43 = status_J42_J46 & 0b11
        return {
            0: 1.0,
            1: 2.0,  # J42
            2: 5.0,  # J43
            3: 10.0,  # J42, J43
        }[status_J42_J43]


class AdLowNoiseFloat2023:
    PRINTF_INTERVAL_S = 10.0
    VID = 0x2E8A
    PID = 0x4242
    MEASURMENT_BYTES = 3
    COMMAND_START = "s"
    COMMAND_STOP = "p"
    COMMAND_MOCKED_ERROR = "e"
    COMMAND_MOCKED_CRC = "c"

    SEQUENCE_LEN_MAX = 32_000
    BYTES_PER_MEASUREMENT = 3
    DECODER_OVERFLOW_SIZE = 4 * BYTES_PER_MEASUREMENT * SEQUENCE_LEN_MAX

    def __init__(self) -> None:
        self.serial = self._open_serial()
        self.success: bool = False
        self.pcb_status = BcbStatus()
        self.decoder = ad_low_noise_float_2023_decoder.Decoder()
        self.connected = False
        """
        False: While we still connect to the pico
        True: We received all the configuration parameters from the pico and are receiving samples. 
        """

    def _open_serial(self) -> serial.Serial:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == self.VID:
                if port.pid == self.PID:
                    return serial.Serial(port=port.device, timeout=1.0)

        raise ValueError(
            f"No board with VID=0x{self.VID:02X} and PID=0x{self.PID:02X} found!"
        )

    def close(self) -> None:
        self.connected = False
        self.serial.close()

    def drain(self) -> None:
        while True:
            line = self.serial.read()
            if len(line) == 0:
                return

    def read_status(self) -> bool:
        self.success, self.pcb_status = self._read_status_inner()
        self.pcb_status.validate()
        return self.success

    def _read_status_inner(self) -> typing.Tuple[bool, BcbStatus]:
        """
        return True on success
        """
        status = BcbStatus()
        while True:
            line_bytes = self.serial.readline()
            if len(line_bytes) == 0:
                return False, status
            line = line_bytes.decode("ascii").strip()
            status.add(line)
            logger.info(f"  status: {line}")
            if line == "END=1":
                return True, status

    def test_usb_speed(self) -> None:
        begin_ns = time.monotonic_ns()
        counter = 0
        while True:
            measurements = self.serial.read(size=1_000_000)
            # print(f"len={len(measurements)/3}")
            self.decoder.push_bytes(measurements)

            while True:
                numpy_array = self.decoder.get_numpy_array()
                if numpy_array is None:
                    print(".", end="")
                    break
                if self.decoder.get_crc() != 0:
                    logger.error(f"ERROR crc={self.decoder.get_crc()}")
                if self.decoder.get_errors() not in (0, 8, 72):
                    logger.error(f"ERROR errors={self.decoder.get_errors()}")

                counter += len(numpy_array)
                duration_ns = time.monotonic_ns() - begin_ns
                logger.info(f"{1e9 * counter / duration_ns:0.1f} SPS")

                # counter += len(measurements) // 3
                # duration_ns = time.monotonic_ns() - begin_ns
                # print(f"{1e9*counter/duration_ns:0.1f} SPS")

        # Pico:197k  PC Peter 96k (0.1% CPU auslasung)

    def iter_measurements_signed32(
        self,
    ) -> typing.Iterable[typing.Tuple[int, np.ndarray]]:
        """
        Returns an array of the last measurement in raw 32 bit signed integers.
        """
        while True:
            measurements = self.serial.read(size=1_000_000)
            # print(f"len={len(measurements)/3}")
            self.decoder.push_bytes(measurements)

            while True:
                adc_value_ain_signed32 = self.decoder.get_numpy_array()
                if adc_value_ain_signed32 is None:
                    # print(".", end="")
                    if self.decoder.size() > self.DECODER_OVERFLOW_SIZE:
                        msg = "f'Segment overflow! decoder.size {self.decoder.size()} > DECODER_OVERFLOW_SIZE {self.DECODER_OVERFLOW_SIZE}'"
                        # print(msg)
                        raise OutOfSyncException(msg)
                    break
                # counter += len(adc_value_ain_signed32)
                if self.decoder.get_crc() != 0:
                    msg = f"ERROR crc={self.decoder.get_crc()}"
                    # print(msg)
                    raise OutOfSyncException(msg)

                errors = self.decoder.get_errors()
                error_strings = self.pcb_status.list_errors(
                    errors,
                    inclusive_status=False,
                )
                if len(error_strings) > 0:
                    msg = f"ERROR: {errors}: {' '.join(error_strings)}"
                    logger.error(msg)

                yield errors, adc_value_ain_signed32

    def _send_command(self, command: str) -> None:
        logger.info(f"send command: {command}")
        command_bytes = f"\n{command}\n".encode("ascii")
        self.serial.write(command_bytes)

    def _send_command_reset(self, pcb_params: PcbParams) -> None:
        assert isinstance(pcb_params, PcbParams)
        msg = f"send command reset: {pcb_params.register_filter1!r} {pcb_params.register_mux!r}"
        logger.info(msg)
        command_reset = f"r-{pcb_params.register_filter1:02X}-{pcb_params.register_mux:02X}-{pcb_params.resolution22:d}-{pcb_params.additional_SPI_reads:d}"
        self._send_command(command_reset)

    def connect(self, pcb_params: PcbParams) -> None:
        self.connected = False
        RETRY_MAX = 2
        for retry0 in range(999):
            try:
                self._connect_inner(pcb_params=pcb_params)
                break
            except Exception as e:
                logger.error(f"retry={retry0 + 1}({RETRY_MAX}): {e}")
                if retry0 >= RETRY_MAX:
                    raise e

        self._assert_pico_firmware_version(
            required_pico_firmware_version=pcb_params.required_pico_firmware_version
        )

    def _assert_pico_firmware_version(
        self, required_pico_firmware_version: str
    ) -> None:
        settings_program = self.pcb_status.settings["PROGRAM"]
        if (settings_program < required_pico_firmware_version) or (
            len(settings_program) < len(required_pico_firmware_version)
        ):
            raise ValueError(
                f"Found '{settings_program}' but required at least '{required_pico_firmware_version}'!"
            )

    def _connect_inner(self, pcb_params: PcbParams) -> None:
        logger.info("Started")
        self._send_command(AdLowNoiseFloat2023.COMMAND_STOP)
        self.drain()
        self.decoder.clear()
        self._send_command_reset(pcb_params=pcb_params)
        self.read_status()
        self._send_command(AdLowNoiseFloat2023.COMMAND_START)
        self.connected = True
        logger.info(
            f"iter_measurements(): gain={self.pcb_status.gain_from_jumpers:0.3f}"
        )

    def iter_measurements_V(
        self,
        pcb_params: PcbParams,
        total_samples: typing.Optional[int] = None,
        cb_out_of_sync: typing.Optional[typing.Callable] = None,
        do_connect: bool = True,
    ) -> typing.Iterable[MeasurementSequence]:
        """
        Returns an array of the last measurement in floats scaled to 'V'.

        If 'total_samples' is None: indefinitely.
        """
        assert isinstance(pcb_params, PcbParams)
        assert isinstance(total_samples, (int, type(None)))
        assert isinstance(cb_out_of_sync, (typing.Callable, type(None)))

        logger.info(f"pcb_params.resolution22={pcb_params.resolution22}")
        logger.info(
            f"pcb_params.additional_SPI_reads={pcb_params.additional_SPI_reads}"
        )
        logger.info(f"pcb_params.scale_factor={pcb_params.scale_factor:0.1f}V")
        logger.info(f"pcb_params.register_filter1={pcb_params.register_filter1.name}")
        logger.info(f"pcb_params.register_mux={pcb_params.register_mux.name}")
        logger.info(f"total_samples={total_samples}")

        if do_connect:
            self.connect(pcb_params=pcb_params)

        actual_sample_count = 0
        printf_interval_s = 10.0
        next_print_s = time.monotonic() + 2.0
        last_sample_count = 0
        factor = (
            pcb_params.scale_factor
            * AD_FS_V
            / self.pcb_status.gain_from_jumpers
            / (2**23)
        )

        def print_interval():
            """
            print status information every 10s
            """
            elements = []
            if total_samples is not None:
                elements = [
                    f"{actual_sample_count / total_samples * 100:3.0f}%",
                    f"{actual_sample_count:10,d}({total_samples:,})samples",
                    # f"{actual_sample_count:10,d} samples of {total_samples:,}",
                ]

            elements.extend(
                [
                    f"{(actual_sample_count - last_sample_count) / printf_interval_s:,.0f}SPS",
                    f"{adc_value_V[0]:3.6f}V",
                ]
            )

            msg = " ".join(elements)
            msg = msg.replace(",", "'")
            logger.info(msg)

        while True:
            try:
                for errors, adc_value_ain_signed32 in self.iter_measurements_signed32():
                    if total_samples is not None:
                        if actual_sample_count > total_samples:
                            print_interval()
                            return

                    actual_sample_count += len(adc_value_ain_signed32)
                    adc_value_V = np.multiply(
                        factor,
                        adc_value_ain_signed32,
                        dtype=np.float32,
                    )

                    if next_print_s < time.monotonic():
                        next_print_s += printf_interval_s
                        print_interval()
                        last_sample_count = actual_sample_count

                    IN_disable = None
                    IN_t = None
                    if pcb_params.resolution22:
                        IN_disable = (adc_value_ain_signed32 & 1).astype(bool)
                        IN_t = ((adc_value_ain_signed32 >> 1) & 1).astype(bool)

                    yield MeasurementSequence(
                        errors=errors,
                        adc_value_V=adc_value_V,
                        IN_disable=IN_disable,
                        IN_t=IN_t,
                    )

            except OutOfSyncException as e:
                logger.error(f"OutOfSyncException: {e}")
                bytes_purged = self.decoder.purge_until_and_with_separator()
                logger.info(f"Purged {bytes_purged} bytes!")
                if cb_out_of_sync is not None:
                    cb_out_of_sync()
