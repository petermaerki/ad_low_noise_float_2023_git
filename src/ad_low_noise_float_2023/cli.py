import argparse
import logging

from .ad import AdLowNoiseFloat2023, LOGGER_NAME
from .constants import PcbParams, RegisterFilter1, RegisterMux

logger = logging.getLogger(LOGGER_NAME)


def main():
    logging.basicConfig()
    logger.setLevel(logging.DEBUG)

    parser = argparse.ArgumentParser(description="ad_low_noise_float_2023")
    parser.add_argument(
        "--scale_factor",
        type=float,
        default=1.0,
        help="Scale Factor",
    )
    parser.add_argument(
        "--SPS",
        type=str,
        default=RegisterFilter1.SPS_97656.name,
        help=f"Samples per second ({RegisterFilter1.values_text()})",
    )
    parser.add_argument(
        "--Mux",
        type=str,
        default=RegisterMux.NORMAL_INPUT_POLARITY.name,
        help=f"Mux register ({RegisterMux.values_text()})",
    )
    parser.add_argument(
        "--resolution22",
        type=int,
        default=0,
        help="0: resolution24 (higher resolution for flicker measurement), 1: resolution22 (lower resolution for labber)",
    )
    parser.add_argument(
        "--additional_SPI_reads",
        type=int,
        default=0,
        help="Additional SPI reads (used to debug SPI performance)",
    )
    args = parser.parse_args()

    adc = AdLowNoiseFloat2023()
    for measurements in adc.iter_measurements_V(
        pcb_params=PcbParams(
            scale_factor=args.scale_factor,
            register_filter1=RegisterFilter1.factory(args.SPS),
            register_mux=RegisterMux.factory(args.Mux),
            resolution22=args.resolution22,
            additional_SPI_reads=args.additional_SPI_reads,
        ),
        total_samples=2_000_000,
        cb_out_of_sync=lambda: print("out_of_sync"),
    ):
        error_codes = adc.pcb_status.list_errors(
            error_code=measurements.errors, inclusive_status=True
        )
        elements = []
        elements.append(f"{measurements.adc_value_V[0]:6.2f}V")
        elements.append(f"{len(measurements.adc_value_V)}")
        if measurements.IN_disable is not None:
            elements.append(f"IN_disable={measurements.IN_disable[0]}")
        if measurements.IN_t is not None:
            elements.append(f"IN_t={measurements.IN_t[0]}")
        elements.append(f"{int(measurements.errors):016b}")
        elements.append(f"{error_codes}")
        print(" ".join(elements))


if __name__ == "__main__":
    main()
