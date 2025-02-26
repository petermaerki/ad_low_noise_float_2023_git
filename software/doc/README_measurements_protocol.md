# Measurements protocol

## Modes (Device -> Host)

* `MEASURE`

  Binary measurement data stream.

* `STATUS`

  Ascii status.
  
  Then switches into `IDLE`.

* `IDLE`

  No data. Waiting for command.


## Commands (Host -> Device)

* `s`: Start

  Start binary communication.

  New mode: `MEASURE`

* `p`: Stop

  Stop binary communication
  
  New mode: `STATUS`

* `r`: Reset

  Reset ADC with new parameters

  * Parameter 1: FLTR_OSR (Speed) as hex number

    See: [ADS127L21.pdf](../../hardware/components/ad/ADS127L21.pdf)

  * Parameter 2: MUX as hex number

    See: [ADS127L21.pdf](../../hardware/components/ad/ADS127L21.pdf)

    Example 97_656SPS: r-02-00<br>
    Example 48_828SPS: r-03-00

  * Parameter 3: `0`: `RESOLUTION24`, `1`: `RESOLUTION22`

    * `RESOLUTION24`: All 24 adc bits are transmitted.
    * `RESOLUTION22`: Only 22 adc bits are transmitted, the last two bits are `IN_t` and `IN_disable`.

  * Parameter 4: optional `additional_SPI_reads` may be provided.

    This third parameter is given as decimal number.<br>
    Example 48_828SPS with 2 additional_SPI_reads : r-03-00-2

 
  New mode: `STATUS`

## Encoding: `MEASURE`

* Sequence of words consisting of 3 bytes:

  * n measurements
  * 1 marker word
  * 1 status word

* A word may be:

  * A measurement.
    0xXX 0xXX 0xXX

    Measurements higher than `0x7F 0xFF 0xFB` will be clipped to `0x7F 0xFF 0xFB`.
    Distinguish from separator: 0x7F 0xFF 0xFF => 0x7F 0xFF 0xFE

    In Mode xy, the two last bytes of the adc value are overwritten by `IN_t` and `IN_disable`.

    The same as bitstreams:
    | bit | hex | Comment |
    | - | - | - |
    | `011111111111111111111111` | `7F FF FF` | Marker |
    | `011111111111111111111011` | `7F FF FB` | Max adc value |
    | `xxxxxxxxxxxxxxxxxxxxxxx1` | `xx xx x1` | `IN_disable`. Only if `RESOLUTION22`. | 
    | `xxxxxxxxxxxxxxxxxxxxxx1x` | `xx xx x2` | `IN_t`, Only if `RESOLUTION22`. | 

    * If `IN_disable` is `True`, the bit is `1`.
    * If `IN_t` is `True`, the bit is `1`.

  * A marker word: 0x7F 0xFF 0xFF

  * A status word:
    * byte 1: application status byte
    * byte 2: ads127l21 error byte
    * byte 3: crc byte

## Encoding: `STATUS`

* Sequence of lines
* First line: `BEGIN=1`
* Middle lines: `<label>=<value>`
* Last line: `END=1`


## Timings

```
INFO:ad_low_noise_float_2023:  status: SEQUENCE_LEN_MIN=1000
INFO:ad_low_noise_float_2023:  status: SEQUENCE_LEN_MAX=30000
INFO:ad_low_noise_float_2023:pcb_params.register_filter1=SPS_97656
```

| Name | Time | Calculation | Comment |
| - | - | - | - |
| `tsmin` | 10ms | 1_000/97_656 | Sequence size min |
| `tsmax` | 307ms | 30_000/97_656 | Sequence size max |
| `tusb` | 0.43ms | 128/3/97_656 | USB buffer size in Pico: CFG_TUD_CDC_TX_BUFSIZE (128) |
| `tpoll` | 0.5ms | 1/2kHz | USB full speed maximum polling rate is 1kHz. We assume 2kHz. |
| `tfifo` | 335ms | 32_768/97_656 | FIFO_LENGTH = 32768 |
| | 0.01ms | `tpoll` or less | Driver armed. Delay till actual e arrives. Min. |
| | 642ms | tsmax+tfifo+tusb+tpoll | Driver armed. Delay till actual measurements arrives. Max. |
| | 0.01ms | 1/97_656 | Delay between measurement and IN_t. Min. (Half of sequence written, IN_t triggers end of sequence) |
| | 10ms | `tsmin` | Delay between measurement and IN_t. Max. (Sequence just started, need to wait for SEQUENCE_LEN_MIN to end end of sequence) |

