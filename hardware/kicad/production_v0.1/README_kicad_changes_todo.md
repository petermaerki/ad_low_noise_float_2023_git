# Changes to the next pcb

## Add UART Debug Connector

See
* https://github.com/raspberrypi/documentation/blob/develop/documentation/asciidoc/microcontrollers/debug-probe/uart-connection.adoc
* https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf "1.2.3. GPIO Functions (Bank 0)"
* CMakeLists.txt
  ```text
  target_compile_definitions(xyz PRIVATE
    CFG_TUSB_DEBUG=1
    LIB_PICO_STDIO_UART=1
    PICO_STDIO_USB=0
    PICO_STDIO_UART=0
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=28
    PICO_DEFAULT_UART_RX_PIN=29
  )
  ```

2 male pins, close to "JST Debug Connector"

  * yellow <-> GPIO28/UART1-TX
  * red    <-> GPIO29/UART1-RX


## Add indicator LED - not implemented!

Green LED on GPIO25

* CMakeLists.txt
  ```text
  target_compile_definitions(xyz PRIVATE
    PICO_DEFAULT_LED_PIN=25
  )
  ```
