tud_cdc_write_available
CFG_TUSB_DEBUG


examples/device/cdc_uac2/README.md

// Audio format type I specifications
#if defined(__RX__)
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE                         48000     // 16bit/48kHz is the best quality for Renesas RX
#else
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE                         96000     // 24bit/96kHz is the best quality for full-speed, high-speed is needed beyond this
#endif

24bit/96kHz is the best quality for full-speed, high-speed is needed beyond this


===================
CFG_TUD_AUDIO

https://github.com/hathach/tinyusb/discussions/2054
  Dual UAC2_headsets

https://github.com/pschatzmann/Adafruit_TinyUSB_Arduino/discussions/7
  Pico PSchatzmann
  
https://github.com/raspberrypi/tinyusb/tree/master/examples/device/audio_4_channel_mic
https://github.com/raspberrypi/tinyusb/blob/master/examples/device/audio_test
  TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR

https://github.com/raspberrypi/tinyusb/tree/pico/examples/device/uac2_headset
  CFG_TUD_AUDIO_ENABLE_EP_IN / CFG_TUD_AUDIO_ENABLE_EP_OUT


===================
https://github.com/raspberrypi/tinyusb/tree/master/examples/device/audio_4_channel_mic

/home/maerki/.pico-sdk/sdk/2.1.1/lib/tinyusb/src/class/audio/audio_device.c
  // Endpoint size must larger than packet size
  TU_ASSERT(packet_sz_tx_max <= audio->ep_in_sz);
   88 <= 22

===================
https://github.com/Panda381/PicoVGA
  PicoVGA - VGA/TV display on Raspberry Pico
  https://github.com/Panda381/PicoVGA/blob/main/_tinyusb/class/audio/audio_device.c#L50
  
=====================================
https://github.com/dawsonjon/PicoRX/tree/master
https://101-things.readthedocs.io/en/latest/breadboard_radio_part3.html
 ==> Sehr gut!

===========================================
https://github.com/ArduCAM/Arducam_Mega/blob/e281fd26e16fb62e047fdec5318f89fea4193e3d/examples/Pico/pico4ml/example/micro_speech_app/src/micro_speech_apps.h#L11

========================================
https://github.com/shermp/Pico-ASHA/blob/a33ff94640bf9758211f1303fc4b7c638b760ab1/src/usb_audio.cpp#L75
Was ist das?

==============
https://github.com/hei5enbrg/pico-usb-spdif/blob/33c911046d9290c01397871a6bc7b61a183a6acf/usb_spdif.c#L5
  Nur 1 commit
======================
https://github.com/kayvontabrizi/microphone-library-for-pico/blob/3f351b64f983a5d5d30413a7846bda4d9077f426/examples/usb_microphone/usb_microphone.c#L30
https://github.com/kayvontabrizi/microphone-library-for-pico/blob/main/examples/usb_microphone/usb_microphone.c#L30
  Microphone Library for Pico
  51 commits
https://github.com/kayvontabrizi/microphone-library-for-pico/blob/main/notes.md
  
  
  
=========
Success: fork_PicoRX, pico-sdk 2.0.0, rp2350
  firmware: https://github.com/hmaerki/fork_PicoRX/actions/runs/14498192556
  python plot_audio_samples.py 15000 -> recording with exactly 0!
  audacity: recoding with 0

Success: fork_PicoRX, pico-sdk 2.0.0, rp2040
  firmware: https://github.com/hmaerki/fork_PicoRX/actions/runs/14498192556
  python plot_audio_samples.py 15000 -> recording with noise!
  audacity: recoding with noice

Success: fork_PicoRX, pico-sdk 2.1.0, rp2040
  firmware: https://github.com/hmaerki/fork_PicoRX/actions/runs/14512836303
  python plot_audio_samples.py 15000 -> recording with exactly 0! (picoprobe?)
  
Fail: fork_PicoRX, pico-sdk 2.1.1, rp2040
  firmware: https://github.com/hmaerki/fork_PicoRX/actions/runs/14512917372
  dmesg 1:1: usb_set_interface failed (-110)

Fail: fork_PicoRX, pico-sdk 2.1.1, rp2040
  tusb_init(BOARD_TUD_RHPORT, &dev_init)
  firmware: https://github.com/hmaerki/fork_PicoRX/actions/runs/14518480860
  dmesg 1:1: usb_set_interface failed (-110)
  
  
================
Documentation
https://www.usb.org/document-library/usb-20-specification
https://www.usb.org/defined-class-codes
https://usb.org/sites/default/files/usb_20_20240927.zip
https://www.usb.org/sites/default/files/audio10.pdf
https://www.usb.org/sites/default/files/USB%20Audio%20v4.0.zip

https://www.cyberciti.biz/faq/linux-how-do-i-list-all-usb-devices/
https://wiki.ubuntu.com/Kernel/Debugging/USB
Documentation/usb/error-codes.txt
https://www.kernel.org/doc/Documentation/usb/error-codes.txt

https://www.silabs.com/documents/public/application-notes/AN295.pdf
  USB AUDIO CLASS TUTORIAL  

Defined Class Codes
* Base Class 00h (Device)
* Base Class 01h (Audio)
  These class codes may only be used in Interface Descriptors
* Base Class 0Ah (CDC-Data)
* Base Class 10h (Audio/Video Devices)
  composite devices
* Base Class EFh (Miscellaneous)
* Base Class FEh (Application Specific)
