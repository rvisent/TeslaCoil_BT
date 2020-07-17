| Supported Targets | ESP32 |
| ----------------- | ----- |

# Musical Tesla Coil firmware

This code allows direct streaming of Bluetooth audio to a Tesla Coil driver and power amplifier through pin IO32.

It is derived from bt_stream demo by Expressif, which handles the connection to the BT client and provides a stream of stereo, 16 bit PCM data.

The modified code converts the stream to mono and interpolates it to the resonant frequency of the Tesla Coil, which in my case is close to 770 kHz. The actual frequency can be fine adjusted with a potentiometer wired to the ADC1 input of the ESP32.

After interpolation, the audio sequence is reduced to a 1 bit flow using a 2nd order sigma-delta approach (function sigma_delta_encode in bt_app_core.c). Then the sequence is doubled, putting a 0 bit after each original bit.

Finally, the ~1540 kHz stream is output on a physical digital output using the I2S interface, clocked by the high accuracy APLL. A tricky routine adjust_pll allows to reconfigure the frequency on the fly when the potentiometer is adjusted, without requiring to reinitialize the I2S subsystem as standard APIs require.


Tesla Coil center frequency is defined in main.c, line 51 (long fOut = xxx).

A frequency much higher than this may not be supported without software optimizations, because the two cores are already quite loaded.

## Build instructions (Windows - similar on Linux)

Install the Expressif esp-idf environment and enter the ESP-IDF command prompt

Enter the project directory TeslaCoil_BT

give the command "idf.py build"

connect ESP32 target with USB, check the configured serial port (in my case COM13)

download code with command "idf.py flash -p COM13"

you can see debug info with command "idf.py monitor -p COM13"

all previous commands can be combined after a modification, e.g. "idf.py build flash monitor -p COM13"


Follows the original README.md from Expressif bt_stream demo
------------------------------------------------------------

ESP-IDF A2DP-SINK demo
======================

Demo of A2DP audio sink role

This is the demo of API implementing Advanced Audio Distribution Profile to receive an audio stream.

This example involves the use of Bluetooth legacy profile A2DP for audio stream reception, AVRCP for media information notifications, and I2S for audio stream output interface.

Applications such as bluetooth speakers can take advantage of this example as a reference of basic functionalities.

## How to use this example

### Hardware Required

To play the sound, there is a need of loudspeaker and possibly an external I2S codec. Otherwise the example will only show a count of audio data packets received silently. Internal DAC can be selected and in this case external I2S codec may not be needed.

For the I2S codec, pick whatever chip or board works for you; this code was written using a PCM5102 chip, but other I2S boards and chips will probably work as well. The default I2S connections are shown below, but these can be changed in menuconfig:

| ESP pin   | I2S signal   |
| :-------- | :----------- |
| GPIO22    | LRCK         |
| GPIO25    | DATA         |
| GPIO26    | BCK          |

If the internal DAC is selected, analog audio will be available on GPIO25 and GPIO26. The output resolution on these pins will always be limited to 8 bit because of the internal structure of the DACs.

### Configure the project

```
idf.py menuconfig
```

* Set the use of external I2S codec or internal DAC for audio output, and configure the output PINs under A2DP Example Configuration

* Enable Classic Bluetooth and A2DP under Component config --> Bluetooth --> Bluedroid Enable

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output.

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

After the program is started, the example starts inquiry scan and page scan, awaiting being discovered and connected. Other bluetooth devices such as smart phones can discover a device named "ESP_SPEAKER". A smartphone or another ESP-IDF example of A2DP source can be used to connect to the local device.

Once A2DP connection is set up, there will be a notification message with the remote device's bluetooth MAC address like the following:

```
I (106427) BT_AV: A2DP connection state: Connected, [64:a2:f9:69:57:a4]
```

If a smartphone is used to connect to local device, starting to play music with an APP will result in the transmission of audio stream. The transmitting of audio stream will be visible in the application log including a count of audio data packets, like this:

```
I (120627) BT_AV: A2DP audio state: Started
I (122697) BT_AV: Audio packet count 100
I (124697) BT_AV: Audio packet count 200
I (126697) BT_AV: Audio packet count 300
I (128697) BT_AV: Audio packet count 400

```

Also, the sound will be heard if a loudspeaker is connected and possible external I2S codec is correctly configured. For ESP32 A2DP source example, the sound is noise as the audio source generates the samples with a random sequence.

## Troubleshooting
* For current stage, the supported audio codec in ESP32 A2DP is SBC. SBC data stream is transmitted to A2DP sink and then decoded into PCM samples as output. The PCM data format is normally of 44.1kHz sampling rate, two-channel 16-bit sample stream. Other decoder configurations in ESP32 A2DP sink is supported but need additional modifications of protocol stack settings.
* As a usage limitation, ESP32 A2DP sink can support at most one connection with remote A2DP source devices. Also, A2DP sink cannot be used together with A2DP source at the same time, but can be used with other profiles such as SPP and HFP.
