# Phoenix Drone Flight Software
Flight software for the Phoenix Drone project, running on an ATmega328p.

![Phoenix](phoenix.png)

## Setup
In theory, you can use any operating system to build. However, we are using mainly OS X (10.10 and 10.11) and Ubuntu 14.04. Windows should work with no issues using `cygwin` and `winavr`.

The following steps are required to configure the board:
1. Install the required packages/software
2. Configure the environment
3. Configure the AVRISP mkII (Optional)

After configuring the board, you can then proceed with building the project and programming the board.

### Required Packages
To build the software and program the microcontroller, you must have the `avrdude` and `avr-gcc` packages installed (in addition to standard tools like `make`).

>**NOTE:** On any of the operating systems, an install of the Arduino IDE ([https://www.arduino.cc/en/main/software](https://www.arduino.cc/en/main/software)) will include all of the command line tools necessary, but they may be in strange locations. You can always just make symbolic links to your desired directories, though.

#### Ubuntu
On Ubuntu (or any Debian-based version of Linux):
```bash
sudo apt-get install avr-gcc
```
```bash
sudo apt-get install avrdude
```

#### Windows
On Windows install `WinAVR` ([winavr.sourceforge.net/](http://winavr.sourceforge.net/)) and use `cygwin` to emulate a Terminal ([www.cygwin.com/](https://www.cygwin.com/)).

#### OS X
Install Homebrew ([http://brew.sh/](http://brew.sh/)) and then:
```bash
brew install avrdude
```
<!-- ```bash
brew tap osx-cross/avr
brew install avr-libc
``` -->

> **Caveat:** To use a non-AVR programmer, such as the Olimex version of the AVRISP mkII, you will need to use version 5.11.1 or lower of `avrdude`, which is not obtainable via Homebrew. Either manually build it from source, or you can fool Homebrew into using the 5.11.1 source tarball instead of its own by changing the SHA256 checksum in the `avrdude.rb` formula.

Install the Arduino IDE either via Homebrew Cask:
```bash
brew cask install arduino
```

Or manually download it from the Arduino website: https://www.arduino.cc/download_handler.php?f=/arduino-1.6.7-macosx.zip

Include Arduino's binaries in your `PATH`:

In your `~/.bashrc`:
```bash
export PATH="/Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/:$PATH"
```

Or if using `fish`, in your `~/.config/fish/config.fish`:
```bash
set -g -x PATH '/Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/' $PATH
```

### Configure the Environment
Next you need to set up a few environmental variables:

In `bash`:
```bash
export PHOENIX_PROGRAMMER=avrispmkII
export PHOENIX_PORT=usb
```

In `fish`:
```bash
set -g -x PHOENIX_PROGRAMMER 'avrispmkII'
set -g -x PHOENIX_PORT 'usb'
```

1. `PHOENIX_PROGRAMMER` is either `avrispmkII` for the AVRISP mkII (including the Olimex version), `usbtiny` for the Sparkfun AVR Pocket Programmer, or `arduino` if you are using the USB port of an Arduino Uno. `PHOENIX_PROGRAMMER` defaults to `avrispmkII` if not set.

2. `PHOENIX_PORT` is the `/dev/...` file for your configuration. If no `/dev/ttyXXX` file exists, try `usb` if you are using the AVRISP mkII. If you are using the Arduino, it may show up as something like `/dev/ttyACM0` on OS X and Linux, or `com5`, or another COM port on Windows. `PHOENIX_PORT` defaults to `usb` if not set.

### Oscillator Setup
There are two make commands: `make setup_1MHz` and `make setup_8MHz` that will set the microcontroller's internal oscillator to either 1MHz or 8MHz. With a fresh batch of chips from the factory, the default is to use the divide-by-eight clock divider, which will therefore use the 1MHz clock. The default fuses corresponding to the 1MHz clock are `(H:07, E:D9, L:62)`.

Running the 8MHz setup (`make setup_8MHz`) only changes the low fuse to remove the default divide-by-eight clock divider. This allow us to run at the higher speed, if needed.

This is semi-optional because we don't *really* need to run at 8MHz. During testing we have been using 8MHz, but there is no reason we couldn't run at 1MHz or lower.

If you decide not to run any `make setup_...` or run `make setup_1MHz`, which will then use the 1MHz clock, remember to modify the `F_CPU` macro in [phoenix.h](./phoenix.h) to reflect the correct clock speed.

### AVRISP mkII Setup
This applies if you are using the AVRISP mkII on Ubuntu. You may try to use the programmer without first configuring it, but if you run into issues then the steps below need to be followed.

The steps are also outlined [here](http://stackoverflow.com/questions/5412727/avrisp-mkii-doesnt-work-with-avrdude-in-linux).

#### Create a new `udev` rules file:
```bash
cd /etc/udev/
sudo touch avrisp.rules
```
The file should contain the following:

```bash
SUBSYSTEM!="usb_device", ACTION!="add", GOTO="avrisp_end"

# Atmel Corp. JTAG ICE mkII
ATTR{idVendor}=="03eb", ATTR{idProduct}=="2103", MODE="660", GROUP="dialout"
# Atmel Corp. AVRISP mkII
ATTR{idVendor}=="03eb", ATTR{idProduct}=="2104", MODE="660", GROUP="dialout"
# Atmel Corp. Dragon
ATTR{idVendor}=="03eb", ATTR{idProduct}=="2107", MODE="660", GROUP="dialout"

LABEL="avrisp_end"
```

Next, create a hard link for the file:
```bash
cd /etc/udev/rules.d/
ln ../avrisp.rules 60-avrisp.rules
```
<br>

#### Add yourself to the `dialout` group
```bash
usermod -a -G dialout $USER
```

#### Restart the `udev` service
```bash
sudo service udev restart
sudo udevadm control --reload-rules; udevadm trigger
```

And finally restart the computer
```bash
sudo reboot
```

#### Getting information about the device
There are a number of different ways to gain information about the device:

To see the device vendor ID and device ID after plugging it in:
```bash
dmesg | tail
```

Very detailed information about the device (vendorID:deviceID):
```bash
lsusb -v -d 03eb:2104
```
* `03eb` is the vendor ID for Atmel
* `2104` is the device ID for the AVRISP mkII
