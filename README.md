# gpio-ts
## Linux driver: streaming GPIO timestamps
###### Project source can be downloaded from [https://github.com/alex-konshin/gpio-ts](https://github.com/alex-konshin/gpio-ts)

### Author and Contributors
Alex Konshin <akonshin@gmail.com>

### Overview
This project contains source code of Linux driver that currently tested on Raspberry Pi 3, Banana Pi M3, ODROID C2 and I have plans to make it work on other Linux platforms.
The main goal of this driver is to simplify developing of applications that need to receive some high frequency signals from GPIO, for example, from RF or IR receivers.  
I use this driver for receiving and decoding messages from RF sensors like thermometers.
See my another project [https://github.com/alex-konshin/f007th-rpi](https://github.com/alex-konshin/f007th-rpi) that can use this driver.   

#### What this driver does? 
- The driver can serve several GPIO simultaneously. Each GPIO is represented by a separate device file `/dev/gpiots*`. Each GPIO file may have different fltering settings.
- The driver supports blocking and non-blocking reading and select/poll calls.
- When user application opens device file the driver starts listening of GPIO level changes and creates a stream of 4-byte items. 2 highest bits of an item are the status (0, 1, NOISE or LOST_DATA). Other bits represent the durations of this status in microseconds. Status LOST_DATA means lost interruption due to delays or buffer overflow.  
- The driver has built-in filter for received data. It considers too short or too long signals as noise. It also can ignore too short sequences of "good" signals. Basically it allows effectively separate good RF signals from noise for further decoding of them in user application. 
- User application actually receives only "good" sequences separated by "noise" items.
- User application can set parameters of the filter by calling ioctl() after opening the device file but before the very first call of select(), poll() or read().
- Application does not need root permissions to open this device file.  

### Files
| File | Description |
| :--- | :--- |
| `gpio-ts.c` | Source code of the driver.|
| `gpio-ts.h` | The driver's header file.|
| `Makefile` | Makefile for building this driver.|
| `README.md` | This file. |

### How to build
TBD
#### Building on Raspberry Pi
Install Linux headers - see instruction [https://www.raspberrypi.org/documentation/linux/kernel/headers.md](https://www.raspberrypi.org/documentation/linux/kernel/headers.md)    

Download this driver sources and build it:    
`cd ~`    
`git clone https://github.com/alex-konshin/gpio-ts.git`    
`cd gpio-ts`    
`make all`    
The driver file `gpio-ts.ko` is created in folder `~/gpio-ts`.

#### Building on ODROID C2
Download and setup Linux headers:    
`git clone --depth 1 https://github.com/hardkernel/linux.git -b odroidc2-3.14.y`    
`cd /lib/modules/$(uname -r)`    
`sudo ln -s /usr/src/linux-headers-$(uname -r) build`    

Download this driver sources and build it:    
`cd ~`    
`git clone https://github.com/alex-konshin/gpio-ts.git`    
`cd gpio-ts`    
`make all`    
The driver file `gpio-ts.ko` is created in folder `~/gpio-ts`.

#### Building on Banana Pi M3
Building Linux kernel module on BPI-M3 is currently a nightmare. I finally figured out how to cross-compile it on another machine and after some hacks I was able to load and use it. But I don't remember all steps I did to achieve it. To make your life easier I provided compiled driver for the current BPI-M3 kernel. Take it from folder `bpi-m3-kernel-3.4.39`.

### Loading and unloading the driver.
##### Important note for Banana Pi M3
Not all GPIOs allow to set interruptions handler. This is a hardware limitation. The following GPIO numbers are supported on BPI-M3:    
32, 33, 34, 35, 202, 203, 204, 205, 226, 227, 228, 229, 234, 360, 361, 362.    
See output of command `gpio readall` to find these GPIOs on the connector.

#### Module parameters:
##### gpios 
The comma-separated list of GPIOs that will be served by this driver. Device `/dev/gpiots*` will be created for each GPIO from this list.
##### min_duration
Minimum duration of the same level to be accepted. This is an optional parameter. It sets the default value that can be changed for a particular GPIO by user application.  
##### max_duration
Maximum duration of the same level to be accepted. This is an optional parameter. It sets the default value that can be changed for a particular GPIO by user application.
##### min_seq_len
Minimum sequence length to be accepted. This is an optional parameter. It sets the default value that can be changed for a particular GPIO by user application.

#### Examples:
`sudo insmod gpio-ts.ko gpios=27`    
`sudo insmod gpio-ts.ko gpios=21,27 min_duration=400 max_duration=1100 min_seq_len=100`    

##### Unloading the driver
`sudo rmmod gpio-ts`

