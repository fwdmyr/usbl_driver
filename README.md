# evologics_driver_cpp

### Prerequisites

1. [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) or newer
2. [ROS Melodic](http://wiki.ros.org/melodic)  or newer

### Overview

This package acts as an EVOLOGICS USBL modem device driver that establishes a TCP/IP connection and enables reading and writing.

It advertises a set of ROS topics for different USBL messages and exposes input topics that allow forwarding of burst and instant messages to the device.

It provides a set of polling requests that are sent to the device at a fixed rate and publishes the response to dedicated ROS topics.

### Supported USBL message types

- USBLLONG
- USBLANGLES
- USBLPHYD
- USBLPHYP
- BITRATE
- SRCLEVEL
- STATUS

### Supported polling commands

- BATTERYLEVEL
- RELATIVEVELOCITY
- SOUNDSPEED
- MULTIPATHSTRUCTURE

### Configuration

Enabled USBL message topics, polling commands and frequencies can be set in the config file
```sh
/conf/message_config.yaml
```

### Starting the modem drivers

When working with the DMACE USBL emulator, launch the set of ROS nodes with
```sh
roslaunch evologics_driver dmace_two_modems.launch
```

When working with the physical modems, launch the set of ROS nodes with
```sh
roslaunch evologics_driver two_modems.launch
```