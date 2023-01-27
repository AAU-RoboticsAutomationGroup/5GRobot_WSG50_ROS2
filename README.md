# 5GRobot_WSG50

The gripper is currently set to operate with DHCP, the MAC-address is: 00:50:c2:cf:20:36
The IP-address will be set to XX.XX.XX.21 on deployment.

## Command protocol
The gripper follows a bespoke command protocol. A ROS-package is being drafted to give access to this, but a short
primer follows. For deeper reference, refer to - 5GRobot_WSG50/WSG50_Command_Set_Reference_Manual.pdf

Data transmissio is based on little endian representation of multibyte words - 0x1234 is transmitted as 0x34 0x12.
Float values are represented by a 4-byte IEEE 754 single precision number.
D31     - Sign
D30..23 - Exponent
D22..0  - Mantissa

### Packet Structure
Packets generally have the following structure:

| [0xAA 0xAA 0xAA] | [0xXX] | [0xXX 0x00] | [0xXX .. 0xXX] | [0xXX 0xXX] |
|------------------|--------|-------------|----------------|-------------|
|3-byte preamble (always AA AA AA)  |  1-byte Command ID  |  2-byte Size (little endian)  |  N-byte Payload  |  2-byte checksum |

#### Commands Used

Preposition gripper
grasp part
release part
disconnect
remove error
get gripper state WIP
