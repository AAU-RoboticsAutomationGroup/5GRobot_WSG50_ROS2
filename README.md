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
|         Command         | Code | Parameters |
|-------------------------|------|------------|
| Preposition Gripper     | 0x21 | <ul><li>B0 - Flag Bits <ul><li>D7..2 Unused (set to 0)</li><li>D1 - Stop on Block: stop (1) or clamp (0) on block</li><li>D0 - Movement Type: relative (1) or absolute to closed fingers (0)</li></ul></li><li>B1..4 - Width (mm)</li><li>B5..8 - Speed (mm/s)</li></ul> |
| Grasp Part              | 0x25 | <ul><li>B0..3 - Part Width (mm)</li><li>B4..7 - Speed (mm/s)</li></ul> |
| Release Part            | 0x26 | <ul><li>B0..3 - Open Width (mm)</li><li>B4..7 - Speed (mm/s)</li></ul> |
| Disconnect Announcement | 0x07 | None |
| Ack Fault Condition     | 0x24 | B0..2 - Ack key (String containing 'ack' = 0x61 0x63 0x6B) |
| Get System State (WIP)  | 0x40 | Sent Parameters: <ul><li>B0 - Flag Bits <ul><li>D7..2 Unused (set to 0)</li><li>D1 - Change-Sensitive Update: Update on change (1) or always (0)</li><li>D0 - Automatic Update: enabled (1) or disabled (0)</li></ul></li><li>B1..2 - Period between auto-sent packets (ms)</li></ul>Returned Parameters:<ul><li>B0..3 - Bit Vector (for definition see Command Set Reference Manual Appendix B)</li></ul> |

---
## Running the ROS2 Package in Docker

For ease of use the package has been implemented in docker and can be added to a docker-compose file or run from it's own docker-compose file.
Firstly, the repository has to be cloned into a desired folder

    $ git clone https://github.com/AAU-RoboticsAutomationGroup/5GRobot_WSG50.git

Once the repository has been downloaded, enter the docker folder.

    $ cd 5GRobot_WSG50/docker

Then simply build the container and run it.

    $ docker-compose build

    $ docker-compose up

If the gripper is connected the fingers should move into the home position(open). The ros2 package will log if this is done successfuly or not. The package will create a set of topics that can be used to control the gripper. These can be found in the list below. 

|                 | Description  | Message Type  | Structure & Units|
|-----------------|-------------|-------------|-------------|
| /wsg50/connect       | Starts a TCP/IP connection to the WSG50 gripper | std_msgs.msg/Empty|           |
| /wsg50/disconnect    | Ends a TCP/IP connection to the WSG50 gripper   | std_msgs.msg/Empty    | |
| /wsg50/preposition   | Moves the fingers into position for grasping | interface_wsg.msg/MoveFingers | float64 width [mm], float64 speed [mm/s]|
| /wsg50/grasp         | Closes the fingers around a part.    | interface_wsg.msg/MoveFingers    | float64 width [mm], float64 speed [mm/s]     |
| /wsg50/release       | Releases a previously grasped part.       | interface_wsg.msg/MoveFingers       | float64 width [mm], float64 speed [mm/s]      |
| /wsg50/gripper_state | Gets the state of the gripper(see Appendix B in the command set reference manual)    | interface_wsg.msg/GripperState    | std_msgs/Header header, string[] state_flags, float64 gripper_width [mm]   |