#################################### MODULES AND IMPORTED CLASSES ###########################################
import struct
import socket
import time
import sys

#################################### CONSTANTS AND GLOBAL VARIABLES #########################################
# #inital message with header for later id and payload to be appended

# command ids
HOMING_ID = 32
PREPOSITION_ID = 33
GRASP_ID = 37
RELEASE_ID = 38

# payload sizes
HOMING_SIZE = 1
PREPOSITION_SIZE = 9
GRASP_SIZE = 8
RELEASE_SIZE = 8

# preambles for each message type
HOMING_PREAMBLE = [170, 170, 170, HOMING_ID, HOMING_SIZE, 0]
PREPOSITION_PREAMBLE = [170, 170, 170, PREPOSITION_ID, PREPOSITION_SIZE, 0, 0]
GRASP_PREAMBLE = [170, 170, 170, GRASP_ID, GRASP_SIZE, 0]
RELEASE_PREAMBLE = [170, 170, 170, RELEASE_ID, RELEASE_SIZE, 0]

# disconnect message
DISCONNECT_MSG = [170, 170, 170, 7, 0, 0, 53, 76]
REMOVE_ERROR_MSG = [170, 170, 170, 36, 3, 0, 97, 99, 107, 220, 185]

class wsg50():
    def __init__(self, server_ip='192.168.1.21', server_port=1000):

        self.server_ip = server_ip # set the local class ip 
        self.server_port = server_port # set the local class port

        self.sckt = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # construct the class for the socket connection
        self.sckt.connect((self.server_ip, self.server_port)) # connect with TCP/IP to the gripper

        errorRemove = bytearray()
        for m in REMOVE_ERROR_MSG: # compile the byte array for the error remove message(this message is static on only needs to be created once)
            errorRemove.append(m)
        self.errorRemove = errorRemove

        disconnect = bytearray()
        for m in DISCONNECT_MSG: # compile the byte array for the disconnect message(this message is static on only needs to be created once)
            disconnect.append(m)
        self.disconnect = disconnect

        self.homing() # Home the gripper once connected to it
        time.sleep(0.5)

    ################ PRIVATE METHODS #####################
    def _remove_err(self):
        """Remove checksum errors from the gripper status. 
        """
        self.sckt.sendall(self.errorRemove)
        self.sckt.setblocking(0)
        
    def _float_to_payload(self, payload):
        """Decomposes a float to IEEE 754 32-bit float representation(4 bytes).

        Args:
            payload (float): Input float that needs to be converted.

        Returns:
            list: List consisting of 4 bytes that describes the float in IEEE 754.
        """
        
        hex_list = hex(struct.unpack('>I', struct.pack('>f', payload))[0])

        payload_list = []
        for i in range(0, len(hex_list[2:]), 2):
            payload_list.append(int(hex_list[i + 2 : i + 4], 16))

        return payload_list[::-1] #return as little endian

    ################ PUBLIC METHODS #####################
    def homing(self):
        """Homing gripper to 110mm and recalibrates finger pose.
        """

        homing_payload = [0]
        combined_payload = HOMING_PREAMBLE + homing_payload # create the specific payload for the homing procedure # checksum , 143, 131
        
        byte_msg = bytearray()
        for byte in combined_payload:
            byte_msg.append(byte)

        self.sckt.sendall(byte_msg)
        self._remove_err()
        self.sckt.setblocking(1)

        err_code = None
        while err_code != 0:
            data = self.sckt.recv(256) 
            err_code = struct.unpack('<h', data[6:8])[0]# byte 6-8 are error codes: 0: SUCCESS, 26: PENDING, 4: RUNNING, 10: DENIED
            print("Homing response", err_code)

    def preposition_gripper(self, width, speed):
        """Preposition the gripper to a set width at a certain speed. \n
        (DO NOT USE THIS FUNCTION TO GRASP PARTS. IT WILL RETURN AN ERROR FROM THE GRIPPER)

        Args:
            width (float): Set the width of the gripper fingers in mm.
            speed (float): Set the speed of the gripper fingers in mm/s.
        """
 
        width_payload = self._float_to_payload(width) # decompose float to 4 bytes that can be attached to message
        speed_payload = self._float_to_payload(speed)
        combined_payload = PREPOSITION_PREAMBLE + width_payload + speed_payload
        
        byte_msg = bytearray()
        for byte in combined_payload:
            byte_msg.append(byte)

        self.sckt.sendall(byte_msg)
        self._remove_err()
        self.sckt.setblocking(1)

        err_code = None
        while err_code != 0:
            data = self.sckt.recv(256) # byte 5-6 are error codes: 10 denied and 1a success
            err_code = struct.unpack('<h', data[6:8])[0]
            print("Preposition reponse from {0}:".format([width, speed]), err_code)
            

    def grasp_part(self, width, speed):
        """This function is used to grasp a part. \n
        (USING THIS FUNCTION WITHOUT HAVING A PART TO GRASP WILL RETURN AN ERROR)

        Args:
            width (float): Set the width of the part that has to be grasped in mm.
            speed (float): Set the speed of the gripper fingers in mm/s.
        """

        width_payload = self._float_to_payload(width)
        speed_payload = self._float_to_payload(speed)
        combined_payload = GRASP_PREAMBLE + width_payload + speed_payload

        byte_msg = bytearray()
        for byte in combined_payload:
            byte_msg.append(byte)

        self.sckt.sendall(byte_msg)
        self._remove_err()
        self.sckt.setblocking(1)

        err_code = None
        while err_code != 0:
            data = self.sckt.recv(256) # byte 5-6 are error codes: 10 denied and 1a success
            err_code = struct.unpack('<h', data[6:8])[0]
            print("Grasp reponse from {0}:".format([width, speed]), err_code)

    
    def release_part(self, width, speed):
        """Release a previously grasped part.

        Args:
            width (float): Set the width of the gripper fingers in mm.
            speed (float): Set the speed of the gripper fingers in mm/s.
        """

        width_payload = self._float_to_payload(width)
        speed_payload = self._float_to_payload(speed)
        combined_payload = RELEASE_PREAMBLE + width_payload + speed_payload

        byte_msg = bytearray()
        for byte in combined_payload:
            byte_msg.append(byte)

        self.sckt.sendall(byte_msg)
        self._remove_err()
        self.sckt.setblocking(1)

        err_code = None
        while err_code != 0:
            data = self.sckt.recv(256) # byte 5-6 are error codes: 10 denied and 1a success
            err_code = struct.unpack('<h', data[6:8])[0]
            print("Release reponse from {0}:".format([width, speed]), err_code)

    def start_connection(self, server_ip='192.168.1.21', server_port=1000):
        """Start a TCP/IP socket connection with a desired IP and port.

        Args:
            server_ip (str, optional): IP address of the desired server. Defaults to '192.168.1.21'.
            server_port (int, optional): Port of the desired server. Defaults to 1000.
        """

        self.sckt = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # construct the class for the socket connection
        self.sckt.connect((server_ip, server_port)) # connect with TCP/IP to the 
        
    def end_connection(self):
        """End TCP/IP socket connection.
        """

        self.sckt.sendall(self.disconnect)
        #self._remove_err()
        self.sckt.setblocking(1)

        err_code = None
        while err_code != 0:
            data = self.sckt.recv(256) # byte 5-6 are error codes: 10 denied and 1a success
            err_code = struct.unpack('<h', data[6:8])[0]
            print("Close connection reponse:", err_code)
            if err_code == 0: 
                print("SUCCESS")

        self.sckt.close()

def test(wsg_instance):

        wsg_instance.preposition_gripper(70, 100)

        wsg_instance.grasp_part(55, 100)

        time.sleep(5)

        wsg_instance.release_part(70, 100)

        wsg_instance.end_connection()


def main(): 
    wsg_instance = wsg50()

    try:
        test(wsg_instance)

    except KeyboardInterrupt:
        print('interrupted!')
        wsg_instance.end_connection()
        sys.exit(0)    

if __name__ == "__main__":
    main()


