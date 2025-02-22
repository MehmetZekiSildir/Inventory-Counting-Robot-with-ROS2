#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import serial
import rclpy
from rclpy.node import Node
from rfid_interface.msg import RFIDMsg, RFIDArray, RFIDString

#Explanation
# In this pacakge there are two seperate .msg files. One of them is rfid_msg and other one is the array version of this message
# The array data type used for multiple rfid readings
# Both rfid_msg and rfid_array data types have their own header messages but they must be the same in this case
# rfid_array data type has a size informatin about the number of rfids

class RFID_reader(Node):
    
    def __init__(self,node):
        super().__init__('RFID_reader')
        self.node = node
        self.RFID_publisher = self.create_publisher(RFIDString, 'rfid_message', 10)
        self.reader_ready = False
        self.macrorun_ready = False
        self.ser = serial.Serial(
            port='/dev/ttyUSB1',  
            baudrate=115200, 
            bytesize=serial.EIGHTBITS,  
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            xonxoff=False, 
            rtscts=False ,
            timeout=4)
        self.get_logger().info('Serial port is opened')


        self.reader_publisher()

    def reader_publisher(self):
        RFID_msg   = RFIDString()
        

        try:
            if self.ser.is_open:
                data = self.ser.readline().decode('utf-8')
                
                if len(data) == 0:
                    self.ser.write("ReaderName?\r\n".encode('utf-8'))
                    data = self.ser.readline()
                    self.get_logger().info(str(data.decode('utf-8')))
                    data = self.ser.readline()
                    
                    if "ReaderName = Alien RFID Reader\r\n" == data.decode('utf-8'):
                        self.get_logger().info("Correct reader name")
                        self.reader_ready = True

                while not self.reader_ready:
                    data = self.ser.readline().decode('utf-8')
                    self.get_logger().info(data)
                    if data == "Boot> Ready\r\n":   
                        self.get_logger().info("READER IS READY FOR INVENTORY PROCESS")
                        self.reader_ready = True

                command = "macrorun automode\r\n"
                self.ser.write(command.encode())
                self.get_logger().info("Macro file is loading")

                while not self.macrorun_ready:
                    data = self.ser.readline().decode('utf-8')
                    if data == "MacroRun = Success!\r\n":
                        self.macrorun_ready = True
                        data = self.ser.readline() #to avoid b\x00\r\n' output
                        self.get_logger().info("RFID Publisher is readey")

                command = "AntennaSequence = 0 1\r\n"
                self.ser.write(command.encode())
                self.get_logger().info("Antenna command sent")
            
                while True:
                    data = self.ser.readline().decode('utf-8')
                    if data == "AntennaSequence = 0 1\r\n":
                        data = self.ser.readline() #to avoid b\x00\r\n' output
                        self.get_logger().info("Antennas configured as 0 1")
                        break
                        
                while True:
                    tag =self.ser.readline().decode()
                    RFID_msg.data = tag
                    RFID_msg.header.stamp = self.get_clock().now().to_msg()
                    rfid_infos = tag.split(",")
                    if (len(rfid_infos)==10):
                        self.get_logger().info(tag)
                        self.RFID_publisher.publish(RFID_msg)
                    else:
                        self.get_logger().info('There is no any tag')
            else:
                self.get_logger().info('Serial port is not aviable')
                

        except Exception as e:
            self.get_logger().info(str(e))

        finally:
            fin = "automode = off\r\n"
            self.ser.write(fin.encode())
            self.ser.close()
            self.get_logger().info('Serial port is close.')
 
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rfid_reader_node')

    reader = RFID_reader(node)

    rclpy.spin(reader)


    reader.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
