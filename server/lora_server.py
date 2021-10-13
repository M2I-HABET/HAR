"""
Example for using the RFM9x Radio with Raspberry Pi.

Learn Guide: https://learn.adafruit.com/lora-and-lorawan-for-raspberry-pi
Author: Brent Rubell for Adafruit Industries
"""
# Import Python System Libraries
import time
# Import Blinka Libraries
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
# Import the SSD1306 module.
import adafruit_ssd1306
# Import RFM9x
import adafruit_rfm9x
import socket
import logging
from datetime import datetime

# Initialize logging
logging.basicConfig(filename='lora_server.log', level=logging.DEBUG, format='%(asctime)s %(message)s')
logging.info("Initialized logger")


# Create the I2C interface.
i2c = busio.I2C(board.SCL, board.SDA)

# 128x32 OLED Display
reset_pin = DigitalInOut(board.D4)
display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=reset_pin)
# Clear the display.
display.fill(0)
display.show()
width = display.width
height = display.height

# Configure LoRa Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 434.0)
rfm9x.tx_power = 23
prev_packet = None
print("LoRa Started")
logging.info("LoRa Started")

# Configure UDP port for information streaming
ip = "192.168.1.205"
port = 4440

connection = False
# Create socket for server
while not connection: 
    try: 
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
        # print("Do Ctrl+c to exit the program !!")
        s.connect((ip,port))
        print("Connection to server completed")
        logging.info("Connection to server completed")
        connection = True
    except Exception as e :
        logging.error("Could not initialize the server")
        logging.error(e)
        print("Could not initialize the server")
        print(e)
        s.close()
        time.sleep(1)
        # exit()



# Open file to save GPS log data for testing and data saving purposes
f = open('datalog.txt', 'a')

last_pkt_time = None
while True:
    packet = None
    # draw a box to clear the image
    display.fill(0)
    display.text('RasPi LoRa', 35, 0, 1)

    # check for packet rx
    packet = rfm9x.receive()
    if packet is None:
        display.show()
        display.text('- Waiting for PKT -', 15, 20, 1)
        print('Last packet received: ', last_pkt_time)
    else:
        # Display the packet text and rssi
        try :
            # Try to get the packet via LoRa
            display.fill(0)
            prev_packet = packet
            packet_text = prev_packet.decode('utf-8') # if the packet is not in unicode then it will fail and print the error message

            # Display the data to the screen and cmd line
            display.text('RX: ', 0, 0, 1)
            display.text(packet_text, 25, 0, 1)
            print(packet_text)
            # try to send the data to the penthouse machine
            try :
               # send_packet = packet_text + "," + str(bytes(rfm9x.rssi+256))
                s.send(prev_packet) # we do not need to encode this into utf since it should already be encoded that way 
                rec_data = s.recv(4096)
                print("Data recieved: ", rec_data.decode('utf-8'))
                last_pkt_time = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
                print("Time: ", last_pkt_time)
                print(rec_data, "\t", last_pkt_time, file=f)
            except Exception as e:
                logging.error("\n\nUnable to send/recieve data.  Error: ")
                logging.error(e)
                print("\n\nUnable to send/recieve data.  Error: ")
                print(e)
            # Send back the rssi data to the payload
            #rssi = bytes(rfm9x.rssi+256)
            #rssi_packet = rssi
            #rfm9x.send(rssi_packet)
        except UnicodeDecodeError:
            print("\n\nPacket error\n\n")
        finally :
            time.sleep(1)


    display.show()
    time.sleep(0.1)

s.close()
f.close()
