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

# Button A
btnA = DigitalInOut(board.D5)
btnA.direction = Direction.INPUT
btnA.pull = Pull.UP

# Button B
btnB = DigitalInOut(board.D6)
btnB.direction = Direction.INPUT
btnB.pull = Pull.UP

# Button C
btnC = DigitalInOut(board.D12)
btnC.direction = Direction.INPUT
btnC.pull = Pull.UP

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

# Configure UDP port for information streaming
ip = "192.168.1.31"
port = 4444
# Create socket for server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
# print("Do Ctrl+c to exit the program !!")
s.connect((ip,port))

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
                s.send(prev_packet) # we do not need to encode this into utf since it should already be encoded that way 
                rec_data = s.recv(4096)
                print("Data recieved: ", rec_data.decode('utf-8'))
            except Exception as e:
                print("\n\nUnable to send/recieve data.  Error: ")
                print(e)
            # Send back the rssi data to the payload
            rssi = bytes(rfm9x.rssi+256)
            rssi_packet = rssi
            rfm9x.send(rssi_packet)
            
        except UnicodeDecodeError:
            print("\n\nPacket error\n\n")
        finally :
            time.sleep(1)

    # if not btnA.value:
    #     # Send Button A
    #     display.fill(0)
    #     button_a_data = bytes("Button A!\r\n","utf-8")
    #     rfm9x.send(button_a_data)
    #     display.text('Sent Button A!', 25, 15, 1)
    # elif not btnB.value:
    #     # Send Button B
    #     display.fill(0)
    #     button_b_data = bytes("Button B!\r\n","utf-8")
    #     rfm9x.send(button_b_data)
    #     display.text('Sent Button B!', 25, 15, 1)
    # elif not btnC.value:
    #     # Send Button C
    #     display.fill(0)
    #     button_c_data = bytes("Button C!\r\n","utf-8")
    #     rfm9x.send(button_c_data)
    #     display.text('Sent Button C!', 25, 15, 1)


    display.show()
    time.sleep(0.1)

s.close()
