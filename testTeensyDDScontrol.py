import numpy as np
import serial
import sys
import time


portArduino = "COM9"
baud = 57600

def do_handshake(serial_port):
    serial_port.write(b'h')
    incoming = serial_port.read().decode()
    if incoming == 'y':
        return True
    elif incoming == 'n':
        print("Got an n from Arduino")
        return False
    else:
        print("do_handshake communication failed")
        return False

def set_frequency(frequency, serial_port):
    handshake = do_handshake(serial_port)
    if not handshake:
        print("Handshake failed in set_frequency")
        return False
    command = 'f' + "{:d}".format(frequency)
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'c':
        print("Freq set")
        return True
    elif incoming == 'n':
        print("Got an n from Arduino")
        return False
    else:
        print("set_frequency communication failed")
        return False


def set_power(power, serial_port):
    handshake = do_handshake(serial_port)
    if not handshake:
        print("Handshake failed in set_power")
        return False
    command = 'p' + "{:.05f}".format(power)
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'c':
        print("power set")
        return True
    elif incoming == 'n':
        print("Got an n from Arduino")
        return False
    else:
        print("set_power communication failed")
        return False

def send_sequence(serial_port):
    handshake = do_handshake(serial_port)
    if not handshake:
        print("Handshake failed in send_sequence")
        return False
    command = "s3"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'c':
        print("Sequence 0 command sN correct")
    elif incoming == 'n':
        print("Received n from sequence 0")
        sys.exit(0)
    else:
        print("Error in send_sequence 0")
        sys.exit(0)

    # First one
    command = "fpvv"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        print("Sequence 1 command fpv correct")
    elif incoming == 'n':
        print("Received n from sequence 1")
        sys.exit(0)
    else:
        print("Error in send_sequence 1")
        sys.exit(0)
    command = "79500000\n"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        print("Sequence 2 command frequency is correct")
    elif incoming == 'n':
        pass
    elif incoming == 'n':
        print("Received n from sequence 2")
        sys.exit(0)
    else:
        print("Error in send_sequence 2")
        sys.exit(0)
    command = "90.3\n"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        print("Sequence 3 command power is correct")
    elif incoming == 'n':
        print("Received n from sequence 3")
        sys.exit(0)
    else:
        print("Error in send_sequence 3")
        sys.exit(0)

    # Second one
    command = "fpvv"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        print("Sequence 1 command fpv correct")
    elif incoming == 'n':
        print("Received n from sequence 1")
        sys.exit(0)
    else:
        print("Error in send_sequence 1")
        sys.exit(0)
    command = "80500000\n"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        print("Sequence 2 command frequency is correct")
    elif incoming == 'n':
        pass
    elif incoming == 'n':
        print("Received n from sequence 2")
        sys.exit(0)
    else:
        print("Error in send_sequence 2")
        sys.exit(0)

    command = "50.3\n"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        print("Sequence 3 command power is correct")
    elif incoming == 'n':
        print("Received n from sequence 3")
        sys.exit(0)
    else:
        print("Error in send_sequence 3")
        sys.exit(0)

    # Third one
    command = "vvwv"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        print("Sequence 3 command vvvw correct")
    elif incoming == 'n':
        print("Received n from sequence 3")
        sys.exit(0)
    else:
        print("Error in send_sequence 3")
        sys.exit(0)
    command = "0\n"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        pass
        print("Ramp type sent")
    elif incoming == 'n':
        print("Got n in ramp data transmission")
        sys.exit(0)
    else:
        print("Ramp data transmission response is undefined")
        sys.exit(0)
    command = "80000000\n"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        pass
        print("Ramp start sent")
    elif incoming == 'n':
        print("Got n in ramp data transmission")
        sys.exit(0)
    else:
        print("Ramp data transmission response is undefined")
        sys.exit(0)
    command = "60000000\n"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        pass
        print("Ramp end sent")
    elif incoming == 'n':
        print("Got n in ramp data transmission")
        sys.exit(0)
    else:
        print("Ramp data transmission response is undefined")
        sys.exit(0)
    command = "2000\n"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        pass
        print("Ramp timing sent")
    elif incoming == 'n':
        print("Got n in ramp time data transmission")
        sys.exit(0)
    else:
        print("Ramp time data transmission response is undefined")
        sys.exit(0)
    command = "200\n"
    serial_port.write(command.encode())
    incoming = serial_port.read().decode()
    if incoming == 'r':
        pass
        print("Ramp exponential sent")
    elif incoming == 'n':
        print("Got n in ramp exponential const data transmission")
        sys.exit(0)
    else:
        print("Ramp data exponential const response is undefined")
        sys.exit(0)


sPort = serial.Serial(portArduino,baud, timeout=7)
if sPort.isOpen():
    print(sPort.name+" is open")
else:
    print("Cannot open "+portArduino)

set_frequency(80000000,sPort)
set_power(100,sPort)
sys.exit(0)
send_sequence(sPort)
sPort.close()
sys.exit(0)

idx = 0
while True:
    idx += 1
    freq_set = set_frequency(int(80e6),sPort)
    if freq_set:
        print("Frequency is set")
        pow_set = set_power(100*(idx%2),sPort)
    else:
        print("Frequency set failed")
        #sPort.close()
        sys.exit(0)
    if pow_set:
        print("Power is set")
    else:
        print("Power set failed")
        #sPort.close()
        sys.exit(0)
    #sPort.close()


sys.exit(0)

for x in range(0,20):
    print("Loop number {:d}".format(x))

    freq_set = set_frequency(79000000+1000000*(x%2),sPort)
    if freq_set:
        print("Frequency is set")
        pow_set = set_power(100-50*(x%2),sPort)
    else:
        print("Frequency set failed")
        sys.exit(0)
    if pow_set:
        print("Power is set")
    else:
        print("Power set failed")
        sys.exit(0)

    time.sleep(1)
