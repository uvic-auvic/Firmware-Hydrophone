import matplotlib.pyplot as plt
from codecs import encode
from struct import unpack
from serial import Serial
from binascii import crc32
from stm32_library import stm32_crc32

print "What port is the hydrophone on?"
port_string = raw_input()

hydro_serial = Serial(port_string, 115200, timeout=1)
hydro_serial.write("RID\n")
device_id = hydro_serial.readline()
if device_id != "Hydrophones v1.0\r\n":
    print device_id
    print "Device is not Hydrophones."
    hydro_serial.close()
    quit()
print "Connected to hydrophone."

print "Starting ADC conversions"
data = []
total_size = 0
packet_idx = 0
packet_count = 0xFFFF

hydro_serial.write("ADCDR\n")
while packet_idx < (packet_count - 1):
    header = hydro_serial.read(12)

    crc = int(encode(header[3] + header[2] + header[1] + header[0], 'hex'), 16)
    command = header[4:6]
    packet_count = int(encode(header[6], 'hex'), 16)
    packet_idx = int(encode(header[7], 'hex'), 16)
    packet_size = int(encode(header[9] + header[8], 'hex'), 16)
    total_size = int(encode(header[11] + header[10], 'hex'), 16)
    print "\nPacket %s:" % packet_idx
    print "CRC: %s" % crc
    print "Command: %s" % command
    print "Packet Count: %s" % packet_count
    print "Packet Index: %s" % packet_idx
    print "Packet Size: %s" % packet_size
    print "Total Size: %s" % total_size

    packet_data = hydro_serial.read(packet_size)
    host_crc = stm32_crc32(packet_data, 0xFFFFFFFF)

    if crc == host_crc:
        hydro_serial.write("NEXT\n")
        data += packet_data
    else:
        print "Bad CRC: Host:%s, Micro:%s" % (host_crc, crc)
        hydro_serial.write("RETRY\n")

hydro_serial.reset_input_buffer()

print "Read data from hydrophone"
print "Closing serial port"
hydro_serial.close()

if len(data) != total_size:
    print "Sizes didn't match up!"
else:
    print "Total data size in packets matched the sizes read"

hydrophone_data = [[], [], [], []]
for i in range(len(data)/2):
    hydrophone_data[i % 4].append(int(encode(data[2*i + 1] + data[(2*i)], 'hex'), 16))

for hydro_data in hydrophone_data:
    print len(hydro_data)

x = [[i for i in range(len(hydrophone_data[0]))], [i for i in range(len(hydrophone_data[1]))], \
        [i for i in range(len(hydrophone_data[2]))], [ i for i in range(len(hydrophone_data[3]))]]

plt.subplot(2, 2, 1)
plt.plot(x[0], hydrophone_data[0])
plt.subplot(2, 2, 2)
plt.plot(x[1], hydrophone_data[1])
plt.subplot(2, 2, 3)
plt.plot(x[2], hydrophone_data[2])
plt.subplot(2, 2, 4)
plt.plot(x[3], hydrophone_data[3])
plt.show()
   
