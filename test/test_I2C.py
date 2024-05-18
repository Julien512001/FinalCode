from smbus import SMBus
from time import sleep

# address of teensy
addr = 0x09
# Create bus
bus = SMBus(1)

# wait for configuration
sleep(1)

# Data to send (turn on the led)
data = "aaaa"
# Convert to a list of ascii char
data = [ord(c) for c in data]
# Write all those chars
print("ok")
i = 0
while(True):
    bus.write_i2c_block_data(addr, 0, data)
    # Read a bloc of 20 data
    val = bus.read_i2c_block_data(addr,0,20)
    # Print the value
    #print(val)
    print(i)
    i += 1

# Wait for the led on the teensy
sleep(1)
# Data to send (turn off the led)
data = "ooooooo"
# Convert it to a list of ascii char
data = [ord(c) for c in data]
# Write all those chars
bus.write_i2c_block_data(addr, 0, data)

