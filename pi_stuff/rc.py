"""
Created with help from:
User raja_961, “Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV”. Instructables. URL:
https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/
"""
import board
import busio
import adafruit_mcp4728
import time

# Initialize devices
i2c = busio.I2C(board.SCL, board.SDA)
mcp4728 = adafruit_mcp4728.MCP4728(i2c, 0x64)

# Drive controlled by channel b
mcp4728.channel_c.value = 37000 # forward
time.sleep(.1)
mcp4728.channel_c.value = 0 # stop
time.sleep(.1)
mcp4728.channel_c.value = 65000 // 2 # stop
time.sleep(.1)

# Steering controlled by channel d
mcp4728.channel_a.value = 65000 # right
time.sleep(.1)
mcp4728.channel_a.value = 0 # left
time.sleep(.1)
mcp4728.channel_a.value = int(65000 / 2) # straight
time.sleep(.1)
