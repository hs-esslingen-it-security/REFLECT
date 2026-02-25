import revpimodio2
import statistics
import threading
import time
import asyncio
from asyncua import Client, Node, ua

SERVER_URL = "opc.tcp://192.168.210.102:4840"
NODE_ID_WHITE = "ns=2;i=6"
NODE_ID_RED = "ns=2;i=7"
NODE_ID_BLUE = "ns=2;i=8" 

class RobotSharedData:
    def __init__(self):
        
        # Values to be written to the server
        self.white = 1
        self.red = 1
        self.blue = 1
        
        self.is_running = True

shared_data = RobotSharedData()

# RevPi object with automatic IO refresh
rpi = revpimodio2.RevPiModIO(autorefresh=True)


async def opcua_client_task(data_obj):
    while data_obj.is_running:
        client = Client(url=SERVER_URL)
        try:
            await client.connect()
            print("OPC UA Client: Connected")
            
            node_white = client.get_node(NODE_ID_WHITE)
            node_red = client.get_node(NODE_ID_RED)
            node_blue = client.get_node(NODE_ID_BLUE)

            while data_obj.is_running:
                
                # WRITE
                await node_white.set_value(data_obj.white)
                await node_red.set_value(data_obj.red)
                await node_blue.set_value(data_obj.blue)
                
                await asyncio.sleep(0.05)
                
        except Exception as e:
            print(f"Connection lost: {e}. Reconnecting in 5s...")
            try:
                await client.disconnect()
            except:
                pass
            await asyncio.sleep(5)


def start_opcua_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(opcua_client_task(shared_data))

# Initialize the RevPi ModIO interface with automatic refresh
rpi = revpimodio2.RevPiModIO(autorefresh=True)

def cycleprogram(cycletools):
    # One-time initialization at program start
    if cycletools.first:
        cycletools.var.state = "idle"                 # Initial state: piston retracted
        cycletools.var.retractCounter = 0             # Countdown for later retraction
        cycletools.var.cycleCount = 0                 # General step counter
        cycletools.var.cycleAuswurf = 70              # Time delay before ejection
        cycletools.var.pistonNumber = 0               # Active piston (1, 2 or 3)
        cycletools.var.color = 0
        cycletools.var.sensors = [0] * 6
        cycletools.var.sensors[1] = 1
        rpi.io.O_1.value = 0                          # Conveyor belt initially stopped
        cycletools.var.fail = 0
        cycletools.var.firstDelay = 0

        # Variables for median calculation
        cycletools.var.color_readings = []
        cycletools.var.num_scan_samples = 10 

        # Start separate OPC UA thread
        opcua_thread = threading.Thread(target=start_opcua_thread)
        opcua_thread.daemon = True 
        opcua_thread.start()

    # === READ SENSORS DIRECTLY ===
    if cycletools.var.firstDelay < 1000:
        cycletools.var.firstDelay += 1
    else:
        # Direct hardware input access instead of virtual Modbus words
        cycletools.var.color = rpi.io.AnalogInput_1.value
        
        cycletools.var.sensors[0] = rpi.io.I_1.value
        cycletools.var.sensors[1] = rpi.io.I_2.value
        cycletools.var.sensors[2] = rpi.io.I_3.value
        cycletools.var.sensors[3] = rpi.io.I_4.value
        cycletools.var.sensors[4] = rpi.io.I_5.value
        cycletools.var.sensors[5] = rpi.io.I_6.value

        shared_data.white = rpi.io.I_4.value
        shared_data.red = rpi.io.I_5.value
        shared_data.blue = rpi.io.I_6.value

    # === STATE MACHINE ===

    # Conveyor running → prepare for scanning process
    if cycletools.var.state == "driveScan":
        if cycletools.var.color <= 7050:
            cycletools.var.cycleCount += 1
            if cycletools.var.cycleCount >= 10:           # Stop after 10 cycles
                rpi.io.O_1.value = 0                      # Stop conveyor belt
                cycletools.var.cycleCount = 0
                cycletools.var.fail = 0
                cycletools.var.state = "scan"             # Next state: scan
        else:
            cycletools.var.fail += 1
            if cycletools.var.fail == 500:
                cycletools.var.fail = 0
                rpi.io.O_1.value = 0
                cycletools.var.state = "idle"
        
    # Scan object → decide which piston should activate
    elif cycletools.var.state == "scan":
        cycletools.var.color_readings.append(cycletools.var.color)
        
        if len(cycletools.var.color_readings) >= cycletools.var.num_scan_samples:
            
            # Median calculation
            measured_color = statistics.median(cycletools.var.color_readings)

            if 2500 <= measured_color <= 5000:
                cycletools.var.cycleAuswurf = 60 
                cycletools.var.pistonNumber = 1
                rpi.io.O_1.value = 1 
                rpi.io.O_2.value = 1 
                cycletools.var.state = "wait"

            elif 5700 <= measured_color <= 6300:
                cycletools.var.cycleAuswurf = 150
                cycletools.var.pistonNumber = 2
                rpi.io.O_1.value = 1
                rpi.io.O_2.value = 1
                cycletools.var.state = "wait"

            elif 6400 <= measured_color <= 7000:
                cycletools.var.cycleAuswurf = 250
                cycletools.var.pistonNumber = 3
                rpi.io.O_1.value = 1
                rpi.io.O_2.value = 1
                cycletools.var.state = "wait"

            else:
                rpi.io.O_1.value = 1 
                cycletools.var.state = "idle" 

            cycletools.var.color_readings = [] 

    elif cycletools.var.state == "wait":
        if cycletools.var.sensors[2] == 0:
            cycletools.var.state = "extend"

    # Wait until predefined cycle count is reached → extend piston
    elif cycletools.var.state == "extend":
        cycletools.var.cycleCount += 1
        if cycletools.var.cycleCount >= cycletools.var.cycleAuswurf:
            rpi.io.O_1.value = 0                      # Stop conveyor
            if cycletools.var.pistonNumber == 1:
                rpi.io.O_3.value = 1                  # Extend piston 1
            elif cycletools.var.pistonNumber == 2:
                rpi.io.O_4.value = 1                  # Extend piston 2
            elif cycletools.var.pistonNumber == 3:
                rpi.io.O_5.value = 1                  # Extend piston 3
            cycletools.var.retractCounter = 5         # Wait before retracting
            cycletools.var.state = "retrace"

    # Retract piston after defined time
    elif cycletools.var.state == "retrace":
        cycletools.var.retractCounter -= 1
        if cycletools.var.retractCounter <= 0:
            if cycletools.var.pistonNumber == 1:
                rpi.io.O_3.value = 0                  # Retract piston 1
            elif cycletools.var.pistonNumber == 2:
                rpi.io.O_4.value = 0                  # Retract piston 2
            elif cycletools.var.pistonNumber == 3:
                rpi.io.O_5.value = 0                  # Retract piston 3
            rpi.io.O_2.value = 0                      # Reset "operation" indicator
            cycletools.var.state = "idle"

    # Wait for user interaction (e.g. button press) to start new cycle
    elif cycletools.var.state == "idle":
        if cycletools.var.sensors[1] == 0:            # If button pressed
            rpi.io.O_1.value = 1                      # Start conveyor belt
            cycletools.var.cycleCount = 0             # Reset counter
            cycletools.var.state = "driveScan"        # New cycle begins


# Called when program ends - safely switch off all possible outputs
def programend():
    rpi.io.O_1.value = 0
    rpi.io.O_2.value = 0
    rpi.io.O_3.value = 0
    rpi.io.O_4.value = 0
    rpi.io.O_5.value = 0

rpi.handlesignalend(programend)

# Start main program loop (cyclic operation)
rpi.cycleloop(cycleprogram, cycletime=10)