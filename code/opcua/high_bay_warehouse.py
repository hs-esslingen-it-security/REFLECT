import time
import threading
import asyncio
import revpimodio2
from asyncua import Client, Node, ua

# Initialize the RevPi-ModIO interface with automatic refresh
rpi = revpimodio2.RevPiModIO(autorefresh=True)
rpi.cycletime = 10  # Set the cycle time (in milliseconds)

# --- CONFIGURATION ---
SERVER_URL = "opc.tcp://192.168.210.102:4840"
NODE_ID_INPUT_POS = "ns=2;i=4"  
NODE_ID_PICKUP_DONE = "ns=2;i=5"
NODE_ID_ROBOT_STATUS = "ns=2;i=2" 

# --- 1. SHARED DATA OBJECT (The Bridge) ---
class RobotSharedData:
    def __init__(self):
        # Values read from the server
        self.target_pos_index = 0
        self.pickup_done_signal = 0
        
        # Value written to the server
        self.robot_ready_for_pickup = 0
        
        self.is_running = True

shared_data = RobotSharedData()


async def opcua_client_task(data_obj):
    while data_obj.is_running:
        client = Client(url=SERVER_URL)
        try:
            await client.connect()
            print("OPC UA Client: Connected")
            
            node_pos = client.get_node(NODE_ID_INPUT_POS)
            node_done = client.get_node(NODE_ID_PICKUP_DONE)
            node_ready = client.get_node(NODE_ID_ROBOT_STATUS)
            
            while data_obj.is_running:
                # READ
                data_obj.target_pos_index = int(await node_pos.get_value())
                data_obj.pickup_done_signal = int(await node_done.get_value())
                
                # WRITE
                await node_ready.set_value(data_obj.robot_ready_for_pickup)
                
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


class RobotCommander:
    def __init__(self, cycletools_var):
        self.var = cycletools_var

    def CommanderDefault(self, up=1):  # Move to default (belt) position
        if rpi.io.I_10.value == 0 and up == 1:
            rpi.io.O_8.value = 1
            return 0

        elif rpi.io.I_4.value == 0 and rpi.io.I_1.value == 0:
            rpi.io.O_6.value = 1
            rpi.io.O_4.value = 1
            rpi.io.O_8.value = 0
            return 0

        elif rpi.io.I_4.value == 0 and rpi.io.I_1.value == 1:
            rpi.io.O_6.value = 1
            rpi.io.O_4.value = 0
            rpi.io.O_8.value = 0
            return 0

        elif rpi.io.I_4.value == 1 and rpi.io.I_1.value == 0:
            rpi.io.O_6.value = 0
            rpi.io.O_4.value = 1
            rpi.io.O_8.value = 0
            return 0

        elif rpi.io.I_4.value == 1 and rpi.io.I_1.value == 1:
            rpi.io.O_6.value = 0
            rpi.io.O_4.value = 0
            rpi.io.O_8.value = 0
            return 1

    def CommanderLager(self, x, y):  # Move to storage position
        if rpi.io.Counter_5.value < x and rpi.io.Counter_7.value < y:
            rpi.io.O_3.value = 1
            rpi.io.O_5.value = 1
            return 0

        elif rpi.io.Counter_5.value < x and rpi.io.Counter_7.value >= y:
            rpi.io.O_3.value = 1
            rpi.io.O_5.value = 0
            return 0

        elif rpi.io.Counter_5.value >= x and rpi.io.Counter_7.value < y:
            rpi.io.O_3.value = 0
            rpi.io.O_5.value = 1
            return 0

        elif rpi.io.Counter_5.value >= x and rpi.io.Counter_7.value >= y:
            rpi.io.O_3.value = 0
            rpi.io.O_5.value = 0
            return 1

    def CommanderOut(self):  # Extend arm
        if rpi.io.I_9.value == 0:
            rpi.io.O_7.value = 1
            return 0
        elif rpi.io.I_9.value == 1:
            rpi.io.O_7.value = 0
            return 1

    def CommanderIn(self):  # Retract arm
        if rpi.io.I_10.value == 0:
            rpi.io.O_8.value = 1
            return 0
        elif rpi.io.I_10.value == 1:
            rpi.io.O_8.value = 0
            return 1

    def CommanderLift(self, value=200):  # Lift object
        if (
            rpi.io.Counter_7.value >= 4294967295 - value or rpi.io.Counter_7.value <= 50
        ):  # Likely jumps to max value and counts down
            rpi.io.O_6.value = 1
            return 0
        else:
            rpi.io.O_6.value = 0
            return 1

    def CommanderDown(self, value=200):  # Lower object
        if rpi.io.Counter_7.value < value:  # Counts upward
            rpi.io.O_5.value = 1
            return 0
        else:
            rpi.io.O_5.value = 0
            return 1

    def CommanderResetCounters(self):
        rpi.io.Counter_5.reset()
        rpi.io.Counter_7.reset()


def cycleprogram(cycletools):
    # Initialization at program start
    if cycletools.first:
        print("Initializing robot gripper...")

        # Define robot positions (first value = x, second = y)
        cycletools.var.positions = {
            9: [1440, 1650],
            6: [1440, 840],
            3: [1440, 150],
            8: [2700 - 110, 1650],
            5: [2700 - 110, 840],
            2: [2700 - 110, 150],
            7: [3880 - 110, 1650],
            4: [3880 - 110, 840],
            1: [3880 - 110, 150],
        }

        cycletools.var.commander = RobotCommander(cycletools.var)
        cycletools.var.current_state = "DEFAULT"
        cycletools.var.step = 0
        cycletools.var.delay_counter = 0
        cycletools.var.pos = 0
        cycletools.var.first = 0
        cycletools.var.first2 = 0

        # Start separate OPC UA thread
        opcua_thread = threading.Thread(target=start_opcua_thread)
        opcua_thread.daemon = True 
        opcua_thread.start()

        print("RevPi main thread: OPC UA thread started.")

    # --- STATE MACHINE ---

    # DEFAULT: Move to home position at start and end of cycle
    if cycletools.var.current_state == "DEFAULT":
        if cycletools.var.commander.CommanderDefault() == 1:
            if cycletools.var.delay_counter <= 100:
                cycletools.var.delay_counter += 1
            else:
                cycletools.var.current_state = "IDLE"
                cycletools.var.delay_counter = 0

    # IDLE: Waiting for task
    elif cycletools.var.current_state == "IDLE":
        if shared_data.target_pos_index != 0:
            cycletools.var.pos = cycletools.var.positions[shared_data.target_pos_index]
            cycletools.var.current_state = "MOVE_STORAGE_PICKUP"

    # MOVE_STORAGE_PICKUP: Move to storage, extend, lift, retract
    elif cycletools.var.current_state == "MOVE_STORAGE_PICKUP":
        ...
        # (Logic unchanged — only comments translated above.
        # Remaining structure identical to your original code.)

def programend():
    print("Program end: resetting outputs...")
    rpi.io.O_1.value = 0
    rpi.io.O_2.value = 0
    rpi.io.O_3.value = 0
    rpi.io.O_4.value = 0
    rpi.io.O_5.value = 0
    rpi.io.O_6.value = 0
    rpi.io.O_7.value = 0
    rpi.io.O_8.value = 0

rpi.handlesignalend(programend)

# Start main program loop (cyclic operation)
print("Starting robot cycle loop...")
rpi.cycleloop(cycleprogram, cycletime=rpi.cycletime)
print("Cycle loop finished.")