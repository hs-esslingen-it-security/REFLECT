import revpimodio2
import queue
import time
import asyncio
import threading
from asyncua import Server, ua

# Shared object for OPC UA and cycle loop
class RobotSharedData:
    def __init__(self):
        self.storage_status = 0
        self.multi_status = 0
        self.position_at_storage = 0
        self.pallet_clear = 0
        self.white = 1
        self.red = 1
        self.blue = 1
        self.is_running = True


shared_data = RobotSharedData()

async def opcua_server_task(data_obj):
    server = Server()
    await server.init()
    
    # Defined IP of the gripper (OPC UA Server)
    server.set_endpoint("opc.tcp://0.0.0.0:4840")
    
    idx = await server.register_namespace("http://revpi")
    obj = await server.nodes.objects.add_object(idx, "Control")
    
    # Create nodes
    node_a = await obj.add_variable(idx, "StorageStatus", data_obj.storage_status)
    node_b = await obj.add_variable(idx, "MultiStatus", data_obj.multi_status)
    node_c = await obj.add_variable(idx, "PositionAtStorage", data_obj.position_at_storage)
    node_d = await obj.add_variable(idx, "PalletClear", data_obj.pallet_clear)
    node_e = await obj.add_variable(idx, "White", data_obj.white)
    node_f = await obj.add_variable(idx, "Red", data_obj.red)
    node_g = await obj.add_variable(idx, "Blue", data_obj.blue)
    
    # Allow write access
    await node_a.set_writable()
    await node_b.set_writable()
    await node_c.set_writable()
    await node_d.set_writable()
    await node_e.set_writable()
    await node_f.set_writable()
    await node_g.set_writable()

    print("Server running at opc.tcp://192.168.210.102:4840")
    
    async with server:
        while data_obj.is_running:
            # 1. Receive
            # Read from nodes and store in shared object
            data_obj.storage_status = await node_a.get_value()
            data_obj.multi_status = await node_b.get_value()

            data_obj.white = await node_e.get_value()
            data_obj.red = await node_f.get_value()
            data_obj.blue = await node_g.get_value()
            
            # 2. Send
            # Take values from shared object and write to nodes
            await node_c.write_value(data_obj.position_at_storage)
            await node_d.write_value(data_obj.pallet_clear)
            
            await asyncio.sleep(0.05)
        

def start_opcua_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(opcua_server_task(shared_data))        


opc_thread = threading.Thread(target=start_opcua_thread, daemon=True)
opc_thread.start()


# Global task queue
task_queue = queue.Queue()

# Helper structure to track tasks already inside the queue
tasks_in_queue_set = set()


# Function to safely add a task to the queue (duplicate check included)
def add_task_to_queue(task):
    if task not in tasks_in_queue_set:
        task_queue.put(task)
        tasks_in_queue_set.add(task)
        print(f"Task {task} added to the list.")
        return True
    else:
        print(f"Task {task} is already in the list. Not added.")
        return False


# Initialize RevPi ModIO interface with auto refresh
rpi = revpimodio2.RevPiModIO(autorefresh=True)
rpi.cycletime = 10  # Cycle time in milliseconds


def cycleprogram(cycletools):

    # Initialization when program starts
    if cycletools.first:
        print("Initializing robot gripper...")

        # Define robot positions
        cycletools.var.positions = {
            "white_pick_up": [910, 690, 1700],
            "red_pick_up": [760, 810, 1700],
            "blue_pick_up": [625, 1100, 1700],
            "storage": [2700, 350, 300],
            "multi_drop_off": [1805, 1680, 1000],
        }

        cycletools.var.current_state = "DEFAULT"
        cycletools.var.current_task = None
        cycletools.var.is_picking_up = True
        cycletools.var.cycle_step = 0
        cycletools.var.delay_counter = 0
        cycletools.var.first = 0
        cycletools.var.move_axis_state = "AXIS1"

        # Turn all motors and grippers off
        rpi.io.O_7.value = 0
        rpi.io.O_9.value = 0
        rpi.io.O_11.value = 0
        rpi.io.O_12.value = 0
        rpi.io.O_10.value = 0
        rpi.io.O_8.value = 0
        rpi.io.O_13.value = 0
        rpi.io.O_14.value = 0


    # ---------------------------
    # STATE MACHINE
    # ---------------------------

    # DEFAULT State: Home robot or fetch next task
    if cycletools.var.current_state == "DEFAULT":

        if rpi.io.I_1.value == 0:
            rpi.io.O_7.value = 1  # Move Axis 1 home

        elif rpi.io.I_2.value == 0:
            rpi.io.O_7.value = 0
            rpi.io.O_9.value = 1  # Move Axis 2 home

        elif rpi.io.I_3.value == 0:
            rpi.io.O_9.value = 0
            rpi.io.O_11.value = 1  # Move Axis 3 home

        else:
            rpi.io.O_11.value = 0

            # Wait after homing (0.5 seconds)
            if cycletools.var.delay_counter < 50:
                cycletools.var.delay_counter += 1

            # Reset counters
            elif cycletools.var.delay_counter == 50:
                rpi.io.Counter_5.reset()
                rpi.io.Counter_7.reset()
                rpi.io.Counter_9.reset()
                print("Counters reset.")
                cycletools.var.delay_counter += 1

            # Additional wait
            elif cycletools.var.delay_counter < 100:
                cycletools.var.delay_counter += 1

            else:
                cycletools.var.delay_counter = 0

                if not task_queue.empty():
                    cycletools.var.current_task = task_queue.get()
                    tasks_in_queue_set.remove(cycletools.var.current_task)

                    print(f"Starting new task: Pick up {cycletools.var.current_task[0]} and drop at {cycletools.var.current_task[1]}")

                    cycletools.var.is_picking_up = True
                    cycletools.var.current_position = cycletools.var.positions[
                        cycletools.var.current_task[0]
                    ]
                    cycletools.var.current_state = "MOVE_TO_POS"
                    cycletools.var.move_axis_state = "AXIS1"

                else:
                    print("All tasks completed. Robot switching to idle.")
                    cycletools.var.current_state = "IDLE"


    # MOVE_TO_POS State
    elif cycletools.var.current_state == "MOVE_TO_POS":

        target_pos = cycletools.var.current_position
        POSITION_TOLERANCE = 10

        # Axis 1
        if cycletools.var.move_axis_state == "AXIS1":
            current_pos = rpi.io.Counter_9.value
            target = target_pos[0]

            if abs(current_pos - target) <= POSITION_TOLERANCE:
                rpi.io.O_12.value = 0
                rpi.io.O_11.value = 0
                print("Axis 1 reached target. Moving to Axis 2.")
                cycletools.var.move_axis_state = "AXIS2"
            else:
                if current_pos < target:
                    rpi.io.O_12.value = 1
                else:
                    rpi.io.O_11.value = 1

        # Axis 2
        elif cycletools.var.move_axis_state == "AXIS2":
            current_pos = rpi.io.Counter_7.value
            target = target_pos[1]

            if abs(current_pos - target) <= POSITION_TOLERANCE:
                rpi.io.O_10.value = 0
                rpi.io.O_9.value = 0
                print("Axis 2 reached target. Moving to Axis 3.")
                cycletools.var.move_axis_state = "AXIS3"
            else:
                if current_pos < target:
                    rpi.io.O_10.value = 1
                else:
                    rpi.io.O_9.value = 1

        # Axis 3
        elif cycletools.var.move_axis_state == "AXIS3":
            current_pos = rpi.io.Counter_5.value
            target = target_pos[2]

            if abs(current_pos - target) <= POSITION_TOLERANCE:
                rpi.io.O_8.value = 0
                rpi.io.O_7.value = 0
                print("Axis 3 reached target. All axes in position.")

                if cycletools.var.delay_counter < 50:
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.delay_counter = 0
                    if cycletools.var.is_picking_up:
                        cycletools.var.current_state = "GRAB"
                    else:
                        cycletools.var.current_state = "DROP"

            else:
                if current_pos < target:
                    rpi.io.O_8.value = 1
                else:
                    rpi.io.O_7.value = 1


    # GRAB State
    elif cycletools.var.current_state == "GRAB":

        rpi.io.O_13.value = 1  # Compressor ON

        if cycletools.var.delay_counter < 50:
            cycletools.var.delay_counter += 1
        else:
            rpi.io.O_14.value = 1  # Magnet ON
            cycletools.var.delay_counter = 0
            cycletools.var.current_state = "UP_AND_IN"


    # DROP State
    elif cycletools.var.current_state == "DROP":

        rpi.io.O_14.value = 0  # Magnet OFF

        if cycletools.var.delay_counter < 50:
            cycletools.var.delay_counter += 1
        else:
            cycletools.var.delay_counter = 0
            cycletools.var.current_state = "UP_AND_IN"


def programend():
    print("Program end: resetting outputs...")

    rpi.io.O_7.value = 0
    rpi.io.O_9.value = 0
    rpi.io.O_11.value = 0
    rpi.io.O_12.value = 0
    rpi.io.O_10.value = 0
    rpi.io.O_8.value = 0
    rpi.io.O_13.value = 0
    rpi.io.O_14.value = 0


rpi.handlesignalend(programend)

print("Starting robot cycle loop...")
rpi.cycleloop(cycleprogram, cycletime=rpi.cycletime)
print("Cycle loop ended.")