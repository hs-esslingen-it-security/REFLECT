import revpimodio2
import queue

# Global queue for tasks
task_queue = queue.Queue()

# Add a helper data structure to track the tasks currently in the queue
# This set tracks which tasks are already in the queue
tasks_in_queue_set = set()


# Function to safely add a task to the queue (with duplicate check)
def add_task_to_queue(task):
    if task not in tasks_in_queue_set:
        task_queue.put(task)
        tasks_in_queue_set.add(task)
        print(f"Task {task} added to the list.")
        return True

    else:
        print(f"Task {task} is already in the list. Not added.")
        return False


# Initialize the RevPi-ModIO interface with automatic refresh
rpi = revpimodio2.RevPiModIO(autorefresh=True)
rpi.cycletime = 10  # Set the cycle time (in milliseconds)


def cycleprogram(cycletools):
    # Initialization at program start
    if cycletools.first:
        print("Initializing the robot gripper...")

        # Define robot positions - using a dictionary for better readability and access
        cycletools.var.positions = {
            "white_pick_up": [910, 690, 1700],     # white pick-up
            "red_pick_up": [760, 810, 1700],       # red pick-up
            "blue_pick_up": [625, 1100, 1700],     # blue pick-up
            "storage": [2700, 350, 300],           # unified storage position
            "multi_drop_off": [1805, 1680, 1000],  # multi drop-off
        }

        cycletools.var.current_state = "DEFAULT"

        # Current task (tuple: (pick_up_pos_name, drop_off_pos_name, storage_slot))
        cycletools.var.current_task = None

        # True when moving to pick-up position, False for drop-off position
        cycletools.var.is_picking_up = True

        cycletools.var.zyklus = 0
        cycletools.var.delay_counter = 0

        # Used in GRAP/DROP states for one-time execution
        cycletools.var.first = 0

        # New sub-state for axis movement
        cycletools.var.move_axis_state = "AXIS1"

        # Ensure all motors and grippers are off initially
        rpi.io.O_7.value = 0
        rpi.io.O_9.value = 0
        rpi.io.O_11.value = 0
        rpi.io.O_12.value = 0
        rpi.io.O_10.value = 0
        rpi.io.O_8.value = 0
        rpi.io.O_13.value = 0
        rpi.io.O_14.value = 0

    # --- STATE MACHINE ---

    # DEFAULT state: home the robot or fetch next task
    if cycletools.var.current_state == "DEFAULT":
        if rpi.io.I_1.value == 0:
            rpi.io.O_7.value = 1  # Move axis 1 to home

        elif rpi.io.I_2.value == 0:
            rpi.io.O_7.value = 0
            rpi.io.O_9.value = 1  # Move axis 2 to home

        elif rpi.io.I_3.value == 0:
            rpi.io.O_9.value = 0
            rpi.io.O_11.value = 1  # Move axis 3 to home

        else:  # All axes are home
            rpi.io.O_11.value = 0

            # New phased logic for wait time and counter reset
            # Phase 1: wait after reaching home position (50 cycles = 0.5s)
            if cycletools.var.delay_counter < 50:
                cycletools.var.delay_counter += 1

            # Phase 2: reset counters (one-time after phase 1)
            elif cycletools.var.delay_counter == 50:
                rpi.io.Counter_5.reset()
                rpi.io.Counter_7.reset()
                rpi.io.Counter_9.reset()
                print("Counters reset.")
                cycletools.var.delay_counter += 1  # immediately continue to phase 3

            # Phase 3: wait after resetting counters (50 cycles = 0.5s)
            elif cycletools.var.delay_counter < 50 + 50:
                cycletools.var.delay_counter += 1

            else:  # all phases completed
                cycletools.var.delay_counter = 0  # reset for next use

                # Check for next task in the queue
                if not task_queue.empty():
                    cycletools.var.current_task = task_queue.get()  # get next task

                    # IMPORTANT: remove from the set since it has been taken from the queue
                    tasks_in_queue_set.remove(cycletools.var.current_task)

                    print(
                        f"Starting new task: pick up from {cycletools.var.current_task[0]} and drop off at {cycletools.var.current_task[1]}"
                    )

                    # Always start with the pick-up position
                    cycletools.var.is_picking_up = True

                    # Waiting logic before moving
                    if cycletools.var.current_task[0] == "storage":
                        # If picking up from storage: go to waiting state first to request permission
                        cycletools.var.current_position = cycletools.var.positions[
                            cycletools.var.current_task[0]
                        ]
                        cycletools.var.first = 0
                        cycletools.var.current_state = "WAIT_FOR_PICKUP_READY"

                    else:
                        # Otherwise start moving immediately
                        cycletools.var.current_position = cycletools.var.positions[
                            cycletools.var.current_task[0]
                        ]
                        cycletools.var.current_state = "MOVE_TO_POS"
                        cycletools.var.move_axis_state = "AXIS1"

                else:
                    print("All tasks completed. Robot switches to idle.")
                    cycletools.var.current_state = "IDLE"

    # NEW STATE: wait for approval to pick up from storage
    elif cycletools.var.current_state == "WAIT_FOR_PICKUP_READY":

        # List of pick-up positions that depend on confirmation from the MULTISTATION
        multistation_dependent_pickups = [
            "white_pick_up",
            "red_pick_up",
            "blue_pick_up",
        ]

        # Only reached when picking up from storage
        if cycletools.var.current_task and cycletools.var.current_task[0] == "storage":
            # Step 1: request storage (only once when entering)
            if cycletools.var.first == 0:
                # The pallet must not be brought back while we are requesting it
                rpi.io.Output_Word_2.value = 0
                rpi.io.Output_Word_1.value = cycletools.var.current_task[2]  # request to storage
                print(
                    f"WAIT_FOR_PICKUP_READY: Sending request {cycletools.var.current_task[2]} to storage..."
                )
                cycletools.var.first = 1

            # Step 2: wait for storage approval
            if rpi.io.Input_Word_1.value != 0:  # assumption: != 0 = storage is ready/confirmed
                print("WAIT_FOR_PICKUP_READY: Storage approved. Starting movement.")
                cycletools.var.first = 0
                cycletools.var.current_state = "MOVE_TO_POS"
                cycletools.var.move_axis_state = "AXIS1"
            else:
                return  # stay here until storage is ready

        elif (
            cycletools.var.current_task
            and cycletools.var.current_task[0] in multistation_dependent_pickups
        ):
            # Here you would check the suction/light-barrier signals of the sorting line / multistation
            # REPLACE THIS PLACEHOLDER with your real multistation input/variable!
            if 1 == 0:  # put the real condition here
                print(
                    f"GRAP: Robot waits for multistation for {cycletools.var.current_task[0]} (signal not received yet)..."
                )
                return

    # EXTENDED STATE: wait for approval for drop-off (storage OR multistation)
    elif cycletools.var.current_state == "WAIT_FOR_DROP_OFF_READY":
        target_drop_off = cycletools.var.current_task[1] if cycletools.var.current_task else None

        if target_drop_off == "storage":
            # --- wait-for-storage logic ---

            # Step 1: request storage (only once when entering)
            if cycletools.var.first == 0:
                rpi.io.Output_Word_2.value = 0
                rpi.io.Output_Word_1.value = cycletools.var.current_task[2]  # request target slot
                print(
                    f"WAIT_FOR_DROP_OFF_READY: Sending request {cycletools.var.current_task[2]} for storage placement..."
                )
                cycletools.var.first = 1

            # Step 2: wait for storage approval
            if rpi.io.Input_Word_1.value != 0:  # assumption: != 0 = storage is ready/confirmed
                print("WAIT_FOR_DROP_OFF_READY: Storage approved for placement. Starting movement.")
                cycletools.var.first = 0
                cycletools.var.current_state = "MOVE_TO_POS"
                cycletools.var.move_axis_state = "AXIS1"
            else:
                return

        elif target_drop_off == "multi_drop_off":
            # --- wait-for-multistation logic ---

            # IMPORTANT: replace '== 1' with your actual READY condition
            # Assumption: Input_Word_1_i05 controls multistation release
            if rpi.io.Input_Word_1_i05.value == 1:
                print("WAIT_FOR_DROP_OFF_READY: Multistation approved. Starting movement.")
                cycletools.var.first = 0
                cycletools.var.current_state = "MOVE_TO_POS"
                cycletools.var.move_axis_state = "AXIS1"
            else:
                print("WAIT_FOR_DROP_OFF_READY: Robot is waiting for multistation (drop-off approval)...")
                return

    # IDLE state: robot waits for tasks (can later be filled by external triggers)
    elif cycletools.var.current_state == "IDLE":
        if cycletools.var.zyklus == 0:
            add_task_to_queue(("storage", "multi_drop_off", 1))
            add_task_to_queue(("storage", "multi_drop_off", 2))
            add_task_to_queue(("storage", "multi_drop_off", 3))
            cycletools.var.zyklus = 1

        elif cycletools.var.zyklus == 1:
            add_task_to_queue(("white_pick_up", "storage", 3))
            add_task_to_queue(("red_pick_up", "storage", 2))
            add_task_to_queue(("blue_pick_up", "storage", 1))
            cycletools.var.zyklus = 2

        elif cycletools.var.zyklus == 2:
            add_task_to_queue(("storage", "multi_drop_off", 4))
            add_task_to_queue(("storage", "multi_drop_off", 5))
            add_task_to_queue(("storage", "multi_drop_off", 6))
            cycletools.var.zyklus = 3

        elif cycletools.var.zyklus == 3:
            add_task_to_queue(("white_pick_up", "storage", 6))
            add_task_to_queue(("red_pick_up", "storage", 5))
            add_task_to_queue(("blue_pick_up", "storage", 4))
            cycletools.var.zyklus = 4

        elif cycletools.var.zyklus == 4:
            add_task_to_queue(("storage", "multi_drop_off", 7))
            add_task_to_queue(("storage", "multi_drop_off", 8))
            add_task_to_queue(("storage", "multi_drop_off", 9))
            cycletools.var.zyklus = 5

        elif cycletools.var.zyklus == 5:
            add_task_to_queue(("white_pick_up", "storage", 9))
            add_task_to_queue(("red_pick_up", "storage", 8))
            add_task_to_queue(("blue_pick_up", "storage", 7))
            cycletools.var.zyklus = 0

        cycletools.var.current_state = "DEFAULT"

    # MOVE_TO_POS state: move to the target position
    elif cycletools.var.current_state == "MOVE_TO_POS":
        target_pos = cycletools.var.current_position
        POSITION_TOLERANCE = 10

        # --- Axis 1 control ---
        if cycletools.var.move_axis_state == "AXIS1":
            current_pos_ax1 = rpi.io.Counter_9.value
            target_pos_ax1 = target_pos[0]
            if abs(current_pos_ax1 - target_pos_ax1) <= POSITION_TOLERANCE:
                rpi.io.O_12.value = 0
                rpi.io.O_11.value = 0
                print("Axis 1 within target range. Starting axis 2.")
                cycletools.var.move_axis_state = "AXIS2"
            else:
                if current_pos_ax1 < target_pos_ax1:
                    rpi.io.O_12.value = 1  # axis 1 forward
                    rpi.io.O_11.value = 0
                elif current_pos_ax1 > target_pos_ax1:
                    rpi.io.O_12.value = 0
                    rpi.io.O_11.value = 1  # axis 1 backward

        # --- Axis 2 control ---
        elif cycletools.var.move_axis_state == "AXIS2":
            current_pos_ax2 = rpi.io.Counter_7.value
            target_pos_ax2 = target_pos[1]
            if abs(current_pos_ax2 - target_pos_ax2) <= POSITION_TOLERANCE:
                rpi.io.O_10.value = 0
                rpi.io.O_9.value = 0
                print("Axis 2 within target range. Starting axis 3.")
                cycletools.var.move_axis_state = "AXIS3"
            else:
                if current_pos_ax2 < target_pos_ax2:
                    rpi.io.O_10.value = 1  # axis 2 forward
                    rpi.io.O_9.value = 0
                elif current_pos_ax2 > target_pos_ax2:
                    rpi.io.O_10.value = 0
                    rpi.io.O_9.value = 1  # axis 2 backward

        # --- Axis 3 control ---
        elif cycletools.var.move_axis_state == "AXIS3":
            current_pos_ax3 = rpi.io.Counter_5.value
            target_pos_ax3 = target_pos[2]
            if abs(current_pos_ax3 - target_pos_ax3) <= POSITION_TOLERANCE:
                rpi.io.O_8.value = 0
                rpi.io.O_7.value = 0
                print("Axis 3 within target range. All axes in position.")

                # All axes reached their targets, transition to next state
                if cycletools.var.delay_counter < 50:  # 0.5 second delay
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.delay_counter = 0
                    cycletools.var.current_state = "GRAP" if cycletools.var.is_picking_up else "DROP"
            else:
                if current_pos_ax3 < target_pos_ax3:
                    rpi.io.O_8.value = 1  # axis 3 forward
                    rpi.io.O_7.value = 0
                elif current_pos_ax3 > target_pos_ax3:
                    rpi.io.O_8.value = 0
                    rpi.io.O_7.value = 1  # axis 3 backward

    # GRAP state: activate gripper to pick up
    elif cycletools.var.current_state == "GRAP":

        # If 'storage' is reached here, approval was already obtained in WAIT_FOR_PICKUP_READY
        if cycletools.var.current_task and cycletools.var.current_task[0] == "storage":
            # Reset the storage request because the pallet has been reached/accepted
            rpi.io.Output_Word_1.value = 0
            print("GRAP: Storage request reset.")

        cycletools.var.first = 0
        rpi.io.O_13.value = 1  # compressor on
        if cycletools.var.delay_counter < 50:  # 0.5 second delay for compressor
            cycletools.var.delay_counter += 1
        else:
            rpi.io.O_14.value = 1  # grip / magnet on
            cycletools.var.delay_counter = 0
            cycletools.var.current_state = "UP_AND_IN"

    # UP_AND_IN state: retract axes after pick-up (or drop-off)
    elif cycletools.var.current_state == "UP_AND_IN":
        if rpi.io.I_1.value == 0:
            rpi.io.O_7.value = 1
        elif rpi.io.I_2.value == 0:
            rpi.io.O_7.value = 0
            rpi.io.O_9.value = 1
        else:
            rpi.io.O_9.value = 0

            # Phased logic: wait + counter reset
            if cycletools.var.delay_counter < 50:
                cycletools.var.delay_counter += 1

            elif cycletools.var.delay_counter == 50:
                rpi.io.Counter_5.reset()
                rpi.io.Counter_7.reset()
                # not counter 9
                print("Counters reset.")
                cycletools.var.delay_counter += 1

            elif cycletools.var.delay_counter < 100:
                cycletools.var.delay_counter += 1

            else:
                cycletools.var.delay_counter = 0

                # If picked up from storage, the pallet can now be returned
                if cycletools.var.current_task[0] == "storage":
                    rpi.io.Output_Word_2.value = 1
                    print("Signal to storage: approval to bring back/finish the removal.")

                if cycletools.var.current_task[1] == "storage":
                    rpi.io.Output_Word_2.value = 1
                    print("DROP: Signal to storage: approval to bring back/finish the placement.")

                if cycletools.var.is_picking_up:
                    # Just picked up -> now move to drop-off position
                    cycletools.var.is_picking_up = False
                    drop_off_pos_name = cycletools.var.current_task[1]
                    cycletools.var.current_position = cycletools.var.positions[drop_off_pos_name]
                    cycletools.var.first = 0

                    # If drop-off is storage OR multi_drop_off, wait for approval first
                    if drop_off_pos_name in ("storage", "multi_drop_off"):
                        cycletools.var.current_state = "WAIT_FOR_DROP_OFF_READY"
                    else:
                        cycletools.var.current_state = "MOVE_TO_POS"
                        cycletools.var.move_axis_state = "AXIS1"

                else:
                    # Just dropped off -> task complete
                    cycletools.var.current_task = None
                    cycletools.var.current_state = "DEFAULT"

    # DROP state: release item
    elif cycletools.var.current_state == "DROP":
        # List of drop-off positions that require confirmation from STORAGE
        storage_dependent_dropoffs = ["storage"]

        if (
            cycletools.var.current_task
            and cycletools.var.current_task[1] in storage_dependent_dropoffs
        ):
            # No waiting/request here, since it already happened in WAIT_FOR_DROP_OFF_READY
            rpi.io.Output_Word_1.value = 0
            print("DROP: Storage request for placement reset.")

        cycletools.var.first = 0
        rpi.io.O_14.value = 0  # open gripper / magnet off

        if cycletools.var.delay_counter < 50:  # 0.5 second delay to release
            cycletools.var.delay_counter += 1
        else:
            cycletools.var.delay_counter = 0
            cycletools.var.current_state = "UP_AND_IN"


# This function is called when the program ends (e.g., Ctrl+C)
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


# Register the programend function for graceful shutdown
rpi.handlesignalend(programend)

# Start the main program loop (cycle operation)
print("Starting the robot cycle loop...")
rpi.cycleloop(cycleprogram, cycletime=rpi.cycletime)
print("Robot cycle loop finished.")