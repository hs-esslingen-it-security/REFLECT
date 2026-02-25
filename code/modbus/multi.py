import revpimodio2
import threading
import time

# RevPi object with automatic IO refresh
rpi = revpimodio2.RevPiModIO(autorefresh=True)


# Events and locks
cycle_event = threading.Event()
stop_event = threading.Event()

# task1 variables
task1_state = "PREPARING_DROPOFF"
task1_state_start_time = 0.0
task1_door = 0
task1_slider = 0

# task2 variables
task2_state = "WAITING_PICKUP"
task2_state_start_time = 0.0
task2_default = 0
task2_down = 0
task2_vac = 0

# --- Task 3 variables (turntable and conveyor belt) ---
task3_state = "POS_VAC"
task3_state_start_time = 0.0
task3_default = 0
task3_belt = 0
VacFinished2 = 0

# shared variables
OvenFinished = 0
VacFinished = 0


# --- Task 1 kiln/oven ---
def task1():
    global task1_state, task1_state_start_time, task1_door, task1_slider, OvenFinished
    while not stop_event.is_set():
        if cycle_event.wait(
            timeout=0.05
        ):  # Wait for event, with a small timeout for robustness
            cycle_event.clear()  # Clear the event immediately after waking up

            # Kiln/oven
            # send status to gripper state variable to suction gripper
            if task1_state == "PREPARING_DROPOFF":  # prepare for drop-off
                if task1_door == 0:
                    if task1_state_start_time == 0.0:
                        task1_state_start_time = time.monotonic()
                        rpi.io.O_9.value = 1  # light on

                    if (time.monotonic() - task1_state_start_time) < 1.0:
                        pass

                    else:
                        rpi.io.O_13.value = 1  # open door
                        task1_state_start_time = 0.0
                        task1_door = 1

                elif task1_slider == 0:
                    if rpi.io.I_7.value == 0:  # reference switch: oven slider outside
                        rpi.io.O_6.value = 1  # move slider out

                    else:  # it is outside
                        rpi.io.O_6.value = 0  # stop moving slider out
                        task1_state_start_time = 0.0
                        task1_slider = 1

                else:
                    task1_state = "WAITING_DROPOFF"
                    rpi.io.Output_1.value = 1  # ready for drop-off

            elif task1_state == "WAITING_DROPOFF":
                # As soon as an object is detected, start the timer
                if rpi.io.I_9.value == 0:  # photoelectric sensor detects object

                    if task1_state_start_time == 0.0:
                        task1_state_start_time = time.monotonic()
                        rpi.io.Output_1.value = 0  # not ready for drop-off

                # After 5 seconds from detection, change state (time for suction gripper to move away)
                if (
                    task1_state_start_time != 0.0
                    and (time.monotonic() - task1_state_start_time) >= 5.0
                ):
                    task1_state_start_time = 0.0
                    task1_state = "PREPARE_BURNING"

            elif task1_state == "PREPARE_BURNING":  # close oven
                if task1_slider == 1:
                    if rpi.io.I_6.value == 0:  # reference switch: oven slider inside
                        rpi.io.O_5.value = 1  # move slider in

                    else:  # slider is in
                        rpi.io.O_5.value = 0  # stop moving slider in
                        task1_state_start_time = 0.0
                        task1_slider = 0

                elif task1_door == 1:
                    if task1_state_start_time == 0.0:
                        task1_state_start_time = time.monotonic()

                    if (time.monotonic() - task1_state_start_time) < 1.0:
                        rpi.io.O_13.value = 0  # close door

                    else:
                        task1_state_start_time = 0.0
                        task1_door = 0
                else:
                    rpi.io.O_9.value = 0  # light off
                    task1_state = "BURNING"

            elif task1_state == "BURNING":
                if task1_state_start_time == 0.0:
                    task1_state_start_time = time.monotonic()

                if (time.monotonic() - task1_state_start_time) < 1.0:
                    pass  # Wait for the delay

                else:
                    task1_state = "PREPARE_PICKUP"
                    task1_state_start_time = 0.0

            elif task1_state == "PREPARE_PICKUP":
                if task1_door == 0:
                    if task1_state_start_time == 0.0:  # first time entering this part
                        task1_state_start_time = time.monotonic()
                        rpi.io.O_9.value = 1  # light on

                    if (time.monotonic() - task1_state_start_time) < 1.0:
                        pass  # Wait for the delay

                    else:
                        rpi.io.O_13.value = 1  # open door
                        task1_state_start_time = 0.0
                        task1_door = 1

                elif task1_slider == 0:

                    if rpi.io.I_7.value == 0:  # reference switch: oven slider outside
                        rpi.io.O_6.value = 1  # move slider out

                    else:  # slider is out
                        rpi.io.O_6.value = 0  # stop slider out
                        task1_state_start_time = 0.0
                        task1_slider = 1
                else:
                    OvenFinished = 1
                    task1_state = "WAITING_PICKUP"

            elif task1_state == "WAITING_PICKUP":
                if OvenFinished == 0:
                    task1_state = "PREPARING_DROPOFF"


# --- Task 2 suction gripper ---
def task2():
    global task2_state, task2_state_start_time, OvenFinished, VacFinished, task2_default, task2_down, task2_vac
    while not stop_event.is_set():
        if cycle_event.wait(timeout=0.05):
            cycle_event.clear()

            # Suction gripper
            if task2_state == "WAITING_PICKUP":
                if task2_default == 0:
                    rpi.io.O_8.value = 1  # drive towards reference
                    if rpi.io.I_5.value == 1:  # reference reached
                        rpi.io.O_8.value = 0
                        task2_default = 1

                else:
                    if OvenFinished == 1:
                        task2_state = "DRIVE_PICKUP"
                        task2_state_start_time = 0.0

            elif task2_state == "DRIVE_PICKUP":
                if rpi.io.I_8.value == 0:  # reference position oven
                    rpi.io.O_7.value = 1  # motor towards oven on
                    task2_default = 0

                else:
                    rpi.io.O_7.value = 0  # motor oven off
                    task2_state = "PICKUP"

            elif task2_state == "PICKUP":
                if task2_down == 0:  # move down
                    if task2_state_start_time == 0.0:
                        task2_state_start_time = time.monotonic()
                        rpi.io.O_12.value = 1  # move down (lower valve)
                        rpi.io.O_11.value = 0  # ensure no vacuum

                    if (time.monotonic() - task2_state_start_time) >= 1.0:
                        task2_state_start_time = 0.0
                        task2_down = 1  # down movement finished

                elif task2_vac == 0 and task2_down == 1:  # suction (after down)
                    if task2_state_start_time == 0.0:
                        task2_state_start_time = time.monotonic()
                        rpi.io.O_11.value = 1  # suction on

                    if (time.monotonic() - task2_state_start_time) >= 1.0:
                        task2_state_start_time = 0.0
                        task2_vac = 1  # suction finished

                elif task2_down == 1 and task2_vac == 1:  # move up (after suction)
                    if task2_state_start_time == 0.0:
                        task2_state_start_time = time.monotonic()
                        rpi.io.O_12.value = 0  # move up

                    if (time.monotonic() - task2_state_start_time) >= 1.0:
                        task2_state_start_time = 0.0
                        task2_down = 0  # reset for next cycle
                        task2_state = "DRIVE_DROPOFF"

            elif task2_state == "DRIVE_DROPOFF":
                if task2_default == 0:
                    rpi.io.O_8.value = 1  # motor on
                    if rpi.io.I_5.value == 1:  # reference switch turntable
                        rpi.io.O_8.value = 0  # motor off
                        task2_default = 1

                else:
                    # This value is also set earlier, but keeping it as in the original code
                    OvenFinished = 0
                    VacFinished = 1  # suction gripper has an object and is at drop-off position
                    task2_state = "WAITING_DROPOFF"
                    task2_default = 0  # reset for next travel

            elif task2_state == "WAITING_DROPOFF":
                # This implies another task (e.g. task3) set VacFinished back to 0
                if VacFinished == 0:
                    # If task3 has taken the object, task2 may start dropping off
                    task2_state = "DROPOFF"

            elif task2_state == "DROPOFF":
                if task2_down == 0:  # move down to place
                    if task2_state_start_time == 0.0:
                        task2_state_start_time = time.monotonic()
                        rpi.io.O_12.value = 1  # move down

                    if (time.monotonic() - task2_state_start_time) >= 1.0:
                        task2_state_start_time = 0.0
                        task2_down = 1

                elif task2_vac == 1:  # release (vacuum off)
                    if task2_state_start_time == 0.0:
                        task2_state_start_time = time.monotonic()
                        rpi.io.O_11.value = 0  # vacuum off

                    if (time.monotonic() - task2_state_start_time) >= 1.0:
                        task2_state_start_time = 0.0
                        task2_vac = 0  # vacuum is off

                elif task2_down == 1 and task2_vac == 0:  # move up (after placing)
                    if task2_state_start_time == 0.0:
                        task2_state_start_time = time.monotonic()
                        rpi.io.O_12.value = 0  # move up

                    if (time.monotonic() - task2_state_start_time) >= 1.0:
                        task2_state_start_time = 0.0
                        task2_down = 0  # reset for next cycle
                        VacFinished = 0  # signals that the object has been placed
                        task2_state = "WAITING_PICKUP"  # back to start state


# --- Task 3 turntable and conveyor belt ---
def task3():
    global task3_state, task3_default, task3_belt, task3_state_start_time, VacFinished, VacFinished2
    while not stop_event.is_set():
        if cycle_event.wait(timeout=0.05):
            cycle_event.clear()

            if task3_state == "POS_VAC":
                # Move to vacuum position and wait until VacFinished is 1
                if task3_default == 0:  # move to default
                    if rpi.io.I_1.value == 0:
                        rpi.io.O_2.value = 1
                    else:
                        rpi.io.O_2.value = 0
                        task3_default = 1
                else:  # wait
                    if VacFinished == 1:
                        VacFinished = 0
                        VacFinished2 = 1

                    elif VacFinished2 == 1:
                        if task3_state_start_time == 0:
                            task3_state_start_time = time.monotonic()

                        if (time.monotonic() - task3_state_start_time) < 5.0:
                            pass  # wait for the delay
                        else:
                            task3_state = "POS_SAW"
                            VacFinished2 = 0
                            task3_default = 0
                            task3_state_start_time = 0.0

            elif task3_state == "POS_SAW":
                if rpi.io.I_4.value == 0:
                    rpi.io.O_1.value = 1
                else:
                    rpi.io.O_1.value = 0
                    task3_state = "SAW"

            elif task3_state == "SAW":
                if task3_state_start_time == 0:
                    task3_state_start_time = time.monotonic()
                    rpi.io.O_4.value = 1

                if (time.monotonic() - task3_state_start_time) < 30.0:
                    pass  # wait for the delay
                else:
                    task3_state = "POS_BELT"
                    rpi.io.O_4.value = 0
                    task3_state_start_time = 0.0

            elif task3_state == "POS_BELT":
                if task3_belt == 0:
                    if rpi.io.I_2.value == 0:
                        rpi.io.O_1.value = 1
                    else:
                        rpi.io.O_1.value = 0
                        task3_belt = 1

                else:
                    if task3_state_start_time == 0:
                        task3_state_start_time = time.monotonic()

                    if (time.monotonic() - task3_state_start_time) < 1.0:
                        pass
                    else:
                        rpi.io.O_14.value = 1
                        task3_belt = 0
                        task3_state_start_time = 0.0
                        task3_state = "BELT"

            elif task3_state == "BELT":
                if task3_belt == 0:
                    rpi.io.O_3.value = 1
                    if rpi.io.I_3.value == 0:
                        task3_belt = 1

                else:
                    if task3_state_start_time == 0:
                        task3_state_start_time = time.monotonic()
                        rpi.io.O_14.value = 0

                    if (time.monotonic() - task3_state_start_time) < 2.0:
                        pass  # wait for the delay
                    else:
                        task3_belt = 0
                        task3_state_start_time = 0.0
                        rpi.io.O_3.value = 0
                        task3_state = "POS_VAC"


# --- Cyclic main function, called by revpimodio2 ---
def main_cycle(cycletools):
    if cycletools.first:
        rpi.io.O_1.value = 0
        rpi.io.O_2.value = 0
        rpi.io.O_3.value = 0
        rpi.io.O_4.value = 0
        rpi.io.O_5.value = 0
        rpi.io.O_6.value = 0
        rpi.io.O_7.value = 0
        rpi.io.O_8.value = 0
        rpi.io.O_9.value = 0
        rpi.io.O_11.value = 0
        rpi.io.O_12.value = 0
        rpi.io.O_13.value = 0
        rpi.io.O_14.value = 0
        rpi.io.O_10.value = 1  # compressor permanently on

    # Send signal to all tasks
    cycle_event.set()


# --- Program shutdown ---
def programend():
    stop_event.set()
    rpi.io.O_10.value = 0  # compressor off
    rpi.io.O_1.value = 0
    rpi.io.O_2.value = 0
    rpi.io.O_3.value = 0
    rpi.io.O_4.value = 0
    rpi.io.O_5.value = 0
    rpi.io.O_6.value = 0
    rpi.io.O_7.value = 0
    rpi.io.O_8.value = 0
    rpi.io.O_9.value = 0
    rpi.io.O_11.value = 0
    rpi.io.O_12.value = 0
    rpi.io.O_13.value = 0
    rpi.io.O_14.value = 0
    rpi.exit()  # cleanly shut down RevPiModIO


# --- Main program flow ---
# Start threads for tasks
threading.Thread(target=task1, daemon=True).start()
threading.Thread(target=task2, daemon=True).start()
threading.Thread(target=task3, daemon=True).start()

# Clean shutdown on CTRL+C
rpi.handlesignalend(programend)

# Start cycle loop
rpi.cycleloop(main_cycle, cycletime=10)  # 10 ms cycle