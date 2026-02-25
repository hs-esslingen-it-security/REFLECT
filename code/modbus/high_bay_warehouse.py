import revpimodio2

# Initialize the RevPi-ModIO interface with automatic refresh
rpi = revpimodio2.RevPiModIO(autorefresh=True)
rpi.cycletime = 10  # Set the cycle time (in milliseconds)


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

    def CommanderLager(self, x, y):  # Move to a position at the storage area
        print()
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

    def CommanderLift(self, value=200):  # Lift up a bit
        if (
            rpi.io.Counter_7.value >= 4294967295 - value or rpi.io.Counter_7.value <= 50
        ):  # probably jumps to the maximum value and counts down, hence the ">"
            rpi.io.O_6.value = 1
            return 0

        else:
            rpi.io.O_6.value = 0
            return 1

    def CommanderDown(self, value=200):  # Lower down a bit / set down
        if rpi.io.Counter_7.value < value:  # counts upward
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

        # Define robot positions (first value = x, second = y).
        # Exactly in front of the pallet; +/- value is used to be above/below the position.
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
        cycletools.var.delay_counter = 0
        cycletools.var.first = 0
        cycletools.var.first2 = 0

    # --- STATE MACHINE ---

    # DEFAULT state: move to home/zero position at the beginning and end of a cycle
    if cycletools.var.current_state == "DEFAULT":
        if cycletools.var.commander.CommanderDefault() == 1:
            if cycletools.var.delay_counter <= 100:
                cycletools.var.delay_counter += 1
            else:
                cycletools.var.current_state = "IDLE"
                cycletools.var.delay_counter = 0

    # IDLE state: waiting
    elif cycletools.var.current_state == "IDLE":
        # Here the robot waits idle for a task via Modbus
        # cycletools.var.pos = cycletools.var.positions["1_1"]  # for testing

        if rpi.io.Input_1.value != 0:
            cycletools.var.pos = cycletools.var.positions[rpi.io.Input_1.value]
            cycletools.var.current_state = "MOVE_STORAGE_PICKUP"

    # MOVE_STORAGE_PICKUP: move to storage, lower (below for lifting), extend, lift, retract
    elif cycletools.var.current_state == "MOVE_STORAGE_PICKUP":

        # counter reset
        if cycletools.var.step == 0:
            if cycletools.var.first == 0:
                cycletools.var.commander.CommanderResetCounters()
                cycletools.var.first = 1
            else:
                if cycletools.var.delay_counter < 100:
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.step = 1
                    cycletools.var.delay_counter = 0
                    cycletools.var.first = 0

        # x and y axis storage
        elif cycletools.var.step == 1:
            if cycletools.var.first2 == 0:
                if (
                    cycletools.var.commander.CommanderLager(
                        cycletools.var.pos[0], cycletools.var.pos[1] + 50
                    )
                    == 1
                ):
                    cycletools.var.first2 = 1
            else:
                if cycletools.var.first == 0:
                    cycletools.var.commander.CommanderResetCounters()
                    cycletools.var.first = 1
                else:
                    if cycletools.var.delay_counter < 100:
                        cycletools.var.delay_counter += 1
                    else:
                        cycletools.var.step = 2
                        cycletools.var.first = 0
                        cycletools.var.first2 = 0
                        cycletools.var.delay_counter = 0

        # extend
        elif cycletools.var.step == 2:
            if cycletools.var.commander.CommanderOut() == 1:
                if cycletools.var.first == 0:
                    cycletools.var.commander.CommanderResetCounters()
                    cycletools.var.first = 1
                else:
                    if cycletools.var.delay_counter < 100:
                        cycletools.var.delay_counter += 1
                    else:
                        cycletools.var.step = 3
                        cycletools.var.first = 0
                        cycletools.var.delay_counter = 0

        # lift
        elif cycletools.var.step == 3:
            if cycletools.var.commander.CommanderLift() == 1:
                if cycletools.var.delay_counter <= 100:
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.delay_counter = 0
                    cycletools.var.step = 4

        # retract
        elif cycletools.var.step == 4:
            if cycletools.var.commander.CommanderIn() == 1:
                if cycletools.var.delay_counter <= 100:
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.delay_counter = 0
                    cycletools.var.step = 5

        elif cycletools.var.step == 5:
            cycletools.var.step = 0
            cycletools.var.current_state = "MOVE_BELT_DROPOFF"

    # MOVE_BELT_DROPOFF: move to belt (x and y default), extend, lower
    elif cycletools.var.current_state == "MOVE_BELT_DROPOFF":
        # move x and y
        if cycletools.var.step == 0:
            # move x and y (default function)
            if cycletools.var.commander.CommanderDefault() == 1:
                if cycletools.var.delay_counter <= 100:
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.delay_counter = 0
                    cycletools.var.step = 1

        # extend
        elif cycletools.var.step == 1:
            if cycletools.var.commander.CommanderOut() == 1:
                if cycletools.var.first == 0:
                    cycletools.var.commander.CommanderResetCounters()
                    cycletools.var.first = 1
                else:
                    if cycletools.var.delay_counter < 100:
                        cycletools.var.delay_counter += 1
                    else:
                        cycletools.var.step = 2
                        cycletools.var.first = 0
                        cycletools.var.delay_counter = 0

        # lower
        elif cycletools.var.step == 2:
            if cycletools.var.commander.CommanderDown(1400) == 1:
                if cycletools.var.delay_counter <= 100:
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.delay_counter = 0
                    cycletools.var.step = 0
                    cycletools.var.current_state = "BELT_FRONT"

    # BELT_FRONT: move forward and wait until it has been picked up
    elif cycletools.var.current_state == "BELT_FRONT":
        if rpi.io.I_2.value == 0:
            # belt on
            rpi.io.O_1.value = 1

        elif rpi.io.I_3.value == 0:
            # belt off
            if cycletools.var.delay_counter <= 25:
                cycletools.var.delay_counter += 1
            else:
                cycletools.var.delay_counter = 0
                rpi.io.O_1.value = 0

            rpi.io.Output_1.value = 1

        # Wait for signal that it was picked up (check Modbus signal)
        if rpi.io.Input_2.value == 1:
            rpi.io.Output_1.value = 0
            cycletools.var.current_state = "BELT_BACK"

    # BELT_BACK: move pallet backwards
    elif cycletools.var.current_state == "BELT_BACK":
        if rpi.io.I_3.value == 0:
            # belt reverse on
            rpi.io.O_2.value = 1

        elif rpi.io.I_2.value == 0:
            # belt reverse off
            rpi.io.O_2.value = 0
            cycletools.var.current_state = "BELT_PICKUP"

    # BELT_PICKUP: up and back in
    elif cycletools.var.current_state == "BELT_PICKUP":
        if cycletools.var.step == 0:
            # all the way up (default function)
            if cycletools.var.commander.CommanderDefault(0) == 1:
                cycletools.var.step = 1

        elif cycletools.var.step == 1:
            # retract in
            if cycletools.var.commander.CommanderIn() == 1:
                cycletools.var.step = 0
                cycletools.var.current_state = "MOVE_STORAGE_DROPOFF"

    # MOVE_STORAGE_DROPOFF: move to storage, lower (above for placing), extend, place, retract
    elif cycletools.var.current_state == "MOVE_STORAGE_DROPOFF":
        if cycletools.var.step == 0:
            if cycletools.var.first == 0:
                cycletools.var.commander.CommanderResetCounters()
                cycletools.var.first = 1
            else:
                if cycletools.var.delay_counter < 100:
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.step = 1
                    cycletools.var.delay_counter = 0
                    cycletools.var.first = 0

        # x and y axis storage
        elif cycletools.var.step == 1:
            if cycletools.var.first2 == 0:
                if (
                    cycletools.var.commander.CommanderLager(
                        cycletools.var.pos[0], cycletools.var.pos[1] - 150
                    )
                    == 1
                ):
                    cycletools.var.first2 = 1
            else:
                if cycletools.var.first == 0:
                    cycletools.var.commander.CommanderResetCounters()
                    cycletools.var.first = 1
                else:
                    if cycletools.var.delay_counter < 100:
                        cycletools.var.delay_counter += 1
                    else:
                        cycletools.var.step = 2
                        cycletools.var.first = 0
                        cycletools.var.first2 = 0
                        cycletools.var.delay_counter = 0

        # extend
        elif cycletools.var.step == 2:
            if cycletools.var.commander.CommanderOut() == 1:
                if cycletools.var.first == 0:
                    cycletools.var.commander.CommanderResetCounters()
                    cycletools.var.first = 1
                else:
                    if cycletools.var.delay_counter < 100:
                        cycletools.var.delay_counter += 1
                    else:
                        cycletools.var.step = 3
                        cycletools.var.first = 0
                        cycletools.var.delay_counter = 0

        # lower / place
        elif cycletools.var.step == 3:
            if cycletools.var.commander.CommanderDown() == 1:
                if cycletools.var.delay_counter <= 100:
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.delay_counter = 0
                    cycletools.var.step = 4

        # retract
        elif cycletools.var.step == 4:
            if cycletools.var.commander.CommanderIn() == 1:
                if cycletools.var.delay_counter <= 100:
                    cycletools.var.delay_counter += 1
                else:
                    cycletools.var.delay_counter = 0
                    cycletools.var.step = 5

        elif cycletools.var.step == 5:
            cycletools.var.step = 0
            cycletools.var.current_state = "DEFAULT"


# This function is called when the program ends (e.g. Ctrl+C)
def programend():
    print("Program end: resetting outputs...")
    # Ensure all outputs are turned off
    rpi.io.O_1.value = 0
    rpi.io.O_2.value = 0
    rpi.io.O_3.value = 0
    rpi.io.O_4.value = 0
    rpi.io.O_5.value = 0
    rpi.io.O_6.value = 0
    rpi.io.O_7.value = 0
    rpi.io.O_8.value = 0


# Register the programend function for graceful shutdown
rpi.handlesignalend(programend)

# Start the main program loop (cycle operation)
print("Starting the robot cycle loop...")
rpi.cycleloop(cycleprogram, cycletime=rpi.cycletime)
print("Robot cycle loop finished.")