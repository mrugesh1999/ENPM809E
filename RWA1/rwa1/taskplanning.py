def initiate():
    """
    A function to initiate all dictionaries
    :return: Initiate status dictionaries
    """

    # Assigning the variable global status
    global robot_status
    global tray_status
    global bin_status
    global goal

    # This dictionary consists of variables corresponding to robot
    robot_status = {
        "at_home": True,
        "at_bin": False,
        "at_tray": False,
        "is_empty_left": True,
        "is_empty_right": True,
        "left_gripper": "None",
        "right_gripper": "None",
    }

    # This dictionary has variables corresponding to tray status
    tray_status = {
        "tray_incomplete": True,
        "R": 0,
        "G": 0,
        "B": 0,
    }

    # This dictionary has variables corresponding to bin status
    bin_status = {
        "R": 0,
        "G": 0,
        "B": 0,
    }

    # This dictionary has variables corresponding to goal
    goal = {
        "R": 0,
        "G": 0,
        "B": 0,
    }

    # The function returns None
    return


def get_user_data():
    """
    A function to get data from the user
    :return: Get user data and store it into variables
    """

    # Assigning the variable global status
    global possible_to_compute

    print("====" * 12)      # Printing pattern
    flag = True             # Flag to loop till user data is received

    while flag:
        print("How many Red / Green / Blue parts in the bin ?")         # User data for Bin
        R_b, G_b, B_b = input().split()
        if (int(R_b) > 10) or (int(G_b) > 10) or (int(B_b) > 10):       # Checking if the entered data is valid or not
            print("The number of parts should be below 10")
        else:
            flag = False

    flag = True
    while flag:
        print("How many Red / Green / Blue parts already in the kit tray ?")    # User data for Kit (presently)
        R_t, G_t, B_t = input().split()
        if (int(R_t) > 10) or (int(G_t) > 10) or (int(B_t) > 10):       # Checking if the entered data is valid or not
            print("The number of parts should be below 10")
        else:
            flag = False

    flag = True
    while flag:
        print("How many Red / Green / Blue parts to place in the kit tray ?")   # User data for Kit (Needed)
        R_goal, G_goal, B_goal = input().split()
        if (int(R_goal) > 10) or (int(G_goal) > 10) or (int(B_goal) > 10):  # Checking if entered data is valid or not
            print("The number of parts should be below 10")
        else:
            flag = False

    print("====" * 12)      # Printing pattern

    possible_to_compute = True      # Setting flag True initially

    # Check if there are enough parts in the system to archive goal state
    if (int(R_b) + int(R_t)) < int(R_goal):
        needed = int(R_goal) - int(R_t)
        print(f"Not enough Red parts for kitting: {needed} needed, {R_b} available")
        possible_to_compute = False
    if (int(G_b) + int(G_t)) < int(G_goal):
        needed = int(G_goal) - int(G_t)
        print(f"Not enough Green parts for kitting: {needed} needed, {G_b} available")
        possible_to_compute = False
    if (int(B_b) + int(B_t)) < int(B_goal):
        needed = int(B_goal) - int(B_t)
        print(f"Not enough Blue parts for kitting: {needed} needed, {B_b} available")
        possible_to_compute = False

    # If kit already has more parts than goal, throw an error
    if int(R_t) > int(R_goal):
        possible_to_compute = False
        print("More Red parts in tray already than required")
    if int(G_t) > int(G_goal):
        possible_to_compute = False
        print("More Green parts in tray already than required")
    if int(B_t) > int(B_goal):
        possible_to_compute = False
        print("More Blue parts in tray already than required")

    # If all the user data is valid, store it into dictionaries as integers
    tray_status["R"] = int(R_t)
    tray_status["G"] = int(G_t)
    tray_status["B"] = int(B_t)

    bin_status["R"] = int(R_b)
    bin_status["G"] = int(G_b)
    bin_status["B"] = int(B_b)

    goal["R"] = int(R_goal)
    goal["G"] = int(G_goal)
    goal["B"] = int(B_goal)
    return                      # Returning nothing


def pick(arm, part):
    """
    A function to pick the part
    :param arm: The arm is either Left or Right accepted in str format
    :param part: The initial of the color of the part in str format
    :return: Changes the status variables accordingly
    """
    if part == "R" and arm == "left":               # Checking condition for parameter
        bin_status["R"] = bin_status["R"] - 1       # Decrementing that part from bin
        robot_status["is_empty_left"] = False       # Change variable is_empty_*arm* as False
        robot_status["left_gripper"] = "R"          # Assigning the part color's initial
        print("---Picked Red part with left arm")   # Printing the action

    if part == "G" and arm == "left":
        bin_status["G"] = bin_status["G"] - 1
        robot_status["is_empty_left"] = False
        robot_status["left_gripper"] = "G"
        print("---Picked Green part with left arm")

    if part == "B" and arm == "left":
        bin_status["B"] = bin_status["B"] - 1
        robot_status["is_empty_left"] = False
        robot_status["left_gripper"] = "B"
        print("---Picked Blue part with left arm")

    if part == "R" and arm == "right":
        bin_status["R"] = bin_status["R"] - 1
        robot_status["is_empty_right"] = False
        robot_status["right_gripper"] = "R"
        print("---Picked Red part with right arm")

    if part == "G" and arm == "right":
        bin_status["G"] = bin_status["G"] - 1
        robot_status["is_empty_right"] = False
        robot_status["right_gripper"] = "G"
        print("---Picked Green part with right arm")

    if part == "B" and arm == "right":
        bin_status["B"] = bin_status["B"] - 1
        robot_status["is_empty_right"] = False
        robot_status["right_gripper"] = "B"
        print("---Picked Blue part with right arm")
    return


def place(arm, part):
    """
    A function to place the part in tray
    :param arm: The arm is either Left or Right accepted in str format
    :param part: The initial of the color of the part in str format
    :return: Changes the status variables accordingly
    """
    if part == "R" and arm == "left":               # Checking condition for parameter
        tray_status["R"] = tray_status["R"] + 1     # Decrementing that part from bin
        robot_status["is_empty_left"] = True        # Change variable is_empty_*arm* as False
        robot_status["left_gripper"] = "None"       # Assigning the part color's initial
        print("---Placed Red part with left arm")   # Printing the action

    if part == "G" and arm == "left":
        tray_status["G"] = tray_status["G"] + 1
        robot_status["is_empty_left"] = True
        robot_status["left_gripper"] = "None"
        print("---Placed Green part with left arm")

    if part == "B" and arm == "left":
        tray_status["B"] = tray_status["B"] + 1
        robot_status["is_empty_left"] = True
        robot_status["left_gripper"] = "None"
        print("---Placed Blue part with left arm")

    if part == "R" and arm == "right":
        tray_status["R"] = tray_status["R"] + 1
        robot_status["is_empty_right"] = True
        robot_status["right_gripper"] = "None"
        print("---Placed Red part with right arm")

    if part == "G" and arm == "right":
        tray_status["G"] = tray_status["G"] + 1
        robot_status["is_empty_right"] = True
        robot_status["right_gripper"] = "None"
        print("---Placed Green part with right arm")

    if part == "B" and arm == "right":
        tray_status["B"] = tray_status["B"] + 1
        robot_status["is_empty_right"] = True
        robot_status["right_gripper"] = "None"
        print("---Placed Blue part with right arm")
        return


def move_to_bin():
    """
    This function will move robot to the bin
    :return: Change variables accordingly
    """
    robot_status["at_bin"] = True
    robot_status["at_home"] = False
    robot_status["at_tray"] = False
    print("---Moving to bin")
    return


def move_to_tray():
    """
    This function will move robot to the tray
    :return: Change variables accordingly
    """
    robot_status["at_bin"] = False
    robot_status["at_home"] = False
    robot_status["at_tray"] = True
    print("---Moving to Tray")
    return


def compute():
    """
    The function that calls all the previous function
    :return: Print the plan
    """
    initiate()                                              # Initiate variables
    get_user_data()                                         # Getting user data
    if possible_to_compute:                                 # Checking condition if it is possible to compute

        # Completing kitting of Red part
        while tray_status["R"] != goal["R"]:                # While the goal is not reached continue
            print("====" * 12)                              # Printing pattern
            move_to_bin()                                   # First move to the bin
            if (goal["R"] - tray_status["R"]) > 1:     # If there are two more parts required in tray
                pick("left", "R")                           # Use both hands to have Red parts
                pick("right", "R")
            elif (goal["R"] - tray_status["R"]) == 1:   # Else use only left hand
                pick("left", "R")
            move_to_tray()                                  # Move to tray
            if not robot_status["is_empty_left"]:           # Empty left hand if not empty
                place("left", "R")                          # Calling place function for that
            if not robot_status["is_empty_right"]:          # Empty right hand if not empty
                place("right", "R")                         # Calling place function for that

        # Completing kitting of Green part
        while tray_status["G"] != goal["G"]:
            print("====" * 12)
            move_to_bin()
            if (goal["G"] - tray_status["G"]) > 1:
                pick("left", "G")
                pick("right", "G")
            elif (goal["G"] - tray_status["G"]) == 1:
                pick("left", "G")
            move_to_tray()
            if not robot_status["is_empty_left"]:
                place("left", "G")
            if not robot_status["is_empty_right"]:
                place("right", "G")

        # Completing kitting for Blue part
        while tray_status["B"] != goal["B"]:
            print("====" * 12)
            move_to_bin()
            if (goal["B"] - tray_status["B"]) > 1:
                pick("left", "B")
                pick("right", "B")
            elif (goal["B"] - tray_status["B"]) == 1:
                pick("left", "B")
            move_to_tray()
            if not robot_status["is_empty_left"]:
                place("left", "B")
            if not robot_status["is_empty_right"]:
                place("right", "B")
    else:                                                   # If it is not possible to compute
        print("Exiting...")                                 # Exit the code

    print("====" * 12)
    print("Summery:")
    r = goal["R"]
    r_l = bin_status["R"]
    print(f"The kit tray has {r} red part(s) -- The bin has {r_l} red part(s) left")
    g = goal["G"]
    g_l = bin_status["G"]
    print(f"The kit tray has {g} red part(s) -- The bin has {g_l} red part(s) left")
    b = goal["B"]
    b_l = bin_status["B"]
    print(f"The kit tray has {b} red part(s) -- The bin has {b_l} red part(s) left")

# Calling the final function
compute()
