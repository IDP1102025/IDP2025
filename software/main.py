# main script
# initialise nodes and graphs

from robot import Robot


def main():
    # create robot object
    wilson = Robot()

    start_status = wilson.robot_standby()
    wilson.robot_standby()

    if start_status == True:
        pass
