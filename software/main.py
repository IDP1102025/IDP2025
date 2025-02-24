from robot import Robot
from time import sleep


def test_start(wilsonbot):
    """
    Begins the test, starts the robot on tanby and awaits a button press
    """
    wilsonbot.robot_standby()  # Start the robot on standby mode, will start when button is pressed


def test_navigation(wilsonbot):
    """
    Sets the robot's current node to the start node and goes to the first depot
    """
    wilsonbot.current_node = wilsonbot.navigation.graph.get_node("Start Node")

    # Go to the first depot
    wilsonbot.goto_node(wilsonbot.navigation.graph.get_node("Depot 1"))


if __name__ == "__main__":
    # Init robot
    wilsonbot = Robot()
    test_start(wilsonbot)
