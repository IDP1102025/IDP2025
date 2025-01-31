from robot import Robot
from time import time


def test_start(wilsonbot):
    # Test the robot to go to the start node
    # Start the robot on standby mode
    # Will start when button is pressed
    wilsonbot.robot_standby()

    # robot should be at the start node

def test_line_following(wilsonbot):

    # Test line following by moving along the line for 2 junctions
    wilsonbot.move(2, 30)

def test_turning(wilsonbot):
    # Test 90 degree turn
    wilsonbot.face_direction(2)

    # Test 180 degree turn
    wilsonbot.face_direction(4)

    # test 90 degree turn
    wilsonbot.face_direction(3)

    # Go back to start
    wilsonbot.face_direction(1)

def test_navigation(wilsonbot):
    # Go to the first depot
    wilsonbot.goto_node(wilsonbot.navigation.graph.get_node("Depot 1"))

    # Go to the 2nd depot
    wilsonbot.goto_node(wilsonbot.navigation.graph.get_node("Depot 2"))
    
    # Return to the start node
    wilsonbot.return_to_start()

if __name__ == "__main__":
    print("Starting...")

    # Init robot 
    wilsonbot = Robot()

    t1 = time()
    test_start(wilsonbot)

    test_line_following(wilsonbot)
    #test_turning(wilsonbot)
    #test_navigation(wilsonbot)
    print(f"test completed in {t1 - time()}")