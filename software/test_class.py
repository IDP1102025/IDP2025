from robot import Robot
from time import sleep
def test_start(wilsonbot):
    # Test the robot to go to the start node
    # Start the robot on standby mode
    # Will start when button is pressed
    wilsonbot.robot_standby()

    # robot should be at the start node

def test_line_following(wilsonbot):

# Test line following by moving along the line for 2 junctions
    print(wilsonbot.direction_facing)
    wilsonbot.move(1)
    wilsonbot.face_direction(2)
    wilsonbot.move(1)
    wilsonbot.face_direction(3)
    wilsonbot.move(2)
    wilsonbot.face_direction(2)
    wilsonbot.move(2)
    wilsonbot.face_direction(4)
    
    
def test_navigation(wilsonbot):
    print("STARTING")
    
    wilsonbot.current_node = wilsonbot.navigation.graph.get_node("Start Node")

    # Go to the first depot
    wilsonbot.goto_node(wilsonbot.navigation.graph.get_node("Depot 1"))

    # Return to the start node
    wilsonbot.return_to_start()

def test_LED(wilsonbot):
    print("light on")
    wilsonbot.led.value(1)
    sleep(3)
    wilsonbot.led.value(0)
    print("light off")

if __name__ == "__main__":
    # Init robot
    wilsonbot = Robot()
    test_start(wilsonbot)
#     wilsonbot.move(1)
    
    test_navigation(wilsonbot)
    wilsonbot.dual_motors.stop()
    print("test complete")




