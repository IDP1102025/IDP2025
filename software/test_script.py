from robot import Robot


def test_script():
    # Test the robot by going to depot 1 then returning to start 
    # Initialise robot object
     
    wilsonbot = Robot()
    
    # Start the robot on standby mode
    # Will start when button is pressed
    wilsonbot.robot_standby()

    print("STARTING")

    # Go to the first depot
    wilsonbot.goto_node(wilsonbot.navigation.graph.get_node("Depot 1"))

    # Go to the 2nd depot
    wilsonbot.goto_node(wilsonbot.navigation.graph.get_node("Depot 2"))
    # Return to the start node
    wilsonbot.return_to_start()

    # Print out robot variables for debugging


if __name__ == "__main__":
    test_script()
    print("test complete")