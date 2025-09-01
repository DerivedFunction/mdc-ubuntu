import pygame
import time
import sys

class Controller:
    def __init__(self):
        # Initialize pygame
        pygame.init()


        # Check if any joysticks are connected
        if pygame.joystick.get_count() == 0:
            print("Error: No joystick connected. Please connect a controller and try again.")
            sys.exit(1)  # Exit the program with a status code of 1 (indicating an error)

        # Get the first joystick
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def read_input(self):
        """
        Reads the current state of the controller's axes and buttons.

        Returns:
            list: A list containing the axis values followed by button values.
        """
        # Handle pygame events
        pygame.event.pump()

        # Read axis values
        axis_values = [round(self.joystick.get_axis(i), 2) for i in range(self.joystick.get_numaxes())]

        # Read button values
        button_values = [round(self.joystick.get_button(i), 2) for i in range(self.joystick.get_numbuttons())]

        return (axis_values, button_values)

    def close(self):
        """Cleans up the controller and pygame resources."""
        self.joystick.quit()
        pygame.quit()

def main():
    # Create a Controller instance

    try:
        controller = Controller()
        print("Controller initialized. Reading input. Press Ctrl+C to stop.")

        # Array to hold the controller input data

        # Read controller input in a loop
        while True:
            (axis, buttons) = controller.read_input()

            # Print the current input for demonstration
            print("Axis Values:", axis, "\tButton Values:", buttons)

            # Wait for a short period before the next reading
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping controller input reading.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up resources
        if 'controller' in locals():
            controller.close()

if __name__ == "__main__":
    main()
