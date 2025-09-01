import pygame

class Keyboard:
    def __init__(self):
        """Initialize the Keyboard class."""
        pygame.init()
        # Create a hidden window to capture keyboard events
        pygame.display.set_mode((100, 100))  # Minimal window size

        print("Keyboard initialized. Press 'esc' to exit.")

    def read_input(self):
        """
        Reads the current state of the keyboard.

        Returns:
            tuple: Two lists, one for axis-like values (e.g., WASD or arrow keys)
                and one for button-like values (e.g., space bar or other keys).
        """
        # Handle pygame events
        pygame.event.pump()

        # Define key mappings for axis values (e.g., WASD or arrow keys)
        keys = pygame.key.get_pressed()

        return keys

    def close(self):
        """Cleans up any resources, if needed."""
        pygame.quit()
        print("Keyboard listener closed.")

def main():
    # Create an instance of the Keyboard class
    keyboard_listener = Keyboard()

    # Continuously read keyboard input and print the values
    while True:
        keys = keyboard_listener.read_input()

        # Get the states of the arrow keys
        up = keys[pygame.K_UP]
        down = keys[pygame.K_DOWN]
        left = keys[pygame.K_LEFT]
        right = keys[pygame.K_RIGHT]

        # Print the states of the arrow keys
        print(f"Up: {up}, Down: {down}, Left: {left}, Right: {right}")

        # Check if the escape key is pressed to exit the loop
        if keys[pygame.K_ESCAPE]:
            print("Escape key pressed. Exiting...")
            break

        # Optional: Add a small delay to reduce CPU usage
        pygame.time.delay(100)

    # Clean up resources after listening stops
    keyboard_listener.close()

if __name__ == "__main__":
    main()
