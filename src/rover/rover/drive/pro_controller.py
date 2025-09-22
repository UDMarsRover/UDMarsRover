import pygame
import os
os.environ["SDL_VIDEODRIVER"] = "dummy"

class NintendoProController:
    def __init__(self):
        self.name = "Nintendo Pro Controller"
        self.buttons = {
            "A": 0,
            "B": 1,
            "X": 2,
            "Y": 3,
            "minus": 4,
            "home": 5,
            "plus": 6,
            "LS": 7,
            "RS": 8,
            "L": 9,
            "R": 10,
            "D_up": 11,
            "D_down": 12,
            "D_left": 13,
            "D_right": 14,
            "circle": 15
        }
        self.analogs = {
            "LS_x": 0,
            "LS_y": 1,
            "RS_x": 2,
            "RS_y": 3,
            "ZL": 4,
            "ZR": 5
        }
        self.callbacks = {}
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick detected. Please connect your Nintendo Pro Controller via Bluetooth.")
            exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def add_button_callback(self, button_name, callback):
        """
        Add a callback function for a specific button press.
        :param button_name: Name of the button to listen for.
        :param callback: Function to call when the button is pressed.
        """
        if button_name in self.buttons:
            self.callbacks[self.buttons[button_name]] = callback
        else:
            raise ValueError(f"Button {button_name} not found in controller.")
        
    def add_analog_callback(self, stick_name, callback):
        """
        Add a callback function for an analog stick movement.
        :param stick_name: Name of the analog stick to listen for.
        :param callback: Function to call when the stick is moved.
        """
        if stick_name in self.analogs:
            self.callbacks[self.analogs[stick_name]] = callback
        else:
            raise ValueError(f"Analog stick {stick_name} not found in controller.")
        
    def run_callbacks(self):
        """
        Poll the joystick and run callbacks for pressed buttons.
        """
        pygame.event.pump()
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        for button_index in range(len(buttons)):
            if buttons[button_index] and button_index in self.callbacks:
                self.callbacks[button_index](buttons[button_index])

        for stick_index in range(len(axes)):
            if stick_index in self.callbacks:
                self.callbacks[stick_index](axes[stick_index])

    def run(self):
        """
        Main loop to keep the controller running and checking for inputs.
        """
        try:
            while True:
                self.run_callbacks()
                pygame.time.wait(50)  # Poll every 50ms
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            pygame.quit()

    def is_pressed(self, value):
        return value > 0.5

if __name__ == "__main__":
    controller = NintendoProController()
    
    # Example callback functions
    def on_a_pressed(value):
        if controller.is_pressed(value):
            print("A button pressed!")

    def zl_value(value):
        print(f"ZL value: {value}")

    # Register callbacks
    # controller.add_button_callback("A", on_a_pressed)
    controller.add_analog_callback("LS_x", zl_value)
    # controller.add_analog_callback("LS_x", on_ls_moved)

    # Run the controller loop
    controller.run()