import time

class GNCPlanner:
    """
    GNC Planner framework for switching between modes on the ADCS-MTQ module.
    Designed to be called by the Lower PPM flight software.
    """
    def __init__(self, adcs_driver):
        self.driver = adcs_driver
        # Tracks when a wait condition was first triggered
        self.wait_start_time = None
        self.command_step = 0
        
    def mode_interp(self, telemetry):
        """
        Interprets the status variable from the ADCS-MTQ unit and responds with the units current mode
        """
        STAT = telemetry.get('STAT', '000')
        # Extract the first 3 bits/chars
        mode_bits = str(STAT)[0:3]
        mode_map = {
            '000': 'safe', '001': 'detumble', '010': 'sunpoint',
            '011': 'finepointing', '100': 'lvlh', '101': 'targettracking',
            '110': 'sunspin', '111': 'manual'
        }
        return mode_map.get(mode_bits, 'safe')
    
    def plan_and_execute(self, telemetry):
        """
        Main entry point for Lower PPM.
        Input: telemetry dictionary (containing angular rates and modes)
        Output: error messages and mode commandeds to the ADCS as applicable.
        """

        # Logic for Mode Transitions
        omega_norm = telemetry.get('omega_norm', 0.0) #Check Variable and units
        current_mode = self.mode_interp(telemetry)
        expected_mode = telemetry.get('adcs_planner_mode', 0.0) #Check Variable
        now = time.time()
        
        # Check for unexpected mode transitions. 
        # Record error message if there has been an unexpected mode transition and set the planners mode to the new mode.
        if expected_mode != current_mode:
            error_message = f'ADCS MTQ unexpectedly transitioned from {expected_mode} mode to {current_mode} mode'
            telemetry('adcs_planner_errors').apend(error_message) # Ask Adhit how to handle error logging.
            self.wait_start_time = None 
            self.command_step = 0
            expected_mode = current_mode

        # Logic for dealing with safe mode.
        if current_mode == "safe":
            # Initial 300s soak in Safe Mode
            if self.command_step == 0:
                if self.wait_start_time is None:
                    self.wait_start_time = now
                elif now - self.wait_start_time >= 300:
                    if omega_norm < 360.0:
                        self.driver.reset()
                        self.command_step = 1
                        self.wait_start_time = now # Start timer for next step

            # Wait 5s after reset, then sync time
            elif self.command_step == 1:
                if now - self.wait_start_time >= 5:
                    self.driver.time_date()
                    self.command_step = 2
                    self.wait_start_time = now

            # Wait 5s after time sync, then upload TLE
            elif self.command_step == 2:
                if now - self.wait_start_time >= 5:
                    tle_data = telemetry.get('TLE_current', 0.0)
                    self.driver.write(17, tle_data)
                    self.command_step = 3
                    self.wait_start_time = now

            # Wait 60s for TLE processing, then command Detumble (likely overkill)
            elif self.command_step == 3:
                if now - self.wait_start_time >= 60:
                    self.driver.detumble()
                    self.command_step = 0 # Sequence complete
                    self.wait_start_time = None
                    return "detumble"

            # Reset sequencer if mode unexpectedly changes
            else:
                self.command_step = 0
                self.wait_start_time = None

        elif self.current_mode == "detumble":
            # Requirement: omega_norm < 5.0 for 300s
            if omega_norm < 5.0:
                if self.wait_start_time is None:
                    self.wait_start_time = now
                elif now - self.wait_start_time >= 300:
                    self.driver.sunpoint()
                    expected_mode = 'sunpoint'
                    self.wait_start_time = None

        elif self.current_mode == "sunpoint":

            return self.current_mode