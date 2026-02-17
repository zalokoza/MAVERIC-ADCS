import time

class GNCPlanner:
    """
    GNC Planner framework for switching between modes on the ADCS-MTQ module.
    Designed to be called by the Lower PPM flight software.
    """
    def __init__(self, adcs_driver):
        self.driver = adcs_driver
        
    def mode_interp(self, telemetry):
        """
        Interprets the status variable from the ADCS-MTQ unit and responds with the units current mode
        """
        STAT = telemetry.get('STAT', 0.0) #Check Variable
        if STAT[0:3] == '000':
            current_mode = 'safe'
            return current_mode
        elif STAT[0:3] == '001':
            current_mode = 'detumble'
            return current_mode
        elif STAT[0:3] == '010':
            current_mode = 'sunpoint'
            return current_mode
        elif STAT[0:3] == '011':
            current_mode = 'finepointing'
            return current_mode
        elif STAT[0:3] == '100':
            current_mode = 'lvlh'
            return current_mode
        elif STAT[0:3] == '101':
            current_mode = 'targettracking'
            return current_mode
        elif STAT[0:3] == '110':
            current_mode = 'sunspin'
            return current_mode
        elif STAT[0:3] == '111':
            current_mode = 'manual'
            return current_mode
    
    def plan_and_execute(self, telemetry):
        """
        Main entry point for Lower PPM.
        Input: telemetry dictionary (containing angular rates and modes)
        Output: error messages and mode commandeds to the ADCS as applicable.
        """

        # Logic for Mode Transitions
        omega_norm = telemetry.get('omega_norm', 0.0) #Check Variable
        current_mode = self.mode_interp(telemetry)
        expected_mode = telemetry.get('adcs_planner_mode', 0.0) #Check Variable
        
        # Check for unexpected mode transitions. 
        # Record error message if there has been an unexpected mode transition and set the planners mode to the new mode.
        if expected_mode != current_mode:
            error_message = f'ADCS MTQ unexpectedly transitioned from {expected_mode} mode to {current_mode} mode'
            telemetry('adcs_planner_errors').apend(error_message) # Ask Adhit how to handle error logging.
            expected_mode = current_mode

        # Logic for dealing with safe mode.
        if self.current_mode == "safe":
            time.sleep(300)
            if self.current_mode == "safe":
                if omega_norm < 360.0:
                    self.driver.reset
                    time.sleep(5)
                    self.driver.time_date
                    time.sleep(5)
                    self.driver.write(17,telemetry.get('TLE_current', 0.0)) #Check TLE_current variable and input order for write function
                    time.sleep(60)
                    self.driver.detumble
                    expected_mode = 'detumble'

        elif self.current_mode == "detumble":
            if omega_norm < 1.0:
                time.sleep(300)
                if omega_norm < 1.0:
                    self.driver.sunpoint

        elif self.current_mode == "sunpoint":

            return self.current_mode
