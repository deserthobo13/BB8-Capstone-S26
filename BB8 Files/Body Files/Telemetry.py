import csv
import time

class TelemetryLogger:
    def __init__(self, filename="pid_telemetry.csv"):
        self.filename = filename
        self.data_buffer = []
        self.headers = ["timestamp", "pitch", "setpoint", "error", "P", "I", "D", "output"]
        self.start_time = time.time()
        
    def log_step(self, pitch, setpoint, pid_controller):
        """
        Appends data to RAM. This takes microseconds and will not stall your loop.
        """
        current_time = time.time() - self.start_time
        self.data_buffer.append([
            round(current_time, 4),
            round(pitch, 2),
            round(setpoint, 2),
            round(pid_controller.error, 4),
            round(pid_controller.p_term, 4),
            round(pid_controller.i_term, 4),
            round(pid_controller.d_term, 4),
            round(pid_controller.output, 4)
        ])

    def save_to_csv(self):
        """
        Call this when the robot stops or the test ends to dump RAM to disk.
        """
        if not self.data_buffer:
            print("No telemetry data to save.")
            return
            
        print(f"Saving {len(self.data_buffer)} data points to {self.filename}...")
        try:
            with open(self.filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(self.headers)
                writer.writerows(self.data_buffer)
            print("Telemetry saved successfully.")
            self.data_buffer.clear() # Clear memory after saving
        except Exception as e:
            print(f"Failed to save telemetry: {e}")