# import requests

# class BatteryMonitor:
#     def __init__(self, ip='192.168.0.172', auth_token='YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='):
#         self.host = f'http://{ip}/api/v2.0.0/'
#         self.headers = {
#             'Content-Type': 'application/json',
#             'Authorization': f'Basic {auth_token}'
#         }
#         self.timeout = 2

#     def update_battery_status(self):
#         try:
#             response = requests.get(self.host + 'status', headers=self.headers, timeout=self.timeout)
#             response.raise_for_status()
#             data = response.json()
#             battery_level = data.get("battery_percentage", "--")
#             if isinstance(battery_level, float):
#                 battery_level = round(battery_level)
#             return f"{battery_level}%"
#         except requests.exceptions.RequestException:
#             return "--%"
#         except Exception:
#             return "--%"
import rospy

class BatteryMonitor:
    def __init__(self, initial_battery_level=15):
        self.battery_level = initial_battery_level
        rospy.loginfo(f"BatteryMonitor initialized. Simulated battery level: {self.battery_level}%")

    def update_battery_status(self):
        rospy.loginfo(f"Reporting battery status: {self.battery_level}%")
        return f"{self.battery_level}%" # Returns string like "15%"

    def get_battery_level_numeric(self):
        rospy.logdebug(f"Getting numeric battery level: {self.battery_level}")
        return float(self.battery_level)

    def start_charging(self):
        rospy.loginfo("Charging command received (simulated).")
        self.battery_level = 100 
        rospy.loginfo(f"Battery is now charging (simulated). Level set to {self.battery_level}%.")
        return True