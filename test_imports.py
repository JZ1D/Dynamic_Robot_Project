import neurapy
from neurapy.robot import Robot
import sys
import prettytable
import win32api

# Import der NeuraPy Version & Klasse
from neurapy.robot import VERSION

print(f"Python Version    : {sys.version.split()[0]}")
print(f"Prettytable       : {prettytable.__version__}")
print(f"NeuraPy SDK       : {VERSION}")
#Robot() und der print kann nur dann erfolgreich ausgefuehrt werden, wenn der Roboter über Ethernet verbuden ist, die 
r = Robot()
print(r.robot_name)