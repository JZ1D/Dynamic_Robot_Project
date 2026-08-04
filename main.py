'''
import time
try:
    from neurapy.robot import Robot
    REAL_HARDWARE = True
except ImportError:
    REAL_HARDWARE = False


class MockRobot:
    def __init__(self):
        print("[MOCK] Initialisiere virtuelles Roboter-Objekt.")
    def init_program(self):
        print("[MOCK] init_program() aufgerufen.")
    def power_on(self):
        print("[MOCK] power_on() aufgerufen.")
    def is_robot_in_teach_mode(self):
        return True
    def switch_to_automatic_mode(self):
        print("[MOCK] In Automatic-Mode gewechselt.")
    def set_override(self, val):
        print(f"[MOCK] Geschwindigkeits-Override gesetzt auf: {val * 100}%")
    def get_current_joint_angles(self):
        return [0.0, -0.26, 1.57, 0.0, 0.78, 0.0]
    def move_joint(self, **kwargs):
        print(f"[MOCK] Joint-Bewegung -> Ziel: {kwargs.get('target_joint')}")
        time.sleep(1.0)
    def move_linear(self, **kwargs):
        print(f"[MOCK] Linear-Bewegung -> Ziel: {kwargs.get('target_pose')}")
        time.sleep(1.0)
    def release(self):
        print("[MOCK] Greifer GEÖFFNET (release)")
        time.sleep(0.5)
    def grasp(self):
        print("[MOCK] Greifer GESCHLOSSEN (grasp)")
        time.sleep(0.5)
    def stop(self):
        print("[MOCK] stop() aufgerufen -> Skript beendet.")

def main():
    print(" NEURA LARA - GRUNDGERÜST FUNKTIONSTEST ")

    # 1. Verbindung herstellen
    if REAL_HARDWARE:
        try:
            print("Verbinde mit Neura Lara...")
            r = Robot()
            print("Verbindung erfolgreich!")
        except Exception as e:
            print(f"Hardware nicht erreichbar ({e}). Starte Mock-Modus...")
            r = MockRobot()
    else:
        r = MockRobot()

    # 2. Pflicht-Initialisierung laut NeuraPy-Handbuch
    # WICHTIG: Ohne init_program() werden Bewegungen blockiert!
    r.init_program()
    r.power_on()

    # Mode-Check: Falls der Roboter im Teach-Modus steht, in Automatic schalten
    if hasattr(r, 'is_robot_in_teach_mode') and r.is_robot_in_teach_mode():
        r.switch_to_automatic_mode()

    # 3. SICHERHEITSEINSTELLUNG
    # Setze die Robotergeschwindigkeit auf 20% für den Ersttest
    r.set_override(0.2)

    # ------------------------------------------------------------------
    # TEST 1: GREIFER (Grasp / Release)
    # ------------------------------------------------------------------
    print("\n--- [TEST 1] Greifer-Funktionen ---")
    print("1a. Öffne Greifer...")
    r.release()
    time.sleep(1.0)

    print("1b. Schließe Greifer...")
    r.grasp()
    time.sleep(1.0)

    print("1c. Öffne Greifer wieder...")
    r.release()
    time.sleep(1.0)

    # ------------------------------------------------------------------
    # TEST 2: JOINT-BEWEGUNG (Gelenkwinkel in Radian)
    # ------------------------------------------------------------------
    print("\n--- [TEST 2] Joint-Bewegung (Achsraum) ---")
    # Beispiel-Gelenkwinkel für 6 Achsen [Achse 1..6] in Radian
    target_joints = [0.0, -0.26, 1.57, 0.0, 0.78, 0.0]

    joint_props = {
        "speed": 25.0,          # Geschwindigkeit in %
        "acceleration": 20.0,   # Beschleunigung in %
        "target_joint": [target_joints],
        "current_joint_angles": r.get_current_joint_angles()
    }
    r.move_joint(**joint_props)

    # ------------------------------------------------------------------
    # TEST 3: LINEAR-BEWEGUNG (Kartesische Koordinaten)
    # ------------------------------------------------------------------
    print("\n--- [TEST 3] Linear-Bewegung (Kartesisch) ---")
    # Pose: [X, Y, Z in Metern | Rx, Ry, Rz in Radian]
    target_cartesian = [0.35, 0.15, 0.25, 3.14, 0.0, 0.0]

    linear_props = {
        "speed": 0.15,          # Lineare Geschwindigkeit in m/s
        "acceleration": 0.10,   # Lineare Beschleunigung in m/s^2
        "target_pose": [target_cartesian],
        "current_joint_angles": r.get_current_joint_angles()  # Für v5.0.8 zwingend erforderlich
    }
    r.move_linear(**linear_props)

    # ------------------------------------------------------------------
    # TEST 4: PROGRAMM SAUBER BEENDEN
    # ------------------------------------------------------------------
    print("\n--- [TEST 4] Programm-Abschluss ---")
    r.stop()
    print(" TEST ERFOLGREICH BEENDET ")

if __name__ == "__main__":
    main()
    '''