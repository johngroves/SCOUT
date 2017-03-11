# ------------------------------------------------------------------------------
# Control Arduino With Python Via Serial
# ------------------------------------------------------------------------------


import PyCmdMessenger
arduino = PyCmdMessenger.ArduinoBoard("/dev/cu.usbmodem1411",baud_rate=115200, timeout=10)
commands = [["get_telemetry_data",""],
            ["telemetry_data","ffcff"],
            ["turn_to","fc"],
            ["new_rudder_position","fc"],
            ["error","s"]]

c = PyCmdMessenger.CmdMessenger(arduino,commands)

while True:
    c.send("get_telemetry_data")
    msg = c.receive()
    print (msg)
    msg_data = msg[1]
    data = {
        "boat_heading": msg_data[0],
        "rudder_angle": msg_data[1],
        "rudder_side": msg_data[2],
        "latitude": msg_data[3],
        "longitude": msg_data[4]
    }
    print (data)