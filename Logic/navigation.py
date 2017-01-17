# ------------------------------------------------------------------------------
# Control Arduino With Python Via Serial
# ------------------------------------------------------------------------------

from math import radians, cos, sin, asin, sqrt, atan2, degrees
from collections import deque
from pid import PID
import cmath, time
import PyCmdMessenger, geo, geomag

global c

waypoints = [(37.526395, -122.258265)]

pid = PID(1.0, 0.0, 0.0, 0, 1)

def setup():
    global c
    serial_port = "/dev/cu.usbmodem1411"
    arduino = PyCmdMessenger.ArduinoBoard(serial_port, baud_rate=115200, timeout=10)
    commands = [["get_telemetry_data",""],
                ["telemetry_data","ffcff"],
                ["turn_to","fc"],
                ["new_rudder_positiaon","fc"],
                ["error","s"]]

    c = PyCmdMessenger.CmdMessenger(arduino,commands)
    ready = startup()
    if ready is False:
        while ready is False:
            print("Waiting for GPS Connection")
            time.sleep(30)
            ready = startup()
    else:
        return


def startup():
    status = get_telemetry()
    if (int(status['latitude']) != 0 ):
        waypoints.append((status['latitude'], status['latitude']))
        return True
    else:
        return False


def navigate ():
    """
    Main loop handling navigation.
    """
    print ("Startup done. Navigating.")
    position_history = deque([], maxlen=10)
    heading_history = deque([], maxlen=10)

    while True:

        # Get latest telemetry data
        tel_data = get_telemetry()

        print (tel_data)

        heading = tel_data['boat_heading']
        rudder_angle = tel_data['rudder_angle']
        rudder_side = tel_data['rudder_side']
        latitude = tel_data['latitude']
        longitude = tel_data['longitude']
        location_tuple = (latitude,longitude)

        # Rolling 10 locations, headings
        position_history.append(location_tuple)
        heading_history.append(heading)

        # Gather center location, heading
        avg_location = cartesian_average(position_history)
        avg_heading = mean_heading(heading_history)

        # Calculate bearing to next waypoint
        next_waypoint = waypoints[-1]
        next_lat, next_lon = next_waypoint
        lat1, lon1 = avg_location
        bearing = get_bearing(lat1, lon1, next_lat, next_lon)
        error = degrees_between(avg_heading,bearing)

        # PID
        new, output = pid.compute(error)

        # Turn
        if new:
            angle, side = scale(output)
            new_angle = turn_to(angle, side)


def cartesian_average (coords):
    """
    Finds the cartesian center of a list of coordinate tuples
    :param a list of coordinate tuples
    :return: center of coordinates
    """
    cart_x = cart_y = cart_z = []

    for lat,lon in coords:
        X = cos(lat) * cos(lon)
        Y = cos(lat) * sin(lon)
        Z = sin(lat)

        cart_x.append(X)
        cart_y.append(Y)
        cart_z.append(Z)

    avg_x = sum(cart_x)/len(cart_x)
    avg_y = sum(cart_y)/len(cart_y)
    avg_z = sum(cart_z)/len(cart_z)

    hyp = sqrt(avg_x * avg_x + avg_y * avg_y)
    avg_lon = atan2(avg_y, avg_x)
    avg_lat = atan2(avg_z, hyp)

    return (avg_lat, avg_lon)


def mean_heading(headings):
    """
    Calculate the average heading from a list of headings
    :param headings:
    :return: average heading
    """
    vectors = [cmath.rect(1, angle) for angle in headings]
    vector_sum = sum(vectors)

    return cmath.phase(vector_sum)


def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 3956 # Radius of earth in miles

    return c * r


def get_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing between two coordinates
    :param a pair of lat,lon coordinates:
    :return: bearing in degrees
    """
    waypoint_1 = geo.xyz(lat1, lon1)
    waypoint_2 = geo.xyz(lat2, lon2)
    true_north = geo.great_circle_angle(waypoint_2, waypoint_1, geo.geographic_northpole)
    bearing = geomag.mag_heading(true_north, dlat=lat1, dlon=lon1)

    return bearing


def turn_to(angle,side):
    """
    Calculate the average heading from a list of headings
    :param headings:
    :return: average heading
    """
    c.send("turn_to", angle, side)
    msg = c.receive()
    msg_data = msg[1]
    data = {
        "rudder_angle": msg_data[0],
        "rudder_side": msg_data[1]
    }

    return data


def degrees_between(angle_1, angle_2):
    """
    Calculates the smallest angle between two headings
    :param angle_1: float
    :param angle_2: float
    :return: float
    """
    difference = ((angle_2 - angle_1) + 180) % 360 - 180

    return difference


def scale(output):
    if output >= 0:
        side = 'p'
    else:
        side = 's'
    return abs(output),side


def get_telemetry():
    """
    Sends command to micro controller to get latest  telemetry data
    Accepts no parameters, returns dict:
        {
            boat_heading: degrees (float) - angle of boat heading
            rudder_angle: degrees (float) - angle of boat rudder
            rudder_side: enum: p (port) or s (starboard)
            latitude: degrees - boat center latitude
            longitude: degrees - boat center longitude
        }
    """
    c.send("get_telemetry_data")
    msg = c.receive()
    print (msg)
    try:
        msg_data = msg[1]
        data = {
            "boat_heading": msg_data[0],
            "rudder_angle": msg_data[1],
            "rudder_side": msg_data[2],
            "latitude": msg_data[3],
            "longitude": msg_data[4]
        }
    except TypeError:
        data = {
            "boat_heading": 0,
            "rudder_angle": 0,
            "rudder_side": 0,
            "latitude": 0,
            "longitude": 0
        }
    return data

if __name__ == "__main__":
    setup()
    navigate()
