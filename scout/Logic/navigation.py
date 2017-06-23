# ------------------------------------------------------------------------------
# Control Arduino With Python Via Serial
# ------------------------------------------------------------------------------
import asyncio
from math import radians, cos, sin, asin, sqrt, atan2, degrees
from collections import deque
from .pid import PID
from .geo import *
import cmath, time
import PyCmdMessenger, geomag

global c

waypoints = [
    (37.527475, -122.256993),
    (37.528414, -122.258465),   # HQ DOCK
    (37.5273881,-122.260400),   # Off Equinix
    (37.5259012,-122.2604400),  # Back Channel Top North
    (37.523426,-122.2627522),   # Back Channel Bottom Center
    (37.527019, -122.25898),    # Lagoon Center
    (37.527475, -122.256993)    # AJ Dock
]



pid = PID(0.2, 0.0, 0.1, 0, 1)

def setup():
    global c
    serial_port = "/dev/cu.usbmodem1421"
    arduino = PyCmdMessenger.ArduinoBoard(serial_port, baud_rate=115200, timeout=20)
    commands = [["get_telemetry_data",""],
                ["telemetry_data","ffcff"],
                ["turn_to","fc"],
                ["new_rudder_position","fc"],
                ["error","s"]]

    c = PyCmdMessenger.CmdMessenger(arduino,commands)
    print(turn_test())
    ready = startup()
    if ready is False:
        while ready is False:
            print("Waiting for GPS Connection")
            time.sleep(15)
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


def turn_test():
    return "Rudder calibration completed."


def navigate ():
    """
    Main loop handling navigation.
    """
    print ("Startup done. Navigating.")
    position_history = deque([], maxlen=10)
    heading_history = deque([], maxlen=10)
    next_waypoint = waypoints.pop(0)
    CLOSENESS = 30
    use_avg = False

    while True:

        # Get latest telemetry data
        tel_data = get_telemetry()

        print(tel_data)

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
        avg_location = center_geolocation(position_history)

        # Calculate bearing to next waypoint
        distance_m = distance(location_tuple,next_waypoint)
        if distance_m < CLOSENESS:
            print("Reached Waypoint: ", next_waypoint)
            next_waypoint = waypoints.pop(0)
        next_lat, next_lon = next_waypoint
        if use_avg:
            avg_heading = mean_heading(heading_history)
            lat1, lon1 = avg_location
        else:
            avg_heading = heading
            lat1, lon1 = location_tuple
        bearing = get_bearing(lat1, lon1, next_lat, next_lon)
        error = degrees_between(avg_heading,bearing)
        print("Heading: ", avg_heading)
        print("Bearing: ", bearing)
        print("Error: ", error)
        print("Distance", distance_m)
        print("Next WP:", next_waypoint)
        # PID
        output, new = pid.compute(error)

        # Turn
        if new:
            angle, side = scale(output)
            turn_to(angle, side)

        time.sleep(.1) # Can't remember why this made sense?


def center_geolocation(geolocations):
    """
    Provide a relatively accurate center lat, lon returned as a list pair, given
    a list of list pairs.
    ex: in: geolocations = ((lat1,lon1), (lat2,lon2),)
        out: (center_lat, center_lon)
    """
    x = 0
    y = 0
    z = 0

    for lat, lon in geolocations:
        lat = float(radians(lat))
        lon = float(radians(lon))
        x += cos(lat) * cos(lon)
        y += cos(lat) * sin(lon)
        z += sin(lat)

    x = float(x / len(geolocations))
    y = float(y / len(geolocations))
    z = float(z / len(geolocations))

    c_lat = degrees(atan2(z, sqrt(x * x + y * y)))
    c_lon = degrees(atan2(y, x))

    return (c_lat, c_lon)


def mean_heading(headings):
    """
    Calculate the average heading from a list of headings
    :param headings:
    :return: average heading
    """
    vectors = [cmath.rect(1, radians(angle)) for angle in headings]
    vector_sum = sum(vectors)
    return degrees(cmath.phase(vector_sum))


def distance(wp1, wp2):

    lat1, lon1, lat2, lon2 = *wp1, *wp2
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    r = 6371000  # Radius of earth in miles

    return c*r

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
    waypoint_1 = xyz(lat1, lon1)
    waypoint_2 = xyz(lat2, lon2)
    true_north = great_circle_angle(waypoint_2, waypoint_1, geographic_northpole)
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
    if output <= 0:
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
