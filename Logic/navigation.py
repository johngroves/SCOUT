# ------------------------------------------------------------------------------
# Control Arduino With Python Via Serial
# ------------------------------------------------------------------------------

from math import radians, cos, sin, asin, sqrt, atan2, degrees
import PyCmdMessenger
import geo
import geomag


def connect():

    arduino = PyCmdMessenger.ArduinoBoard("/dev/cu.usbmodem1421",baud_rate=115200, timeout=10)
    commands = [["get_telemetry_data",""],
                ["telemetry_data","ffcff"],
                ["turn_to","fc"],
                ["new_rudder_position","fc"],
                ["error","s"]]
    c = PyCmdMessenger.CmdMessenger(arduino,commands)

def navigate ():

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

def turn_to(angle,side):
    # ex (34.30,'p')
    c.send("turn_to", angle, side)
    msg = c.receive()
    msg_data = msg[1]
    data = {
        "rudder_angle": msg_data[0],
        "rudder_side": msg_data[1]
    }
    return data


def cartesian_average (coords):

    cart_x, cart_y, cart_z = []

    for lat,lon in coords:
        X = cos(lat) * cos(lon)
        Y = cos(lat) * sin(lon)
        Z = sin(lat)

        cart_x.append(X)
        cart_y.append(Y)
        cart_z.append(Z)

    avg_x = sum(cartX)/len(cartX)
    avg_y = sum(cartY)/len(cartY)
    avg_z = sum(cartZ)/len(cartZ)

    hyp = math.sqrt(avg_x * avg_x + avg_y * avg_y)
    avg_lon = atan2(avg_y, avg_x)
    avg_lat = atan2(avg_z, hyp)

    return avg_lat, avg_lon


def mean_heading (headings):
    """
    Calculate the average heading from a list of headings
    :param headings:
    :return: average heading
    """
    vectors = [cmath.rect(1, angle) for angle in list_of_angles]
    vector_sum = sum(vectors)

    return cmath.phase(vector_sum)


def calc_distance(lat1, lon1, lat2, lon2):
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
    r = 3956 # Radius of earth in kilometers. Use 3956 for miles

    return c * r

def calc_angle(lat1, lon1, lat2, lon2):
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
    r = 3956 # Radius of earth in kilometers. Use 3956 for miles

    return a

def get_bearing(lat1, lon1, lat2, lon2):

    waypoint_1 = geo.xyz(lat1, lon1)
    waypoint_2 = geo.xyz(lat2, lon2)

    true_north = geo.great_circle_angle(waypoint_1, waypoint_2, geo.geographic_northpole)
    bearing = geomag.mag_heading(true_north, dlat=lat1, dlon=lon1)

    return bearing

print (get_bearing(48.137222,11.575556,52.518611,13.408056))
