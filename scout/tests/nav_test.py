from .context import *

# Waypoints
# HQ Dock: (37.528414, -122.258465)
# Off Equinix (37.5273881,-122.260400)
# Back Channel Top - North (37.5259012,-122.2604400)
# Back Channel Bottom - North (37.523426,-122.2627522)
# Back Channel Bottom - South (37.523426,-122.2624622)
# Lagoon Diamond Center (37.527019, -122.25898)
# AJ Dock: (37.527475, -122.256993)



print(navigation.center_geolocation([(37.527848, -122.256481),(37.527848, -122.256481)]))
print(navigation.mean_heading([0.0,0.0,0.0,350.3]))
print(navigation.distance((37.5287152,-122.2585541),(37.528414, -122.258465)))
print(navigation.get_bearing(37.527475, -122.256993,37.528414, -122.258465))