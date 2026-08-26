import math


EARTH_RADIUS_M = 6_371_000.0


def distance_between_coordinates(
    lat1_deg: float,
    lon1_deg: float,
    lat2_deg: float,
    lon2_deg: float,
) -> float:
    """
    Calculate the surface distance between two latitude/longitude
    coordinates using the Haversine formula.

    Returns:
        Distance in meters.
    """
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    a = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat1)
        * math.cos(lat2)
        * math.sin(delta_lon / 2.0) ** 2
    )

    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    return EARTH_RADIUS_M * c


def angle_between_coordinates(
    lat1_deg: float,
    lon1_deg: float,
    lat2_deg: float,
    lon2_deg: float,
) -> float:
    """
    Calculate the bearing from the first coordinate to the second.

    Uses a locally flat approximation suitable for nearby coordinates. The
    returned angle is in degrees in the range [0, 360), measured clockwise
    from north (north=0, east=90, south=180, west=270).
    """
    mean_lat_rad = math.radians((lat1_deg + lat2_deg) / 2.0)
    delta_lat = lat2_deg - lat1_deg
    delta_lon = (lon2_deg - lon1_deg + 180.0) % 360.0 - 180.0

    north = delta_lat
    east = delta_lon * math.cos(mean_lat_rad)

    return math.degrees(math.atan2(east, north)) % 360.0


def wrap_angle_deg(angle:float) -> float:
    return ((angle + 180) % 360) - 180

def goal_is_reached(
    current_lat: float,
    current_lon: float,
    goal_lat: float,
    goal_lon: float,
    tolerance_m: float,
) -> bool:
    distance_m = distance_between_coordinates(
        current_lat,
        current_lon,
        goal_lat,
        goal_lon,
    )

    return distance_m <= tolerance_m

def clamp(val, minimum, maximum):
    return max(minimum, min(val, maximum))

def int_map(val, in_min, in_max, out_min, out_max) -> int:
    return int(out_min + (out_max - out_min) / (in_max - in_min) * (val - in_min))

def map(val, in_min, in_max, out_min, out_max):
    return out_min + (out_max - out_min) / (in_max - in_min) * (val - in_min)
