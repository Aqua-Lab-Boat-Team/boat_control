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