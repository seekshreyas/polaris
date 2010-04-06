from math import sin, cos, atan2, degrees

def gps_coords_to_heading(lat1, lon1, lat2, lon2):
    # lat = lat2-lat1
    lon = lon2 - lon1
    targetHeading = degrees(atan2(sin(lon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon)))
    return (targetHeading + 360) % 360
