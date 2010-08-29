from math import sin, cos, atan2, degrees, pi

def gps_coords_to_heading(lat1, lon1, lat2, lon2):
    # lat = lat2-lat1
    lon = lon2 - lon1
    targetHeading = degrees(atan2(sin(lon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon)))
    return (targetHeading + 360) % 360

def heading(lat1,lon1,lat2,lon2):
    #calculated intial course to second waypoint
    heading = atan2(cos(lat1*(pi/180))*sin(lat2*(pi/180))-sin(lat1*(pi/180))*cos(lat2*(pi/180))*cos((lon2-lon1)*(pi/180)), sin((lon2-lon1)*(pi/180))*cos(lat2*(pi/180)))
    heading = -(heading*(180/pi)-90)
    if heading <0:
        heading = heading +360
    if heading >180:
        heading = heading -360
    heading = heading*(pi/180)
    return heading