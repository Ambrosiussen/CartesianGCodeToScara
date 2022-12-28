"""
Easily convert Cartesian XY style Gcode to SCARA GCode in Funky Format.
Code has been built based on original code from JavaScript app called Sandify - www.sandify.org
Please check out https://github.com/jeffeb3/sandify
"""
import math
import re
import numpy as np

# USER VARS
INPUT_NAME = r"H:\Personal\SCAHARA\heart_cartesian.gcode"
OUTPUT_NAME = r"H:\Personal\SCAHARA\square_CONVERTED.gcode"
MAX_RADIUS = 460.0
MAX_RHO = 1.0 # Needs to be between 0.0 and 1.0
UNITS_PER_CIRCLE = 360.0
LINE_SEGMENT_SIZE = 1.0
CENTER_DRAWING = True
SCARA = True
SCALE_TO_FIT = True
UNIFORM_SCALE = 1.0 
DEFAULT_DRAWING_SPEED = 1000.0

#####################################
_last_succesful_vertex = np.array([0.0,0.0])
def get_vertex_from_g01_command(command):
    """
    Creates a Vector from a G01 GCode command.
    """
    X = re.search(r"X\d+\.*\d*", command)
    X = float(X.group(0)[1:]) if X else _last_succesful_vertex[0]

    Y = re.search(r"Y\d+\.*\d*", command)
    Y = float(Y.group(0)[1:]) if Y else _last_succesful_vertex[1]

    _last_succesful_vertex[0] = X
    _last_succesful_vertex[1] = Y
    return np.array([X,Y])

def get_samples_between_vertices(p1, p2, resample_distance):
    """
    Returns a list of Vectors which are linear samples between pt1 and pt2.
    """
    pt_distance = np.linalg.norm(p1 - p2)

    if abs(pt_distance) < 0.0001 or resample_distance > pt_distance or abs(resample_distance)<0.0001:
        return [p1]
    else:
        step_x, step_y = (p2 - p1) * (resample_distance / pt_distance)
        
        if abs(step_x) < 0.0001:
            y = np.arange(p1[1], p2[1], step_y)
            x = np.zeros(len(y)) + p1[0]
        elif abs(step_y) < 0.0001:
            x = np.arange(p1[0], p2[0], step_x)
            y = np.zeros(len(x)) + p1[1]
        else:
            x = np.arange(p1[0], p2[0], step_x)
            y = np.arange(p1[1], p2[1], step_y)    

        return [np.array([x, y[i]]) for i, x in enumerate(x)]

def scara_xy_from_thetarho(theta, rho):
    """
    Convert theta rho to funky SCARA XY coordinates in cartesian space.
    """
    m1 = theta + math.acos(rho)
    m2 = theta - math.acos(rho)
    x = UNITS_PER_CIRCLE * m1 / (2.0*math.pi)
    y = UNITS_PER_CIRCLE * m2 / (2.0*math.pi)
    return x,y

# Generate Clean Original GCode Array
valid_commands = []
x_vals = []
y_vals = []
with open(INPUT_NAME, 'r', encoding="utf-8") as infile:
    for line in infile:
        line = line.strip()
        if line.startswith("G1") or line.startswith("G01"):
            valid_commands.append(line)
            vertex = get_vertex_from_g01_command(line)
            x_vals.append(vertex[0])
            y_vals.append(vertex[1])

# Computing the average position of all gcode commands.
# We do this to center the drawing for SCARA machines with the center being 0,0
# We also compute a scaling factor for scaling the drawing to fit MAX_RADIUS
centroid = np.array([sum(x_vals)/len(x_vals), sum(y_vals)/len(x_vals)]) if CENTER_DRAWING else np.array([0,0])
_maxX = max(map(lambda x: abs(x-centroid[0]),x_vals))
_maxY = max(map(lambda y: abs(y-centroid[1]),y_vals))
autofit = (MAX_RADIUS/(max(_maxX, _maxY)+1.5)) if SCALE_TO_FIT else 1.0

#TODO: Remove Houdini code
# node = hou.pwd() 
# geo = node.geometry()

# Generate Clean "Funky SCARA GCode" with resampled points between original GCode Positions.
with open(OUTPUT_NAME, 'w', encoding="utf-8") as outfile:
    # Reset Coordinates to be 0 on start file. This assumes that the end point
    # of the original Gcode ends with rho = 1.0
    outfile.write("G92 X0 Y0 Z0\n") 
    # At the top of the file write the default drawing speed.
    outfile.write("G91\n")
    outfile.write(f"G01 X0 F{DEFAULT_DRAWING_SPEED}\n")
    outfile.write("G90\n")

    _previousTheta = 0
    _previousRawTheta = 0

    for idx, line in enumerate(valid_commands):
        # Create a vertex for the next line in the original Gcode.
        elements = line.split(" ")
        vertex1 = get_vertex_from_g01_command(line)
        vertex1 -= centroid
        vertex1 *= autofit
        vertex1 *= UNIFORM_SCALE

        # If the GCode file has one more command after it, create samples from current
        # vertex to the next with the specified stepsize. This has to be done
        # because the SCARA bot is incapable of moving in straight lines.
        if idx+1 < len(valid_commands):
            vertex2 = get_vertex_from_g01_command(valid_commands[idx+1])
            vertex2 -= centroid
            vertex2 *= autofit
            vertex2 *= UNIFORM_SCALE
            vertices = get_samples_between_vertices(vertex1, vertex2, LINE_SEGMENT_SIZE)
        else:
            vertices = [vertex1]

        # We now have a new (series of) line segments to create commands for.
        for vertex in vertices:
            if SCARA:
                # Get the distance from origin for the polar coordinate.
                # Needs to be in 0-MAX_RHO range.
                rho = min((vertex.magnitude() / MAX_RADIUS) * MAX_RHO, MAX_RHO)

                # Compute the clockwise angle relative to Vector(1,0) around the origin.
                # Theta gets offset to ensure shortest path in one direction.
                rawTheta = math.atan2(vertex[0], vertex[1])
                rawTheta = (rawTheta + 2.0 * math.pi) % (2.0 * math.pi)
                deltaTheta = rawTheta - _previousRawTheta
                if deltaTheta < -math.pi:
                    deltaTheta += 2.0 * math.pi
                if deltaTheta > math.pi:
                    deltaTheta -= 2.0 * math.pi
                theta = _previousTheta + deltaTheta

                # Keep track of previous theta to ensure we always compute shortest
                # distance for the next theta.
                _previousRawTheta = rawTheta
                _previousTheta = theta

                x,y = scara_xy_from_thetarho(theta, rho)
            else:
                x = vertex[0]
                y = vertex[1]

            # TODO: Remove Houdini Code
            # pt = geo.createPoint()
            # pt.setPosition(hou.Vector3(x, y, 0))

            line_command = f"G01 X{x:.3f} Y{y:.3f} Z0.0"

            # Write Gcode command to file
            outfile.write(f"{line_command}\n")

    # Force the drawing to end on the perimeter of the canvas
    x,y = scara_xy_from_thetarho(_previousTheta, MAX_RHO)
    line_command = f"G01 X{x:.3f} Y{y:.3f} Z0.0"
    outfile.write(f"{line_command}\n")
