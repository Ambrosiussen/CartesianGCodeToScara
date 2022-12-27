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
LINE_SEGMENT_SIZE = 1
CENTER_DRAWING = True
SCARA = True
UNIFORM_SCALE = 1.0
DEFAULT_DRAWING_SPEED = 1000.0

#####################################
class Vertex():
    """
    Utility for storing and accessing x and y components of a Vector2 for a GCode vertex.
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def magnitude(self):
        """
        Compute magnitude of vertex position from origin.
        """
        return math.sqrt(self.x**2 + self.y**2)

    def distance_to(self, other):
        """
        Compute distance between this vertex instance and another.
        """
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y) **2)

    def __sub__(self, other):
        """
        Implements - operator for this class.
        """
        return np.array([self.x - other.x, self.y - other.y])

    def __isub__(self, other):
        """
        Implements -= operator for this class
        """
        self.x -= other.x
        self.y -= other.y
        return self

    def __imul__(self, other):
        """
        Implements *= operator for this class.
        """
        self.x *= other
        self.y *= other
        return self

def get_vertex_from_g01_command(command):
    """
    Creates a Vertex instance from a G01 GCode command.
    """
    X = float(re.search(r"X\d+\.\d+", command).group(0)[1:])
    Y = float(re.search(r"Y\d+\.\d+", command).group(0)[1:])
    return Vertex(X, Y)
    #return Vertex(float(elements[1][1:]), float(elements[2][1:]))

def get_samples_between_vertices(p1, p2, resample_distance):
    """
    Returns a list of Vertex() instances which are linear samples between pt1 and pt2.
    """
    pt_distance = p1.distance_to(p2)

    if abs(pt_distance) < 0.0001 or resample_distance > pt_distance or abs(resample_distance)<0.0001:
        return [p1]
    else:
        step_x, step_y = (p2 - p1) * (resample_distance / pt_distance)
        
        if abs(step_x) < 0.0001:
            y = np.arange(p1.y, p2.y, step_y)
            x = np.zeros(len(y)) + p1.x
        elif abs(step_y) < 0.0001:
            x = np.arange(p1.x, p2.x, step_x)
            y = np.zeros(len(x)) + p1.y
        else:
            x = np.arange(p1.x, p2.x, step_x)
            y = np.arange(p1.y, p2.y, step_y)    

        return [Vertex(x, y[i]) for i, x in enumerate(x)]

def cartesian_xy_from_thetarho(theta, rho):
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
            x_vals.append(vertex.x)
            y_vals.append(vertex.y)

# Computing the average position of all gcode commands.
# We do this to center the drawing for SCARA machines with the center being 0,0
centroid = Vertex(sum(x_vals)/len(x_vals), sum(y_vals)/len(x_vals)) if CENTER_DRAWING else Vertex(0,0)

#TODO: Remove Houdini code
node = hou.pwd() 
geo = node.geometry()

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
        vertex1 *= UNIFORM_SCALE

        # If the GCode file has one more command after it, create samples from current
        # vertex to the next with the specified stepsize. This has to be done
        # because the SCARA bot is incapable of moving in straight lines.
        if idx+1 < len(valid_commands):
            vertex2 = get_vertex_from_g01_command(valid_commands[idx+1])
            vertex2 -= centroid
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
                rawTheta = math.atan2(vertex.x, vertex.y)
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

                x,y = cartesian_xy_from_thetarho(theta, rho)
            else:
                x = vertex.x
                y = vertex.y

            # TODO: Remove Houdini Code
            pt = geo.createPoint()
            pt.setPosition(hou.Vector3(x, y, 0))

            line_command = f"G01 X{x:.3f} Y{y:.3f} Z0.0"

            # Write Gcode command to file
            outfile.write(f"{line_command}\n")

    # Force the drawing to end on the perimeter of the canvas
    x,y = cartesian_xy_from_thetarho(_previousTheta, MAX_RHO)
    line_command = f"G01 X{x:.3f} Y{y:.3f} Z0.0"
    outfile.write(f"{line_command}\n")
