# https://github.com/openscopeproject/InteractiveHtmlBom/blob/07a3d203caaf31b3994f6417edeb6d19c6793d91/InteractiveHtmlBom/ecad/kicad.py#L368

import math
import pcbnew
from pcbnew import FromMM
import os
from typing import Self

ORIENTATION_COLLINEAR = 0
ORIENTATION_CW = 1
ORIENTATION_CCW = 2

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other: Self):
        return Vector(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other: Self):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        if isinstance(other, Vector):
            return Vector(self.x * other.x, self.y * other.y)
        else:
            return Vector(self.x * other, self.y * other)
    
    def dot(self, other: Self):
        a = (self * other)
        return a.x + a.y
    
    def min(self, other: Self):
        return Vector(min(self.x, other.x), min(self.y, other.y))
    
    def max(self, other: Self):
        return Vector(max(self.x, other.x), max(self.y, other.y))

    def orientation(self, p2: Self, p3: Self): 
        """ Find the orientation of three points (self, p2, p3)
        # Returns returns one of: 
        # ORIENTATION_COLLINEAR
        # ORIENTATION_CW
        # ORIENTATION_CCW
        """
        p1 = self
        
        # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
        val = (p2.y - p1.y) * (p3.x - p2.x) - (p2.x - p1.x) * (p3.y - p2.y)
        if (val > 0): 
            return ORIENTATION_CW
        elif (val < 0): 
            return ORIENTATION_CCW
        else: 
            return ORIENTATION_COLLINEAR

class BoundingBox:
    def __init__(self, min_v: Vector, max_v: Vector):
        self.min = min_v
        self.max = max_v
    
    def intersects_point(self, p) -> bool:
        return p.x >= self.min.x and p.y >= self.min.y and p.x <= self.max.x and p.y <= self.max.y

class LineSegment:
    def __init__(self, start: Vector, end: Vector):
        self.start = start
        self.end = end

    def direction(self) -> Vector:
        """ returns normalized direction vector"""
        k = 1 / self.length()
        return (self.end - self.start) * k
    
    def length(self):
        delta = (self.end - self.start)
        return delta.dot(delta).sqrt()

    def bounding_box(self) -> BoundingBox:
        return BoundingBox(self.start.min(self.end), self.start.max(self.end))

    def intersects(self, l2: Self):
        """ The main function that returns true if 'self' and 'l2' intersect. """

        l1 = self

        # Find the four orientations needed for general and 
        # special cases 
        o1 = l1.start.orientation(l1.end, l2.start) 
        o2 = l1.start.orientation(l1.end, l2.end) 
        o3 = l2.start.orientation(l2.end, l1.start) 
        o4 = l2.start.orientation(l2.end, l1.end)
    
        # General case 
        if o1 != o2 and o3 != o4:
            return True
    
        # Special Cases 
        # l1 and l2.start are collinear and l2.start lies on segment l1 
        if o1 == 0 and l1.bounding_box().intersects_point(l2.start):
            return True
    
        # l1 and l2.end are collinear and l2.end lies on segment l1
        if o2 == 0 and l1.bounding_box().intersects_point(l2.end):
            return True
    
        # l2 and l1.start are collinear and l1.start lies on segment l2 
        if o3 == 0 and l2.bounding_box().intersects_point(l1.start):
            return True
    
        # l2 and l1.end are collinear and l1.end lies on segment l2 
        if o4 == 0 and l2.bounding_box().intersects_point(l1.end):
            return True
    
        return False # Doesn't fall in any of the above cases 

def is_inside_poly(p: Vector, lines: list[LineSegment]) -> bool:
    # This is a line along the x axis to p
    l1 = LineSegment(Vector(-math.inf, p.y), p)

    hits = 0
    for line in lines:
        hits += l1.intersects(line)
    
    # Odd number of line intersections means p is inside
    return (hits & 1) == 1

def vertices_to_lines(vertices: list[Vector]):
    lines = []
    lines.append(LineSegment(vertices[-1], vertices[0]))
    for i in range(0, len(vertices) - 1):
        lines.append(LineSegment(vertices[i], vertices[i + 1]))
    return lines

def fill_poly(board: pcbnew.BOARD, lines: list[LineSegment], step: float, via_drill_size: float, via_pad_size: float, x_step=0, net_name=None):
    bounding_box = BoundingBox(Vector(math.inf, math.inf), Vector(-math.inf, -math.inf))
    for line in lines:
        bounding_box.min = bounding_box.min.min(line.start)
        bounding_box.min = bounding_box.min.min(line.end)

        bounding_box.max = bounding_box.max.max(line.start)
        bounding_box.max = bounding_box.max.max(line.end)

    for i, y in enumerate(range(bounding_box.min.y, bounding_box.max.y, int(step * pcbnew.PCB_IU_PER_MM))):
        for x in range(bounding_box.min.x - int(i * x_step * pcbnew.PCB_IU_PER_MM), bounding_box.max.x, int(step * pcbnew.PCB_IU_PER_MM)):
            if not is_inside_poly(Vector(x, y), lines):
                continue
            
            # https://forum.kicad.info/t/use-python-script-to-place-via/18268
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pcbnew.VECTOR2I(int(x), int(y)))
            via.SetDrill(int(via_drill_size * pcbnew.PCB_IU_PER_MM))
            via.SetWidth(int(via_pad_size * pcbnew.PCB_IU_PER_MM))
            if not net_name is None:
                net = board.FindNet(net_name)
                via.SetNetCode(net.GetNetCode())
            board.Add(via)

def clear_vias_in_poly(board: pcbnew.BOARD, lines: list[LineSegment]):
    vias_to_remove = []
    for via in board.GetTracks():
        if via.GetClass() != 'PCB_VIA':
            continue

        if is_inside_poly(Vector(via.GetPosition().x, via.GetPosition().y), lines):
            vias_to_remove.append(via)

    for via in vias_to_remove:
        board.Remove(via)

"""
pcbnew.S_SEGMENT: "segment",
pcbnew.S_CIRCLE: "circle",
pcbnew.S_ARC: "arc",
pcbnew.S_CURVE: "curve",
pcbnew.S_RECT: "rect",
"""

def place_vias(via_drill_size, via_pad_size, hole_to_hole_clearance=0.254):
    distance = None
    board = pcbnew.GetBoard()
    drawings = [d for d in list(board.GetDrawings()) if d.GetLayer() == pcbnew.User_1 and d.GetClass() in ["DRAWSEGMENT", "MGRAPHIC", "PCB_SHAPE"]]
    texts = [d for d in list(board.GetDrawings()) if d.GetLayer() == pcbnew.User_1 and d.GetClass() == "PCB_TEXT"]

    distance = via_drill_size + hole_to_hole_clearance if distance is None else distance
    step = distance * math.sin(math.pi/3)
    x_step = distance * math.cos(math.pi/3)

    for d in drawings:
        if d.GetShape() == pcbnew.S_POLYGON:
            poly = d.GetPolyShape()
            for i in range(poly.OutlineCount()):
                shape = poly.Outline(i)
                vertices = []
                for point_index in range(shape.PointCount()):
                    p = shape.CPoint(point_index)
                    vertices.append(Vector(p.x, p.y))
                lines = vertices_to_lines(vertices)

                text = next((t.GetText() for t in texts if is_inside_poly(Vector(t.GetPosition().x, t.GetPosition().y), lines)), None)
                via_net = None
                step_factor = 1.0
                if text:
                    text_lines = text.splitlines()
                    via_net = text_lines[0].strip()
                    step_factor = float(text_lines[1].strip()) if len(text_lines) > 1 else 1.0


                clear_vias_in_poly(board, lines)
                fill_poly(board, lines, step * step_factor, via_drill_size, via_pad_size, x_step * step_factor, via_net)

class ViaPlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "usbalbin's via placer"
        self.category = "A descriptive category name"
        self.description = "A plugin for automatically placing via grids in polygons"
        self.show_toolbar_button = True # Optional, defaults to False
        self.icon_file_name = "" #os.path.join(os.path.dirname(__file__), 'simple_plugin.png') # Optional, defaults to ""

    def Run(self):
        hole_to_hole_clearance = 0.254

        # Hex pattern 0.48 mm
        hole_to_hole_clearance = 0.28
        via_drill_size = 0.2
        via_pad_size = 0.35

        #hole_to_hole_clearance = 0.254
        #via_drill_size = 0.3
        #via_pad_size = 0.45
        place_vias(via_drill_size, via_pad_size, hole_to_hole_clearance)

class LargerViaPlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "usbalbin's large via placer"
        self.category = "A descriptive category name"
        self.description = "A plugin for automatically placing via grids in polygons"
        self.show_toolbar_button = True # Optional, defaults to False
        self.icon_file_name = "" #os.path.join(os.path.dirname(__file__), 'simple_plugin.png') # Optional, defaults to ""

    def Run(self):
        hole_to_hole_clearance = 0.28
        via_drill_size = 0.3
        via_pad_size = 0.45
        place_vias(via_drill_size, via_pad_size, hole_to_hole_clearance)


ViaPlugin().register() # Instantiate and register to Pcbnew
LargerViaPlugin().register() # Instantiate and register to Pcbnew
