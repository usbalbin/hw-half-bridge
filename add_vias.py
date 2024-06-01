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
    def __init__(self, x: int, y: int):
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
        if math.isnan(val):
            raise Exception(f'p1: {p1}, p2: {p2}, p3: {p3}')
        elif val > 0:
            return ORIENTATION_CW
        elif val < 0:
            return ORIENTATION_CCW
        else:
            return ORIENTATION_COLLINEAR
    
    def __str__(self):
        return f'V(x: {self.x}, y: {self.y})'

class BoundingBox:
    def __init__(self, min_v: Vector, max_v: Vector):
        self.min = min_v
        self.max = max_v
    
    def intersects_point(self, p) -> bool:
        return p.x >= self.min.x and p.y >= self.min.y and p.x <= self.max.x and p.y <= self.max.y
    
    def __str__(self):
        return f'BB[min: {self.min}, max: {self.max}]'

i = 0

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

    def intersects(self, p: Vector):
        """ The main function that returns true if 'self' and 'l2' intersect. """
        # This is together with p forms a line along the x axis to p
        limit = Vector(int(2**31-1), p.y)

        # Find the four orientations needed for general and
        # special cases
        o1 = self.start.orientation(self.end, limit)
        o2 = self.start.orientation(self.end, p)
        o3 = limit.orientation(p, self.start)
        o4 = limit.orientation(p, self.end)
    
        # General case
        if o1 != o2 and o3 != o4:
            return True
    
        # Special Cases
        # l1 and l2.start are collinear and l2.start lies on segment l1
        if o1 == 0 and self.bounding_box().intersects_point(limit):
            #global i
            #i += 1
            #if i == 5:
            #    raise Exception(f'poop\n{l1}\n{l1.bounding_box()}\n{l2}')
            return True
    
        # l1 and l2.end are collinear and l2.end lies on segment l1
        if o2 == 0 and self.bounding_box().intersects_point(p):
            #raise "paap"
            return True
    
        # l2 and l1.start are collinear and l1.start lies on segment l2
        #if o3 == 0 and l2.bounding_box().intersects_point(l1.start):
        #    raise "pooop"
        #    return True
    
        # l2 and l1.end are collinear and l1.end lies on segment l2
        #if o4 == 0 and l2.bounding_box().intersects_point(l1.end):
        #    raise "baap"
        #    return True
    
        return False # Doesn't fall in any of the above cases

    def __str__(self):
        return f'Line{{start: {self.start}, end: {self.end}}}'

def is_inside_poly(p: Vector, lines: list[LineSegment]) -> bool:    
    hits = 0
    for line in lines:
        hits += line.intersects(p)
    
    # Odd number of line intersections means p is inside
    return (hits & 1) == 1

def vertices_to_lines(vertices: list[Vector]):
    lines = []
    lines.append(LineSegment(vertices[-1], vertices[0]))
    for i in range(0, len(vertices) - 1):
        lines.append(LineSegment(vertices[i], vertices[i + 1]))
    return lines

def calc_bounding_box(lines: list[LineSegment]) -> BoundingBox:
    bounding_box = BoundingBox(Vector(math.inf, math.inf), Vector(-math.inf, -math.inf))
    for line in lines:
        bounding_box.min = bounding_box.min.min(line.start)
        bounding_box.min = bounding_box.min.min(line.end)

        bounding_box.max = bounding_box.max.max(line.start)
        bounding_box.max = bounding_box.max.max(line.end)
    return bounding_box

def fill_poly(board: pcbnew.BOARD, lines: list[LineSegment], via_drill_size: float, via_pad_size: float, x_step: float, y_step: float, x_offset_per_line: float, net_name=None):
    bounding_box = calc_bounding_box(lines)
    margin = int(pcbnew.PCB_IU_PER_MM * via_pad_size // 2)
    y_min = int(bounding_box.min.y) + margin
    y_max = int(bounding_box.max.y) - margin
    x_min = int(bounding_box.min.x) + margin
    x_max = int(bounding_box.max.x) - margin
    y_step = int(y_step * pcbnew.PCB_IU_PER_MM)
    x_step = int(x_step * pcbnew.PCB_IU_PER_MM)

    for i, y in enumerate(range(y_min, y_max, y_step)):
        x_offset = int(i * x_offset_per_line * pcbnew.PCB_IU_PER_MM)
        for x in range(x_min - x_offset, x_max, x_step):
            if not is_inside_poly(Vector(x, y), lines):
                continue
            
            # https://forum.kicad.info/t/use-python-script-to-place-via/18268
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pcbnew.VECTOR2I(x, y))
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

        p = Vector(via.GetPosition().x, via.GetPosition().y)

        if calc_bounding_box(lines).intersects_point(p) and is_inside_poly(p, lines):
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
    board = pcbnew.GetBoard()
    drawings = [d for d in list(board.GetDrawings()) if d.GetLayer() == pcbnew.User_1 and d.GetClass() in ["DRAWSEGMENT", "MGRAPHIC", "PCB_SHAPE"]]
    texts = [d for d in list(board.GetDrawings()) if d.GetLayer() == pcbnew.User_1 and d.GetClass() == "PCB_TEXT"]

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

                distance = (via_drill_size + hole_to_hole_clearance) * step_factor
                x_step = distance
                y_step= distance * math.sin(math.pi/3)
                x_offset_per_line = distance * math.cos(math.pi/3)

                clear_vias_in_poly(board, lines)
                fill_poly(board, lines, via_drill_size, via_pad_size, x_step, y_step, x_offset_per_line, via_net)

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
        hole_to_hole_clearance = 0.254
        via_drill_size = 0.3
        via_pad_size = 0.45
        place_vias(via_drill_size, via_pad_size, hole_to_hole_clearance)


ViaPlugin().register() # Instantiate and register to Pcbnew
LargerViaPlugin().register() # Instantiate and register to Pcbnew
