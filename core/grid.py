"""Dependency-free cell traversal shared by the planner's collision checks."""

import math


def crossed_cells(start, end):
    """Yield crossed cells inside the endpoint cells' bounding rectangle.

    Cells use floor coordinates. At an interior grid-corner crossing both
    side neighbours are included conservatively, not just the diagonal cell.
    An axis already in its final cell never steps beyond the endpoint.
    """
    x0, y0 = float(start[0]), float(start[1])
    x1, y1 = float(end[0]), float(end[1])
    ix, iy = math.floor(x0), math.floor(y0)
    final = (math.floor(x1), math.floor(y1))
    yield (ix, iy)
    dx, dy = x1 - x0, y1 - y0
    sx = 1 if dx > 0 else -1
    sy = 1 if dy > 0 else -1
    t_max_x = ((ix + (sx > 0)) - x0) / dx if dx else math.inf
    t_max_y = ((iy + (sy > 0)) - y0) / dy if dy else math.inf
    t_dx = abs(1.0 / dx) if dx else math.inf
    t_dy = abs(1.0 / dy) if dy else math.inf
    while (ix, iy) != final:
        # A negative-going endpoint on a boundary belongs to the cell above
        # that boundary. Crossing it at t=1 would overshoot that final cell.
        tx = t_max_x if ix != final[0] else math.inf
        ty = t_max_y if iy != final[1] else math.inf
        if tx == ty:
            yield (ix + sx, iy)
            yield (ix, iy + sy)
            ix += sx
            iy += sy
            t_max_x += t_dx
            t_max_y += t_dy
        elif tx < ty:
            ix += sx
            t_max_x += t_dx
        else:
            iy += sy
            t_max_y += t_dy
        yield (ix, iy)
