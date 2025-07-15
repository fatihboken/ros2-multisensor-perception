import numpy as np
import heapq
from scipy.ndimage import distance_transform_edt

def points_to_grid(points, grid_size=50):
    points = np.array(points)
    x_min, y_min = points.min(axis=0)
    x_max, y_max = points.max(axis=0)
    x_range = int(np.ceil((x_max - x_min) / grid_size)) + 1
    y_range = int(np.ceil((y_max - y_min) / grid_size)) + 1
    grid = np.zeros((x_range, y_range), dtype=np.uint8)
    for x, y in points:
        xi = int((x - x_min) / grid_size)
        yi = int((y - y_min) / grid_size)
        grid[xi, yi] = 1  # Engel
    return grid, x_min, y_min, grid_size

def grid_to_xy(xi, yi, x_min, y_min, grid_size):
    x = x_min + xi * grid_size + grid_size / 2
    y = y_min + yi * grid_size + grid_size / 2
    return x, y

def xy_to_grid(x, y, x_min, y_min, grid_size, grid_shape):
    xi = int((x - x_min) / grid_size)
    yi = int((y - y_min) / grid_size)
    xi = np.clip(xi, 0, grid_shape[0]-1)
    yi = np.clip(yi, 0, grid_shape[1]-1)
    return xi, yi

def find_nearest_free(grid, xi, yi):
    if grid[xi, yi] == 0:
        return (xi, yi)
    free_mask = (grid == 0)
    dist, (xarr, yarr) = distance_transform_edt(free_mask, return_indices=True)
    return xarr[xi, yi], yarr[xi, yi]

def astar(grid, start, goal):
    neighbors = [(-1,0),(1,0),(0,-1),(0,1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:np.linalg.norm(np.subtract(start, goal))}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        close_set.add(current)
        for i, j in neighbors:
            neighbor = (current[0]+i, current[1]+j)
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0], neighbor[1]] == 1:
                    continue
            else:
                continue
            tentative_g_score = gscore[current] + 1
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue
            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + np.linalg.norm(np.subtract(neighbor, goal))
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return None
