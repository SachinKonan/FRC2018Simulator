import numpy as np
import matplotlib.pyplot as plt

grid = np.array([
[0,0,1,0,1],
[1,0,1,0,0],
[0,0,0,0,1],
[0,0,1,0,1]
])
shape = grid.shape
start = [0,0]
end = [1,4]
"""plt.imshow(grid, cmap='Greys',  interpolation='nearest')
plt.grid()
plt.show()"""

def getOptions(coords):
    neighbor_coords = []
    global grid
    global shape
    if(grid[coords[0]][coords[1]] != 1):
        if(coords[0] < shape[0] -1 and grid[coords[0] + 1, coords[1]] != 1):
            neighbor_coords.append([coords[0] + 1, coords[1]])
        if(coords[0] > 0 and grid[coords[0] - 1, coords[1]] != 1):
            neighbor_coords.append([coords[0] - 1, coords[1]])
        if(coords[1] < shape[1] - 1 and grid[coords[0], coords[1] + 1] != 1):
            neighbor_coords.append([coords[0], coords[1] + 1])
        if(coords[1] > 0 and grid[coords[0], coords[1] - 1] != 1):
            neighbor_coords.append([coords[0], coords[1] - 1])

    return neighbor_coords

def minPicker(mat,mask):
    min = 999
    idx = []
    global grid
    global start
    for i in range(0, len(mat)):
        for j in range(0, len(mat[0])):
            if(grid[i][j] != 1 and mat[i][j] < min and mask[i][j] == 0):
                min = mat[i][j]
                idx = [i,j]
    return idx

def euclidist(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def backwards(distance_mat, pos, path = []):
    if(distance_mat[pos[0],pos[1]] == 0):
        return path
    opts = getOptions(pos)
    cell = min(opts, key = lambda x: distance_mat[x[0], x[1]])
    diff = [cell[0] - pos[0], cell[1] - pos[1]]
    if(diff[0] != 0 and diff[1] == 0):
        if(diff[0] > 0):
            path.insert(0,'N1')
        else:
            path.insert(0, 'S1')
    elif(diff[0] == 0 and diff[1] != 0):
        if(diff[1] > 0):
            path.insert(0,'W1')
        else:
            path.insert(0, 'E1')
    else:
        path.insert(0, 'invalid')
    return backwards(distance_mat, cell, path)

opts = getOptions(start)
visited = np.zeros(shape = shape)
distances = np.ones(shape = shape)*200
distances[start[0], start[1]] = 0

"""for i in range(0, len(grid)):
    for j in range(0, len(grid[0])):
        print('At row %s and col %s' % (i,j), end= '')
        print(getOptions([i,j], shape))"""


while(not any(map(lambda x: x[0] == end[0] and x[1] == end[1], opts))):
    cell = minPicker(distances, visited)
    visited[cell[0], cell[1]] = 1
    opts = getOptions(cell)
    for i,j in opts:
        if(visited[i,j] != 1):
            distances[i,j] = min(distances[i,j], distances[cell[0], cell[1]] + 1 + euclidist([i,j], end) )
    print("at cell: %s r and %s c" % (cell[0], cell[1]))
    print(opts)
    print(distances)
    print(visited)

directions = backwards(distances, end)
print(directions)
