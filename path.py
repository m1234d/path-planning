import heapq
import math
import numpy as np

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

possibleDir = ["North","East","South","West"]
possibleMoves = ["Left","Right","Forward"]
dirVectors = {"North":(0,-1),"East":(1,0),"South":(0,1),"West":(-1,0)}

def makeMove(state,action,grid):
    (x,y,d) = state
    lenX = len(grid[0])
    lenY = len(grid)

    if action == "Left":
        ind = possibleDir.index(d)
        d = possibleDir[(ind-1)%4]
        return (x,y,d)

    if action == "Right":
        ind = possibleDir.index(d)
        d = possibleDir[(ind+1)%4]
        return (x,y,d)

    if action == "Forward":
        vec = dirVectors[d]
        (newX,newY) = (x+vec[0],y+vec[1])
        if 0 <= newX and newX < lenX and 0 <= newY and newY < lenY and grid[newY][newX] != 1:
            return (newX,newY,d)
    return None


def expandNode(grid,state):
    states = []
    for action in possibleMoves:
        nextState = makeMove(state,action,grid)
        if nextState != None:
            states += [(nextState,action)]
    return states

def heuristic(cur,final):
    (x,y,d) = cur
    (x1,y1,d1) = final
    return abs(x-x1) + abs(y-y1)

def a_star_search(grid, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            return make_path(came_from,goal,start)
        for (next,action) in expandNode(grid,current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = (current,action)
    print("No path found")
    return None

def make_path(came_from,goal,start):
    path = []
    curr = goal
    while came_from[curr] != None:
        (prev,action) = came_from[curr]
        path.insert(0,action)
        curr = prev
    return path

def turn(current, direction):
    value = 0
    if current == 'North':
        value = 0
    elif current == 'East':
        value = 1
    elif current == 'South':
        value = 2
    elif current == 'West':
        value = 3

    if direction == 'Right':
        value = (value + 1) % 4
    elif direction == 'Left':
        value = (value - 1) % 4

    new = 'North'
    if value == 0:
        new = 'North'
    elif value == 1:
        new = 'East'
    elif value == 2:
        new = 'South'
    elif value == 3:
        new = 'West'

    return new

def get_angle(direction):
    if direction == 'South':
        return 0
    elif direction == 'East':
        return 90
    elif direction == 'North':
        return 180
    elif direction == 'West':
        return -90

def get_direction(angle):
    if angle == 0:
        return 'South'
    elif angle == 90:
        return 'East'
    elif angle == 180:
        return 'North'
    elif angle == -90:
        return 'West'

def get_points(start, path):
    start = (start[0], start[1], get_angle(start[2]))
    points = [start]
    lastD = ''
    count = 0
    print(path)
    for i in range(len(path) + 1):
        if i >= len(path):
            direction = ""
        else:
            direction = path[i]
        if lastD == direction:
            count += 1
        else:
            if lastD == '':
                count += 1
                lastD = direction
                continue
            currentDirection = get_direction(points[-1][2])
            currentY = points[-1][0]
            currentX = points[-1][1]
            if lastD == 'Forward':
                if currentDirection == 'South':
                    # Increase x
                    newPoint = (currentY, currentX + count, get_angle('South'))
                    points.append(newPoint)
                elif currentDirection == 'East':
                    # Increase y
                    newPoint = (currentY + count, currentX, get_angle('East'))
                    points.append(newPoint)
                elif currentDirection == 'North':
                    # Decrease x
                    newPoint = (currentY, currentX - count, get_angle('North'))
                    points.append(newPoint)
                elif currentDirection == 'West':
                    # Decrease y
                    newPoint = (currentY - count, currentX, get_angle('West'))
                    points.append(newPoint)

            else:
                newDirection = currentDirection
                for j in range(count):
                    newDirection = turn(newDirection, lastD)

                points[-1] = (currentY, currentX, get_angle(newDirection))

            count = 1
            lastD = direction

    if points[0][2] != start[2]:
        return points
    return points[1:]

def main(start, end):
    #54x72
    #add buffer of 5 to everything
    grid = np.zeros([54, 72])

    #Walls
    grid[0:5] = 1
    grid[49:53] = 1
    grid[:, 0:5] = 1
    grid[:, 67:72] = 1

    #First square
    grid[31:47, 49:65] = 1

    #This will be removed
    grid[12:40, 0:52] = 1

    #grid = np.zeros([54, 72])

    #grid[20:54, 0:60] = 1
    print(grid)
    # grid = [[0, 1, 0, 0, 0],
    #         [0, 0, 0, 0, 1],
    #         [0, 1, 0, 0, 1],
    #         [0, 1, 0, 1, 0],
    #         [0, 1, 0, 0, 0]]

    # South: +x, East: +y

    # start = (y, x, 'South')
    path = a_star_search(grid, start, end)


    points = get_points(start, path)
    for i in range(len(points)):
        points[i] = (points[i][1], points[i][0], points[i][2]*math.pi/180)

    return points