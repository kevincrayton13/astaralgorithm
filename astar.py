import matplotlib.pyplot as plt
import numpy as np
from numpy import random
from matplotlib import animation
from math import dist

#models the queue for the algorithm
class Queue:
    def __init__(self, g, r):
        self.queue = []
        self.finished = []
        self.found = False
        self.exploded = []
        self.pathx = []
        self.pathy = []
    def reset(self, event):
        if event.key == 'r':
            self.queue = []
            self.finished = []
            self.found = False
            self.exploded = []
            self.pathx = []
            self.pathy = []
#models the start and end points
class dot:
    def __init__(self, start, finish, id, width, height):
        self.position = [round(float((width * random.rand(1)))), round(float((height * random.rand(1))))]

        #these must be stored here so they can be used in controls()
        self.width = width
        self.height = height
        #part of the click and drag feature for obstacles and endpoints
        self.move = None
        self.obstacles = None
        #stores coordinates of obstacle nodes ALL OBSTACLES ARE HANDLED THROUGH DOT OBJECT 'g'
        self.obstacle_x = []
        self.obstacle_y = []
        #stores coordinates of the algorithm's path

        #mutes input when algorithm is running
        self.algore = None

        #stores info on discovered points
        self.fillx = []
        self.filly = []
        #needed to end the algorithn
        self.found = False
        if id == 'start':
            start.set_data(self.position)
        else:
            finish.set_data(self.position)

    def controls(self, event):
        #defined to save a little typing
        x = self.position[0]
        y = self.position[1]
        try:
            #zone of selection for endpoints
            if event.button == 1 and self.algore != True:
                if (event.xdata > (x - (.035 * self.width)) and event.xdata < (x + (.035 * self.width)) and
                event.ydata > (y - (.035 * self.height)) and event.ydata < (y + (.035 * self.height))):
                    self.move = True
        except:
            pass
        if event.button == 3 and self.algore != True:
            self.obstacles = True
        try:
            if self.move and self.algore != True: #allows dragging of the dot
                #enables user to see endpoints being dragged
                if event.xdata != None and event.ydata != None:
                    self.position[0] = event.xdata
                    self.position[1] = event.ydata
            if self.obstacles and self.algore != True:
                if event.xdata != None and event.ydata != None:
                    self.obstacle_x.append(round(event.xdata))
                    self.obstacle_y.append(round(event.ydata))

        except:
            pass

    #controls when a button is released
    def release(self, event):
        try:
            #snaps endpoint to nearest whole coordinate
            if event.button == 1 and self.move and self.algore != True:
                self.position[0] = round(event.xdata)
                self.position[1] = round(event.ydata)
                self.move = False
        except TypeError:
                if self.algore != True:
                    self.position[0] = round(self.position[0])
                    self.position[1] = round(self.position[1])
                    self.move = False
        if event.button == 3:
            self.obstacles = False

    #clears all lists except endpoint positions
    def reset(self, event):
        if event.key == 'r':
            self.obstacle_x = []
            self.obstacle_y = []
            self.pathx = []
            self.pathy = []
            self.algore = False
            self.fillx = []
            self.filly = []

            #mutes input
    def algor_start(self, event):
        if event.key == 'enter':
            #disables all user input
            self.algore = True
            self.move = False
            self.obstacles = False


#ensures that no dots are too close together or on top of each other
def divider(g, r):
    if (r.position[0] > (g.position[0] - (.035 * g.width)) and r.position[0] < (g.position[0] + (.035 * g.width)) and
    r.position[1] > (g.position[1] - (.035 * g.height)) and r.position[1] < (g.position[1] + (.035 * g.height))):
        r.position = [round(float((g.width * random.rand(1)))), round(float(g.height * random.rand(1)))]

    obstacle_x = []
    obstacle_y = []
    for i, c in enumerate(zip(g.obstacle_x, g.obstacle_y)):
        if c[0] == r.position[0] and c[1] == r.position[1]:
            g.obstacle_x.pop(i)
            g.obstacle_y.pop(i)
            continue
        if c[0] == g.position[0] and c[1] == g.position[1]:
            g.obstacle_x.pop(i)
            g.obstacle_y.pop(i)
            continue

#assists with sorting the finished items for the path_builder
def finish_sort(ls):
    return ls[1]

#builds a blue path after the algorithm has been completed
def path_builder(queue, g):
    node = queue.queue[0]
    queue.finished.sort(key = finish_sort)
    while True:
        queue.pathx.append(node[2][0])
        queue.pathy.append(node[2][1])
        if node[2][0] == g.position[0] and node[2][1] == g.position[1]:
            break

        for i, path in enumerate(queue.finished):
            if node[2][0] == path[2][0] and node[2][1] == path[2][1]:
                continue
            if ((node[2][0] - 1) <= path[2][0] <= (node[2][0] + 1)
            and (node[2][1] - 1) <= path[2][1] <= (node[2][1] + 1)):
                node = queue.finished[i]
                break


#function for animation
def tick(frame, start, finish, path, obstacles, g, r, fill, current, queue):
    if g.fillx == []:
        current.set_data([], [])
    fill.set_data(g.fillx, g.filly)
    start.set_data(g.position)
    finish.set_data(r.position)
    path.set_data([queue.pathx, queue.pathy])
    if g.algore != True:
        obstacles.set_data(g.obstacle_x, g.obstacle_y)

        divider(g,r)
    #runs algorithm on detected keypress
    if g.algore == True and queue.found == False:
        #just in case if enter is pressed mid-click
        r.position[0] = round(r.position[0])
        r.position[1] = round(r.position[1])
        g.position[0] = round(g.position[0])
        g.position[1] = round(g.position[1])

        astar(g, r, fill, path, queue, current)

#returns all possible nodes for astar algorithm
def explode(g, r, queue, current):
    #node tuple order: value, cost, location
    node = queue.queue[0]
    queue.finished.append(node)
    possible = []
    x = node[2][0]
    y = node[2][1]
    for xx, yy in queue.exploded:
        if x == xx and y == yy:
            return
        else:
            continue

    #list exploded makes sure that a point isn't searched twice
    queue.exploded.append((x, y))
    current.set_data(x, y)

    cost = dist([(x + 1), (y + 1)], [x, y]) + node[1]
    possible.append((dist(r.position, [(x + 1), (y + 1)]) + cost, cost, [(x + 1), (y + 1)]))
    cost1 = dist([(x - 1), (y + 1)], [x, y]) + node[1]
    possible.append((dist(r.position, [(x - 1), (y + 1)]) + cost1, cost1, [(x - 1), (y + 1)]))
    cost2 = dist([(x + 1), (y - 1)], [x, y]) + node[1]
    possible.append((dist(r.position, [(x + 1), (y - 1)]) + cost2, cost2, [(x + 1), (y - 1)]))
    cost3 = dist([(x - 1), (y - 1)], [x, y]) + node[1]
    possible.append((dist(r.position, [(x - 1), (y - 1)]) + cost3, cost3, [(x - 1), (y - 1)]))
    cost4 = dist([(x), (y + 1)], [x, y]) + node[1]
    possible.append((dist(r.position, [(x), (y + 1)]) + cost4, cost4, [(x), (y + 1)]))
    cost5 = dist([(x + 1), (y)], [x, y]) + node[1]
    possible.append((dist(r.position, [(x + 1), (y)]) + cost5, cost5, [(x + 1), (y)]))
    cost6 = dist([(x), (y - 1)], [x, y]) + node[1]
    possible.append((dist(r.position, [(x), (y - 1)]) + cost6, cost6, [(x), (y - 1)]))
    cost7 = dist([(x - 1), (y)], [x, y]) + node[1]
    possible.append((dist(r.position, [(x - 1), (y)]) + cost7, cost7, [(x - 1), (y)]))
    #removes point if it exists as obstacle or endpoint
    #must be done in two stages: First nested loop id's what needs to be eliminated, 2nd loop does removal
    hitlist = []
    for i, node in enumerate(possible):
        x = node[2][0]
        y = node[2][1]
        for xx, yy in zip(g.obstacle_x, g.obstacle_y):
            if x == xx and y == yy:
                if i not in hitlist:
                    hitlist.append(i)

        if x == g.position[0] and y == g.position[1]:
            if i not in hitlist:
                hitlist.append(i)
        if x < 0 or y < 0:
            if i not in hitlist:
                hitlist.append(i)
        if x > g.width or y > g.height:
            if i not in hitlist:
                hitlist.append(i)
    hitlist.sort(reverse = True)
    for index in hitlist:
        possible.pop(index)
    #adds to graph
    for nnode in possible:
        if nnode[2][0] == r.position[0] and nnode[2][1] == r.position[1]:
            continue
        g.fillx.append(nnode[2][0])
        g.filly.append(nnode[2][1])
    queue.finished.extend(possible)
    #found comes back true if the endpoint has been investigated
    if queue.queue[0][2][0] == r.position[0] and queue.queue[0][2][1] == r.position[1]:
        queue.found = True
        current.set_data([], [])
        path_builder(queue, g)
    return possible
#the actual algorithm: value = dist to end + cost to travel
def astar(g, r, fill, path, queue, current):
    #node tuple order: value, path cost, location
    if queue.queue == []:
        queue.queue = [(dist(g.position, r.position) + 0, 0, g.position, [(g.position)])]
    new_nodes = explode(g, r, queue, current)
    if new_nodes != None:
        queue.queue.extend(new_nodes)
    queue.queue.sort()
    fill.set_data(g.fillx, g.filly)
    queue.queue.pop(0)


def main():
    width = 20
    height = 20
    #all of this is figure construction/animation
    figure = plt.figure()
    axes = plt.axes(xlim = (0, width), ylim = (0, height))
    start, = axes.plot([], [], markersize = 12, c = 'g', marker = 'o', ls = '')
    finish, = axes.plot([], [], markersize = 12, c = 'r', marker = 'o', ls = '')
    fill, = axes.plot([], [], markersize = 12, c = '#EEEE00', marker = 'o', ls = '')
    path, = axes.plot([], [], markersize = 12, c = '#0000EE', marker = 'o', linewidth = 3, ls = '-')
    obstacles, = axes.plot([], [], markersize = 12, c = '#000000', marker = 'o', ls = '')
    current, = axes.plot([], [], markersize = 12, c = '#00EEEE', marker = 'o', ls = '')
    g = dot(start, finish, 'start', width, height)
    r = dot(start, finish, 'finish', width, height)
    queue = Queue(g, r)
    plt.xticks(np.arange(0, width, step = 1))
    plt.yticks(np.arange(0, height, step = 1))
    plt.title('A* Algorithm Visualizer (press enter to run)')
    plt.xlabel('''left click and drag Start/Finish Points to move, right click and drag to create obstacles''')
    plt.ylabel('Press R to reset')
    plt.grid()
    ani = animation.FuncAnimation(figure, tick, fargs = (start, finish, path, obstacles, g, r, fill, current, queue), interval = 60)

    #all of the button/key controls
    connection = figure.canvas.mpl_connect('button_press_event', g.controls)
    connection1 = figure.canvas.mpl_connect('button_press_event', r.controls)
    connection2 = figure.canvas.mpl_connect('motion_notify_event', g.controls)
    connection3 = figure.canvas.mpl_connect('motion_notify_event', r.controls)
    connection4 = figure.canvas.mpl_connect('button_release_event', g.release)
    connection5 = figure.canvas.mpl_connect('button_release_event', r.release)
    connection6 = figure.canvas.mpl_connect('key_press_event', g.reset)
    connection7= figure.canvas.mpl_connect('key_press_event', r.reset)
    connection8 = figure.canvas.mpl_connect('key_press_event', g.algor_start)
    connection9 = figure.canvas.mpl_connect('key_press_event', r.algor_start)
    connection10= figure.canvas.mpl_connect('key_press_event', queue.reset)



    plt.show()

if __name__ == '__main__':
    main()
