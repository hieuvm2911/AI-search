from array import *
from queue import PriorityQueue
from unittest.mock import seal
import pygame
from pygame.locals import *
import os
import time
import sys


red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
gray = (128, 128, 128)
black = (0, 0, 0)
white = (255, 255, 255)
light_yellow = (255, 255, 102)
light_green = (0,128,0)
# Initialize pygame
pygame.init()
#create the screen
screen = pygame.display.set_mode((20*35, 20*20))

# fill a cell with a color and draw a border
def fill_cell (x1, y1, x2, y2, color):
    pygame.draw.rect(screen, color, (x1, y1, x2, y2))
    pygame.draw.rect(screen, black, (x1, y1, x2, y2), 1)


#fill screen background with white
screen.fill(white)


def read_maze(file_path):
    maze = []
    bonus = {}
    with open(file_path, "r") as f:
        n = int(f.readline()) # number of bonus point
        if n > 0:
            for i in range(n):
                i, j, p = map(int, f.readline().split())
                bonus[(i, j)] = p
        for line in f:
            maze.append(line)

    # check the goal in maze (first and last line + first and last column)
    for i in range(len(maze[0])):
        if maze[0][i] == " ":
            maze[0] = maze[0][:i] + "G" + maze[0][i+1:]
    for i in range(len(maze[len(maze) - 1])):
        if maze[len(maze) - 1][i] == " ":
            maze[len(maze) - 1] = maze[len(maze) - 1][:i] + "G" + maze[len(maze) - 1][i+1:]
    for i in range(1, len(maze) - 2):
        if maze[i][0] == " ":
            maze[i] = maze[i][:0] + "G" + maze[i][1:]
        elif maze[i][len(maze[i]) - 1] == " ":
            maze[i] = maze[i][:len(maze[i]) - 1] + "G" + maze[i][len(maze[i]):]

    return maze, bonus


# draw the maze
def draw_maze (maze):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == "+":
                fill_cell(j*20, i*20, 20, 20, blue)
            elif maze[i][j] == "S":
                fill_cell(j*20, i*20, 20, 20, green)
            elif maze[i][j] == " " or maze[i][j] == "G":
                fill_cell(j*20, i*20, 20, 20, white)
            elif maze[i][j] == "x" or maze[i][j] == "X":
                fill_cell(j*20, i*20, 20, 20, gray)


#heruistic function using euler distance
def euler_heuristic(current, goal):
    return (float) ((current[0] - goal[0])**2 + (current[1] - goal[1])**2)**0.5


#heruistic function using manhattan distance
def manhattan_heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])


def heuristic(type, current, goal):
    if type == 1:
        return euler_heuristic(current, goal)
    else:
        return manhattan_heuristic(current, goal)

#find the start and goal position
def find_start_goal(maze):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == "S":
                start = (i, j)
            elif maze[i][j] == "G":
                goal = (i, j)
    return start, goal


#find the neighbors of a cell
def find_neighbors(maze, current):
    neighbors = []
    if current[0] > 0:
        if maze[current[0] - 1][current[1]] != "x" and maze[current[0] - 1][current[1]] != "X":
            neighbors.append((current[0] - 1, current[1]))
    if current[0] < len(maze) - 1:
        if maze[current[0] + 1][current[1]] != "x" and maze[current[0] + 1][current[1]] != "X":
            neighbors.append((current[0] + 1, current[1]))
    if current[1] > 0:
        if maze[current[0]][current[1] - 1] != "x" and maze[current[0]][current[1] - 1] != "X":
            neighbors.append((current[0], current[1] - 1))
    if current[1] < len(maze[0]) - 1:
        if maze[current[0]][current[1] + 1] != "x" and maze[current[0]][current[1] + 1] != "X":
            neighbors.append((current[0], current[1] + 1))
    return neighbors


#find the path from start to goal using dfs
def dfs(maze):
    start, goal = find_start_goal(maze)
    frontier = []
    frontier.append(start)
    came_from = {}
    came_from[start] = None
    while len(frontier) > 0:
        current = frontier.pop()
        if current == goal:
            fill_cell(current[1]*20, current[0]*20, 20, 20, red)
            break
        for next in find_neighbors(maze, current):
            if next not in came_from:
                frontier.append(next)
                fill_cell(next[1]*20, next[0]*20, 20, 20, light_yellow)
                pygame.display.update()
                pygame.time.wait(20)
                came_from[next] = current
    return came_from

#find the path from start to goal using uniform-cost search
def ucs(maze):
    start, goal = find_start_goal(maze)
    frontier = []
    frontier.append(start)
    came_from = {}
    came_from[start] = None
    cost = {}
    cost[start] = 0
    while True:
        # if cannot find the goal, return None
        if len(frontier) == 0:
            return None
        current = frontier[0]
        for i in range(len(frontier)):
            if cost[frontier[i]]  < cost[current] :
                current = frontier[i]
        if current == goal:
            fill_cell(current[1]*20, current[0]*20, 20, 20, red)
            break
        frontier.remove(current)
        for next in find_neighbors(maze, current):
            new_cost = cost[current] + 1
            if next not in cost or new_cost < cost[next]:
                cost[next] = new_cost
                frontier.append(next)
                fill_cell(next[1]*20, next[0]*20, 20, 20, light_yellow)
                pygame.display.update()
                pygame.time.wait(20)
                came_from[next] = current
    return came_from

#find the path from start to goal using greedy best first search
def gbfs(maze, heur_type):
    start, goal = find_start_goal(maze)
    frontier = []
    frontier.append(start)
    came_from = {}
    came_from[start] = None
    while True:
        # if cannot find the goal, return None
        if len(frontier) == 0:
            return None
        current = frontier[0]
        for i in range(len(frontier)):
            if heuristic(heur_type,frontier[i], goal) < heuristic(heur_type, current, goal):
                current = frontier[i]
        if current == goal:
            fill_cell(current[1]*20, current[0]*20, 20, 20, red)
            break
        frontier.remove(current)
        for next in find_neighbors(maze, current):
            if next not in came_from:
                frontier.append(next)
                fill_cell(next[1]*20, next[0]*20, 20, 20, light_yellow)
                pygame.display.update()
                pygame.time.wait(20)
                came_from[next] = current
    return came_from
    
#find the path from start to goal using A* search
def astar(maze, heur_type):
    start, goal = find_start_goal(maze)
    frontier = []
    frontier.append(start)
    came_from = {}
    came_from[start] = None
    cost = {}
    cost[start] = 0
    while True:
        # if cannot find the goal, return None
        if len(frontier) == 0:
            return None
        current = frontier[0]
        for i in range(len(frontier)):
            if cost[frontier[i]] + heuristic(heur_type, frontier[i], goal) < cost[current] + heuristic(heur_type, current, goal):
                current = frontier[i]
        if current == goal:
            fill_cell(current[1]*20, current[0]*20, 20, 20, red)
            break
        frontier.remove(current)
        for next in find_neighbors(maze, current):
            new_cost = cost[current] + 1
            if next not in cost or new_cost < cost[next]:
                cost[next] = new_cost
                frontier.append(next)
                fill_cell(next[1]*20, next[0]*20, 20, 20, light_yellow)
                pygame.display.update()
                pygame.time.wait(20)
                came_from[next] = current
    return came_from


def astar2(maze, bonus):
    start, goal = find_start_goal(maze)
    frontier = []
    frontier.append(start)
    came_from = {}
    came_from[start] = None
    closed = []
    cost = {}
    cost[start] = 0
    while True:
        # if cannot find the goal, return None
        if len(frontier) == 0:
            return None
        current = frontier[0]
        for i in range(len(frontier)):
            if cost[frontier[i]] + heuristic(1, frontier[i], goal) < cost[current] + heuristic(1, current, goal):
                current = frontier[i]
        if current == goal:
            fill_cell(current[1]*20, current[0]*20, 20, 20, red)
            break
        frontier.remove(current)
        closed.append(current)
        for next in find_neighbors(maze, current):
            if next in closed:
                continue
            if next in bonus:
                new_cost = cost[current] + bonus[next]
            else:
                new_cost = cost[current] + 1
            if next not in cost or new_cost < cost[next]:
                cost[next] = new_cost
                frontier.append(next)
                if next in bonus:
                    fill_cell(next[1]*20, next[0]*20, 20, 20, blue)
                else:
                    fill_cell(next[1]*20, next[0]*20, 20, 20, light_yellow)
                pygame.display.update()
                pygame.time.wait(20)

                came_from[next] = current
    return came_from, cost[goal]


#draw the path from start to goal
def draw_path(maze, came_from):
    if came_from == None:
        return
    start, goal = find_start_goal(maze)
    current = goal
    while current != start:
        current = came_from[current]
        fill_cell(current[1]*20, current[0]*20, 20, 20, green)
        pygame.display.update()
        pygame.time.wait(20)

def draw_path1(maze, came_from, bonus):
    if came_from == None:
        return
    start, goal = find_start_goal(maze)
    current = goal
    while current != start: 
        current = came_from[current]
        if current in bonus:
            fill_cell(current[1]*20, current[0]*20, 20, 20, light_green)
        else:
            fill_cell(current[1]*20, current[0]*20, 20, 20, green)
        pygame.display.update()
        pygame.time.wait(20)


#write path to output file
def export_output(maze, came_from, file_path):
    # directory of output file
    f = open("./output/" + file_path + ".txt", "w")
    start, goal = find_start_goal(maze)
    if came_from == None:
        f.write("NO")
    else:
        cost = 0
        current = goal
        while current != start:
            current = came_from[current]
            cost += 1
        f.write(str(cost))
    f.close()
    # save screen to output file
    pygame.image.save(screen, "./output/" + file_path + ".jpg")

def export_output1(file_path, cost):
    # directory of output file
    f = open("./output/" + file_path + ".txt", "w")
    f.write(str(cost))
    f.close()
    # save screen to output file
    pygame.image.save(screen, "./output/" + file_path + ".jpg")


def main():
    # read input file and draw maze

    maze, bonus = read_maze("./input/" + sys.argv[1] + "/input" + sys.argv[2] + ".txt")
    draw_maze(maze)
    # create output folder
    os.makedirs("./output/" + sys.argv[1] + "/input" + sys.argv[2], exist_ok=True)
    file_path = sys.argv[1] + "/input" + sys.argv[2] + "/" + sys.argv[3]

    # find the path from start to goal, draw and export output files
    if sys.argv[3] == "gbfs":
        came_from = gbfs(maze, 1)
        draw_path(maze, came_from)
        export_output(maze, came_from, file_path + "_h1")
        draw_maze(maze)
        came_from = gbfs(maze, 2)
    elif sys.argv[3] == "astar":
        came_from = astar(maze, 1)
        draw_path(maze, came_from)
        export_output(maze, came_from, file_path + "_h1")
        draw_maze(maze)
        came_from = astar(maze, 2)
    elif sys.argv[3] == "dfs":
        came_from = dfs(maze)
    elif sys.argv[3] == "ucs":
        came_from = ucs(maze)
    elif sys.argv[3] == "astar2":
        came_from, cost = astar2(maze, bonus)

    draw_path1(maze, came_from, bonus)
    if sys.argv[3] == "gbfs" or sys.argv[3] == "astar":
        export_output(maze, came_from, file_path + "_h2")
    elif sys.argv[3] == "astar2":
        export_output1(file_path, cost)
    else:
        export_output(maze, came_from, file_path)
    pygame.display.update()
    pygame.time.wait(2000)  


    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()