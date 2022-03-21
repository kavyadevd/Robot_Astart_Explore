import numpy as np

def GetInput():
    flag = True
    robot_radius = 1
    clearance = 5
    start_pos = []
    goal_pos = []
    theta_start = 0
    theta_goal = 0
    clearance = float(input('Enter map clearance: '))
    while flag:
        x = float(input('Enter Start x coordinate: '))
        y = float(input('Enter Start y coordinate: '))
        if check_obstacle_space(x, y, clearance):
            print('Invalid start position. Try again.')
        else:
            flag = False
            theta_start = float(input('Enter theta start: '))
            start_pos = (x,y,theta_start)
    flag = True
    while flag:
        x = float(input('Enter Goal x coordinate: '))
        y = float(input('Enter Goal y coordinate: '))
        if check_obstacle_space(x, y, clearance):
            print('Invalid goal position. Try again.')
        else:
            flag = False
            theta_goal = float(input('Enter theta goal: '))
            goal_pos = (x,y,theta_goal)
    robot_radius = float(input('Enter map robot radius: '))
    step = float(input('Enter map step size : '))
    return robot_radius, start_pos, goal_pos, clearance,step


polygon1_points = [[115, 210], [80, 180], [105, 100], [36, 185]]
polygon2_points = [[200, 140], [235, 120], [235, 80], [200, 60], [165, 80], [165, 120]]

screen_size = width, height = 400, 250
