import numpy as np

def check_obstacle_space(x, y, clearance=0):
    try:
        width = [0+clearance, 400-clearance]
        height = [0+clearance, 250-clearance]
        p1 = [[105, 100], [36, 185], [115, 210], [80, 180], [105, 100]]
        pm1 = [((p1[2][1] - p1[1][1])/(p1[2][0] - p1[1][0])),((p1[3][1] - p1[2][1]) / (p1[3][0] - p1[2][0])),((p1[1][1] - p1[4][1]) / (p1[1][0] - p1[4][0])),((p1[4][1] - p1[3][1]) / (p1[4][0] - p1[3][0])),((p1[3][1] - p1[1][1]) / (p1[3][0] - p1[1][0]))]

        p2 = [[165, 120], [200, 140], [235, 120], [
            235, 80], [200, 60], [165, 80], [165, 120]]
        pm2 = (p2[2][1] - p2[6][1]) / (p2[5][1] - p2[6][0])

        if (x < width[0]) or (x > width[1]) or (y < height[0]) or (y > height[1]):
            #print('\n Not within map coordinates \n')
            return True

        # Circle
        if ((x - 300) ** 2 + (y - 185) ** 2 - (40+clearance)**2) <= 0:
            #print('\n Inside Circle \n')
            return True

        # Convex Polygon
        elif (((pm1[0] * x - pm1[0]*p1[2][0] + p1[2][1] + clearance - y) >= 0) and ((pm1[1] * x - pm1[1]*p1[2][0] + p1[2][1] - clearance - y) <= 0) and ((pm1[4] * x - pm1[4] * p1[3][0] + p1[3][1] - y) <= 0)) \
                or (((pm1[2] * x - pm1[2]*p1[4][0] + p1[4][1] - clearance - y) <= 0) and ((pm1[3] * x - pm1[3]*p1[4][0] + p1[4][1] + clearance - y) >= 0) and not((pm1[4] * x - pm1[4] * p1[3][0] + p1[3][1] - y) <= 0)):
            
            #print('\n Inside Convex Polygon \n')
            return True

        # Hexagon
        elif (((-pm2 * x - (-pm2 * p2[4][0]) + p2[4][1] - clearance - y) <= 0) and
              ((pm2 * x - (pm2 * p2[4][0]) + p2[4][1] - clearance - y) <= 0) and
              ((-pm2 * x - (-pm2 * p2[1][0]) + p2[1][1] + clearance - y) >= 0) and
              ((pm2 * x - (pm2 * p2[1][0]) + p2[1][1] + clearance - y) >= 0) and
              ((p2[2][0] + clearance - x) >= 0) and
                ((p2[6][0] - clearance - x) <= 0)):
            #print('\n Inside Hexagon \n')
            return True

        else:
            return False
    except:
        print(' Input is Invalid. Try Again. Press Ctrl+C to exit.\n')
        return True


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
image = np.zeros((height, width, 3), np.uint8)
image.fill(255)

poly_color = [195,89,143]
blue = [0, 0, 255]
border_c = [64, 106, 151]


def CreateObstacleMatrix(clearance):
    obstacles = []
    map_ = np.zeros((400, 250), np.uint8)
    #map_ = np.zeros((250, 400), np.uint8)
    for x in range(map_.shape[0]):
        for y in range(map_.shape[1]):
            if check_obstacle_space(x,y,clearance):
                obstacles.append([x,y])
                map_[x,y] = 255
    img = cv2.flip(map_, 0)
    return img, obstacles
