import numpy as np
import cv2
import math
import heapq
from skimage.draw import line_aa

class PQueue:
    def __init__(self):
        self.node = []
    def insert_(self, val, cost):
        heapq.heappush(self.node, (cost, val))
    def queue_pop(self):
        return heapq.heappop(self.node)[1]
    def empty_(self):
        return not self.node


def Backtrack(prev, curr, next_):
    current = next_
    nodes = []
    while current != curr:
        nodes.append(current)
        current = prev[current]
    nodes.append(curr)
    nodes.reverse()
    return nodes

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

def DrawMap(points, path):
    global image
    image = cv2.flip(image, 0)
    image = cv2.flip(image, 1)
    nppolygon1_points = np.array(polygon1_points,np.int32)
    nppolygon1_points = nppolygon1_points.reshape(-1,1,2)
    image = cv2.fillPoly(image, [nppolygon1_points], poly_color)

    # Plot start goal points
    image[start_pos[0], start_pos[1]] = [0, 0, 255]

    cv2.putText(image, 'Start', (start_pos[0], start_pos[1]),
                 cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1, cv2.LINE_AA)
    image[goal_pos[0], goal_pos[1]] = [0, 0, 255]
    cv2.putText(image, 'Goal', (goal_pos[0], goal_pos[1]),
                 cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1, cv2.LINE_AA)

    # Circle
    image = cv2.circle(image, (300, 185), 40, poly_color, -1)

    # Hexagon
    nppolygon1_points = np.array(polygon2_points,np.int32)
    image = cv2.fillPoly(image, [nppolygon1_points], poly_color)



    image = cv2.circle(image, (300, 185), 45, border_c, 1)

    # Plot point space : EXPLORED
    image_ =image.copy()
    prev = (int(points[0][1]),int(points[0][0]))
    counter = 0
    fivep=[]
    for vis in points:
        counter+=1
        # cv2.arrowedLine(image_, (int(vis[0]), int(vis[1])),(int(path[i + 1][0]), 300 - int(path[i + 1][1])), (0, 0, 255), 1,
        #                 cv2.LINE_AA)
        x=int(vis[1])
        y=int(vis[0])
        fivep.append((x,y))
        # x =  (int)(vis[1] + 2 * math.sin(vis[2]))
        # y =  (int)(vis[0] + 2 * math.cos(vis[2]))
        #try:
        #cv2.arrowedLine(image_,(start_pos[1],start_pos[0]),(int(vis[1]),int(vis[0])) , (255,255,0),1,cv2.LINE_AA,0,2)
        print(x,' : ',y,'\n')
        #image_ = cv2.line(image_,prev,(x,y) , (255,255,0),1)

        rows, cols, weights = line_aa(x, y, prev[0],prev[1])    # antialias line

        w = weights.reshape([-1, 1])            # reshape anti-alias weights
        lineColorRgb = [255,255,0]           # color of line, orange here

        image_[rows, cols, 0:3] = (
          np.multiply((1 - w) * np.ones([1, 3]),image_[rows, cols, 0:3]) +
          w * np.array([lineColorRgb])
        )
        image_[x,y, :] = list(np.random.choice(range(256), size=3))


        if counter%5==0:
            prev = (fivep[2][0],fivep[2][1])
            fivep = []
        # except:
        # #     pass
        # img = image_.copy()
        # img = cv2.flip(img, -1)
        img = cv2.flip(image_, 0)
        cv2.imshow("Moving", img)
        #image = img
        cv2.waitKey(1)
    cv2.waitKey(0)

    # Plot PATH
    for p in path:
        image = cv2.circle(image, (int(p[0]), int(p[1])), 2, blue, 1)        
 

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

def GetMoves(theta, step):
    # round(number * 2) / 2  To queue_pop closest 0.5
    move1 = (step * (math.cos(math.radians(theta + (2 * 30)))),step * (math.sin(math.radians(theta + (2 * 30)))))
    move1 = ( (round(move1[0]*2)/2) , (round(move1[1]*2)/2) )

    move2 = (step * (math.cos(math.radians(theta + 30))),step * (math.sin(math.radians(theta + 30))))
    move2 = ( (round(move2[0]*2)/2) , (round(move2[1]*2)/2) )

    move3 = (step * (math.cos(math.radians(theta))),step * (math.sin(math.radians(theta))))
    move3 = ( (round(move3[0]*2)/2) , (round(move3[1]*2)/2) )

    move4 = (step * (math.cos(math.radians(theta - (2 * 30)))),step * (math.sin(math.radians(theta - (2 * 30)))))
    move4 = ( (round(move4[0]*2)/2) , (round(move4[1]*2)/2) )

    move5 = (step * (math.cos(math.radians(theta - 30))),step * (math.sin(math.radians(theta - 30))))
    move5 = ( (round(move5[0]*2)/2) , (round(move5[1]*2)/2) )

    return{"move1": (move1[0], move1[1], 60),"move2": (move2[0], move2[1], 30),"move3": (move3[0], move3[1], 0),"move4": (move4[0], move4[1], -30),"move5": (move5[0], move5[1], -60)}


def CorrectAngle(angle,offset):
    if offset+angle >= 360:
        angle -= 360
    if offset+angle < 0:
        angle = 360 + angle
    return angle

m = ["move1","move2","move3","move4","move5"]
def AStar(start, goal, step):
    map, o_list = CreateObstacleMatrix(clearance)
    pq = PQueue()
    pq.insert_(start, 0)
    prev_ = {start: None}
    visited = {start: 0}
    coordinates = list()
    explore = []
    path = []
    while not pq.empty_():
        current_cell = pq.queue_pop()
        if ((current_cell[0] - goal[0]) ** 2) + ((current_cell[1] - goal[1]) ** 2) - (1.5 * step) ** 2 <= 0:
            path_list = Backtrack(prev_, start, current_cell)
            break
        moves = GetMoves(current_cell[2], step)        
        for i in m:
            row_offset, col_offset, orientation = moves[i]
            orientation = CorrectAngle(orientation,current_cell[2])
            next = (current_cell[0] + row_offset, current_cell[1] + col_offset, current_cell[2] + orientation)
            if  not ([int(next[0]),int(next[1])] in o_list) and next not in visited:
                visited[next] = visited[current_cell] + 1
                f_value = visited[current_cell] + 1 + abs(math.sqrt((goal[0] - next[0]) ** 2 + (goal[1] - next[1]) ** 2))

                pq.insert_(next, f_value)
                prev_[next] = current_cell
                print("Explored:", next)
                coordinates.append(next)
                explore.append([next[0],next[1],next[2]])

    for point in path_list:
        path.append([point[0],point[1]])

    DrawMap(explore,path)
    return None


# take_input = False
# robot_radius, start_pos, goal_pos, clearance,step = 2,(6,6,30),(150,200,30),5,8
# if take_input:
robot_radius, start_pos, goal_pos, clearance,step = GetInput()
map_, obstacles = CreateObstacleMatrix(clearance)
AStar(start_pos, goal_pos, step)
image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
image = cv2.flip(image, -1)
image = cv2.flip(image, 0)
cv2.imshow('Path', image)
cv2.waitKey(0)
cv2.imwrite("Output.png", image)