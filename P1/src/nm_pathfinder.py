# To Run: $ python nm_interactive.py ../input/homer.png ../input/homer.png.mesh.pickle <sampling_factor(number)>
from collections import deque
from queue import PriorityQueue
import math
# helper function to find the box that contains a point
def contains_point(point, allBoxes):
    x = point[0]
    y = point[1]
    for box in allBoxes:
        if box[0]<=x<=box[1]:
            if box[2]<=y<=box[3]:
                return box
    return False

#given the cameFrom dictionary it adds to 'path' the path from goal back to start
# creating a path of boxes
def create_box_path(path, start, goal, cameFrom):
    current = goal
    while current!=start:
        path.append(current)
        current = cameFrom[current]
    path.reverse()
    #remove the destination box from the path because we just want to add the destination point not the box itself
    return


def create_point_path(path, pathOfBoxes, source_point, destination_point, detail_points):
    #add starting point to the path
    path.append(source_point)
    #add the closest point from the current point to the box into the path and detail points
    for box in pathOfBoxes:
        path.append(detail_points[box])
    #add the destination point to the path
    path.append(destination_point)

#finds the center point of the box
def center(box):
    return ((box[0] + box[1]) / 2, (box[2] + box[3]) / 2)

# returns a point closes to the edge of the next box
def constrain_point(point, box):
    x = None
    y = None
    #find the closest spot on the box to the point
    if point[0]<=box[0]:
        x = box[0]
    elif point[0]>=box[1]:
        x = box[1]
    else:
        x = point[0]
    if point[1]<=box[2]:
        y = box[2]
    elif point[1]>=box[3]:
        y = box[3]
    else:
        y = point[1]
    return (x, y)

#finds the euclidean distance between 2 points
def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def heuristic(point1, point2):
    x1,y1 = point1
    x2, y2 = point2
    return (abs(x1-x2)+abs(y1-y2))

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = {}

    allBoxes = mesh['boxes']
    adjLists = mesh['adj']
 
    # workflow #1 find the source and destination boxes --------------------------------------
    '''
    box = contains_point(source_point, allBoxes)
    if box:
        print("source point: ", source_point, "apears in box: ", box)
    box = contains_point(destination_point, allBoxes)
    if box:
        print("destination point: ", destination_point, "apears in box: ", box)
    '''
    #---------------------------------------------------------------------------------------- 


    # workflow #2 implement simplest complete search algorithm-------------------------------
    
    # #BFS
    # startBox = contains_point(source_point, allBoxes)
    # endBox = contains_point(destination_point, allBoxes)
    # #incase the source point and destination point are in the same box
    # if startBox==endBox:
    #     path.append(source_point)
    #     path.append(destination_point)
    #     return path, boxes.keys()
    # queue = deque()
    # #add starting box to queue
    # queue.append(startBox)
    # cameFrom = dict()
    # cameFrom[startBox] = None
    
    # #table for points within boxes from workflow #3
    # detail_points = {startBox: source_point}

    # # keep track if something was found
    # found = False

    # while len(queue)>0:
    #     # get current point
    #     currentBox = queue.popleft()
    #     # if we didnt get something from the queue break
    #     if not currentBox:
    #         break
    #     # if we have reached the final box break
    #     if currentBox == endBox:
    #         found = True
    #         break
    #     #find all neighbors of square and add to queue
    #     for neighbor in adjLists[currentBox]:
    #         if neighbor not in cameFrom:
    #             queue.append(neighbor)
    #             cameFrom[neighbor] = currentBox
    # # if we didnt find a path print to console and return
    # if not found:
    #     print("No path!")
    #     return path, boxes.keys()
    # #get the box path to destination
    # pathOfBoxes = []
    # create_box_path(pathOfBoxes, startBox, endBox, cameFrom)
    # #make a path of points from that box path
    # create_point_path(path, pathOfBoxes, source_point, destination_point, detail_points) 
    # #add the final box to the detail_points
    # detail_points[endBox] = destination_point

    #------------------------------------------------------------------------------------------

    # workflow 3 office hours nnotes
    '''
    when I do the max and min of the two boxes i need to compare the current point to these maxes and mins in order to find what the next detail point should be
    detail points should be the closest reachable point from the current point to the next box
    '''

    # workflow #4 Modify the supplied Dijkstra's implementation into an A* implementation. -----
    
    # Dijkstra's Alg: used some ideas from Amit Patel's website "Introduction to A*" https://www.redblobgames.com/pathfinding/a-star/introduction.html
    # startBox = contains_point(source_point, allBoxes)
    # endBox = contains_point(destination_point, allBoxes)
    # #incase the source point and destination point are in the same box
    # if not startBox or not endBox:
    #     print("No path!")
    #     return path, boxes.keys()

    # frontier = PriorityQueue()
    # #save points too for the boxes 
    # frontier.put((0, startBox, source_point))
    # cameFrom = dict()
    # costSoFar = dict()
    # cameFrom[startBox] = None
    # costSoFar[startBox] = 0
    # found = False
    # detail_points = {startBox: source_point}

    # while not frontier.empty():
    #     #get the current point so that we can use it for euclidian distance
    #     currentPriority, currentBox, currentPoint = frontier.get()
    #     boxes[currentBox] = None

    #     if not currentBox:
    #         break

    #     if currentBox == endBox:
    #         found = True
    #         break
        
    #     for neighbor in adjLists[currentBox]:
    #         #add the euclidean distance from current point to the next point
    #         nextPoint = constrain_point(currentPoint, neighbor)
    #         newCost = costSoFar[currentBox] + euclidean_distance(currentPoint, nextPoint)
    #         if neighbor not in costSoFar or newCost < costSoFar[neighbor]:
    #             costSoFar[neighbor] = newCost
    #             # dont know if we should use euclidian distance or the heuristic
    #             priority = newCost + euclidean_distance(destination_point, nextPoint)
    #             frontier.put((priority, neighbor, nextPoint))
    #             cameFrom[neighbor] = currentBox
    #             detail_points[neighbor] = nextPoint
    
    # if not found:
    #     print("No path!")
    #     return path, boxes.keys()
    
    # pathOfBoxes = []
    # create_box_path(pathOfBoxes, startBox, endBox, cameFrom)
    # create_point_path(path, pathOfBoxes, source_point, destination_point, detail_points)
    # detail_points[endBox] = destination_point
    
    #------------------------------------------------------------------------------------------

    # workflow #5 modify A* to be bidirectional --------------------------------------------------
    startBox = contains_point(source_point, allBoxes)
    endBox = contains_point(destination_point, allBoxes)
    #incase the source point and destination point are in the same box
    if not startBox or not endBox:
        print("No path!")
        return path, boxes.keys()

    # keep track of one frontier
    frontier = PriorityQueue()
    #save points  for forward/backward frontier searches
    frontier.put((0, startBox, 'forward'))
    frontier.put((0, endBox, 'backward'))
    # keep track of boxes visited from both directions
    cameFromForward = {startBox: None}
    cameFromBackward = {endBox: None}
    # keep track of costs for both directions
    costSoFarForward = {startBox: 0}
    costSoFarBackward = {endBox: 0}
    # have detail_points for forwards and backwards direction
    detail_points_forward = {startBox: source_point}
    detail_points_backward = {endBox: destination_point}
    found = False
    meetingPoint = None

    while not frontier.empty():
        currentPriority, currentBox, direction = frontier.get()
        boxes[currentBox] = None

        # check if a box is found that was found by other direction
        if (direction == 'forward' and currentBox in cameFromBackward) or (direction== 'backward' and currentBox in cameFromForward):
            found = True
            meetingPoint = currentBox
            break

        # see which variables to use depending on direction
        cameFrom = cameFromForward if direction == 'forward' else cameFromBackward
        costSoFar = costSoFarForward if direction == 'forward' else costSoFarBackward
        detail_points = detail_points_forward if direction == 'forward' else detail_points_backward
        goal_point = destination_point if direction == 'forward' else source_point

        for neighbor in adjLists[currentBox]:
            currentPoint = detail_points[currentBox]
            nextPoint = constrain_point(currentPoint, neighbor)
            newCost = costSoFar[currentBox] + euclidean_distance(currentPoint, nextPoint)
            if neighbor not in costSoFar or newCost < costSoFar[neighbor]:
                costSoFar[neighbor] = newCost             
                priority = newCost + euclidean_distance(goal_point, nextPoint)
                frontier.put((priority, neighbor, direction))
                cameFrom[neighbor] = currentBox
                detail_points[neighbor] = nextPoint

    if not found:
        print("No path!")
        return path, boxes.keys()
        
    # final path reconstruction logic
    path = reconstruct_path(cameFromForward, cameFromBackward, meetingPoint, detail_points_forward, detail_points_backward, source_point, destination_point)
        
    return path, boxes.keys()

# creates the final path depending on the points used
def reconstruct_path(cameFromForward, cameFromBackward, meetingPoint, detail_points_forward, detail_points_backward, source_point, destination_point):
    path = []

    # create path from source point to meeting point
    current = meetingPoint
    while current:
        path.append(detail_points_forward.get(current, current))
        current = cameFromForward.get(current)
    path.reverse()

    # create path from meeting to destination point
    current = cameFromBackward.get(meetingPoint)
    while current:
        path.append(detail_points_backward.get(current, current))
        current = cameFromBackward.get(current)
    
    # add source/destination points
    if path[0] != source_point:
        path.insert(0, source_point)
    if path[-1] != destination_point:
        path.append(destination_point)

    return path