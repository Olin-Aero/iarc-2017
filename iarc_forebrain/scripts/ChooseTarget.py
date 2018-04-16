import tf
import tf.transformations
import numpy as np
import math

def orientationToHeading(orientation):
    """
    Converts a quaterion in the form of a pose orientation into a heading.
    :param orientation:
    :return:
    """
    res = [0, 0, 0, 0]
    res[0] = orientation.x
    res[1] = orientation.y
    res[2] = orientation.z
    res[3] = orientation.w
    return tf.transformations.euler_from_quaternion(res)[2]
def goodnessScore(roombas, obstacles, targeting, C1=3, C2=1, C3=1, C4=1, C5=2):
    """
    Determines which Roomba we pick to lead to the goal.
    Higher score is better.

    Returns: [(Roomba, Score)]
    :rtype list[tuple[Roomba, float]]
    """

    def headingScore(roomba):
        # print(roomba.visible_location.pose.pose.orientation)
        heading = orientationToHeading(roomba.visible_location.pose.pose.orientation)

        return abs(np.sin(heading))

    def positionScore(roomba):
        # the closer to the center, the lower the xScore, from 0 to 1
        posX = roomba.visible_location.pose.pose.position.x
        posY = roomba.visible_location.pose.pose.position.y
        if(abs(posX) > 9.95 or abs(posY) > 9.95):
            return -10000
        xScore = abs(posX) / 10
        # the closer to the green zone, the higher the yScore, from 0 to 1
        yScore = posY / 20 + 0.5
        return (xScore + yScore)/2

    def distanceFromObstaclesScore(roomba, obstacles):
        """
        (-infinity, 0)
        """

        score = 0
        MIN_OBSTACLE_DISTANCE = 0.5
        for obstacle in obstacles:
            x = roomba.visible_location.pose.pose.position.x - obstacle.visible_location.pose.pose.position.x
            y = roomba.visible_location.pose.pose.position.y - obstacle.visible_location.pose.pose.position.y
            dist = np.sqrt(x ** 2 + y ** 2)

            if dist < MIN_OBSTACLE_DISTANCE:
                return -10000

            score -= 1 / dist ** 2

        return score

    def stateQualtityScore(roomba):
        """
        How precisely we know the Roombas' state.
        Compare position accuracy to view radius to know if it's possible
        to see the given roomba when drone arrives.
        """
        return 0

    def sameRoomba(roomba, targeting):
        if(targeting != None and roomba.frame_id == targeting.frame_id):
            return 1
        else:
            return 0

    result = []

    for i in xrange(0, len(roombas)):
        roomba = roombas[i]

        score = C1 * headingScore(roomba) + \
                C2 * positionScore(roomba) + \
                C3 * distanceFromObstaclesScore(roomba, obstacles) + \
                C4 * stateQualtityScore(roomba) + \
                C5 * sameRoomba(roomba, targeting)
        result.append((roomba, score))

    return result

def targetSelect(roombaScore):
    # print(roombaScore)
    # print(roombaScore != [])
    if roombaScore != []:
        # return [0,Drone()]
        # print(max(roombaScore, key=lambda x: x[1]))
        return max(roombaScore, key=lambda x: x[1])
    else:
        return [None,0]
        res = Drone()
        # print("Garb")
        res.tag = ''
        return [res, 0]
