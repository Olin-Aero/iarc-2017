#!/usr/bin/env python2
"""
Owner: Nathan Estill
"""

import matplotlib.pyplot as plt
import random
import math

sim_time = 1.3
pred_time = 20
num_roombas = 100
ROOMBA_OBSTACLE_NOISE_PERIOD = 1000
ROOMBA_LINEAR_SPEED = 0.33
ROOMBA_OBSTACLE_NOISE_MAX = 4 * (math.pi / 180)
ROOMBA_OBSTACLE_TURN_RADIUS = 3


def heatmapping(sim_time, pred_time, num_roombas):
    xArray = []
    yArray = []
    for i in range(num_roombas):
        current_time = sim_time
        xPosNew = 0
        yPosNew = 0
        newHeading = 0
        while (current_time < sim_time + pred_time):
            new_time = ROOMBA_OBSTACLE_NOISE_PERIOD / 1000 - sim_time % (ROOMBA_OBSTACLE_NOISE_PERIOD / 1000)
            if (current_time + new_time >= sim_time + pred_time):
                new_time = sim_time + pred_time - current_time
                print(new_time)
            xPos = xPosNew
            yPos = yPosNew
            heading = newHeading
            CircleRad = heading - math.pi / 2
            CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS * math.cos(CircleRad))
            CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS * math.sin(CircleRad))
            newCircleRad = CircleRad + new_time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS
            xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS * math.cos(newCircleRad))
            yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS * math.sin(newCircleRad))
            newHeading = newCircleRad + math.pi / 2
            heading = newHeading + random.uniform(-ROOMBA_OBSTACLE_NOISE_MAX, ROOMBA_OBSTACLE_NOISE_MAX)
            xPos = xPosNew
            yPos = yPosNew
            current_time += new_time
            new_time = sim_time % (ROOMBA_OBSTACLE_NOISE_PERIOD / 1000)
            if (current_time + new_time >= sim_time + pred_time):
                new_time = sim_time + pred_time - current_time
            CircleRad = heading - math.pi / 2
            CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS * math.cos(CircleRad))
            CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS * math.sin(CircleRad))
            newCircleRad = CircleRad + new_time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS
            xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS * math.cos(newCircleRad))
            yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS * math.sin(newCircleRad))
            newHeading = newCircleRad + math.pi / 2
            current_time += new_time
            xArray.append(xPosNew)
            yArray.append(yPosNew)
        plt.plot(xArray, yArray, 'ro')
        plt.plot(xPosNew, yPosNew, 'bo')
    plt.axis('equal')
    plt.show()


def test():
    print heatmapping(1.3, 20, 20)


if __name__ == '__main__':
    test()
