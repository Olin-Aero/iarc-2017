#!/usr/bin/env python2

from transitions import Machine

states = [
    'init',
    'search',
    'follow',
    'waitForValidRoomba',  # If a roomba is not availible to contact
    'avoidObstacle',
    'landInFront',
    'pushButton',
    'waitForGoodRoomba']

transitions = [
    {'trigger': 'start', 'source': 'init', 'dest': 'search'},
    {'trigger': 'goodRoombaFound', 'source': 'search', 'dest': 'follow'},
    {'trigger': 'timeInvalid', 'source': 'follow', 'dest': 'waitForValidRoomba'},
    # timeInvalid signals that it's not the right time to follow the roomba.

    # Null actions result in returning to search algorithm
    {'trigger': 'null', 'source': 'follow', 'dest': 'search'},
    {'trigger': 'null', 'source': 'waitForValidRoomba', 'dest': 'search'},
    {'trigger': 'null', 'source': 'waitForGoodRoomba', 'dest': 'search'},
    {'trigger': 'null', 'source': 'pushButton', 'dest': 'search'},
    {'trigger': 'null', 'source': 'landInFront', 'dest': 'search'},

    # Action triggers
    {'trigger': '180Needed', 'source': 'follow', 'dest': 'landInFront'},
    {'trigger': '45Needed', 'source': 'follow', 'dest': 'pushButton'},
    {'trigger': 'timeElapsedNeeded', 'source': 'follow', 'dest': 'waitForGoodRoomba'},

    {'trigger': 'actionCompleted', 'source': 'landInFront', 'dest': 'follow'},
    {'trigger': 'actionCompleted', 'source': 'pushButton', 'dest': 'follow'},
    {'trigger': 'actionCompleted', 'source': 'waitForGoodRoomba', 'dest': 'follow'},

    # Obstacle triggers
    {'trigger': 'obstacleInPath', 'source': 'search', 'dest': 'avoidObstacle'},
    {'trigger': 'obstacleInPath', 'source': 'follow', 'dest': 'avoidObstacle'},
    {'trigger': 'obstacleInPath', 'source': 'waitForValidRoomba', 'dest': 'avoidObstacle'},
]

machine = Machine(model=drone, states=states, transitions=transitions, initial='init')

drone.get_graph().draw('state_diagram.png', prog='dot')

drone.start()
print(drone.state)

print(cfg.ROOMBA_HEIGHT)
drone.goodRoombaFound()

print(drone.state)
