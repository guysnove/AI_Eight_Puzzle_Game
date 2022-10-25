'''
Solving Eight Puzzle Game using Search Strategies
The eight-puzzle game is a 3 × 3 version of the 15-puzzle in which eight tiles can be moved around in nine spaces.
CSC 425 Artificial Intelligence
Instructor: Dr. Yangyang Tao
Semester: Fall 2022

Your name:
'''

import numpy as np
from EightPuzzleGame_State import State
#from EightPuzzleGame_UinformedSearch import UninformedSearchSolver
from EightPuzzleGame_InformedSearch import InformedSearchSolver


class EightPuzzleGame:
    titles = 8
    def __init__(self, initial=[], goal=[], tiles=8):
        self.initial = initial
        self.goal = goal
        self.tiles = tiles

    def start(self):
        '''Please test your program with different start state to explore the feature of the algorithms'''
        # initialize the init state and goal state as 2d array
        #init_tile = np.array([[2, 3, 6], [1, 4, 8], [7, 5, 0]])
        init_tile = np.array([[1, 2, 3], [0, 4, 6], [7, 5, 8]])
        init = State(init_tile, 0, 0)
        print("\nStart state:")
        init_str = np.array2string(init_tile, precision=2, separator=' ')
        print(init_str[1:-1])


        goal_tile = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 0]])
        goal = State(goal_tile, 0, 0)

        print("\nExpected goal state: ")
        goal_str = np.array2string(goal_tile, precision = 2, separator=' ')
        print(goal_str[1:-1])


        self.tiles = 8

        #UIS_solver = UninformedSearchSolver(init, goal)
        #UIS_solver.run()

        IS_solver = InformedSearchSolver(init, goal)
        IS_solver.run()


# start the puzzle game
epp = EightPuzzleGame()
epp.start()

