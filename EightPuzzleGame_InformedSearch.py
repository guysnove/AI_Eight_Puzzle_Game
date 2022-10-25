#!/usr/bin/env python
# coding: utf-8

# In[9]:


import numpy as np
from EightPuzzleGame_State import State
import time
'''
This class implement the Best-First-Search (BFS) algorithm along with the Heuristic search strategies

In this algorithm, an Open list is used to store the unexplored states and 
a Closed list is used to store the visited state. Open list is a priority queue (First-In-First-Out). 
The priority is insured through sorting the Open list each time after new states are generated 
and added into the list. The heuristics are used to decide which node should be visited next.

In this informed search, reducing the state space search complexity is the main criterion. 
We define heuristic evaluations to reduce the states that need to be checked every iteration. 
Evaluation function is used to express the quality of informedness of a heuristic algorithm. 

'''

class InformedSearchSolver:
    current = State()
    goal = State()
    openlist = []
    closed = []
    depth = 0

    def __init__(self, current, goal):
        self.current = current
        self.goal = goal
        self.openlist.append(current)

    def sortFun(self, e):
        return e.weight


    def check_inclusive(self, s):
        """
         * The check_inclusive function is designed to check if the expanded state is in open list or closed list
         * This is done to prevent looping. (You can use a similar code from uninformedsearch program)
         * @param s
         * @return
        """
        in_open = 0
        in_closed = 0
        ret = [-1, -1]

        # TODO your code start here
        # exist_Index : This variable is used to track the index that is already in the open/closed list
        exist_Index = None
        
        # A verification to make sure that the argument that is passed to the check_inclusive method is a State object
        if not isinstance(s, State):
            raise TypeError
        # A loop to verify to see if the given state is already in the open list
        for child in self.openlist:
            if isinstance(child, State):
                if s.equals(child):
                    exist_Index = self.openlist.index(child)
                    in_open = 1
                    break
        # A loop to see if the given state is already in the closed list
        for child in self.closed:
            if isinstance(child, State):
                if s.equals(child):
                    exist_Index = self.closed.index(child)
                    in_closed = 1
                    break

        # the child is not in open or closed
        if in_open == 0 and in_closed == 0:
            ret[0] = 1
            ret[1] = exist_Index
            return ret
        # the child is already in open
        if in_open == 1:
            ret[0] = 2
            ret[1] = exist_Index
            return ret
        # the child is already in closed
        if in_closed == 1:
            ret[0] = 3
            ret[1] = exist_Index
            return ret



        # TODO your code start here


    def state_walk(self):
        """
        * The following state_walk function is designed to move the blank tile --> perform actions
        * There are four types of possible actions/walks of for the blank tile, i.e.,
        *  ↑ ↓ ← → (move up, move down, move left, move right)
        * Note that in this framework the blank tile is represent by '0'
        """

        # add closed state
        self.closed.append(self.current)
        self.openlist.remove(self.current)
        # move to the next heuristic state
        walk_state = self.current.tile_seq
        row = 0
        col = 0

        for i in range(len(walk_state)):
            for j in range(len(walk_state[i])):
                if walk_state[i, j] == 0:
                    row = i
                    col = j
                    break

        self.depth += 1

        ''' The following program is used to do the state space actions
         The 4 conditions for moving the tiles all use similar logic, they only differ in the location of the 
         tile that needs to be swapped. That being the case, I will only comment the first subroutine'''
        # TODO your code start here
    
        available_moves = []
        
        ### ↑(move up) action ###
        if (row - 1) >= 0:
            available_moves.append([row - 1, col])
            """
             *get the 2d array of current 
             *define a temp 2d array and loop over current.tile_seq
             *pass the value from current.tile_seq to temp array
             *↑ is correspond to (row, col) and (row-1, col)
             *exchange these two tiles of temp
             *define a new temp state via temp array
             *call check_inclusive(temp state)
             *do the next steps according to flag
             *if flag = 1 //not in open and closed
             *begin
             *assign the child a heuristic value via heuristic_test(temp state);
             *add the child to open
             *end;
             *if flag = 2 //in the open list
             *if the child was reached by a shorter path
             *then give the state on open the shorter path
             *if flag = 3 //in the closed list
             *if the child was reached by a shorter path then
             *begin
             *remove the state from closed;
             *add the child to open
             *end;
            """


        ### ↓(move down) action ###
        if (row + 1) < 3:
            available_moves.append([row + 1, col])



        ### ←(move left) action ###
        if (col - 1) > -1:
            available_moves.append([row, col - 1])




        ### →(move right) action ###
        if (col + 1) < 3:
            available_moves.append([row, col + 1])
        
        #the folowing children list is used to store all the child states generated from performing all the available moves on the current state
        
        children = []
        for move in available_moves:
            move_tiles = walk_state.copy()
            temp_val = move_tiles[row][col]
            move_tiles[row][col] = move_tiles[move[0]][move[1]]
            move_tiles[move[0]][move[1]] = temp_val
            children.append(State(move_tiles.copy(), depth=self.current.depth + 1))
        
        for child in children:
            """
            We call check_inclusive function and do the next steps according to flag
            
            """
            flag = self.check_inclusive(child)
            
            if flag[0] == 1:
                self.heuristic_test(child)
                self.openlist.append(child)
            if flag == 2:
                open_child = self.openlist[flag[1]]
                if child.depth < open_child.depth:
                    open_child.depth = child.depth
            
            if flag[0] == 3:
                closed_child = self.closed[flag[1]]
                if child.depth < closed_child.depth:
                    self.closed.remove(closed_child)
                    self.openlist.append(closed_child)
                



        # sort the open list first by h(n) then g(n)
        self.openlist.sort(key=lambda x: x.weight)
        self.openlist.sort(key=lambda x: x.depth)
       
        # Set the next current state
        
        self.current = self.openlist[0]

        #TODO your code end here




    def heuristic_test(self, current):
        """
        * Solve the game using heuristic search strategies

        * There are three types of heuristic rules:
        * (1) Tiles out of place
        * (2) Sum of distances out of place
        * (3) 2 x the number of direct tile reversals

        * evaluation function
        * f(n) = g(n) + h(n)
        * g(n) = depth of path length to start state
        * h(n) = (1) + (2) + (3)
        """
        if not isinstance(current, State):
            raise TypeError
        if not isinstance(self.goal, State):
            raise TypeError

        curr_seq = current.tile_seq
        goal_seq = self.goal.tile_seq

        # (1) Tiles out of place
        h1 = 0
        #TODO your code start here
        """
         *loop over the curr_seq
         *check the every entry in curr_seq with goal_seq
        """
        #TODO your code end here
        if not isinstance(curr_seq, np.ndarray):
            raise TypeError
        num_misplaced = 0
        for i in range(len(curr_seq)):
            for j in range(len(curr_seq[i])):
                if curr_seq[i][j] != curr_seq[i][j]:
                    num_misplaced += 1

        h1 = num_misplaced
        

        # (2) Sum of distances out of place
        h2 = 0
        #TODO your code start here
        """
         *loop over the goal_seq and curr_seq in nested way
         *locate the entry which has the same value in 
         *curr_seq and goal_seq then calculate the offset
         *through the absolute value of two differences
         *of curr_row-goal_row and curr_col-goal_col
         *absoulte value can be calculated by abs(...)
        """
        #TODO your code end here
        
        def dist(search_val):
            for gRow in range(len(goal_seq)):
                for gCol in range(len(goal_seq[gRow])):
                    if goal_seq[gRow][gCol] == search_val:
                        return gRow, gCol
        
        answer_2darr = np.ndarray(shape=(3, 3), dtype=int)
        for curRow in range(len(curr_seq)):
            for curCol in range(len(curr_seq[curRow])):
                search_value = curr_seq[curRow][curCol]
                goalRow, goalCol = dist(search_value)
                rowDist = goalRow - curRow
                colDist = goalCol - curCol
                if rowDist < 0:
                    rowDist = rowDist * -1
                if colDist < 0:
                    colDist = colDist * -1
                answer_2darr[curRow][curCol] = rowDist + colDist
        
        h2 = answer_2darr.sum()

        
        # (3) 2 x the number of direct tile reversals
        h3 = 0
        #TODO your code start here
        """
         *loop over the curr_seq
         *use a Γ(gamma)shap slider to walk throught curr_seq and goal_seq
         *rule out the entry with value 0
         *set the boundry restriction
         *don't forget to time 2 at last
         *for example 
         *goal_seq  1 2 3   curr_seq  2 1 3 the Γ shape starts 
         *       4 5 6          4 5 6
         *       7 8 0          7 8 0
         *with 1 2 in goal_seq and 2 1 in curr_seq thus the 
         *    4             4
         *reversal is 1 2 and 2 1
        """
        
        # curr_seq and goal_seq are flattened from 2d arrays into 1d arrays
        curr_seq = curr_seq.flatten()
        goal_seq = goal_seq.flatten()
        
        for index in range(0, len(curr_seq) - 1):
            curr_element = goal_seq[index]
            next_element = goal_seq[index + 1]
            if curr_element == 0 or next_element == 0:
                continue
            index_of_curr = np.where(curr_seq == curr_element)[0][0]
            index_of_next = np.where(curr_seq == next_element)[0][0]
            
            if index_of_next - index_of_curr == -1:
                h3 += 1


        # update the heuristic value for current state
        
        current.weight = h1 + h2 + (2 * h3)

        #TODO your code end here




    # You can change the following code to print all the states on the search path
    def run(self):
        # output the goal state
        target = self.goal.tile_seq
        print("\nReached goal state: ")
        target_str = np.array2string(target, precision=2, separator=' ')
        print(target_str[1:-1])

        print("\n The visited states are: ")
        path = 0
        # record the time the search begins
        t1 = time.time()
        while not self.current.equals(self.goal):
            self.state_walk()
            print('Visited State number ', path + 1)
            pathstate_str = np.array2string(self.current.tile_seq, precision=2, separator=' ')
            print(pathstate_str[1:-1])
            path += 1
        # record the time the search ends
        t2 = time.time()
        
        # Difference of the end time and the start time.
        print(f"Time taken in seconds: {t2-t1:.10f}")

        print("\nIt took ", path, " iterations to reach to the goal state")
        print("The length of the path is: ", self.current.depth)


# In[ ]:




