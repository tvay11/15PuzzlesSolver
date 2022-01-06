import math

from SlidingTilePuzzle import *
from SlidingTileSolver import *
from PriorityQueue import *

from math import *


class MySolver(SlidingTileSolver):
    def heurstic(self, state):
        lll = sqrt(len(state))
        manhatten = 0
        for i, item in enumerate(state):
            if item != 0:
                prow, pcol = int(i / lll), i % lll
                grow, gcol = int(item / lll), item % lll
                manhatten += abs(prow - grow) + abs(pcol - gcol)
        return manhatten

    def __init__(self, problem, maxTime):
        SlidingTileSolver.__init__(self, problem, maxTime)


    # You need to redefine this function for your algorithm
    # It is currently using breadth-first search which is very slow
    def solve(self):
        frontier = PriorityQueue()
        frontier.push(0, (0, self._problem.getInitial()))
        seen = set()
        parent = dict()

        # Do not remove the timeRemaining check from the while loop
        while len(frontier) > 0 and self.timeRemaining():
            self._numExpansions += 1

            priority, (depth, currentState) = frontier.pop()
            seen.add(currentState)
            for action in self._problem.actions(currentState):
                resultingState = self._problem.result(currentState, action)

                if self._problem.isGoal(resultingState):
                    # Goal reached
                    parent[resultingState] = (currentState, action)
                    path = ""
                    current = resultingState
                    while current != self._problem.getInitial():
                        (current, action) = parent[current]
                        path = action + path
                    return path

                if resultingState not in seen:

                    g = self.heurstic(resultingState)
                    #print("priotity: %d, depth: %d"%(g+depth,depth))
                    frontier.push(g + depth, (depth + 1, resultingState))
                    seen.add(resultingState)
                    parent[resultingState] = (currentState, action)


        return []

