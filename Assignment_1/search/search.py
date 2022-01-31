# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
from typing import Dict, Any

import util
from game import Directions


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def get_directions_from_map(dir_map, goal, start):
    # print(dir_map)
    list_of_directions = []

    cur_node = goal
    while cur_node != start:
        list_of_directions.append(dir_map[cur_node][1])
        cur_node = dir_map[cur_node][0]
    return list_of_directions[::-1]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    visited = []
    dfs_stack = util.Stack()
    start = problem.getStartState()
    dfs_stack.push((start, []))

    while not dfs_stack.isEmpty():
        top, dirs = dfs_stack.pop()
        if top in visited:
            continue
        if problem.isGoalState(top):
            return dirs
        for successor in problem.getSuccessors(top):
            location = successor[0]
            direction = successor[1]
            if location in visited:
                continue
            dfs_stack.push((location, dirs + [direction]))
        visited.append(top)

    print("Path not found")


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    bfs_queue = util.Queue()
    bfs_queue.push((problem.getStartState(), []))
    visited = []
    while not bfs_queue.isEmpty():
        top, dirs = bfs_queue.pop()
        if top in visited:
            continue
        if problem.isGoalState(top):
            return dirs
        for successor in problem.getSuccessors(top):
            location = successor[0]
            direction = successor[1]
            if location in visited:
                continue
            bfs_queue.push((location, dirs + [direction]))
        visited.append(top)

    print("Path not found")


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    ucs_queue = util.PriorityQueue()
    start = (problem.getStartState(), [])
    ucs_queue.push(start, 0)
    visited = []
    while not ucs_queue.isEmpty():
        top_cost, top = ucs_queue.pop_with_priority()
        if top[0] in visited:
            continue
        if problem.isGoalState(top[0]):
            return top[1]
        for successor in problem.getSuccessors(top[0]):
            location = successor[0]
            direction = successor[1]
            cost = successor[2]
            if location not in visited:
                ucs_queue.update((location, top[1] + [direction]), top_cost + cost)
        visited.append(top[0])

    print("Path not found")

    # util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    aStarQueue = util.PriorityQueue()
    start = (problem.getStartState(), [])
    aStarQueue.push(start, 0)
    visited = [start[0]]
    cheapest_cost_to_location = {0: heuristic(start[0], problem)}

    while not aStarQueue.isEmpty():
        top = aStarQueue.pop()
        top_cost = cheapest_cost_to_location[visited.index(top[0])]
        if problem.isGoalState(top[0]):
            return top[1]
        for successor in problem.getSuccessors(top[0]):
            location = successor[0]
            direction = successor[1]
            cost = successor[2]
            if location not in visited or cheapest_cost_to_location[visited.index(location)] > cost + top_cost:
                #print(f"Location {location}, Heuristic {heuristic(location, problem)}")
                aStarQueue.update((location, top[1] + [direction]), top_cost + cost + heuristic(location, problem))
                visited.append(location)
                cheapest_cost_to_location[len(visited) - 1] = cost + top_cost

    print("Path not found")


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
