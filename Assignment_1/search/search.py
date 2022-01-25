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
    list_of_directions = []

    cur_node = goal
    while cur_node != start:
        list_of_directions.append(dir_map[cur_node][1])
        cur_node = dir_map[cur_node][0]
    return list_of_directions


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
    visited = set()
    dfs_stack = util.Stack()
    start = problem.getStartState()
    successor_node_and_direction = {}
    goal_state = None
    dfs_stack.push(start)

    while not dfs_stack.isEmpty():
        top = dfs_stack.pop()
        if top in visited:
            continue
        if problem.isGoalState(top):
            goal_state = top
            break
        for successor in problem.getSuccessors(top):
            location = successor[0]
            direction = successor[1]
            if location in visited:
                continue
            successor_node_and_direction[location] = (top, direction)
            dfs_stack.push(location)
        visited.add(top)

    direction_list = get_directions_from_map(successor_node_and_direction, goal_state, start)
    return direction_list[::-1]


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    visited = set()
    bfs_queue = util.Queue()
    start = problem.getStartState()
    successor_node_and_direction = {}
    goal_state = None
    bfs_queue.push(start)
    visited.add(start)

    while not bfs_queue.isEmpty():
        top = bfs_queue.pop()
        if problem.isGoalState(top):
            goal_state = top
            break
        for successor in problem.getSuccessors(top):
            location = successor[0]
            direction = successor[1]
            if location in visited:
                continue
            visited.add(location)
            successor_node_and_direction[location] = (top, direction)
            bfs_queue.push(location)

    direction_list = get_directions_from_map(successor_node_and_direction, goal_state, start)
    return direction_list[::-1]


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    visited = set()
    ucs_queue = util.PriorityQueue()
    start = problem.getStartState()
    successor_node_and_direction = {}
    goal_state = None
    cheapest_cost_to_get_to_node = {}

    visited.add(start)
    ucs_queue.push(start, 0)
    cheapest_cost_to_get_to_node[start] = 0
    best_solution = float('inf')

    while not ucs_queue.isEmpty():
        top = ucs_queue.pop_with_priority()
        topCost = top[0]
        topNode = top[1]
        if topCost >= best_solution:
            break
        if problem.isGoalState(topNode):
            best_solution = topCost
            goal_state = topNode
            continue
        for successor in problem.getSuccessors(topNode):
            location = successor[0]
            direction = successor[1]
            cost = successor[2]
            if location not in visited or cheapest_cost_to_get_to_node[location] > cost + topCost:
                successor_node_and_direction[location] = (topNode, direction)
                cheapest_cost_to_get_to_node[location] = cost + topCost
                visited.add(location)
                ucs_queue.update(location, topCost + cost)

    direction_list = get_directions_from_map(successor_node_and_direction, goal_state, start)
    return direction_list[::-1]

    # util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    visited = set()
    aStar_queue = util.PriorityQueue()
    start = problem.getStartState()
    successor_node_and_direction = {}
    goal_state = None
    cheapest_cost_to_get_to_node = {}

    visited.add(start)
    aStar_queue.push(start, 0)
    cheapest_cost_to_get_to_node[start] = 0

    while not aStar_queue.isEmpty():
        top = aStar_queue.pop_with_priority()
        topNode = top[1]
        topCost = cheapest_cost_to_get_to_node[topNode]
        if problem.isGoalState(topNode):
            goal_state = topNode
            break
        for successor in problem.getSuccessors(topNode):
            location = successor[0]
            direction = successor[1]
            cost = successor[2]
            if location not in visited or cheapest_cost_to_get_to_node[location] > cost + topCost:
                successor_node_and_direction[location] = (topNode, direction)
                cheapest_cost_to_get_to_node[location] = cost + topCost
                visited.add(location)
                aStar_queue.update(location, topCost + cost + heuristic(location, problem))

    direction_list = get_directions_from_map(successor_node_and_direction, goal_state, start)
    return direction_list[::-1]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
