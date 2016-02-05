# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

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
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    return _depth_first_search(problem)
9
def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    return _breadth_first_search(problem)

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    return _uniform_cost_search(problem)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    return _a_star_search(problem, heuristic)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

# Define some indices for accessing our tuples
(state, action, cost, parent) = 0, 1, 2, 3


def _cost_function(state_tuple):
    return state_tuple[cost]


def _build_path(state_tuple, parent_of):
    actions = list()
    while state_tuple[action] != Directions.STOP:
        actions.append(state_tuple[action])
        state_tuple = parent_of[state_tuple]
    return list(reversed(actions))

def _find_path(state_tuple):
    actions = list()
    while state_tuple[parent] is not None:
        actions.append(state_tuple[action])
        state_tuple = state_tuple[parent]
    # print 'about to return'
    return list(reversed(actions))


def _depth_first_search(problem):
    explored_states = set()

    # convert the start state into a
    # 4-tuple (state, action, cost, parent):
    start_state_tuple = (problem.getStartState(), Directions.STOP, 0, None)

    fringe = util.Stack()
    fringe.push(start_state_tuple)

    # map of the parent of each state
    parent_of = dict()

    while not fringe.isEmpty():
        current_tuple = fringe.pop()

        if problem.isGoalState(current_tuple[state]):
            path = _find_path(current_tuple)
            return path

        if current_tuple[state] not in explored_states:
            explored_states.add(current_tuple[state])

            for successor_tuple in problem.getSuccessors(current_tuple[state]):
                parent_of[successor_tuple] = current_tuple
                successor_tuple = successor_tuple + (current_tuple,)
                fringe.push(successor_tuple)

    return None


def _breadth_first_search(problem):
    explored_states = set()

    # convert the start state into a
    # 4-tuple (state, action, cost, parent):
    start_state_tuple = (problem.getStartState(), Directions.STOP, 0, None)

    fringe = util.Queue()
    fringe.push(start_state_tuple)

    explored_states.add(start_state_tuple[state])

    # map of the parent of each state
    parent_of = dict()

    while not fringe.isEmpty():
        current_tuple = fringe.pop()

        if problem.isGoalState(current_tuple[state]):
            path = _find_path(current_tuple)
            return path

        for successor_tuple in problem.getSuccessors(current_tuple[state]):
            if successor_tuple[state] not in explored_states:
                explored_states.add(successor_tuple[state])
                parent_of[successor_tuple] = current_tuple
                successor_tuple = successor_tuple + (current_tuple,)
                fringe.push(successor_tuple)

    return None


def _uniform_cost_search(problem):
    explored_states = set()

    # convert the start state into a
    # 3-tuple (state, action, cost):
    start_state_tuple = (problem.getStartState(), Directions.STOP, 0)

    fringe = util.PriorityQueueWithFunction(_cost_function)
    fringe.push(start_state_tuple)

    # map of the parent of each state
    parent_of = dict()

    while not fringe.isEmpty():
        current_tuple = fringe.pop()

        if problem.isGoalState(current_tuple[state]):
            return _build_path(current_tuple, parent_of)

        if current_tuple[state] not in explored_states:
            explored_states.add(current_tuple[state])

            for successor_tuple in problem.getSuccessors(current_tuple[state]):
                if successor_tuple[state] not in explored_states:
                    cumulative_cost = successor_tuple[cost] + current_tuple[cost]
                    new_successor_tuple = successor_tuple[:-action] + (cumulative_cost,)
                    fringe.push(new_successor_tuple)
                    parent_of[new_successor_tuple] = current_tuple

    return None


def _a_star_search(problem, heuristic):
    explored_states = set()
    # map of the parent of each state
    parent_of = dict()

    # convert the start state into a
    # 3-tuple (state, action, cost)
    # and push into fringe:
    start_state_tuple = (problem.getStartState(), Directions.STOP, 0)
    fringe = util.PriorityQueue()

    cost_so_far = dict()
    cost_so_far[start_state_tuple[state]] = 0

    fringe.push(start_state_tuple, start_state_tuple[cost])

    while not fringe.isEmpty():
        current_tuple = fringe.pop()

        if problem.isGoalState(current_tuple[state]):
            return _build_path(current_tuple, parent_of)

        for successor_tuple in problem.getSuccessors(current_tuple[state]):
            new_cost = cost_so_far[current_tuple[state]] + successor_tuple[cost]
            if successor_tuple[state] not in cost_so_far or new_cost < cost_so_far[successor_tuple[state]]:
                cost_so_far[successor_tuple[state]] = new_cost
                priority = new_cost + heuristic(successor_tuple[state], problem)
                fringe.push(successor_tuple, priority)
                parent_of[successor_tuple] = current_tuple

    return None
