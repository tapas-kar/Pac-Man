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

import util

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
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "***YOUR CODE HERE***"

    # fringe to hold the subsequent tentative nodes to be expanded
    stack_fringe = util.Stack()

    # list of actions taken by the agent to get to the next state
    agent_actions = []

    # cost of each step taken by the agent
    step_costs = 0

    # list of explored nodes or states by the agent
    explored_states = []

    # initial state of the agent, which is equal to the problem's start state as the argument
    start_state = problem.getStartState()

    # push the initial state of the agent onto the stack fringe
    stack_fringe.push((start_state, agent_actions, step_costs))

    # the depth-first search algorithm

    # while the stack_fringe is not empty
    while stack_fringe:

        # list of the states already expanded, stored in the temporary list
        temporary_list = stack_fringe.pop()

        # condition to check if the expanded state is in the explored states list
        if temporary_list[0] not in explored_states:

            # add the expanded state to the explored states
            explored_states.append(temporary_list[0])

            # condition to check if the goal state has been reached by the agent,
            # if True then return the respective action that resulted in agent reaching the goal
            if problem.isGoalState(temporary_list[0]):
                return temporary_list[1]

            # update the state of the agent by getting the successor state, new_action and the new_cost
            for s, a, c in problem.getSuccessors(temporary_list[0]):
                new_action = temporary_list[1] + [a]
                new_cost = temporary_list[2] + c
                new_state = (s, new_action, new_cost)

                # push the new_state onto the stack fringe
                stack_fringe.push(new_state)

    util.raiseNotDefined()

# Sub questions
# The exploration order was not what I expected. The Pacman actually does not go to all the explored
# squares on his way to the goal.
# It is not the least cost solution, since it traverses along a lot of nodes that do not lead to the goal
# efficiently.


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "***YOUR CODE HERE***"

    # fringe to hold the subsequent tentative nodes to be expanded
    queue_fringe = util.Queue()

    # list of actions taken by the agent to get to the next state
    agent_actions = []

    # cost of each step taken by the agent
    step_costs = 0

    # list of explored nodes or states by the agent
    explored_states = []

    # initial state of the agent, which is equal to the problem's start state as the argument
    start_state = problem.getStartState()

    # push the initial state of the agent onto the queue fringe
    queue_fringe.push((start_state, agent_actions, step_costs))

    # the breadth-first search algorithm

    # while the queue_fringe has states to expand
    while queue_fringe:

        # list of the states already expanded, stored in the temporary list
        temporary_list = queue_fringe.pop()

        # condition to check if the expanded state is in the explored states list
        if temporary_list[0] not in explored_states:

            # add the expanded state to the explored states
            explored_states.append(temporary_list[0])

            # condition to check if the goal state has been reached by the agent,
            # if True then return the respective action that resulted in agent reaching the goal
            if problem.isGoalState(temporary_list[0]):
                return temporary_list[1]

            # update the state of the agent by getting the successor state, new_action and the new_cost
            for s, a, c in problem.getSuccessors(temporary_list[0]):
                new_action = temporary_list[1] + [a]
                new_cost = temporary_list[2] + c
                new_state = (s, new_action, new_cost)

                # push the new_state onto the queue fringe
                queue_fringe.push(new_state)

    util.raiseNotDefined()

# The BFS provides a better solution at a lower cost for the medium maze compared to the DFS. However,
# it does not provide the least cost solution while solving the bigMaze problem.

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "***YOUR CODE HERE***"

    # fringe to hold the subsequent tentative nodes to be expanded
    priorityQueue_fringe = util.PriorityQueue()

    # list of actions taken by the agent to get to the next state
    agent_actions = []

    # cost of each step taken by the agent
    step_costs = 0

    # list of explored nodes or states by the agent
    explored_states = []

    # initial state of the agent, which is equal to the problem's start state as the argument
    start_state = problem.getStartState()

    # push the initial state of the agent onto the priority queue fringe
    priorityQueue_fringe.push((start_state, agent_actions, step_costs), step_costs)

    # the uniform-cost search algorithm

    # while the priority queue still has nodes to be expanded
    while priorityQueue_fringe:

        # list of the states already expanded, stored in the temporary list
        temporary_list = priorityQueue_fringe.pop()

        # condition to check if the goal state has been reached by the agent,
        # if True then return the respective action that resulted in agent reaching the goal
        if problem.isGoalState(temporary_list[0]):
            return temporary_list[1]

        # condition to check if the expanded state is in the explored states list
        if temporary_list[0] not in explored_states:

            # add the expanded state to the explored states
            explored_states.append(temporary_list[0])

            # update the state of the agent by getting the successor state, new_action and the new_cost
            for s, a, c in problem.getSuccessors(temporary_list[0]):
                new_action = temporary_list[1] + [a]
                new_cost = temporary_list[2] + c
                new_state = (s, new_action, new_cost)

                # push the new_state onto the priority queue fringe
                priorityQueue_fringe.push(new_state, new_cost)

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE"

    # fringe to hold the subsequent tentative nodes to be expanded
    priorityQueue_fringe = util.PriorityQueue()

    # list of actions taken by the agent to get to the next state
    agent_actions = []

    # cost of each step taken by the agent
    step_costs = 0

    # cost of the heuristic designed to give a better idea of the goal state to the agent
    heuristic_cost = 0

    # list of explored nodes or states by the agent
    explored_states = []

    # initial state of the agent, which is equal to the problem's start state as the argument
    start_state = problem.getStartState()

    # push the initial state of the agent onto the priority queue fringe
    # this state also takes the heuristic cost into account in order to make better decisions while
    # expanding nodes
    priorityQueue_fringe.push((start_state, agent_actions, step_costs), heuristic_cost)

    # while there are more nodes to expand
    while priorityQueue_fringe:

        # list of the states already expanded, stored in the temporary list
        temporary_list = priorityQueue_fringe.pop()

        # condition to check if the expanded state is in the explored states list
        if temporary_list[0] not in explored_states:

            # add the expanded state to the explored states
            explored_states.append(temporary_list[0])

            # condition to check if the goal state has been reached by the agent,
            # if True then return the respective action that resulted in agent reaching the goal
            if problem.isGoalState(temporary_list[0]):
                return temporary_list[1]

            # update the state of the agent by getting the successor state, new_action and the new_cost
            for s, a, c in problem.getSuccessors(temporary_list[0]):
                new_action = temporary_list[1] + [a]
                new_cost = temporary_list[2] + c
                new_state = (s, new_action, new_cost)
                heuristic_cost = new_cost + heuristic(s, problem)

                # push the new_state onto the priority queue fringe
                priorityQueue_fringe.push(new_state, heuristic_cost)

    util.raiseNotDefined()

# The BFS and UCS search algorithm solve the open maze with the most number of nodes expanded.
# The DFS solves it with the least average score. The AStar search solves the open maze with medium
# number of nodes expanded but with the most average score.


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
