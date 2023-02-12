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

#Own class node to facilitate graph implementation

class Node():
    #classe pour les noeuds.
    
    def __init__( self,parent_node = None , action_made = None , action_cost = 0 , node_state = None):
        self.parent_node , self.action_made , self.action_cost , self.node_state  = (parent_node , action_made , action_cost , node_state)
        
    def getAchild(self,successor_items):
        return Node(parent_node =self , action_made=successor_items[1] , action_cost=successor_items[2] + self.action_cost , node_state = successor_items[0])
    
    


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

    
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    start_node = Node(node_state=problem.getStartState())
    open_list=util.Stack()
    open_list.push(start_node)
    closed_list=[]
    action_list=[]
    continue_search = True
    while continue_search:
        if open_list.isEmpty():
            return []
        
        current_node=open_list.pop()

        if problem.isGoalState(current_node.node_state):
            current_parent=current_node.parent_node
            action_made=current_node.action_made
            while current_parent is not None:
                action_list.append(action_made)
                action_made=current_parent.action_made
                current_parent=current_parent.parent_node
            action_list.reverse()
            return action_list
        
        if current_node.node_state not in closed_list:
            closed_list.append(current_node.node_state)
            successors_items=problem.getSuccessors(current_node.node_state)
            for successor_items in successors_items:
                if successor_items[0] not in closed_list:
                    open_list.push(current_node.getAchild(successor_items))
    util.raiseNotDefined()
def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    start_node = Node(node_state=problem.getStartState())
    open_list=util.Queue()
    open_list.push(start_node)
    closed_list=[]
    action_list=[]
    continue_search = True
    while continue_search:
        if open_list.isEmpty():
            return []
        
        current_node=open_list.pop()

        if problem.isGoalState(current_node.node_state):
            current_parent=current_node.parent_node
            action_made=current_node.action_made
            while current_parent is not None:
                action_list.append(action_made)
                action_made=current_parent.action_made
                current_parent=current_parent.parent_node
            action_list.reverse()
            return action_list
        
        if current_node.node_state not in closed_list:
            closed_list.append(current_node.node_state)
            successors_items=problem.getSuccessors(current_node.node_state)
            for successor_items in successors_items:
                if successor_items[0] not in closed_list:
                    open_list.push(current_node.getAchild(successor_items))
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start_node = Node(node_state=problem.getStartState())
    start_cost = 0
    open_list = util.PriorityQueue()
    open_list.push(start_node , start_cost)
    closed_list = []
    action_list = []
    continue_search = True
    while continue_search:
        if open_list.isEmpty():
            return []
        
        current_node=open_list.pop()

        if problem.isGoalState(current_node.node_state):
            current_parent=current_node.parent_node
            action_made=current_node.action_made
            while current_parent is not None:
                action_list.append(action_made)
                action_made=current_parent.action_made
                current_parent=current_parent.parent_node
            action_list.reverse()
            return action_list
        
        if current_node.node_state not in closed_list:
            closed_list.append(current_node.node_state)
            successors_items=problem.getSuccessors(current_node.node_state)
            for successor_items in successors_items:
                if successor_items[0] not in closed_list:
                    open_list.update(current_node.getAchild(successor_items) ,current_node.getAchild(successor_items).action_cost)
    util.raiseNotDefined()
    

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    start_node = Node(node_state=problem.getStartState())
    start_cost = 0
    open_list = util.PriorityQueue()
    open_list.push(start_node , start_cost)
    closed_list = []
    action_list = []
    continue_search = True
    while continue_search:
        if open_list.isEmpty():
            return []
        
        current_node=open_list.pop()

        if problem.isGoalState(current_node.node_state):
            current_parent=current_node.parent_node
            action_made=current_node.action_made
            while current_parent is not None:
                action_list.append(action_made)
                action_made=current_parent.action_made
                current_parent=current_parent.parent_node
            action_list.reverse()
            return action_list
        
        if current_node.node_state not in closed_list:
            closed_list.append(current_node.node_state)
            successors_items=problem.getSuccessors(current_node.node_state)
            for successor_items in successors_items:
                if successor_items[0] not in closed_list:
                    h = heuristic(current_node.getAchild(successor_items).node_state, problem)
                    open_list.update(current_node.getAchild(successor_items) ,current_node.getAchild(successor_items).action_cost + h)
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
