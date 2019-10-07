# pacmanAgents.py
# ---------------
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


from pacman import Directions
from game import Agent
from heuristics import *
import random


# node class is the class that store all the state value and its has the following feature:
# parent : stores the parent state of the node
# state : current node state
# depth : current node depth( root is at depth :0)

class Node:
    def __init__(self, parent, action, state, depth):
        self.parent = parent
        self.action = action
        self.state = state
        self.depth = depth

    # recursive finding the  depth 1 action
    def getparent(self):
        if self.depth is not 1:
            node = self.parent.getparent()
        else:
            return self.action
        return node


class RandomAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        actions = state.getLegalPacmanActions()
        # returns random action from all the valide actions
        return actions[random.randint(0, len(actions) - 1)]


class OneStepLookAheadAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        legal = state.getLegalPacmanActions()
        # get all the successor state for these actions
        successors = [(state.generatePacmanSuccessor(action), action) for action in legal]
        # evaluate the successor states using scoreEvaluation heuristic
        scored = [(admissibleHeuristic(state), action) for state, action in successors]
        # get best choice
        bestScore = min(scored)[0]
        # get all actions that lead to the highest score
        bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
        # return random action from the list of the best actions
        return random.choice(bestActions)


# this fucntion finds the successor states for BFS and DFS algo and append in the list known as fringe
# and if successor are out of call returns flag = true with fringe of left elements

def findlegalStateAndSuccessors(fringe, parent, explored, flag):
    # get all legal actions for pacman
    legal = parent.state.getLegalPacmanActions()
    if flag is False:
        for act in legal:
            dummy = parent.state.generatePacmanSuccessor(act)
            if dummy is not None:
                newnode = Node(parent, act, dummy, parent.depth + 1)
                fringe.append(newnode)
            else:
                flag = True
                fringe.append(parent)
                break
    return fringe, flag


class BFSAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write BFS Algorithm instead of returning Directions.STOP
        flag = False # flag to track the successor function calls

        parent = Node(None, None, state, 0)
        # fringe is the list that contain all the nodes that will be expanded
        fringe = [parent]
        explored = []
        while fringe:
            #pop the first element using FIFO
            node = fringe.pop(0)
            explored.append(node.state)
            if node.state.isLose():
                # print "kick"
                continue
            elif node.state.isWin():
                print "win"
                return node.getparent()
            else:
                fringe, flag = findlegalStateAndSuccessors(fringe, node, explored, flag)
            if flag:
                break

            # If not reaching a terminal state, return the actions leading to the node with
            # the best score and no children based on the heuristic function (admissibleHeuristic)
        if not fringe == []:
            # evaluate the successor states using scoreEvaluation heuristic
            scored = [(admissibleHeuristic(node.state) + node.depth, node) for node in fringe]
            bestScore = min(scored)[0]
            bestnode = [pair[1] for pair in scored if pair[0] == bestScore]
            choice = random.choice(bestnode).getparent()
            return choice

        print "irrr"
        return node.action


class DFSAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        flag = False  # flag to track the successor function calls

        parent = Node(None, None, state, 0)
        # fringe is the list that contain all the nodes that will be expanded
        fringe = [parent]
        explored = []
        while fringe:
            # pop the first element using LIFO
            node = fringe.pop(-1)
            explored.append(node.state)
            if node.state.isLose():
                # print "kick"
                continue
            elif node.state.isWin():
                print "win"
                return node.getparent()
            else:
                fringe, flag = findlegalStateAndSuccessors(fringe, node, explored, flag)
            if flag:
                break

            # If not reaching a terminal state, return the actions leading to the node with
            # the best score and no children based on the heuristic function (admissibleHeuristic)
        if not fringe == []:
            # evaluate the successor states using scoreEvaluation heuristic
            scored = [(admissibleHeuristic(node.state) + node.depth, node) for node in fringe]
            bestScore = min(scored)[0]
            bestnode = [pair[1] for pair in scored if pair[0] == bestScore]
            choice = random.choice(bestnode).getparent()
            return choice

        print "irrr"
        return node.action


class AStarAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write A* Algorithm instead of returning Directions.STOP
        flag = False
        # get all the successor state for these actions

        parent = Node(None, None, state, 0)

        fringe = [(parent, admissibleHeuristic(parent.state))]



        # scored = [(admissibleHeuristic(state), action) for state, action in successors]
        explored = []
        while fringe:
            fringe.sort(key=lambda tup: tup[1])  # sort the list and pop the least , making it a priority queue
            node, cost = fringe.pop(0)
            explored.append(node.state)
            if node.state.isLose():
                continue
            elif node.state.isWin():
                print "win"
                return node.getparent()
            else:      # find and evaluate the successor states using scoreEvaluation heuristic
                legal = node.state.getLegalPacmanActions()
                if flag is False:
                    for act in legal:
                        dummy = node.state.generatePacmanSuccessor(act)
                        if dummy is not None:
                            newnode = Node(node, act, dummy, node.depth + 1)
                            newcost = (admissibleHeuristic(dummy)) + newnode.depth
                            fringe.append((newnode, newcost))
                        else:
                            flag = True
                            fringe.append((node, cost))
                            break
            if flag:
                break

        if flag:
            fringe.sort(key=lambda tup: tup[1])
            node, cost = fringe.pop(0)
            choice = node.getparent()
            return choice