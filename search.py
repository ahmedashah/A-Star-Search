

import queue
import random
import math
import time
import psutil
import os
from collections import deque
import sys
import heapq

CUTOFF = 0 
FAILURE = 1 
depth_expand = 0 
heuristic = " "

# This class defines the state of the problem in terms of board configuration
class Board:
    def __init__(self, tiles):
        if len(tiles) != 16:
            raise ValueError("Not a 4x4 grid.")
        self.tiles = tiles
    
    def __lt__(self, other):
        return self.state < other.state


    # This function returns the resulting state from taking particular action from current state
    def execute_action(self, action):
        newTiles = self.tiles[:]
        emptySpot = newTiles.index('0')
        
        if action == 'U' and emptySpot - 4 >= 0:
            origSpot = newTiles[emptySpot - 4]
            newTiles[emptySpot - 4] = newTiles[emptySpot]
            newTiles[emptySpot] = origSpot 
        elif action == 'D' and emptySpot + 4 < 16:
            origSpot = newTiles[emptySpot + 4]
            newTiles[emptySpot + 4] = newTiles[emptySpot]
            newTiles[emptySpot] = origSpot 
        elif action == 'L' and emptySpot % 4 > 0: 
            origSpot = newTiles[emptySpot - 1]
            newTiles[emptySpot - 1] = newTiles[emptySpot]
            newTiles[emptySpot] = origSpot 
        elif action == 'R' and emptySpot % 4 < 3 :
            origSpot = newTiles[emptySpot + 1]
            newTiles[emptySpot + 1] = newTiles[emptySpot]
            newTiles[emptySpot] = origSpot 
        
        return Board(newTiles)


            



# This class defines the node on the search tree, consisting of state, parent and previous action
class Node:
    def __init__(self, state, parent, action, depth):
        self.state = state
        self.parent = parent
        self.action = action 
        self.depth = depth

    # Returns string representation of the state
    def __repr__(self):
        return str(self.state.tiles)

    # Comparing current node with other node. They are equal if states are equal
    def __eq__(self, other):
        return self.state.tiles == other.state.tiles

    def __hash__(self):
        return hash(tuple(self.state.tiles))

    def __lt__(self, other):
        return self.depth < other.depth


class Search:




    # This function returns the list of children obtained after simulating the actions on current node
    def get_children(self, parent_node):
        actions = ['U', 'D', 'L', 'R']
        children = []
        for action in actions:
            children.append(Node(parent_node.state.execute_action(action), parent_node, action, parent_node.depth +1) )
        return children

    # This function backtracks from current node to reach initial configuration. The list of actions would constitute a solution path
    def find_path(self, node):
        path = []
        while node.parent:
            path.insert(0, node.action)
            node = node.parent 
        return path 


    def HueristicFunction(self, node, heuristic):
        if heuristic == "h1": #misplaced tiles
            goal_state = ['1','2','3','4','5','6','7','8','9','10','11','12','13','14','15','0']
            misplaced_tiles = 0
            for current_tile, goal_tile in zip(node.state.tiles, goal_state):
                if current_tile != goal_tile and current_tile != '0':
                    misplaced_tiles += 1
            return node.depth + misplaced_tiles

        else: # manhattan distance
            final_positions = [
            (0, 0), (0, 1), (0, 2), (0, 3),
            (1, 0), (1, 1), (1, 2), (1, 3),
            (2, 0), (2, 1), (2, 2), (2, 3),
            (3, 0), (3, 1), (3, 2), (3, 3)
            ]
            total_distance = 0

            for index, tile in enumerate(node.state.tiles):
                if tile == '0':
                    continue
        
                current_row, current_col = divmod(index, 4)
                goal_row, goal_col = final_positions[int(tile) - 1]
                total_distance += abs(goal_row - current_row) + abs(goal_col - current_col)
            return total_distance + node.depth
            
        

   
 # This function runs depth limited search from the given root node and returns path
    def run_dfs(self, curr_node, lim):
        global depth_expand  
        if self.goal_test(curr_node.state.tiles):
            return curr_node, depth_expand
    
        elif lim == 0: 
            return CUTOFF, depth_expand
    
        depth_expand += 1 
        cutoff_check = False 
    
        for child in self.get_children(curr_node):
            result, expanded_nodes = self.run_dfs(child, lim - 1)
        
            if not isinstance(result, Node):
                if result != FAILURE:
                    return FAILURE, expanded_nodes
                elif result == CUTOFF:
                    cutoff_check = True
            else: 
                return result, expanded_nodes
    
        if cutoff_check:
            return CUTOFF, depth_expand
        else: 
            return FAILURE, depth_expand


    def run_a_star(self, root, heuristic):
        global depth_expand  

        openL = [(self.HueristicFunction(root, heuristic), root)]
        closedL = set()

        while openL:
            curr = heapq.heappop(openL)[1]

            if self.goal_test(curr.state.tiles):
                return curr, depth_expand
                
            for child in self.get_children(curr):
                if not any(child == insideL for _, insideL in openL) and not child in closedL :
                    heapq.heappush(openL, (self.HueristicFunction(child,heuristic),child))
                    depth_expand += 1
            closedL.add(curr)
        
        return None, 0


          

    def goal_test(self, cur_tiles):
        return cur_tiles == [str(i) for i in range(1,16)] + ['0']

    def solve(self, input):
        heuristic = "h1"
        initial_list = input.split(" ")
        root = Node(Board(initial_list), None, None, 0)
        process =  psutil.Process(os.getpid())
        initMem = process.memory_info().rss / 1024.0
        expanded_nodes_total = 0
        time_taken_total = 0
        memory_consumed_total = 0

        notFoundYet = True 
        i = 0 
        path = None
        startTime = time.time()
        # interative deepening search
        while(notFoundYet):
            depth_expand = 0
            result, expanded_nodes_total = self.run_a_star(root, heuristic)
            if isinstance(result, Node):
                path = self.find_path(result)
                notFoundYet = False
            i += 1
        finalMem = process.memory_info().rss / 1024.0
        endTime = time.time()
        totalTime = endTime - startTime
        memoryUsed = finalMem - initMem
        #path, expanded_nodes, time_taken, memory_consumed = self.run_bfs(root)
        print("Moves: " + " ".join(path))
        print("Number of expanded Nodes: " + str(expanded_nodes_total))
        print("Time Taken: " + str(totalTime))
        print("Max Memory (Bytes): " + str(memoryUsed))
        return "".join(path)

# Testing the algorithm locally
if __name__ == '__main__':
    heuristic = "h2"
    agent = Search()
    #agent.solve("1 2 3 4 5 6 7 8 9 10 11 0 13 14 15 12")
    #agent.solve("2 8 1 0 4 6 3 7 5 9 10 12 13 14 11 15", "h2")


