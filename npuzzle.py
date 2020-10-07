# -*- coding: utf-8 -*-

from queue import Queue, LifoQueue, PriorityQueue
import time, psutil, math, argparse, heapq

class Node:  
    """
    Builds up current board with all possible movements to the next one
    """
    def __init__(self, board, goal, parent = None, move = None, depth = 0):   
        
        self.board = board
        self.parent = parent  
        self.move = move
        self.depth = depth
        self.goal = goal
        
        self.cost = manhattan(self.board, self.goal)
        
        self.zero = self.board.index(0)
        
    def __repr__(self):
        return tuple(self.board)
        
    def __hash__(self):
        return hash(tuple(self.board))
    
    def __eq__(self, other):
        return self.board == other.board and isinstance(other, Node)
    
    def __lt__(self, other):
        return bool((self.depth + self.cost)  < (other.depth + other.cost))

    def posMoves(self):
        moves = []
        # border conditions
        if self.zero not in {0, 1, 2}:
            moves.append('Up')
        if self.zero not in {6, 7, 8}:
            moves.append('Down')
        if self.zero not in {0, 3, 6}:
            moves.append('Left')
        if self.zero not in {2, 5, 8}:
            moves.append('Right')
        return moves

    def childNodes(self):     
        children = []
        for move in self.posMoves():
            newBoard = list(self.board) # simple deepcopy of board 
            if move == 'Up':
                newBoard[self.zero], newBoard[self.zero-3] = newBoard[self.zero-3], newBoard[self.zero]
            if move == 'Down':
                newBoard[self.zero], newBoard[self.zero+3] = newBoard[self.zero+3], newBoard[self.zero]
            if move == 'Left':
                newBoard[self.zero], newBoard[self.zero-1] = newBoard[self.zero-1], newBoard[self.zero]
            if move == 'Right':
                newBoard[self.zero], newBoard[self.zero+1] = newBoard[self.zero+1], newBoard[self.zero]
                
            children.append(Node(newBoard, self.goal, parent = self, move = move, depth = self.depth +1))
        return children


def manhattan(ini, goal):
    w = int(math.sqrt(len(ini)))
    return sum(abs(goal.index(i) - ini.index(i))//w + abs(goal.index(i) - ini.index(i))%w for i in ini if i != 0)

def print_output(steps, nodes, depth, cost, exec_time):
    out = f'Steps:\n{", ".join(steps)}\n\nNodes Expanded: {nodes}\nDepth: {depth}\nCost: {cost}\n'+\
          f'Exec Time: {exec_time:.2f} secs\nRam Usage: {(psutil.Process().memory_info().rss/(1e6)):.2f} MB'
    print(out)
    
def display_board(board, init = True):
    if init == True: 
        print('Board to solve:')
    else: 
        print('Expected solution:')
    str_board = ['X' if x == 0 else str(x) for x in board]
    for i in [0, 3, 6]:
        print(f"| {'  '.join(str_board[i:i+3])} |")
    print()


def bfs(iniState, goalState):
    start_time = time.time()
    
    iniState = Node(iniState, goalState)
    
    frontier = Queue()
    frontier.put(iniState)
    
    # explored nodes
    frontier_explored = set()

    nodesExpanded = 0
    while not frontier.empty():
        
        currState = frontier.get()

        frontier_explored.add(tuple(currState.board))
                           
        if goalState == currState.board: 
            steps = []
            currNode = currState
            while currNode.depth > 0:
                #list.insert(index = 0, value) is a way of inserting values in inverse order
                steps.append(currNode.move)
                currNode = currNode.parent
            steps = steps[::-1]
            print_output(steps, nodesExpanded, currState.depth, len(steps), time.time() - start_time)
            return True
            
        nodesExpanded +=1
            
        for neighbor in currState.childNodes():
            if tuple(neighbor.board) not in frontier_explored:
                frontier_explored.add(tuple(neighbor.board))
                frontier.put(neighbor)                       
    return False
    
def dfs(iniState, goalState):
    start_time = time.time()
    
    iniState = Node(iniState, goalState)
    
    frontier = LifoQueue()
    frontier.put(iniState)
    
    #explored nodes
    frontier_explored = set()

    nodesExpanded = 0
    while not frontier.empty():
        
        currState = frontier.get()

        frontier_explored.add(tuple(currState.board))
                           
        if goalState == currState.board:
            steps = []
            currNode = currState
            while currNode.depth > 0:
                steps.append(currNode.move)
                currNode = currNode.parent
            steps = steps[::-1]
            print_output(steps, nodesExpanded, currState.depth, len(steps), time.time() - start_time)
            return True
        nodesExpanded +=1
            
        for neighbor in reversed(currState.childNodes()):
            if tuple(neighbor.board) not in frontier_explored:
                
                frontier_explored.add(tuple(neighbor.board))
                frontier.put(neighbor)                       
    return False

def astar(iniState, goalState):
    start_time = time.time()
    
    iniState = Node(iniState, goalState)
    
    frontier = PriorityQueue()
    frontier.put(iniState)
    
    # explored nodes
    frontier_explored = set()

    nodesExpanded = 0
    while not frontier.empty():
        
        currState = frontier.get()

        frontier_explored.add(tuple(currState.board))
                           
        if goalState == currState.board:
            steps = []
            currNode = currState
            while currNode.depth > 0:
                steps.append(currNode.move)
                currNode = currNode.parent
            steps = steps[::-1]
            print_output(steps, nodesExpanded, currState.depth, len(steps), time.time() - start_time)
            return True
        nodesExpanded +=1
            
        for neighbor in currState.childNodes():
            if tuple(neighbor.board) not in frontier_explored:
                
                frontier_explored.add(tuple(neighbor.board))
                frontier.put(neighbor)                       
    return False

def astar2(iniState, goalState):
    
    start_time = time.time()
    
    iniState = Node(iniState, goalState)
    
    frontier = []
    #frontier.put(iniState)
    heapq.heappush(frontier,iniState)
    
    #explored nodes
    frontier_explored = set()

    nodesExpanded = 0
    while len(frontier) > 0:
        
        currState = heapq.heappop(frontier)
        
        frontier_explored.add(tuple(currState.board))

        if goalState == currState.board:
            steps = []
            currNode = currState
            while currNode.depth > 0:
                steps.append(currNode.move)
                currNode = currNode.parent
            steps = steps[::-1]
            print_output(steps, nodesExpanded, currState.depth, len(steps), time.time() - start_time)
            return True
        nodesExpanded +=1
            
        for neighbor in currState.childNodes():

            if tuple(neighbor.board) not in frontier_explored:
                frontier_explored.add(tuple(neighbor.board))
                heapq.heappush(frontier, neighbor)
                
            elif neighbor.board in [i.board for i in frontier]:
                
                index = [frontier.index(i) for i in frontier if i.board == neighbor.board][0]

                frontier[index].cost = min(neighbor.cost, frontier[index].cost)
                frontier.remove(frontier[index])
                heapq.heappush(frontier, neighbor)
                heapq.heapify(frontier)
                
              
    return False


def input_check(board):
    if board is None:
        raise ValueError("board is not defined")
    if isinstance(board, str): 
        input_eval = set(sorted(eval(board)))
    else:
        input_eval = set(sorted(board))
    if input_eval == set(range(9)):
        return True
    raise ValueError(f"Invalid board. Board must have {list(range(9))} as input")


if __name__ == '__main__':
    
    solvers = ['bfs', 'dfs', 'astar', 'astar2']
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-i","--input", help="input board 0,1,2,3,4...", 
                        type=str)
    parser.add_argument("-o","--output", help="output board 0,1,2,3,4...", 
                        type=str, default = list(range(9)))
    parser.add_argument("-m","--method", help=f"solver method: {solvers}", 
                        type=str, default = 'astar')
    args = parser.parse_args()

    input_check(args.input)
    input_check(args.output)
    
    input_board = list(eval(args.input))
    goal = list(eval(args.input)) if isinstance(args.output, str) else args.output

    if args.method not in solvers:
        raise ValueError("Invalid Solver")
        
    display_board(input_board, init = True)
    display_board(goal, init = False)

    if args.method == 'bfs':
        sol = bfs(input_board, goal)
    elif args.method == 'dfs':
        sol = dfs(input_board, goal)
    elif args.method == 'astar':
        sol = astar(input_board, goal)
    elif args.method == 'astar2':
        sol = astar2(input_board, goal)
        
    if sol == False:
        print("\nNo solution was found.")

