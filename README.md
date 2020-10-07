# simple_npuzzle_solver
Solves a small 3x3 n-puzzle using BFS, DFS and A-Star algorithms.
Code uses 0 (X) as the center piece which is moved

#### Usage:
```
npuzzle.py [-h] [-i INPUT] [-o OUTPUT] [-m METHOD]

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT, --input INPUT
                        input board 0,1,2,3,4...
  -o OUTPUT, --output OUTPUT
                        output board 0,1,2,3,4...
  -m METHOD, --method METHOD
                        solver method: ['bfs', 'dfs', 'astar', 'astar2']
```

The code contains 2 versions of A*
- astar2 uses heapq while astar uses a priorityQueue
- astar2 removes possible duplicates which may affect astar

#### Example: 
```
$python npuzzle.py -i 1,5,8,7,6,4,3,2,0
```
Uses `0,1,2,3,4,5,6,7,8` as the default solution state and `star` as the default solver

otherwise:
```
$python npuzzle.py -i 1,5,8,7,6,4,3,2,0 -o 0,1,2,3,4,5,6,7,8 -m astar
```

Output:
```
Board to solve:
| 1  5  8 |
| 7  6  4 |
| 3  2  X |

Expected solution:
| X  1  2 |
| 3  4  5 |
| 6  7  8 |

Steps:
Up, Left, Down, Right, Up, Up, Left, Down, Down, Right, Up, Left, Left, Down, Right, Up, Right, Up, Left, Left

Nodes Expanded: 497
Depth: 20
Cost: 20
Exec Time: 0.03 secs
Ram Usage: 15.61 MB
```
