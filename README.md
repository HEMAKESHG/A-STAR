<h1>ExpNo 4 : Implement A* search algorithm for a Graph</h1> 
<h3>Name: Hemakesh G    </h3>
<h3>Register Number: 212223040064          </h3>
<H3>Aim:</H3>
<p>To ImplementA * Search algorithm for a Graph using Python 3.</p>
<H3>Algorithm:</H3>

``````
// A* Search Algorithm
1.  Initialize the open list
2.  Initialize the closed list
    put the starting node on the open 
    list (you can leave its f at zero)

3.  while the open list is not empty
    a) find the node with the least f on 
       the open list, call it "q"

    b) pop q off the open list
  
    c) generate q's 8 successors and set their 
       parents to q
   
    d) for each successor
        i) if successor is the goal, stop search
        
        ii) else, compute both g and h for successor
          successor.g = q.g + distance between 
                              successor and q
          successor.h = distance from goal to 
          successor (This can be done using many 
          ways, we will discuss three heuristics- 
          Manhattan, Diagonal and Euclidean 
          Heuristics)
          
          successor.f = successor.g + successor.h

        iii) if a node with the same position as 
            successor is in the OPEN list which has a 
           lower f than successor, skip this successor

        iV) if a node with the same position as 
            successor  is in the CLOSED list which has
            a lower f than successor, skip this successor
            otherwise, add  the node to the open list
     end (for loop)
  
    e) push q on the closed list
    end (while loop)

``````
### PROGRAM
```
from collections import defaultdict
import heapq

# Heuristic distances
H_dist = {}

def aStarAlgo(start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start), 0, start))  # (f = g + h, g, node)

    came_from = {}
    g_cost = defaultdict(lambda: float('inf'))
    g_cost[start] = 0

    while open_set:
        f, g, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor, weight in get_neighbors(current):
            tentative_g = g_cost[current] + weight
            if tentative_g < g_cost[neighbor]:
                came_from[neighbor] = current
                g_cost[neighbor] = tentative_g
                heapq.heappush(open_set, (tentative_g + heuristic(neighbor), tentative_g, neighbor))

    print("Path does not exist!")
    return None

def get_neighbors(node):
    return Graph_nodes[node]

def heuristic(node):
    return H_dist.get(node, 0)

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    print("Path found:", path)
    return path

# Input part
Graph_nodes = defaultdict(list)
n, e = map(int, input().split())

for _ in range(e):
    u, v, cost = input().split()
    cost = int(cost)
    Graph_nodes[u].append((v, cost))
    Graph_nodes[v].append((u, cost))  # Undirected

for _ in range(n):
    node, h = input().split()
    H_dist[node] = int(h)

start = input()
goal = input()

aStarAlgo(start, goal)

```


<hr>
<h2>Sample Graph I</h2>
<hr>

![image](https://github.com/natsaravanan/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/87870499/b1377c3f-011a-4c0f-a843-516842ae056a)

<hr>
<h2>Sample Input</h2>
<hr>
10 14 <br>
A B 6 <br>
A F 3 <br>
B D 2 <br>
B C 3 <br>
C D 1 <br>
C E 5 <br>
D E 8 <br>
E I 5 <br>
E J 5 <br>
F G 1 <br>
G I 3 <br>
I J 3 <br>
F H 7 <br>
I H 2 <br>
A 10 <br>
B 8 <br>
C 5 <br>
D 7 <br>
E 3 <br>
F 6 <br>
G 5 <br>
H 3 <br>
I 1 <br>
J 0 <br>
<hr>
<h2>Sample Output</h2>
<hr>
Path found: ['A', 'F', 'G', 'I', 'J']


<hr>
<h2>Sample Graph II</h2>
<hr>

![image](https://github.com/natsaravanan/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/87870499/acbb09cb-ed39-48e5-a59b-2f8d61b978a3)


<hr>
<h2>Sample Input</h2>
<hr>
6 6 <br>
A B 2 <br>
B C 1 <br>
A E 3 <br>
B G 9 <br>
E D 6 <br>
D G 1 <br>
A 11 <br>
B 6 <br>
C 99 <br>
E 7 <br>
D 1 <br>
G 0 <br>
<hr>
<h2>Sample Output</h2>
<hr>
Path found: ['A', 'E', 'D', 'G']

<h2>RESULT</h2>
Implementing A * Search algorithm for a Graph using Python 3. is executed successfully.
