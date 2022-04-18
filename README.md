# Pacman Project for CS4100 Foundations of Artififial Intelligence at Northeastern University Boston

This repository contains the files associated with the work I did for the Pacman project
to practice applying AI techniques in a challenging environment.

- The homepage of the UC Berkely Pac-man Project can be found [here](https://inst.eecs.berkeley.edu/~cs188/fa19/projects/).
- The homepage for CS4100 at Northeastern can be found [here](https://www.ccs.neu.edu/home/marsella/CS5100/Syllabus_2018_Spring/).

## Project Summaries

### Project 1 - Search

This project was oriented towards writing search algorithms, heuristic creations and game state definitions.

#### Search Algorithms
Famous AI search algorithms implemented
- DFS
- BFS
- UCS
- A* search

All these algorithms were implemented in a generic manner so they can be used
for any game states.

#### Heuristic creations

I also created heuristics for A* search for various versions of the pacman game. An
example of this was the FoodSearchProblem where a consistent and admissible heuristic
had to be designed so that the pacman agent eats all the food in the maze
while expanding the lowest number of nodes possible.

__Note: Developed second fastest path finding heuristic in my class__

### Project 2 - MultiAgent Search

#### Algorithms implemented
- MiniMax
- MiniMax with Alpha Beta Pruning
- ExpectiMax

__Note: State evaluation function placed second in my class for max score in a game of pacman__

#### State evaluation function

Also developed a state evaluation function for a game of Pacman that takes into account several factors like
- Distance to food
- Distance to Ghost
- Bonus for power pellets
- Distance to scared ghosts

### Project 3 - GhostBusters

#### Algorithms implemented
- Bayesian Networks
- Probabilistic inference

### Project 4 - Reinforcement Learning

Implemented several RL techniques to create a game agent capable of learning its environment and finding optimal 
actions and policy.
- Value Iteration
- Q Learning
- Policy iteration
- Approximate Q Learning
- Epsilon Greedy action selection
- Prioritised Sweeping Value Iteration



