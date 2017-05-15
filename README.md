# CVRP-genetic
The capacitated vehicle routing problem is a well-studied combinatorial computing problem. There are many ways to solve the problem like evolutionary computing, reinforcement learning and exact methods. A genetic algorithm was implemented to solve a given CVRP instance with 250 customers. An evolutionary computing approach was chosen because of the following reasons: 

1. The exact methods like branch and cut explore the entire search space which is most of the time unfeasible given the computation time. A genetic algorithm can be fine tuned to explore the search space in a specific way.  

2. In this case, the constraints are very simple; maximum vehicle capacity of 500 and a vehicle can visit a customer only once. A real world CVRP can have multiple objectives and constraints like traffic conditions, number of turns, multiple depots etc. A genetic algorithm can adapt to these constraints without the need to change the entire programmatic structure by simply formulating the fitness function again.

# How to run
bash start-cvrp

# How to plot aniated gif
1. Navigate to Plot_tools
2. Place your solution file there – best-solution.txt. (if you want to plot the entire search process, add the intermediate routes as-well).
3. Run ./animate.sh
4. Output: animate.gif

Note: The vrp data is already present in animate.cc. So if you need to use this with other
vrp data, it should be changed there.

# Results
Best so far
![alt text](https://github.com/Asheeshkrsharma/CVRP-genetic/blob/master/5.png "Logo Title Text 1")

Animated gif
![alt text](https://github.com/Asheeshkrsharma/CVRP-genetic/blob/master/output.gif "Logo Title Text 1")

