## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

The `motion_planning.py` file contains the finite state machine for the flight plan. In addition to the states in `backyard_flyer_solution.py`, the set of states in `motion_planning.py` contains an extra state, `PLANNING` with tranzitions `ARMING` --> `PLANNING` using the method `plan_path` and `PLANNING` --> `TAKEOFF` using the method `takeoff_transition`. The path planning method load the information about obstacles from the `colliders.csv` file, then creates the grid representation from the data loaded by calling the function `create_grid` from the file `planning_utils.py`.  Next, the start and goal points are initiliazed and uthe A* algorithm is used to find a path from start to goal. The path is jerky because all the waypoints are kept, even if they are collinear. The drone stops to each waypoint before moving on to the next.

The `planning_utils.py` file contains code for creating the grid based on the obstacles file, `colliders.csv`, a definition of the possible actions from a grid cell `Action`, a function `valid_actions` that checks which of the actions are valid from a cell (the next move stays within the grid limits and the grid cell is free), an implementation of the A* algorithm in `a_star` with heuristic the Euclidean distance.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

**Step 1: Read and parse the first line of the csv file**

I added a method that replaces the spaces with commas and then splits the line using comma as delimiter:
```
def get_home(self):
        with open('colliders.csv') as f:
            line = f.readline().rstrip()
            tokens = line.replace(' ', ',').split(',')
            return (float(tokens[1]), float(tokens[4]))
```

**Step 2: Set the home position**
```
#read lat0, lon0 from colliders into floating point values
lat0, lon0 = self.get_home()        
#set home position to (lon0, lat0, 0)
self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position

Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position

I used the current global position to determine the local postion:
```
local_north, local_east, _ = global_to_local(self.global_position, self.global_home)
```
Then I calculated the grid coordinates of the local position:
```
grid_start = (int(local_north) - north_offset, int(local_east) - east_offset)
```

#### 4. Set grid goal position from geodetic coords

First, I set the latitude and longitude of the goal. To get this positions, I used the manual control to fly the drone. I used the values displayed at the end of the flight.
```
goal_lat = 37.79456
goal_lon = -122.397437
```
Next, I translated the GPS coordinates to local coordinates:
```
local_goal = global_to_local( (goal_lon, goal_lat, 0) ,self.global_home)
```
Finally, I converted the local coordinates to grid coordinates:
```
grid_goal = (int(local_goal[0]) - north_offset, int(local_goal[1]) - east_offset)
```
        
#### 5. Modify A* to include diagonal motion (or replace A* altogether)

I added diagonal motions. I added the following attributes to the `Action`:
```
NE = (-1, 1, np.sqrt(2))
SE = (1, 1, np.sqrt(2))
NW = (-1, -1, np.sqrt(2))
SW = (1, -1, np.sqrt(2))
```

I also changed the `valid_actions` functions to check the validity of the diagonal actions:
```
if x - 1 < 0 and y + 1 > m and grid[x - 1, y + 1] == 1:
  valid_actions.remove(Action.NE)
if x + 1 > n and y + 1 > m and grid[x + 1, y + 1] == 1:
  valid_actions.remove(Action.SE)
if x - 1 < 0 and y - 1 < 0 and grid[x - 1, y - 1] == 1:
  valid_actions.remove(Action.NW)
if x + 1 > n and y - 1 < 0 and grid[x + 1, y + 1] == 1:
  valid_actions.remove(Action.SW)
```


#### 6. Cull waypoints 

I used the collinearity test and prunning code provided in the lessons.


### Execute the flight
#### 1. Does it work?
Almost. I installed the simulator on my home computer and the waypoints are calculated correctly. However, the take off stage never reaches the correct altitude. This part of the project is provided and even if the backayard test seems to fly correctly, the take off in the motion planing is not executed.

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


