# What to do
- improve movement/collision
- make navigation work (robots don't stuck into wall if task is across it) [Nova]
- estimate task utility and with regard to energy budget [Petr]
	- estimate real distance to target
- implement limited communication range

#### Questions
- Auctioning in order - does it have to be the case, is it known to robots?
- Can we reauction tasks that were already auctioned?
- Is the goal of the project to achieve the best performance or implement the algorithms as was in lecture
- What is considered as market based strategy for the distributed case? 
	- Do the robots bid in pairs? 
	- Can we use a central robot for communication
	- Can the robots 'strategize' so that they bid low in order to get two tasks close together
- Confirm that robots know the task position and types at unlimited range


#### Notes
- Arena is 1.25m x 1.25m
- Movement is fast related to task completion

#### Implementation
- robots has 120s of budget
- they assign some value to retaining their budget in expectation of future task assignment, value decreases as time passes and we approach the 180s time limit: $u_{0}$
- robots can buy and sell task from the auctioneer -> bidding happens at all times (or only when new information is available)
	- when two robots meets there is possibility to change tasks (they can have up to 1 or 3 tasks at time)
- All tasks have utility of one
- value of buying a task with $t$ time to complete $1-u_{0}t$ from current trajectory
- value of selling a task will reduce completion time by $\Delta t$: $u_{0}\Delta t-1$


# Suggestion of task allocation setup

1. Initialize for each robot:
    1. A target list $\mathbf{p}_i = \{ p_{i1}, p_{i2}, p_{i3} \}$, ordered after how the robot plans to complete the tasks.
    2. A bundle list $\mathbf{b}_i = \{ b_{i1}, b_{i2}, b_{i3} \}$, ordered by when it bid on the task. So that task 1 was added before 2. 
    3. A list of winning robots $\mathbf{z}_i = \{z_{i1}, \ldots, z_{in_t} \}$  where $n_t$ is the total number of events and $z_{ij}$ is who robot i believes is winning the task j.  -1 equals that no one is winning. This will not be static in size. 
    4. A list of winning bids $\mathbf{y}_i = \{y_{i1}, \ldots, y_{in_t} \}$  where $y_{ij}$  is what robot i believes is teh best bid on task j by winner $z_{ij}$.  -inf means no one is winning. 
    5. A list of timestamps $\mathbf{s}_i = \{s_{i1}, \ldots, s_{in_r} \}$ , where $n_r$  is the number or robots and $s_{ik}$ is the timestamp that robot i communicated with robot j.
2. Create bundle:
    1. Greedy bidding for target list. The bid is calculated the same way as before and the bid is compared to the value in y. 
3. Consensus: 
    1. Send lists z, y to neighbouring robots and recieves theirs. 
    2. Update the information after what the highest bids are in the y. If robot i has assigned it self to a task which has a better bidder, all tasks after tha b has to be reallocated. with the bundle creating loop. 
    3. Update the time stamp of communication in s. 

So the robots initially is in bundle phase and then enters it if :

- Completed event.
- New event is created ?
- Conflict in consesus.

The robot enters consensus phase if:

- If within communication range of another robot and timestamp ie enough long ago

[https://arxiv.org/pdf/1806.04836](https://arxiv.org/pdf/1806.04836)
