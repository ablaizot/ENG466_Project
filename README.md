# What to do
- make sure the basic implementation works (Aymeric)
- implement movement/collision 
- estimate task utility and with regard to energy budget (Petr)
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
