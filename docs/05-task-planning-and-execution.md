# Part V: Task Planning and Execution

## Why Task Planning?

Every tool built so far solves one problem at a time: a controller drives to a single goal, a planner finds one path around one set of obstacles. But a real task is rarely just one of those in isolation — it's a sequence of different behaviors, each active only under the right conditions. A robot might need to follow a light source, until an obstacle gets in the way, at which point it should switch to avoiding it, then go back to following the light once the obstacle is cleared.

None of the controllers or planners from earlier chapters know how to make that switch on their own — they just do the one thing they were built for. What's missing is a layer above them that decides *which* behavior should be running right now, and *when* to switch to another one. That's what this chapter covers, starting with the simplest tool for the job: the **state machine**.

## State Machines

A state machine describes a system as a finite set of **states**, together with the rules for moving between them. Formally, it's the tuple $(S, \Sigma, \delta, s_0, F)$:

- $S$: a finite set of **states** — the distinct behaviors or modes the robot can be in
- $s_0$: the **initial state**, the one the robot starts in
- $\Sigma$: the **input alphabet** — the set of conditions or events the robot can react to
- $\delta$: the **transition function**, $\delta: S \times \Sigma \rightarrow S$, mapping a state and an input to the next state
- $F$: the set of **final states**, where the machine stops

<p markdown="1" style="text-align:center;">
![A state machine with four states: Follow light (initial), Avoid obstacle, Follow wall, and Stop (final), connected by labeled transitions](assets/images/state_machine_example.svg)
</p>

As the robot's environment gets more dynamic, more transitions are needed to react to it, and the diagram above quickly turns into a tangle of crossing arrows. Adding a single new state makes this worse: every state needs its own outbound transition for every condition it might face, so the number of transitions grows combinatorially with the number of states.

## Behavior Trees

State machines get even harder to manage once failure handling is added — every step also needs a way to retry or recover when it doesn't succeed, which means wiring up still more transitions by hand. **Behavior trees** sidestep this by organizing behavior as a tree of composable nodes instead of a flat set of states and transitions.

The actual implementation lives in the **leaves** — the individual actions the robot performs, each of which either **succeeds** or **fails**.

A **sequence** node (`→`) groups leaves together and runs them in order, advancing only as long as each one succeeds; if any leaf fails, the whole sequence fails, and it only succeeds once every leaf has.

A **selector** node (`?`) instead tries its children left to right and stops at the first one that succeeds, only failing if all of them do — a natural way to express a fallback, like trying to avoid an obstacle before falling back to just following the light.

<p markdown="1" style="text-align:center;">
![A behavior tree for the same light-following robot: a selector node tries four behaviors in order, one of which, Avoid Obstacle, is itself a sequence of a condition and an action](assets/images/behavior_tree_navigate.svg)
</p>

Each leaf actually returns one of three results: success, failure, or **running**, if it isn't done yet. Instead of letting a slow leaf block everything else, the whole tree gets re-checked on a fixed interval called a **tick** — say, every 32 ms — picking back up from whichever leaf was still running last time. This is what makes behavior trees reactive in real time, and it's also what makes it possible to run several branches at once.

A **parallel** node (`⇉`) does exactly that: instead of trying its children one at a time like a sequence or selector, it runs all of them at the same time, succeeding based on a policy set on the node — every child must succeed, only one has to, or at least $n$ of them do. In our light-following robot, one branch can keep sensing the environment while another branch, the `Navigate` tree from before, decides what to do with it.

<p markdown="1" style="text-align:center;">
![A parallel node running a sensing leaf and the Navigate tree at the same time, sharing data through a blackboard](assets/images/behavior_tree_parallel.svg)
</p>

Running side by side only works if the two branches can share what they find, so parallel nodes are usually paired with a **blackboard** — memory any branch can read or write. Here, `Sense Environment` writes the light level and obstacle distance it measures, and `Navigate` reads them back on its next tick.

## Examples in Practice

### Finite State Machines

The transition function $\delta$ from the light-following state machine maps directly onto a dictionary keyed by `(state, event)` pairs, exactly like the graphs from the previous chapter:

```python
transitions = {
    ('Follow light', 'Obstacle detected'): 'Avoid obstacle',
    ('Avoid obstacle', 'Obstacle free'):   'Follow light',
    ('Avoid obstacle', 'Light decreases'): 'Follow wall',
    ('Follow wall', 'Light increases'):    'Avoid obstacle',
    ('Follow light', 'Under light'):       'Stop',
}

def step(state, event):
    return transitions.get((state, event), state)  # ignore unhandled events

state = 'Follow light'
events = ['Obstacle detected', 'Obstacle free', 'Under light']

for event in events:
    state = step(state, event)
    print(state)

# Avoid obstacle
# Follow light
# Stop
```

### Behavior Trees with `py_trees`

The `Navigate` tree from earlier maps onto [py_trees](https://py-trees.readthedocs.io/), a Python behavior tree library used widely in robotics. Conditions and actions are just behaviours that return `SUCCESS` or `FAILURE`, grouped under `Sequence` and `Selector` composites:

```python
import py_trees
from py_trees.common import Status

class Condition(py_trees.behaviour.Behaviour):
    def __init__(self, name, check):
        super().__init__(name)
        self.check = check

    def update(self):
        return Status.SUCCESS if self.check() else Status.FAILURE

class Action(py_trees.behaviour.Behaviour):
    def __init__(self, name, act):
        super().__init__(name)
        self.act = act

    def update(self):
        self.act()
        return Status.SUCCESS

handle_obstacle = py_trees.composites.Sequence(name="Handle Obstacle", memory=False, children=[
    Condition("Obstacle Detected?", obstacle_detected),
    Action("Avoid Obstacle", avoid_obstacle),
])

handle_wall = py_trees.composites.Sequence(name="Handle Wall", memory=False, children=[
    Condition("Light Decreasing?", light_decreasing),
    Action("Follow Wall", follow_wall),
])

navigate = py_trees.composites.Selector(name="Navigate", memory=False, children=[
    handle_obstacle,
    handle_wall,
    Action("Follow Light", follow_light),
])

tree = py_trees.trees.BehaviourTree(navigate)

while True:
    tree.tick()  # re-evaluate the whole tree once per tick
```

`obstacle_detected`, `light_decreasing`, `avoid_obstacle`, `follow_wall`, and `follow_light` are the same sensing and control functions from earlier chapters — the tree just decides which of them to call, and when.




