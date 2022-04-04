import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        max_solution_time = 0
        ### custom constraints
        # guided constraint for task 1.2
        # constraints.append(self.create_constraint(0, [(1, 5)], 4))
        # guided constraint for task 1.3
        # constraints.append(self.create_constraint(1, [(1, 2),(1,3)], 1))
        # guided constraint for task 1.4
        # constraints.append(self.create_constraint(0, [(1, 5)], 10))
        # constraints for minimal path in task 1.5
        # Minimal path length is 8
        # The constraints force agent 1 to drop to the small corridor in the maze, in order to let agent 0 pass him,
        # so that they can each avoid any collision on their path to their respective goal
        # constraints.append(self.create_constraint(1, [(1, 2)], 2))
        # constraints.append(self.create_constraint(1, [(1, 3)], 2))
        # constraints.append(self.create_constraint(1, [(1, 4)], 2))
        ### /custom constraints

        for i in range(self.num_of_agents):  # Find path for each agent
            # There can only be as many steps as the size of the board, we add the maximal number of
            # steps an agent will make to buffer cases where agent need extra steps to avoid collisions
            max_number_of_timesteps = max_solution_time + len(self.my_map) * len(self.my_map[0])
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None or len(path) >= max_number_of_timesteps:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            # If this agent takes longer than all previous agents to reach his goal we update
            max_solution_time = max_solution_time if max_solution_time > len(path) else len(path)
            for timestep in range(max_number_of_timesteps + 1):
                for agent_index in range(i, self.num_of_agents):
                    # Add a vertex constraint to each future agent in the runtime of the current agent
                    if timestep < len(path):
                        constraints.append(self.create_constraint(agent_index, [path[timestep]], timestep))
                        ### Part 2.2
                        # Add edge constraints to each future agent
                        if timestep > 0:
                            constraints.append(
                                self.create_constraint(agent_index, [path[timestep], path[timestep - 1]], timestep))
                        ### /Part 2.2
                    else:
                        # Add a vertex constraint to each future agent, even after the current agent has
                        # finished his run
                        constraints.append(self.create_constraint(agent_index, [path[-1]], timestep))
            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result

    @staticmethod
    def create_constraint(agent: int, loc: list[tuple], timestep: int):
        # print(f'Agent {agent} can\'t occupy {loc} at timestep: {timestep}')
        return {'agent': agent, 'loc': loc, 'timestep': timestep}
