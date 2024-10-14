import math
from maze_generator import generate_maze, maze_to_walls, segment_intersect_walls, sense_wall

import Roko2


def distance(point, robot):
    return math.sqrt((point[0] - robot._x)**2 + (point[1] - robot._y)**2)

class Simulator:
    def __init__(self, sim_time, calculate_control, calculate_target_data, trajectory=[[0, 0]], width = 5, height = 5):
        maze = generate_maze(width, height)
        self.walls = maze_to_walls(maze)

        self.sim_time = sim_time
        self.time = 0
        self.time_delta = 0.1
        self.trajectory = trajectory
        self.target = self.trajectory[0]
        self.points_clear = 0
        self.robot = Roko2.Roko2(1.5, 1.5, 0, 0, 0)
        self.params = self.robot.get_measurements()
        self.distances = (0, 0, 0)
        self.trajectory_cleared = False
        self.velocity_control = 0.0
        self.heading_control = 0.0

        self.calculate_control_c = calculate_control
        self.calculate_target_data_c = calculate_target_data

        self.X_array = []
        self.Y_array = []
        self.time_array = []
        self.speed_array = []
        self.heading_array = []
        self.heading_control_array = []
        self.distance_checked = []

        self.target_data = []

    def measure(self):
        self.params = self.robot.get_measurements()
        self.distances = (sense_wall(self.walls, (self.robot._x, self.robot._y), self.robot._heading+math.pi/2),
                          sense_wall(self.walls, (self.robot._x, self.robot._y), self.robot._heading),
                          sense_wall(self.walls, (self.robot._x, self.robot._y), self.robot._heading-math.pi/2))

    def calculate_target_data(self):
        self.target_data = self.calculate_target_data_c(
            self.params.x,
            self.params.y,
            self.target[0],
            self.target[1],
            self.params.heading
        )

    def calculate_control(self):
        (self.velocity_control, self.heading_control) = self.calculate_control_c(
            self.target_data,
            self.distances
        )

    def check_target(self):
        target_reached = distance(self.target, self.robot) < 0.5
        if target_reached:
            self.distance_checked.append(distance(self.target, self.robot))
            self.points_clear += 1
            if self.points_clear < len(self.trajectory):
                self.target[0] = self.trajectory[self.points_clear][0]
                self.target[1] = self.trajectory[self.points_clear][1]
            else:
                self.target[0] = 0
                self.target[1] = 0
                self.trajectory_cleared = True

    def move(self):
        self.robot.set_motion(self.velocity_control, self.heading_control)
        self.robot.update()
        if (self.time > 0 and segment_intersect_walls(self.walls, self.X_array[-1], self.Y_array[-1], self.params.x, self.params.y)):
            print("DANGER! COLLISION!!")
            return False
        self.time += self.time_delta
        return True

    def accumulate_data(self):
        self.X_array.append(self.robot._x)
        self.Y_array.append(self.robot._y)
        self.time_array.append(self.time)
        self.speed_array.append(self.params.velocity)
        self.heading_array.append(self.params.heading)
        self.heading_control_array.append(self.heading_control)

    def check_simulation_done(self):
        return (self.time >= self.sim_time) or self.trajectory_cleared

    def get_simulation_data(self):
        return {
            "X": self.X_array,
            "Y": self.Y_array,
            "Time": self.time_array,
            "Speed": self.speed_array,
            "Heading": self.heading_array,
            "Heading Control": self.heading_control_array,
            "Distances Checked": self.distance_checked
        }
