import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import time
from Simulator import Simulator

class SimulationRunner:
    def __init__(self, control_system, trajectory, sim_time=200, show_animation=True):
        self.control_system = control_system
        self.sim_time = sim_time
        self.trajectory = trajectory
        self.show_animation = show_animation
        self.simulation_done = False

        # Создание симулятора с использованием функций из ControlSystem
        self.simulator = Simulator(
            self.sim_time,
            self.control_system.calculate_control,
            self.control_system.calculate_target_data,
            self.trajectory
        )

        if self.show_animation:
            # Инициализация фигуры и осей для графика
            self.fig, self.ax = plt.subplots(figsize=(5, 5))

    def run_simulation(self):
        while not self.simulation_done:
            self.simulator.measure()
            self.simulator.calculate_target_data()
            self.simulator.calculate_control()
            self.simulator.check_target()

            if not self.simulator.move():
                print("Simulation stopped due to collision")
                break

            self.simulator.accumulate_data()
            self.simulation_done = self.simulator.check_simulation_done()

            if self.show_animation:
                # Очищаем предыдущие данные с графика
                self.ax.cla()

                # Отображение положения робота
                self.ax.plot(self.simulator.X_array, self.simulator.Y_array, label="Путь робота")

                # Отображение стен
                for wall in self.simulator.walls:
                    x_coords = [wall[0][0], wall[0][1]]
                    y_coords = [wall[1][0], wall[1][1]]
                    self.ax.plot(x_coords, y_coords, 'k-', linewidth=2)  # Стены

                # Текущая позиция робота
                self.ax.scatter(self.simulator.robot._x, self.simulator.robot._y, color='red', label='Робот')

                # Целевая точка
                self.ax.scatter(self.simulator.target[0], self.simulator.target[1], color='blue', label='Цель')

                self.ax.set_xlim([0, 11])
                self.ax.set_ylim([0, 11])
                self.ax.set_title(f"Время: {self.simulator.time:.1f} сек | Целей пройдено: {self.simulator.points_clear}")
                self.ax.set_xlabel("X")
                self.ax.set_ylabel("Y")
                self.ax.grid(True)
                self.ax.legend()

                # Обновляем график
                clear_output(wait=True)
                display(self.fig)

                # Небольшая пауза для анимации
                time.sleep(0.0001)

        # Получение всех данных по завершении симуляции
        simulation_data = self.simulator.get_simulation_data()
        self.plot_final_trajectory(simulation_data)

    def plot_final_trajectory(self, simulation_data):
        # Построение итоговой траектории на отдельном графике
        plt.figure(figsize=(6, 6))
        plt.plot(simulation_data["X"], simulation_data["Y"], label="Итоговая траектория", color='green')
        for wall in self.simulator.walls:
            x_coords = [wall[0][0], wall[0][1]]
            y_coords = [wall[1][0], wall[1][1]]
            plt.plot(x_coords, y_coords, 'k-', linewidth=2)  # Стены
        plt.title("Итоговая траектория робота")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.xlim([0, 11])
        plt.ylim([0, 11])
        plt.grid(True)
        plt.legend()
        plt.show()
