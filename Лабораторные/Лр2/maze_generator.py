import random
import math
import time
import matplotlib.pyplot as plt


def generate_maze(width, height):
    # Создаем сетку из стен
    maze = [[1] * (2 * width + 1) for _ in range(2 * height + 1)]

    # Рекурсивный обход
    def recursive_backtracking(x, y):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        random.shuffle(directions)

        for dx, dy in directions:
            nx, ny = x + 2 * dx, y + 2 * dy
            if 0 < nx < 2 * height and 0 < ny < 2 * width:
                if maze[nx][ny] == 1:
                    maze[x + dx][y + dy] = 0
                    maze[nx][ny] = 0
                    recursive_backtracking(nx, ny)

    # Стартовая точка
    start_x, start_y = 1, 1
    maze[start_x][start_y] = 0
    recursive_backtracking(start_x, start_y)

    return maze


def display_maze(maze):
    # Выводим лабиринт на экран
    plt.imshow(maze, cmap='Greys')
    plt.axis('off')
    plt.show()


def maze_to_walls(maze):
    walls = []
    for i, row in enumerate(maze):
        for j, cell in enumerate(row):
            if cell == 0:
                x, y = j, i
                if maze[i-1][j] == 1:
                    walls.append(((x, x+1), (y, y)))
                if maze[i+1][j] == 1:
                    walls.append(((x, x+1), (y+1, y+1)))
                if maze[i][j-1] == 1:
                    walls.append(((x, x), (y, y+1)))
                if maze[i][j+1] == 1:
                    walls.append(((x+1, x+1), (y, y+1)))
    return walls


def sense_wall(walls, point, angle_rad):
    x, y = point
    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)

    distance = 0.0
    while True:
        x += dx * 0.01
        y += dy * 0.01
        distance += 0.01

        for wall in walls:
            x1, x2 = wall[0]
            y1, y2 = wall[1]

            # Check for vertical wall segment
            if x1 == x2:
                if min(y1, y2) <= y <= max(y1, y2) and abs(x - x1) < 0.1:
                    return distance

            # Check for horizontal wall segment
            if y1 == y2:
                if min(x1, x2) <= x <= max(x1, x2) and abs(y - y1) < 0.1:
                    return distance

        # Breaking condition to avoid infinite loop in case of no walls
        if distance > 1000:
            return None


def segment_intersect_walls(walls, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    segment_length = math.sqrt(dx*dx + dy*dy)
    dx /= segment_length
    dy /= segment_length

    distance = 0.0
    while True:
        x = x1 + dx * distance
        y = y1 + dy * distance
        distance += 0.1

        for wall in walls:
            wx1, wx2 = wall[0]
            wy1, wy2 = wall[1]

            # Check for vertical wall segment
            if wx1 == wx2:
                if min(wy1, wy2) <= y <= max(wy1, wy2) and abs(x - wx1) < 0.1:
                    return True

            # Check for horizontal wall segment
            if wy1 == wy2:
                if min(wx1, wx2) <= x <= max(wx1, wx2) and abs(y - wy1) < 0.1:
                    return True

        # Check if we have reached the end of the segment
        if distance >= segment_length:
            return False


def plot_maze(walls, points=None, angles_rad=None, distances=None):
    for wall in walls:
        plt.plot(wall[0], wall[1], 'k-', lw=2)

    for point, angle_rad, distance in zip(points, angles_rad, distances):
        if point and angle_rad is not None and distance:
            x, y = point
            end_x = x + math.cos(angle_rad) * distance
            end_y = y + math.sin(angle_rad) * distance
            plt.plot([x, end_x], [y, end_y], 'r-', lw=2, label='Sensed Direction')

    plt.axis('off')
    plt.axis('equal')
    plt.legend(loc='upper right')
    plt.show()


if __name__ == '__main__':
    maze = generate_maze(5, 5)
    walls = maze_to_walls(maze)

    # Координаты точки и угол в радианах для демонстрации
    points = [(1.5, 1.5), (1.5, 1.5), (1.5, 1.5)]
    angles_rad = [math.radians(0), math.radians(90), math.radians(-90)]  # 45 градусов в радианах

    # Получаем геометрическое расстояние до ближайшей стены
    distances = []
    for point, angle_rad in zip(points, angles_rad):
        distances.append(sense_wall(walls, point, angle_rad))

    # Отрисовка лабиринта и линии направления
    plot_maze(walls, points, angles_rad, distances)
