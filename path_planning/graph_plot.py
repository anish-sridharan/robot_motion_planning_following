import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class Plotting:
    def __init__(self):
        plt.ion()
        self.fig = plt.figure(figsize=(16, 8))
        self.ax = self.fig.add_subplot(111)

    def plot_line(self, line_x, line_y, colour="green"):
        self.ax.plot(line_x, line_y, colour)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def plot_obstacles(self, obstacle_list):
        for i in range(len(obstacle_list)):
            self.ax.add_patch(
                Rectangle(
                    (
                        obstacle_list[i][0] - obstacle_list[i][2] / 2,
                        obstacle_list[i][1] - obstacle_list[i][2] / 2,
                    ),
                    obstacle_list[i][2],
                    obstacle_list[i][2],
                    edgecolor="black",
                    facecolor="black",
                    fill=True,
                    lw=5,
                )
            )
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def stop_plot(self):
        plt.ioff()
        plt.show()
