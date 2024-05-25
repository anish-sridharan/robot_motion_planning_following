import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class Plotting:
    # Class to help plot obstacles and lines, and also show live animation of planning.
    def __init__(self):
        plt.ion()
        self.fig = plt.figure(figsize=(16, 8))
        self.ax = self.fig.add_subplot(111)

    def plot_line(self, line_x: list, line_y: list, color="green") -> None:
        """
        Plots a line given x,y coordinates.
        line_x: [x1,x2,x3......] x coordinates of the line
        line_y: [y1,y2,y3......] y coordinates of the line
        color: color of line while plotting

        """
        self.ax.plot(line_x, line_y, color)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def plot_obstacles(self, obstacle_list: list) -> None:
        """
        Plot all obstacles as a rectangle.
        obstacle_list: [ox,os,oy] center and shape of obstacles present inside the map (m,m,m)
        """
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

    def stop_plot(self) -> None:
        # Shuts down the animation
        plt.ioff()
        plt.show()
