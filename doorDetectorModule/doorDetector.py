import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import pandas as pd

# modlule for handling door detection
class DoorDetector:
    def __init__(self, point_map_path):
        points = pd.read_csv(point_map_path)

        # set member values of point map dots
        self.x = points[(points.columns)[0]].values
        self.y = points[(points.columns)[1]].values
        self.z = points[(points.columns)[2]].values

        self.points_3d = np.array(list(zip(self.x, self.y, self.z)))
        self.points_2d = np.array(list(zip(self.x, self.y)))

    # draw_distribution_map = True for graph information plotting
    # function return estimated door location based on gaussian distribution
    def find_door_coordinates(self, draw_distribtion_map=False):
        # Sample parameters
        mu = np.mean(self.points_2d, axis=0)
        sigma = np.cov(np.stack((self.x, self.y)))
        rv = multivariate_normal(mu, sigma)

        # Bounds parameters
        x_abs = 2.5
        y_abs = 2.5
        x_grid, y_grid = np.mgrid[-x_abs:x_abs:.02, -y_abs:y_abs:.02]

        pos = np.empty(x_grid.shape + (2,))
        pos[:, :, 0] = x_grid
        pos[:, :, 1] = y_grid

        levels = np.linspace(0, 1, 40)
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        # Removes the grey panes in 3d plots
        ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

        # The heatmap
        ax.contourf(x_grid, y_grid, 0.1 * rv.pdf(pos),
                    zdir='z', levels=0.1 * levels, alpha=0.9)

        # The wireframe
        ax.plot_wireframe(x_grid, y_grid, rv.pdf(
            pos), rstride=10, cstride=10, color='k')

        # The scatter. Note that the altitude is defined based on the pdf of the
        # random variable
        ax.scatter(self.points_2d[:, 0], self.points_2d[:, 1], 1.1 * rv.pdf(self.points_2d), c='r')

        points_2d_1 = enumerate(list(self.points_2d))
        door_point_idx, door_point = min(points_2d_1, key=lambda pair: rv.pdf(pair[1]))

        ax.scatter(door_point[0], door_point[1], c='g', s=1000)

        ax.legend()
        ax.set_title("Find lowest point of the gaussian")
        ax.set_xlim3d(-x_abs, x_abs)
        ax.set_ylim3d(-y_abs, y_abs)
        ax.set_zlim3d(0, 1)

        if draw_distribtion_map:
            plt.show()

        return self.points_3d[door_point_idx]

