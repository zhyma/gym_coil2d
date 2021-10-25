import numpy as np
import matplotlib.pyplot as plt 

def bezier_3(p, gran=0.01):
    # 3-order bezier curve with 4 control points
    print(p)
    c = lambda t: (1 - t)**3*p[0] + 3*t*(1 - t)**2*p[1] + 3*t**2*(1-t)*p[2] + t**3*p[3]
    points = np.array([c(t) for t in np.arange(0, 1, gran)])
    return points

if __name__ == '__main__':

    p = np.array([[0, 0], [0, 8], [8, 10], [6, 5]])
    points = bezier_3(p, gran=0.01)

    x_b, y_b = points[:, 0], points[:, 1]
    x_p, y_p = p[:, 0], p[:, 1]

    plt.plot(x_b, y_b, 'b-')
    plt.plot(x_p, y_p, 'r.')
    plt.show()