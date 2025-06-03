import numpy as np


def apply_Rt_batch(points, R, t):
    p = R @ points.T + t[:, None]
    return p.T


def apply_transformation_batch(points, T_matrix):
    # make point homogenous
    points = np.hstack([points, np.ones((points.shape[0], 1))])
    p = T_matrix @ points.T
    return p[:3].T


if __name__ == "__main__":
    # random points, R, t
    points = np.random.rand(1000, 3)
    R = np.random.rand(3, 3)
    t = np.random.rand(3)

    # create the transformation matrix from R, t
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    # apply the transformation
    p1 = apply_Rt_batch(points, R, t)
    p2 = apply_transformation_batch(points, T)

    # check if the results are the same
    print(np.allclose(p1, p2))
