import numpy
from scipy.interpolate import interp1d

def computePathFromWaypoint(startX, startY, step =0.1):

    # The purpose of this function is to generate some kind of a path using linear interpolation
    # This is a simple and easy to process optimization tool we can use for our MPC (temporarily, at least until SAL is implemented)

    """
    Args:
        startX (array-like object): 1 Dimensional array of x positions
        startY (array-like object): 1 Dimensional array of y positions
        step (float): interpolation step (Length of each step in our path)

    Returns:
        3xN numpy array
        The rows will be represented by:
        X coordinates of the interpolated path
        Y coordinates of the interpolated path
        Angles of each point on the path
    """

def getNN(state, path):

def getTrajectory(state, path, targetSpeed, horizionDuration, horizonTimeStep):