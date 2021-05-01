import numpy as np
import csv
from scipy.spatial import distance
from scipy.stats import chi2


# reading data from a csv file --> point cloud (list of coordinates)
# input: f --> string file name of CSV file with Lidar Data
# return: coords --> 2D list containing lists of coordinates for each point
def get_data(f):
    coords = []
    with open(f, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            point = [row[0], row[1], row[2]]
            coords.append(point)
    return coords


# returns a list of mahalanobis distances for each point from the origin
# input: points --> 2D list containing lists of coordinates for each point
# return: distances --> list of Mahalanobis Distances (double) for each point from the origin
def get_distances(points):
    data = np.array(points)
    cov = np.cov(data, rowvar=False)  # rowvar false because each row is one point and each column is a variable(x,y,z)
    inv_cov = np.linalg.inv(cov)
    distances = []
    for point in points:
        origin = np.array([0, 0, 0])
        p = np.array(point)
        distances.append(distance.mahalanobis(origin, p, inv_cov))
    return distances


# takes in mahalanobis distances for each point, and through a Chi^2 analysis, determines the outliers
# input: points --> 2D list containing lists of coordinates for each point
# return: O --> 2D list containing lists of coordinates for each outlier
def get_outliers(points):
    O = []  # list of outliers
    distances = get_distances(points)
    degrees = 2 * (len(distances) - 1)  # df = (r-1)(c-1) [r = # of rows, c = # of columns]
    chi2.ppf((1 - 0.05), df=degrees)  # Chi^2 analysis with 0.05 significance and (r-1)(c-1) degrees of freedom
    p_values = 1 - chi2.cdf(distances, degrees)
    for i in range(len(p_values)):
        if p_values[i] < 0.05:
            O.append(points[i])
    return O


if __name__ == '__main__':
    filename = 'test.csv'
    point_cloud = get_data(filename)
    outliers = get_outliers(point_cloud)
    for o in outliers:
        print(o, '\n')
