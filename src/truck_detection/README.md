# Truck Detection

## LIDAR Data Processing
- The system receives LIDAR point cloud data representing the surroundings.

## Point Analyzer
- The PointAnalyzer class processes the point cloud data to identify valid segments that represent the back of the front truck. It uses linear regression to analyze the points and determine the position of the truck.
- All points that produce the same angle within a tolerance are grouped together.
- The system then takes the average of these points to afterwards count the distance to next truck's position.

## Dependency
- Python [Numpy](https://pypi.org/project/numpy/) package
- Python [Scipy](https://pypi.org/project/scipy/) package

