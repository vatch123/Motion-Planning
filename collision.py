import numpy as np

def collision_check(line, blocks, num=4):

    def point_in_block(point, block):
        if block[0] <= point[0] <= block[3] and \
           block[1] <= point[1] <= block[4] and \
           block[2] <= point[2] <= block[5]:
            return True
        return False 
    
    # Generate points on the line segment
    point1 = line[:,0]
    point2 = line[:,1]
    line_points = np.vstack([np.linspace(point1[i], point2[i], num) for i in range(3)])

    for block in blocks:
        for point in line_points.T:
            if point_in_block(point, block):
                return True
    
    return False
