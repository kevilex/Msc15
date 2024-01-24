import pyvista as pv

points = pv.read('points.ply')

with open('points_output.txt', 'w') as file:
    for each in points.points:
        file.write(str(each[0]) + ' ' + str(each[1]) + ' ' + str(each[2]) + ' ' + '\n')