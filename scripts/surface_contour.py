#!/usr/bin/env python

## IMPORTS
import numpy as np
from scipy.interpolate import griddata


##
def plot_ply(file):
    # Imports
    from plyfile import PlyData, PlyElement
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    # Load & Filter Data
    plydata = PlyData.read(file)
    x = plydata.elements[0].data['x']
    y = plydata.elements[0].data['y']
    z = plydata.elements[0].data['z']
    
    # Generate Figure & Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.scatter(x, y, z, s=5)
    
    plt.show()


class SurfaceContour():
    
    def __init__(self, filePath):
        self.surface = self.load_ply(filePath)
        
        #self.plot_surface_colormap(self.surface)
        self.plot_surface_colormap(self.surf_interpolated)
    
    def load_ply(self, filePath):
        """
        Import PLY file and return a numpy array with points and
        :param filePath: Explicit filepath to a ply file
        :return: numpy depth array.
        """
        
        from plyfile import PlyData, PlyElement
        
        # Import Lists XYZ of components
        plydata = PlyData.read(filePath)
        x = plydata.elements[0].data['x']
        y = plydata.elements[0].data['y']
        z = plydata.elements[0].data['z']
        
        points = np.stack((x,y,z), axis=-1)
        
        # Find Extremes (min have been set to zero)
        # Todo: refactor to use points array instead of xyz
        limits = np.array( [[min(x), max(x) ],
                            [min(y), max(y) ],
                            [min(z), max(z) ]] )
        
        # Ranges = [x_range, y_range, z_range]
        ranges = [ limits[0,1]-limits[0,0], limits[1,1]-limits[1,0], limits[2,1]-limits[2,0] ]
        
        # Set & Apply Scale
        # todo; when & how to scale. earlier than this line? move up?
        scale = 1 # 1=meter (10=cm)
        points = points / scale
        
        mesh_increment = 100/scale  #(1000 = 1mm increment, 100=1cm increment)
        
        # Create Blank XY Field
        # Truncate limit values. Arbitrary buffer of 10 cells greater in each direction
        x_edge = int( max(abs(limits[0,0]), abs(limits[0,1])) * mesh_increment ) + 10
        y_edge = int( max(abs(limits[1,0]), abs(limits[1,1])) * mesh_increment ) + 10
        
        surf = np.zeros([x_edge*2, y_edge*2])
        
        # Filter XY points onto new surf grid
        points_meshed = points * mesh_increment
        for point in points_meshed:
            point = np.rint(point).astype(int)          #todo; this approach is memory wasteful, look into more efficient method
            loc_x = point[0] + x_edge
            loc_y = point[1] + y_edge
            surf[loc_x, loc_y] = point[2]

        np.savetxt("surf_debug.csv", surf, delimiter=',')
        
        # Points at which to perform interpolation for Z-value
        (x_ext, y_ext) = surf.shape
        xi = np.arange(0, x_ext, 1)
        yi = np.arange(0, y_ext, 1)
        (xi, yi) = np.meshgrid(xi, yi)
        
        #print(points_meshed[:,:-1].astype(int))
        #print(points_meshed[:,2].astype(int))
        # print(xi)
        # print(yi)
        
        # Apply interpolation
        z_interpolated = griddata(points_meshed[:, :-1].astype(int), points_meshed[:,2].astype(int), (xi,yi), method='linear')
        print(z_interpolated.shape)
        
        #print(z_interpolated)
        
        np.savetxt("debug.csv",z_interpolated, delimiter=',')
        
        
        
        self.surf_interpolated = z_interpolated

        
        
        
        ### THINGS TO CHECK.
        # Is the origin place in the wrong spot? Like comparing surf to the z_interp it looks like the origins are incorrect.
        # Looks like row & column might have been swapped. The original surf increased from mid to bottom. Z_interp goes left to right.
        
        
        
        
        
        
        
        
        
        
        return surf
    
    def plot_surface(self):
        """
        Assumes center of array is origin. Plots all non-zero Z-values
        """
        
        # Imports
        from plyfile import PlyData, PlyElement
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
    
        # Generate Figure & Plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    
        y_count, x_count = self.surface.shape
            
        x = np.arange(0, x_count, 1)
        y = np.arange(0, y_count, 1)
        
        #####################################
        y_count, x_count = self.surface.shape
        
        #print(y_count,x_count)
        #print(self.surface.size)
        
        X,Y,Z = [],[],[]
        for y_index, row in enumerate(self.surface):
            y_index = y_count - y_index
            for x_index, cell in enumerate(row):
                if cell:
                    X.append(x_index)
                    Y.append(y_index)
                    Z.append(cell)
        #print(len(X),len(Y),len(Z))
        
        #ax.scatter(X,Y,Z)
        ax.plot_trisurf(X, Y, Z)
        plt.show()
        
    def plot_surface_colormap(self, surf):
        # Imports
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
    
        # Generate Figure
        fig, (ax1,ax2) = plt.subplots(1,2)
        
        # Axes
        #(x_axis, y_axis) = np.divide(np.shape(surf),2)

        # Plot
        psm = ax1.pcolormesh(surf)
        fig.colorbar(psm, ax=ax1)
        ax1.set_title("Color Height Map of obj.Surface")
        ax2.contour(surf)
        ax2.set_title("Contour Height Map of obj.Surface")

        plt.show()
        
        return 1
        
    def self_report(self, array):
        """"
        Print dimensions, properties, and more about object
        """
        # todo
        print("TODO")



## MAIN
def main():
    # Load in File
    input_filepath = "/tmp/SAMPLE/PATH/maze-runner/sample_content/surfaces/sample_airfoil_remeshed.ply"
    
    # Quick Test. Plot Point Cloud using MatLibPlot
    #plot_ply_points(input_filepath)
    
    # Create Class Instance
    surface = SurfaceContour(input_filepath)



if __name__ == '__main__':
    main()
