#!/usr/bin/env python3
#


import cv2
import numpy as np

class Maze:
    class Node:
        def __init__(self, location: tuple) -> None:
            self.Position = location
            self.Neighbours = [None, None, None, None]

    def __init__(self, maze_img) -> None:
        ''' Preprocess the maze to convert into a node frame.'''

        maze_w = maze_img.shape[0]
        maze_h = maze_img.shape[1]

        self.start = None
        self.end   = None

        # Find start node
        for ix, v in np.ndenumerate(maze_img[0,:]):

        
        # Midsection Node Identificaiton
        for iy, v in range(1, maze_h-1):
            pass


        for (row, col), v in np.ndeumerate(maze_img):

            # First row
            if row ==0 and v > 0: 
                self.start = self.Node((0, ix))
                break

            # Middle Section


            # End row
            if row == maze_h-1 and v > 0:
                



def solve(input_file, factory, method, output_file):






if __name__ == "__main__":
    solve()