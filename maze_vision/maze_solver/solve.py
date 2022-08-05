import time
import cv2
import numpy as np
from PIL import Image

from .mazes import Maze
from .factory import SolverFactory


def MazeSolver(input_img: np.ndarray, solver: Maze):

    ## Convert input image from CV2 to PIL formatting

    
    img = Image.fromarray(input_img)


    ## Create the maze (and time it) - for many mazes this is more time consuming than solving the maze
    print ("Creating Maze")
    t0 = time.time()
    maze = Maze(img)
    t1 = time.time()
    print ("Node Count:", maze.count)
    total = t1-t0
    print ("Time elapsed:", total, "\n")


    ## Run solver
    t0 = time.time()
    [result, stats] = solver(maze)
    t1 = time.time()
    total = t1-t0


    ## Print solve stats
    print ("Nodes explored: ", stats[0])
    if (stats[2]):
        print ("Path found, length", stats[1])
    else:
        print ("No Path Found")
    print ("Time elapsed: ", total, "\n")


    ## Post-Process Solution
    resultpath = [n.Position for n in result]
    resultpath_flipped = []
    for node in resultpath:
        resultpath_flipped.append((node[1], node[0]))


    ## Rendering Image of solve maze path
    output_img = cv2.cvtColor(input_img, cv2.COLOR_GRAY2RGB)
    length = len(resultpath_flipped)

    for i in range(0, length - 1):
        a = resultpath_flipped[i]
        b = resultpath_flipped[i+1]

        # Blue - red
        r = int((i / length) * 255)
        px = (r, 0, 255 - r)

        if a[0] == b[0]:
            # Ys equal - horizontal line
            for x in range(min(a[1],b[1]), max(a[1],b[1])):
                output_img[x,a[0]] = px
        elif a[1] == b[1]:
            # Xs equal - vertical line
            for y in range(min(a[0],b[0]), max(a[0],b[0]) + 1):
                output_img[a[1],y] = px
    
    ## Output Solution
    return resultpath_flipped, output_img.astype('uint8')


def main():
    sf = SolverFactory()

    c = cv2.imread('/home/buynak.9/AA_DEVL/ws_mzrun/debug_step5_maurer_cleaned.tiff')
    result_path, result_img = MazeSolver(c, sf, 'dijkstra')

    cv2.imshow('debug', result_img)
    cv2.waitKey()

if __name__ == "__main__":
    main()

