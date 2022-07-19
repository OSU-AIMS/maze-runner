# A simple factory class that imports and returns a relevant solver when provided a string
# Not hugely necessary, but reduces the code in solve.py, making it easier to read.

class SolverFactory:
    def __init__(self):
        self.Default = "breadthfirst"
        self.Choices = ["breadthfirst","depthfirst","dijkstra", "astar","leftturn"]

    def createsolver(self, type: str) -> tuple:
        if type == "leftturn":
            from .leftturn import solve
            return ("Left turn only", solve)
        elif type == "depthfirst":
            from .depthfirst import solve
            return ("Depth first search", solve)
        elif type == "dijkstra":
            from .dijkstra import solve
            return ("Dijkstra's Algorithm", solve)
        elif type == "astar":
            from .astar import solve
            return ("A-star Search", solve)
        else:
            from .breadthfirst import solve
            return ("Breadth first search", solve)
