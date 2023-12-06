#import sys
import math
import heapq

# This class was used in an assignment for another class assignment, and I just 
# copied and pasted the code from that assignment to use this data structure for this assignment.
class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.
    """
    def  __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        # If item already in priority queue with higher priority, update its priority and rebuild the heap.
        # If item already in priority queue with equal or lower priority, do nothing.
        # If item not in priority queue, do the same thing as self.push.
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)


# This is the method that will be used to return the euclidean heuristic value.
# I have not decided whether or not to choose this or manhattan for the heuristic
# so both will be included until I make a decision
def manhattanHeuristic(currentState, terminalState):                                                                 # Method Block
   
                                                                                                                     # VARIABLE DEFINITIONS
   curXYPosition = currentState                                                                                      # Defines a variable to look at the values in the tuple currentState
   terminationXYPosition = terminalState                                                                             # Defines a variable to look at the values in the tuple terminalState
   
   # Manhattan = |x1 - x2| + |y1 - y2|
   return abs(curXYPosition[0] - terminationXYPosition[0]) + abs(curXYPosition[1] - terminationXYPosition[1])        # Returns the manhattan distance to the user
   
   
   
# This is the method that will be used to return the manhattan heuristic value.
# I have not decided whether or not to choose this or euclidean for the heuristic
# so both will be included until I make a decision
def euclideanHeuristic(currentState, terminalState):                                                                 # Method Block
   
                                                                                                                     # VARIABLE DEFINITIONS
   curXYPosition = currentState                                                                                      # Defines a variable to look at the values in the tuple currentState
   terminationXYPosition = terminalState                                                                             # Defines a variable to look at the values in the tuple terminalState
   
   # Euclidean Heuristic equation = (|x1 - x2|^2 + |y1 - y2|^2) ^ 0.5
   firstPart = pow(curXYPosition[0] - terminationXYPosition[0], 2)                                                   # |x1 - x2| ^ 2
   secondPart = pow(curXYPosition[1] - terminationXYPosition[1], 2)                                                  # |y1 - y2| ^ 2
   
   summation = firstPart + secondPart                                                                                # |x1 - x2| ^ 2 + |y1 - y2| ^ 2
   
   return math.sqrt(summation)                                                                                       # Returns (|x1 - x2|^2 + |y1 - y2|^2) ^ 0.5
   


# This is a helper method that is being used to assist in the computation of the pathCost function
# Because I am using an astar search algorithm to traverse the graph and I am storing the traversed path
# in a list, I need to be able to break up the number of times that a given direction was taken.
   # i.e. When 13S is passed in I need to return the boolean value to return false when i have an index 
   # value for the string when only the number is left. 
   
   # Since I am traversing the list backwards, starting at index[-1:] I will return true as long as the character at 
   # the passed index is one of the cardinal directions: N, S, E, or W
def singularPath(path, index):                                                                                       # Function Block
   
   # Checks to see if the index being looked at is still one of the cardinal directions
   if path[index: index + 1] == "N" or path[index: index + 1] == "E" or path[index: index + 1] == "S" or path[index: index + 1] == "W":
      return True                                                                                                    # Returns True which will keep the while loop in pathCost running
      
   return False                                                                                                      # Returns False to break out of the loop in pathCost



# This is the method that will be used to compute the singular path cost for just one direction.
# This will be used as a helper method to compute the total cost of the path traversed up to the specified point.      
def pathCost(direction):                                                                                             # Method Block      
   
                                                                                                                     # VARIABLE DEFINITIONS
   cost = 0                                                                                                          # Sets the initial value of the cost to 0
  
   # Through trial and error, if I am looking backwards this has to be -2 otherwise an error would be thrown 
   backwardsIndex = -2                                                                                               # Sets the initial value to look at the last index in the passed direction
  
   # While Loop that will run until we bypass the cardinal directions in the
   # string 'direction' that was passed to the function
   while singularPath(direction, backwardsIndex):                                                                         
      backwardsIndex -= 1                                                                                            # Repeatedly subtracts from the value of backwards
   
   cost = int(direction[: backwardsIndex + 1])                                                                       # Converts the string value to an integer
   
   return cost                                                                                                       # Returns the cost of a singular direction taken



# This is the method that will be done to compute the total cost of the path traversed up until a specific 
# point in the astar search. This method will be called everytime that we reach a different color arrow 
# then the one that we initially started at, and will compute the cost that it took to get to that specific
# spot from the top left corner. This will be used to determine the priority of the next thing to deque 
# in the priority queue that is used for the astar search traversal.
def totalCost(directions):                                                                                           # Method Block
   
                                                                                                                     # VARIABLE DEFINITION
   cost = 0                                                                                                          # Sets the value of the total cost to initially be 0
   
   # I am curious as to if an empty list is passed into this, 
   # like for instance the starting position, if this for loop will run at all.
   # The assumption is that it will not and the value returned should be 0,
   # which is correct for the first position in the traversal.
   for singularDirection in directions:                                                                              # For Loop for all the directions
      cost += pathCost(singularDirection)                                                                            # Adds to the value of cost by calling the function pathCost
      
   return cost                                                                                                       # Returns the value of cost to the user
   


# This is the method that will be used to determine which "direction" to travel.
# This method will return the increments that will be added to the coordinate we start 
# at any given deque from the priority queue. It should only be called once in the astar 
# search method right after we deque the next coordinate to look at in the grid.
def travelThatWay(arrow):                                                                                            # Method Block
   
                                                                                                                     # VARIABLE DEFINITIONS
   rowIncrement = 0                                                                                                  # Sets the value of rowIncrement
   colIncrement = 0                                                                                                  # Sets the value of colIncrement
   
   directionsTaken = arrow[2: ]                                                                                      # String to get the directions taken at the given arrow
   
   for cardinalDirection in directionsTaken:                                                                         # For Loop
      
      if cardinalDirection == "N":                                                                                   # If statement   
         rowIncrement = -1                                                                                           # Sets the value of rowIncrement
      
      elif cardinalDirection == "E":                                                                                 # Elif Statement
         colIncrement = 1                                                                                            # Sets the value of colIncrement
      
      elif cardinalDirection == "S":                                                                                 # Elif Statement
         rowIncrement = 1                                                                                            # Sets the value of rowIncrement
      
      elif cardinalDirection == "W":                                                                                 # Last Elif statement
         colIncrement = -1                                                                                           # Sets the value of colIncrement
   
   return rowIncrement, colIncrement                                                                                 # Returns the values of rowIncrement and colIncrement to the user   
   


# This is the method that will be used to perform the actual astar search to find a possible
# path between the top left corner of the directed graph and the terminal state, which we know
# to be at the bottom right of the graph. The method will make use of a priority queue to choose
# the next coordinate on the graph to look at and explore, as well as the directions taken from 
# the original start state (top left corner) to a state that will be pushed into the priority queue.
# We were given the assumption that every directed graph that was given will have a solution so the
# method will return a list of directions to get from the start state to the end state. On the off
# chance that we are eventually given a graph with no attianable solution 
# then the method will return that no path could be found.
def astarSearch(grid):                                                                                               # Method Block
   
                                                                                                                     # VARIABLE DEFINITIONS
   pQueue = PriorityQueue()                                                                                          # Creates a priority queue for the astar computation
   directions = []                                                                                                   # Creates an empty list that will store the directions taken at any given coordinate
   
   startingPos = (0, 0)                                                                                              # Creates a tuple for the starting position for the graph traversal
   terminal = (len(grid) - 1, len(grid[0]) - 1)                                                                      # Sets the coordinate value for the terminal state
   
   pQueue.push((startingPos, directions), 0)                                                                         # Adds to the priority queue
   
   while not pQueue.isEmpty():                                                                                          # While Loop
      curPos, path = pQueue.pop()                                                                                    # Pops the first item in the priority queue
      
      if curPos == terminal:                                                                                         # Checks to see if the terminal state has been reached
         return path                                                                                                 # Returns the path to the user
      
      xCoordinate = curPos[0]                                                                                        # Sets the value of the xCoordinate
      yCoordinate = curPos[1]                                                                                        # Sets the value of the yCoordinate
      
      rowIncrement, colIncrement = travelThatWay(grid[xCoordinate][yCoordinate])                                     # Function call to travelThatWay
      
      travelDirection = 0                                                                                           # Value to be used to keep track of how many times we traveled in a given direction
      arrowColor = (grid[xCoordinate][yCoordinate])[: 1]                                                             # Gets the color of the arrow at the current position
      
      while (xCoordinate >= 0 and xCoordinate < len(grid)) and (yCoordinate >= 0 and yCoordinate < len(grid[0])):    # While Loop
         
         if arrowColor != (grid[xCoordinate][yCoordinate])[: 1]:                                                     # Checking to see if the arrow color is different from the starting arrow color
            
            newPos = (xCoordinate, yCoordinate)                                                                      # Creates a tuple for the position 
            
            heuristic = manhattanHeuristic(newPos, terminal)                                                         # Gets the heuristic at the specific location
            
            direction = "" + str(travelDirection) + (grid[curPos[0]][curPos[1]])[2: ]                                # Creates the string for the length of which direction you went
            
            newRoute = path + [direction]                                                                            # Creates a list for the route taken to the specific area
            
            pQueue.push((newPos, newRoute), heuristic + totalCost(path))                                             # Adds to the priorityQueue
            
         travelDirection += 1                                                                                        # Adds to the value of travelDirection
         xCoordinate += rowIncrement                                                                                 # Adds to the value of xCoordinate
         yCoordinate += colIncrement                                                                                 # Adds to the value of yCoordinate
            
   

def main():
   
   #text_file = "large.txt"                                                                                           # Sets the variable for the text file
   #text_file = sys.argv[1]                                                                                           # Sets the value for the text_file
   #output = sys.argv[2]                                                                                              # Gets the output value
   text_file = input("Enter the name of the input file: ")
   output = input("Enter the name of the output file: ")
   
   if ".txt" not in text_file:                                                                                       # Redundancy check to see if the person includes the .txt file type in the input
      text_file += ".txt"                                                                                            # Adds to the value of text_file
   
   if ".txt" not in output:                                                                                          # Redundancy check to see if the person includes the .txt file type in the input
      output += ".txt"                                                                                               # Adds to the value of output
      
   fileInfo = []                                                                                                     # Creates an empty list to store the info from the file
   
   fileData = open(text_file, "r")                                                                                   # Opens the file
   
   fileInfo = fileData.readlines()                                                                                   # Reads the data from the file
   
   gridDimensions = fileInfo[0].split()                                                                              # Retrieves the grid dimensions
   
   row = int(gridDimensions[0])                                                                                      # Gets the row value for the grid
   column = int(gridDimensions[1])                                                                                   # Gets the column value for the grid
   
   grid = [["-" for LCV in range(column)] for LCV2 in range(row)]                                                    # Makes a row by column 2d Array
   
   # Redundancy print to make sure the dimensions are correct  
   # Will be commented out after I know the grid is good
   #for row in grid:                                                                                                  # For Loop
   #   print(row)                                                                                                     # Prints out to the user
   
   #fileInfo = fileInfo[1: ]                                                                                          # Removes the grid dimensions from things to look at+
   
   # Populate the grid with the values
   for i, rowVal in enumerate(fileInfo[1:]):                                                                          # For Loop
      
      # Removing spaces and new line characters
      # so only the values for the arrows remain
      infoForRow = rowVal.split()                                                                                    
      
      # Looping through the obtained list to place values into the 2d grid
      for j, val in enumerate(infoForRow):                                                               
         grid[i][j] = val
   
   fileData.close()
   
   # Redundancy print to make sure the information was loaded into the grid correctly  
   # Will be commented out after I know the grid is good
   for rows in grid:                                                                                                  # For Loop
      for item in rows:                                                                                               # Nested For Loop
         print("{:^6s}".format(item), end = " ")                                                                     # Formatted print statement because I am anal
      print("\n")                                                                                                    # Prints the newline character
   
   traversalPath = astarSearch(grid)
   
   #fileData = open(text_file[: -4] + "-soln.txt", "w")
   
   fileData = open(output, "w")                                                                                      
   fileData.write(" ".join(traversalPath))
   
   fileData.close()
   
   print(f"The path to traverse the graph is: {' '.join(traversalPath)}")
   
main()