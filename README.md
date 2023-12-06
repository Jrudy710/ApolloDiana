# ApolloDiana

# This is a program that will read from a file, with the first line in the file representing the row and column values 
# for a 2 dimensional array. The program will then pass the rest of the information from the file into a 2-d array, 
# which after the rest of the information from the file has been passed into the grid the program will start at the 
# top left of the grid and look for the target position which we were informed is at the bottom right of the array. 
# The program will perform this traversal by using a form of astar search to find the most optimal path, and then 
# return that path to the user. The assumption is that any given grid will have at least one possible path from the
# start state to the target state. 