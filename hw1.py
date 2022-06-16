from queue import Queue
from queue import LifoQueue
from queue import PriorityQueue
import math 

def to_int_tuple(str1):
   coords = str1.split(" ")
   for i in range(len(coords)):
      coords[i] = int(coords[i])
   return tuple(coords)
   
def tuple_to_string(loc):
   loc_str = ""
   for num in loc: 
      loc_str = loc_str + str(num) + " "
   return loc_str
   
def in_bounds(loc, bounds):
   return bool((0 <= loc[0] < bounds[0])
               & (0 <= loc[1] < bounds[1])
               & (0 <= loc[2] < bounds[2]))

#straight moves: 
def mv1(loc):
   loc = list(loc)
   loc[0]+=1
   return tuple(loc)
def mv2(loc):
   loc = list(loc)
   loc[0]-=1
   return tuple(loc)
def mv3(loc):
   loc = list(loc)
   loc[1]+=1
   return tuple(loc)
def mv4(loc):
   loc = list(loc)
   loc[1]-=1
   return tuple(loc)
def mv5(loc):
   loc = list(loc)
   loc[2]+=1
   return tuple(loc)
def mv6(loc):
   loc = list(loc)
   loc[2]-=1
   return tuple(loc)

#diagonal moves: 
def mv7(loc):
   loc = list(loc) 
   loc[0]+=1
   loc[1]+=1
   return tuple(loc)
def mv8(loc): 
   loc = list(loc)
   loc[0]+=1
   loc[1]-=1
   return tuple(loc)
def mv9(loc): 
   loc = list(loc)
   loc[0]-=1
   loc[1]+=1
   return tuple(loc)
def mv10(loc): 
   loc = list(loc)
   loc[0]-=1
   loc[1]-=1
   return tuple(loc)
def mv11(loc): 
   loc = list(loc)
   loc[0]+=1
   loc[2]+=1
   return tuple(loc)
def mv12(loc): 
   loc = list(loc)
   loc[0]+=1
   loc[2]-=1
   return tuple(loc)
def mv13(loc): 
   loc = list(loc)
   loc[0]-=1
   loc[2]+=1
   return tuple(loc)
def mv14(loc): 
   loc = list(loc)
   loc[0]-=1
   loc[2]-=1
   return tuple(loc)
def mv15(loc): 
   loc = list(loc)
   loc[1]+=1
   loc[2]+=1
   return tuple(loc)
def mv16(loc): 
   loc = list(loc)
   loc[1]+=1
   loc[2]-=1
   return tuple(loc)
def mv17(loc): 
   loc = list(loc)
   loc[1]-=1
   loc[2]+=1
   return tuple(loc)
def mv18(loc): 
   loc = list(loc)
   loc[1]-=1
   loc[2]-=1
   return tuple(loc)

#move functions dict
mvs = {
   1: mv1,
   2: mv2,
   3: mv3,
   4: mv4,
   5: mv5,
   6: mv6,
   7: mv7,
   8: mv8,
   9: mv9,
   10: mv10,
   11: mv11,
   12: mv12,
   13: mv13,
   14: mv14,
   15: mv15,
   16: mv16,
   17: mv17,
   18: mv18,
}

def uninformed_dict(loc_mvs, num_locs, lines):
   for i in range(5, num_locs+5):
      coords = lines[i].split(" ")
      for i in range(len(coords)):
         coords[i] = int(coords[i])
      grid_loc = tuple(coords[:3])
      mv_list = coords[3:]
      loc_mvs[grid_loc] = mv_list   

def uninformed_search(entry_loc, loc_mvs, exit_goal, bounds): 
   frontier = Queue(maxsize=0)
   parent_dict = {} # maps nodes to their parent

   if not in_bounds(entry_loc, bounds) or entry_loc not in loc_mvs:
      return parent_dict # no sol'n 
   
   elif entry_loc == exit_goal: 
      parent_dict[entry_loc] = 0
      return parent_dict # sol'n only one node
   
   visited = set(entry_loc)
   frontier.put(entry_loc)
   
   # exits loop when there are no more nodes to explore or 
   # the exit goal has been found
   while frontier.qsize() > 0 and exit_goal not in parent_dict: 
      curr = frontier.get()
      if curr in loc_mvs:
         for move in loc_mvs[curr]: 
            neighbor = mvs[move](curr)
            if in_bounds(neighbor, bounds) and neighbor in loc_mvs and neighbor not in visited: 
               visited.add(neighbor) 
               frontier.put(neighbor) 
               parent_dict[neighbor] = curr
   return parent_dict

def output(parent_dict, exit_goal, entry):
   path = LifoQueue(maxsize=0)
   if exit_goal not in parent_dict: 
      return path ##no sol'n = empty stack 

   curr = exit_goal

   while curr != entry: 
      path.put(curr)
      curr = parent_dict[curr] #find parent of current node, update curr node
   path.put(curr) ##puts entry into stack 
   return path 

def write_uninformed_soln(path): ##where path is stack and the first element is the entry loc
   with open('output.txt', 'w') as maze_soln:
      if path.qsize()==0:
         maze_soln.write("FAIL")
         return
      maze_soln.write(str(path.qsize()-1) + "\n")
      maze_soln.write(str(path.qsize()) + "\n")
      maze_soln.write(tuple_to_string(path.get()) + "0\n")
      
      cost = "1"
      while path.qsize()>0: 
         curr = path.get()
         if path.qsize() == 0: 
            maze_soln.write(tuple_to_string(curr) + cost)
         else: 
            maze_soln.write(tuple_to_string(curr) + cost + "\n")   
def calc_distance(tuple1, tuple2):
   return math.sqrt(((tuple2[0] - tuple1[0])**2) + ((tuple2[1] - tuple1[1])**2) + ((tuple2[2] - tuple1[2])**2))   

def informed_dict(loc_mvs, num_locs, lines):
   straight_moves = [1, 2, 3, 4, 5, 6]

   for i in range(5, num_locs+5):
      coords = lines[i].split(" ") 
      for i in range(len(coords)):
         coords[i] = int(coords[i])
      grid_loc = tuple(coords[:3])

      ## create dictionary of moves to cost 
      mv_list = coords[3:]
      mv_dict = {move: 10 if move in straight_moves else 14 for move in mv_list}
      loc_mvs[grid_loc] = mv_dict

def informed_search(a_star, entry_loc, loc_mvs, exit_goal, bounds):

   pq = PriorityQueue()
   pq.put((0, entry_loc))
   min_cost = {entry_loc : [0,0]} ##first element total cost of path, second element cost of last move
   parent_dict = {} ## map node to parent

   if not in_bounds(entry_loc, bounds) or entry_loc not in loc_mvs:
      return parent_dict, min_cost # no sol'n
   elif entry_loc == exit_goal:
      parent_dict[entry_loc] = 0
      return parent_dict, min_cost # sol'n one node
   
   while not pq.empty(): 
      elem = pq.get()
      curr = elem[1] 
      if curr in loc_mvs: #test if curr is valid grid point
         mv_cost = loc_mvs[curr] #dictionary of encoded moves to their cost
         for move in mv_cost:
            dist = mv_cost[move] + min_cost[curr][0] 
            dist_to_exit = calc_distance(curr, exit_goal)
            heuristic_cost = dist + a_star*math.ceil(dist_to_exit) ##mutliply by some binary variable 
            neighbor = mvs[move](curr)
            #if neighbor is valid point and either not already in min_cost or found cheaper overall path to neighbor
            if in_bounds(neighbor, bounds) and (neighbor not in min_cost or min_cost[neighbor][0] > dist): 
               min_cost[neighbor] = [dist, mv_cost[move]]
               pq.put((heuristic_cost, neighbor))
               parent_dict[neighbor] = curr
   if exit_goal not in parent_dict: 
      return {}, {} ##no sol'n
   return parent_dict, min_cost

def write_informed_soln(path, min_cost, exit_goal): ##where path is stack and the first element is the entry loc
   with open('output.txt', 'w') as maze_soln:
      if path.qsize()==0:
         maze_soln.write("FAIL")
         return
      
      total = min_cost[exit_goal][0]
      maze_soln.write(str(total) + "\n")
      maze_soln.write(str(path.qsize()) + "\n")
      maze_soln.write(tuple_to_string(path.get()) + "0\n")

      while path.qsize() > 0: 
         curr = path.get()
         cost = min_cost[curr][1] #cost of last move to curr
         if path.qsize() == 0: 
            maze_soln.write(tuple_to_string(curr) + str(cost))
         else: 
            maze_soln.write(tuple_to_string(curr) + str(cost) + "\n")

def main():
   with open('input.txt', 'r') as input_file:
      lines = input_file.read().split('\n') 
      algo = lines[0]
      bounds = to_int_tuple(lines[1])
      entry = to_int_tuple(lines[2])
      exit_goal = to_int_tuple(lines[3])
      num_locs = int(lines[4])

      loc_mvs = {} # dictionary matching grid locations to their possible moves [tuple : list]    
   if algo=="BFS" :
      uninformed_dict(loc_mvs, num_locs, lines)
      parent_dict = uninformed_search(entry, loc_mvs, exit_goal, bounds)
      soln_path = output(parent_dict, exit_goal, entry)
      write_uninformed_soln(soln_path)
   else:
      a_star = 1 if algo =="A*" else 0
      informed_dict(loc_mvs, num_locs, lines)
      parent_dict, min_cost = informed_search(a_star, entry, loc_mvs, exit_goal, bounds)
      soln_path = output(parent_dict, exit_goal, entry)
      write_informed_soln(soln_path, min_cost, exit_goal)

main()