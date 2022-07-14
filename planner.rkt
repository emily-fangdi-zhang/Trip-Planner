#lang dssl2

# Final project: Trip Planner

import cons
import sbox_hash
import 'project-lib/graph.rkt'
import 'project-lib/dictionaries.rkt'
import 'project-lib/binheap.rkt'
### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Entity Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?

let eight_principles = ["Know your rights.",
                        "Acknowledge your sources.",
                        "Protect your work.",
                        "Avoid suspicion.",
                        "Do your own work.",
                        "Never falsify a record or permit another person to do so.",
                        "Never fabricate data, citations, or experimental results.",
                        "Always tell the truth when discussing your work with your instructor."]    
struct category_name:
    let category
    let name

struct pos_num_distance:
    let position_number
    let dist
    
struct struct_position:
    let latitude
    let longitude  
      
struct dijkstra_struct:
    let distances
    let previous
                            
interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


class TripPlanner (TRIP_PLANNER):
    let position_to_pos_num
    let graph
    let position_number
    let POI_OG_list
    let position_to_POI
    let name_to_position
    let pos_num_to_position
    
    def __init__(self, RawSegments, RawPOIs):
        let POI_position
        let position1
        let position2
        
        self.position_number = 0
        self.POI_OG_list = RawPOIs        
        self.position_to_pos_num = HashTable(len(RawSegments) * 2 + len(RawPOIs), make_sbox_hash())        
        self.position_to_POI = HashTable(len(RawPOIs), make_sbox_hash())        
        self.name_to_position = HashTable(len(RawPOIs), make_sbox_hash())        
        self.pos_num_to_position = HashTable(len(RawSegments) * 2 + len(RawPOIs), make_sbox_hash())        
        self.graph = WuGraph(len(RawSegments) * 2 + len(RawPOIs))
        
        for index in range(len(RawSegments)):
            position1 = [RawSegments[index][0], RawSegments[index][1]]
            position2 = [RawSegments[index][2], RawSegments[index][3]]
           
            if not self.position_to_pos_num.mem?(position1):
                self.position_to_pos_num.put(position1, self.position_number)
                self.pos_num_to_position.put(self.position_number, position1)
                self.position_number = self.position_number + 1
                
            if not self.position_to_pos_num.mem?(position2):
                self.position_to_pos_num.put(position2, self.position_number)
                self.pos_num_to_position.put(self.position_number, position2)
                self.position_number = self.position_number + 1
                
            self.graph.set_edge(self.position_to_pos_num.get(position1), self.position_to_pos_num.get(position2), 
                                self.distance(position1[0], position1[1], position2[0], position2[1]))  
                                   
        for index in range(len(RawPOIs)):        
            POI_position = [RawPOIs[index][0], RawPOIs[index][1]]
            if self.position_to_pos_num.mem?(POI_position):
                if not self.position_to_POI.mem?(POI_position):
                    self.position_to_POI.put(POI_position, cons(category_name(RawPOIs[index][2], RawPOIs[index][3]), 
                                                None))  
                else: 
                    self.position_to_POI.put(POI_position, cons(category_name(RawPOIs[index][2], RawPOIs[index][3]), 
                                                self.position_to_POI.get(POI_position))) 
                self.name_to_position.put(RawPOIs[index][3], POI_position)
    
    def locate_all(self, dst_cat: Cat?): 
        let position_list = None 
        let POI_no_duplicates = HashTable(self.position_to_POI.len(), make_sbox_hash())
                
        for POI in self.POI_OG_list:
            let position = struct_position(POI[0], POI[1])
            if not POI_no_duplicates.mem?(position) and POI[2] == dst_cat:
                position_list = cons([position.latitude, position.longitude], position_list)
                POI_no_duplicates.put(position, 0)
        return position_list
                            
    def plan_route(self,src_lat: Lat?, src_lon: Lon?, dst_name: Name?):  
        if not self.position_to_pos_num.mem?([src_lat, src_lon]) or not self.name_to_position.mem?(dst_name):
            return None
        let previous = self.dijkstras(src_lat, src_lon).previous
        let position_number = self.position_to_pos_num.get([src_lat, src_lon])
        let end = self.position_to_pos_num.get(self.name_to_position.get(dst_name))
        let route = None
               
        if previous[end] is None and not end == position_number:
            return None
            
        while not end == position_number and previous[end] is not None:
                route = cons(self.pos_num_to_position.get(end), route)
                end = previous[end]
    
        route = cons(self.pos_num_to_position.get(position_number), route)
        return route
  
    def find_nearby(self, src_lat: Lat?, src_lon: Lon?, dst_cat: Cat?, n: nat?):
        if not self.position_to_pos_num.mem?([src_lat, src_lon]):
            return None   
        let todo = BinHeap(self.position_to_pos_num.len(), lambda x, y: x.dist < y.dist)
        let distances = self.dijkstras(src_lat, src_lon).distances
        let list_POIs = None
        let i = 0
          
        for pos_num in range(self.position_to_pos_num.len()):
            if distances[pos_num] != inf:
                todo.insert(pos_num_distance(pos_num, distances[pos_num]))   
            
        while i < n:
            if todo.len() == 0:
                break
            let position_number = todo.find_min().position_number
            todo.remove_min()
            
            if self.position_to_POI.mem?(self.pos_num_to_position.get(position_number)):
                let cons_list = self.position_to_POI.get(self.pos_num_to_position.get(position_number))
                while cons_list is not None:
                    if cons_list.data.category == dst_cat:
                       let hafoo = [self.pos_num_to_position.get(position_number)[0], self.pos_num_to_position.get(position_number)[1], dst_cat, cons_list.data.name]
                       list_POIs = cons(hafoo, list_POIs)
                       i = i + 1
                    if i >= n:
                        break
                    cons_list = cons_list.next
                                      
        return list_POIs
        
    def distance(self, x1, y1, x2, y2):
        return (((x2 - x1)**2 + (y2 - y1)**2).sqrt()) 
           
    def dijkstras(self, lat, lon):
       let finished = HashTable(self.graph.len(),make_sbox_hash()) 
       let needs = BinHeap(self.graph.len(), lambda x, y: x.dist < y.dist)
       let start = self.position_to_pos_num.get([lat, lon])
       let dist_vec = [inf; self.graph.len()]
       let prev_vec = [None; self.graph.len()]
       let temp
       let minimum
       let adjacent_position 
       
       dist_vec[start] = 0
       for index in range(self.graph.len()): 
           finished.put(index, False)

       needs.insert(pos_num_distance(start, 0)) 

       while not needs.len() == 0:
             minimum = needs.find_min().position_number
             needs.remove_min()

             if finished.get(minimum) == False:
                 finished.put(minimum, True) 
                 
             adjacent_position = self.graph.get_adjacent(minimum)      
             while adjacent_position is not None: 
                     temp = dist_vec[minimum] + self.graph.get_edge(minimum, adjacent_position.data)
                     temp = self.graph.get_edge(minimum, adjacent_position.data)
                     
                     if dist_vec[minimum] + temp < dist_vec[adjacent_position.data]: 
                         dist_vec[adjacent_position.data] = dist_vec[minimum] + temp
                         prev_vec[adjacent_position.data] = minimum
                         needs.insert(pos_num_distance(adjacent_position.data, dist_vec[adjacent_position.data]))
                         
                     adjacent_position = adjacent_position.next  
       
       return(dijkstra_struct(dist_vec, prev_vec))
       
def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pelmeni"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pelmeni") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pelmeni"], None)

test 'multiple POIs in the same location, relevant one is first':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'food', 'Jollibee']])
    let result = tp.locate_all('bar')
    assert Cons.to_vec(result) \
      == [[5, 0]]

test 'multiple POIs in the same location, relevant one is last':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'food', 'Jollibee']])
    let result = tp.locate_all('bar')
    assert Cons.to_vec(result) \
      == [[5, 0]]

test 'multiple POIs in the same location, two relevant ones':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'food', 'Jollibee']])
    let result = tp.locate_all('barber')
    assert Cons.to_vec(result) \
      == [[5, 0], [3, 0]]
    
test '3 relevant POIs, 2 at same location':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
    let result = tp.locate_all('barber')
    assert Cons.to_vec(result) \
      == [[5, 0], [3, 0]]
      
test '1-step route':
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[1, 0, 'bank', 'Union']])
    let result = tp.plan_route(0, 0, 'Union')
    assert Cons.to_vec(result) \
      == [[0, 0], [1, 0]]
      
test '2-step route':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [2.5, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Tony')
    assert Cons.to_vec(result) \
      == [[0, 0], [1.5, 0], [2.5, 0]]
      
test '3-step route':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Tony')
    assert Cons.to_vec(result) \
      == [[0, 0], [1.5, 0], [2.5, 0], [3, 0]]
      
test 'from barber to bank':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(3, 0, 'Union')
    assert Cons.to_vec(result) \
      == [[3, 0], [2.5, 0], [1.5, 0]]
      
test '0-step route':
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[0, 0, 'bank', 'Union']])
    let result = tp.plan_route(0, 0, 'Union')
    assert Cons.to_vec(result) \
      == [[0, 0]]
      
test 'BFS is not SSSP (route)':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.plan_route(0, 0, 'Cem')
    assert Cons.to_vec(result) \
      == [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 8], [9, 9]]
      
test 'MST is not SSSP (route)':
    let tp = TripPlanner(
      [[-1.1, -1.1, 0, 0],
       [0, 0, 3, 0],
       [3, 0, 3, 3],
       [3, 3, 3, 4],
       [0, 0, 3, 4]],
      [[0, 0, 'food', 'Sandwiches'],
       [3, 0, 'bank', 'Union'],
       [3, 3, 'barber', 'Judy'],
       [3, 4, 'barber', 'Tony']])
    let result = tp.plan_route(-1.1, -1.1, 'Tony')
    assert Cons.to_vec(result) \
      == [[-1.1, -1.1], [0, 0], [3, 4]]
      
test 'Destination is the 2nd of 3 POIs at that location':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'food', 'Jollibee']])
    let result = tp.plan_route(0, 0, 'Judy')
    assert Cons.to_vec(result) \
      == [[0, 0], [1.5, 0], [2.5, 0], [3, 0], [4, 0], [5, 0]]
      
test 'Two equivalent routes':
    let tp = TripPlanner(
      [[-2, 0, 0, 2],
       [0, 2, 2, 0],
       [2, 0, 0, -2],
       [0, -2, -2, 0]],
      [[2, 0, 'cooper', 'Dennis']])
    let result = tp.plan_route(-2, 0, 'Dennis')
    assert Cons.to_vec(result) \
      in [[[-2, 0], [0, 2], [2, 0]],
          [[-2, 0], [0, -2], [2, 0]]]
   
test 'BinHeap needs capacity > |V|':
    let tp = TripPlanner(
      [[0, 0, 0, 1],
       [0, 1, 3, 0],
       [0, 1, 4, 0],
       [0, 1, 5, 0],
       [0, 1, 6, 0],
       [0, 0, 1, 1],
       [1, 1, 3, 0],
       [1, 1, 4, 0],
       [1, 1, 5, 0],
       [1, 1, 6, 0],
       [0, 0, 2, 1],
       [2, 1, 3, 0],
       [2, 1, 4, 0],
       [2, 1, 5, 0],
       [2, 1, 6, 0]],
      [[0, 0, 'blacksmith', "Revere's Silver Shop"],
       [6, 0, 'church', 'Old North Church']])
    let result = tp.plan_route(0, 0, 'Old North Church')
    assert Cons.to_vec(result) \
      == [[0, 0], [2, 1], [6, 0]]
      
test '1 bank nearby':
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[1, 0, 'bank', 'Union']])
    let result = tp.find_nearby(0, 0, 'bank', 1)
    assert Cons.to_vec(result) \
      == [[1, 0, 'bank', 'Union']]
      
test '1 barber nearby':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.find_nearby(0, 0, 'barber', 1)
    assert Cons.to_vec(result) \
      == [[3, 0, 'barber', 'Tony']]
      
test 'find bank from barber':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.find_nearby(3, 0, 'bank', 1)
    assert Cons.to_vec(result) \
      == [[1.5, 0, 'bank', 'Union']]
      
test 'BFS is not SSSP (nearby)':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.find_nearby(0, 0, 'haberdasher', 2)
    assert Cons.to_vec(result) \
      == [[8, 8, 'haberdasher', 'Braden'], [7, 7, 'haberdasher', 'Archit']]
      
test 'MST is not SSSP (nearby)':
    let tp = TripPlanner(
      [[-1.1, -1.1, 0, 0],
       [0, 0, 3, 0],
       [3, 0, 3, 3],
       [3, 3, 3, 4],
       [0, 0, 3, 4]],
      [[0, 0, 'food', 'Sandwiches'],
       [3, 0, 'bank', 'Union'],
       [3, 3, 'barber', 'Judy'],
       [3, 4, 'barber', 'Tony']])
    let result = tp.find_nearby(-1.1, -1.1, 'barber', 1)
    assert Cons.to_vec(result) \
      == [[3, 4, 'barber', 'Tony']]
      
test '2 relevant POIs; 1 reachable':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) \
      == [[3, 0, 'barber', 'Tony']]
      
test '2 relevant POIs; limit 3':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 3)
    assert Cons.to_vec(result) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
      
test '2 relevant equidistant POIs; limit 1':
    let tp = TripPlanner(
      [[-1, -1, 0, 0],
       [0, 0, 3.5, 0],
       [0, 0, 0, 3.5],
       [3.5, 0, 0, 3.5]],
      [[-1, -1, 'food', 'Jollibee'],
       [0, 0, 'bank', 'Union'],
       [3.5, 0, 'barber', 'Tony'],
       [0, 3.5, 'barber', 'Judy']])
    let result = tp.find_nearby(-1, -1, 'barber', 1)
    assert Cons.to_vec(result) \
      in [[[3.5, 0, 'barber', 'Tony']],
          [[0, 3.5, 'barber', 'Judy']]]
          
test '3 relevant POIs; farther 2 at same location; limit 2':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) \
      in [[[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']],
          [[5, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]]
          
test '3 relevant POIs; farther 2 equidistant; limit 2':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [0, 0, 'barber', 'Lily'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(2.5, 0, 'barber', 2)
    assert Cons.to_vec(result) \
      in [[[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']],
          [[0, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]]
          
test 'POI is 2nd of 3 in that location':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) \
      == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]