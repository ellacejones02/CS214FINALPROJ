#lang dssl2
#netID: ecj9654
# Final project: Trip Planner

import cons
import sbox_hash
import 'project-lib/dictionaries.rkt'
import 'project-lib/graph.rkt'
import 'project-lib/binheap.rkt'

### Basic Types ###
let eight_principles = ["Know your rights.",
"Acknowledge your sources.",
"Protect your work.",
"Avoid suspicion.",
"Do your own work.",
"Never falsify a record or permit another person to do so.",
"Never fabricate data, citations, or experimental results.",
"Always tell the truth when discussing your work with your instructor."]

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

# position_weight?: [vertix,distance]
let position_weight? = TupC[num?, num?]

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
    #initialize bariables
     #poi_counter
    let poi_counter: nat?
     #vertex_counter
    let vertex_counter: nat?
    # DICTIONARIES
    let position_to_vertex_map: HashTable?
    let vertex_to_position_map: HashTable?
    let category_to_vertex_map: HashTable?
    let vertex_to_name_map: HashTable?
    let name_to_vertex_map: HashTable?
     #graph 
    let graph: WuGraph? #graph for sorting edges
 
     #distance of each vertix to source
    let vertex_dist 
     #parent - path 
    let parent #parent vertex for each vertex
     #status 
    let status #status of each vertex
     
    def __init__(self, raw_segments: VecC[RawSeg?], raw_pois: VecC[RawPOI?]):
       # initialize variables
       self.poi_counter = 0
       self.vertex_counter = 0
       self.position_to_vertex_map = HashTable(raw_segments.len()*2+raw_pois.len(),make_sbox_hash())
       self.vertex_to_position_map = HashTable(raw_segments.len()*2+raw_pois.len(),make_sbox_hash())
       self.category_to_vertex_map = HashTable(raw_pois.len(),make_sbox_hash())
       self.vertex_to_name_map = HashTable(raw_pois.len(),make_sbox_hash())
       self.name_to_vertex_map = HashTable(raw_pois.len(),make_sbox_hash())
       self.graph = WuGraph(raw_segments.len()*2+raw_pois.len())
       self.vertex_dist =  None
       self.parent = None
       self.status = None

        #add vertices for points of interest (POIs)
       for i in range (raw_pois.len()):

           if not self.position_to_vertex_map.mem?([raw_pois[i][0],raw_pois[i][1]]):                
               self.position_to_vertex_map.put([raw_pois[i][0],raw_pois[i][1]], self.poi_counter)
               self.vertex_to_position_map.put(self.poi_counter, [raw_pois[i][0],raw_pois[i][1]])
               self.poi_counter = self.poi_counter+1

           if not self.category_to_vertex_map.mem?(raw_pois[i][2]):
               # potentially may want to change this from 100 to a different number if fail stress tests in code submission
               let raw_pois_map = HashTable(100,make_sbox_hash())
               raw_pois_map.put(self.poi_counter-1, raw_pois[i][3])
               self.category_to_vertex_map.put(raw_pois[i][2], raw_pois_map)

           if self.category_to_vertex_map.mem?(raw_pois[i][2]):
               let raw_pois_map = self.category_to_vertex_map.get(raw_pois[i][2])
               raw_pois_map.put(self.poi_counter-1, raw_pois[i][3])                
                
           self.vertex_to_name_map.put(raw_pois[i][3], self.poi_counter-1)
           self.name_to_vertex_map.put([self.poi_counter-1, raw_pois[i][2]], raw_pois[i][3])
   
       self.vertex_counter = self.poi_counter
        # add vertices for raw segments
       for i in range(raw_segments.len()):
           if not self.position_to_vertex_map.mem?([raw_segments[i][0],raw_segments[i][1]]):
               self.position_to_vertex_map.put([raw_segments[i][0],raw_segments[i][1]],self.vertex_counter)
               self.vertex_to_position_map.put(self.vertex_counter,[raw_segments[i][0],raw_segments[i][1]])
               self.vertex_counter = self.vertex_counter+1
           if not self.position_to_vertex_map.mem?([raw_segments[i][2],raw_segments[i][3]]):
               self.position_to_vertex_map.put([raw_segments[i][2],raw_segments[i][3]],self.vertex_counter)
               self.vertex_to_position_map.put(self.vertex_counter,[raw_segments[i][2],raw_segments[i][3]])
               self.vertex_counter = self.vertex_counter+1
                
       self.graph = WuGraph(self.vertex_counter)
        # add edges to graph
       for i in range(raw_segments.len()):
           let w = ((raw_segments[i][2] - raw_segments[i][0])*(raw_segments[i][2] - raw_segments[i][0]) + (raw_segments[i][3] - raw_segments[i][1])*(raw_segments[i][3] - raw_segments[i][1])).sqrt()
           self.graph.set_edge(self.position_to_vertex_map.get([raw_segments[i][0],raw_segments[i][1]]), self.position_to_vertex_map.get([raw_segments[i][2],raw_segments[i][3]]), w)

    def locate_all(self, dst_cat:Cat?) ->ListC[RawPos?]:
        # find all positions of a given category
       if dst_cat is None or not self.category_to_vertex_map.mem?(dst_cat):
            return None
       let category_list = self.category_to_vertex_map.get(dst_cat)
       let positions = None
       for i in range(self.poi_counter):
           if category_list.mem?(i):
               let position = self.vertex_to_position_map.get(i)
               positions = cons(position, positions)
       return positions

         
    def find_closest_vertex(self):
        # find the closest vertex based on distance
        let dist = inf
        let index = -1
        for i in range(self.vertex_counter):
            if self.status[i] == False and dist > self.vertex_dist[i]:
                dist = self.vertex_dist[i]
                index = i
        return index
                    
    def dijkstra(self, start_vertex: nat?):
        # implementation of dijkstra's algorithm to find the shortest paths 
        self.vertex_dist = [inf; self.vertex_counter]
        self.parent = [None; self.vertex_counter]
        self.status = [False; self.vertex_counter]
        self.vertex_dist[start_vertex] = 0
        for i in range(self.vertex_counter):
             let closest_vertex = self.find_closest_vertex()
             if not closest_vertex == -1:
                 self.status[closest_vertex] = True
                 let adjs = Cons.to_vec(self.graph.get_adjacent(closest_vertex))
                 for k in range (adjs.len()):
                     let adj = adjs[k]
                     let w = self.graph.get_edge(closest_vertex, adj)
                     if not w == None:
                         if w + self.vertex_dist[closest_vertex] < self.vertex_dist[adj]:
                             self.vertex_dist[adj] = w + self.vertex_dist[closest_vertex]
                             self.parent[adj] = closest_vertex
        return
        
    def shortest_route(self, start_pos: num?, _destination: num?)  ->ListC[RawPos?]:
        # Find the shortest route from the starting position to the destination
        let route = cons(self.vertex_to_position_map.get(_destination), None)
         
        if start_pos == _destination:
            return route
         
        for i in range(self.vertex_counter):
            let k = self.parent[_destination]             
            #no route, end updating
            if k == None:
                return None                  
            if k == start_pos:
                return cons(self.vertex_to_position_map.get(k), route)
            if (not k == -1) and (not k == start_pos):
                route = cons(self.vertex_to_position_map.get(k), route)
                _destination = k
        return None
                
    def plan_route(self,src_lat:  Lat?,src_lon:  Lon?,dst_name: Name?)   ->ListC[RawPos?]:
        # Plan the route from the source position to the destination name
        if [src_lat, src_lon] is None or not self.position_to_vertex_map.mem?([src_lat, src_lon]):
            return None
        let start_vertex = self.position_to_vertex_map.get([src_lat, src_lon])
        if dst_name is None or not self.vertex_to_name_map.mem?(dst_name):
            return None  # Return an empty list if the destination is not found

        let destination_vertex = self.vertex_to_name_map.get(dst_name)

        
        if destination_vertex is None:
            return None
         #run dijkstra to populate the necessary data structs
        self.dijkstra(start_vertex)
         
        let myroute = self.shortest_route(start_vertex, destination_vertex)
        # If the shortest route is None or only contains the starting position, return an empty list
        #if myroute is None or myroute.next is None:
         #  return None
        #myroute = myroute.next
        return myroute
         
    def smaller_pos_weight(self, x:position_weight?,y:position_weight?):
        # Compare position weights based on their distances
        return x[1] < y[1]
                
    def find_nearby(self,src_lat: Lat?,src_lon: Lon?, dst_cat: Cat?, n: nat?) -> ListC[RawPOI?]:
        # Find nearby pois of a specific category
        if dst_cat is None or not self.category_to_vertex_map.mem?(dst_cat):
            return None
        let categ_list = self.category_to_vertex_map.get(dst_cat)
        if [src_lat, src_lon] is None or not self.position_to_vertex_map.mem?([src_lat, src_lon]):
            return None
        let start_vertex = self.position_to_vertex_map.get([src_lat, src_lon])       
         #dijkstra to populate
        self.dijkstra(start_vertex)         
         #sort using binheap
        let pq = BinHeap[position_weight?](self.vertex_counter, self.smaller_pos_weight)
        for i in range(self.vertex_counter):
            pq.insert([i, self.vertex_dist[i]])            
        let close_cat = None
        let close_count = 0
        for i in range(self.vertex_counter):
            let node = pq.find_min()
            pq.remove_min()
            let vert = node[0]
            if (close_count < n) and (not node[1] == inf) and categ_list.mem?(vert):
                close_count = close_count + 1
                let position = self.vertex_to_position_map.get(vert)
                let name = self.name_to_vertex_map.get([vert, dst_cat])
                close_cat = cons([position[0], position[1], dst_cat, name], close_cat)
        return close_cat
     


def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pierogi"], None)
        
test 'Locate all test with multiple points of interest':
    assert my_first_example().locate_all("bar") == cons([0,0], None)
    
def my_second_example():
    return TripPlanner([[0,0, 0,1], [0,2, 0,3], [0,0, 1,0], [2,0, 3,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"],
                        [0,2, "food", "Sushi House"],
                        [0,3, "bar", "Mischung"],
                        [1,0, "attraction", "Art Gallary"],
                        [2,0, "attraction", "Science Center"],
                        [3,0, "food", "Pizza Paradise"]])

# Test cases for locate_all method
test 'locate_all_no_duplicates()':
    assert my_second_example().locate_all("food") == cons([3, 0], cons([0,2], cons([0,1], None)))

# Test cases for plan_route method
test 'plan_route_unreachable_destination()':
    assert my_second_example().plan_route(0, 0, "Pizza Paradise") == None

test 'plan_route_nonexistent_destination()':
    assert my_second_example().plan_route(0, 0, "Burger Joint") == None

# Test case for find_nearby method
test 'find_nearby_with_ties()':
    assert my_second_example().find_nearby(0, 0, "bar", 2) == cons([0,0, "bar", "The Empty Bottle"], None)
    
test 'find_nearby_no_points_of_interest()':
    assert my_second_example().find_nearby(2, 2, "attraction", 1) == None

test 'find_nearby_same_category_same_distance()':
    assert my_second_example().find_nearby(0, 0, "food", 2) == cons([0, 1, "food", "Pierogi"], None)
    
def my_third_example():
    return TripPlanner(
        [[0, 0, 0, 1], [0, 1, 1, 1], [1, 1, 1, 2], [1, 2, 2, 2], [2, 2, 2, 3], [2, 3, 3, 3]],
        [[0, 0, "bar", "The Empty Bottle"],
         [0, 1, "food", "Pierogi"],
         [1, 1, "food", "Sushi House"],
         [1, 2, "bar", "Mischung"],
         [2, 2, "attraction", "Art Gallary"],
         [2, 3, "attraction", "Science Center"],
         [3, 3, "food", "Pizza Paradise"]]
    )

test 'long_route_plan_route()':
    assert my_third_example().plan_route(0, 0, "Science Center") == cons {data: [0, 0], next: cons {data: [0, 1], next: cons {data: [1, 1], next: cons {data: [1, 2], next: cons {data: [2, 2], next: cons {data: [2, 3], next: None}}}}}}

test 'long_route_find_nearby()':
    assert my_third_example().find_nearby(0, 0, "food", 2) ==  cons {data: [1, 1, 'food', 'Sushi House'], next: cons {data: [0, 1, 'food', 'Pierogi'], next: None}}

test 'long_route_locate_all()':
    assert my_third_example().locate_all("bar") == cons {data: [1, 2], next: cons {data: [0, 0], next: None}}

test 'long_route_unreachable_destination()':
    assert my_third_example().plan_route(0, 0, "Burger Joint") == None

test 'long_route_find_nearby_limit_greater_than_available()':
    assert my_third_example().find_nearby(0, 0, "attraction", 5) == cons {data: [2, 3, 'attraction', 'Science Center'], next: cons {data: [2, 2, 'attraction', 'Art Gallary'], next: None}}

test 'long_route_find_nearby_no_points_of_interest()':
    assert my_third_example().find_nearby(4, 4, "food", 1) == None

test 'long_route_find_nearby_same_category_same_distance()':
    assert my_third_example().find_nearby(2, 2, "attraction", 2) == cons {data: [2, 3, 'attraction', 'Science Center'], next: cons {data: [2, 2, 'attraction', 'Art Gallary'], next: None}}

### MORE TESTS (and using different formatting from above)

test 'BFS does not give shortest path (route)':
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
      [[0, 0, 'bank', 'USAA'],
       [9, 9, 'cafe', 'Coffee Lab']])
    let result = tp.plan_route(0, 0, 'Coffee Lab')
    assert Cons.to_vec(result) \
      == [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 8], [9, 9]] 

test 'multiple cafes in the same area, closest one is last':
    let tp = TripPlanner(
        [[0, 0, 1.5, 0],
        [1.5, 0, 2.5, 0],
        [2.5, 0, 3, 0],
        [4, 0, 5, 0],
        [3, 0, 4, 0]],
        [[1.5, 0, 'cafe', 'Coffee Lab'],
        [3, 0, 'cafe', 'Collectivo'],
        [5, 0, 'cafe', 'Backlot'],
        [5, 0, 'coffee shop', 'Newport'],
        [5, 0, 'tea house', 'Tea Time']])
    assert Cons.to_vec(tp.locate_all('cafe')) == [[5, 0], [3, 0], [1.5, 0]]

test '2-stop itinerary':
    let tp = TripPlanner(
        [[0, 0, 1.5, 0],
        [1.5, 0, 2.5, 0],
        [2.5, 0, 3, 0]],
        [[1.5, 0, 'bank', 'USAA'],
        [2.5, 0, 'barber', 'Gordon']])
    let result = tp.plan_route(0, 0, 'Gordon')
    assert Cons.to_vec(result) == [[0, 0], [1.5, 0], [2.5, 0]]

test '0-stop route':
    let tp = TripPlanner(
        [[0, 0, 1, 0]],
        [[0, 0, 'bank', 'USAA']])
    let result = tp.plan_route(0, 0, 'USAA')
    assert Cons.to_vec(result) == [[0,0]]

test 'Cafe is 2nd of 3 establishments in that area':
    let tp = TripPlanner(
        [[0, 0, 1.5, 0],
        [1.5, 0, 2.5, 0],
        [2.5, 0, 3, 0],
        [4, 0, 5, 0],
        [3, 0, 4, 0]],
        [[1.5, 0, 'bank', 'USAA'],
        [3, 0, 'cafe', 'Cofee Lab'],
        [5, 0, 'tea house', 'Colelctivo'],
        [5, 0, 'cafe', 'Backlot'],
        [5, 0, 'coffee shop', 'Newport']])
    let result = tp.find_nearby(0, 0, 'cafe', 2)
    assert Cons.to_vec(result) ==  [[5, 0, 'cafe', 'Backlot'], [3, 0, 'cafe', 'Cofee Lab']]
    
#### More TEST CASES as from PDF
test 'Basic locate: construction of the example trip planner from figure 1':
    let tp = TripPlanner(
        [[0, 0, 1, 0],
        [1, 0, 2, 0],
        [2, 0, 3, 0]],
        [[1, 0, 'cafe', 'Coffee Lab'],
        [2, 0, 'restaurant', 'Sushi']])
    let result = tp.locate_all('cafe')
    assert result == cons {data: [1, 0], next: None}

test 'locate-all with at most one POI per location':
    let tp = TripPlanner(
        [[0, 0, 1, 0],
        [1, 0, 2, 0],
        [2, 0, 3, 0]],
        [[1, 0, 'cafe', 'Coffee Lab'],
        [2, 0, 'restaurant', 'Sushi']])
    let result = tp.locate_all('restaurant')
    assert Cons.to_vec(result) == [[2, 0]]

test 'Advanced locate: locate-all multiple POIs per location':
    let tp = TripPlanner(
        [[0, 0, 1, 0],
        [1, 0, 2, 0],
        [2, 0, 3, 0]],
        [[1, 0, 'cafe', 'Coffee Lab'],
        [2, 0, 'restaurant', 'Sushi'],
        [2, 0, 'cafe', 'Collectivo']])
    let result = tp.locate_all('cafe')
    assert Cons.to_vec(result) == [[2, 0], [1, 0]]

test 'locate-all edge cases':
    let tp = TripPlanner([], [])
    let result = tp.locate_all('restaurant')
    assert Cons.to_vec(result) == []

test 'Basic route: plan-route with one possible path':
    let tp = TripPlanner(
        [[0, 0, 1, 0],
        [1, 0, 2, 0],
        [2, 0, 3, 0]],
        [[1, 0, 'cafe', 'Coffee Lab'],
        [2, 0, 'restaurant', 'Sushi']])
    let result = tp.plan_route(0, 0, 'Sushi')
    assert Cons.to_vec(result) == [[0, 0], [1, 0], [2, 0]]

test 'Advanced route: plan-route with multiple possible paths':
    let tp = TripPlanner(
        [[0, 0, 1, 0],
        [1, 0, 2, 0],
        [2, 0, 3, 0],
        [1, 0, 2, 0]],
        [[1, 0, 'cafe', 'Coffee Lab'],
        [2, 0, 'restaurant', 'Sushi']])
    let result = tp.plan_route(0, 0, 'Sushi')
    assert Cons.to_vec(result) == [[0, 0], [1, 0], [2, 0]]

test 'plan-route edge cases':
    let tp = TripPlanner([], [])
    let result = tp.plan_route(0, 0, 'Sushi')
    assert Cons.to_vec(result) == []

test 'Basic nearby: find-nearby looking for the one closest relevant POI':
    let tp = TripPlanner(
        [[0, 0, 1, 0],
        [1, 0, 2, 0],
        [2, 0, 3, 0]],
        [[1, 0, 'cafe', 'Coffee Lab'],
        [2, 0, 'restaurant', 'Sushi']])
    let result = tp.find_nearby(0, 0, 'cafe', 1)
    assert Cons.to_vec(result) == [[1, 0, 'cafe', 'Coffee Lab']]

test 'Advanced nearby: find-nearby looking for multiple closest POIs':
    let tp = TripPlanner(
        [[0, 0, 1, 0],
        [1, 0, 2, 0],
        [2, 0, 3, 0]],
        [[1, 0, 'cafe', 'Coffee Lab'],
        [2, 0, 'restaurant', 'Sushi'],
        [2, 0, 'cafe', 'Collectivo']])
    let result = tp.find_nearby(0, 0, 'cafe', 2)
    assert Cons.to_vec(result) == [[2, 0, 'cafe', 'Collectivo'], [1, 0, 'cafe', 'Coffee Lab']]

test 'find-nearby edge cases':
    let tp = TripPlanner([], [])
    let result = tp.find_nearby(0, 0, 'cafe', 1)
    assert Cons.to_vec(result) == []
    
# TEST CASES FROM REPORT
test 'from report 1':
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[1, 0, 'bank', 'Union']])
    let result = tp.plan_route(0, 0, 'Union')
    assert Cons.to_vec(result) == [[0, 0], [1, 0]]
test 'from report 2':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [2.5, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Tony')
    assert Cons.to_vec(result) == [[0, 0], [1.5, 0], [2.5, 0]]
test 'from report 3':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Tony')
    assert Cons.to_vec(result) == [[0, 0], [1.5, 0], [2.5, 0], [3, 0]]
test 'from report 4':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(3, 0, 'Union')
    assert Cons.to_vec(result) == [[3, 0], [2.5, 0], [1.5, 0]]
test 'from report 5':
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[0, 0, 'bank', 'Union']])
    let result = tp.plan_route(0, 0, 'Union')
    assert Cons.to_vec(result) == [[0, 0]]
test 'from report ella':
    let tp = TripPlanner(
    [[0,0,1,0],[0,1,2,3]],
    [[0,0,'bank','Union'],
    [1,0,'coffee','starbs'],
    [2,3,'tea','lemon']])
    let result = tp.plan_route(0,0,'lemon')
    assert result == None
test 'from report 6':    
    let tp = TripPlanner(
      [[0, 0, 1, 0]],
      [[1, 0, 'bank', 'Union']])
    let result = tp.plan_route(0, 0, 'Union')
    assert Cons.to_vec(result) == [[0, 0], [1, 0]]
test 'from report 7':   
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [2.5, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Tony')
    assert Cons.to_vec(result) == [[0, 0], [1.5, 0], [2.5, 0]]
test 'from report 8':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Tony')
    assert Cons.to_vec(result) == [[0, 0], [1.5, 0], [2.5, 0], [3, 0]]
test 'from report 9':
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

test 'from report 10':
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

test 'from report 11':
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

test 'from report 12': 
    let tp = TripPlanner(
      [[0, 0, 1, 0],
       [1, 0, 2, 0],
       [2, 0, 3, 0],
       [3, 0, 4, 0],
       [4, 0, 5, 0]],
      [[1, 0, 'food', 'Pizza Hut'],
       [3, 0, 'bar', 'Sports Bar'],
       [5, 0, 'bar', 'Pub']])
    let result = tp.plan_route(0, 0, 'Pub')
    assert Cons.to_vec(result) \
      == [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0]]
test 'from report 13':
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
    assert Cons.to_vec(result) == [[0, 0], [2, 1], [6, 0]]

test 'from report 16':
    let tp = TripPlanner(
      [[0, 0, 1, 0],
       [1, 0, 2, 0],
       [2, 0, 3, 0],
       [3, 0, 4, 0],
       [4, 0, 5, 0]],
      [[1, 0, 'food', 'Pizza Hut'],
       [3, 0, 'bar', 'Sports Bar'],
       [5, 0, 'bar', 'Pub']])
    let result1 = tp.plan_route(0, 0, 'Pub')
    let result2 = tp.plan_route(0, 0, 'Sports Bar')
    assert Cons.to_vec(result1) == [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0]]
    assert Cons.to_vec(result2) == [[0, 0], [1, 0], [2, 0], [3, 0]]

       
