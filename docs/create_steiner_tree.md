# Creating a Steiner Graph in PR3DCluster

This document explains the process of creating a Steiner Graph in PR3DCluster as outlined in the code files. The Steiner Graph creation is an important step in processing 3D point clusters and finding optimal connections between points.

## Overview

A Steiner Graph is a graph that connects a set of required points (terminals) using additional optional points (Steiner points) to minimize the total edge length. In the context of PR3DCluster, this helps in connecting related points in 3D space while minimizing connections.

## The Creation Process

The creation of a Steiner Graph involves several steps:

1. **Initialization**: Call `create_steiner_graph()` method in PR3DCluster
2. **Create a new improved cluster**: Use `Improve_PR3DCluster_2()` to create a refined cluster
3. **Create Point Cloud and Graph**: Initialize data structures in the new cluster
4. **Establish steiner edges**: Add edges between points in the same merged cell
5. **Find shortest paths**: Run Dijkstra's algorithm to find shortest paths between boundary points
6. **Remove temporary edges**: Clean up the graph by removing the same-mcell steiner edges
7. **Create the Steiner Tree**: Generate the final tree and save it in the original cluster

## Detailed Workflow

### 1. Creating the Initial Steiner Graph Structure

The process begins with calling `create_steiner_graph()` which checks if a graph already exists and then creates a new one:

```cpp
void WCPPID::PR3DCluster::create_steiner_graph(WCP::ToyCTPointCloud& ct_point_cloud, WCPSst::GeomDataSource& gds, int nrebin, int frame_length, double unit_dis) {
  if (graph_steiner == (MCUGraph*)0) {
    // ... implementation follows
  }
}
```

### 2. Creating an Improved Cluster

The method `Improve_PR3DCluster_2()` is called to create a refined cluster based on the original:

```cpp
WCPPID::PR3DCluster *new_cluster = WCPPID::Improve_PR3DCluster_2(this, ct_point_cloud, gds, temp_holder, nrebin, frame_length, unit_dis);
```

This improvement process involves:
- Creating a graph from the cluster
- Adding edges between points within the same merged cell
- Finding boundary points
- Calculating shortest paths between those points

### 3. Point Cloud and Graph Creation

The new cluster creates its point cloud and graph structures:

```cpp
new_cluster->Create_point_cloud();
new_cluster->Create_graph(ct_point_cloud, point_cloud);
```

### 4. Steiner Tree Creation

After establishing paths, the Steiner tree is created:

```cpp
graph_steiner = new_cluster->Create_steiner_tree(point_cloud_steiner, flag_steiner_terminal, gds, mcells, true, false);
```

This process:
1. Identifies terminal points (steiner_terminal_indices)
2. Connects these terminals efficiently using additional points as needed
3. Creates a minimal spanning tree between terminals

### 5. Terminal Point Processing

Finally, the terminal points from the Steiner tree are extracted and saved:

```cpp
point_cloud_steiner_terminal = new ToyPointCloud();
// ... code to extract terminal points
```

## The Steiner Tree Structure

The created Steiner Tree consists of two main components:

1. **Point Cloud** (`point_cloud_steiner`): Contains all points in the Steiner tree
2. **Graph** (`graph_steiner`): Contains the connectivity information between points

The terminal points (required points that must be connected) are flagged in the `flag_steiner_terminal` vector and also saved in a separate point cloud (`point_cloud_steiner_terminal`).

## Technical Details

The Steiner tree algorithm used is a variant of greedy algorithms for Steiner tree problems:

1. Identify terminal points using charge values and spatial proximity
2. Create a Voronoi diagram to partition the space
3. Build connections between terminals using minimum spanning tree
4. Optimize the tree by adding necessary Steiner points

The underlying implementation uses Boost Graph Library for graph operations and efficient algorithms like Dijkstra and Kruskal's minimum spanning tree.

## Diagram of the Process

```
+----------------+      +---------------------+
| Original       |      | Improved Cluster    |
| PR3DCluster    |----->| (via Improve_PR3D   |
|                |      | Cluster_2)          |
+----------------+      +---------------------+
                              |
                              v
                        +----------------+
                        | Create Point   |
                        | Cloud & Graph  |
                        +----------------+
                              |
                              v
                        +----------------+
                        | Establish      |
                        | Steiner Edges  |
                        +----------------+
                              |
                              v
                        +----------------+
                        | Find Shortest  |
                        | Paths          |
                        +----------------+
                              |
                              v
                        +----------------+
                        | Remove         |
                        | Temporary Edges|
                        +----------------+
                              |
                              v
                        +----------------+      +------------------+
                        | Create Final   |----->| Point Cloud for  |
                        | Steiner Tree   |      | Steiner Tree     |
                        +----------------+      +------------------+
                              |                        |
                              v                        v
                        +----------------+      +------------------+
                        | Store in       |      | Terminal Point   |
                        | Original       |      | Cloud (flagged   |
                        | PR3DCluster    |      | terminals)       |
                        +----------------+      +------------------+
```







# Steiner Tree Creation Algorithm in PR3DCluster

## Overview

The `Create_Steiner_Tree` function in PR3DCluster implements a variant of the Steiner tree algorithm for finding optimal connections between a set of terminal points in 3D space. The algorithm aims to minimize the total connection distance while potentially using additional points (Steiner points) that weren't part of the original set of required points.

## Algorithm Flow

The algorithm follows these major steps:

1. **Graph Creation**: Initialize the underlying graph structure
2. **Terminal Identification**: Find and mark the terminal points
3. **Voronoi Partitioning**: Create Voronoi regions around terminals
4. **Terminal Graph Construction**: Build a graph connecting terminals
5. **Minimum Spanning Tree**: Compute the MST on the terminal graph
6. **Path Recovery**: Recover full paths between terminals
7. **Final Tree Creation**: Build the actual Steiner tree

## Detailed Algorithm

### 1. Graph Creation

```cpp
Create_graph();
```

The function begins by creating an initial graph representing all points in the point cloud. This serves as the basis for the Steiner tree construction.

### 2. Terminal Identification

```cpp
find_steiner_terminals(gds, disable_dead_mix_cell);
```

Terminal points are identified through the `find_steiner_terminals` method. This step:

- Uses charge information from merged cells (mcells)
- Identifies peak points with high charge values
- Filters based on spatial constraints
- Creates a set of terminal indices (`steiner_terminal_indices`)

The terminal finding process is crucial as it determines the required connection points. The algorithm specifically:

1. Maps cells to their associated points
2. For each merged cell, finds peak charge points
3. Examines charge values and connectivity to neighboring points
4. Selects points with charge above threshold (around 4000 units)
5. Uses a graph component analysis to ensure unique peak points per connected region

### 3. Voronoi Partitioning & Terminal Graph Construction

```cpp
// Create terminal vectors from sets
std::vector<int> terminals(steiner_terminal_indices.begin(), steiner_terminal_indices.end());

// Compute Voronoi diagram to partition the space
boost::dijkstra_shortest_paths(*graph, terminals.begin(), terminals.end(), ...);
```

A Voronoi diagram is implicitly created by:

1. Running Dijkstra's algorithm from all terminal points simultaneously
2. For each vertex in the graph, finding the nearest terminal
3. Recording the path to that nearest terminal
4. Creating a "nearest_terminal_map" that assigns each vertex to its closest terminal

This partitioning helps group points around their nearest terminals, essential for the next step.

### 4. Terminal Graph Construction

```cpp
// Creating terminal_graph with edges between Voronoi regions
TerminalGraph terminal_graph(N);
std::map<std::pair<int, int>, std::pair<Weight, Edge>> map_saved_edge;

// Find cross-region edges
for (auto w : boost::as_array(edges(*graph))) {
  auto const &nearest_to_source = nearest_terminal_map[source(w, *graph)];
  auto const &nearest_to_target = nearest_terminal_map[target(w, *graph)];
  
  if (nearest_to_source != nearest_to_target) {
    // Process edges that cross Voronoi boundaries
  }
}
```

The terminal graph connects different Voronoi regions by:

1. Examining all edges in the original graph
2. Finding edges that cross Voronoi region boundaries (connect vertices assigned to different terminals)
3. Computing the weight of a path between the corresponding terminals through this edge
4. Keeping the minimum-weight edge for each terminal pair

### 5. Minimum Spanning Tree Computation

Although the commented-out code mentions Kruskal's algorithm, the implementation uses a direct edge addition approach for all terminal-terminal connections:

```cpp
std::vector<Edge> terminal_edge;
for (auto it = map_saved_edge.begin(); it!=map_saved_edge.end(); it++){
  std::pair<Edge, bool> p = add_edge(it->first.first, it->first.second,
                     WeightProperty(it->second.first, Base(it->second.second)),terminal_graph);
  terminal_edge.push_back(p.first);
}
```

This builds a complete graph between terminals where the edge weights represent the shortest path distances between terminals.

### 6. Path Recovery

```cpp
// Computing result
std::vector<Edge> tree_edges;
for (auto edge : terminal_edge) {
  auto base = get(edge_base, terminal_graph, edge);
  tree_edges.push_back(base);
  
  // Recover paths between terminals
  for (auto pom : { source(base, *graph), target(base, *graph) }) {
    while (nearest_terminal_map[pom] != pom) {
      tree_edges.push_back(vpred[pom]);
      pom = source(vpred[pom], *graph);
    }
  }
}
```

The algorithm recovers the full paths between terminals by:

1. Starting with the edges connecting terminals in the MST
2. For each edge endpoint, following the predecessor chain back to its assigned terminal
3. Adding all edges along this path to the result set
4. Removing duplicate edges

This creates a connected tree that includes both terminal points and necessary Steiner points.

### 7. Final Tree Creation and Charge-Based Adjustment

```cpp
// Create the final Steiner graph with charge-adjusted weights
MCUGraph* graph_steiner = new MCUGraph(flag_steiner_terminal.size());
for (auto e : unique_edges){
  int index1 = map_old_new_indices[index[source(e,*graph)]];
  int index2 = map_old_new_indices[index[target(e,*graph)]];
  float dis = get(boost::edge_weight_t(), *graph, e);

  // Charge-based weight adjustment
  float Q0 = 10000; // constant term
  float Qs = map_index_charge[index[source(e,*graph)]];
  float Qt = map_index_charge[index[target(e,*graph)]];
  float factor1=0.8, factor2=0.4; 
  
  float temp_dis = dis * (factor1 + factor2 * (0.5*Q0/(Qs+Q0) + 0.5*Q0/(Qt+Q0)));
  
  auto edge = add_edge(index1, index2, temp_dis, *graph_steiner);
}
```

The final graph is created with charge-aware weighting:

1. A new graph is created specifically for the Steiner tree
2. For each edge in the selected path, an edge is added to the Steiner graph
3. The edge weight is adjusted based on charge values at the endpoints
4. This adjusts the importance of edges connecting high-charge points
5. Terminal points are marked with a flag vector for later reference

## Charge-Based Weight Adjustment

A distinctive feature of this algorithm is the charge-based weight adjustment. For each edge, the physical distance is modified by a formula:

```
weight = distance * (0.8 + 0.4 * (0.5*Q0/(Qs+Q0) + 0.5*Q0/(Qt+Q0)))
```

Where:
- `distance` is the spatial distance
- `Q0` is a constant (10000)
- `Qs` and `Qt` are the charge values at source and target vertices

This adjustment:
- Reduces the effective distance between high-charge points
- Makes high-charge paths more likely to be included in the solution
- Produces more physically meaningful connections in detector data

## Algorithm Diagram

```
       ┌─────────────────┐
       │  Create Graph   │
       └────────┬────────┘
                │
       ┌────────▼────────┐
       │    Identify     │
       │  Terminal Points│
       └────────┬────────┘
                │
       ┌────────▼────────┐
       │  Run Dijkstra   │
       │ from Terminals  │◄───┐
       └────────┬────────┘    │
                │             │ Assigns each vertex
                │             │ to nearest terminal
       ┌────────▼────────┐    │
       │Create Voronoi   │────┘
       │  Partitioning   │
       └────────┬────────┘
                │
       ┌────────▼────────┐
       │ Find Boundary   │
       │     Edges       │
       └────────┬────────┘
                │
       ┌────────▼────────┐
       │Build Terminal   │
       │     Graph       │
       └────────┬────────┘
                │
       ┌────────▼────────┐
       │  Connect All    │
       │  Terminal Pairs │
       └────────┬────────┘
                │
       ┌────────▼────────┐
       │ Recover Full    │
       │  Steiner Paths  │
       └────────┬────────┘
                │
       ┌────────▼────────┐
       │Create Final Tree│
       │with Charge-Based│
       │  Weighting      │
       └────────┬────────┘
                │
       ┌────────▼────────┐
       │Return Steiner   │
       │     Tree        │
       └─────────────────┘
```

## Visual Example of Steiner Tree Creation

```
                Terminal Points                                Final Steiner Tree
                
        *                   *                          *─────────────*
                                                       │             │
                                                       │             │
                                                       │             │
                * Terminal                             *             *
                                      ───────►        /│\           
                                                      / │ \          
        *               *                            /  │  \         
                                                    /   │   \        
                                                   *────┼────*       
                                                        │            
                                                        │            
                                                        │            
        *               *                               *             *
```

## Performance Considerations

The implemented algorithm has certain performance characteristics:

1. **Time Complexity**: The dominant costs are:
   - Dijkstra's algorithm: O(E + V log V) where E is edges and V is vertices
   - Edge processing: O(E)
   - Path recovery: O(V)

2. **Space Complexity**: O(V + E) for the graph structures and mappings

3. **Optimizations**:
   - Uses Boost Graph Library for efficient graph operations
   - Employs spatial-based filtering to reduce the search space
   - Caches shortest paths to avoid redundant computations
   - Implements charge-based weighting to prioritize important connections

This implementation balances computational efficiency with the need to create physically meaningful Steiner trees for 3D point clusters in detector data.




# Recovering the Steiner Graph: `recover_steiner_graph()` Algorithm

## Overview

The `recover_steiner_graph()` function in the PR3DCluster class is designed to recover a Steiner tree from a previously created Steiner graph. This process effectively reconstructs the Steiner tree by identifying the optimal paths between terminal points. Unlike the initial Steiner graph creation, this recovery function works with an existing graph structure and focuses on finding the minimal connecting tree.

## Algorithm Purpose

The primary purpose of this algorithm is to:

1. Identify the terminal points in the Steiner graph
2. Create a Voronoi partitioning around these terminals
3. Find optimal connections between terminal regions
4. Compute a minimum spanning tree between terminals
5. Recover the complete Steiner tree with all necessary intermediate points

## Detailed Algorithm Steps

### 1. Terminal Identification

```cpp
if (graph_steiner != (MCUGraph*)0) {
  steiner_graph_terminal_indices.clear();
  std::vector<int> terminals;
  for (size_t i = 0; i != flag_steiner_terminal.size(); i++) {
    if (flag_steiner_terminal[i]) {
      terminals.push_back(i);
      steiner_graph_terminal_indices.insert(i);
    }
  }
```

The function first checks if a Steiner graph exists (`graph_steiner != (MCUGraph*)0`). If it does, it:
- Clears any existing terminal indices
- Creates a vector to hold terminal point indices
- Iterates through the `flag_steiner_terminal` vector to identify terminal points
- Adds these terminal indices to both the vector and a set for easy lookup

### 2. Graph Type Definitions

```cpp
const int N = point_cloud_steiner->get_num_points();
using Vertex = typename boost::graph_traits<WCPPID::MCUGraph>::vertex_descriptor;
using Edge = typename boost::graph_traits<WCPPID::MCUGraph>::edge_descriptor;
using Base = typename boost::property<edge_base_t, Edge>;
using EdgeWeightMap = typename boost::property_map<WCPPID::MCUGraph, boost::edge_weight_t>::type;
using Weight = typename boost::property_traits<EdgeWeightMap>::value_type;
using WeightProperty = typename boost::property<boost::edge_weight_t, Weight, Base>;
using TerminalGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                           boost::no_property, WeightProperty>;
using EdgeTerminal = typename boost::graph_traits<TerminalGraph>::edge_descriptor;
```

The algorithm defines various type aliases using the Boost Graph Library (BGL) to:
- Define vertex and edge descriptors for the MCUGraph
- Set up property maps for edge weights
- Create a specialized terminal graph type that will store connections between terminals

### 3. Edge Weight Map and Distance Array

```cpp
EdgeWeightMap edge_weight = boost::choose_pmap(boost::get_param(boost::no_named_parameters(), 
                                               boost::edge_weight), *graph_steiner, boost::edge_weight);
std::vector<Weight> distance(N);
```

The function:
- Obtains the edge weight property map from the Steiner graph
- Creates a distance array that will be used to store shortest distances during Dijkstra's algorithm

### 4. Voronoi Partitioning with Dijkstra's Algorithm

```cpp
std::vector<Vertex> nearest_terminal(num_vertices(*graph_steiner));
auto index = get(boost::vertex_index, *graph_steiner);
auto nearest_terminal_map = boost::make_iterator_property_map(nearest_terminal.begin(), 
                            get(boost::vertex_index, *graph_steiner));

for (auto terminal : terminals) {
  nearest_terminal_map[terminal] = terminal;
}

auto distance_map = make_iterator_property_map(distance.begin(), index);
std::vector<Edge> vpred(N);
auto last_edge = boost::make_iterator_property_map(vpred.begin(), 
              get(boost::vertex_index, *graph_steiner));

boost::dijkstra_shortest_paths(*graph_steiner, terminals.begin(), terminals.end(), 
                              boost::dummy_property_map(),
                              distance_map, edge_weight, index, paal::utils::less(),
                              boost::closed_plus<Weight>(), std::numeric_limits<Weight>::max(), 0,
                              boost::make_dijkstra_visitor(paal::detail::make_nearest_recorder(
                              nearest_terminal_map, last_edge, boost::on_edge_relaxed{})));
```

This critical step:
- Creates a vector to store the nearest terminal for each vertex
- Sets up a property map for easy access to this information
- Initializes each terminal to be its own nearest terminal
- Creates a distance map for Dijkstra's algorithm
- Sets up a predecessor edge map to record the path to each vertex
- Runs a multi-source Dijkstra's shortest paths algorithm from all terminals simultaneously
- Uses a custom visitor that records the nearest terminal and the last edge on the path to it

This effectively creates a Voronoi partition of the graph, where each vertex is assigned to its closest terminal.

### 5. Terminal Graph Construction

```cpp
TerminalGraph terminal_graph(N);
std::map<std::pair<int, int>, std::pair<Weight, Edge>> map_saved_edge;

for (auto w : boost::as_array(edges(*graph_steiner))) {
  auto const &nearest_to_source = nearest_terminal_map[source(w, *graph_steiner)];
  auto const &nearest_to_target = nearest_terminal_map[target(w, *graph_steiner)];
  
  if (nearest_to_source != nearest_to_target) {
    Weight temp_weight = distance[source(w, *graph_steiner)] + distance[target(w, *graph_steiner)] + edge_weight[w];
    
    // Track the minimum-weight edge between each terminal pair
    if (map_saved_edge.find(std::make_pair(nearest_to_source, nearest_to_target)) != map_saved_edge.end()) {
      if (temp_weight < map_saved_edge[std::make_pair(nearest_to_source, nearest_to_target)].first)
        map_saved_edge[std::make_pair(nearest_to_source, nearest_to_target)] = std::make_pair(temp_weight, w);
    } else if (map_saved_edge.find(std::make_pair(nearest_to_target, nearest_to_source)) != map_saved_edge.end()) {
      if (temp_weight < map_saved_edge[std::make_pair(nearest_to_target, nearest_to_source)].first)
        map_saved_edge[std::make_pair(nearest_to_target, nearest_to_source)] = std::make_pair(temp_weight, w);
    } else {
      map_saved_edge[std::make_pair(nearest_to_source, nearest_to_target)] = std::make_pair(temp_weight, w);
    }
  }
}
```

This step:
- Creates a new graph specifically for terminal connections
- Loops through all edges in the original Steiner graph
- Identifies edges that cross Voronoi region boundaries (connect vertices assigned to different terminals)
- Calculates the weight of a path between the corresponding terminals through this edge
- Maintains a map of the minimum-weight edge for each terminal pair

### 6. Terminal Graph Edge Creation

```cpp
std::vector<Edge> terminal_edge;
for (auto it = map_saved_edge.begin(); it != map_saved_edge.end(); it++) {
  std::pair<Edge, bool> p = add_edge(it->first.first, it->first.second,
                          WeightProperty(it->second.first, Base(it->second.second)), terminal_graph);
  terminal_edge.push_back(p.first);
}
```

The function:
- Creates edges in the terminal graph for each terminal pair
- Uses the minimum-weight edges found in the previous step
- Stores the edge property with both the weight and a reference to the original edge

### 7. Kruskal's Minimum Spanning Tree

```cpp
boost::kruskal_minimum_spanning_tree(terminal_graph, std::back_inserter(terminal_edge));
```

This line is a bit confusing in the original code since terminal_edge is both used to store the initial edges and the MST edges. In practice, this would create a new terminal_edge vector with only the MST edges.

### 8. Path Recovery

```cpp
std::vector<Edge> tree_edges;
for (auto edge : terminal_edge) {
  auto base = get(edge_base, terminal_graph, edge);
  tree_edges.push_back(base);
  for (auto pom : { source(base, *graph), target(base, *graph) }) {
    while (nearest_terminal_map[pom] != pom) {
      tree_edges.push_back(vpred[pom]);
      pom = source(vpred[pom], *graph);
    }
  }
}
```

This step:
- Creates a vector to store all edges in the final Steiner tree
- For each edge in the terminal MST, recovers the original edge from the base property
- For each endpoint of the edge, follows the predecessor chain back to its terminal
- Adds all edges along these paths to the result set

### 9. Remove Duplicate Edges

```cpp
boost::sort(tree_edges);
auto unique_edges = boost::unique(tree_edges);
```

Since the paths between terminals may share edges, this step:
- Sorts the edge vector to group duplicates
- Removes duplicate edges using the `unique` algorithm

### 10. Terminal Selection

```cpp
steiner_graph_selected_terminal_indices.clear();
for (auto e : unique_edges) {
  steiner_graph_selected_terminal_indices.insert(index[source(e, *graph)]);
  steiner_graph_selected_terminal_indices.insert(index[target(e, *graph)]);
}
```

Finally, the function:
- Clears any existing selected terminal indices
- For each edge in the unique edges set, adds both endpoints to the selected terminal indices set
- This set represents all points (both terminals and intermediate points) that are part of the Steiner tree

## Visual Algorithm Flow

```
┌─────────────────────┐
│ Check if Steiner    │
│ Graph Exists        │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Identify Terminal   │
│ Points              │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Set Up Voronoi      │
│ Partitioning        │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Run Multi-Source    │
│ Dijkstra's Algorithm│
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Find Boundary Edges │
│ Between Regions     │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Build Terminal Graph│
│ with Min-Weight     │
│ Connections         │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Compute Minimum     │
│ Spanning Tree       │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Recover Complete    │
│ Paths Between       │
│ Terminals           │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Remove Duplicate    │
│ Edges               │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Update Selected     │
│ Terminal Indices    │
└─────────────────────┘
```

## Voronoi Partitioning Illustration

```
         Original Graph with Terminals (*)
         
         O───────O───────O───────O
         │       │       │       │
         │       │       │       │
         │       │       │       │
         O───────*───────O───────O
         │       │       │       │
         │       │       │       │
         │       │       │       │
         O───────O───────*───────O
         │       │       │       │
         │       │       │       │
         │       │       │       │
         O───────O───────O───────O


         After Voronoi Partitioning
         
         1───────1───────2───────2
         │       │       │       │
         │       │       │       │
         │       │       │       │
         1───────*1──────2───────2
         │       │       │       │
         │       │       │       │
         │       │       │       │
         1───────1───────*2──────2
         │       │       │       │
         │       │       │       │
         │       │       │       │
         1───────1───────2───────2

         (Numbers indicate region assignment)
```

## Boundary Edge and MST Construction

```
      Terminal Graph with Boundary Edges
      
              *1
              │
              │
              │ (weight=5)
              │
              *2

      After MST Computation (with path recovery)
      
              *1
              │
              O Terminal 1's region
              │
              O Intermediate point
              │
              O Terminal 2's region
              │
              *2
```

## Complexity Analysis

- **Time Complexity**:
  - Dijkstra's algorithm: O(E + V log V) where E is the number of edges and V is the number of vertices
  - Edge processing: O(E)
  - Kruskal's algorithm: O(E log E) where E is the number of terminal-to-terminal edges
  - Path recovery: O(V)
  - Overall: O(E log V) dominated by Dijkstra's and Kruskal's algorithms

- **Space Complexity**:
  - O(V + E) for the graph structures, distance arrays, and mappings

## Key Differences from `Create_Steiner_Tree`

While both functions generate a Steiner tree, `recover_steiner_graph` differs from `Create_Steiner_Tree` in several ways:

1. **Starting Point**: Works with an existing Steiner graph rather than creating one
2. **Terminal Selection**: Uses pre-defined terminals from `flag_steiner_terminal` rather than finding them
3. **Result Storage**: Updates `steiner_graph_selected_terminal_indices` rather than creating a new graph
4. **Weight Handling**: Does not apply charge-based weight adjustments
5. **Optimization Focus**: Focuses purely on topological optimization rather than incorporating physics-based weighting

## Applications

The `recover_steiner_graph` function is particularly useful for:

1. Updating an existing Steiner tree when small changes occur in the terminal set
2. Analyzing different Steiner tree configurations without recreating the entire graph
3. Extracting a Steiner tree from a graph where the terminal points are already known
4. Finding the minimal connecting structure in a 3D point cloud with predefined terminals

This algorithm provides an efficient way to extract optimal connecting paths in 3D space while minimizing the total edge weight, making it valuable for applications in 3D reconstruction, network design, and path planning.