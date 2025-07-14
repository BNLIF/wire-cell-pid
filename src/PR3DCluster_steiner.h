#include "WCPPaal/graph_metrics.h"
#include "WCPPaal/steiner_tree_greedy.h"
#include "WCPPID/ImprovePR3DCluster.h"
#include "WCP2dToy/WCPHolder.h"
#include "WCPPID/CalcPoints.h"




void WCPPID::PR3DCluster::create_steiner_graph(WCP::ToyCTPointCloud& ct_point_cloud,WCPSst::GeomDataSource& gds, int nrebin, int frame_length, double unit_dis){

  if (graph_steiner == (MCUGraph*)0){
    WCP2dToy::WCPHolder *temp_holder = new WCP2dToy::WCPHolder();
    WCPPID::PR3DCluster *new_cluster = WCPPID::Improve_PR3DCluster_2(this, ct_point_cloud, gds, temp_holder, nrebin, frame_length, unit_dis); 
    
    WCPPID::calc_sampling_points(gds,new_cluster,nrebin, frame_length, unit_dis,false);


    
    new_cluster->Create_point_cloud(); 
    new_cluster->Create_graph(ct_point_cloud, point_cloud);

    std::cout << "ImproveCluster_2: " << new_cluster->get_point_cloud()->get_num_points() << " points created " << new_cluster->get_mcells().size() << std::endl;

    std::cout << "CreateSteinerTree: Graph vertices " << num_vertices(*new_cluster->get_graph()) << " edges " << num_edges(*new_cluster->get_graph()) << " points " << new_cluster->get_point_cloud()->get_num_points() << std::endl;

    new_cluster->establish_same_mcell_steiner_edges(gds,false);

    // find the shortest path
    std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> wcps = new_cluster->get_two_boundary_wcps(); 

    std::cout << "CreateSteinerTree: " << wcps.first.x << " " << wcps.first.y << " " << wcps.first.z << " | " << wcps.second.x << " " << wcps.second.y << " " << wcps.second.z << std::endl;

    new_cluster->dijkstra_shortest_paths(wcps.first); 
    new_cluster->cal_shortest_path(wcps.second);

    std::cout << "CreateSteinerTree: Shortest path indices " << new_cluster->get_path_wcps().size() << " Graph vertices " << num_vertices(*new_cluster->get_graph()) << " edges " << num_edges(*new_cluster->get_graph()) << " points " << new_cluster->get_point_cloud()->get_num_points() << std::endl;

    new_cluster->remove_same_mcell_steiner_edges();
    std::cout << "CreateSteinerTree: Graph vertices " << num_vertices(*new_cluster->get_graph()) << " edges " << num_edges(*new_cluster->get_graph()) << " points " << new_cluster->get_point_cloud()->get_num_points() << std::endl;

    
    point_cloud_steiner = new ToyPointCloud();

    // steiner tree with some basic cuts ...
    graph_steiner = new_cluster->Create_steiner_tree(point_cloud_steiner, flag_steiner_terminal, gds, mcells, true, false);
    establish_same_mcell_steiner_edges(gds,true,2);

    size_t num_true_terminals = std::count(flag_steiner_terminal.begin(), flag_steiner_terminal.end(), true);

    std::cout << "CreateSteinerTree: steiner graph with " << num_vertices(*graph_steiner) << " vertices and " << num_edges(*graph_steiner) << " edges " <<  point_cloud_steiner->get_num_points() << " " << num_true_terminals << std::endl;

    new_cluster->Del_graph();
    new_cluster->Del_point_cloud();
    
    delete new_cluster;
    delete temp_holder;
    
    point_cloud_steiner_terminal = new ToyPointCloud();
    WCP::WCPointCloud<double>& cloud = point_cloud_steiner->get_cloud();
    WCP::WC2DPointCloud<double>& cloud_u = point_cloud_steiner->get_cloud_u();
    WCP::WC2DPointCloud<double>& cloud_v = point_cloud_steiner->get_cloud_v();
    WCP::WC2DPointCloud<double>& cloud_w = point_cloud_steiner->get_cloud_w();
    for (size_t i=0;i!=flag_steiner_terminal.size();i++){
      if (flag_steiner_terminal[i]){
	         point_cloud_steiner_terminal->AddPoint(cloud.pts[i],cloud_u.pts[i],cloud_v.pts[i],cloud_w.pts[i]);
      }
    }
    
    // examine steiner tree terminals
    
    //return new_cluster;
  }
  
}

void WCPPID::PR3DCluster::recover_steiner_graph(){

  //std::cout << "haha " << graph_steiner << std::endl;
  // holder for more sophisticated algorithm later ...
  if (graph_steiner != (MCUGraph*)0){
    steiner_graph_terminal_indices.clear();
    std::vector<int> terminals;
    for (size_t i = 0;i!=flag_steiner_terminal.size();i++){
      if (flag_steiner_terminal[i]){
	terminals.push_back(i);
	steiner_graph_terminal_indices.insert(i);
      }
    }
    const int N = point_cloud_steiner->get_num_points();
    using Vertex = typename boost::graph_traits<WCPPID::MCUGraph>::vertex_descriptor;
    using Edge = typename boost::graph_traits<WCPPID::MCUGraph>::edge_descriptor;
    using Base = typename boost::property<edge_base_t, Edge>;
    using EdgeWeightMap = typename boost::property_map<WCPPID::MCUGraph, boost::edge_weight_t>::type;
    using Weight = typename boost::property_traits<EdgeWeightMap>::value_type;
    using WeightProperty =
      typename boost::property<boost::edge_weight_t, Weight, Base>;
    using TerminalGraph =
      boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
      boost::no_property, WeightProperty>;
    using EdgeTerminal =
      typename boost::graph_traits<TerminalGraph>::edge_descriptor;

    EdgeWeightMap edge_weight = boost::choose_pmap(boost::get_param(boost::no_named_parameters(), boost::edge_weight), *graph_steiner, boost::edge_weight);
  
    // distance array used in the dijkstra runs
    std::vector<Weight> distance(N);

    std::vector<Vertex> nearest_terminal(num_vertices(*graph_steiner));
    auto index = get(boost::vertex_index, *graph_steiner);
    auto nearest_terminal_map = boost::make_iterator_property_map(nearest_terminal.begin(), get(boost::vertex_index, *graph_steiner));
    for (auto terminal : terminals) {
      nearest_terminal_map[terminal] = terminal;
    }
    // compute voronoi diagram each vertex get nearest terminal and last edge on
    // path to nearest terminal
    auto distance_map = make_iterator_property_map(distance.begin(), index);
    std::vector<Edge> vpred(N);
    auto last_edge = boost::make_iterator_property_map(vpred.begin(), get(boost::vertex_index, *graph_steiner));
    boost::dijkstra_shortest_paths(*graph_steiner, terminals.begin(), terminals.end(), boost::dummy_property_map(),
				   distance_map, edge_weight, index, paal::utils::less(),
				   boost::closed_plus<Weight>(), std::numeric_limits<Weight>::max(), 0,
				   boost::make_dijkstra_visitor(paal::detail::make_nearest_recorder(
												    nearest_terminal_map, last_edge, boost::on_edge_relaxed{})));
    
    // computing distances between terminals
    // creating terminal_graph
    TerminalGraph terminal_graph(N);
    std::map<std::pair<int, int>, std::pair<Weight, Edge> > map_saved_edge;
    
    for (auto w : boost::as_array(edges(*graph_steiner))) {
      auto const &nearest_to_source = nearest_terminal_map[source(w, *graph_steiner)];
      auto const &nearest_to_target = nearest_terminal_map[target(w, *graph_steiner)];
      if (nearest_to_source != nearest_to_target) {
	Weight temp_weight = distance[source(w, *graph_steiner)] + distance[target(w, *graph_steiner)] + edge_weight[w];
	if (map_saved_edge.find(std::make_pair(nearest_to_source, nearest_to_target))!=map_saved_edge.end()){
	  if (temp_weight < map_saved_edge[std::make_pair(nearest_to_source, nearest_to_target)].first)
	    map_saved_edge[std::make_pair(nearest_to_source, nearest_to_target)] = std::make_pair(temp_weight,w);
	}else if (map_saved_edge.find(std::make_pair(nearest_to_target, nearest_to_source))!=map_saved_edge.end()){
	  if (temp_weight < map_saved_edge[std::make_pair(nearest_to_target, nearest_to_source)].first)
	    map_saved_edge[std::make_pair(nearest_to_target, nearest_to_source)] = std::make_pair(temp_weight,w);
	}else{
	  map_saved_edge[std::make_pair(nearest_to_source, nearest_to_target)] = std::make_pair(temp_weight,w);
	}
      }
    }

    std::vector<Edge> terminal_edge;
    for (auto it = map_saved_edge.begin(); it!=map_saved_edge.end(); it++){
      std::pair<Edge, bool> p = add_edge(it->first.first, it->first.second,
					 WeightProperty(it->second.first, Base(it->second.second)),terminal_graph);
      //      terminal_edge.push_back(p.first);
    }
    // minimal spanning tree ...
    boost::kruskal_minimum_spanning_tree(terminal_graph,
					 std::back_inserter(terminal_edge));
  

    // computing result
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
    boost::sort(tree_edges);
    auto unique_edges = boost::unique(tree_edges);

    steiner_graph_selected_terminal_indices.clear(); 
    for (auto e : unique_edges){ 
      steiner_graph_selected_terminal_indices.insert(index[source(e,*graph)]); 
      steiner_graph_selected_terminal_indices.insert(index[target(e,*graph)]); 
    } 
  }
}

WCPPID::MCUGraph* WCPPID::PR3DCluster::Create_steiner_tree(WCP::ToyPointCloud *point_cloud_steiner, std::vector<bool>& flag_steiner_terminal, WCP::GeomDataSource& gds, WCP::SMGCSelection& old_mcells, bool flag_path, bool disable_dead_mix_cell){
  Create_graph();

  // find all the steiner terminal indices ...
  find_steiner_terminals(gds, disable_dead_mix_cell);
  
  std::cout << "Test1: " << steiner_terminal_indices.size() << " steiner terminals found." << std::endl;
  
  // form point cloud 
  WCP::ToyPointCloud temp_pcloud;
  if (flag_path && path_wcps.size()>0){
   
    WCP::Point prev_p((*(path_wcps.begin())).x, (*(path_wcps.begin())).y, (*(path_wcps.begin())).z);
    std::tuple<int,int,int> prev_wire_index = std::make_tuple((*(path_wcps.begin())).index_u, (*(path_wcps.begin())).index_v, (*(path_wcps.begin())).index_w);
      
    for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
      if (temp_pcloud.get_num_points()==0){
        WCP::Point p((*it).x, (*it).y, (*it).z);
        std::tuple<int, int, int> wire_index = std::make_tuple((*it).index_u, (*it).index_v, (*it).index_w);
        temp_pcloud.AddPoint(p, wire_index ,0);
            }else{
        WCP::Point p((*it).x, (*it).y, (*it).z);
        std::tuple<int, int, int> wire_index = std::make_tuple((*it).index_u, (*it).index_v, (*it).index_w);

        float step_dis = 0.6*units::cm;
        float dis = sqrt(pow(p.x - prev_p.x,2) + pow(p.y - prev_p.y,2) + pow(p.z - prev_p.z,2));
      
      if (dis <= step_dis){
        temp_pcloud.AddPoint(p, wire_index, 0);
        prev_p = p;
        prev_wire_index = wire_index;
      }else{
        int num_steps = dis/step_dis;
        for (int qx = 0; qx!=num_steps;qx ++){
          WCP::Point temp_p(prev_p.x + (p.x-prev_p.x)/num_steps*(qx+1),
            prev_p.y + (p.y-prev_p.y)/num_steps*(qx+1),
            prev_p.z + (p.z-prev_p.z)/num_steps*(qx+1)
            );
          std::tuple<int, int, int> temp_wire_index = std::make_tuple(int(std::get<0>(prev_wire_index) + (std::get<0>(wire_index) - std::get<0>(prev_wire_index))*(qx+1.0)/num_steps),
                            int(std::get<1>(prev_wire_index) + (std::get<1>(wire_index) - std::get<1>(prev_wire_index))*(qx+1.0)/num_steps),
                      int(std::get<2>(prev_wire_index) + (std::get<2>(wire_index) - std::get<2>(prev_wire_index))*(qx+1.0)/num_steps)
                      );
          temp_pcloud.AddPoint(temp_p, temp_wire_index, 0);
        }
        temp_pcloud.AddPoint(p, wire_index, 0);
        prev_p = p;
        prev_wire_index = wire_index;
      }
	
      }
    }
    temp_pcloud.build_kdtree_index();  
  }


  // organize mcells
  std::map<int,SMGCSelection> old_time_mcells_map;
  for (auto it = old_mcells.begin(); it!=old_mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    int time_slice = mcell->GetTimeSlice();
    if (old_time_mcells_map.find(time_slice)==old_time_mcells_map.end()){
      SMGCSelection mcells;
      mcells.push_back(mcell);
      old_time_mcells_map[time_slice] = mcells;
    }else{
      old_time_mcells_map[time_slice].push_back(mcell);
    }
  }
  
  WCP::WCPointCloud<double>& cloud = point_cloud->get_cloud();

  std::set<int> indices_to_be_removal;
  
  for (auto it = steiner_terminal_indices.begin(); it!=steiner_terminal_indices.end(); it++){
    // examine the steiner terminals according to the mcells, inside known mcells
    int time_slice = cloud.pts[*it].mcell->GetTimeSlice();
    bool flag_remove = true;


    // std::cout << "Test: " << *it << " " << old_time_mcells_map.size() << std::endl;

    //    if (cloud.pts[*it].y/units::cm >-100 && cloud.pts[*it].y/units::cm<-50) 
    //std::cout << cloud.pts[*it].x/units::cm << " " <<
    //	cloud.pts[*it].y/units::cm << " " <<
    //	cloud.pts[*it].z/units::cm << " " << std::endl;
    
    if (old_time_mcells_map.find(time_slice)!=old_time_mcells_map.end()){
      for (auto it1 = old_time_mcells_map[time_slice].begin(); it1!= old_time_mcells_map[time_slice].end(); it1++){
        SlimMergeGeomCell *mcell = *it1;
        int u1_low_index = mcell->get_uwires().front()->index();
        int u1_high_index = mcell->get_uwires().back()->index();
        
        int v1_low_index = mcell->get_vwires().front()->index();
        int v1_high_index = mcell->get_vwires().back()->index();
        
        int w1_low_index = mcell->get_wwires().front()->index();
        int w1_high_index = mcell->get_wwires().back()->index();

          /* if (cloud.pts[*it].y/units::cm <-100) */
          /*   std::cout << cloud.pts[*it].index_u << " " << u1_low_index << " " << u1_high_index << ", " */
          /* 	      << cloud.pts[*it].index_v << " " << v1_low_index << " " << v1_high_index << ", " */
          /* 	      << cloud.pts[*it].index_w << " " << w1_low_index << " " << w1_high_index << ", " << std::endl; */
        
        if (cloud.pts[*it].index_u <= u1_high_index +1&&
            cloud.pts[*it].index_u >= u1_low_index -1&&
            cloud.pts[*it].index_v <= v1_high_index +1&&
            cloud.pts[*it].index_v >= v1_low_index -1&&
            cloud.pts[*it].index_w <= w1_high_index +1&&
            cloud.pts[*it].index_w >= w1_low_index -1){
          flag_remove = false;
          break;
        }
      }
    }
    
    if (flag_remove){
      //+1 time slice
      if (old_time_mcells_map.find(time_slice+1)!=old_time_mcells_map.end()){
        for (auto it1 = old_time_mcells_map[time_slice+1].begin(); it1!= old_time_mcells_map[time_slice+1].end(); it1++){
          SlimMergeGeomCell *mcell = *it1;
          int u1_low_index = mcell->get_uwires().front()->index();
          int u1_high_index = mcell->get_uwires().back()->index();
          
          int v1_low_index = mcell->get_vwires().front()->index();
          int v1_high_index = mcell->get_vwires().back()->index();
          
          int w1_low_index = mcell->get_wwires().front()->index();
          int w1_high_index = mcell->get_wwires().back()->index();
          if (cloud.pts[*it].index_u <= u1_high_index +1&&
              cloud.pts[*it].index_u >= u1_low_index -1&&
              cloud.pts[*it].index_v <= v1_high_index +1&&
              cloud.pts[*it].index_v >= v1_low_index -1&&
              cloud.pts[*it].index_w <= w1_high_index +1&&
              cloud.pts[*it].index_w >= w1_low_index -1){
            flag_remove = false;
            break;
          }
        }
      }
    }

    if (flag_remove){
      //-1
      if (old_time_mcells_map.find(time_slice-1)!=old_time_mcells_map.end()){
        for (auto it1 = old_time_mcells_map[time_slice-1].begin(); it1!= old_time_mcells_map[time_slice-1].end(); it1++){
          SlimMergeGeomCell *mcell = *it1;
          int u1_low_index = mcell->get_uwires().front()->index();
          int u1_high_index = mcell->get_uwires().back()->index();
          
          int v1_low_index = mcell->get_vwires().front()->index();
          int v1_high_index = mcell->get_vwires().back()->index();
          
          int w1_low_index = mcell->get_wwires().front()->index();
          int w1_high_index = mcell->get_wwires().back()->index();
          if (cloud.pts[*it].index_u <= u1_high_index +1&&
              cloud.pts[*it].index_u >= u1_low_index -1&&
              cloud.pts[*it].index_v <= v1_high_index +1&&
              cloud.pts[*it].index_v >= v1_low_index -1&&
              cloud.pts[*it].index_w <= w1_high_index +1&&
              cloud.pts[*it].index_w >= w1_low_index -1){
            flag_remove = false;
            break;
          }
        }
      }
    }
    
    
    // within a mcell check path ...
    if ((!flag_remove) && flag_path){
      // examine the steiner terminals according to the path,
      
      // if within cerntain distance in projection, but far away in 3D, remove ...
      WCP::Point p(cloud.pts[*it].x, cloud.pts[*it].y, cloud.pts[*it].z);
      double dis_3d = temp_pcloud.get_closest_dis(p);
      double dis_2du = (temp_pcloud.get_closest_2d_dis(p,0)).second;
      double dis_2dv = (temp_pcloud.get_closest_2d_dis(p,1)).second;
      double dis_2dw = (temp_pcloud.get_closest_2d_dis(p,2)).second;


      // std::cout << "Test1: " << p << " " << dis_3d << " " << dis_2du << " " << dis_2dv << " " << dis_2dw << std::endl;
      /* if(cluster_id == 2){ */
      //}
      
      if ((dis_2du < 1.8*units::cm && dis_2dv < 1.8*units::cm ||
	   dis_2du < 1.8*units::cm && dis_2dw < 1.8*units::cm ||
	   dis_2dv < 1.8*units::cm && dis_2dw < 1.8*units::cm ) && 
	   dis_3d > 6 * units::cm)
	    flag_remove = true;
    }

    /* if (flag_remove) */
    /*   std::cout << cloud.pts[*it].x/units::cm << " " <<cloud.pts[*it].y/units::cm << " " << cloud.pts[*it].z/units::cm<< " " << flag_remove << " " << old_time_mcells_map[time_slice].size() << std::endl; */
    
    
    if (flag_remove){
      indices_to_be_removal.insert(*it);
    }
  }

  // std::cout << indices_to_be_removal.size() << std::endl;
  
  for (auto it = indices_to_be_removal.begin(); it!=indices_to_be_removal.end(); it++){
    steiner_terminal_indices.erase(*it);
  }


 std::cout << "Test3: " << steiner_terminal_indices.size() << " steiner terminals found." << std::endl;

   // figure out the extreme points ...
  std::vector<std::vector<WCPointCloud<double>::WCPoint>> extreme_wcps = get_extreme_wcps(1, &old_time_mcells_map);
  for (size_t i=0;i!=extreme_wcps.size();i++){
    for (size_t j = 0; j!=extreme_wcps.at(i).size(); j++){
      if (steiner_terminal_indices.find(extreme_wcps.at(i).at(j).index)==steiner_terminal_indices.end()){
	        steiner_terminal_indices.insert(extreme_wcps.at(i).at(j).index);
	//terminals.push_back(extreme_wcps.at(i).at(j).index);
	/* std::cout << extreme_wcps.at(i).at(j).x/units::cm << " " << */
	/*   extreme_wcps.at(i).at(j).y/units::cm << " " << */
	/*   extreme_wcps.at(i).at(j).z/units::cm << " " << std::endl; */
      }
    }
  }

  std::cout << "Test4: " << steiner_terminal_indices.size() << " steiner terminals found." << std::endl;


  // form the tree ... 
  std::vector<int> terminals(steiner_terminal_indices.begin(), steiner_terminal_indices.end());

 
  
  const int N = point_cloud->get_num_points();
  /* std::vector<int> nonterminals; */
  /* for (int i=0;i!=N;i++){ */
  /*   if (steiner_terminal_indices.find(i)==steiner_terminal_indices.end()) */
  /*     nonterminals.push_back(i); */
  /* } */
  //std::cout << N << " " << terminals.size() + nonterminals.size() << std::endl;

  
  //Now try to manually write the Steiner Tree Greedy algorithm ...

  using Vertex = typename boost::graph_traits<WCPPID::MCUGraph>::vertex_descriptor;
  using Edge = typename boost::graph_traits<WCPPID::MCUGraph>::edge_descriptor;
  using Base = typename boost::property<edge_base_t, Edge>;
  // using EdgeWeightMap = typename boost::choose_const_pmap(boost::get_param(boost::no_named_parameters(), boost::edge_weight), *graph, boost::edge_weight);
  using EdgeWeightMap = typename boost::property_map<WCPPID::MCUGraph, boost::edge_weight_t>::type;
  using Weight = typename boost::property_traits<EdgeWeightMap>::value_type;
  using WeightProperty =
        typename boost::property<boost::edge_weight_t, Weight, Base>;
  using TerminalGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
    boost::no_property, WeightProperty>;
  using EdgeTerminal =
    typename boost::graph_traits<TerminalGraph>::edge_descriptor;

  EdgeWeightMap edge_weight = boost::choose_pmap(boost::get_param(boost::no_named_parameters(), boost::edge_weight), *graph, boost::edge_weight);
  
  // distance array used in the dijkstra runs
  std::vector<Weight> distance(N);

  std::vector<Vertex> nearest_terminal(num_vertices(*graph));
  auto index = get(boost::vertex_index, *graph);
  auto nearest_terminal_map = boost::make_iterator_property_map(nearest_terminal.begin(), get(boost::vertex_index, *graph));
  for (auto terminal : terminals) {
    nearest_terminal_map[terminal] = terminal;
  }
  // compute voronoi diagram each vertex get nearest terminal and last edge on
  // path to nearest terminal
  auto distance_map = make_iterator_property_map(distance.begin(), index);
  std::vector<Edge> vpred(N);
  auto last_edge = boost::make_iterator_property_map(vpred.begin(), get(boost::vertex_index, *graph));
  boost::dijkstra_shortest_paths(*graph, terminals.begin(), terminals.end(), boost::dummy_property_map(),
  				 distance_map, edge_weight, index, paal::utils::less(),
  				 boost::closed_plus<Weight>(), std::numeric_limits<Weight>::max(), 0,
  				 boost::make_dijkstra_visitor(paal::detail::make_nearest_recorder(
  											    nearest_terminal_map, last_edge, boost::on_edge_relaxed{})));

  // computing distances between terminals
  // creating terminal_graph
  TerminalGraph terminal_graph(N);
  std::map<std::pair<int, int>, std::pair<Weight, Edge> > map_saved_edge;
    
  for (auto w : boost::as_array(edges(*graph))) {
    auto const &nearest_to_source = nearest_terminal_map[source(w, *graph)];
    auto const &nearest_to_target = nearest_terminal_map[target(w, *graph)];
    if (nearest_to_source != nearest_to_target) {
      Weight temp_weight = distance[source(w, *graph)] + distance[target(w, *graph)] + edge_weight[w];
      if (map_saved_edge.find(std::make_pair(nearest_to_source, nearest_to_target))!=map_saved_edge.end()){
	if (temp_weight < map_saved_edge[std::make_pair(nearest_to_source, nearest_to_target)].first)
	  map_saved_edge[std::make_pair(nearest_to_source, nearest_to_target)] = std::make_pair(temp_weight,w);
      }else if (map_saved_edge.find(std::make_pair(nearest_to_target, nearest_to_source))!=map_saved_edge.end()){
	if (temp_weight < map_saved_edge[std::make_pair(nearest_to_target, nearest_to_source)].first)
	  map_saved_edge[std::make_pair(nearest_to_target, nearest_to_source)] = std::make_pair(temp_weight,w);
      }else{
	map_saved_edge[std::make_pair(nearest_to_source, nearest_to_target)] = std::make_pair(temp_weight,w);
      }
    }
  }

  std::vector<Edge> terminal_edge;
  for (auto it = map_saved_edge.begin(); it!=map_saved_edge.end(); it++){
    std::pair<Edge, bool> p = add_edge(it->first.first, it->first.second,
				       WeightProperty(it->second.first, Base(it->second.second)),terminal_graph);
    terminal_edge.push_back(p.first);
  }
  // minimal spanning tree ...
  //  boost::kruskal_minimum_spanning_tree(terminal_graph,
  //				       std::back_inserter(terminal_edge));
  

  // computing result
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
  boost::sort(tree_edges);
  auto unique_edges = boost::unique(tree_edges);

  selected_terminal_indices.clear(); 
  std::map<int, float> map_index_charge;
  for (auto e : unique_edges){ 
    if (map_index_charge.find(index[source(e,*graph)]) == map_index_charge.end()){
      selected_terminal_indices.insert(index[source(e,*graph)]);
      WCPointCloud<double>::WCPoint& wcp = cloud.pts[index[source(e,*graph)]];
      std::pair<bool, double> temp_charge = calc_charge_wcp(wcp,gds, disable_dead_mix_cell);
      map_index_charge[index[source(e,*graph)]] = temp_charge.second;
      //std::cout << index[source(e,*graph)] << " " << temp_charge.second << std::endl;
    }
    if (map_index_charge.find(index[target(e,*graph)]) == map_index_charge.end()){
      selected_terminal_indices.insert(index[target(e,*graph)]);
      WCPointCloud<double>::WCPoint& wcp = cloud.pts[index[target(e,*graph)]];
      std::pair<bool, double> temp_charge = calc_charge_wcp(wcp,gds, disable_dead_mix_cell);
      map_index_charge[index[target(e,*graph)]] = temp_charge.second;
      //std::cout << index[target(e,*graph)] << " " << temp_charge.second << std::endl;
    }

  } 

  // STG try ...


  // fill the data for point_cloud_steiner, also the flag
  // terminal first, and then non-terminals ...
  
  std::map<int,int> map_old_new_indices;
  std::map<int,int> map_new_old_indices;
  
  for (auto it = selected_terminal_indices.begin(); it!=selected_terminal_indices.end(); it++){
    int index = *it;
    WCP::SlimMergeGeomCell *mcell = 0;
    
    int time_slice = cloud.pts[index].mcell->GetTimeSlice();
    if (old_time_mcells_map.find(time_slice)!=old_time_mcells_map.end()){
      for (auto it1 = old_time_mcells_map[time_slice].begin(); it1!= old_time_mcells_map[time_slice].end(); it1++){
	SlimMergeGeomCell *mcell1 = *it1;
	int u1_low_index = mcell1->get_uwires().front()->index();
	int u1_high_index = mcell1->get_uwires().back()->index();
	 
	int v1_low_index = mcell1->get_vwires().front()->index();
	int v1_high_index = mcell1->get_vwires().back()->index();
	 
	int w1_low_index = mcell1->get_wwires().front()->index();
	int w1_high_index = mcell1->get_wwires().back()->index();
	 if (cloud.pts[index].index_u <= u1_high_index &&
	     cloud.pts[index].index_u >= u1_low_index &&
	     cloud.pts[index].index_v <= v1_high_index &&
	     cloud.pts[index].index_v >= v1_low_index &&
	     cloud.pts[index].index_w <= w1_high_index &&
	     cloud.pts[index].index_w >= w1_low_index){
	   mcell = mcell1;
	   break;
	 }
      }
    }

    Point p(cloud.pts[index].x,cloud.pts[index].y,cloud.pts[index].z);
    /* if (p.z>30*units::cm) */
    /*   std::cout << p.x/units::cm << " " << p.y/units::cm << " " << p.z/units::cm << " " << mcell <<  std::endl; */
    
    std::tuple<int,int,int> temp_indices = std::make_tuple(cloud.pts[index].index_u, cloud.pts[index].index_v, cloud.pts[index].index_w);
    point_cloud_steiner->AddPoint(p,temp_indices, mcell);
    //std::cout << index << " " << cloud.pts[*it].index << std::endl;
    map_old_new_indices[index] = flag_steiner_terminal.size();
    map_new_old_indices[flag_steiner_terminal.size()] = index;
    
    if (steiner_terminal_indices.find(index)==steiner_terminal_indices.end()){
      flag_steiner_terminal.push_back(false);
    }else{
      flag_steiner_terminal.push_back(true);
    }
    
  }
  point_cloud_steiner->build_kdtree_index();
  //std::cout << point_cloud_steiner->get_num_points() << " " << flag_steiner_terminal.size() << std::endl;
  
  temp_map_new_old_indices = map_new_old_indices;

  // fill the graph ...
  MCUGraph* graph_steiner = new MCUGraph(flag_steiner_terminal.size());
  for (auto e : unique_edges){
    int index1 = map_old_new_indices[index[source(e,*graph)]];
    int index2 = map_old_new_indices[index[target(e,*graph)]];
    float dis = get(boost::edge_weight_t(), *graph, e);


    float Q0 = 10000; // constant term ...
    float Qs = map_index_charge[index[source(e,*graph)]];
    float Qt = map_index_charge[index[target(e,*graph)]];
    float factor1=0.8, factor2=0.4; 
    
    /* int nsteiner = 0; */
    /* if (steiner_terminal_indices.find(index[source(e,*graph)])!=steiner_terminal_indices.end()) */
    /*   nsteiner ++; */
    /* if (steiner_terminal_indices.find(index[target(e,*graph)])!=steiner_terminal_indices.end()) */
    /*   nsteiner ++; */
    /* if (nsteiner==1){ */
    /*   factor1 = 0.5; */
    /*   factor2 = 0.5; */
    /* }else if (nsteiner==2){ */
    /*   factor1 = 0.25; */
    /*   factor2 = 0.75; */
    /* } */
    
    
    
    float temp_dis = dis * (factor1 + factor2 * (0.5*Q0/(Qs+Q0) + 0.5*Q0/(Qt+Q0)));
    //    if (Qs>0 || Qt>0)
    //  std::cout << Qs << " " << Qt << std::endl;
    
    //std::cout << dis << std::endl;
    auto edge = add_edge(index1, index2, temp_dis , *graph_steiner);
  } 

//   {
//   // debug 
//   std::cout << "Test5: " <<  " Steiner Graph vertices: " << boost::num_vertices(*graph_steiner)
//               << ", edges: " << boost::num_edges(*graph_steiner) << std::endl;
//     WCP::WCPointCloud<double>& cloud = point_cloud_steiner->get_cloud();
//     std::map<SlimMergeGeomCell*, std::set<int> > map_mcell_all_indices;
        
//     for (size_t i=0;i!=cloud.pts.size();i++){
//       if (cloud.pts.at(i).mcell==0) continue;
      
//       if (map_mcell_all_indices.find(cloud.pts.at(i).mcell)==map_mcell_all_indices.end()){
// 		std::set<int> temp_vec;
// 		temp_vec.insert(i);
// 		map_mcell_all_indices[cloud.pts.at(i).mcell] = temp_vec;
//       }else{
// 		map_mcell_all_indices[cloud.pts.at(i).mcell].insert(i);
//       }
//     }

//     for (auto it = map_mcell_all_indices.begin(); it!=map_mcell_all_indices.end();  it++){ 
//       std::set<int>& temp_vec = it->second;
//       for (auto it1 = temp_vec.begin(); it1!=temp_vec.end(); it1++){
// 		int index1 = *it1;
// 		WCPointCloud<double>::WCPoint& wcp1 = cloud.pts[index1];
// 		bool flag_index1 = flag_steiner_terminal[index1];
// 			for (auto  it2 = it1; it2!=temp_vec.end();it2++){
// 				if (it2==it1) continue;
// 					int index2 = *it2;
// 					bool flag_index2 = flag_steiner_terminal[index2];
// 					WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[index2];
				
// 				if (flag_index1 && flag_index2){
// 					auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph_steiner);
// 				}else if (flag_index1 || flag_index2){
// 					auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph_steiner);
// 				}
// 			}
//       }
//     }    
//    std::cout << "Test5: " <<  " Steiner Graph vertices: " << boost::num_vertices(*graph_steiner)
//               << ", edges: " << boost::num_edges(*graph_steiner) << std::endl;
// // debug
//   }

  return graph_steiner;
  

  
  /* auto index = get(boost::vertex_index, *graph); */
  /* typedef boost::graph_traits<WCPPID::MCUGraph>::edge_descriptor Edge;  */
  /* std::set<Edge> steinerEdges;  */
  /* std::vector<int> color(terminals.size()+nonterminals.size()); */
  /* { */
  /*   auto c = &color[0]; */
  /*   for (size_t i=0;i!=terminals.size();i++){ */
  /*     put(c, terminals.at(i),paal::Terminals::TERMINAL); */
  /*   } */
  /*   for (size_t i=0;i!=nonterminals.size();i++){ */
  /*     put(c, nonterminals.at(i), paal::Terminals::NONTERMINAL); */
  /*   } */
  /* } */
  
  /* paal::steiner_tree_greedy(*graph, std::inserter(steinerEdges, steinerEdges.begin()), */
  /* 			    boost::vertex_color_map(boost::make_iterator_property_map(color.begin(),index))); */
  /* auto weight = get(boost::edge_weight, *graph); */
  /* auto sum = 0; */
  /* selected_terminal_indices.clear(); */
  /* for (auto e : steinerEdges){ */
  /*   //sum += get(weight,e); */
  /*   selected_terminal_indices.insert(index[source(e,*graph)]); */
  /*   selected_terminal_indices.insert(index[target(e,*graph)]); */
  /*   //std::cout << index[source(e,*graph)] << " " << index[target(e,*graph)] << std::endl; */
  /* } */


  
  //  std::cout << terminals.size() << " " << selected_terminal_indices.size() << std::endl;
  //  std::cout << "result " << sum/units::cm << std::endl;

  
  // too slow ...
  /* using GraphMT = paal::data_structures::graph_metric<WCPPID::MCUGraph, float, paal::data_structures::graph_type::sparse_tag>; */
  /* auto metric = GraphMT(*graph); */
  // solve it
  /* paal::ir::steiner_tree_iterative_rounding(metric, terminals, */
  /* 					    nonterminals, std::back_inserter(selected_nonterminals)); */
}

void WCPPID::PR3DCluster::find_steiner_terminals(WCP::GeomDataSource& gds, bool disable_dead_mix_cell){
  // reset ...
  steiner_terminal_indices.clear();
  
  // form all the maps ...
  form_cell_points_map();

  // double sum = 0,sum1=0;
  for (size_t i=0;i!=mcells.size();i++){
    SMGCSelection temp_mcells;
    temp_mcells.push_back(mcells.at(i));
    std::set<int> indices = find_peak_point_indices(temp_mcells, gds, disable_dead_mix_cell);
    steiner_terminal_indices.insert(indices.begin(), indices.end());
    /* sum += indices.size(); */
    /* sum1 += mcells.at(i)->get_sampling_points().size(); */
  }
  // std::cout << sum << " " << sum/sum1 << std::endl;
  //std::cout << steiner_terminal_indices.size() << std::endl;
}



std::set<int> WCPPID::PR3DCluster::find_peak_point_indices(SMGCSelection mcells, WCP::GeomDataSource& gds, bool disable_dead_mix_cell, int nlevel){
  std::set<int> all_indices;
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    std::set<int>& indices = cell_point_indices_map[mcell];
    all_indices.insert(indices.begin(), indices.end());
  }
  
  WCP::WCPointCloud<double>& cloud = point_cloud->get_cloud();

  // form another set with the actual points according to their charge
  std::map<int,double> map_index_charge;
  std::set<std::pair<double, int>, std::greater<std::pair<double, int> > > candidates_set;
  for (auto it = all_indices.begin(); it!=all_indices.end(); it++){
    //std::cout << all_indices.size() << std::endl;
    WCPointCloud<double>::WCPoint& wcp = cloud.pts[(*it)];
    std::pair<bool, double> temp_charge = calc_charge_wcp(wcp,gds, disable_dead_mix_cell);

    //    bool flag_good_point = wcp.mcell->IsPointGood(wcp.index_u, wcp.index_v, wcp.index_w,2);
    
    double charge = temp_charge.second;
    map_index_charge[(*it)] = charge;

    /* if ( cloud.pts[(*it)].y<-80*units::cm) */
    /*   std::cout << cloud.pts[(*it)].y/units::cm << " " << cloud.pts[(*it)].z/units::cm << " " << charge << " " << temp_charge.first << std::endl; */
    
    if ((charge > 4000 ) && temp_charge.first ){
    // if ((charge > 4000 || charge == 0 && (!disable_dead_mix_cell) ) && temp_charge.first){
      candidates_set.insert(std::make_pair(charge,*it));
    }
    //std::cout <<  << std::endl;
  }

  // std::cout << "Xin2: candidates_set size: " << candidates_set.size() << std::endl;


  //std::cout << candidates_set.size() << std::endl;
  std::set<int> peak_indices;
  std::set<int> non_peak_indices;

  typedef boost::property_map<MCUGraph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index,*graph);
  typedef boost::graph_traits<MCUGraph>::adjacency_iterator adjacency_iterator;

  

  
  for (auto it = candidates_set.begin(); it!=candidates_set.end(); it++){
    int current_index = it->second;
    double current_charge = it->first;

    //    if (fabs(cloud.pts[current_index].x/units::cm -126.175)<0.1) 
    // std::cout << cloud.pts[current_index].x/units::cm << " A " << cloud.pts[current_index].y/units::cm << " " << cloud.pts[current_index].z/units::cm << " " << current_charge << std::endl;  
    
    
    std::set<int> total_vertices_found;
    total_vertices_found.insert(current_index);
    {
      std::set<int> vertices_to_be_examined;
      vertices_to_be_examined.insert(current_index);
      for (int j=0;j!=nlevel;j++){
	std::set<int> vertices_saved_for_next;
	for (auto it = vertices_to_be_examined.begin(); it!=vertices_to_be_examined.end(); it++){
	  int temp_current_index = (*it);
	  std::pair<adjacency_iterator, adjacency_iterator> neighbors = boost::adjacent_vertices(vertex(temp_current_index,*graph),*graph);
	  for (; neighbors.first!=neighbors.second; ++neighbors.first){
	    if (total_vertices_found.find(index(*neighbors.first))==total_vertices_found.end()){
	      total_vertices_found.insert(index(*neighbors.first));
	      vertices_saved_for_next.insert(index(*neighbors.first));
	    }
	  }
	}
	vertices_to_be_examined = vertices_saved_for_next;
      }
      total_vertices_found.erase(current_index);
    }
    //std::cout << total_vertices_found.size() << std::endl;
    
    // find the vertices with the point
    // std::pair<adjacency_iterator, adjacency_iterator> neighbors = boost::adjacent_vertices(vertex(current_index,*graph),*graph);

    if (peak_indices.size()==0){
      // if the current's charge is the biggest, push into good list 
      peak_indices.insert(current_index);
      for (auto it = total_vertices_found.begin(); it!= total_vertices_found.end(); it++){
	//      for (; neighbors.first!=neighbors.second; ++neighbors.first){
	//std::cout << *neighbors.first << " " << *neighbors.second << std::endl;
	if (map_index_charge.find(*it)==map_index_charge.end()) continue;
	// if charge smaller, push into dead list
	if (current_charge > map_index_charge[*it])
	  non_peak_indices.insert(*it);
      }
    }else{
      if (peak_indices.find(current_index)!=peak_indices.end() ||
	  non_peak_indices.find(current_index)!=non_peak_indices.end())
	continue;
      bool flag_insert = true;
      // if charge bigger, push current into dead list
      // loop over the connected vertices (not in the dead or good list)
      //      for (; neighbors.first!=neighbors.second; ++neighbors.first){
      for (auto it = total_vertices_found.begin(); it!= total_vertices_found.end(); it++){
	if (map_index_charge.find(*it)==map_index_charge.end()) continue;
	if (current_charge > map_index_charge[*it]){
	  non_peak_indices.insert(*it);
	}else if (current_charge <  map_index_charge[*it]){
	  flag_insert = false;
	  break;
	}
      }


       /* if (cloud.pts[current_index].y/units::cm < -100) */
      /* std::cout << cloud.pts[current_index].x/units::cm << " " << cloud.pts[current_index].y/units::cm << " " << cloud.pts[current_index].z/units::cm << " " << current_charge << " " << flag_insert << std::endl; */
      
      if (flag_insert)
	peak_indices.insert(current_index);
    }
    
  }

  /*  std::cout << peak_indices.size() << " " << non_peak_indices.size() << " " << candidates_set.size() << " " << all_indices.size() << std::endl; */
  /* for (auto it = peak_indices.begin(); it!=peak_indices.end(); it++){ */
  /*   int current_index = *it; */
  /*   std::cout << "Peak: " << current_index << " " << map_index_charge[current_index] << std::endl; */
  /*   std::pair<adjacency_iterator, adjacency_iterator> neighbors = boost::adjacent_vertices(vertex(current_index,*graph),*graph); */
  /*   for (; neighbors.first!=neighbors.second; ++neighbors.first){ */
  /*     if (map_index_charge.find(*neighbors.first)==map_index_charge.end()) continue; */
  /*     std::cout << *neighbors.first << " " << map_index_charge[*neighbors.first] << std::endl; */
  /*   } */
  /* } */

 
  /* for (auto it = peak_indices.begin();  it!= peak_indices.end(); it++){  */
  /*   if (fabs(cloud.pts[(*it)].x/units::cm -126.175)<0.1)  */
  /*     // if (cloud.pts[(*it)].y/units::cm > -90 && cloud.pts[(*it)].y/units::cm < -70 && */
  /*     // 	cloud.pts[(*it)].z/units::cm > 490 && cloud.pts[(*it)].z/units::cm < 510) */
  /*     std::cout <<  cloud.pts[(*it)].x/units::cm << " " << cloud.pts[(*it)].y/units::cm << " " << cloud.pts[(*it)].z/units::cm  << " " << map_index_charge[*it] << std::endl; */
  /* } */
  
  
  // form a graph to find the independent component ... 

  if (peak_indices.size()>1){
    std::vector<int> vec_peak_indices(peak_indices.begin(), peak_indices.end());
    peak_indices.clear();
    
    const int N = vec_peak_indices.size();
    boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
      boost::no_property, boost::property<boost::edge_weight_t, double>>
      temp_graph(N);
    for (int j=0;j!=N;j++){
      for (int k=0;k!=N;k++){
  	int index1 = j;
  	int index2 = k;
  	if (boost::edge(vertex(vec_peak_indices.at(index1),*graph), vertex(vec_peak_indices.at(index2),*graph), *graph).second)
  	  add_edge(index1, index2, temp_graph);
      }
    }
    std::vector<int> component(num_vertices(temp_graph));
    const int num = connected_components(temp_graph,&component[0]);
    
    double min_dis[num];
    PointVector points(num);
    int min_index[num];
    int ncounts[num];
    for (int i=0;i!=num;i++){
      min_dis[i] = 1e9;
      points.at(i).x = 0;
      points.at(i).y = 0;
      points.at(i).z = 0;
      min_index[i] = -1;
      ncounts[i] = 0;
    }
    
    std::vector<int>::size_type i;
    for (i=0;i!=component.size(); ++i){
      ncounts[component[i]]++;
      points.at(component[i]).x += cloud.pts[vec_peak_indices.at(i)].x;
      points.at(component[i]).y += cloud.pts[vec_peak_indices.at(i)].y;
      points.at(component[i]).z += cloud.pts[vec_peak_indices.at(i)].z;
    }
    // for each independent component, find the average position, and the closest point, maybe looping is good enough ...
    for (int i=0;i!=num;i++){
      points.at(i).x /= ncounts[i];
      points.at(i).y /= ncounts[i];
      points.at(i).z /= ncounts[i];
    }

    for (i=0;i!=component.size(); ++i){
      double dis = pow( points.at(component[i]).x - cloud.pts[vec_peak_indices.at(i)].x,2) +
  	pow( points.at(component[i]).y - cloud.pts[vec_peak_indices.at(i)].y,2) +
  	pow( points.at(component[i]).z - cloud.pts[vec_peak_indices.at(i)].z,2) ;
      if (dis < min_dis[component[i]]){
  	min_dis[component[i]] = dis;
  	min_index[component[i]] = vec_peak_indices.at(i);
      }
    }

    for (int i=0;i!=num;i++){
      peak_indices.insert(min_index[i]);
    }
    
    //  std::cout << num << " " << peak_indices.size() << std::endl;
    /* for (auto it = peak_indices.begin();  it!= peak_indices.end(); it++){ */
    /*   auto it1 = it; */
    /*   it1++; */
    /*   if (it1!=peak_indices.end()){ */
    /* 	std::cout << *it << " " << *it1 << " " << boost::edge(vertex(*it,*graph), vertex(*it1,*graph), *graph).second << " " << map_index_charge[*it] << " " << map_index_charge[*it1] */
    /* 		  << " " << cloud.pts[(*it)].x/units::cm << " " << cloud.pts[(*it)].y/units::cm << " " << cloud.pts[(*it)].z/units::cm << " " */
    /* 		  << " " << cloud.pts[(*it1)].x/units::cm << " " << cloud.pts[(*it1)].y/units::cm << " " << cloud.pts[(*it1)].z/units::cm << std::endl; */
    /*   } */
    /* } */
  }
 
   /* for (auto it = peak_indices.begin();  it!= peak_indices.end(); it++){ */
   /*   if (cloud.pts[(*it)].y/units::cm < -100) */
   /*     std::cout <<  cloud.pts[(*it)].x/units::cm << " " << cloud.pts[(*it)].y/units::cm << " " << cloud.pts[(*it)].z/units::cm  << std::endl; */
   /* } */

  return peak_indices;
}

std::pair<bool,double> WCPPID::PR3DCluster::calc_charge_wcp(WCP::WCPointCloud<double>::WCPoint& wcp, WCP::GeomDataSource& gds, bool disable_dead_mix_cell, double charge_cut){
  double charge = 0;
  double ncharge = 0;
  SlimMergeGeomCell *mcell = wcp.mcell;
  
  const GeomWire *uwire = gds.by_planeindex(WirePlaneType_t(0),wcp.index_u);
  const GeomWire *vwire = gds.by_planeindex(WirePlaneType_t(1),wcp.index_v);
  const GeomWire *wwire = gds.by_planeindex(WirePlaneType_t(2),wcp.index_w);

  double charge_u = mcell->Get_Wire_Charge(uwire);
  double charge_v = mcell->Get_Wire_Charge(vwire);
  double charge_w = mcell->Get_Wire_Charge(wwire);
  
  bool flag_charge_u = false;
  bool flag_charge_v = false;
  bool flag_charge_w = false;

  if (charge_u>charge_cut) flag_charge_u = true;
  if (charge_v>charge_cut) flag_charge_v = true;
  if (charge_w>charge_cut) flag_charge_w = true;
  

  if (disable_dead_mix_cell){
    charge += charge_u*charge_u; ncharge ++;
    charge += charge_v*charge_v; ncharge ++;
    charge += charge_w*charge_w; ncharge ++;
    //std::cout << charge_u << " " << charge_v << " " << charge_w << std::endl;
    
    // deal with bad planes ... 
    std::vector<WirePlaneType_t> bad_planes = mcell->get_bad_planes();
    for (size_t i=0;i!=bad_planes.size();i++){
      if (bad_planes.at(i)==WirePlaneType_t(0)){
	flag_charge_u = true;
	charge -= charge_u*charge_u; ncharge--;
      }else if (bad_planes.at(i)==WirePlaneType_t(1)){
	flag_charge_v = true;
	charge -= charge_v*charge_v; ncharge--;
      }else if (bad_planes.at(i)==WirePlaneType_t(2)){
	flag_charge_w = true;
	charge -= charge_w*charge_w; ncharge--;
      }
    }

    //  std::cout << wcp.index_u << " " << wcp.index_v << " " << wcp.index_w << " " << charge_u << " " << charge_v << " " << charge_w << " " << flag_charge_u << " " << flag_charge_v << " " << flag_charge_w << " " << bad_planes.size() << std::endl;

    
  }else{
    if (charge_u==0) flag_charge_u = true;
    if (charge_v==0) flag_charge_v = true;
    if (charge_w==0) flag_charge_w = true;

    if (charge_u!=0){
      charge += charge_u*charge_u; ncharge ++;
    }
    if (charge_v!=0){
      charge += charge_v*charge_v; ncharge ++;
    }
    if (charge_w!=0){
      charge += charge_w*charge_w; ncharge ++;
    }

    
    /* return std::make_pair(flag_charge_u && flag_charge_v || */
    /* 			flag_charge_v && flag_charge_w || */
    /* 			flag_charge_u && flag_charge_w, charge); */
  }

 
  
  // get charge for each indices ...
  
  
  // require more than two planes are good 
  if (ncharge>1) {
    charge = sqrt(charge/ncharge);
  }else{
    charge = 0;
  }
  return std::make_pair(flag_charge_u && flag_charge_v && flag_charge_w, charge);
  
}


void WCPPID::PR3DCluster::form_cell_points_map(){
  cell_point_indices_map.clear();
  WCP::WCPointCloud<double>& cloud = point_cloud->get_cloud();
  
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    std::vector<int>& wcps = point_cloud->get_mcell_indices(mcell);
    if (cell_point_indices_map.find(mcell)==cell_point_indices_map.end()){
      std::set<int> point_indices;
      cell_point_indices_map[mcell] = point_indices;
    }
    
    for (auto it1 = wcps.begin(); it1!=wcps.end(); it1++){
      WCPointCloud<double>::WCPoint& wcp = cloud.pts[*it1]; 
      cell_point_indices_map[mcell].insert(wcp.index);
    }
  }

}
