#include "WireCellPaal/graph_metrics.h"
#include "WireCellPaal/steiner_tree_greedy.h"

void WireCellPID::PR3DCluster::Create_steiner_tree(WireCell::GeomDataSource& gds){
  Create_graph();

  // find all the steiner terminal indices ...
  find_steiner_terminals(gds);

  std::vector<int> terminals(steiner_terminal_indices.begin(), steiner_terminal_indices.end());
  const int N = point_cloud->get_num_points();
  std::vector<int> nonterminals;
  for (int i=0;i!=N;i++){
    if (steiner_terminal_indices.find(i)==steiner_terminal_indices.end())
      nonterminals.push_back(i);
  }
  //std::cout << N << " " << terminals.size() + nonterminals.size() << std::endl;

  auto index = get(boost::vertex_index, *graph);
  typedef boost::graph_traits<WireCellPID::MCUGraph>::edge_descriptor Edge; 
  std::set<Edge> steinerEdges; 
  std::vector<int> color(terminals.size()+nonterminals.size());
  {
    auto c = &color[0];
    for (size_t i=0;i!=terminals.size();i++){
      put(c, terminals.at(i),paal::Terminals::TERMINAL);
    }
    for (size_t i=0;i!=nonterminals.size();i++){
      put(c, nonterminals.at(i), paal::Terminals::NONTERMINAL);
    }
  }
  paal::steiner_tree_greedy(*graph, std::inserter(steinerEdges, steinerEdges.begin()),
			    boost::vertex_color_map(boost::make_iterator_property_map(color.begin(),index)));
  auto weight = get(boost::edge_weight, *graph);
  auto sum = 0;

  selected_terminal_indices.clear();
  for (auto e : steinerEdges){
    //sum += get(weight,e);
    selected_terminal_indices.insert(index[source(e,*graph)]);
    selected_terminal_indices.insert(index[target(e,*graph)]);
    //std::cout << index[source(e,*graph)] << " " << index[target(e,*graph)] << std::endl;
  }
  std::cout << terminals.size() << " " << selected_terminal_indices.size() << std::endl;
  //  std::cout << "result " << sum/units::cm << std::endl;
  
  
  /* using GraphMT = paal::data_structures::graph_metric<WireCellPID::MCUGraph, float, paal::data_structures::graph_type::sparse_tag>; */

  /* auto metric = GraphMT(*graph); */
  
  // solve it
  /* paal::ir::steiner_tree_iterative_rounding(metric, terminals, */
  /* 					    nonterminals, std::back_inserter(selected_nonterminals)); */
}

void WireCellPID::PR3DCluster::find_steiner_terminals(WireCell::GeomDataSource& gds){
  // reset ...
  steiner_terminal_indices.clear();
  
  // form all the maps ...
  form_cell_points_map();

  // double sum = 0,sum1=0;
  for (size_t i=0;i!=mcells.size();i++){
    SMGCSelection temp_mcells;
    temp_mcells.push_back(mcells.at(i));
    std::set<int> indices = find_peak_point_indices(temp_mcells, gds);
    steiner_terminal_indices.insert(indices.begin(), indices.end());
    /* sum += indices.size(); */
    /* sum1 += mcells.at(i)->get_sampling_points().size(); */
  }
  // std::cout << sum << " " << sum/sum1 << std::endl;
  //std::cout << steiner_terminal_indices.size() << std::endl;
}



std::set<int> WireCellPID::PR3DCluster::find_peak_point_indices(SMGCSelection mcells, WireCell::GeomDataSource& gds, int nlevel){
  std::set<int> all_indices;
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    std::set<int>& indices = cell_point_indices_map[mcell];
    all_indices.insert(indices.begin(), indices.end());
  }
  
  WireCell::WCPointCloud<double>& cloud = point_cloud->get_cloud();

  // form another set with the actual points according to their charge
  std::map<int,double> map_index_charge;
  std::set<std::pair<double, int>, std::greater<std::pair<double, int> > > candidates_set;
  for (auto it = all_indices.begin(); it!=all_indices.end(); it++){
    //std::cout << all_indices.size() << std::endl;
    WCPointCloud<double>::WCPoint& wcp = cloud.pts[(*it)];
    std::pair<bool, double> temp_charge = calc_charge_wcp(wcp,gds);
    double charge = temp_charge.second;
    map_index_charge[(*it)] = charge;
    if (charge > 4000 && temp_charge.first){
      candidates_set.insert(std::make_pair(charge,*it));
    }
    //std::cout <<  << std::endl;
  }

  //std::cout << candidates_set.size() << std::endl;
  std::set<int> peak_indices;
  std::set<int> non_peak_indices;

  typedef boost::property_map<MCUGraph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index,*graph);
  typedef boost::graph_traits<MCUGraph>::adjacency_iterator adjacency_iterator;
  
  for (auto it = candidates_set.begin(); it!=candidates_set.end(); it++){
    int current_index = it->second;
    double current_charge = it->first;

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

      if (flag_insert)
	peak_indices.insert(current_index);
    }
    
  }

   /* std::cout << peak_indices.size() << " " << non_peak_indices.size() << " " << candidates_set.size() << " " << all_indices.size() << std::endl; */
  /* for (auto it = peak_indices.begin(); it!=peak_indices.end(); it++){ */
  /*   int current_index = *it; */
  /*   std::cout << "Peak: " << current_index << " " << map_index_charge[current_index] << std::endl; */
  /*   std::pair<adjacency_iterator, adjacency_iterator> neighbors = boost::adjacent_vertices(vertex(current_index,*graph),*graph); */
  /*   for (; neighbors.first!=neighbors.second; ++neighbors.first){ */
  /*     if (map_index_charge.find(*neighbors.first)==map_index_charge.end()) continue; */
  /*     std::cout << *neighbors.first << " " << map_index_charge[*neighbors.first] << std::endl; */
  /*   } */
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
 


  return peak_indices;
}

std::pair<bool,double> WireCellPID::PR3DCluster::calc_charge_wcp(WireCell::WCPointCloud<double>::WCPoint& wcp, WireCell::GeomDataSource& gds, double charge_cut){
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
  if (ncharge>0) {
    charge = sqrt(charge/ncharge);
  }else{
    charge = 0;
  }

  // get charge for each indices ...
  // how to average ??? 
  return std::make_pair(flag_charge_u && flag_charge_v && flag_charge_w ,charge);
}


void WireCellPID::PR3DCluster::form_cell_points_map(){
  cell_point_indices_map.clear();
  WireCell::WCPointCloud<double>& cloud = point_cloud->get_cloud();
  
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