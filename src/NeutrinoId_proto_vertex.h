void WCPPID::NeutrinoID::find_other_segments(WCPPID::PR3DCluster* temp_cluster,  double search_range, double scaling_2d){
  

  const int N = temp_cluster->get_point_cloud_steiner()->get_num_points();
  std::vector<bool> flag_tagged(N, false);
  int num_tagged = 0;
  WCP::WCPointCloud<double>& cloud = temp_cluster->get_point_cloud_steiner()->get_cloud();
  for (size_t i=0;i!=N;i++){
    Point p(cloud.pts[i].x, cloud.pts[i].y, cloud.pts[i].z);
    double min_dis_u = 1e9, min_dis_v = 1e9, min_dis_w = 1e9;
    for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
      WCPPID::ProtoSegment *sg1 = it->first;
      std::pair<double, WCP::Point> closest_dis_point = sg1->get_closest_point(p);
      std::tuple<double, double, double> closest_2d_dis = sg1->get_closest_2d_dis(p);
      if (closest_dis_point.first < search_range){
	flag_tagged[i] = true;
	num_tagged ++;
	break;
      }
      if (std::get<0>(closest_2d_dis) < min_dis_u) min_dis_u = std::get<0>(closest_2d_dis);
      if (std::get<1>(closest_2d_dis) < min_dis_v) min_dis_v = std::get<1>(closest_2d_dis);
      if (std::get<2>(closest_2d_dis) < min_dis_w) min_dis_w = std::get<2>(closest_2d_dis);
    }

    if (!flag_tagged[i]){
      if ((min_dis_u < scaling_2d * search_range || ct_point_cloud->get_closest_dead_chs(p, 0) ) &&
	  (min_dis_v < scaling_2d * search_range || ct_point_cloud->get_closest_dead_chs(p, 1) ) &&
	  (min_dis_w < scaling_2d * search_range || ct_point_cloud->get_closest_dead_chs(p, 2) ) )
	flag_tagged[i] = true;
	  // std::cout << min_dis_u/units::cm << " " << min_dis_v/units::cm << " " << min_dis_w/units::cm << std::endl;
    }
  }

  // Now figure out the Terminal Graph ...
  std::vector<int> terminals;
  std::map<int, int> map_oindex_tindex;
  std::vector<bool>& flag_steiner_terminal = temp_cluster->get_flag_steiner_terminal();
  for (size_t i = 0;i!=flag_steiner_terminal.size();i++){
    if (flag_steiner_terminal[i]){
      map_oindex_tindex[i] = terminals.size();
      terminals.push_back(i);
    }
  }

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
  
  WCPPID::MCUGraph* graph_steiner = temp_cluster->get_graph_steiner();
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
  
  // new Graph ...
  std::map<int, std::set<int> > map_connection;
  TerminalGraph terminal_graph_cluster(terminals.size());
  for (auto it = terminal_edge.begin(); it!=terminal_edge.end(); it++){
    if (flag_tagged[source(*it, terminal_graph)] == flag_tagged[target(*it, terminal_graph)]){
      add_edge(map_oindex_tindex[source(*it, terminal_graph)],map_oindex_tindex[target(*it, terminal_graph)], terminal_graph_cluster);
    }else{
      if (map_connection.find(source(*it, terminal_graph))==map_connection.end()){
	std::set<int> temp_results;
	temp_results.insert(target(*it,terminal_graph));
	map_connection[source(*it, terminal_graph)] = temp_results;
      }else{
	map_connection[source(*it, terminal_graph)].insert(target(*it,terminal_graph));
      }
      if (map_connection.find(target(*it, terminal_graph))==map_connection.end()){
	std::set<int> temp_results;
	temp_results.insert(source(*it,terminal_graph));
	map_connection[target(*it, terminal_graph)] = temp_results;
      }else{
	map_connection[target(*it, terminal_graph)].insert(source(*it,terminal_graph));
      }
    }
    //      std::cout << source(*it, terminal_graph) << " " << target(*it, terminal_graph) << std::endl;
  }
  //std::cout << map_connection.size() << std::endl;
  //    std::cout << map_saved_edge.size() << " " << terminal_edge.size() << std::endl;

  std::vector<int> component(num_vertices(terminal_graph_cluster));
  const int num = connected_components(terminal_graph_cluster,&component[0]);
  int ncounts[num];
  std::vector<std::vector<int> > sep_clusters(num);
  for (size_t i=0;i!=num;i++){
    ncounts[i] = 0;
    std::vector<int> temp;
    sep_clusters[i] = temp;
  }
  
  {
    std::vector<int>::size_type i;
    for (i=0;i!=component.size(); ++i){
      ncounts[component[i]]++;
      sep_clusters[component[i]].push_back(terminals.at(i));
    }
  }

  
  std::vector<int> saved_clusters;
  std::vector<std::pair<int, int> > saved_cluster_points(num);
  for (size_t i=0;i!=num;i++){
    // inside the original track or just one point ...
    if (flag_tagged[sep_clusters[i].front()] || ncounts[i] == 1) continue;
    int special_A = -1;
    for (size_t j=0;j!=ncounts[i];j++){
      if (map_connection.find(sep_clusters[i].at(j))!=map_connection.end()){
	special_A = sep_clusters[i].at(j);
	break;
      }
    }
    int special_B = -1;
    double min_dis = 0;
    
    int number_not_faked = 0;

    double max_dis_u = 0, max_dis_v = 0, max_dis_w = 0;
    for (size_t j=0;j!=ncounts[i];j++){
      double dis = sqrt(pow(cloud.pts[sep_clusters[i].at(j)].x - cloud.pts[special_A].x,2)
			+ pow(cloud.pts[sep_clusters[i].at(j)].y - cloud.pts[special_A].y,2)
			+ pow(cloud.pts[sep_clusters[i].at(j)].z - cloud.pts[special_A].z,2));
      if (dis > min_dis){
	min_dis = dis;
	special_B = sep_clusters[i].at(j); // furthest away from special_A
      }

      // also judge whether this track is fake ...
      WCP::Point p(cloud.pts[sep_clusters[i].at(j)].x, cloud.pts[sep_clusters[i].at(j)].y, cloud.pts[sep_clusters[i].at(j)].z);
      double min_dis_u = 1e9, min_dis_v = 1e9, min_dis_w = 1e9;

      for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
	WCPPID::ProtoSegment *sg1 = it->first;
	std::tuple<double, double, double> closest_2d_dis = sg1->get_closest_2d_dis(p);
	if (std::get<0>(closest_2d_dis) < min_dis_u) min_dis_u = std::get<0>(closest_2d_dis);
	if (std::get<1>(closest_2d_dis) < min_dis_v) min_dis_v = std::get<1>(closest_2d_dis);
	if (std::get<2>(closest_2d_dis) < min_dis_w) min_dis_w = std::get<2>(closest_2d_dis);
      }
      
      int flag_num = 0;
      if (min_dis_u > scaling_2d * search_range   && (!ct_point_cloud->get_closest_dead_chs(p, 0))) flag_num ++;
      if (min_dis_v > scaling_2d * search_range   && (!ct_point_cloud->get_closest_dead_chs(p, 1))) flag_num ++;
      if (min_dis_w > scaling_2d * search_range   && (!ct_point_cloud->get_closest_dead_chs(p, 2))) flag_num ++;

      //std::cout << flag_num << " " << min_dis_u/units::cm << " " << min_dis_v/units::cm << " " << min_dis_w/units::cm << std::endl;
      if (min_dis_u > max_dis_u && (!ct_point_cloud->get_closest_dead_chs(p, 0))) max_dis_u = min_dis_u;
      if (min_dis_v > max_dis_v && (!ct_point_cloud->get_closest_dead_chs(p, 1))) max_dis_v = min_dis_v;
      if (min_dis_w > max_dis_w && (!ct_point_cloud->get_closest_dead_chs(p, 2))) max_dis_w = min_dis_w;
      if (flag_num>=2) number_not_faked ++;
    }
    //std::cout << number_not_faked << " " << ncounts[i] <<std::endl;
    if (number_not_faked < 4 && (number_not_faked < 0.15 * ncounts[i] || number_not_faked ==1)) continue;
    //std::cout << max_dis_u/units::cm << " " << max_dis_v/units::cm << " " << max_dis_w/units::cm << std::endl;
    if (! (( max_dis_u/units::cm > 4 || max_dis_v/units::cm > 4 || max_dis_w/units::cm > 4 ) && max_dis_u + max_dis_v + max_dis_w > 7*units::cm || number_not_faked > 4 && number_not_faked >= 0.75*ncounts[i] ) ) continue;
    
    
    //std::cout << special_A << " " << special_B << " " << min_dis/units::cm << std::endl;
    //std::cout << i << " " << ncounts[i] << " " << flag_tagged[sep_clusters[i].front()] << " " << cloud.pts[sep_clusters[i].front()].x/units::cm << " " <<  cloud.pts[sep_clusters[i].front()].y/units::cm << " " <<  cloud.pts[sep_clusters[i].front()].z/units::cm << std::endl;
    
    saved_clusters.push_back(i);
    saved_cluster_points.at(i) = std::make_pair(special_A, special_B);
  }


  // save segments ...
  for (auto it = saved_clusters.begin(); it!= saved_clusters.end(); it++){
    temp_cluster->dijkstra_shortest_paths(cloud.pts[(saved_cluster_points.at(*it)).first],2); 
    temp_cluster->cal_shortest_path(cloud.pts[(saved_cluster_points.at(*it)).second],2);
    temp_cluster->collect_charge_trajectory(*ct_point_cloud);
    temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond, true);

    if (temp_cluster->get_fine_tracking_path().size() >1){

      // front
      std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, Point> check_results = check_end_point(temp_cluster->get_fine_tracking_path(), true);
      // end 
      std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, Point> check_results1 = check_end_point(temp_cluster->get_fine_tracking_path(), false);


      // front
      WCPPID::ProtoVertex *v1 = 0;
      if (std::get<0>(check_results) != 0){
	v1 = std::get<0>(check_results);
      }else if (std::get<1>(check_results) != 0){
	WCPPID::ProtoSegment *sg1 = std::get<1>(check_results);
	Point test_p = std::get<2>(check_results);
	WCP::WCPointCloud<double>::WCPoint break_wcp = sg1->get_closest_wcpt(test_p);
	WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
	for (auto it2 = map_segment_vertices[sg1].begin(); it2!=map_segment_vertices[sg1].end(); it2++){
	  if ((*it2)->get_wcpt().index == sg1->get_wcpt_vec().front().index) start_v = *it2;
	  if ((*it2)->get_wcpt().index == sg1->get_wcpt_vec().back().index) end_v = *it2;
	}
	if (start_v ==0 || end_v ==0) std::cout << "Error in finding vertices for a sgement" << std::endl;

	//	if (break_wcp.index==start_v->get_wcpt().index){
	if (sqrt(pow(start_v->get_wcpt().x - break_wcp.x,2)+pow(start_v->get_wcpt().y - break_wcp.y,2) + pow(start_v->get_wcpt().z - break_wcp.z,2)) < 0.9*units::cm){
	  v1 = start_v;
	}else if (sqrt(pow(end_v->get_wcpt().x - break_wcp.x,2)+pow(end_v->get_wcpt().y - break_wcp.y,2) + pow(end_v->get_wcpt().z - break_wcp.z,2)) < 0.9*units::cm){
	  //}else if (break_wcp.index == end_v->get_wcpt().index){
	  v1 = end_v;
	}else{
	  std::list<WCP::WCPointCloud<double>::WCPoint> wcps_list1;
	  std::list<WCP::WCPointCloud<double>::WCPoint> wcps_list2;
	  temp_cluster->proto_break_tracks(start_v->get_wcpt(), break_wcp, end_v->get_wcpt(), wcps_list1, wcps_list2, true);
	  
	  v1 = new WCPPID::ProtoVertex(break_wcp);
	  WCPPID::ProtoSegment *sg2 = new WCPPID::ProtoSegment(wcps_list1);
	  
	  WCPPID::ProtoSegment *sg3 = new WCPPID::ProtoSegment(wcps_list2);
	  
	  //	  std::cout << start_v->get_wcpt().index << " " << break_wcp.index << " " << end_v->get_wcpt().index << " " << wcps_list1.size() << " " << wcps_list2.size() << std::endl;
	  //std::cout << map_vertex_segments.size() << " " << map_segment_vertices.size() << std::endl;
	  
	  add_proto_connection(start_v, sg2, temp_cluster);
	  add_proto_connection(v1, sg2, temp_cluster);
	  add_proto_connection(v1, sg3, temp_cluster);
	  add_proto_connection(end_v, sg3, temp_cluster);
	  del_proto_segment(sg1);
	  
	  //std::cout << map_vertex_segments.size() << " " << map_segment_vertices.size() << std::endl;
	  
	  // temporary fit hack ...
	  temp_cluster->set_path_wcps(wcps_list1);
	  temp_cluster->collect_charge_trajectory(*ct_point_cloud);
	  temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond, false);
	  sg2->set_fit_vec(temp_cluster->get_fine_tracking_path(), temp_cluster->get_dQ(), temp_cluster->get_dx(), temp_cluster->get_pu(), temp_cluster->get_pv(), temp_cluster->get_pw(), temp_cluster->get_pt(), temp_cluster->get_reduced_chi2());
	  v1->set_fit(temp_cluster->get_fine_tracking_path().back(), temp_cluster->get_dQ().back(), temp_cluster->get_dx().back(), temp_cluster->get_pu().back(), temp_cluster->get_pv().back(), temp_cluster->get_pw().back(), temp_cluster->get_pt().back(), temp_cluster->get_reduced_chi2().back());
	  temp_cluster->set_path_wcps(wcps_list2);
	  temp_cluster->collect_charge_trajectory(*ct_point_cloud);
	  temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond, false);
	  sg3->set_fit_vec(temp_cluster->get_fine_tracking_path(), temp_cluster->get_dQ(), temp_cluster->get_dx(), temp_cluster->get_pu(), temp_cluster->get_pv(), temp_cluster->get_pw(), temp_cluster->get_pt(), temp_cluster->get_reduced_chi2());
	}
	
      }else{
	v1 = new WCPPID::ProtoVertex(cloud.pts[(saved_cluster_points.at(*it)).first]);
      }

      

      // end
      // std::cout << std::get<0>(check_results1) << " " << std::get<1>(check_results1) << " " << std::get<2>(check_results1) << std::endl;
      
      WCPPID::ProtoVertex *v2 = 0;
      if (std::get<0>(check_results1) != 0){
	v2 = std::get<0>(check_results1);
      }else if (std::get<1>(check_results1) != 0){
	WCPPID::ProtoSegment *sg1 = std::get<1>(check_results1);
	Point test_p = std::get<2>(check_results1);
	WCP::WCPointCloud<double>::WCPoint break_wcp = sg1->get_closest_wcpt(test_p);
	WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
	for (auto it2 = map_segment_vertices[sg1].begin(); it2!=map_segment_vertices[sg1].end(); it2++){
	  if ((*it2)->get_wcpt().index == sg1->get_wcpt_vec().front().index) start_v = *it2;
	  if ((*it2)->get_wcpt().index == sg1->get_wcpt_vec().back().index) end_v = *it2;
	}
	if (start_v ==0 || end_v ==0) std::cout << "Error in finding vertices for a sgement" << std::endl;

	//	if (start_v->get_wcpt().index == break_wcp.index){
	if (sqrt(pow(start_v->get_wcpt().x - break_wcp.x,2)+pow(start_v->get_wcpt().y - break_wcp.y,2) + pow(start_v->get_wcpt().z - break_wcp.z,2)) < 0.9*units::cm){
	  v2 = start_v;
	  //	}else if (end_v->get_wcpt().index == break_wcp.index){
	}else if (sqrt(pow(end_v->get_wcpt().x - break_wcp.x,2)+pow(end_v->get_wcpt().y - break_wcp.y,2) + pow(end_v->get_wcpt().z - break_wcp.z,2)) < 0.9*units::cm){
	  v2 = end_v;
	}else{
	  std::list<WCP::WCPointCloud<double>::WCPoint> wcps_list1;
	  std::list<WCP::WCPointCloud<double>::WCPoint> wcps_list2;
	  temp_cluster->proto_break_tracks(start_v->get_wcpt(), break_wcp, end_v->get_wcpt(), wcps_list1, wcps_list2,true);
	  
	  v2 = new WCPPID::ProtoVertex(break_wcp);
	  WCPPID::ProtoSegment *sg2 = new WCPPID::ProtoSegment(wcps_list1);
	  
	  WCPPID::ProtoSegment *sg3 = new WCPPID::ProtoSegment(wcps_list2);
	  
	  //std::cout << map_vertex_segments.size() << " " << map_segment_vertices.size() << std::endl;
	  
	  add_proto_connection(start_v, sg2, temp_cluster);
	  add_proto_connection(v2, sg2, temp_cluster);
	  add_proto_connection(v2, sg3, temp_cluster);
	  add_proto_connection(end_v, sg3, temp_cluster);
	  del_proto_segment(sg1);
	  
	  //std::cout << map_vertex_segments.size() << " " << map_segment_vertices.size() << std::endl;
	  
	  // temporary fit hack ...
	  temp_cluster->set_path_wcps(wcps_list1);
	  temp_cluster->collect_charge_trajectory(*ct_point_cloud);
	  temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond, false);
	  sg2->set_fit_vec(temp_cluster->get_fine_tracking_path(), temp_cluster->get_dQ(), temp_cluster->get_dx(), temp_cluster->get_pu(), temp_cluster->get_pv(), temp_cluster->get_pw(), temp_cluster->get_pt(), temp_cluster->get_reduced_chi2());
	  v2->set_fit(temp_cluster->get_fine_tracking_path().back(), temp_cluster->get_dQ().back(), temp_cluster->get_dx().back(), temp_cluster->get_pu().back(), temp_cluster->get_pv().back(), temp_cluster->get_pw().back(), temp_cluster->get_pt().back(), temp_cluster->get_reduced_chi2().back());
	  temp_cluster->set_path_wcps(wcps_list2);
	  temp_cluster->collect_charge_trajectory(*ct_point_cloud);
	  temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond, false);
	  sg3->set_fit_vec(temp_cluster->get_fine_tracking_path(), temp_cluster->get_dQ(), temp_cluster->get_dx(), temp_cluster->get_pu(), temp_cluster->get_pv(), temp_cluster->get_pw(), temp_cluster->get_pt(), temp_cluster->get_reduced_chi2());
	}
      }else{
	v2 = new WCPPID::ProtoVertex(cloud.pts[(saved_cluster_points.at(*it)).second]);
      }
      
      if (v1->get_wcpt().index == cloud.pts[(saved_cluster_points.at(*it)).first].index && v2->get_wcpt().index == cloud.pts[(saved_cluster_points.at(*it)).second].index){
      }else{
	temp_cluster->dijkstra_shortest_paths(v1->get_wcpt(),2); 
	temp_cluster->cal_shortest_path(v2->get_wcpt(),2);
	temp_cluster->collect_charge_trajectory(*ct_point_cloud);
	temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond, true);
      }
      
      WCPPID::ProtoSegment *sg1 = new WCPPID::ProtoSegment(temp_cluster->get_path_wcps());
      sg1->set_fit_vec(temp_cluster->get_fine_tracking_path(), temp_cluster->get_dQ(), temp_cluster->get_dx(), temp_cluster->get_pu(), temp_cluster->get_pv(), temp_cluster->get_pw(), temp_cluster->get_pt(), temp_cluster->get_reduced_chi2());
      v1->set_fit(temp_cluster->get_fine_tracking_path().front(), temp_cluster->get_dQ().front(), temp_cluster->get_dx().front(), temp_cluster->get_pu().front(), temp_cluster->get_pv().front(), temp_cluster->get_pw().front(), temp_cluster->get_pt().front(), temp_cluster->get_reduced_chi2().front());
      
      v2->set_fit(temp_cluster->get_fine_tracking_path().back(), temp_cluster->get_dQ().back(), temp_cluster->get_dx().back(), temp_cluster->get_pu().back(), temp_cluster->get_pv().back(), temp_cluster->get_pw().back(), temp_cluster->get_pt().back(), temp_cluster->get_reduced_chi2().back());

      //      std::cout << map_vertex_segments.size() << " " << map_segment_vertices.size() << std::endl;
      add_proto_connection(v1,sg1,temp_cluster);
      //std::cout << map_vertex_segments.size() << " " << map_segment_vertices.size() << std::endl;
      add_proto_connection(v2,sg1,temp_cluster);
      
      // test ...
      std::cout << map_vertex_segments.size() << " " << map_segment_vertices.size() << std::endl;
            
    }
    
  }
  
  
}




std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, WCP::Point> WCPPID::NeutrinoID::check_end_point(WCP::PointVector& tracking_path, bool flag_front ){

  if (tracking_path.size()<2) std::cout << "vector size wrong!" << std::endl;
  
  Point test_p(0,0,0);
  Point dir_p(0,0,0);
  int ncount =0;
  if (flag_front){
    test_p = tracking_path.front();
    for (size_t k = 1; k<tracking_path.size(); k++){
      dir_p.x += test_p.x - tracking_path.at(k).x;
      dir_p.y += test_p.x - tracking_path.at(k).y;
      dir_p.z += test_p.x - tracking_path.at(k).z;
      ncount ++;
      if (ncount==5) break;
    }
  }else{
    test_p = tracking_path.back();
    for (size_t k=tracking_path.size()-2; k>=0; k--){
      dir_p.x += test_p.x - tracking_path.at(k).x;
      dir_p.y += test_p.x - tracking_path.at(k).y;
      dir_p.z += test_p.x - tracking_path.at(k).z;
      ncount ++;
      if (ncount==5) break;
    }
  }
  TVector3 temp_dir(dir_p.x, dir_p.y, dir_p.z);
  temp_dir = temp_dir.Unit();
  Line temp_l(test_p, temp_dir);


  
  WCPPID::ProtoVertex *vtx=0;
  WCPPID::ProtoSegment *seg=0;
  Point rtp = test_p;
  
  // check vertex
  for (auto it1 = map_vertex_segments.begin(); it1!= map_vertex_segments.end(); it1++){
    WCPPID::ProtoVertex *test_v = it1->first;
    Point p1(test_v->get_wcpt().x, test_v->get_wcpt().y, test_v->get_wcpt().z);
    Point p2 = test_v->get_fit_pt();

    double dis1 = sqrt(pow(test_p.x - p1.x,2)
		       + pow(test_p.y - p1.y,2)
		       + pow(test_p.z - p1.z,2));
    double dis2 = temp_l.closest_dis(p1);
    double dis3 = sqrt(pow(test_p.x - p2.x,2)
		       + pow(test_p.y - p2.y,2)
		       + pow(test_p.z - p2.z,2));
    double dis4 = temp_l.closest_dis(p2);

    // std::cout << dis1/units::cm << " " << dis2/units::cm << " " <<
    //   dis3/units::cm << " " << dis4/units::cm 
    //  	      << std::endl;
       
    if (std::min(dis1,dis2) < 0.9*units::cm && std::max(dis1,dis2) < 2*units::cm || std::min(dis3,dis4)<0.9*units::cm && std::max(dis3,dis4) < 2*units::cm){
      vtx = test_v;
      break;
    }
    
    
  }
  // std::cout << std::endl;


  // check segment 
  if (vtx == 0){
    WCPPID::ProtoSegment *min_sg = 0;
    double min_dis = 1e9;
    double min_dis1 = 1e9;
    for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
      std::pair<double, WCP::Point> results = it->first->get_closest_point(test_p);
      if (results.first < min_dis){
  	min_sg = it->first;
  	min_dis = results.first;
      }
    }

    
    if (min_dis < 2*units::cm){
      PointVector& points = min_sg->get_point_vec();

      for (size_t i= 0; i!=points.size();i++){
  	double dis1 = temp_l.closest_dis(points.at(i) );
  	if (dis1 < min_dis1){
  	  min_dis1 = dis1;
  	  test_p = points.at(i);
  	}
      }
      // find the closest point
      if (min_dis1 > 1.2*units::cm) min_sg = 0;
    }else{
      min_sg = 0;
    }
    seg = min_sg;

    //    std::cout << min_sg << " " << min_dis/units::cm << " " << min_dis1/units::cm << std::endl;
    
    //    if (min_sg!=0) std::cout << min_sg->get_wcpt_vec().size() << std::endl;
    
  }

  
  // return ...
  return std::make_tuple(vtx, seg, test_p);
}




void WCPPID::NeutrinoID::break_segments(std::vector<WCPPID::ProtoSegment*>& remaining_segments, WCPPID::PR3DCluster* temp_cluster){
  
  while(remaining_segments.size()!=0){
    WCPPID::ProtoSegment* curr_sg = remaining_segments.back();
    remaining_segments.pop_back();
    WCPPID::ProtoVertex *start_v=0, *end_v=0;
    for (auto it = map_segment_vertices[curr_sg].begin(); it!=map_segment_vertices[curr_sg].end(); it++){
      if ((*it)->get_wcpt().index == curr_sg->get_wcpt_vec().front().index) start_v = *it;
      if ((*it)->get_wcpt().index == curr_sg->get_wcpt_vec().back().index) end_v = *it;
    }
    if (start_v==0 || end_v==0){
      std::cout << "Error in finding vertices for a segment" << std::endl; 
      //std::cout << "Vertex: " << start_v->get_wcpt().index << " " << end_v->get_wcpt().index << std::endl;
    }
    
    // initialize the start points
    WCP::WCPointCloud<double>::WCPoint break_wcp = start_v->get_wcpt();
    Point test_start_p = curr_sg->get_point_vec().front();
    
    while(sqrt(pow(start_v->get_wcpt().x - break_wcp.x,2) +
	       pow(start_v->get_wcpt().y - break_wcp.y,2) +
	       pow(start_v->get_wcpt().z - break_wcp.z,2)) <= 1*units::cm &&
	  sqrt(pow(end_v->get_wcpt().x - break_wcp.x,2) +
	       pow(end_v->get_wcpt().y - break_wcp.y,2) +
	       pow(end_v->get_wcpt().z - break_wcp.z,2)) > 1*units::cm){
      std::tuple<Point, TVector3, bool> kink_tuple = curr_sg->search_kink(test_start_p);
      if (std::get<1>(kink_tuple).Mag()!=0 ){
	// find the extreme point ... PR3DCluster function
	break_wcp = temp_cluster->proto_extend_point(std::get<0>(kink_tuple), std::get<1>(kink_tuple), std::get<2>(kink_tuple));
	//std::cout << "Break point: " << remaining_segments.size() << " " << std::get<0>(kink_tuple) << " " << std::get<1>(kink_tuple).Mag() << " " << std::get<2>(kink_tuple) << " " << break_wcp.x/units::cm << " " << break_wcp.y/units::cm << " " << break_wcp.z/units::cm << " " << break_wcp.index << std::endl;
        if (sqrt(pow(start_v->get_wcpt().x - break_wcp.x,2) +
	       pow(start_v->get_wcpt().y - break_wcp.y,2) +
	       pow(start_v->get_wcpt().z - break_wcp.z,2)) <= 1*units::cm &&
	    sqrt(pow(end_v->get_wcpt().x - break_wcp.x,2) +
		 pow(end_v->get_wcpt().y - break_wcp.y,2) +
		 pow(end_v->get_wcpt().z - break_wcp.z,2)) > 1*units::cm)
	  test_start_p = std::get<0>(kink_tuple);
      }else{
	break;
      }
    }

    if (sqrt(pow(start_v->get_wcpt().x - break_wcp.x,2) +
	     pow(start_v->get_wcpt().y - break_wcp.y,2) +
	     pow(start_v->get_wcpt().z - break_wcp.z,2)) > 1*units::cm){
      // find the new path ... PR3DCluster function
      // break into two tracks and adding a ProtoVertex in this function ...
      std::list<WCP::WCPointCloud<double>::WCPoint> wcps_list1;
      std::list<WCP::WCPointCloud<double>::WCPoint> wcps_list2;
      bool flag_break = temp_cluster->proto_break_tracks(start_v->get_wcpt(), break_wcp, end_v->get_wcpt(), wcps_list1, wcps_list2);
      
      std::cout << "Break tracks: " << flag_break << " " << wcps_list1.front().index << " " << wcps_list1.back().index << " " << wcps_list2.front().index << " " << wcps_list2.back().index << std::endl;
	  
      if (flag_break){
	WCPPID::ProtoVertex *v3 = new WCPPID::ProtoVertex(break_wcp);
	WCPPID::ProtoSegment *sg2 = new WCPPID::ProtoSegment(wcps_list1);
	WCPPID::ProtoSegment *sg3 = new WCPPID::ProtoSegment(wcps_list2);
	
	add_proto_connection(start_v, sg2, temp_cluster);
	add_proto_connection(v3, sg2, temp_cluster);
	add_proto_connection(v3, sg3, temp_cluster);
	add_proto_connection(end_v, sg3, temp_cluster);
	
	del_proto_segment(curr_sg);
	//delete curr_sg;
	
	// temporary fit hack ...
	temp_cluster->set_path_wcps(wcps_list1);
	temp_cluster->collect_charge_trajectory(*ct_point_cloud);
	temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond, false);
	sg2->set_fit_vec(temp_cluster->get_fine_tracking_path(), temp_cluster->get_dQ(), temp_cluster->get_dx(), temp_cluster->get_pu(), temp_cluster->get_pv(), temp_cluster->get_pw(), temp_cluster->get_pt(), temp_cluster->get_reduced_chi2());
	v3->set_fit(temp_cluster->get_fine_tracking_path().back(), temp_cluster->get_dQ().back(), temp_cluster->get_dx().back(), temp_cluster->get_pu().back(), temp_cluster->get_pv().back(), temp_cluster->get_pw().back(), temp_cluster->get_pt().back(), temp_cluster->get_reduced_chi2().back());
	temp_cluster->set_path_wcps(wcps_list2);
	temp_cluster->collect_charge_trajectory(*ct_point_cloud);
	temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond, false);
	sg3->set_fit_vec(temp_cluster->get_fine_tracking_path(), temp_cluster->get_dQ(), temp_cluster->get_dx(), temp_cluster->get_pu(), temp_cluster->get_pv(), temp_cluster->get_pw(), temp_cluster->get_pt(), temp_cluster->get_reduced_chi2());
	remaining_segments.push_back(sg3);
      }
    }
    
    //    std::cout << proto_vertices.size() << " " << map_vertex_segments.size() << " " << map_segment_vertices[sg2].size() << " " << map_cluster_vertices[main_cluster].size() << " " << map_vertex_cluster.size()  << std::endl;
    // std::cout << proto_segments.size() << " " << map_segment_vertices.size() << " " << map_vertex_segments[v3].size() << " " << map_cluster_segments[main_cluster].size() << " " << map_segment_cluster.size() << std::endl;
  }
}


bool WCPPID::NeutrinoID::add_proto_connection(WCPPID::ProtoVertex *pv, WCPPID::ProtoSegment *ps, WCPPID::PR3DCluster* cluster){

  if (pv->get_wcpt().index != ps->get_wcpt_vec().front().index && pv->get_wcpt().index != ps->get_wcpt_vec().back().index){
    std::cout << "Error! Vertex and Segment does not match " << pv->get_wcpt().index << " " << ps->get_wcpt_vec().front().index << " " << ps->get_wcpt_vec().back().index << std::endl;
    return false;
  }

  if (map_vertex_cluster.find(pv)==map_vertex_cluster.end())
    proto_vertices.push_back(pv);

  map_vertex_cluster[pv] = cluster;
  if (map_cluster_vertices.find(cluster)!=map_cluster_vertices.end()){
    map_cluster_vertices[cluster].insert(pv);
  }else{
    ProtoVertexSet temp_vertices;
    temp_vertices.insert(pv);
    map_cluster_vertices[cluster] = temp_vertices;
  }

  if (map_segment_cluster.find(ps)==map_segment_cluster.end())
    proto_segments.push_back(ps);
  
  map_segment_cluster[ps] = cluster;
  if (map_cluster_segments.find(cluster)!=map_cluster_segments.end()){
    map_cluster_segments[cluster].insert(ps);
  }else{
    ProtoSegmentSet temp_segments;
    temp_segments.insert(ps);
    map_cluster_segments[cluster] = temp_segments;
  }

  if (map_vertex_segments.find(pv)!=map_vertex_segments.end()){
    map_vertex_segments[pv].insert(ps);
  }else{
    ProtoSegmentSet temp_segments;
    temp_segments.insert(ps);
    map_vertex_segments[pv] = temp_segments;
  }

  if (map_segment_vertices.find(ps)!=map_segment_vertices.end()){
    map_segment_vertices[ps].insert(pv);
  }else{
    ProtoVertexSet temp_vertices;
    map_segment_vertices[ps] = temp_vertices;
    map_segment_vertices[ps].insert(pv);
  }

  return true;
}


bool WCPPID::NeutrinoID::del_proto_vertex(WCPPID::ProtoVertex *pv){
  if (map_vertex_segments.find(pv)!=map_vertex_segments.end()){
    // overall
    //    proto_vertices.erase(proto_vertices.find(pv));
    // map between vertex and segment
    for (auto it = map_vertex_segments[pv].begin(); it!= map_vertex_segments[pv].end(); it++){ // loop very semeng *it
      map_segment_vertices[*it].erase(map_segment_vertices[*it].find(pv));
    }
    map_vertex_segments.erase(pv);
    // map between cluster and vertices
    map_cluster_vertices[map_vertex_cluster[pv]].erase(map_cluster_vertices[map_vertex_cluster[pv]].find(pv));
    map_vertex_cluster.erase(pv);
    
    return true;
  }else{
    return false;
  }
}

bool WCPPID::NeutrinoID::del_proto_segment(WCPPID::ProtoSegment *ps){
  if (map_segment_vertices.find(ps)!=map_segment_vertices.end()){
    // overall
    //    proto_segments.erase(proto_segments.find(ps));
    //map between segment and vertex
    for (auto it = map_segment_vertices[ps].begin(); it!=map_segment_vertices[ps].end(); it++){
      map_vertex_segments[*it].erase(map_vertex_segments[*it].find(ps));
    }
    map_segment_vertices.erase(ps);
    //map between cluster and segment
    map_cluster_segments[map_segment_cluster[ps]].erase(map_cluster_segments[map_segment_cluster[ps]].find(ps));
    map_segment_cluster.erase(ps);
    return true;
  }else{
    return false;
  }
  
}