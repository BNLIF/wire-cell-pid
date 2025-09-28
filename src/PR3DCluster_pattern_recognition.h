
void WCPPID::PR3DCluster::search_other_tracks(WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, double flash_time, double search_range, double scaling_2d){
  if (fit_tracks.size()==0 && fine_tracking_path.size() == 0) return;
    
  if (fit_tracks.size()==0){
    WCP::TrackInfo *track = new WCP::TrackInfo(fine_tracking_path, dQ, dx, pu, pv, pw, pt, reduced_chi2);
    fit_tracks.push_back(track);
  }
  
  // number of terminals ... 
  const int N = point_cloud_steiner->get_num_points();
  std::vector<bool> flag_tagged(N, false);
  int num_tagged = 0;
  WCP::WCPointCloud<double>& cloud = point_cloud_steiner->get_cloud();
  for (size_t i=0;i!=N;i++){
    Point p(cloud.pts[i].x, cloud.pts[i].y, cloud.pts[i].z);
    double min_dis_u = 1e9, min_dis_v = 1e9, min_dis_w = 1e9;
    for (size_t j=0;j!=fit_tracks.size();j++){
      std::pair<double, WCP::Point> closest_dis_point = fit_tracks.at(j)->get_closest_point(p);
      std::tuple<double, double, double> closest_2d_dis = fit_tracks.at(j)->get_closest_2d_dis(p);
      if (closest_dis_point.first < search_range){
        flag_tagged[i] = true;
        num_tagged ++;
        break;
      }
      if (std::get<0>(closest_2d_dis) < min_dis_u) min_dis_u = std::get<0>(closest_2d_dis);
      if (std::get<1>(closest_2d_dis) < min_dis_v) min_dis_v = std::get<1>(closest_2d_dis);
      if (std::get<2>(closest_2d_dis) < min_dis_w) min_dis_w = std::get<2>(closest_2d_dis);

      //  std::cout << i << " " << p.x << " " << p.y << " " << p.z  << " " << closest_dis_point.first << " " << min_dis_u/units::cm << " " << min_dis_v/units::cm << " " << min_dis_w/units::cm << std::endl;
    }
    if (!flag_tagged[i]){
      if ((min_dis_u < scaling_2d * search_range || ct_point_cloud.get_closest_dead_chs(p, 0) ) &&
	  (min_dis_v < scaling_2d * search_range || ct_point_cloud.get_closest_dead_chs(p, 1) ) &&
	  (min_dis_w < scaling_2d * search_range || ct_point_cloud.get_closest_dead_chs(p, 2) ) )
	      flag_tagged[i] = true;
    }
   

  }

  //  std::cout << num_tagged << " " << N << std::endl;


  // Now figure out the TerminalGraph ...
  std::vector<int> terminals;
  std::map<int, int> map_oindex_tindex;
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
  
  //    std::cout << N << " " << terminals.size() << " " << map_saved_edge.size() << " " << terminal_edge.size() << std::endl;
  
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
      
      for (size_t k=0;k!=fit_tracks.size();k++){
	std::tuple<double, double, double> closest_2d_dis = fit_tracks.at(k)->get_closest_2d_dis(p);
	if (std::get<0>(closest_2d_dis) < min_dis_u) min_dis_u = std::get<0>(closest_2d_dis);
	if (std::get<1>(closest_2d_dis) < min_dis_v) min_dis_v = std::get<1>(closest_2d_dis);
	if (std::get<2>(closest_2d_dis) < min_dis_w) min_dis_w = std::get<2>(closest_2d_dis);
      }
      int flag_num = 0;
      if (min_dis_u > scaling_2d * search_range   && (!ct_point_cloud.get_closest_dead_chs(p, 0))) flag_num ++;
      if (min_dis_v > scaling_2d * search_range   && (!ct_point_cloud.get_closest_dead_chs(p, 1))) flag_num ++;
      if (min_dis_w > scaling_2d * search_range   && (!ct_point_cloud.get_closest_dead_chs(p, 2))) flag_num ++;

      //std::cout << flag_num << " " << min_dis_u/units::cm << " " << min_dis_v/units::cm << " " << min_dis_w/units::cm << std::endl;
      if (min_dis_u > max_dis_u && (!ct_point_cloud.get_closest_dead_chs(p, 0))) max_dis_u = min_dis_u;
      if (min_dis_v > max_dis_v && (!ct_point_cloud.get_closest_dead_chs(p, 1))) max_dis_v = min_dis_v;
      if (min_dis_w > max_dis_w && (!ct_point_cloud.get_closest_dead_chs(p, 2))) max_dis_w = min_dis_w;
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
  
  //    std::cout << num <<  " " << component.size() << " " << map_connection.size() << std::endl;
  // test
  /* for (size_t k = 0;k!=ncounts[2];k++){ */
  /*   int i = sep_clusters[2].at(k); */
  /*   Point p(cloud.pts[i].x, cloud.pts[i].y, cloud.pts[i].z); */
  /*   double min_dis_u = 1e9, min_dis_v = 1e9, min_dis_w = 1e9, min_dis = 1e9; */
  /*   for (size_t j=0;j!=fit_tracks.size();j++){ */
  /*     std::pair<double, WCP::Point> closest_dis_point = fit_tracks.at(j)->get_closest_point(p); */
  /*     std::tuple<double, double, double> closest_2d_dis = fit_tracks.at(j)->get_closest_2d_dis(p); */
  /*     if (closest_dis_point.first < min_dis){ */
  /* 	min_dis = closest_dis_point.first; */
  /*     } */
  /*     if (std::get<0>(closest_2d_dis) < min_dis_u) min_dis_u = std::get<0>(closest_2d_dis); */
  /*     if (std::get<1>(closest_2d_dis) < min_dis_v) min_dis_v = std::get<1>(closest_2d_dis); */
  /*     if (std::get<2>(closest_2d_dis) < min_dis_w) min_dis_w = std::get<2>(closest_2d_dis); */
  /*   } */
    
  /*   std::cout << i << " " << flag_tagged[i] << " " << min_dis/units::cm << " " << min_dis_u/units::cm << " " << min_dis_v/units::cm << " " << min_dis_w/units::cm << " " << ct_point_cloud.get_closest_dead_chs(p,0) << " " << */
  /*     ct_point_cloud.get_closest_dead_chs(p,1) << " " << ct_point_cloud.get_closest_dead_chs(p,2) << std::endl; */
  /* } */


  // fit the trajectory and dQ/dx ...
  for (auto it = saved_clusters.begin(); it!= saved_clusters.end(); it++){
    
    dijkstra_shortest_paths(cloud.pts[(saved_cluster_points.at(*it)).first],2); 
    cal_shortest_path(cloud.pts[(saved_cluster_points.at(*it)).second],2);
    collect_charge_trajectory(ct_point_cloud);
    
    // std::cout << " " << " " << global_wc_map.size() << " " << flash_time << std::endl;

    do_tracking(ct_point_cloud, global_wc_map, flash_time);
    if (dQ.size() >1){
      WCP::TrackInfo *track = new WCP::TrackInfo(fine_tracking_path, dQ, dx, pu, pv, pw, pt, reduced_chi2);
      fit_tracks.push_back(track);
      //std::cout << fit_tracks.size() << " " << track->get_track_length()/units::cm << " " << track->get_medium_dQ_dx()*units::cm << " " << track->get_track_length_threshold()/units::cm << std::endl;

      // hack print ... 
      //  {        
      //       // Print fitted points with dQ and dx
      //       PointVector& fitted_points = fine_tracking_path;
      //       // std::vector<double>& dQ = main_cluster->get_dQ();
      //       // std::vector<double>& dx = main_cluster->get_dx();
            
      //       for (size_t point_idx = 0; point_idx < fitted_points.size(); point_idx++) {
      //         double dq_val = (point_idx < dQ.size()) ? dQ.at(point_idx) : 0.0;
      //         double dx_val = (point_idx < dx.size()) ? dx.at(point_idx) : 0.0;
      //         std::cout << "Point " << point_idx << ": position=(" 
      //                   << fitted_points.at(point_idx).x/units::cm << ", " 
      //                   << fitted_points.at(point_idx).y/units::cm << ", " 
      //                   << fitted_points.at(point_idx).z/units::cm 
      //                   << "), dQ=" << dq_val << ", dx=" << dx_val/units::cm << std::endl;
      //       }
      //     }

    }
  }

  // recover the first track ...
  fine_tracking_path = fit_tracks.front()->get_tracking_path();
  dQ = fit_tracks.front()->get_dQ();
  dx = fit_tracks.front()->get_dx();
  pu = fit_tracks.front()->get_pu();
  pv = fit_tracks.front()->get_pv();
  pw = fit_tracks.front()->get_pw();
  pt = fit_tracks.front()->get_pt();
  reduced_chi2 = fit_tracks.front()->get_reduced_chi2();
  

}

