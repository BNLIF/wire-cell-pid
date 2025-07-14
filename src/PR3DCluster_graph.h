void WCPPID::PR3DCluster::Del_graph(){
  if (graph!=(WCPPID::MCUGraph*)0){
    delete graph;
    graph = 0;
    //    std::cout << "Del graph! " << cluster_id << std::endl;
  }
}


void WCPPID::PR3DCluster::remove_same_mcell_steiner_edges(int flag){

  if (flag==1){
    //std::cout << num_edges(*graph) << std::endl;
    for (auto it = same_mcell_steiner_edges.begin(); it!=same_mcell_steiner_edges.end(); it++){
      remove_edge(*it,*graph);
    }
    same_mcell_steiner_edges.clear();
    //  std::cout << num_edges(*graph) << std::endl;
  }else if (flag==2){
    //clean up added edges for the steiner tree graph ... 
    for (auto it = same_mcell_steiner_edges.begin(); it!=same_mcell_steiner_edges.end(); it++){
      remove_edge(*it,*graph_steiner);
    }
    same_mcell_steiner_edges.clear();
  }
}

void WCPPID::PR3DCluster::establish_same_mcell_steiner_edges(WCP::GeomDataSource& gds, bool disable_dead_mix_cell, int flag){

  if (flag==1){
    if (graph==(MCUGraph*)0)
      Create_graph();
    
    WCP::WCPointCloud<double>& cloud = point_cloud->get_cloud();
    std::map<SlimMergeGeomCell*, std::set<int> > map_mcell_all_indices;
    
    //std::map<SlimMergeGeomCell*, std::set<int> > map_mcell_steiner_indices;
    
    for (size_t i=0;i!=cloud.pts.size();i++){
      if (map_mcell_all_indices.find(cloud.pts.at(i).mcell)==map_mcell_all_indices.end()){
	std::set<int> temp_vec;
	temp_vec.insert(i);
	map_mcell_all_indices[cloud.pts.at(i).mcell] = temp_vec;
      }else{
	map_mcell_all_indices[cloud.pts.at(i).mcell].insert(i);
      }
    }
    
    
    find_steiner_terminals(gds, disable_dead_mix_cell);
    /* for (auto it = steiner_terminal_indices.begin(); it!=steiner_terminal_indices.end(); it++){ */
    /*   if (map_mcell_steiner_indices.find(cloud.pts[*it].mcell)==map_mcell_steiner_indices.end()){ */
    /*     std::set<int> temp_vec; */
    /*     temp_vec.insert(*it); */
    /*     map_mcell_steiner_indices[cloud.pts[*it].mcell] = temp_vec; */
    /*   }else{ */
    /*     map_mcell_steiner_indices[cloud.pts[*it].mcell].insert(*it); */
    /*   } */
    /* } */
    
    for (auto it = map_mcell_all_indices.begin(); it!=map_mcell_all_indices.end();  it++){ 
      std::set<int>& temp_vec = it->second;
      for (auto it1 = temp_vec.begin(); it1!=temp_vec.end(); it1++){
	int index1 = *it1;
	WCPointCloud<double>::WCPoint& wcp1 = cloud.pts[index1];
	bool flag_index1 = steiner_terminal_indices.find(index1)!=steiner_terminal_indices.end();
	for (auto  it2 = it1; it2!=temp_vec.end();it2++){
	  if (it2==it1) continue;
	  int index2 = *it2;
	  bool flag_index2 = steiner_terminal_indices.find(index2)!=steiner_terminal_indices.end();
	  WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[index2];
	  
	  if (flag_index1 && flag_index2){
	    auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))*0.8 ),*graph);
	    if (edge.second)
	      same_mcell_steiner_edges.push_back(edge.first);
	  }else if (flag_index1 || flag_index2){
	    auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))*0.9 ),*graph);
	    if (edge.second)
	      same_mcell_steiner_edges.push_back(edge.first);
	  }
	}
      }
      
    }
  }else if (flag==2){
    WCP::WCPointCloud<double>& cloud = point_cloud_steiner->get_cloud();
    std::map<SlimMergeGeomCell*, std::set<int> > map_mcell_all_indices;
        
    for (size_t i=0;i!=cloud.pts.size();i++){
      if (cloud.pts.at(i).mcell==0) continue;
      
      if (map_mcell_all_indices.find(cloud.pts.at(i).mcell)==map_mcell_all_indices.end()){
		std::set<int> temp_vec;
		temp_vec.insert(i);
		map_mcell_all_indices[cloud.pts.at(i).mcell] = temp_vec;
      }else{
		map_mcell_all_indices[cloud.pts.at(i).mcell].insert(i);
      }
    }

    for (auto it = map_mcell_all_indices.begin(); it!=map_mcell_all_indices.end();  it++){ 
      std::set<int>& temp_vec = it->second;
      for (auto it1 = temp_vec.begin(); it1!=temp_vec.end(); it1++){
		int index1 = *it1;
		WCPointCloud<double>::WCPoint& wcp1 = cloud.pts[index1];
		bool flag_index1 = flag_steiner_terminal[index1];
			for (auto  it2 = it1; it2!=temp_vec.end();it2++){
				if (it2==it1) continue;
					int index2 = *it2;
					bool flag_index2 = flag_steiner_terminal[index2];
					WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[index2];
				
				if (flag_index1 && flag_index2){
					auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph_steiner);
					if (edge.second)
						same_mcell_steiner_edges.push_back(edge.first);
				}else if (flag_index1 || flag_index2){
					auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph_steiner);
					if (edge.second)
						same_mcell_steiner_edges.push_back(edge.first);
				}
			}
      }
    }    
  }

//   std::cout << "Test: " << same_mcell_steiner_edges.size() << " edges established for same mcell steiner edges." << std::endl;
  
}

void WCPPID::PR3DCluster::Connect_graph(WCP::ToyCTPointCloud& ct_point_cloud, WCP::ToyPointCloud* ref_point_cloud){
  WCP::WCPointCloud<double>& cloud = point_cloud->get_cloud();
  WCP::WC2DPointCloud<double>& cloud_u = point_cloud->get_cloud_u();
  WCP::WC2DPointCloud<double>& cloud_v = point_cloud->get_cloud_v();
  WCP::WC2DPointCloud<double>& cloud_w = point_cloud->get_cloud_w();

  TVector3 drift_dir(1,0,0);
  
  // now form the connected components
  std::vector<int> component(num_vertices(*graph));
  const int num = connected_components(*graph,&component[0]);
  if (num > 1){
    std::vector<ToyPointCloud*> pt_clouds;
    for (int j=0;j!=num;j++){
      ToyPointCloud *pt_cloud = new ToyPointCloud();
      pt_clouds.push_back(pt_cloud);
    }
    
    std::vector<int>::size_type i;
    for (i=0;i!=component.size(); ++i){
      if(cloud.pts[i].mcell!=0){
	//if (cloud.pts[i].mcell->Estimate_total_charge() > 0)
	if (cloud.pts[i].mcell->IsPointGood(cloud.pts[i].index_u, cloud.pts[i].index_v, cloud.pts[i].index_w, 2)){
	  double temp_min_dis = 0;
	  if (ref_point_cloud!=0){
	    Point temp_p(cloud.pts[i].x,cloud.pts[i].y,cloud.pts[i].z);
	    temp_min_dis = ref_point_cloud->get_closest_dis(temp_p);
	  }
	  if (temp_min_dis < 1.0*units::cm)
	    pt_clouds.at(component[i])->AddPoint(cloud.pts[i],cloud_u.pts[i],cloud_v.pts[i],cloud_w.pts[i]);
	  else
	    excluded_points.insert(i);
	}else{
	  excluded_points.insert(i);
	}
      }else{
	excluded_points.insert(i);
      }
      
      //   std::cout << "Vertex " << i << " " << cloud.pts[i].x << " " << cloud.pts[i].y << " " << cloud.pts[i].z << " " << cloud.pts[i].index_u << " " << cloud.pts[i].index_v << " " << cloud.pts[i].index_w << " " << cloud.pts[i].mcell << " " << cloud.pts[i].mcell->GetTimeSlice()  << " is in component " << component[i] << std::endl;
    }
    for (int j=0;j!=num;j++){
      //      std::cout << j << " " << pt_clouds.at(j)->get_cloud().pts.size() << std::endl;
      pt_clouds.at(j)->build_kdtree_index();
    }
    
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis(num, std::vector< std::tuple<int,int,double> >(num));
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis_mst(num, std::vector< std::tuple<int,int,double> >(num));
    
    
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis_dir1(num, std::vector< std::tuple<int,int,double> >(num));
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis_dir2(num, std::vector< std::tuple<int,int,double> >(num));
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis_dir_mst(num, std::vector< std::tuple<int,int,double> >(num));
    
    for (int j=0;j!=num;j++){
      for (int k=0;k!=num;k++){
  	index_index_dis[j][k] = std::make_tuple(-1,-1,1e9);
  	index_index_dis_mst[j][k] = std::make_tuple(-1,-1,1e9);
	
  	index_index_dis_dir1[j][k] = std::make_tuple(-1,-1,1e9);
  	index_index_dis_dir2[j][k] = std::make_tuple(-1,-1,1e9);
	index_index_dis_dir_mst[j][k] = std::make_tuple(-1,-1,1e9);
      }
    }
    
    // check against the closest distance ...
    // no need to have MST ... 
    for (int j=0;j!=num;j++){
      for (int k=j+1;k!=num;k++){
	if (pt_clouds.at(j)->get_cloud().pts.size() == 0 || pt_clouds.at(k)->get_cloud().pts.size() ==0 ) continue;
	
  	index_index_dis[j][k] = pt_clouds.at(j)->get_closest_points(pt_clouds.at(k));

	// if (cluster_id==6)
	//   std::cout << j << " " << k << " " << num << " " << pt_clouds.at(j)->get_num_points() << " " << pt_clouds.at(k)->get_num_points() << std::endl;
	
  	if (num < 100 && pt_clouds.at(j)->get_num_points()>100 && pt_clouds.at(k)->get_num_points()>100 &&
  	    (pt_clouds.at(j)->get_num_points()+pt_clouds.at(k)->get_num_points()) > 400 ||
  	    pt_clouds.at(j)->get_num_points()>500 && pt_clouds.at(k)->get_num_points()>500){
  	  WCPointCloud<double>::WCPoint wp1 = cloud.pts.at(std::get<0>(index_index_dis[j][k]));
  	  WCPointCloud<double>::WCPoint wp2 = cloud.pts.at(std::get<1>(index_index_dis[j][k]));
  	  Point p1(wp1.x,wp1.y,wp1.z);
  	  Point p2(wp2.x,wp2.y,wp2.z);
	  
  	  TVector3 dir1 = VHoughTrans(p1, 30*units::cm, pt_clouds.at(j));
	  dir1 *= -1;
	  std::pair<int,double> result1 = pt_clouds.at(k)->get_closest_point_along_vec(p1, dir1, 80*units::cm, 5*units::cm, 7.5, 3*units::cm);

	  if (result1.first <0 && fabs(dir1.Angle(drift_dir)/3.1415926*180.-90)<10.){
	    if (fabs(dir1.Angle(drift_dir)/3.1415926*180.-90)<5.) dir1 = VHoughTrans(p1, 80*units::cm,pt_clouds.at(j));
	    else if (fabs(dir1.Angle(drift_dir)/3.1415926*180.-90)<10.) dir1 = VHoughTrans(p1, 50*units::cm,pt_clouds.at(j));
	    dir1 *= -1;
	    result1 = pt_clouds.at(k)->get_closest_point_along_vec(p1, dir1, 80*units::cm, 5*units::cm, 7.5, 3*units::cm);
	  }
	  if (result1.first >=0){
	    index_index_dis_dir1[j][k] = std::make_tuple(std::get<0>(index_index_dis[j][k]), result1.first, result1.second);
  	  }
	   
	     
  	 
	    
	  // search for iso connection ...
	  /* if (result1.first < 0 && fabs(dir1.Angle(drift_dir)/3.1415926*180.-90)<10. ){ */
	  /*   if (fabs(dir1.Angle(drift_dir)/3.1415926*180.-90)<5.) dir1 = VHoughTrans(p1, 80*units::cm,pt_clouds.at(j)); */
	  /*   else  dir1 = VHoughTrans(p1, 50*units::cm,pt_clouds.at(j)); */
	  /*   dir1 *= -1; */
	  /*   search_for_connection_isochronous(result1, p1, dir1, pt_clouds.at(j), pt_clouds.at(k), 80*units::cm, 10, 1.5*units::cm); */
	  /* } */
	  
	  // if (result1.first <0)
	  //   result1 = pt_clouds.at(k)->get_closest_point_along_vec(p1, dir1, 6*units::cm, 1*units::cm, 25, 1.5*units::cm);
	  	  // if (cluster_id==6)
	  //   std::cout << p1.x/units::cm << " " << p1.y/units::cm << " " << p1.z/units::cm << " " << dir1.X() << " " << dir1.Y() << " " << dir1.Z() << std::endl;

	  
  	  

	  TVector3 dir2 = VHoughTrans(p2, 30*units::cm, pt_clouds.at(k));
	  dir2 *= -1;
  	  std::pair<int,double> result2 = pt_clouds.at(j)->get_closest_point_along_vec(p2, dir2, 80*units::cm, 5*units::cm, 7.5, 3*units::cm);

	  if (result2.first <0 && fabs(dir2.Angle(drift_dir)/3.1415926*180.-90)<10.){
	    if (fabs(dir2.Angle(drift_dir)/3.1415926*180.-90)<5.) dir2 = VHoughTrans(p2, 80*units::cm,pt_clouds.at(k));
	    else if (fabs(dir2.Angle(drift_dir)/3.1415926*180.-90)<10.) dir2 = VHoughTrans(p2, 50*units::cm,pt_clouds.at(k));
	    dir2 *= -1;
	    result2 = pt_clouds.at(j)->get_closest_point_along_vec(p2, dir2, 80*units::cm, 5*units::cm, 7.5, 3*units::cm);
	  }
	  
  	  
	
	  
	  
  	  if (result2.first >=0){
	    index_index_dis_dir2[j][k] = std::make_tuple(result2.first, std::get<1>(index_index_dis[j][k]), result2.second);
  	  }
  	}

  		// Now check the path ... 
  	{
  	  WCPointCloud<double>::WCPoint wp1 = cloud.pts.at(std::get<0>(index_index_dis[j][k]));
  	  WCPointCloud<double>::WCPoint wp2 = cloud.pts.at(std::get<1>(index_index_dis[j][k]));
  	  Point p1(wp1.x,wp1.y,wp1.z);
  	  Point p2(wp2.x,wp2.y,wp2.z);
	  
  	  double dis = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
  	  double step_dis = 1.0*units::cm;
  	  int num_steps = dis/step_dis + 1;
  	  int num_bad = 0;
  	  for (int ii=0;ii!=num_steps;ii++){
  	    Point test_p;
  	    test_p.x = p1.x + (p2.x-p1.x)/num_steps*(ii+1);
  	    test_p.y = p1.y + (p2.y-p1.y)/num_steps*(ii+1);
  	    test_p.z = p1.z + (p2.z-p1.z)/num_steps*(ii+1);
  	    if (!ct_point_cloud.is_good_point(test_p)){
  	      num_bad ++;
  	      /* if (cluster->get_cluster_id()==11) */
  	      /* 	std::cout << test_p.x/units::cm << " " << test_p.y/units::cm << " " << test_p.z/units::cm << std::endl; */
  	    }
  	  }
	  
  	  //  std::cout << cluster->get_cluster_id() << " " << j << " " << k << " " << num_bad << " " << num_steps << std::endl;
	   
  	  if (num_bad > 7 ||
  	      num_bad > 2 && num_bad >=0.75*num_steps){
  	    index_index_dis[j][k] = std::make_tuple(-1,-1,1e9);
  	  }
	  
  	  // if (cluster_id==13)
	  //   std::cout << cluster_id << " " << p1.x/units::cm << " " << p1.y/units::cm << " " << p1.z/units::cm << " " << p2.x/units::cm << " " << p2.y/units::cm << " " << p2.z/units::cm << " " << j << " " << k << " " << num_bad << " " << num_steps << std::endl;
  	}
	
  	// Now check the path ... 
  	if (std::get<0>(index_index_dis_dir1[j][k])>=0){
  	  WCPointCloud<double>::WCPoint wp1 = cloud.pts.at(std::get<0>(index_index_dis_dir1[j][k]));
  	  WCPointCloud<double>::WCPoint wp2 = cloud.pts.at(std::get<1>(index_index_dis_dir1[j][k]));
  	  Point p1(wp1.x,wp1.y,wp1.z);
  	  Point p2(wp2.x,wp2.y,wp2.z);
	  
  	  double dis = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
  	  double step_dis = 1.0*units::cm;
  	  int num_steps = dis/step_dis + 1;
  	  int num_bad = 0;
  	  for (int ii=0;ii!=num_steps;ii++){
  	    Point test_p;
  	    test_p.x = p1.x + (p2.x-p1.x)/num_steps*(ii+1);
  	    test_p.y = p1.y + (p2.y-p1.y)/num_steps*(ii+1);
  	    test_p.z = p1.z + (p2.z-p1.z)/num_steps*(ii+1);
  	    if (!ct_point_cloud.is_good_point(test_p))
  	      num_bad ++;
  	  }
	  
	  
  	  if (num_bad > 7 ||
  	      num_bad > 2 && num_bad >=0.75*num_steps){
  	    index_index_dis_dir1[j][k] = std::make_tuple(-1,-1,1e9);
  	  }
	  // if (cluster_id==13)
	  //   std::cout << "A: " << p1.x/units::cm << " " << p1.y/units::cm << " " << p1.z/units::cm << " " << p2.x/units::cm << " " << p2.y/units::cm << " " << p2.z/units::cm << " " << j << " " << k << " " << num_bad << " " << num_steps << std::endl;
  	}
	
	
  	// Now check the path ... 
  	if (std::get<0>(index_index_dis_dir2[j][k])>=0){
  	  WCPointCloud<double>::WCPoint wp1 = cloud.pts.at(std::get<0>(index_index_dis_dir2[j][k]));
  	  WCPointCloud<double>::WCPoint wp2 = cloud.pts.at(std::get<1>(index_index_dis_dir2[j][k]));
  	  Point p1(wp1.x,wp1.y,wp1.z);
  	  Point p2(wp2.x,wp2.y,wp2.z);
	  
  	  double dis = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
  	  double step_dis = 1.0*units::cm;
  	  int num_steps = dis/step_dis + 1;
  	  int num_bad = 0;
  	  for (int ii=0;ii!=num_steps;ii++){
  	    Point test_p;
  	    test_p.x = p1.x + (p2.x-p1.x)/num_steps*(ii+1);
  	    test_p.y = p1.y + (p2.y-p1.y)/num_steps*(ii+1);
  	    test_p.z = p1.z + (p2.z-p1.z)/num_steps*(ii+1);
  	    if (!ct_point_cloud.is_good_point(test_p))
  	      num_bad ++;
  	  }
	 
	  
  	  if (num_bad > 7 ||
  	      num_bad > 2 && num_bad >=0.75*num_steps){
  	    index_index_dis_dir2[j][k] = std::make_tuple(-1,-1,1e9);
  	  }
  	  // if (cluster_id==13)
  	  //   std::cout << "B: " << p1.x/units::cm << " " << p1.y/units::cm << " " << p1.z/units::cm << " " << p2.x/units::cm << " " << p2.y/units::cm << " " << p2.z/units::cm << " " << j << " " << k << " " << num_bad << " " << num_steps << std::endl;
  	}
      }
    }

     // deal with MST of first type
    {
      const int N = num;
      boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
    	boost::no_property, boost::property<boost::edge_weight_t, double>>
    	temp_graph(N);
      
      for (int j=0;j!=num;j++){
    	for (int k=j+1;k!=num;k++){
	  int index1 = j;
	  int index2 = k;
    	  if (std::get<0>(index_index_dis[j][k])>=0)
    	    auto edge = add_edge(index1,index2, std::get<2>(index_index_dis[j][k]), temp_graph);
    	}
      }
      
      {
	std::vector<int> possible_root_vertex;
	std::vector<int> component(num_vertices(temp_graph));
	const int num1 = connected_components(temp_graph,&component[0]);
	possible_root_vertex.resize(num1);
	std::vector<int>::size_type i;
	for (i=0;i!=component.size(); ++i){
	  possible_root_vertex.at(component[i]) = i;
	}
	
	for (size_t i=0;i!=possible_root_vertex.size();i++){
	  std::vector<boost::graph_traits < MCUGraph >::vertex_descriptor> predecessors(num_vertices(temp_graph));
	  
	  prim_minimum_spanning_tree( temp_graph , &predecessors[0], boost::root_vertex(possible_root_vertex.at(i)));
      
	  for (size_t j=0;j!=predecessors.size();++j){
	    if (predecessors[j]!=j){
	      if (j < predecessors[j]){
		index_index_dis_mst[j][predecessors[j]] = index_index_dis[j][predecessors[j]];
	      }else{
		index_index_dis_mst[predecessors[j]][j] = index_index_dis[predecessors[j]][j];
	      }
	      //std::cout << j << " " << predecessors[j] << " " << std::endl;
	    }else{
	      //std::cout << j << " " << std::endl;
	    }
	  }
	}
      }
    }

    // MST of the direction ... 
    {
      const int N = num;
      boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
    	boost::no_property, boost::property<boost::edge_weight_t, double>>
    	temp_graph(N);
      
      for (int j=0;j!=num;j++){
    	for (int k=j+1;k!=num;k++){
	  int index1 = j;
	  int index2 = k;
    	  if (std::get<0>(index_index_dis_dir1[j][k])>=0 || std::get<0>(index_index_dis_dir2[j][k])>=0)
    	    auto edge = add_edge(index1,index2, std::min(std::get<2>(index_index_dis_dir1[j][k]), std::get<2>(index_index_dis_dir2[j][k])), temp_graph);
    	}
      }

      {
	std::vector<int> possible_root_vertex;
	std::vector<int> component(num_vertices(temp_graph));
	const int num1 = connected_components(temp_graph,&component[0]);
	possible_root_vertex.resize(num1);
	std::vector<int>::size_type i;
	for (i=0;i!=component.size(); ++i){
	  possible_root_vertex.at(component[i]) = i;
	}
	
	for (size_t i=0;i!=possible_root_vertex.size();i++){
	  std::vector<boost::graph_traits < MCUGraph >::vertex_descriptor> predecessors(num_vertices(temp_graph));
	  prim_minimum_spanning_tree( temp_graph , &predecessors[0], boost::root_vertex(possible_root_vertex.at(i)));
	  for (size_t j=0;j!=predecessors.size();++j){
	    if (predecessors[j]!=j){
	      if (j < predecessors[j]){
		index_index_dis_dir_mst[j][predecessors[j]] = index_index_dis[j][predecessors[j]];
	      }else{
		index_index_dis_dir_mst[predecessors[j]][j] = index_index_dis[predecessors[j]][j];
	      }
	      //std::cout << j << " " << predecessors[j] << " " << std::endl;
	    }else{
	      //std::cout << j << " " << std::endl;
	    }
	  }
	}
      }
    }

	
    for (int j=0;j!=num;j++){
      for (int k=j+1;k!=num;k++){
	if (std::get<2>(index_index_dis[j][k])<3*units::cm){
	  index_index_dis_mst[j][k] = index_index_dis[j][k];
	}
    
	
  	// establish the path ... 
  	if (std::get<0>(index_index_dis_mst[j][k])>=0){
	  if (std::get<2>(index_index_dis_mst[j][k])>5*units::cm){
	    auto edge = add_edge(std::get<0>(index_index_dis_mst[j][k]),std::get<1>(index_index_dis_mst[j][k]),std::get<2>(index_index_dis_mst[j][k]),*graph);
	  }else{
	    auto edge = add_edge(std::get<0>(index_index_dis_mst[j][k]),std::get<1>(index_index_dis_mst[j][k]),std::get<2>(index_index_dis_mst[j][k]),*graph);
	  }
	  //	  std::cout << "Xin1: " << std::get<0>(index_index_dis_mst[j][k]) << " " << std::get<1>(index_index_dis_mst[j][k]) << " " << std::get<2>(index_index_dis_mst[j][k]) << std::endl;
  	}

	// if (std::get<0>(index_index_dis[j][k])>=0){
  	//   auto edge = add_edge(std::get<0>(index_index_dis[j][k]),std::get<1>(index_index_dis[j][k]),*graph);
  	//   if (edge.second){
  	//     if (std::get<2>(index_index_dis[j][k])>5*units::cm){
  	//       (*graph)[edge.first].dist = std::get<2>(index_index_dis[j][k]);
  	//     }else{
  	//       (*graph)[edge.first].dist = std::get<2>(index_index_dis[j][k]);
  	//     }
  	//   }
  	// }
	if (std::get<0>(index_index_dis_dir_mst[j][k])>=0){
	  if (std::get<0>(index_index_dis_dir1[j][k])>=0){
	    if (std::get<2>(index_index_dis_dir1[j][k])>5*units::cm){
	      auto edge = add_edge(std::get<0>(index_index_dis_dir1[j][k]),std::get<1>(index_index_dis_dir1[j][k]),std::get<2>(index_index_dis_dir1[j][k])*1.1,*graph);
	    }else{
	      auto edge = add_edge(std::get<0>(index_index_dis_dir1[j][k]),std::get<1>(index_index_dis_dir1[j][k]),std::get<2>(index_index_dis_dir1[j][k]),*graph);
	    }
	    //	    std::cout << "Xin2: " << std::get<0>(index_index_dis_dir1[j][k]) << " " << std::get<1>(index_index_dis_dir1[j][k]) << " " << std::get<2>(index_index_dis_dir1[j][k]) << std::endl;
	  }
	  if (std::get<0>(index_index_dis_dir2[j][k])>=0){
	    if (std::get<2>(index_index_dis_dir2[j][k])>5*units::cm){
		auto edge = add_edge(std::get<0>(index_index_dis_dir2[j][k]),std::get<1>(index_index_dis_dir2[j][k]), std::get<2>(index_index_dis_dir2[j][k])*1.1,*graph);
	    }else{
		auto edge = add_edge(std::get<0>(index_index_dis_dir2[j][k]),std::get<1>(index_index_dis_dir2[j][k]),std::get<2>(index_index_dis_dir2[j][k]),*graph);
	    }
	    //	    std::cout << "Xin3: " << std::get<0>(index_index_dis_dir2[j][k]) << " " << std::get<1>(index_index_dis_dir2[j][k]) << " " << std::get<2>(index_index_dis_dir2[j][k]) << std::endl;
	  }
	}
	
      } // k
    } // j

    // delete newly created point cloud
    for (int i=0;i!=num;i++){
      delete pt_clouds.at(i);
    }
    pt_clouds.clear();
  }
}


void WCPPID::PR3DCluster::Create_graph(WCP::ToyCTPointCloud& ct_point_cloud, WCP::ToyPointCloud* ref_point_cloud){
  if (graph!=(MCUGraph*)0)
    return;

  if (point_cloud==(ToyPointCloud*)0)
    Create_point_cloud();

  
  
  const int N = point_cloud->get_num_points();
  graph = new MCUGraph(N);


  if (ref_point_cloud != 0)
     std::cout << "Graph Creation " << mcells.size() << " " << get_num_points() << " using reference filtering " << " " << ref_point_cloud->get_num_points() << std::endl;



  //  std::cout << "test1 " << std::endl; 
  Establish_close_connected_graph();
  Connect_graph(ct_point_cloud, ref_point_cloud);
  Connect_graph(ref_point_cloud);

  //std::cout <<"Create Graph! " << cluster_id  << " " << N << " " << num_edges(*graph) << std::endl;
}


void WCPPID::PR3DCluster::Create_graph(WCP::ToyPointCloud* ref_point_cloud){
  
  
  if (graph!=(WCPPID::MCUGraph*)0)
    return;
  
  if (point_cloud==(ToyPointCloud*)0)
    Create_point_cloud();
  
  // create Graph ...
  const int N = point_cloud->get_num_points();
  graph = new WCPPID::MCUGraph(N);

  
  //std::cout << "test " << std::endl; 
  Establish_close_connected_graph();
  
  Connect_graph(ref_point_cloud);
  //std::cout <<"Create Graph! " << cluster_id  << " " << N << std::endl;
  //Connect_graph(ref_point_cloud);
  //calc_num_components();
}



void WCPPID::PR3DCluster::Establish_close_connected_graph(){
 

  
  WCP::WCPointCloud<double>& cloud = point_cloud->get_cloud();

  //  bool flag_reduce_memory = true;
  const int max_num_edges = 12;
  const int max_num_nodes = 5;
  double ref_dir_y[max_num_edges];// = {0,-1./sqrt(2.),-1, -1./sqrt(2.),  0,  1./sqrt(2.), 1, 1./sqrt(2.)};
  double ref_dir_z[max_num_edges];// = {1, 1./sqrt(2.), 0, -1./sqrt(2.), -1, -1./sqrt(2.), 0, 1./sqrt(2.)};
  for (size_t i=0;i!=max_num_edges;i++){
    ref_dir_z[i] = cos(2*3.1415926/max_num_edges * i);
    ref_dir_y[i] = sin(2*3.1415926/max_num_edges * i);
  }

  
  
  std::map<SlimMergeGeomCell*, std::map<int, std::set<int>>> map_mcell_uindex_wcps;
  std::map<SlimMergeGeomCell*, std::map<int, std::set<int>>> map_mcell_vindex_wcps;
  std::map<SlimMergeGeomCell*, std::map<int, std::set<int>>> map_mcell_windex_wcps;

   for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    std::map<int, std::set<int>> map_uindex_wcps;
    std::map<int, std::set<int>> map_vindex_wcps;
    std::map<int, std::set<int>> map_windex_wcps;
    std::vector<int>& wcps = point_cloud->get_mcell_indices(mcell);
    for (auto it1 = wcps.begin(); it1!=wcps.end(); it1++){
      WCPointCloud<double>::WCPoint& wcp = cloud.pts[*it1]; 
      // int index = wcp.index;
      // std::cout << index << " " << wcp.x << " " << wcp.y << " " << wcp.z << " " << wcp.index_u << " " << wcp.index_v << " " << wcp.index_w << std::endl;
      
      auto v = vertex(wcp.index, *graph); // retrieve vertex descriptor
      (*graph)[v].index = wcp.index;
      
      if (map_uindex_wcps.find(wcp.index_u)==map_uindex_wcps.end()){
   	std::set<int> wcps;
   	wcps.insert(wcp.index);
   	map_uindex_wcps[wcp.index_u] = wcps;
      }else{
   	map_uindex_wcps[wcp.index_u].insert(wcp.index);
      }
      
      if (map_vindex_wcps.find(wcp.index_v)==map_vindex_wcps.end()){
  	std::set<int> wcps;
  	wcps.insert(wcp.index);
  	map_vindex_wcps[wcp.index_v] = wcps;
      }else{
  	map_vindex_wcps[wcp.index_v].insert(wcp.index);
      }

      if (map_windex_wcps.find(wcp.index_w)==map_windex_wcps.end()){
  	std::set<int> wcps;
  	wcps.insert(wcp.index);
  	map_windex_wcps[wcp.index_w] = wcps;
      }else{
  	map_windex_wcps[wcp.index_w].insert(wcp.index);
      }
      
      
    }
    map_mcell_uindex_wcps[mcell] = map_uindex_wcps;
    map_mcell_vindex_wcps[mcell] = map_vindex_wcps;
    map_mcell_windex_wcps[mcell] = map_windex_wcps;
  }

    int num_edges = 0;
  
  // create graph for points inside the same mcell
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    std::vector<int>& wcps = point_cloud->get_mcell_indices(mcell);
    int max_wire_interval = mcell->get_max_wire_interval();
    int min_wire_interval = mcell->get_min_wire_interval();
    std::map<int, std::set<int>>* map_max_index_wcps;
    std::map<int, std::set<int>>* map_min_index_wcps;
    if (mcell->get_max_wire_type()==WirePlaneType_t(0)){
      map_max_index_wcps = &map_mcell_uindex_wcps[mcell];
    }else if (mcell->get_max_wire_type()==WirePlaneType_t(1)){
      map_max_index_wcps = &map_mcell_vindex_wcps[mcell];
    }else{
      map_max_index_wcps = &map_mcell_windex_wcps[mcell];
    }
    if (mcell->get_min_wire_type()==WirePlaneType_t(0)){
      map_min_index_wcps = &map_mcell_uindex_wcps[mcell];
    }else if (mcell->get_min_wire_type()==WirePlaneType_t(1)){
      map_min_index_wcps = &map_mcell_vindex_wcps[mcell];
    }else{
      map_min_index_wcps = &map_mcell_windex_wcps[mcell];
    }
    
    for (auto it1 = wcps.begin(); it1!=wcps.end(); it1++){
      WCPointCloud<double>::WCPoint& wcp1 = cloud.pts[*it1];
      int index1 = wcp1.index;
      int index_max_wire;
      int index_min_wire;
      if (mcell->get_max_wire_type()==WirePlaneType_t(0)){
  	index_max_wire = wcp1.index_u;
      }else if (mcell->get_max_wire_type()==WirePlaneType_t(1)){
  	index_max_wire = wcp1.index_v;
      }else{
  	index_max_wire = wcp1.index_w;
      }
      if (mcell->get_min_wire_type()==WirePlaneType_t(0)){
  	index_min_wire = wcp1.index_u;
      }else if (mcell->get_min_wire_type()==WirePlaneType_t(1)){
  	index_min_wire = wcp1.index_v;
      }else{
  	index_min_wire = wcp1.index_w;
      }

      std::vector<std::set<int>*> max_wcps_set;
      std::vector<std::set<int>*> min_wcps_set;
      
      // go through the first map and find the ones satisfying the condition
      for (auto it2 = map_max_index_wcps->begin(); it2!=map_max_index_wcps->end(); it2++){
   	if (fabs(it2->first - index_max_wire)<=max_wire_interval){
   	  max_wcps_set.push_back(&(it2->second));
   	}
      }
      // go through the second map and find the ones satisfying the condition
      for (auto it2 = map_min_index_wcps->begin(); it2!=map_min_index_wcps->end(); it2++){
   	if (fabs(it2->first - index_min_wire)<=min_wire_interval){
   	  min_wcps_set.push_back(&(it2->second));
   	}
      }

      std::set<int> wcps_set1;
      std::set<int> wcps_set2;

      for (auto it2 = max_wcps_set.begin(); it2!=max_wcps_set.end(); it2++){
	wcps_set1.insert((*it2)->begin(), (*it2)->end());
      }
      for (auto it3 = min_wcps_set.begin(); it3!=min_wcps_set.end(); it3++){
	wcps_set2.insert((*it3)->begin(), (*it3)->end());
      }
      
      
      // std::cout << max_wcps_set.size() << " " << min_wcps_set.size() << std::endl;
      {
	std::set<int> common_set;
	set_intersection(wcps_set1.begin(), wcps_set1.end(), wcps_set2.begin(), wcps_set2.end(),std::inserter(common_set,common_set.begin()));

	//	std::cout << "S0: " << common_set.size() << std::endl;
	//	if (common_set.size() <= max_num_edges){
	for (auto it4 = common_set.begin(); it4!=common_set.end(); it4++){
	  WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[*it4];
	  if (wcp2.index != wcp1.index){
	    int index2 = wcp2.index;
	    // add edge ...
	    auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph);
	    
		// std::cout << mcell->GetTimeSlice()*4 << " " << (*mcell->get_uwires().begin())->index() << " " << (*mcell->get_vwires().begin())->index() << " " << (*mcell->get_wwires().begin())->index() << " " << index1 << " " << index2 << " " << edge.second << std::endl;
	    if (edge.second){
	      num_edges ++;
	    }
	  }
	}
	/* }else{ */
	/*   std::vector<int> temp_saved_indices_min; */
	/*   std::vector<int> temp_saved_dis_min; */
	/*   //std::vector<int> temp_saved_indices_max; */
	/*   //std::vector<int> temp_saved_dis_max; */
	/*   temp_saved_indices_min.resize(max_num_edges,-1); */
	/*   temp_saved_dis_min.resize(max_num_edges,1e9); */
	/*   //temp_saved_indices_max.resize(max_num_edges,-1); */
	/*   //temp_saved_dis_max.resize(max_num_edges,0); */
	/*   for (auto it4 = common_set.begin(); it4!=common_set.end(); it4++){ */
	/*     WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[*it4]; */
	/*     if (wcp2.index != wcp1.index){ */
	/*       for (int qx = 0;qx!=max_num_edges; qx++){ */
	/* 	if (sqrt(pow(wcp2.y-wcp1.y,2)+pow(wcp2.z-wcp1.z,2))==0) continue; */
	/* 	double dis = acos(((wcp2.y-wcp1.y) * ref_dir_y[qx] + (wcp2.z-wcp1.z)*ref_dir_z[qx])/sqrt(pow(wcp2.y-wcp1.y,2)+pow(wcp2.z-wcp1.z,2))); */
	/* 	if (dis < temp_saved_dis_min.at(qx)){ */
	/* 	  temp_saved_dis_min[qx] = dis; */
	/* 	  temp_saved_indices_min[qx] = (*it4); */
	/* 	} */
	/* 	/\* if (dis > 0 && dis > temp_saved_dis_max.at(qx)){ *\/ */
	/* 	/\*   temp_saved_dis_max[qx] = dis; *\/ */
	/* 	/\*   temp_saved_indices_max[qx] = (*it4); *\/ */
	/* 	/\* } *\/ */
	/*       } */
	/*     } */
	/*   } */
	/*   for (int qx = 0; qx!=max_num_edges;qx++){ */
	/*     if (temp_saved_indices_min[qx] >=0){ */
	/*       WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[temp_saved_indices_min[qx]]; */
	/*       int index2 = wcp2.index; */
	/*       // add edge ... */
	/*       auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph); */
	/*       if (edge.second){ */
	/* 	num_edges ++; */
	/*       } */
	/*     } */
	/*     /\* if (temp_saved_indices_max[qx] >=0){ *\/ */
	/*     /\*   WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[temp_saved_indices_max[qx]]; *\/ */
	/*     /\*   int index2 = wcp2.index; *\/ */
	/*     /\*   // add edge ... *\/ */
	/*     /\*   auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph); *\/ */
	/*     /\*   if (edge.second){ *\/ */
	/*     /\* 	num_edges ++; *\/ */
	/*     /\*   } *\/ */
	/*     /\* } *\/ */
	/*   } */
	/* } */

	
      }
    }
  }


  //  std::cout << "Xin: " << num_edges << " " << N << std::endl;
  
  
  
  std::vector<int> time_slices;
  for (auto it1 = time_cells_set_map.begin(); it1!=time_cells_set_map.end(); it1++){
    time_slices.push_back((*it1).first);
  }

  std::vector<std::pair<SlimMergeGeomCell*,SlimMergeGeomCell*>> connected_mcells;
  
  for (size_t i=0; i!= time_slices.size(); i++){
    SMGCSet& mcells_set = time_cells_set_map[time_slices.at(i)];
    
    // create graph for points in mcell inside the same time slice
    if (mcells_set.size()>=2){
      for (auto it2 = mcells_set.begin(); it2!=mcells_set.end();it2++){
  	SlimMergeGeomCell *mcell1 = *it2;
  	auto it2p = it2;
  	if (it2p!=mcells_set.end()){
  	  it2p++;
  	  for (auto it3 = it2p; it3!=mcells_set.end(); it3++){
  	    SlimMergeGeomCell *mcell2 = *(it3);
  	    //std::cout << mcell1 << " " << mcell2 << " " << mcell1->Overlap_fast(mcell2,2) << std::endl;
	    if (mcell1->Overlap_fast(mcell2,2))
	      connected_mcells.push_back(std::make_pair(mcell1,mcell2));
  	  }
  	}
      }
    }
    // create graph for points between connected mcells in adjacent time slices + 1, if not, + 2
    std::vector<SMGCSet> vec_mcells_set;
    if (i+1 < time_slices.size()){
      if (time_slices.at(i+1)-time_slices.at(i)==1){
	vec_mcells_set.push_back(time_cells_set_map[time_slices.at(i+1)]);
	if (i+2 < time_slices.size())
	  if (time_slices.at(i+2)-time_slices.at(i)==2){
	    vec_mcells_set.push_back(time_cells_set_map[time_slices.at(i+2)]);
	  }
      }else if (time_slices.at(i+1) - time_slices.at(i)==2){
	vec_mcells_set.push_back(time_cells_set_map[time_slices.at(i+1)]);
      }
    }
    //    bool flag = false;
    for (size_t j=0; j!=vec_mcells_set.size(); j++){
      //      if (flag) break;
      SMGCSet& next_mcells_set = vec_mcells_set.at(j);
      for (auto it1 = mcells_set.begin(); it1!= mcells_set.end(); it1++){
	SlimMergeGeomCell *mcell1 = (*it1);
	for (auto it2 = next_mcells_set.begin(); it2!=next_mcells_set.end(); it2++){
	  SlimMergeGeomCell *mcell2 = (*it2);
	  if (mcell1->Overlap_fast(mcell2,2)){
	    //	    std::cout << mcell1->get_sampling_points().size() << " " << mcell2->get_sampling_points().size() << std::endl;
	    // flag = true; // correct???
	    connected_mcells.push_back(std::make_pair(mcell1,mcell2));
	  }
	}
      }
    }
  }
//  std::cout << "Connected mcells across time slices: " << connected_mcells.size() << std::endl;


  // establish edge ... 
  std::map<std::pair<int,int>, std::set<std::pair<double,int> > > closest_index;

  // std::cout << connected_mcells.size() << std::endl;
  for (auto it = connected_mcells.begin(); it!= connected_mcells.end(); it++){
    SlimMergeGeomCell *mcell1 = (*it).first;
    SlimMergeGeomCell *mcell2 = (*it).second;

    std::vector<int>& wcps1 = point_cloud->get_mcell_indices(mcell1);
    std::vector<int>& wcps2 = point_cloud->get_mcell_indices(mcell2);

    // test 2 against 1 ... 
    int max_wire_interval = mcell1->get_max_wire_interval();
    int min_wire_interval = mcell1->get_min_wire_interval();
    std::map<int, std::set<int>>* map_max_index_wcps;
    std::map<int, std::set<int>>* map_min_index_wcps;
    
    if (mcell1->get_max_wire_type()==WirePlaneType_t(0)){
      map_max_index_wcps = &map_mcell_uindex_wcps[mcell2];
    }else if (mcell1->get_max_wire_type()==WirePlaneType_t(1)){
      map_max_index_wcps = &map_mcell_vindex_wcps[mcell2];
    }else{
      map_max_index_wcps = &map_mcell_windex_wcps[mcell2];
    }
    if (mcell1->get_min_wire_type()==WirePlaneType_t(0)){
      map_min_index_wcps = &map_mcell_uindex_wcps[mcell2];
    }else if (mcell1->get_min_wire_type()==WirePlaneType_t(1)){
      map_min_index_wcps = &map_mcell_vindex_wcps[mcell2];
    }else{
      map_min_index_wcps = &map_mcell_windex_wcps[mcell2];
    }

    for (auto it1 = wcps1.begin(); it1!=wcps1.end(); it1++){
      WCPointCloud<double>::WCPoint& wcp1 = cloud.pts[*it1];
      int index1 = wcp1.index;
      int index_max_wire;
      int index_min_wire;
      if (mcell1->get_max_wire_type()==WirePlaneType_t(0)){
  	index_max_wire = wcp1.index_u;
      }else if (mcell1->get_max_wire_type()==WirePlaneType_t(1)){
  	index_max_wire = wcp1.index_v;
      }else{
  	index_max_wire = wcp1.index_w;
      }
      if (mcell1->get_min_wire_type()==WirePlaneType_t(0)){
  	index_min_wire = wcp1.index_u;
      }else if (mcell1->get_min_wire_type()==WirePlaneType_t(1)){
  	index_min_wire = wcp1.index_v;
      }else{
  	index_min_wire = wcp1.index_w;
      }
      std::vector<std::set<int>*> max_wcps_set;
      std::vector<std::set<int>*> min_wcps_set;
      // go through the first map and find the ones satisfying the condition
      for (auto it2 = map_max_index_wcps->begin(); it2!=map_max_index_wcps->end(); it2++){
   	if (fabs(it2->first - index_max_wire)<=max_wire_interval){
   	  max_wcps_set.push_back(&(it2->second));
   	}
      }
      // go through the second map and find the ones satisfying the condition
      for (auto it2 = map_min_index_wcps->begin(); it2!=map_min_index_wcps->end(); it2++){
   	if (fabs(it2->first - index_min_wire)<=min_wire_interval){
   	  min_wcps_set.push_back(&(it2->second));
   	}
      }

      std::set<int> wcps_set1;
      std::set<int> wcps_set2;

      for (auto it2 = max_wcps_set.begin(); it2!=max_wcps_set.end(); it2++){
	wcps_set1.insert((*it2)->begin(), (*it2)->end());
      }
      for (auto it3 = min_wcps_set.begin(); it3!=min_wcps_set.end(); it3++){
	wcps_set2.insert((*it3)->begin(), (*it3)->end());
      }

      
      {
	std::set<int> common_set;
	set_intersection(wcps_set1.begin(), wcps_set1.end(), wcps_set2.begin(), wcps_set2.end(),std::inserter(common_set,common_set.begin()));

	//std::cout << "S1: " << common_set.size() << std::endl;
	//std::cout << common_set.size() << std::endl; std::map<int,std::pair<int,double> > closest_index;

	for (auto it4 = common_set.begin(); it4!=common_set.end(); it4++){
	  WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[*it4];
	  if (wcp2.index != wcp1.index){
	    int index2 = wcp2.index;
	    double dis = sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2));
	    
	    if (closest_index.find(std::make_pair(index1,wcp2.mcell->GetTimeSlice()))==closest_index.end()){
	      std::set<std::pair<double, int> > temp_sets;
	      temp_sets.insert(std::make_pair(dis,index2));
	      closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())] = temp_sets;//std::make_pair(index2,dis);
	    }else{
	      closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].insert(std::make_pair(dis,index2));
	      if (closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].size()>max_num_nodes){
		auto it5 = closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].begin();
		for (int qx = 0; qx!=max_num_nodes;qx++){
		  it5++;
		}
		closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].erase(it5,closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].end());
	      }
	      //if (dis < closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].second)
	      //closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())] = std::make_pair(index2,dis);
	    }
	  }
	}

	
	//	if (common_set.size() <= max_num_edges){
	  /* for (auto it4 = common_set.begin(); it4!=common_set.end(); it4++){ */
	  /*   WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[*it4]; */
	  /*   if (wcp2.index != wcp1.index){ */
	  /*     int index2 = wcp2.index; */
	  /*     auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph); */
	  /*     if (edge.second){ */
	  /* 	num_edges ++; */
	  /*     } */
	  /*   } */
	  /* } */
	/* }else{ */
	/*   std::vector<int> temp_saved_indices; */
	/*   std::vector<int> temp_saved_dis; */
	/*   temp_saved_indices.resize(max_num_edges,-1); */
	/*   temp_saved_dis.resize(max_num_edges,1e9); */
	/*   for (auto it4 = common_set.begin(); it4!=common_set.end(); it4++){ */
	/*     WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[*it4]; */
	/*     if (wcp2.index != wcp1.index){ */
	/*       for (int qx = 0;qx!=max_num_edges; qx++){ */
	/* 	double dis = (wcp2.y-wcp1.y) * ref_dir_y[qx] + (wcp2.z-wcp1.z)*ref_dir_z[qx]; */
	/* 	if (dis > 0 && dis < temp_saved_dis.at(qx)){ */
	/* 	  temp_saved_dis[qx] = dis; */
	/* 	  temp_saved_indices[qx] = (*it4); */
	/* 	} */
	/*       } */
	/*     } */
	/*   } */
	/*   for (int qx = 0; qx!=max_num_edges;qx++){ */
	/*     if (temp_saved_indices[qx] >=0){ */
	/*       WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[temp_saved_indices[qx]]; */
	/*       int index2 = wcp2.index; */
	/*       // add edge ... */
	/*       auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph); */
	/*       if (edge.second){ */
	/* 	num_edges ++; */
	/*       } */
	/*     } */
	/*   } */
	/* } */

	
      }
    }


    // test 1 against 2 ...
    max_wire_interval = mcell2->get_max_wire_interval();
    min_wire_interval = mcell2->get_min_wire_interval();
    if (mcell2->get_max_wire_type()==WirePlaneType_t(0)){
      map_max_index_wcps = &map_mcell_uindex_wcps[mcell1];
    }else if (mcell2->get_max_wire_type()==WirePlaneType_t(1)){
      map_max_index_wcps = &map_mcell_vindex_wcps[mcell1];
    }else{
      map_max_index_wcps = &map_mcell_windex_wcps[mcell1];
    }
    if (mcell2->get_min_wire_type()==WirePlaneType_t(0)){
      map_min_index_wcps = &map_mcell_uindex_wcps[mcell1];
    }else if (mcell2->get_min_wire_type()==WirePlaneType_t(1)){
      map_min_index_wcps = &map_mcell_vindex_wcps[mcell1];
    }else{
      map_min_index_wcps = &map_mcell_windex_wcps[mcell1];
    }
    for (auto it1 = wcps2.begin(); it1!=wcps2.end(); it1++){
      WCPointCloud<double>::WCPoint& wcp1 = cloud.pts[*it1];
      int index1 = wcp1.index;
      int index_max_wire;
      int index_min_wire;
      if (mcell2->get_max_wire_type()==WirePlaneType_t(0)){
  	index_max_wire = wcp1.index_u;
      }else if (mcell2->get_max_wire_type()==WirePlaneType_t(1)){
  	index_max_wire = wcp1.index_v;
      }else{
  	index_max_wire = wcp1.index_w;
      }
      if (mcell2->get_min_wire_type()==WirePlaneType_t(0)){
  	index_min_wire = wcp1.index_u;
      }else if (mcell2->get_min_wire_type()==WirePlaneType_t(1)){
  	index_min_wire = wcp1.index_v;
      }else{
  	index_min_wire = wcp1.index_w;
      }
      std::vector<std::set<int>*> max_wcps_set;
      std::vector<std::set<int>*> min_wcps_set;
      // go through the first map and find the ones satisfying the condition
      for (auto it2 = map_max_index_wcps->begin(); it2!=map_max_index_wcps->end(); it2++){
   	if (fabs(it2->first - index_max_wire)<=max_wire_interval){
   	  max_wcps_set.push_back(&(it2->second));
   	}
      }
      // go through the second map and find the ones satisfying the condition
      for (auto it2 = map_min_index_wcps->begin(); it2!=map_min_index_wcps->end(); it2++){
   	if (fabs(it2->first - index_min_wire)<=min_wire_interval){
   	  min_wcps_set.push_back(&(it2->second));
   	}
      }

      std::set<int> wcps_set1;
      std::set<int> wcps_set2;

      for (auto it2 = max_wcps_set.begin(); it2!=max_wcps_set.end(); it2++){
	wcps_set1.insert((*it2)->begin(), (*it2)->end());
      }
      for (auto it3 = min_wcps_set.begin(); it3!=min_wcps_set.end(); it3++){
	wcps_set2.insert((*it3)->begin(), (*it3)->end());
      }

      
      {
	std::set<int> common_set;
	set_intersection(wcps_set1.begin(), wcps_set1.end(), wcps_set2.begin(), wcps_set2.end(),std::inserter(common_set,common_set.begin()));

	//	std::cout << "S2: " << common_set.size() << std::endl;
	//	std::map<int,std::pair<int,double> > closest_index;
	for (auto it4 = common_set.begin(); it4!=common_set.end(); it4++){
	  WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[*it4];
	  if (wcp2.index != wcp1.index){
	    int index2 = wcp2.index;
	    double dis = sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2));
	    
	    if (closest_index.find(std::make_pair(index1,wcp2.mcell->GetTimeSlice()))==closest_index.end()){
	      std::set<std::pair<double, int> > temp_sets;
	      temp_sets.insert(std::make_pair(dis,index2));
	      closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())] = temp_sets;//std::make_pair(index2,dis);
	    }else{
	      closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].insert(std::make_pair(dis,index2));
	      if (closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].size()>max_num_nodes){
		auto it5 = closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].begin();
		for (int qx = 0; qx!=max_num_nodes;qx++){
		  it5++;
		}
		closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].erase(it5,closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].end());
	      //if (dis < closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())].second)
	      //	closest_index[std::make_pair(index1,wcp2.mcell->GetTimeSlice())] = std::make_pair(index2,dis);
	      }
	    }
	  }
	}

	//if (common_set.size() <= max_num_edges){
	  /* for (auto it4 = common_set.begin(); it4!=common_set.end(); it4++){ */
	  /*   WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[*it4]; */
	  /*   if (wcp2.index != wcp1.index){ */
	  /*     int index2 = wcp2.index; */
	  /*     auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph); */
	  /*     if (edge.second){ */
	  /* 	num_edges ++; */
	  /*     } */
	  /*   } */
	  /* } */
	/* }else{ */
	/*   std::vector<int> temp_saved_indices; */
	/*   std::vector<int> temp_saved_dis; */
	/*   temp_saved_indices.resize(max_num_edges,-1); */
	/*   temp_saved_dis.resize(max_num_edges,1e9); */
	/*   for (auto it4 = common_set.begin(); it4!=common_set.end(); it4++){ */
	/*     WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[*it4]; */
	/*     if (wcp2.index != wcp1.index){ */
	/*       for (int qx = 0;qx!=max_num_edges; qx++){ */
	/* 	double dis = (wcp2.y-wcp1.y) * ref_dir_y[qx] + (wcp2.z-wcp1.z)*ref_dir_z[qx]; */
	/* 	if (dis > 0 && dis < temp_saved_dis.at(qx)){ */
	/* 	  temp_saved_dis[qx] = dis; */
	/* 	  temp_saved_indices[qx] = (*it4); */
	/* 	} */
	/*       } */
	/*     } */
	/*   } */
	/*   for (int qx = 0; qx!=max_num_edges;qx++){ */
	/*     if (temp_saved_indices[qx] >=0){ */
	/*       WCPointCloud<double>::WCPoint& wcp2 = cloud.pts[temp_saved_indices[qx]]; */
	/*       int index2 = wcp2.index; */
	/*       // add edge ... */
	/*       auto edge = add_edge(index1,index2,WCPPID::EdgeProp(sqrt(pow(wcp1.x-wcp2.x,2)+pow(wcp1.y-wcp2.y,2)+pow(wcp1.z-wcp2.z,2))),*graph); */
	/*       if (edge.second){ */
	/* 	num_edges ++; */
	/*       } */
	/*     } */
	/*   } */
	/* } */
	
      }
    }
  }

//    std::cout << closest_index.size() << " closest index entries created" << std::endl;


  for (auto it4 = closest_index.begin(); it4!=closest_index.end(); it4++){
    int index1 = it4->first.first;
    //std::cout << it4->second.size() << std::endl;

    for (auto it5 = it4->second.begin(); it5!=it4->second.end(); it5++){
      int index2 = (*it5).second;
      double dis = (*it5).first;
      auto edge = add_edge(index1,index2,WCPPID::EdgeProp(dis),*graph);

	  if (edge.second)
	//   std::cout << "Adding edge " << index1 << " " << index2 << " " << dis << " " << edge.second << std::endl;

      if (edge.second){
	//      (*graph)[edge.first].dist = dis;
		num_edges ++;
      }
      // protect against dead cells ...
      if (it5 == it4->second.begin() && dis > 0.25*units::cm)
	break;
    }
  }
  // end of copying ... 
}



void WCPPID::PR3DCluster::Connect_graph(WCP::ToyPointCloud* ref_point_cloud){
  WCP::WCPointCloud<double>& cloud = point_cloud->get_cloud();
  WCP::WC2DPointCloud<double>& cloud_u = point_cloud->get_cloud_u();
  WCP::WC2DPointCloud<double>& cloud_v = point_cloud->get_cloud_v();
  WCP::WC2DPointCloud<double>& cloud_w = point_cloud->get_cloud_w();

  TVector3 drift_dir(1,0,0);
  
  std::vector<int> component(num_vertices(*graph));
  const int num = connected_components(*graph,&component[0]);
  
  if (num >1){
    //std::cout << cluster_id << " " << num << std::endl;
    //for separated kd tree to find the closest points between disconnected components,
    std::vector<ToyPointCloud*> pt_clouds;
    for (int j=0;j!=num;j++){
      ToyPointCloud *pt_cloud = new ToyPointCloud();
      pt_clouds.push_back(pt_cloud);
    }
    
    std::vector<int>::size_type i;
    for (i=0;i!=component.size(); ++i){
      if(cloud.pts[i].mcell!=0){
	//	if (cloud.pts[i].mcell->Estimate_total_charge() > 0)
	if (cloud.pts[i].mcell->IsPointGood(cloud.pts[i].index_u, cloud.pts[i].index_v, cloud.pts[i].index_w,2)){
	  double temp_min_dis = 0;
	  if (ref_point_cloud!=0){
	    Point temp_p(cloud.pts[i].x,cloud.pts[i].y,cloud.pts[i].z);
	    temp_min_dis = ref_point_cloud->get_closest_dis(temp_p);
	  }
	  if (temp_min_dis < 1.0*units::cm)
	    pt_clouds.at(component[i])->AddPoint(cloud.pts[i],cloud_u.pts[i],cloud_v.pts[i],cloud_w.pts[i]);
	  else
	    excluded_points.insert(i);

	}else{
	  excluded_points.insert(i);
	}
      }else{
	excluded_points.insert(i);
      }
      //   std::cout << "Vertex " << i << " " << cloud.pts[i].x << " " << cloud.pts[i].y << " " << cloud.pts[i].z << " " << cloud.pts[i].index_u << " " << cloud.pts[i].index_v << " " << cloud.pts[i].index_w << " " << cloud.pts[i].mcell << " " << cloud.pts[i].mcell->GetTimeSlice()  << " is in component " << component[i] << std::endl;
    }
    for (int j=0;j!=num;j++){
      //std::cout << j << " " << pt_clouds.at(j)->get_num_points() << std::endl;
      pt_clouds.at(j)->build_kdtree_index();
    }
    
    //std::cout << "Xin: " << num << std::endl;
    // connect these graphs according to closest distance some how ...
    
    // std::tuple<int,int,double> index_index_dis[num][num];
    // std::tuple<int,int,double> index_index_dis_mst[num][num];
    // std::tuple<int,int,double> index_index_dis_dir1[num][num];
    // std::tuple<int,int,double> index_index_dis_dir2[num][num];
    
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis(num, std::vector< std::tuple<int,int,double> >(num));
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis_mst(num, std::vector< std::tuple<int,int,double> >(num));
    
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis_dir1(num, std::vector< std::tuple<int,int,double> >(num));
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis_dir2(num, std::vector< std::tuple<int,int,double> >(num));
    std::vector< std::vector< std::tuple<int,int,double> > > index_index_dis_dir_mst(num, std::vector< std::tuple<int,int,double> >(num));
    
    for (int j=0;j!=num;j++){
      for (int k=0;k!=num;k++){
	index_index_dis[j][k] = std::make_tuple(-1,-1,1e9);
	index_index_dis_mst[j][k] = std::make_tuple(-1,-1,1e9);
	index_index_dis_dir1[j][k] = std::make_tuple(-1,-1,1e9);
	index_index_dis_dir2[j][k] = std::make_tuple(-1,-1,1e9);
	index_index_dis_dir_mst[j][k] = std::make_tuple(-1,-1,1e9);
      }
    }
    
    
    //MST ...
    const int N = num;
    boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
			  boost::no_property, boost::property<boost::edge_weight_t, double>>
      temp_graph(N);
    
    
    
    for (int j=0;j!=num;j++){
      for (int k=j+1;k!=num;k++){
	if (pt_clouds.at(j)->get_cloud().pts.size() == 0 || pt_clouds.at(k)->get_cloud().pts.size() ==0 ) continue;
	index_index_dis[j][k] = pt_clouds.at(j)->get_closest_points(pt_clouds.at(k));
	int index1 = j;
	int index2 = k;
	auto edge = add_edge(index1,index2, std::get<2>(index_index_dis[j][k]), temp_graph);
      }
    }

    {
      std::vector<int> possible_root_vertex;
      std::vector<int> component(num_vertices(temp_graph));
      const int num1 = connected_components(temp_graph,&component[0]);
      possible_root_vertex.resize(num1);
      std::vector<int>::size_type i;
      for (i=0;i!=component.size(); ++i){
	possible_root_vertex.at(component[i]) = i;
      }
    
      for (size_t i=0;i!=possible_root_vertex.size();i++){
	std::vector<boost::graph_traits < WCPPID::MCUGraph >::vertex_descriptor> predecessors(num_vertices(temp_graph));
	
	prim_minimum_spanning_tree( temp_graph , &predecessors[0], boost::root_vertex(possible_root_vertex.at(i)));
	
	for (size_t j=0;j!=predecessors.size();++j){
	  if (predecessors[j]!=j){
	    if (j < predecessors[j]){
	      index_index_dis_mst[j][predecessors[j]] = index_index_dis[j][predecessors[j]];
	    }else{
	      index_index_dis_mst[predecessors[j]][j] = index_index_dis[predecessors[j]][j];
	    }
	    //std::cout << j << " " << predecessors[j] << " " << std::endl;
	  }else{
	    //std::cout << j << " " << std::endl;
	  }
	}
      }
    }
	//end of mst ...
    
    
    // short distance part
    for (int j=0;j!=num;j++){
      for (int k=j+1;k!=num;k++){
	if (pt_clouds.at(j)->get_cloud().pts.size() == 0 || pt_clouds.at(k)->get_cloud().pts.size() ==0 ) continue;
	// closest distance one ... 
	if (std::get<2>(index_index_dis[j][k])<3*units::cm){
	  index_index_dis_mst[j][k] = index_index_dis[j][k];
	}
	
	if (num < 100)
	  if (pt_clouds.at(j)->get_num_points()>100 && pt_clouds.at(k)->get_num_points()>100 &&
	      (pt_clouds.at(j)->get_num_points()+pt_clouds.at(k)->get_num_points()) > 400){
	    WCPointCloud<double>::WCPoint wp1 = cloud.pts.at(std::get<0>(index_index_dis[j][k]));
	    WCPointCloud<double>::WCPoint wp2 = cloud.pts.at(std::get<1>(index_index_dis[j][k]));
	    Point p1(wp1.x,wp1.y,wp1.z);
	    Point p2(wp2.x,wp2.y,wp2.z);
	    
	    TVector3 dir1 = VHoughTrans(p1, 30*units::cm,pt_clouds.at(j));
	    dir1 *= -1;
	    std::pair<int,double> result1 = pt_clouds.at(k)->get_closest_point_along_vec(p1, dir1, 80*units::cm, 5*units::cm, 7.5, 3*units::cm);
	    if (result1.first < 0 && fabs(dir1.Angle(drift_dir)/3.1415926*180.-90)<10.){
	      if (fabs(dir1.Angle(drift_dir)/3.1415926*180.-90)<5.) dir1 = VHoughTrans(p1, 80*units::cm,pt_clouds.at(j)); 
	      else if (fabs(dir1.Angle(drift_dir)/3.1415926*180.-90)<10.) dir1 = VHoughTrans(p1, 50*units::cm,pt_clouds.at(j));
	      dir1 *= -1;
	      result1 = pt_clouds.at(k)->get_closest_point_along_vec(p1, dir1, 80*units::cm, 5*units::cm, 7.5, 3*units::cm);
	    }
	    if (result1.first >=0){
	      index_index_dis_dir1[j][k] = std::make_tuple(std::get<0>(index_index_dis[j][k]), result1.first, result1.second);
	    }
	    
	   

	    TVector3 dir2 = VHoughTrans(p2, 30*units::cm,pt_clouds.at(k));
	    dir2 *= -1;
	    std::pair<int,double> result2 = pt_clouds.at(j)->get_closest_point_along_vec(p2, dir2, 80*units::cm, 5*units::cm, 7.5, 3*units::cm);
	    if (result2.first <0 && fabs(dir2.Angle(drift_dir)/3.1415926*180.-90)<10.){
	      if (fabs(dir2.Angle(drift_dir)/3.1415926*180.-90)<5.) dir2 = VHoughTrans(p2, 80*units::cm,pt_clouds.at(k));
	      else if (fabs(dir2.Angle(drift_dir)/3.1415926*180.-90)<10.) dir2 = VHoughTrans(p2, 50*units::cm,pt_clouds.at(k));
	      dir2 *= -1;
	      result2 = pt_clouds.at(j)->get_closest_point_along_vec(p2, dir2, 80*units::cm, 5*units::cm, 7.5, 3*units::cm);
	    }
	    
	    if (result2.first >=0){
	      index_index_dis_dir2[j][k] = std::make_tuple(result2.first, std::get<1>(index_index_dis[j][k]), result2.second);
	    }
	  }
      }
    }

    // test ...
    /* for (int j=0;j!=num;j++){ */
    /*   for (int k=j+1;k!=num;k++){ */
    /* 	if (std::get<0>(index_index_dis_dir1[j][k])>=0){ */
    /* 	  std::cout << "Xin5_1: " << j << " " << k << " " << std::get<0>(index_index_dis_dir1[j][k]) << " " << std::get<1>(index_index_dis_dir1[j][k]) << " " << std::get<2>(index_index_dis_dir1[j][k]) << std::endl; */
    /* 	} */
    /* 	if (std::get<0>(index_index_dis_dir2[j][k])>=0){ */
    /* 	  std::cout << "Xin6_1: " << std::get<0>(index_index_dis_dir2[j][k]) << " " << std::get<1>(index_index_dis_dir2[j][k]) << " " << std::get<2>(index_index_dis_dir2[j][k]) << std::endl; */
    /* 	} */
    /*   } */
    /* } */
    
    // MST for the directionality ...
    {
      const int N = num;
      boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
    	boost::no_property, boost::property<boost::edge_weight_t, double>>
    	temp_graph(N);
      
      for (int j=0;j!=num;j++){
    	for (int k=j+1;k!=num;k++){
	  int index1 = j;
	  int index2 = k;
    	  if (std::get<0>(index_index_dis_dir1[j][k])>=0 || std::get<0>(index_index_dis_dir2[j][k])>=0)
    	    auto edge = add_edge(index1,index2, std::min(std::get<2>(index_index_dis_dir1[j][k]), std::get<2>(index_index_dis_dir2[j][k])), temp_graph);
    	}
      }

      {
	std::vector<int> possible_root_vertex;
	std::vector<int> component(num_vertices(temp_graph));
	const int num1 = connected_components(temp_graph,&component[0]);
	possible_root_vertex.resize(num1);
	std::vector<int>::size_type i;
	for (i=0;i!=component.size(); ++i){
	  possible_root_vertex.at(component[i]) = i;
	}
	
	for (size_t i=0;i!=possible_root_vertex.size();i++){
	  std::vector<boost::graph_traits < WCPPID::MCUGraph >::vertex_descriptor> predecessors(num_vertices(temp_graph));
	  prim_minimum_spanning_tree( temp_graph , &predecessors[0], boost::root_vertex(possible_root_vertex.at(i)));
	  for (size_t j=0;j!=predecessors.size();++j){
	    if (predecessors[j]!=j){
	      if (j < predecessors[j]){
		index_index_dis_dir_mst[j][predecessors[j]] = index_index_dis[j][predecessors[j]];
	      }else{
		index_index_dis_dir_mst[predecessors[j]][j] = index_index_dis[predecessors[j]][j];
	      }
	      //std::cout << j << " " << predecessors[j] << " " << std::endl;
	    }else{
	      //std::cout << j << " " << std::endl;
	    }
	  }
	}
      }
    }
	
    // now complete graph according to the direction
    // according to direction ...
    for (int j=0;j!=num;j++){
      for (int k=j+1;k!=num;k++){
	if (std::get<0>(index_index_dis_mst[j][k])>=0){	  
	  if (std::get<2>(index_index_dis_mst[j][k])>5*units::cm){
	    auto edge = add_edge(std::get<0>(index_index_dis_mst[j][k]),std::get<1>(index_index_dis_mst[j][k]),WCPPID::EdgeProp(std::get<2>(index_index_dis_mst[j][k])),*graph);
	  }else{
	    auto edge = add_edge(std::get<0>(index_index_dis_mst[j][k]),std::get<1>(index_index_dis_mst[j][k]),WCPPID::EdgeProp(std::get<2>(index_index_dis_mst[j][k])),*graph);
	  }
	  // std::cout << "Xin4: " << std::get<0>(index_index_dis_mst[j][k]) << " " << std::get<1>(index_index_dis_mst[j][k]) << " " << std::get<2>(index_index_dis_mst[j][k]) << std::endl;
	  
	}

	if (std::get<0>(index_index_dis_dir_mst[j][k])>=0){
	  if (std::get<0>(index_index_dis_dir1[j][k])>=0){
	    if (std::get<2>(index_index_dis_dir1[j][k])>5*units::cm){
	      auto edge = add_edge(std::get<0>(index_index_dis_dir1[j][k]),std::get<1>(index_index_dis_dir1[j][k]),WCPPID::EdgeProp(std::get<2>(index_index_dis_dir1[j][k])*1.2),*graph);
	    }else{
	      auto edge = add_edge(std::get<0>(index_index_dis_dir1[j][k]),std::get<1>(index_index_dis_dir1[j][k]),WCPPID::EdgeProp(std::get<2>(index_index_dis_dir1[j][k])),*graph);
	    }
	    //std::cout << "Xin5: " << std::get<0>(index_index_dis_dir1[j][k]) << " " << std::get<1>(index_index_dis_dir1[j][k]) << " " << std::get<2>(index_index_dis_dir1[j][k])/units::cm << " " << cloud.pts[std::get<0>(index_index_dis_dir1[j][k])].x << " " << cloud.pts[std::get<0>(index_index_dis_dir1[j][k])].y << " " << cloud.pts[std::get<0>(index_index_dis_dir1[j][k])].z << " -> " << cloud.pts[std::get<1>(index_index_dis_dir1[j][k])].x << " " << cloud.pts[std::get<1>(index_index_dis_dir1[j][k])].y << " " << cloud.pts[std::get<1>(index_index_dis_dir1[j][k])].z <<std::endl;
	  }
	  if (std::get<0>(index_index_dis_dir2[j][k])>=0){
	    if (std::get<2>(index_index_dis_dir2[j][k])>5*units::cm){
		auto edge = add_edge(std::get<0>(index_index_dis_dir2[j][k]),std::get<1>(index_index_dis_dir2[j][k]),WCPPID::EdgeProp(std::get<2>(index_index_dis_dir2[j][k])*1.2),*graph);
	    }else{
		auto edge = add_edge(std::get<0>(index_index_dis_dir2[j][k]),std::get<1>(index_index_dis_dir2[j][k]),WCPPID::EdgeProp(std::get<2>(index_index_dis_dir2[j][k])),*graph);
	    }
	    //	    std::cout << "Xin6: " << std::get<0>(index_index_dis_dir2[j][k]) << " " << std::get<1>(index_index_dis_dir2[j][k]) << " " << std::get<2>(index_index_dis_dir2[j][k]) << std::endl;
	  }
	}
	
      }
    }
    
    
    
    
    
    
    for (int i=0;i!=num;i++){
      delete pt_clouds.at(i);
    }
  }
  
}


void WCPPID::PR3DCluster::search_for_connection_isochronous(std::pair<int,double>& result1, WCP::Point& p1, TVector3& dir1, ToyPointCloud* pcloud1, ToyPointCloud* pcloud2, double search_distance, double angle_cut, double tran_dis_cut){
  TVector3 drift_dir(1,0,0);
  if (result1.first <0 && fabs(drift_dir.Angle(dir1)/3.1415926*180.-90.) < angle_cut){ 
    // isochrounous track situation ... 
    dir1 = dir1.Unit();
    TVector3 dir2 = dir1.Cross(drift_dir);
    dir2 = dir2.Unit(); // in plane
    TVector3 dir3 = dir1.Cross(dir2);

    // calculate RMS values
    double rms[3]={0,0,0};
    double mean[3]={0,0,0};
    std::vector<std::pair<WCP::SlimMergeGeomCell*,Point>>pts = pcloud1->get_closest_points(p1,30*units::cm);

    if (pts.size()<=1) return;
    
    for (size_t i=0;i!=pts.size();i++){
      TVector3 dir(pts.at(i).second.x - p1.x, pts.at(i).second.y - p1.y, pts.at(i).second.z - p1.z);
      double tmp1 = dir.Dot(dir1);
      double tmp2 = dir.Dot(dir2);
      double tmp3 = dir.Dot(dir3);

      mean[0] += tmp1;
      rms[0] += pow(tmp1,2);

      mean[1] += tmp2;
      rms[1] += pow(tmp2,2);

      mean[2] += tmp3;
      rms[2] += pow(tmp3,2);
    }
    rms[0] = sqrt( (rms[0] - pow(mean[0],2)/pts.size())/(pts.size()-1.));
    rms[1] = sqrt( (rms[1] - pow(mean[1],2)/pts.size())/(pts.size()-1.));
    rms[2] = sqrt( (rms[2] - pow(mean[2],2)/pts.size())/(pts.size()-1.));
    mean[0] /= pts.size();
    mean[1] /= pts.size();
    mean[2] /= pts.size();
    double orig_dis = sqrt(pow(mean[0],2) + pow(mean[1],2) + pow(mean[2],2));
    //    std::cout << rms[0]/units::cm << " " << rms[1]/units::cm << " " << rms[2]/units::cm << std::endl;
    /* 	      std::cout << dir1.X() << " " << dir1.Y() << " " << dir1.Z() << " " << dir1_2.X() << " " << dir1_2.Y() << " " << dir1_2.Z() << " " << dir1_3.X() << " " << dir1_3.Y() << " " << dir1_3.Z() << std::endl; */
    WCP::WCPointCloud<double>& tmp_cloud = pcloud2->get_cloud(); 
    double min_dis = 1e9; int min_index = -1; 
    double min_dis1, min_dis2, min_dis3; 
    for (size_t qx = 0; qx!= tmp_cloud.pts.size(); qx++){ 
      TVector3 tmp_dir(tmp_cloud.pts.at(qx).x - p1.x, tmp_cloud.pts.at(qx).y - p1.y, tmp_cloud.pts.at(qx).z - p1.z); 
      double tmp_dis = tmp_dir.Mag();
      double tmp_dis1 = tmp_dir.Dot(dir1); 
      double tmp_dis2 = tmp_dir.Dot(dir2); 
      double tmp_dis3 = tmp_dir.Dot(dir3); 
      if (fabs(tmp_dis3)<tran_dis_cut && fabs(tmp_dis3) < rms[2]/orig_dis * tmp_dis || fabs(tmp_dis3) < 0.9*units::cm){ 
	double value = pow(2*tmp_dis1/search_distance,2) + pow(tmp_dis2*orig_dis,2)/(pow(rms[0]*tmp_dis,2)+pow(rms[1]*tmp_dis,2))
	  + pow(tmp_dis3*orig_dis,2)/pow(rms[2]*tmp_dis,2);
	if (value < min_dis){ 
	  min_dis = value;
	  min_index = qx; 
	  min_dis1 = tmp_dis1; 
	  min_dis2 = tmp_dis2;
	  min_dis3 = tmp_dis3;
	} 

      }
    }
    if (min_index >=0){
      result1.first = tmp_cloud.pts[min_index].index;
      result1.second = sqrt(pow(tmp_cloud.pts[min_index].x-p1.x,2)+pow(tmp_cloud.pts[min_index].y-p1.y,2)+pow(tmp_cloud.pts[min_index].z-p1.z,2));
      std::cout << p1 << " " << min_dis1/units::cm << " " << min_dis2/units::cm << " " << min_dis3/units::cm << " " << sqrt(pow(min_dis1,2) + pow(min_dis2,2) + pow(min_dis3,2))/units::cm << " " << tmp_cloud.pts[min_index].x << " " << tmp_cloud.pts[min_index].y << " " << tmp_cloud.pts[min_index].z << std::endl;
    }
      
    
 
  } 


}
