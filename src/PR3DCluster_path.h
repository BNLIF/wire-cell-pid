std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> WCPPID::PR3DCluster::get_highest_lowest_wcps(int flag){
  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  
  WCP::WCPointCloud<double>& cloud = temp_point_cloud->get_cloud();
  WCPointCloud<double>::WCPoint highest_wcp = cloud.pts[0];
  WCPointCloud<double>::WCPoint lowest_wcp = cloud.pts[0];

  bool flag_init = false;
  
  for (size_t i=0;i<cloud.pts.size();i++){
    if (excluded_points.find(i)!=excluded_points.end()) continue;
    if (!flag_init){
      highest_wcp = cloud.pts[i];
      lowest_wcp = cloud.pts[i];
      flag_init = true;
    }else{
      if (cloud.pts[i].y > highest_wcp.y)
	highest_wcp = cloud.pts[i];
      if (cloud.pts[i].y < lowest_wcp.y)
	lowest_wcp = cloud.pts[i];
    }
  }
  return std::make_pair(highest_wcp,lowest_wcp);
}

std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> WCPPID::PR3DCluster::get_front_back_wcps(int flag){
  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  WCP::WCPointCloud<double>& cloud = temp_point_cloud->get_cloud();
  WCPointCloud<double>::WCPoint highest_wcp = cloud.pts[0];
  WCPointCloud<double>::WCPoint lowest_wcp = cloud.pts[0];
  bool flag_init = false;
  for (size_t i=0;i<cloud.pts.size();i++){
    if (excluded_points.find(i)!=excluded_points.end()) continue;
    if (!flag_init){
      highest_wcp = cloud.pts[i];
      lowest_wcp = cloud.pts[i];
      flag_init = true;
    }else{
      if (cloud.pts[i].z > highest_wcp.z)
	highest_wcp = cloud.pts[i];
      if (cloud.pts[i].z < lowest_wcp.z)
	lowest_wcp = cloud.pts[i];
    }
  }
  return std::make_pair(highest_wcp,lowest_wcp);
}


std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> WCPPID::PR3DCluster::get_earliest_latest_wcps(int flag){
  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  WCP::WCPointCloud<double>& cloud = temp_point_cloud->get_cloud();
  WCPointCloud<double>::WCPoint highest_wcp = cloud.pts[0];
  WCPointCloud<double>::WCPoint lowest_wcp = cloud.pts[0];
  bool flag_init = false;
  for (size_t i=0;i<cloud.pts.size();i++){
    if (excluded_points.find(i)!=excluded_points.end()) continue;
    if (!flag_init){
      highest_wcp = cloud.pts[i];
      lowest_wcp = cloud.pts[i];
      flag_init = true;
    }else{
      if (cloud.pts[i].x > highest_wcp.x)
	highest_wcp = cloud.pts[i];
      if (cloud.pts[i].x < lowest_wcp.x)
	lowest_wcp = cloud.pts[i];
    }
  }
  return std::make_pair(lowest_wcp,highest_wcp);
}

std::vector<std::vector<WCPointCloud<double>::WCPoint>> WCPPID::PR3DCluster::get_extreme_wcps(int flag, std::map<int,SMGCSelection>* old_time_mcells_map){
  
  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  WCP::WCPointCloud<double>& cloud = temp_point_cloud->get_cloud();

  std::vector<int> all_indices;
  if (old_time_mcells_map==0){
    for (size_t i=0;i<cloud.pts.size();i++){
      all_indices.push_back(i);
    }
  }else{
    // scan
    for (size_t i=0;i<cloud.pts.size();i++){
      int time_slice = cloud.pts[i].mcell->GetTimeSlice();
      bool flag_add = false;
      
      if (old_time_mcells_map->find(time_slice)!=old_time_mcells_map->end()){
	for (auto it1 = (*old_time_mcells_map)[time_slice].begin(); it1!= (*old_time_mcells_map)[time_slice].end(); it1++){
	  SlimMergeGeomCell *mcell = *it1;
	  int u1_low_index = mcell->get_uwires().front()->index();
	  int u1_high_index = mcell->get_uwires().back()->index();
	  
	  int v1_low_index = mcell->get_vwires().front()->index();
	  int v1_high_index = mcell->get_vwires().back()->index();
	  
	  int w1_low_index = mcell->get_wwires().front()->index();
	  int w1_high_index = mcell->get_wwires().back()->index();
	  
	  if (cloud.pts[i].index_u <= u1_high_index &&
	      cloud.pts[i].index_u >= u1_low_index &&
	      cloud.pts[i].index_v <= v1_high_index &&
	      cloud.pts[i].index_v >= v1_low_index &&
	      cloud.pts[i].index_w <= w1_high_index &&
	      cloud.pts[i].index_w >= w1_low_index ){
	   flag_add = true;
	   break;
	  }
	}
      }
      if (flag_add){
	all_indices.push_back(i);
      }
    }
  }

  std::vector<std::vector<WCPointCloud<double>::WCPoint>> out_vec_wcps;
  if (all_indices.size()==0){
    return out_vec_wcps;  
  }
    
  
  Calc_PCA();
  WCPointCloud<double>::WCPoint wcps[8];
  for (int i=0;i!=8;i++){
    wcps[i] = cloud.pts[all_indices.at(0)];
  }
  Vector main_axis = get_PCA_axis(0);
  if (main_axis.y <0){
    main_axis.x = -main_axis.x;
    main_axis.y = -main_axis.y;
    main_axis.z = -main_axis.z;
  }
  double high_value = wcps[0].x*main_axis.x + wcps[0].y*main_axis.y + wcps[0].z*main_axis.z;
  double low_value = wcps[1].x * main_axis.x + wcps[1].y * main_axis.y + wcps[1].z * main_axis.z ;

  bool flag_init = false;
  
  for (size_t i=0;i<all_indices.size();i++){
    
    if (excluded_points.find(all_indices.at(i))!=excluded_points.end()) continue;
    if (!flag_init){
      for (int j=0;j!=8;j++){
	wcps[j] = cloud.pts[all_indices.at(i)];
      }
      high_value = wcps[0].x*main_axis.x + wcps[0].y*main_axis.y + wcps[0].z*main_axis.z;
      low_value = wcps[1].x * main_axis.x + wcps[1].y * main_axis.y + wcps[1].z * main_axis.z ;
      flag_init = true;
    }else{
      double value = cloud.pts[all_indices.at(i)].x*main_axis.x + cloud.pts[all_indices.at(i)].y*main_axis.y + cloud.pts[all_indices.at(i)].z*main_axis.z;
      if (value > high_value){
	wcps[0] = cloud.pts[all_indices.at(i)];
	high_value = value;
      }
      if (value < low_value){
	wcps[1] = cloud.pts[all_indices.at(i)];
	low_value = value;
      }
      // top down
      if (cloud.pts[all_indices.at(i)].y > wcps[2].y)
	wcps[2] = cloud.pts[all_indices.at(i)];
      if (cloud.pts[all_indices.at(i)].y < wcps[3].y)
	wcps[3] = cloud.pts[all_indices.at(i)];
      
      // front back
      if (cloud.pts[all_indices.at(i)].z > wcps[4].z)
	wcps[4] = cloud.pts[all_indices.at(i)];
      if (cloud.pts[all_indices.at(i)].z < wcps[5].z)
	wcps[5] = cloud.pts[all_indices.at(i)];
      
      // early late
      if (cloud.pts[all_indices.at(i)].x > wcps[6].x)
	wcps[6] = cloud.pts[all_indices.at(i)];
      if (cloud.pts[all_indices.at(i)].x < wcps[7].x)
	wcps[7] = cloud.pts[all_indices.at(i)];
    }
  }

  
  

  
  

  {
    // first extreme along the main axis
    std::vector<WCPointCloud<double>::WCPoint> saved_wcps;
    saved_wcps.push_back(wcps[0]);
    out_vec_wcps.push_back(saved_wcps);
  }

  {
    // second extreme along the main axis
    std::vector<WCPointCloud<double>::WCPoint> saved_wcps;
    saved_wcps.push_back(wcps[1]);
    out_vec_wcps.push_back(saved_wcps);
  }

    // std::cout << out_vec_wcps.at(0).size() << " " << out_vec_wcps.at(1).size() << " " << out_vec_wcps.size() << std::endl;


  // std::cout << std::endl;
  for (int i=2;i!=8;i++){
    //if (cluster_id==16)
    

    bool flag_save = true;
    for (size_t j=0;j!=out_vec_wcps.size(); j++){
      double dis = sqrt(pow(out_vec_wcps[j].at(0).x-wcps[i].x,2) + pow(out_vec_wcps[j].at(0).y - wcps[i].y,2) + pow(out_vec_wcps[j].at(0).z - wcps[i].z,2));
      if (dis < 5*units::cm){
        out_vec_wcps.at(j).push_back(wcps[i]);
        flag_save = false;
        break;
      }
    }
    
        // std::cout << i << " C " << wcps[i].x/units::cm << " " << wcps[i].y/units::cm << " " << wcps[i].z/units::cm << " " << flag_save << std::endl;


    if (flag_save){
      std::vector<WCPointCloud<double>::WCPoint> saved_wcps;
      saved_wcps.push_back(wcps[i]);
      out_vec_wcps.push_back(saved_wcps);
    }
  }


  return out_vec_wcps;  
}

std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> WCPPID::PR3DCluster::get_main_axis_wcps(int flag){
  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  
  WCP::WCPointCloud<double>& cloud = temp_point_cloud->get_cloud();
  Calc_PCA();
  WCPointCloud<double>::WCPoint highest_wcp = cloud.pts[0];
  WCPointCloud<double>::WCPoint lowest_wcp = cloud.pts[0];
  Vector main_axis = get_PCA_axis(0);
  if (main_axis.y <0){
    main_axis.x = -main_axis.x;
    main_axis.y = -main_axis.y;
    main_axis.z = -main_axis.z;
  }
  
  double high_value = highest_wcp.x*main_axis.x + highest_wcp.y*main_axis.y + highest_wcp.z*main_axis.z;
  double low_value = lowest_wcp.x * main_axis.x + lowest_wcp.y * main_axis.y + lowest_wcp.z * main_axis.z ;

  bool flag_init = false;
  
  for (size_t i=0;i<cloud.pts.size();i++){
     if (excluded_points.find(i)!=excluded_points.end()) continue;
     if (!flag_init){
       highest_wcp = cloud.pts[i];
       lowest_wcp = cloud.pts[i];
       high_value = highest_wcp.x*main_axis.x + highest_wcp.y*main_axis.y + highest_wcp.z*main_axis.z;
       low_value = lowest_wcp.x * main_axis.x + lowest_wcp.y * main_axis.y + lowest_wcp.z * main_axis.z ;
       flag_init = true;
    }else{
       double value = cloud.pts[i].x*main_axis.x + cloud.pts[i].y*main_axis.y + cloud.pts[i].z*main_axis.z;
       if (value > high_value){
	 highest_wcp = cloud.pts[i];
	 high_value = value;
       }
       
       if (value < low_value){
	 lowest_wcp = cloud.pts[i];
	 low_value = value;
       }
     }
  }
  return std::make_pair(highest_wcp,lowest_wcp);
}

WCP::WCPointCloud<double>::WCPoint WCPPID::PR3DCluster::get_local_extension(WCP::WCPointCloud<double>::WCPoint wcp, int flag){
  Create_point_cloud();
  
  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  WCP::Point p(wcp.x, wcp.y, wcp.z);
  
  TVector3 dir1 = VHoughTrans(p, 10*units::cm);
  dir1 *= (-1);
  TVector3 drift_dir(1,0,0);

  if (fabs(dir1.Angle(drift_dir)/3.1415926*180.-90) < 7.5) return wcp;
  
  
  double max_val = 0;
  std::vector<WCP::WCPointCloud<double>::WCPoint > wcps = temp_point_cloud->get_closest_wcpoints(p, 10*units::cm);
  for (size_t i=0;i!=wcps.size();i++){
    double val = dir1.X() * (wcps.at(i).x -p.x) + dir1.Y() * (wcps.at(i).y - p.y) + dir1.Z() *(wcps.at(i).z - p.z);
    if (val > max_val){
      max_val =val;
      wcp = wcps.at(i);
    }
  }
  
  
  return wcp;
}

int WCPPID::PR3DCluster::get_num_outside_range_points(int flag ){
  Create_point_cloud();
  
  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  //  std::cout << temp_point_cloud << std::endl;
  WCP::WCPointCloud<double>& cloud = temp_point_cloud->get_cloud();

  double min_x = 1e9, max_x = -1e9;
  double min_u = 1e9, max_u = -1e9;
  double min_v = 1e9, max_v = -1e9;
  double min_w = 1e9, max_w = -1e9;
  for (auto it = path_wcps.begin(); it!= path_wcps.end(); it++){
    if ( (*it).x < min_x) min_x = (*it).x;
    if ( (*it).x > max_x) max_x = (*it).x;
    if ( (*it).index_u < min_u) min_u = (*it).index_u;
    if ( (*it).index_u > max_u) max_u = (*it).index_u;
    if ( (*it).index_v < min_v) min_v = (*it).index_v;
    if ( (*it).index_v > max_v) max_v = (*it).index_v;
    if ( (*it).index_w < min_w) min_w = (*it).index_w;
    if ( (*it).index_w > max_w) max_w = (*it).index_w;
  }

  int num = 0;
  for (size_t i=0;i<cloud.pts.size();i++){
    if (cloud.pts[i].index_u > max_u || cloud.pts[i].index_u < min_u
	|| cloud.pts[i].index_v > max_v || cloud.pts[i].index_v < min_v
	|| cloud.pts[i].index_w > max_w || cloud.pts[i].index_w < min_w
	|| cloud.pts[i].x > max_x || cloud.pts[i].x < min_x)
      num ++;
  }
  
  //  std::cout << min_u << " " << max_u << " " << min_x << " " << max_x << " " << min_v << " " << max_v << " " << min_w << " " << max_w << std::endl;
  return num;
}

std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> WCPPID::PR3DCluster::get_two_boundary_wcps(int flag, bool flag_cosmic){
  Create_point_cloud();
  
  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  //  std::cout << temp_point_cloud << std::endl;
  WCP::WCPointCloud<double>& cloud = temp_point_cloud->get_cloud();
  if (cloud.pts.size()==1) return std::make_pair(cloud.pts[0], cloud.pts[0]);
  // std::cout << cloud.pts.size() << " " << flag << std::endl;
  WCPointCloud<double>::WCPoint extreme_wcp[14];
  
  
  Calc_PCA();
  Vector main_axis = get_PCA_axis(0);
  Vector second_axis = get_PCA_axis(1);
  double values[4];
  
  
  bool flag_init = false;
  for (size_t i=0;i<cloud.pts.size();i++){
    if (cloud.pts[i].mcell==0) continue;
    if (cloud.pts[i].mcell->Estimate_total_charge() < 1500) continue;
    if (excluded_points.find(i)!=excluded_points.end()) continue;
    if (!flag_init){
      flag_init = true;
      for (int j=0;j!=14;j++){
	extreme_wcp[j] = cloud.pts[i];
      }
      values[0] = extreme_wcp[0].x*main_axis.x + extreme_wcp[0].y*main_axis.y + extreme_wcp[0].z*main_axis.z;
      values[1] = extreme_wcp[1].x*main_axis.x + extreme_wcp[1].y*main_axis.y + extreme_wcp[1].z*main_axis.z;
      values[2] = extreme_wcp[2].x*second_axis.x + extreme_wcp[2].y*second_axis.y + extreme_wcp[2].z*second_axis.z;
      values[3] = extreme_wcp[3].x*second_axis.x + extreme_wcp[3].y*second_axis.y + extreme_wcp[3].z*second_axis.z;
    }else{
      // main axis points
      double value = cloud.pts[i].x*main_axis.x + cloud.pts[i].y*main_axis.y + cloud.pts[i].z*main_axis.z;
      if (value > values[0]){
	values[0] = value;
	extreme_wcp[0] = cloud.pts[i];
      }
      if (value < values[1]){
	values[1] = value;
	extreme_wcp[1] = cloud.pts[i];
      }
      // second axis points
      value = cloud.pts[i].x*second_axis.x + cloud.pts[i].y*second_axis.y + cloud.pts[i].z*second_axis.z;
      if (value > values[2]){
	values[2] = value;
	extreme_wcp[2] = cloud.pts[i];
      }
      if (value < values[3]){
	values[3] = value;
	extreme_wcp[3] = cloud.pts[i];
      }
      // early-late  x
      if (cloud.pts[i].x > extreme_wcp[4].x){
	extreme_wcp[4] = cloud.pts[i];
      }
      if (cloud.pts[i].x < extreme_wcp[5].x){
	extreme_wcp[5] = cloud.pts[i];
      }
      // top-bottom  y 
      if (cloud.pts[i].y > extreme_wcp[6].y){
	extreme_wcp[6] = cloud.pts[i];
      }
      if (cloud.pts[i].y < extreme_wcp[7].y){
	extreme_wcp[7] = cloud.pts[i];
      }
      // left-right  z or w
      if (cloud.pts[i].z > extreme_wcp[8].z){
	extreme_wcp[8] = cloud.pts[i];
      }
      if (cloud.pts[i].z < extreme_wcp[9].z){
	extreme_wcp[9] = cloud.pts[i];
      }
      // U range
      if (cloud.pts[i].index_u > extreme_wcp[10].index_u){
	extreme_wcp[10] = cloud.pts[i];
      }
      if (cloud.pts[i].index_u < extreme_wcp[11].index_u){
	extreme_wcp[11] = cloud.pts[i];
      }
      // V range
      if (cloud.pts[i].index_v > extreme_wcp[12].index_v){
	extreme_wcp[12] = cloud.pts[i];
      }
      if (cloud.pts[i].index_v < extreme_wcp[13].index_v){
	extreme_wcp[13] = cloud.pts[i];
      }
    }
  }
  
  if (!flag_init){
    flag_init = true;
    for (int i=0;i!=14;i++){
      extreme_wcp[i] = cloud.pts[0];
    }
    extreme_wcp[1] = cloud.pts[1];
  }

  // count live channels
  std::set<int> live_u_index;
  std::set<int> live_v_index;
  std::set<int> live_w_index;
  for (auto it = mcells.begin();  it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = *it;
    GeomWireSelection& uwires = mcell->get_uwires();
    GeomWireSelection& vwires = mcell->get_vwires();
    GeomWireSelection& wwires = mcell->get_wwires();

    std::vector<WirePlaneType_t> bad_planes = mcell->get_bad_planes();

    if (find(bad_planes.begin(), bad_planes.end(), WirePlaneType_t(0))==bad_planes.end()){
      for (auto it1 = uwires.begin(); it1!=uwires.end(); it1++){
	live_u_index.insert((*it1)->index());
      }
    }

    if (find(bad_planes.begin(), bad_planes.end(), WirePlaneType_t(1))==bad_planes.end()){
      for (auto it1 = vwires.begin(); it1!=vwires.end(); it1++){
	live_v_index.insert((*it1)->index());
      }
    }

    if (find(bad_planes.begin(), bad_planes.end(), WirePlaneType_t(2))==bad_planes.end()){
      for (auto it1 = wwires.begin(); it1!=wwires.end(); it1++){
	live_w_index.insert((*it1)->index());
      }
    }
    //    std::cout << live_u_index.size() << " " << live_v_index.size() << " " << live_w_index.size() << std::endl;
    /* for (auto it = bad_planes.begin(); it!=bad_planes.end(); it++){ */
    /*   std::cout << *it << std::endl; */
    /* } */
  }
  //  std::cout << live_u_index.size() << " " << live_v_index.size() << " " << live_w_index.size() << std::endl;

  
  std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> boundary_points;
  boundary_points.first = extreme_wcp[0];
  boundary_points.second = extreme_wcp[1];
  int ncount_live_u = 0;
  int ncount_live_v = 0;
  int ncount_live_w = 0;
  int temp_min_u, temp_max_u;
  int temp_min_v, temp_max_v;
  int temp_min_w, temp_max_w;
  if (boundary_points.first.index_u < boundary_points.second.index_u){
    temp_min_u = boundary_points.first.index_u;
    temp_max_u = boundary_points.second.index_u;
  }else{
    temp_max_u = boundary_points.first.index_u;
    temp_min_u = boundary_points.second.index_u;
  }
  if (boundary_points.first.index_v < boundary_points.second.index_v){
    temp_min_v = boundary_points.first.index_v;
    temp_max_v = boundary_points.second.index_v;
  }else{
    temp_max_v = boundary_points.first.index_v;
    temp_min_v = boundary_points.second.index_v;
  }
  if (boundary_points.first.index_w < boundary_points.second.index_w){
    temp_min_w = boundary_points.first.index_w;
    temp_max_w = boundary_points.second.index_w;
  }else{
    temp_max_w = boundary_points.first.index_w;
    temp_min_w = boundary_points.second.index_w;
  }
  
  for (int temp_index = temp_min_u; temp_index <= temp_max_u; temp_index ++ ){
    if (live_u_index.find(temp_index)!=live_u_index.end()) ncount_live_u++;
  }
  for (int temp_index = temp_min_v; temp_index <= temp_max_v; temp_index ++ ){
    if (live_v_index.find(temp_index)!=live_v_index.end()) ncount_live_v++;
  }
  for (int temp_index = temp_min_w; temp_index <= temp_max_w; temp_index ++ ){
    if (live_w_index.find(temp_index)!=live_w_index.end()) ncount_live_w++;
  }
  
  double boundary_value = fabs(boundary_points.first.x-boundary_points.second.x)/(2.22*units::mm)
    + fabs(boundary_points.first.index_u - boundary_points.second.index_u) * 0.0 + ncount_live_u * 1.0
    + fabs(boundary_points.first.index_v - boundary_points.second.index_v) * 0.0 + ncount_live_v * 1.0
    + fabs(boundary_points.first.index_w - boundary_points.second.index_w) * 0.0 + ncount_live_w * 1.0;
  if (flag_cosmic)
    boundary_value = fabs(boundary_points.first.x-boundary_points.second.x)/(units::mm)
    + fabs(boundary_points.first.index_u - boundary_points.second.index_u) * 1.0 + ncount_live_u * 1.
    + fabs(boundary_points.first.index_v - boundary_points.second.index_v) * 1.0 + ncount_live_v * 1.
    + fabs(boundary_points.first.index_w - boundary_points.second.index_w) * 1.0 + ncount_live_w * 1.
      + sqrt(pow(boundary_points.first.x - boundary_points.second.x,2) + pow(boundary_points.first.y - boundary_points.second.y,2) + pow(boundary_points.first.z - boundary_points.second.z,2))/(units::mm);
  
  for (int i=0;i<14;i++){
    for (int j=i+1;j<14;j++){
      ncount_live_u = 0;
      ncount_live_v = 0;
      ncount_live_w = 0;
     
      if (extreme_wcp[i].index_u < extreme_wcp[j].index_u){
	temp_min_u = extreme_wcp[i].index_u;
	temp_max_u = extreme_wcp[j].index_u;
      }else{
	temp_max_u = extreme_wcp[i].index_u;
	temp_min_u = extreme_wcp[j].index_u;
      }
      if (extreme_wcp[i].index_v < extreme_wcp[j].index_v){
	temp_min_v = extreme_wcp[i].index_v;
	temp_max_v = extreme_wcp[j].index_v;
      }else{
	temp_max_v = extreme_wcp[i].index_v;
	temp_min_v = extreme_wcp[j].index_v;
      }
      if (extreme_wcp[i].index_w < extreme_wcp[j].index_w){
	temp_min_w = extreme_wcp[i].index_w;
	temp_max_w = extreme_wcp[j].index_w;
      }else{
	temp_max_w = extreme_wcp[i].index_w;
	temp_min_w = extreme_wcp[j].index_w;
      }
      
      for (int temp_index = temp_min_u; temp_index <= temp_max_u; temp_index ++ ){
	if (live_u_index.find(temp_index)!=live_u_index.end()) ncount_live_u++;
      }
      for (int temp_index = temp_min_v; temp_index <= temp_max_v; temp_index ++ ){
	if (live_v_index.find(temp_index)!=live_v_index.end()) ncount_live_v++;
      }
      for (int temp_index = temp_min_w; temp_index <= temp_max_w; temp_index ++ ){
	if (live_w_index.find(temp_index)!=live_w_index.end()) ncount_live_w++;
      }
      
      
      double value = fabs(extreme_wcp[i].x - extreme_wcp[j].x)/(2.22*units::mm)
	+ fabs(extreme_wcp[i].index_u - extreme_wcp[j].index_u) * 0 + ncount_live_u * 1.0
	+ fabs(extreme_wcp[i].index_v - extreme_wcp[j].index_v) * 0 + ncount_live_v * 1.0
	+ fabs(extreme_wcp[i].index_w - extreme_wcp[j].index_w) * 0 + ncount_live_w * 1.0;

      if (flag_cosmic)
	value = fabs(extreme_wcp[i].x - extreme_wcp[j].x)/(units::mm)
	+ fabs(extreme_wcp[i].index_u - extreme_wcp[j].index_u) * 1.0 + ncount_live_u * 1.
	+ fabs(extreme_wcp[i].index_v - extreme_wcp[j].index_v) * 1.0 + ncount_live_v * 1.
	+ fabs(extreme_wcp[i].index_w - extreme_wcp[j].index_w) * 1.0 + ncount_live_w * 1.
	  + sqrt(pow(extreme_wcp[i].x-extreme_wcp[j].x,2) + pow(extreme_wcp[i].y-extreme_wcp[j].y,2) + pow(extreme_wcp[i].z-extreme_wcp[j].z,2))/(units::mm)*1;
     

      if (value > boundary_value){
	/* if (cluster_id==28&& value > 275) */
	
	 /* std::cout << extreme_wcp[i].x/units::cm << " " << extreme_wcp[i].y/units::cm << " " << extreme_wcp[i].z/units::cm << " " */
	 /* 	  << extreme_wcp[j].x/units::cm << " " << extreme_wcp[j].y/units::cm << " " << extreme_wcp[j].z/units::cm << " " << value << " " << ncount_live_u << " " << ncount_live_v << " " << ncount_live_w << std::endl; */
	 
	boundary_value = value;
	if (extreme_wcp[i].y > extreme_wcp[j].y){
	  boundary_points.first = extreme_wcp[i];
	  boundary_points.second = extreme_wcp[j];
	}else{
	  boundary_points.first = extreme_wcp[j];
	  boundary_points.second = extreme_wcp[i];
	}
      }
    }
  }

  if (boundary_points.first.y < boundary_points.second.y){
    WCP::WCPointCloud<double>::WCPoint temp_wcp = boundary_points.first;
    boundary_points.first = boundary_points.second;
    boundary_points.second = temp_wcp;
  }
  
  return boundary_points;
}


std::pair<Point,Point> WCPPID::PR3DCluster::get_two_extreme_points(int flag){
  Create_point_cloud();
  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  WCP::WCPointCloud<double>& cloud = temp_point_cloud->get_cloud();
  WCPointCloud<double>::WCPoint extreme_wcp[6];
  for (int i=0;i!=6;i++){
    extreme_wcp[i] = cloud.pts[0];
  }

  bool flag_init = false;
  
  for (size_t i=0;i< cloud.pts.size();i++){
    if (excluded_points.find(i)!=excluded_points.end()) continue;
    if (!flag_init){
      flag_init = true;
      for (int j=0;j!=6;j++){
	extreme_wcp[j] = cloud.pts[i];
      }
    }else{
      if (cloud.pts[i].y > extreme_wcp[0].y)
	extreme_wcp[0] = cloud.pts[i];
      if (cloud.pts[i].y < extreme_wcp[1].y)
	extreme_wcp[1] = cloud.pts[i];
      
      if (cloud.pts[i].x > extreme_wcp[2].x)
	extreme_wcp[2] = cloud.pts[i];
      if (cloud.pts[i].x < extreme_wcp[3].x)
	extreme_wcp[3] = cloud.pts[i];
      
      if (cloud.pts[i].z > extreme_wcp[4].z)
	extreme_wcp[4] = cloud.pts[i];
      if (cloud.pts[i].z < extreme_wcp[5].z)
	extreme_wcp[5] = cloud.pts[i];
    }
  }

  double max_dis = -1;
  WCPointCloud<double>::WCPoint wcp1, wcp2;
  for (int i=0;i!=6;i++){
    for (int j=i+1;j!=6;j++){
      double dis = sqrt(pow(extreme_wcp[i].x - extreme_wcp[j].x,2)+pow(extreme_wcp[i].y - extreme_wcp[j].y,2)+pow(extreme_wcp[i].z - extreme_wcp[j].z,2));
      if (dis > max_dis){
	max_dis = dis;
	wcp1 = extreme_wcp[i];
	wcp2 = extreme_wcp[j];
      }
    }
  }
  Point p1(wcp1.x,wcp1.y,wcp1.z);
  Point p2(wcp2.x,wcp2.y,wcp2.z);
  p1 = calc_ave_pos(p1,5*units::cm);
  p2 = calc_ave_pos(p2,5*units::cm);
  
  return std::make_pair(p1,p2);
}


void WCPPID::PR3DCluster::dijkstra_shortest_paths(WCPointCloud<double>::WCPoint& wcp, int flag){
  if (graph==(MCUGraph*)0)
    Create_graph();
  
  //  if (wcp.index==source_wcp_index)
  //  return;
  source_wcp_index = wcp.index;
  parents.clear();
  distances.clear();
  
  if (flag==1){
    parents.resize(num_vertices(*graph));
    distances.resize(num_vertices(*graph));

    //    if (num_vertices(*graph)>1){
    auto v0 = vertex(wcp.index,*graph);
    boost::dijkstra_shortest_paths(*graph, v0,
				   weight_map(get(edge_weight, *graph))
				   .predecessor_map(&parents[0])
				   .distance_map(&distances[0])
				   );
    //}
  }else if (flag==2){
    parents.resize(num_vertices(*graph_steiner));
    distances.resize(num_vertices(*graph_steiner));
    
    //    if (num_vertices(*graph_steiner)>1){
    auto v0 = vertex(wcp.index,*graph_steiner);
    boost::dijkstra_shortest_paths(*graph_steiner, v0,
				   weight_map(get(edge_weight, *graph_steiner))
				   .predecessor_map(&parents[0])
				   .distance_map(&distances[0])
				   );
    //}
  }
}


void WCPPID::PR3DCluster::cal_shortest_path(WCPointCloud<double>::WCPoint& wcp_target, int flag){
  dest_wcp_index = wcp_target.index;
  path_wcps.clear();
  path_mcells.clear();

  ToyPointCloud *temp_point_cloud = point_cloud;
  if (flag==2){
    temp_point_cloud = point_cloud_steiner;
  }
  
  WCP::WCPointCloud<double>& cloud = temp_point_cloud->get_cloud();
  //  std::cout << dest_wcp_index << " " << source_wcp_index << std::endl;
  //  if ( dest_wcp_index != source_wcp_index && cloud.pts.size()>1){
  int prev_i = -1;
  for(int i = dest_wcp_index; i!=source_wcp_index; i = parents[i]) {
    if (path_wcps.size()==0){
      path_wcps.push_front(cloud.pts[i]);
      path_mcells.push_front(cloud.pts[i].mcell);
    }else{
      path_wcps.push_front(cloud.pts[i]);
      if (cloud.pts[i].mcell!=path_mcells.front())
	path_mcells.push_front(cloud.pts[i].mcell);
    }
    if (i==prev_i) break;
    prev_i = i;
  }
  path_wcps.push_front(cloud.pts[source_wcp_index]);
  if (cloud.pts[source_wcp_index].mcell!=path_mcells.front())
    path_mcells.push_front(cloud.pts[source_wcp_index].mcell);
  //}else{
  //  path_wcps.push_front(cloud.pts[source_wcp_index]);
  //  path_mcells.push_front(cloud.pts[source_wcp_index].mcell);
  // }
  parents.clear();
  distances.clear();
}


