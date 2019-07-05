std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> WireCellPID::PR3DCluster::get_highest_lowest_wcps(){
  WireCell::WCPointCloud<double>& cloud = point_cloud->get_cloud();
  WCPointCloud<double>::WCPoint highest_wcp = cloud.pts[0];
  WCPointCloud<double>::WCPoint lowest_wcp = cloud.pts[0];
  for (size_t i=1;i<cloud.pts.size();i++){
    if (cloud.pts[i].y > highest_wcp.y)
      highest_wcp = cloud.pts[i];
    if (cloud.pts[i].y < lowest_wcp.y)
      lowest_wcp = cloud.pts[i];
  }
  return std::make_pair(highest_wcp,lowest_wcp);
}

std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> WireCellPID::PR3DCluster::get_front_back_wcps(){
  WireCell::WCPointCloud<double>& cloud = point_cloud->get_cloud();
  WCPointCloud<double>::WCPoint highest_wcp = cloud.pts[0];
  WCPointCloud<double>::WCPoint lowest_wcp = cloud.pts[0];
  for (size_t i=1;i<cloud.pts.size();i++){
    if (cloud.pts[i].z > highest_wcp.z)
      highest_wcp = cloud.pts[i];
    if (cloud.pts[i].z < lowest_wcp.z)
      lowest_wcp = cloud.pts[i];
  }
  return std::make_pair(highest_wcp,lowest_wcp);
}


std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> WireCellPID::PR3DCluster::get_earliest_latest_wcps(){
  WireCell::WCPointCloud<double>& cloud = point_cloud->get_cloud();
  WCPointCloud<double>::WCPoint highest_wcp = cloud.pts[0];
  WCPointCloud<double>::WCPoint lowest_wcp = cloud.pts[0];
  for (size_t i=1;i<cloud.pts.size();i++){
    if (cloud.pts[i].x > highest_wcp.x)
      highest_wcp = cloud.pts[i];
    if (cloud.pts[i].x < lowest_wcp.x)
      lowest_wcp = cloud.pts[i];
  }
  return std::make_pair(lowest_wcp,highest_wcp);
}

std::vector<std::vector<WCPointCloud<double>::WCPoint>> WireCellPID::PR3DCluster::get_extreme_wcps(){
  
  WireCell::WCPointCloud<double>& cloud = point_cloud->get_cloud();
  Calc_PCA();
  WCPointCloud<double>::WCPoint wcps[8];
  for (int i=0;i!=8;i++){
    wcps[i] = cloud.pts[0];
  }
  Vector main_axis = get_PCA_axis(0);
  if (main_axis.y <0){
    main_axis.x = -main_axis.x;
    main_axis.y = -main_axis.y;
    main_axis.z = -main_axis.z;
  }
  double high_value = wcps[0].x*main_axis.x + wcps[0].y*main_axis.y + wcps[0].z*main_axis.z;
  double low_value = wcps[1].x * main_axis.x + wcps[1].y * main_axis.y + wcps[1].z * main_axis.z ;

  for (size_t i=1;i<cloud.pts.size();i++){
    double value = cloud.pts[i].x*main_axis.x + cloud.pts[i].y*main_axis.y + cloud.pts[i].z*main_axis.z;
    if (value > high_value){
      wcps[0] = cloud.pts[i];
      high_value = value;
    }
    if (value < low_value){
      wcps[1] = cloud.pts[i];
      low_value = value;
    }
    // top down
    if (cloud.pts[i].y > wcps[2].y)
      wcps[2] = cloud.pts[i];
    if (cloud.pts[i].y < wcps[3].y)
      wcps[3] = cloud.pts[i];
    
    // front back
    if (cloud.pts[i].z > wcps[4].z)
      wcps[4] = cloud.pts[i];
    if (cloud.pts[i].z < wcps[5].z)
      wcps[5] = cloud.pts[i];
    
    // early late
    if (cloud.pts[i].x > wcps[6].x)
      wcps[6] = cloud.pts[i];
    if (cloud.pts[i].x < wcps[7].x)
      wcps[7] = cloud.pts[i];
  }

  
  

  
  std::vector<std::vector<WCPointCloud<double>::WCPoint>> out_vec_wcps;

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
  
  // std::cout << std::endl;
  for (int i=2;i!=8;i++){
    //if (cluster_id==16)
    //std::cout << i << " C " << wcps[i].x/units::cm << " " << wcps[i].y/units::cm << " " << wcps[i].z/units::cm << std::endl;
    
    bool flag_save = true;
    for (size_t j=0;j!=out_vec_wcps.size(); j++){
      double dis = sqrt(pow(out_vec_wcps[j].at(0).x-wcps[i].x,2) + pow(out_vec_wcps[j].at(0).y - wcps[i].y,2) + pow(out_vec_wcps[j].at(0).z - wcps[i].z,2));
      if (dis < 5*units::cm){
	out_vec_wcps.at(j).push_back(wcps[i]);
	flag_save = false;
	break;
      }
    }
    
    if (flag_save){
      std::vector<WCPointCloud<double>::WCPoint> saved_wcps;
      saved_wcps.push_back(wcps[i]);
      out_vec_wcps.push_back(saved_wcps);
    }
  }

  return out_vec_wcps;
  
  // WCPointCloud<double>::WCPoint max_wcps = saved_wcps.at(0);
  // int max_count = counters.at(0);
  // WCPointCloud<double>::WCPoint min_wcps;
  // double value = -1e9;
  
  // for (size_t i=1;i!=saved_wcps.size();i++){
  //   if (counters.at(i)>max_count){
  //     max_wcps = saved_wcps.at(i);
  //     max_count = counters.at(i);
  //   }
  // }

  // //  TVector3 m_pca(main_axis.x, main_axis.y, main_axis.z);
  // Point p1(max_wcps.x,max_wcps.y,max_wcps.z);
  // TVector3 m_dir = VHoughTrans(p1,30*units::cm);
    
  // for (size_t i=0;i!=saved_wcps.size();i++){
  //   TVector3 dir(saved_wcps.at(i).x-max_wcps.x,
  // 		 saved_wcps.at(i).y-max_wcps.y,
  // 		 saved_wcps.at(i).z-max_wcps.z);
  //   double l1 = fabs(dir.Dot(m_dir)/m_dir.Mag());
  //   TVector3 dir1 = dir.Cross(m_dir);
  //   double l2 = dir1.Mag()/m_dir.Mag();
  //   if (l1-l2 > value){
  //     value = l1-l2;
  //     min_wcps = saved_wcps.at(i);
  //   }
  //   //  std::cout << i << " " << saved_wcps.at(i).x/units::cm << " " << saved_wcps.at(i).y/units::cm << " " << saved_wcps.at(i).z/units::cm << " " << l1/units::cm << " " << l2/units::cm << std::endl;
    
  // }
  
  //return std::make_pair(max_wcps,min_wcps);
  
  //  std::cout << max_wcps.x/units::cm << " " << max_wcps.y/units::cm << " " << max_wcps.z/units::cm << " " << max_count << std::endl;

  //  return std::make_pair(wcps[0],wcps[1]);
  
}

std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> WireCellPID::PR3DCluster::get_main_axis_wcps(){
  WireCell::WCPointCloud<double>& cloud = point_cloud->get_cloud();
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
  
  for (size_t i=1;i<cloud.pts.size();i++){
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
  return std::make_pair(highest_wcp,lowest_wcp);
  
}


std::pair<Point,Point> WireCellPID::PR3DCluster::get_two_extreme_points(){
  Create_point_cloud();
  WireCell::WCPointCloud<double>& cloud = point_cloud->get_cloud();
  WCPointCloud<double>::WCPoint extreme_wcp[6];
  for (int i=0;i!=6;i++){
    extreme_wcp[i] = cloud.pts[0];
  }
  for (size_t i=1;i< cloud.pts.size();i++){
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
