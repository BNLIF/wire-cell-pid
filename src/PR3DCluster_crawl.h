#include <algorithm>

void WCPPID::PR3DCluster::do_rough_path(WCP::WCPointCloud<double>::WCPoint& first_wcp, WCP::WCPointCloud<double>::WCPoint& last_wcp){
  path_wcps.clear();
  
  Point test_p;
  // find the corresponding points in the Steiner Tree Point Cloud
  {
    test_p.x = first_wcp.x;
    test_p.y = first_wcp.y;
    test_p.z = first_wcp.z;
    first_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);
    test_p.x = last_wcp.x;
    test_p.y = last_wcp.y;
    test_p.z = last_wcp.z;
    last_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);
  }
  dijkstra_shortest_paths(first_wcp,2);
  cal_shortest_path(last_wcp,2);
}

WCP::Point WCPPID::PR3DCluster::adjust_rough_path(){
  
  Point test_p;

  test_p.x = fine_tracking_path.at(0).x;
  test_p.y = fine_tracking_path.at(0).y;
  test_p.z = fine_tracking_path.at(0).z;

  TVector3 drift_dir(1,0,0);

  int save_i = 0;
  bool flag_crawl = false;

  std::vector<double> refl_angles(fine_tracking_path.size(),0);
  std::vector<double> para_angles(fine_tracking_path.size(),0);
  
  for (size_t i=0;i!=fine_tracking_path.size(); i++){
    
    /* TVector3 v10(0,0,0); */
    /* TVector3 v20(0,0,0); */

    /* TVector3 v11(0,0,0); */
    /* TVector3 v21(0,0,0); */

    /* TVector3 v12(0,0,0); */
    /* TVector3 v22(0,0,0); */
    
    /* if (i>0) */
    /*   v10.SetXYZ(fine_tracking_path.at(i).x - fine_tracking_path.at(i-1).x, */
    /* 		 fine_tracking_path.at(i).y - fine_tracking_path.at(i-1).y, */
    /* 		 fine_tracking_path.at(i).z - fine_tracking_path.at(i-1).z); */
    /* if (i>1) */
    /*   v11.SetXYZ(fine_tracking_path.at(i).x - fine_tracking_path.at(i-2).x, */
    /* 		 fine_tracking_path.at(i).y - fine_tracking_path.at(i-2).y, */
    /* 		 fine_tracking_path.at(i).z - fine_tracking_path.at(i-2).z); */
    /* if (i>2) */
    /*   v12.SetXYZ(fine_tracking_path.at(i).x - fine_tracking_path.at(i-3).x, */
    /* 		 fine_tracking_path.at(i).y - fine_tracking_path.at(i-3).y, */
    /* 		 fine_tracking_path.at(i).z - fine_tracking_path.at(i-3).z); */
    
    /* if (i+1<fine_tracking_path.size()) */
    /*   v20.SetXYZ(fine_tracking_path.at(i+1).x - fine_tracking_path.at(i).x, */
    /* 		 fine_tracking_path.at(i+1).y - fine_tracking_path.at(i).y, */
    /* 		 fine_tracking_path.at(i+1).z - fine_tracking_path.at(i).z); */
    /* if (i+2<fine_tracking_path.size()) */
    /*   v21.SetXYZ(fine_tracking_path.at(i+2).x - fine_tracking_path.at(i).x, */
    /* 		 fine_tracking_path.at(i+2).y - fine_tracking_path.at(i).y, */
    /* 		 fine_tracking_path.at(i+2).z - fine_tracking_path.at(i).z); */
    /* if (i+3<fine_tracking_path.size()) */
    /*   v22.SetXYZ(fine_tracking_path.at(i+3).x - fine_tracking_path.at(i).x, */
    /* 		 fine_tracking_path.at(i+3).y - fine_tracking_path.at(i).y, */
    /* 		 fine_tracking_path.at(i+3).z - fine_tracking_path.at(i).z); */
    
    /* double angle1 = v10.Angle(v20)/3.1415926*180.; */
    /* angle1 = std::min(v11.Angle(v21)/3.1415926*180.,angle1); */
    /* angle1 = std::min(v12.Angle(v22)/3.1415926*180., angle1); */

    /* TVector3 v30 = v10+v20; */
    /* double angle2 = fabs(v30.Angle(drift_dir)/3.1415926*180.-90); */
    /* v30 = v11 + v21; */
    /* angle2 = std::min(fabs(v30.Angle(drift_dir)/3.1415926*180.-90),angle2); */
    /* v30 = v12 + v22; */
    /* angle2 = std::min(fabs(v30.Angle(drift_dir)/3.1415926*180.-90),angle2); */
    

    double angle1 = 0;
    double angle2 = 0;

    for (int j=0;j!=6;j++){    
      TVector3 v10(0,0,0);
      TVector3 v20(0,0,0);
      if (i>j)
	v10.SetXYZ(fine_tracking_path.at(i).x - fine_tracking_path.at(i-j-1).x,
		   fine_tracking_path.at(i).y - fine_tracking_path.at(i-j-1).y,
		   fine_tracking_path.at(i).z - fine_tracking_path.at(i-j-1).z);
      
      if (i+j+1<fine_tracking_path.size())
	v20.SetXYZ(fine_tracking_path.at(i+j+1).x - fine_tracking_path.at(i).x,
		   fine_tracking_path.at(i+j+1).y - fine_tracking_path.at(i).y,
		   fine_tracking_path.at(i+j+1).z - fine_tracking_path.at(i).z);
      
      if (j==0){
	angle1 = v10.Angle(v20)/3.1415926*180.;
	angle2 = std::max(fabs(v10.Angle(drift_dir)/3.1415926*180.-90.),
			  fabs(v20.Angle(drift_dir)/3.1415926*180.-90.));
      }else{
	if (v10.Mag()!=0 && v20.Mag()!=0){
	  angle1 = std::min(v10.Angle(v20)/3.1415926*180., angle1);
	  angle2 = std::min(std::max(fabs(v10.Angle(drift_dir)/3.1415926*180.-90.),
				     fabs(v20.Angle(drift_dir)/3.1415926*180.-90.)),angle2);
	}
      }
    }
    
    refl_angles.at(i) = angle1;
    para_angles.at(i) = angle2;

      //  std::cout << i << " " << angle1 << " " << angle2 << " " << std::endl;
  }


   for (int i=0;i!=fine_tracking_path.size();i++){
     double min_dQ_dx = dQ.at(i)/dx.at(i);
     for (size_t j = 1;j!=6;j++){
       if (i+j<fine_tracking_path.size())
	 if (dQ.at(i+j)/dx.at(i+j) < min_dQ_dx)
	   min_dQ_dx = dQ.at(i+j)/dx.at(i+j);
     }
     
    double sum_angles = 0;
    double nsum = 0;

    for (int j = -2; j!=3;j++){
      if (i+j>=0 && i+j<fine_tracking_path.size()){
	if (para_angles.at(i+j)>10){
	  sum_angles += pow(refl_angles.at(i+j),2);
	  nsum ++;
	}
      }
    }
    if (nsum!=0) sum_angles=sqrt(sum_angles/nsum);
       
                // std::cout << i << " " << min_dQ_dx << " " << para_angles.at(i) << " " << refl_angles.at(i) <<" " << sum_angles << std::endl;

    
    if (min_dQ_dx < 1000 && para_angles.at(i) > 10 && refl_angles.at(i) > 25){
      std::cout << "Mid_Point_Break: " << i << " " << refl_angles.at(i) << " " << para_angles.at(i) << " " << min_dQ_dx << " " << fine_tracking_path.at(i).x << " " << fine_tracking_path.at(i).y << " " << fine_tracking_path.at(i).z << std::endl;
      flag_crawl = true;
      save_i = i;
      break;
    }else if (para_angles.at(i) > 15 && refl_angles.at(i) > 27 && sum_angles > 12.5){
       TVector3 v10(fine_tracking_path.at(i).x - fine_tracking_path.front().x,
		   fine_tracking_path.at(i).y - fine_tracking_path.front().y,
		   fine_tracking_path.at(i).z - fine_tracking_path.front().z);
      TVector3 v20(fine_tracking_path.back().x - fine_tracking_path.at(i).x,
		   fine_tracking_path.back().y - fine_tracking_path.at(i).y,
		   fine_tracking_path.back().z - fine_tracking_path.at(i).z);
      double angle3 = v10.Angle(v20)/3.1415926*180.;
      if (angle3 < 20) continue;
      
      std::cout << "Mid_Point_Break: " << i << " " << refl_angles.at(i) << " " << para_angles.at(i) << " " << angle3 << " " << min_dQ_dx << " " << fine_tracking_path.at(i).x << " " << fine_tracking_path.at(i).y << " " << fine_tracking_path.at(i).z << std::endl;
      flag_crawl = true;
      save_i = i;
      break;
    }
      //
  }


  if (flag_crawl){
    // Start to Crawl
    float step_dis = 1*units::cm;
    
    // initialization
    Point p(fine_tracking_path.at(save_i).x, fine_tracking_path.at(save_i).y, fine_tracking_path.at(save_i).z);
    WCP::WCPointCloud<double>::WCPoint curr_wcp = point_cloud_steiner->get_closest_wcpoint(p);
    WCP::WCPointCloud<double>::WCPoint next_wcp = curr_wcp;
    
    Point prev_p(0,0,0);
    int num_p = 0;
    for (size_t i=1;i!=6;i++){
      if (save_i>=i){
	prev_p.x += fine_tracking_path.at(save_i-i).x;
	prev_p.y += fine_tracking_path.at(save_i-i).y;
	prev_p.z += fine_tracking_path.at(save_i-i).z;
	num_p ++;
      }
    }
    prev_p.x /= num_p;
    prev_p.y /= num_p;
    prev_p.z /= num_p;
    
    TVector3 dir(p.x - prev_p.x, p.y - prev_p.y, p.z - prev_p.z);
    dir = dir.Unit();
    
    bool flag_continue = true;
    while(flag_continue){
      flag_continue = false;
      for (int i=0; i!=3; i++){
	test_p.x = curr_wcp.x + dir.X() * step_dis * (i+1);
	test_p.y = curr_wcp.y + dir.Y() * step_dis * (i+1);
	test_p.z = curr_wcp.z + dir.Z() * step_dis * (i+1);
	
	next_wcp = point_cloud->get_closest_wcpoint(test_p);
	TVector3 dir1(next_wcp.x - curr_wcp.x, next_wcp.y - curr_wcp.y, next_wcp.z - curr_wcp.z);
	
	/* std::cout << dir.X() << " " << dir.Y() << " " << dir.Z() << " " << dir1.X() << " " << dir1.Y() << " " << dir1.Z() << " " << drift_dir.Angle(dir)/3.1415926*180. << " " << drift_dir.Angle(dir1)/3.1415926*180. << std::endl; */
	/* std::cout <<curr_wcp.x << " " << curr_wcp.y << " " << curr_wcp.z << " " << i << " " << dir1.Mag() << " " << dir1.Angle(dir)/3.14151926*180. << " " << test_p << " " << next_wcp.x << " " << next_wcp.y << " " << next_wcp.z << std::endl; */
	
	if (dir1.Mag()!=0 && (dir1.Angle(dir)/3.1415926*180. < 30)){
	  //|| (fabs(drift_dir.Angle(dir)/3.1415926*180.-90.)<10. && fabs(drift_dir.Angle(dir1)/3.1415926*180.-90.)<10.) && dir1.Angle(dir)/3.1415926*180. < 60)){
	  flag_continue = true;
	  curr_wcp = next_wcp;
	  dir = dir1 + dir * 5 * units::cm; // momentum trick ...
	  dir = dir.Unit();
	  break;
	}
	
	next_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);
	TVector3 dir2(next_wcp.x - curr_wcp.x, next_wcp.y - curr_wcp.y, next_wcp.z - curr_wcp.z);
		
	//	std::cout << dir2.Angle(dir)/3.1415926*180. << " " << i << " " << drift_dir.Angle(dir2)/3.1415926*180. << " "<< drift_dir.Angle(dir)/3.1415926*180. << std::endl;
	
	if (dir2.Mag()!=0 && (dir2.Angle(dir)/3.1415926*180. < 30)){
	    //|| (fabs(drift_dir.Angle(dir)/3.1415926*180.-90.)<10. && fabs(drift_dir.Angle(dir2)/3.1415926*180.-90.)<10.) && dir2.Angle(dir)/3.1415926*180. < 60)){
	  flag_continue = true;
	  curr_wcp = next_wcp;
	  dir = dir1 + dir * 5 * units::cm; // momentum trick ...
	  dir = dir.Unit();
	  break;
	}
	
      }
    }
    
    test_p.x = fine_tracking_path.front().x;
    test_p.y = fine_tracking_path.front().y;
    test_p.z = fine_tracking_path.front().z;
    WCP::WCPointCloud<double>::WCPoint& first_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);

    test_p.x = fine_tracking_path.back().x;
    test_p.y = fine_tracking_path.back().y;
    test_p.z = fine_tracking_path.back().z;
    WCP::WCPointCloud<double>::WCPoint& last_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);

    test_p.x = curr_wcp.x;
    test_p.y = curr_wcp.y;
    test_p.z = curr_wcp.z;
    curr_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);
  
    
  
    double dis = sqrt(pow(curr_wcp.x-last_wcp.x,2) + pow(curr_wcp.y-last_wcp.y,2) + pow(curr_wcp.z-last_wcp.z,2));

    std::cout << "First, Center: " << first_wcp.x << " " << first_wcp.y << " " << first_wcp.z << " " << curr_wcp.x << " " << curr_wcp.y << " " << 
    curr_wcp.z << " " << dis/units::cm << std::endl;


    if (dis > 1*units::cm){
      
      dijkstra_shortest_paths(first_wcp,2);
      cal_shortest_path(curr_wcp,2);


      std::list<WCP::WCPointCloud<double>::WCPoint> temp_path_wcps = path_wcps;
      dijkstra_shortest_paths(curr_wcp,2);
      cal_shortest_path(last_wcp,2);
      
      auto it1 = temp_path_wcps.rbegin();
      int count = 0;
      for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
      	if ( (*it).index==(*it1).index){
      	  count ++;
      	  it1++;
      	}
      }

      // std::cout <<  temp_path_wcps.size() << " " << count << " " << path_wcps.size() << std::endl;

      for (int i=0;i!=count;i++){
      	if (i!=count-1){
      	  temp_path_wcps.pop_back();
      	}
      	path_wcps.pop_front();
      }
      
      for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
      	temp_path_wcps.push_back(*it);
      }
      
      path_wcps = temp_path_wcps;
      
      for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
      	//std::cout << (*it).x << " " << (*it).y << " " << (*it).z << std::endl;
	
      	if (path_mcells.size()==0){
      	  path_mcells.push_back( (*it).mcell);
      	}else{
      	  if ( (*it).mcell!=path_mcells.back())
      	    path_mcells.push_back( (*it).mcell);
      	}
      }
      
    }
    
  }
  
  
  return test_p;
}


WCP::Point WCPPID::PR3DCluster::do_stm_crawl(WCP::WCPointCloud<double>::WCPoint& first_wcp, WCP::WCPointCloud<double>::WCPoint& last_wcp, int flag_end){
  path_wcps.clear();
  
  Point test_p;
  // find the corresponding points in the Steiner Tree Point Cloud
  {
    test_p.x = first_wcp.x;
    test_p.y = first_wcp.y;
    test_p.z = first_wcp.z;
    first_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);
    test_p.x = last_wcp.x;
    test_p.y = last_wcp.y;
    test_p.z = last_wcp.z;
    last_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);
  }

  TVector3 drift_dir(1,0,0);
  
  //Crawl in point cloud 
  float step_dis = 1*units::cm;
  
  // initialization
  WCP::WCPointCloud<double>::WCPoint curr_wcp = first_wcp;
  WCP::WCPointCloud<double>::WCPoint next_wcp = first_wcp;
  Point p(first_wcp.x, first_wcp.y, first_wcp.z);
  TVector3 dir = VHoughTrans(p, 15*units::cm); // start direction
  
  
  bool flag_continue = true;
  while(flag_continue){
    flag_continue = false;
    for (int i=0; i!=3; i++){
      test_p.x = curr_wcp.x + dir.X() * step_dis * (i+1);
      test_p.y = curr_wcp.y + dir.Y() * step_dis * (i+1);
      test_p.z = curr_wcp.z + dir.Z() * step_dis * (i+1);

      next_wcp = point_cloud->get_closest_wcpoint(test_p);
      TVector3 dir1(next_wcp.x - curr_wcp.x, next_wcp.y - curr_wcp.y, next_wcp.z - curr_wcp.z);

      std::cout << dir.X() << " " << dir.Y() << " " << dir.Z() << " " << dir1.X() << " " << dir1.Y() << " " << dir1.Z() << " " << drift_dir.Angle(dir)/3.1415926*180. << " " << drift_dir.Angle(dir1)/3.1415926*180. << std::endl;
      std::cout <<curr_wcp.x << " " << curr_wcp.y << " " << curr_wcp.z << " " << i << " " << dir1.Mag() << " " << dir1.Angle(dir)/3.14151926*180. << " " << test_p << " " << next_wcp.x << " " << next_wcp.y << " " << next_wcp.z << std::endl;

      if (dir1.Mag()!=0 && (dir1.Angle(dir)/3.1415926*180. < 30 || (fabs(drift_dir.Angle(dir)/3.1415926*180.-90.)<10. && fabs(drift_dir.Angle(dir1)/3.1415926*180.-90.)<10.) && fabs(drift_dir.Angle(dir)-drift_dir.Angle(dir1))/3.1415926*180. < 20 && dir1.Angle(dir)/3.1415926*180. < 60)){
	flag_continue = true;
	curr_wcp = next_wcp;
      	dir = dir1 + dir * 5 * units::cm; // momentum trick ...
	dir = dir.Unit();
	break;
      }
      
      next_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);
      TVector3 dir2(next_wcp.x - curr_wcp.x, next_wcp.y - curr_wcp.y, next_wcp.z - curr_wcp.z);

      std::cout << dir2.Angle(dir)/3.1415926*180. << " " << i << " " << drift_dir.Angle(dir2)/3.1415926*180. << " "<< drift_dir.Angle(dir)/3.1415926*180. << std::endl;
      
      if (dir2.Mag()!=0 && (dir2.Angle(dir)/3.1415926*180. < 30 || (fabs(drift_dir.Angle(dir)/3.1415926*180.-90.)<10. && fabs(drift_dir.Angle(dir2)/3.1415926*180.-90.)<10.) && fabs(drift_dir.Angle(dir) - drift_dir.Angle(dir2))/3.1415926*180. < 20 && dir2.Angle(dir)/3.1415926*180. < 60)){
	flag_continue = true;
	curr_wcp = next_wcp;
      	dir = dir1 + dir * 5 * units::cm; // momentum trick ...
	dir = dir.Unit();
	break;
      }
      
    }
  }
  test_p.x = curr_wcp.x;
  test_p.y = curr_wcp.y;
  test_p.z = curr_wcp.z;
  curr_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);

  
  //std::cout << first_wcp.x << " " << first_wcp.y << " " << first_wcp.z << " " << curr_wcp.x << " " << curr_wcp.y << " " << curr_wcp.z << std::endl;

  double dis = sqrt(pow(curr_wcp.x-last_wcp.x,2) + pow(curr_wcp.y-last_wcp.y,2) + pow(curr_wcp.z-last_wcp.z,2));
  // Find the shortest path between first and middle point
  
  // If middle point is far away from last_wcp, find the shortest path
  if (dis < 1*units::cm){
    
    dijkstra_shortest_paths(first_wcp,2);
    cal_shortest_path(last_wcp,2);

  }else{
    //    double dis1 = sqrt(pow(curr_wcp.x - last_wcp.x,2)+pow(curr_wcp.y - last_wcp.y,2)+pow(curr_wcp.z - last_wcp.z,2));
    //    double dis2 = sqrt(pow(first_wcp.x - last_wcp.x,2)+pow(first_wcp.y - last_wcp.y,2)+pow(first_wcp.z - last_wcp.z,2));

    std::cout << first_wcp.x << " " << first_wcp.y << " " << first_wcp.z << std::endl;
    std::cout << curr_wcp.x << " " << curr_wcp.y << " " << curr_wcp.z << std::endl;
    std::cout << last_wcp.x << " " << last_wcp.y << " " << last_wcp.z << std::endl;
    // std::cout << dis1 << " " << dis2 << std::endl;

    dijkstra_shortest_paths(first_wcp,2);
    cal_shortest_path(curr_wcp,2);

    
    if (flag_end == 1 ){
      std::list<WCP::WCPointCloud<double>::WCPoint> temp_path_wcps = path_wcps;
      
      dijkstra_shortest_paths(curr_wcp,2);
      cal_shortest_path(last_wcp,2);
      
      auto it1 = temp_path_wcps.rbegin();
      int count = 0;
      for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
	if ( (*it).index==(*it1).index){
	  count ++;
	  it1++;
	}
      }
      for (int i=0;i!=count;i++){
	if (i!=count-1){
	  temp_path_wcps.pop_back();
	}
	path_wcps.pop_front();
      }
      
      for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
	temp_path_wcps.push_back(*it);
      }
      
      path_wcps = temp_path_wcps;
     
      for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
	//std::cout << (*it).x << " " << (*it).y << " " << (*it).z << std::endl;
	
	if (path_mcells.size()==0){
	  path_mcells.push_back( (*it).mcell);
	}else{
	  if ( (*it).mcell!=path_mcells.back())
	    path_mcells.push_back( (*it).mcell);
	}
      }
    }
    
  }
       
  // for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
  //  std::cout << (*it).x << " " << (*it).y << " " << (*it).z << std::endl;
  // }
  
  test_p.x = curr_wcp.x;
  test_p.y = curr_wcp.y;
  test_p.z = curr_wcp.z;
  
  return test_p;
  
 
}
