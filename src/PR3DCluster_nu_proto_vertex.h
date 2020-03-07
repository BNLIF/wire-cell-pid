bool WCPPID::PR3DCluster::proto_break_tracks(WCP::WCPointCloud<double>::WCPoint& first_wcp, WCP::WCPointCloud<double>::WCPoint& curr_wcp, WCP::WCPointCloud<double>::WCPoint& last_wcp, std::list<WCP::WCPointCloud<double>::WCPoint>& wcps_list1, std::list<WCP::WCPointCloud<double>::WCPoint>& wcps_list2){

  double dis1 = sqrt(pow(curr_wcp.x-first_wcp.x,2) + pow(curr_wcp.y-first_wcp.y,2) + pow(curr_wcp.z-first_wcp.z,2));
  double dis2 = sqrt(pow(curr_wcp.x-last_wcp.x,2) + pow(curr_wcp.y-last_wcp.y,2) + pow(curr_wcp.z-last_wcp.z,2));

  //std::cout << dis1/units::cm << " " << dis2/units::cm << std::endl;
  
  if (dis1 > 1*units::cm && dis2 > 1*units::cm){
    dijkstra_shortest_paths(first_wcp,2);
    cal_shortest_path(curr_wcp,2);

    wcps_list1 = path_wcps;

    dijkstra_shortest_paths(curr_wcp,2);
    cal_shortest_path(last_wcp,2);
    wcps_list2 = path_wcps;

    
    auto it1 = wcps_list1.rbegin();
    int count = 0;
    for (auto it = wcps_list2.begin(); it!=wcps_list2.end(); it++){
      if ( (*it).index==(*it1).index){
	count ++;
	it1++;
      }
    }
    for (int i=0;i!=count;i++){
      if (i!=count-1){
	wcps_list1.pop_back();
	wcps_list2.pop_front();
      }
    }
    
    
    return true;
  }else {
    return false;
  }
}



WCP::WCPointCloud<double>::WCPoint WCPPID::PR3DCluster::proto_extend_point(WCP::Point& p, TVector3& dir, bool flag_continue){

  float step_dis = 1*units::cm;
  
  WCP::WCPointCloud<double>::WCPoint curr_wcp = point_cloud_steiner->get_closest_wcpoint(p);
  WCP::WCPointCloud<double>::WCPoint next_wcp = curr_wcp;

  WCP::Point test_p;
  
  while(flag_continue){
    flag_continue = false;
    
    for (int i=0; i!=3; i++){
      test_p.x = curr_wcp.x + dir.X() * step_dis * (i+1);
      test_p.y = curr_wcp.y + dir.Y() * step_dis * (i+1);
      test_p.z = curr_wcp.z + dir.Z() * step_dis * (i+1);


      next_wcp = point_cloud_steiner->get_closest_wcpoint(test_p);
      TVector3 dir2(next_wcp.x - curr_wcp.x, next_wcp.y - curr_wcp.y, next_wcp.z - curr_wcp.z);
      
      // std::cout << dir2.Angle(dir)/3.1415926*180. << " " << i << " " << std::endl;
      
      if (dir2.Mag()!=0 && (dir2.Angle(dir)/3.1415926*180. < 25)){
	//|| (fabs(drift_dir.Angle(dir)/3.1415926*180.-90.)<10. && fabs(drift_dir.Angle(dir2)/3.1415926*180.-90.)<10.) && dir2.Angle(dir)/3.1415926*180. < 60)){
	flag_continue = true;
	curr_wcp = next_wcp;
	dir = dir2 + dir * 5 * units::cm; // momentum trick ...
	dir = dir.Unit();
	break;
      }
            
      next_wcp = point_cloud->get_closest_wcpoint(test_p);
      TVector3 dir1(next_wcp.x - curr_wcp.x, next_wcp.y - curr_wcp.y, next_wcp.z - curr_wcp.z);
      
      /* std::cout << dir.X() << " " << dir.Y() << " " << dir.Z() << " " << dir1.X() << " " << dir1.Y() << " " << dir1.Z() << " " << drift_dir.Angle(dir)/3.1415926*180. << " " << drift_dir.Angle(dir1)/3.1415926*180. << std::endl; */
      // std::cout <<curr_wcp.x << " " << curr_wcp.y << " " << curr_wcp.z << " " << i << " " << dir1.Mag() << " " << dir1.Angle(dir)/3.14151926*180. << " " << test_p << " " << next_wcp.x << " " << next_wcp.y << " " << next_wcp.z << std::endl; 
      
      if (dir1.Mag()!=0 && (dir1.Angle(dir)/3.1415926*180. < 17.5)){
      	//|| (fabs(drift_dir.Angle(dir)/3.1415926*180.-90.)<10. && fabs(drift_dir.Angle(dir1)/3.1415926*180.-90.)<10.) && dir1.Angle(dir)/3.1415926*180. < 60)){
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

  return curr_wcp;
  
}
    

void WCPPID::PR3DCluster::set_fit_parameters(std::map<WCPPID::ProtoVertex*, WCPPID::ProtoSegmentSet>& map_vertex_segments, std::map<WCPPID::ProtoSegment*, WCPPID::ProtoVertexSet>& map_segment_vertices){
  fine_tracking_path.clear();
  dQ.clear();
  dx.clear();
  pu.clear();
  pv.clear();
  pw.clear();
  pt.clear();
  reduced_chi2.clear();

  for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end(); it++){
    fine_tracking_path.push_back((it->first)->get_fit_pt());
    dQ.push_back((it->first)->get_dQ());
    dx.push_back((it->first)->get_dx());
    pu.push_back((it->first)->get_pu());
    pv.push_back((it->first)->get_pv());
    pw.push_back((it->first)->get_pw());
    pt.push_back((it->first)->get_pt());
    reduced_chi2.push_back((it->first)->get_reduced_chi2());

    flag_vertex.push_back(true);
    sub_cluster_id.push_back(-1);
  }

  int tmp_id = cluster_id*1000 + 1;
  for (auto it=map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    fine_tracking_path.insert(fine_tracking_path.end(),(it->first)->get_point_vec().begin(), (it->first)->get_point_vec().end());
    dQ.insert(dQ.end(),(it->first)->get_dQ_vec().begin(), (it->first)->get_dQ_vec().end());
    dx.insert(dx.end(),(it->first)->get_dx_vec().begin(), (it->first)->get_dx_vec().end());
    pu.insert(pu.end(),(it->first)->get_pu_vec().begin(), (it->first)->get_pu_vec().end());
    pv.insert(pv.end(),(it->first)->get_pv_vec().begin(), (it->first)->get_pv_vec().end());
    pw.insert(pw.end(),(it->first)->get_pw_vec().begin(), (it->first)->get_pw_vec().end());
    pt.insert(pt.end(),(it->first)->get_pt_vec().begin(), (it->first)->get_pt_vec().end());
    reduced_chi2.insert(reduced_chi2.end(),(it->first)->get_reduced_chi2_vec().begin(), (it->first)->get_reduced_chi2_vec().end());
    for (size_t i=0;i!=(it->first)->get_point_vec().size();i++){
      flag_vertex.push_back(false);
      sub_cluster_id.push_back(tmp_id);
    }
    tmp_id ++;
  }
  //
}