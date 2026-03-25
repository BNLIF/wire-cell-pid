
bool WCPPID::NeutrinoID::examine_structure_final(WCPPID::PR3DCluster* temp_cluster){
  
  // results need to repeat the fit and the PID related work ...
  examine_structure_final_1(temp_cluster);
  examine_structure_final_1p(temp_cluster);
  
  examine_structure_final_2(temp_cluster);

  
  examine_structure_final_3(temp_cluster);
  return true;
}

bool WCPPID::NeutrinoID::examine_structure_final_3(WCPPID::PR3DCluster* temp_cluster){
  bool flag_update;

  WCP::ToyPointCloud *pcloud = temp_cluster->get_point_cloud_steiner();
  bool flag_updated = false;

  if (main_vertex !=0 && main_vertex->get_cluster_id() == temp_cluster->get_cluster_id()){
    bool flag_continue = true;
    while(flag_continue){
      flag_continue = false;
      flag_update = false;
      
      for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end(); 
it++){
        WCPPID::ProtoSegment *sg = *it;
        WCPPID::ProtoVertex *vtx1 = find_other_vertex(sg, main_vertex);
	
        if (map_vertex_segments[vtx1].size()==1) continue;
        double dis = sqrt(pow(main_vertex->get_fit_pt().x - vtx1->get_fit_pt().x,2) + pow(main_vertex->get_fit_pt().y - vtx1->get_fit_pt().y, 2) + pow(main_vertex->get_fit_pt().z - vtx1->get_fit_pt().z,2));

	
	
        if (dis <2.5*units::cm){
          // check to see if the vtx1 can be merged into main_vertex ...
	  flag_update = true;
	  for (auto it1 = map_vertex_segments[main_vertex].begin(); it1 != map_vertex_segments[main_vertex].end(); it1++){
	    WCPPID::ProtoSegment *sg1 =  *it1;
	    if (sg == sg1) continue;
	    PointVector& pts = sg1->get_point_vec();
            bool flag_start;
	    if (sg->get_wcpt_vec().front().index == main_vertex->get_wcpt().index){
              flag_start = true;
            }else if (sg->get_wcpt_vec().back().index == main_vertex->get_wcpt().index){
              flag_start = false;
            }
	    
	    Point min_point = pts.front();
            double min_dis = 1e9;
            int min_index;
            for (size_t i=0;i!=pts.size();i++){
              double dis = fabs(sqrt(pow(pts.at(i).x - main_vertex->get_fit_pt().x,2) + pow(pts.at(i).y - main_vertex->get_fit_pt().y,2) + pow(pts.at(i).z - main_vertex->get_fit_pt().z,2))-3*units::cm);
              if (dis < min_dis){
                min_dis = dis;
                min_point = pts.at(i);
                min_index = i;
              }
            }

	    bool flag_connect = true;
            if (flag_start){
              for (int i = min_index; i>=0;i--){
                if (!ct_point_cloud->is_good_point(pts.at(i), 0.2*units::cm, 0, 0)) {
                  flag_connect = false;
                  break;
                }
              }
            }else{
              for (int i = min_index; i<pts.size(); i++){
                if (!ct_point_cloud->is_good_point(pts.at(i), 0.2*units::cm, 0, 0)) {
                  flag_connect = false;
                  break;
                }
              }     
            }
	    if (flag_connect){
              double step_size = 0.3*units::cm;
              Point start_p = min_point;
              Point end_p = vtx1->get_fit_pt();
              int ncount = std::round(sqrt(pow(start_p.x - end_p.x,2) + pow(start_p.y - end_p.y,2) + pow(start_p.z - end_p.z,2))/step_size);
              int n_bad = 0;
              for (int i=1;i<ncount;i++){
                Point test_p(start_p.x + (end_p.x - start_p.x)/ncount*i,
                             start_p.y + (end_p.y - start_p.y)/ncount*i,
                             start_p.z + (end_p.z - start_p.z)/ncount*i);
                if (!ct_point_cloud->is_good_point(test_p, 0.3*units::cm, 0, 0)) {
                  n_bad ++;
                }
		//	std::cout << i << " " << test_p << " " << ct_point_cloud->is_good_point(test_p, 0.2*units::cm, 0, 0) << " " << ct_point_cloud->is_good_point(test_p, 0.3*units::cm, 0, 0) << std::endl;
              }
              if (n_bad >0) flag_update = false;
	      //  std::cout << sg1->get_id() << " " << n_bad << " " << ncount << std::endl;
            }
	  } // loop over segment around main_vertex

	  //	  std::cout << temp_cluster->get_cluster_id() << " " << flag_update << " " << map_vertex_segments[main_vertex].size() << " " << map_vertex_segments[vtx1].size() << " " << dis/units::cm << " " << vtx1->get_id() << " " << main_vertex->get_id() << std::endl;
	  
	  if (flag_update){
	    std::cout << "[examine_structure_final_3] Cluster: " << temp_cluster->get_cluster_id() << " Final stage merge main_vertex " << main_vertex->get_id() << " " << main_vertex->get_fit_pt() << " into vtx " << vtx1->get_id() <<  " " << vtx1->get_fit_pt() << " -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;
	    
	    for (auto it1 = map_vertex_segments[main_vertex].begin(); it1 != map_vertex_segments[main_vertex].end(); it1++) { 
              WCPPID::ProtoSegment *sg1 = *it1;
              if (sg1 == sg) continue;
              WCP::WCPointCloud<double>::WCPoint& vtx_new_wcp = vtx1->get_wcpt();
              std::vector<WCP::WCPointCloud<double>::WCPoint >& vec_wcps = sg1->get_wcpt_vec();
              bool flag_front = false;
              if (vec_wcps.front().index == main_vertex->get_wcpt().index) flag_front = true;
              
              WCP::WCPointCloud<double>::WCPoint min_wcp;
              double min_dis = 1e9;
              double max_dis = std::max(sqrt(pow(vec_wcps.front().x -main_vertex->get_fit_pt().x,2)+pow(vec_wcps.front().y - main_vertex->get_fit_pt().y,2)+pow(vec_wcps.front().z - main_vertex->get_fit_pt().z,2)), sqrt(pow(vec_wcps.back().x - main_vertex->get_fit_pt().x,2)+pow(vec_wcps.back().y - main_vertex->get_fit_pt().y,2)+pow(vec_wcps.back().z - main_vertex->get_fit_pt().z,2)));
	      for (size_t j=0;j!=vec_wcps.size();j++){
                double dis1 = sqrt(pow(vec_wcps.at(j).x -main_vertex->get_fit_pt().x,2)+pow(vec_wcps.at(j).y -main_vertex->get_fit_pt().y,2)+pow(vec_wcps.at(j).z - main_vertex->get_fit_pt().z,2));
                double dis = fabs(dis1 - 3.0*units::cm);
                if (dis < min_dis){
                  min_wcp = vec_wcps.at(j);
                  min_dis = dis;
                }
              }
              // establish the shortest path ...
	      std::list<WCP::WCPointCloud<double>::WCPoint> new_list;
              new_list.push_back(vtx_new_wcp);
              {
                double dis_step = 1.0*units::cm;
                int ncount = std::round(sqrt(pow(vtx_new_wcp.x - min_wcp.x,2) + pow(vtx_new_wcp.y - min_wcp.y,2) + pow(vtx_new_wcp.z - min_wcp.z,2))/dis_step);
                if (ncount <2) ncount = 2;
                for (int qx=1;qx<ncount;qx++){
                  Point tmp_p(vtx_new_wcp.x + (min_wcp.x-vtx_new_wcp.x)/ncount*qx, vtx_new_wcp.y + (min_wcp.y-vtx_new_wcp.y)/ncount*qx, vtx_new_wcp.z + (min_wcp.z-vtx_new_wcp.z)/ncount*qx);
                  WCP::WCPointCloud<double>::WCPoint& tmp_wcp = pcloud->get_closest_wcpoint(tmp_p);
                  //std::cout << qx << " " << sqrt(pow(tmp_wcp.x - tmp_p.x,2) + pow(tmp_wcp.y - tmp_p.y,2) + pow(tmp_wcp.z - tmp_p.z,2))/units::cm << std::endl;
                  if (sqrt(pow(tmp_wcp.x - tmp_p.x,2) + pow(tmp_wcp.y - tmp_p.y,2) + pow(tmp_wcp.z - tmp_p.z,2)) > 0.3*units::cm) continue; // too large ...
                  if (tmp_wcp.index != new_list.back().index && tmp_wcp.index != min_wcp.index)
                    new_list.push_back(tmp_wcp);
                }
	      }
              new_list.push_back(min_wcp);
	      std::list<WCP::WCPointCloud<double>::WCPoint> old_list;
              std::copy(vec_wcps.begin(), vec_wcps.end(), std::back_inserter(old_list));
              // append the list back ...
              //std::cout << min_wcp.index << " " << temp_cluster->get_path_wcps().size() << std::endl;
              if (flag_front){
                while(old_list.front().index != min_wcp.index && old_list.size()>0 ){
                  old_list.pop_front();
                }
                old_list.pop_front();
                //      std::cout << "f2 " << old_list.size() << std::endl;
                for (auto it = new_list.rbegin(); it!=new_list.rend(); it++){
                  old_list.push_front(*it);
		}
		 //      std::cout << "f3 " << old_list.size() << std::endl;
              }else{
                //      std::cout << "b1 " << old_list.size() << std::endl;
                while(old_list.back().index != min_wcp.index && old_list.size()>0 ){
                  old_list.pop_back();
                }
                old_list.pop_back();
                //      std::cout << "b2 " << old_list.size() << std::endl;
                for (auto it = new_list.rbegin(); it!=new_list.rend(); it++){
                  old_list.push_back(*it);
                }
                //      std::cout << "b3 " << old_list.size() << std::endl;
              }
              vec_wcps.clear();
              vec_wcps.reserve(old_list.size());
              std::copy(std::begin(old_list), std::end(old_list), std::back_inserter(vec_wcps));
              

            }
	    main_vertex->set_wcpt(vtx1->get_wcpt());

	    for (auto it1 = map_vertex_segments[vtx1].begin(); it1 != map_vertex_segments[vtx1].end(); it1++){
	      WCPPID::ProtoSegment *sg1 = (*it1);
	      if (sg1 == sg) continue;

	      WCPPID::ProtoVertex *tt_vtx = find_other_vertex(sg1, vtx1);
	      if (tt_vtx != main_vertex){
		add_proto_connection(main_vertex, sg1, temp_cluster);
	      }else{
		del_proto_segment(sg1);
	      }
	    }
	    del_proto_vertex(vtx1);
	    del_proto_segment(sg);
	    break;
	  }
	} // distance is close
      } // loop over segment of main vertices ...
    } // while continue
    
    if (flag_update){
      flag_continue = true;
      flag_updated = true;
      temp_cluster->do_multi_tracking(map_vertex_segments, map_segment_vertices, *ct_point_cloud, global_wc_map, flash_time*units::microsecond, true, true, true);
    }
  }
  
  
  return flag_updated;
}


bool WCPPID::NeutrinoID::examine_structure_final_2(WCPPID::PR3DCluster* temp_cluster){
  bool flag_update;
  WCP::ToyPointCloud *pcloud = temp_cluster->get_point_cloud_steiner();
  bool flag_updated = false;
  
  if (main_vertex !=0 && main_vertex->get_cluster_id() == temp_cluster->get_cluster_id()){
    bool flag_continue = true;
    while(flag_continue){
      flag_continue = false;
      flag_update = false;
      for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end(); it++){
	WCPPID::ProtoSegment *sg = *it;
	WCPPID::ProtoVertex *vtx1 = find_other_vertex(sg, main_vertex);
	//	std::cout << vtx1 << " " << main_vertex << " " << sg->get_id() << " " << map_segment_vertices[sg].size() << std::endl;

	if (map_vertex_segments[vtx1].size()==1 || map_vertex_segments[main_vertex].size()==1) continue;
	double dis = sqrt(pow(main_vertex->get_fit_pt().x - vtx1->get_fit_pt().x,2) + pow(main_vertex->get_fit_pt().y - vtx1->get_fit_pt().y, 2) + pow(main_vertex->get_fit_pt().z - vtx1->get_fit_pt().z,2));

	if (dis <2.0*units::cm){
	  // check to see if the vtx1 can be merged into main_vertex ...
	  flag_update = true;
	  for (auto it1 = map_vertex_segments[vtx1].begin(); it1 != map_vertex_segments[vtx1].end(); it1++){
	    WCPPID::ProtoSegment *sg1 = *it1;
	    if (sg == sg1) continue;
	    PointVector& pts = sg1->get_point_vec();
	    bool flag_start;
	    if (sg->get_wcpt_vec().front().index == vtx1->get_wcpt().index){
	      flag_start = true;
	    }else if (sg->get_wcpt_vec().back().index == vtx1->get_wcpt().index){
	      flag_start = false;
	    }
	    
	    Point min_point = pts.front();
	    double min_dis = 1e9;
	    int min_index;
	    for (size_t i=0;i!=pts.size();i++){
	      double dis = fabs(sqrt(pow(pts.at(i).x - vtx1->get_fit_pt().x,2) + pow(pts.at(i).y - vtx1->get_fit_pt().y,2) + pow(pts.at(i).z - vtx1->get_fit_pt().z,2))-3*units::cm);
	      if (dis < min_dis){
		min_dis = dis;
		min_point = pts.at(i);
		min_index = i;
	      }
	    }
	    
	    bool flag_connect = true;
	    if (flag_start){
	      for (int i = min_index; i>=0;i--){
		if (!ct_point_cloud->is_good_point(pts.at(i), 0.2*units::cm, 0, 0)) {
		  flag_connect = false;
		  break;
		}
	      }
	    }else{
	      for (int i = min_index; i<pts.size(); i++){
		if (!ct_point_cloud->is_good_point(pts.at(i), 0.2*units::cm, 0, 0)) {
		  flag_connect = false;
		  break;
		}
	      }	    
	    }

	    // std::cout << min_index << " " << pts.size() << " " << min_point << " " << flag_connect << std::endl;
	    
	    if (flag_connect){
	      double step_size = 0.3  *units::cm;
	      Point start_p = min_point;
	      Point end_p = main_vertex->get_fit_pt();
	      int ncount = std::round(sqrt(pow(start_p.x - end_p.x,2) + pow(start_p.y - end_p.y,2) + pow(start_p.z - end_p.z,2))/step_size);
	      int n_bad = 0;
	      for (int i=1;i<ncount;i++){
		Point test_p(start_p.x + (end_p.x - start_p.x)/ncount*i,
			     start_p.y + (end_p.y - start_p.y)/ncount*i,
			     start_p.z + (end_p.z - start_p.z)/ncount*i);
		if (!ct_point_cloud->is_good_point(test_p, 0.2*units::cm, 0, 0)) {
		  n_bad ++;
		}
	      }
	      if (n_bad >0) flag_update = false;
	      // std::cout << flag_connect << " " << n_bad << " " << start_p << " " << end_p << " " << ncount << std::endl;
	    }
	  } // loop segments connecting to vtx1

	  //	  std::cout << flag_update << " " << vtx1->get_id() << " " << vtx1->get_fit_pt() << " " << main_vertex->get_id()  << " " << main_vertex->get_fit_pt() << std::endl;

	  // sg needs to be solid in all three views ...
	  if ((!flag_update) && map_vertex_segments[vtx1].size()==2){
	    PointVector& tmp_pts = sg->get_point_vec();
	    for (size_t i=0;i!= tmp_pts.size(); i++){
	      Point test_p = tmp_pts.at(i);
	      if (!ct_point_cloud->is_good_point(test_p, 0.2*units::cm, 0, 0)) flag_update = true;
	      if (i+1 != tmp_pts.size()){
		test_p.x += (tmp_pts.at(i+1).x - tmp_pts.at(i).x)/2.;
		test_p.y += (tmp_pts.at(i+1).y - tmp_pts.at(i).y)/2.;
		test_p.z += (tmp_pts.at(i+1).z - tmp_pts.at(i).z)/2.;
		if (!ct_point_cloud->is_good_point(test_p, 0.2*units::cm, 0, 0)) flag_update = true;
	      }
	    }
	  }

	  
	  if (flag_update){
	    std::cout << "[examine_structure_final_2] Cluster: " << temp_cluster->get_cluster_id() << " Final stage merge vertex " << vtx1->get_id() << " " << vtx1->get_fit_pt() << " to main_vertex " << main_vertex->get_id()  << " " << main_vertex->get_fit_pt() << " -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;
	    for (auto it1 = map_vertex_segments[vtx1].begin(); it1 != map_vertex_segments[vtx1].end(); it1++){
	      WCPPID::ProtoSegment *sg1 = *it1;
	      if (sg1 == sg) continue;
	      WCP::WCPointCloud<double>::WCPoint& vtx_new_wcp = main_vertex->get_wcpt();
	      std::vector<WCP::WCPointCloud<double>::WCPoint >& vec_wcps = sg1->get_wcpt_vec();
	      bool flag_front = false;
	      if (vec_wcps.front().index == vtx1->get_wcpt().index) flag_front = true;
	      
	      WCP::WCPointCloud<double>::WCPoint min_wcp;
	      double min_dis = 1e9;
	      double max_dis = std::max(sqrt(pow(vec_wcps.front().x -vtx1->get_fit_pt().x,2)+pow(vec_wcps.front().y -vtx1->get_fit_pt().y,2)+pow(vec_wcps.front().z -vtx1->get_fit_pt().z,2)), sqrt(pow(vec_wcps.back().x -vtx1->get_fit_pt().x,2)+pow(vec_wcps.back().y -vtx1->get_fit_pt().y,2)+pow(vec_wcps.back().z -vtx1->get_fit_pt().z,2)));
	      for (size_t j=0;j!=vec_wcps.size();j++){
		double dis1 = sqrt(pow(vec_wcps.at(j).x -vtx1->get_fit_pt().x,2)+pow(vec_wcps.at(j).y -vtx1->get_fit_pt().y,2)+pow(vec_wcps.at(j).z -vtx1->get_fit_pt().z,2));
		double dis = fabs(dis1 - 3.0*units::cm);
		if (dis < min_dis){
		  min_wcp = vec_wcps.at(j);
		  min_dis = dis;
		}
	      }
	      // establish the shortest path ...
	      std::list<WCP::WCPointCloud<double>::WCPoint> new_list;
	      new_list.push_back(vtx_new_wcp);
	      {
		double dis_step = 1.0*units::cm;
		int ncount = std::round(sqrt(pow(vtx_new_wcp.x - min_wcp.x,2) + pow(vtx_new_wcp.y - min_wcp.y,2) + pow(vtx_new_wcp.z - min_wcp.z,2))/dis_step);
		if (ncount <2) ncount = 2;
		for (int qx=1;qx<ncount;qx++){
		  Point tmp_p(vtx_new_wcp.x + (min_wcp.x-vtx_new_wcp.x)/ncount*qx, vtx_new_wcp.y + (min_wcp.y-vtx_new_wcp.y)/ncount*qx, vtx_new_wcp.z + (min_wcp.z-vtx_new_wcp.z)/ncount*qx);
		  WCP::WCPointCloud<double>::WCPoint& tmp_wcp = pcloud->get_closest_wcpoint(tmp_p);
		  //std::cout << qx << " " << sqrt(pow(tmp_wcp.x - tmp_p.x,2) + pow(tmp_wcp.y - tmp_p.y,2) + pow(tmp_wcp.z - tmp_p.z,2))/units::cm << std::endl;
		  if (sqrt(pow(tmp_wcp.x - tmp_p.x,2) + pow(tmp_wcp.y - tmp_p.y,2) + pow(tmp_wcp.z - tmp_p.z,2)) > 0.3*units::cm) continue; // too large ...
		  if (tmp_wcp.index != new_list.back().index && tmp_wcp.index != min_wcp.index)
		    new_list.push_back(tmp_wcp);
		}
	      }
	      new_list.push_back(min_wcp);
	      
	      
	      std::list<WCP::WCPointCloud<double>::WCPoint> old_list;
	      std::copy(vec_wcps.begin(), vec_wcps.end(), std::back_inserter(old_list));
	      // append the list back ...
	      //std::cout << min_wcp.index << " " << temp_cluster->get_path_wcps().size() << std::endl;
	      if (flag_front){
		while(old_list.front().index != min_wcp.index && old_list.size()>0 ){
		  old_list.pop_front();
		}
		old_list.pop_front();
		//      std::cout << "f2 " << old_list.size() << std::endl;
		for (auto it = new_list.rbegin(); it!=new_list.rend(); it++){
		  old_list.push_front(*it);
		}
		//      std::cout << "f3 " << old_list.size() << std::endl;
	      }else{
		//      std::cout << "b1 " << old_list.size() << std::endl;
		while(old_list.back().index != min_wcp.index && old_list.size()>0 ){
		  old_list.pop_back();
		}
		old_list.pop_back();
		//      std::cout << "b2 " << old_list.size() << std::endl;
		for (auto it = new_list.rbegin(); it!=new_list.rend(); it++){
		  old_list.push_back(*it);
		}
		//      std::cout << "b3 " << old_list.size() << std::endl;
	      }
	      vec_wcps.clear();
	      vec_wcps.reserve(old_list.size());
	      std::copy(std::begin(old_list), std::end(old_list), std::back_inserter(vec_wcps));

	      WCPPID::ProtoVertex *tt_vtx = find_other_vertex(sg1, vtx1);
	      if (tt_vtx != main_vertex){
		add_proto_connection(main_vertex, sg1, temp_cluster);
	      }else{
		del_proto_segment(sg1);
	      }
	    }
	    
	    del_proto_vertex(vtx1);
	    del_proto_segment(sg);
	    break;
	  }
	} // distacen is close
      } //loop over segment connecting to main vertex
      if (flag_update){
	flag_continue = true;
	flag_updated = true;
	temp_cluster->do_multi_tracking(map_vertex_segments, map_segment_vertices, *ct_point_cloud, global_wc_map, flash_time*units::microsecond, true, true, true);
      }
    } // flag continue loop ...
  } // general check


  return flag_updated;
}


bool WCPPID::NeutrinoID::examine_structure_final_1p(WCPPID::PR3DCluster* temp_cluster){
  bool flag_update = false;
  std::set<WCPPID::ProtoSegment*> new_segments; // need to fit and then PID ...

  auto it = map_vertex_segments.find(main_vertex);
  if (it == map_vertex_segments.end() || it->second.size()!=2) return flag_update;
  
  WCPPID::ProtoSegment *sg1 = *(it->second.begin());
  WCPPID::ProtoSegment *sg2 = *(it->second.rbegin());

  TVector3 dir1 = sg1->cal_dir_3vector(main_vertex->get_fit_pt(), 15*units::cm);
  TVector3 dir2 = sg2->cal_dir_3vector(main_vertex->get_fit_pt(), 15*units::cm);
  double angle = dir1.Angle(dir2)/3.1415926*180.;
  double length1 = sg1->get_length();
  double length2 = sg2->get_length();

  //  std::cout << angle << " " << main_vertex->get_wcpt().index << std::endl;
  
  if (angle > 175){
    if (length1 < 6*units::cm && length1 < length2){
      WCPPID::ProtoVertex *vtx = find_other_vertex(sg1, main_vertex);
      std::vector<WCP::WCPointCloud<double>::WCPoint >& vec_wcps = sg2->get_wcpt_vec();
      bool flag_front;
      if (vec_wcps.front().index == main_vertex->get_wcpt().index){
	flag_front = true;
      }else{
	flag_front = false;
      }
      std::vector<WCP::WCPointCloud<double>::WCPoint >& vec_wcps1 = sg1->get_wcpt_vec();
      bool flag_front1;
      if (vec_wcps1.front().index == main_vertex->get_wcpt().index){
	flag_front1 = true;
      }else{
	flag_front1 = false;
      }
      
      
      std::list<WCP::WCPointCloud<double>::WCPoint> old_list;
      std::copy(vec_wcps.begin(), vec_wcps.end(), std::back_inserter(old_list));

      if (flag_front && flag_front1){
	for (auto it1 = vec_wcps1.begin(); it1 != vec_wcps1.end(); it1++){
	  if ((*it1).index != old_list.front().index) old_list.push_front(*it1);
	}
      }else if (flag_front && (!flag_front1)){
	for (auto it1 = vec_wcps1.rbegin(); it1 != vec_wcps1.rend(); it1++){
	  if ((*it1).index != old_list.front().index) old_list.push_front(*it1);
	}
      }else if ((!flag_front) && flag_front1){
	for (auto it1 = vec_wcps1.begin(); it1 != vec_wcps1.end(); it1++){
	  if ((*it1).index != old_list.back().index) old_list.push_back(*it1);
	}
      }else if ((!flag_front) && (!flag_front1)){
	for (auto it1 = vec_wcps1.rbegin(); it1 != vec_wcps1.rend(); it1++){
	  if ((*it1).index != old_list.back().index) old_list.push_back(*it1);
	}
      }
      
      vec_wcps.clear();
      vec_wcps.reserve(old_list.size());
      std::copy(std::begin(old_list), std::end(old_list), std::back_inserter(vec_wcps));
      main_vertex->set_wcpt(vtx->get_wcpt());

      //      std::cout << main_vertex->get_wcpt().index << " A " << vtx->get_wcpt().index << std::endl;
      
      std::cout << "[examine_structure_final_1p] Cluster: " << temp_cluster->get_cluster_id() << " Merge short sg " << sg1->get_id() << " (len=" << length1/units::cm << "cm) into sg " << sg2->get_id() << " (len=" << length2/units::cm << "cm), angle=" << angle << " deg, remove vtx " << vtx->get_id() << " -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;
      for (auto it1 = map_vertex_segments[vtx].begin(); it1 != map_vertex_segments[vtx].end(); it1++){
	if ((*it1) == sg1) continue;
	add_proto_connection(main_vertex,*it1,temp_cluster);
      }
      del_proto_segment(sg1);
      del_proto_vertex(vtx);

      flag_update =true;

    }else if (length2 < 6*units::cm && length2 < length1){
      
      WCPPID::ProtoVertex *vtx = find_other_vertex(sg2, main_vertex);
      std::vector<WCP::WCPointCloud<double>::WCPoint >& vec_wcps = sg1->get_wcpt_vec();
      bool flag_front;
      if (vec_wcps.front().index == main_vertex->get_wcpt().index){
	flag_front = true;
      }else{
	flag_front = false;
      }
      std::vector<WCP::WCPointCloud<double>::WCPoint >& vec_wcps1 = sg2->get_wcpt_vec();
      bool flag_front1;
      if (vec_wcps1.front().index == main_vertex->get_wcpt().index){
	flag_front1 = true;
      }else{
	flag_front1 = false;
      }
      
      
      std::list<WCP::WCPointCloud<double>::WCPoint> old_list;
      std::copy(vec_wcps.begin(), vec_wcps.end(), std::back_inserter(old_list));

      if (flag_front && flag_front1){
	for (auto it1 = vec_wcps1.begin(); it1 != vec_wcps1.end(); it1++){
	  if ((*it1).index != old_list.front().index) old_list.push_front(*it1);
	}
      }else if (flag_front && (!flag_front1)){
	for (auto it1 = vec_wcps1.rbegin(); it1 != vec_wcps1.rend(); it1++){
	  if ((*it1).index != old_list.front().index) old_list.push_front(*it1);
	}
      }else if ((!flag_front) && flag_front1){
	for (auto it1 = vec_wcps1.begin(); it1 != vec_wcps1.end(); it1++){
	  if ((*it1).index != old_list.back().index) old_list.push_back(*it1);
	}
      }else if ((!flag_front) && (!flag_front1)){
	for (auto it1 = vec_wcps1.rbegin(); it1 != vec_wcps1.rend(); it1++){
	  if ((*it1).index != old_list.back().index) old_list.push_back(*it1);
	}
      }
      
      vec_wcps.clear();
      vec_wcps.reserve(old_list.size());
      std::copy(std::begin(old_list), std::end(old_list), std::back_inserter(vec_wcps));
      main_vertex->set_wcpt(vtx->get_wcpt());
      

      //      std::cout << main_vertex->get_wcpt().index << " B " << vtx->get_wcpt().index << std::endl;
      std::cout << "[examine_structure_final_1p] Cluster: " << temp_cluster->get_cluster_id() << " Merge short sg " << sg2->get_id() << " (len=" << length2/units::cm << "cm) into sg " << sg1->get_id() << " (len=" << length1/units::cm << "cm), angle=" << angle << " deg, remove vtx " << vtx->get_id() << " -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;
      for (auto it1 = map_vertex_segments[vtx].begin(); it1 != map_vertex_segments[vtx].end(); it1++){
	if ((*it1) == sg2) continue;
	add_proto_connection(main_vertex,*it1,temp_cluster);
      }
      del_proto_segment(sg2);
      del_proto_vertex(vtx);

      flag_update =true;
    }
    
    //    std::cout << main_vertex->get_wcpt().index << std::endl;
    
    if (flag_update)
      temp_cluster->do_multi_tracking(map_vertex_segments, map_segment_vertices, *ct_point_cloud, global_wc_map, flash_time*units::microsecond, true, true, true);
  }
  
  return flag_update;
}


bool WCPPID::NeutrinoID::examine_structure_final_1(WCPPID::PR3DCluster* temp_cluster){
  // merge two segments if a direct connection is better ...
  bool flag_update = false;
  bool flag_continue = true;

  std::set<WCPPID::ProtoSegment*> new_segments; // need to fit and then PID ...
  
  while(flag_continue){
    flag_continue = false;
    for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end(); it++){
      WCPPID::ProtoVertex *vtx = it->first;
      if (vtx->get_cluster_id()!=temp_cluster->get_cluster_id()) continue;
      if (it->second.size()!=2) continue;
      if (vtx == main_vertex) continue;
      WCPPID::ProtoSegment *sg1 = *(it->second.begin());
      WCPPID::ProtoSegment *sg2 = *(it->second.rbegin());

      if (sg1->get_wcpt_vec().front().index == sg2->get_wcpt_vec().front().index && sg1->get_wcpt_vec().back().index == sg2->get_wcpt_vec().back().index
	  || sg1->get_wcpt_vec().front().index == sg2->get_wcpt_vec().back().index && sg1->get_wcpt_vec().back().index == sg2->get_wcpt_vec().front().index){
	std::cout << "[examine_structure_final_1] Cluster: " << temp_cluster->get_cluster_id() << " Remove duplicate segment " << sg2->get_id() << " (same path as " << sg1->get_id() << ") at vtx " << vtx->get_id() << " -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;
	del_proto_segment(sg2);
	flag_update = true;
	flag_continue = true;
	break;
      }else{
	double length1 = sg1->get_length();
	double length2 = sg2->get_length();
	
	
	WCPPID::ProtoVertex *vtx1 = find_other_vertex(sg1, vtx);
	WCPPID::ProtoVertex *vtx2 = find_other_vertex(sg2, vtx);
	
	//      std::cout << map_segment_vertices[sg1].size() << " " << map_segment_vertices[sg2].size() << " " << sg1->get_id() << " " << sg1->get_wcpt_vec().front().index << " " << sg1->get_wcpt_vec().back().index << " " << sg2->get_id() << " " << sg2->get_wcpt_vec().front().index << " " << sg2->get_wcpt_vec().back().index << std::endl;
	
	double step_size = 0.6*units::cm;
	Point start_p = vtx1->get_fit_pt();
	Point end_p = vtx2->get_fit_pt();
	int ncount = std::round(sqrt(pow(start_p.x-end_p.x,2)+pow(start_p.y-end_p.y,2)+pow(start_p.z-end_p.z,2))/step_size);
	
	PointVector new_pts;
	bool flag_replace = true;
	int n_bad = 0;
	for (int i=1;i<ncount;i++){
	  Point test_p;
	  test_p.x = start_p.x + (end_p.x - start_p.x)/ncount*i;
	  test_p.y = start_p.y + (end_p.y - start_p.y)/ncount*i;
	  test_p.z = start_p.z + (end_p.z - start_p.z)/ncount*i;
	  new_pts.push_back(test_p);
	  if (!ct_point_cloud->is_good_point(test_p, 0.2*units::cm, 0, 0)) n_bad ++;
	  if (n_bad>1) flag_replace = false;
	  //	std::cout << i << " " << test_p << " " << ct_point_cloud->is_good_point(test_p, 0.2*units::cm, 0, 0) << std::endl;
	}
	// std::cout << flag_replace << " " << sg1->get_id() << " " << sg2->get_id() << " " << length1/units::cm << " " << length2/units::cm << " " << n_bad << " " << ncount * step_size << std::endl;
	
	if (flag_replace){
	  // form a new segment
	  std::cout << "[examine_structure_final_1] Cluster: " << temp_cluster->get_cluster_id() << " Final stage merge two short segments: " << sg1->get_id() << " (len=" << length1/units::cm << "cm) + " << sg2->get_id() << " (len=" << length2/units::cm << "cm) with a straight one, vtx id " << vtx->get_id() << " -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;
	  
	  WCP::ToyPointCloud* pcloud_steiner = temp_cluster->get_point_cloud_steiner();
	  std::list<WCP::WCPointCloud<double>::WCPoint > wcps;
	  WCP::WCPointCloud<double>::WCPoint start_wcp = vtx1->get_wcpt();
	  WCP::WCPointCloud<double>::WCPoint end_wcp = vtx2->get_wcpt();
	  wcps.push_back(start_wcp);
	  for (size_t i=0; i!=new_pts.size(); i++){
	    WCP::WCPointCloud<double>::WCPoint& wcp =  pcloud_steiner->get_closest_wcpoint(new_pts.at(i));
	    if (wcp.index != wcps.back().index) wcps.push_back(wcp);
	  }
	  if (end_wcp.index != wcps.back().index) wcps.push_back(end_wcp);
	  
	  
	  WCPPID::ProtoSegment *sg3 = new WCPPID::ProtoSegment(acc_segment_id, wcps, temp_cluster->get_cluster_id()); acc_segment_id++;
	  add_proto_connection(vtx1,sg3,temp_cluster);
	  add_proto_connection(vtx2,sg3,temp_cluster);
	  
	  del_proto_segment(sg1);
	  del_proto_segment(sg2);
	  del_proto_vertex(vtx);
	  
	  flag_update = true;
	  flag_continue = true;
	  break;
	}
      }
    }
  } // continue
  if (flag_update){

    /* for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end();it++){ */
    /*   WCPPID::ProtoSegment *sg = it->first; */
    /*   if (sg->get_cluster_id() != temp_cluster->get_cluster_id()) continue; */
    /*   std::cout << sg->get_id() << " " << it->second.size() << " " << sg->get_wcpt_vec().front().index << " " << sg->get_wcpt_vec().back().index << " " << (*it->second.begin())->get_wcpt().index << " " << (*it->second.rbegin())->get_wcpt().index << std::endl; */
    /* } */
    
    temp_cluster->do_multi_tracking(map_vertex_segments, map_segment_vertices, *ct_point_cloud, global_wc_map, flash_time*units::microsecond, true, true, true);
  }
  return flag_update;
}




/* bool WCPPID::NeutrinoID::examine_vertices_5(WCPPID::PR3DCluster* temp_cluster){ */

/*   std::cout << "haha1 " << std::endl; */


  
/*   for (auto it = map_vertex_segments.begin(); it != map_vertex_segments.end(); it++){ */
/*     WCPPID::ProtoVertex *vtx = it->first; */
/*     if (vtx->get_cluster_id() != temp_cluster->get_cluster_id()) continue; */
/*     if (it->second.size()!=2) continue; // require two segments ... */
    
/*     for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){ */
/*       WCPPID::ProtoSegment *sg1 = (*it1); */
/*       WCPPID::ProtoVertex *vtx1 = find_other_vertex(sg1, vtx); */
      
/*       auto it2 = map_vertex_segments.find(vtx1); */
/*       if ((it2->second).size()!=2) continue; // require two segments ... */
/*       WCPPID::ProtoSegment *sg2  = 0; */
/*       for (auto it3 = it2->second.begin(); it3 != it2->second.end(); it3++){ */
/* 	if (*it3 != sg1){ */
/* 	  sg2 = *it3; */
/* 	  break; */
/* 	} */
/*       } */

/*       WCPPID::ProtoVertex *vtx2 = find_other_vertex(sg2, vtx1); */
/*       double dis = sqrt(pow(vtx->get_fit_pt().x - vtx2->get_fit_pt().x,2) + pow(vtx->get_fit_pt().y - vtx2->get_fit_pt().y,2) + pow(vtx->get_fit_pt().z - vtx2->get_fit_pt().z,2)); */
/*       if (dis < 5*units::cm){ */
/* 	WCPPID::ProtoSegment *sg = 0; */
/* 	for (auto it3 = it->second.begin(); it3 != it->second.end(); it3++){ */
/* 	  if ( *it3 != sg1) { */
/* 	    sg = *it3; */
/* 	    break; */
/* 	  } */
/* 	} */
/* 	std::cout << sg1->get_id() << " " << sg2->get_id() << std::endl; */
/* 	examine_vertices_5(sg, vtx, vtx2, sg1, vtx1, sg2); */
/*       } */
/*     } */
/*   } */

/*     std::cout << "haha2 " << std::endl; */
/* } */


/* bool WCPPID::NeutrinoID::examine_vertices_5(WCPPID::ProtoSegment *sg, WCPPID::ProtoVertex* vtx, WCPPID::ProtoVertex *vtx2, WCPPID::ProtoSegment *sg1, WCPPID::ProtoVertex *vtx1, WCPPID::ProtoSegment *sg2){ */
/*   TVector3 dir1(vtx2->get_fit_pt().x - vtx->get_fit_pt().x, vtx2->get_fit_pt().y - vtx->get_fit_pt().y, vtx2->get_fit_pt().z - vtx->get_fit_pt().z); */
/*   TVector3 dir2 = sg->cal_dir_3vector(vtx->get_fit_pt(), 6*units::cm); */

/*   std::cout << dir1.Angle(dir2)/3.1415926*180. << std::endl; */
/* } */
