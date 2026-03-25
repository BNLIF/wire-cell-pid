void WCPPID::NeutrinoID::separate_track_shower(WCPPID::PR3DCluster* temp_cluster){
  const bool flag_timing = true;
  using STS_Clock = std::chrono::high_resolution_clock;
  using STS_ms = std::chrono::duration<double, std::milli>;
  auto sts_t_total = STS_Clock::now();
  auto sts_t0 = STS_Clock::now();

  STS_ms t_shower_topology{0}, t_shower_trajectory{0};
  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != temp_cluster->get_cluster_id()) continue;

    sts_t0 = STS_Clock::now();
    sg->is_shower_topology();
    t_shower_topology += std::chrono::duration_cast<STS_ms>(STS_Clock::now()-sts_t0);

    if (!sg->get_flag_shower_topology()){
      sts_t0 = STS_Clock::now();
      sg->is_shower_trajectory();
      t_shower_trajectory += std::chrono::duration_cast<STS_ms>(STS_Clock::now()-sts_t0);
    }
  }
  if (flag_timing) std::cout << "separate_track_shower timing: is_shower_topology took " << t_shower_topology.count() << " ms" << std::endl;
  if (flag_timing) std::cout << "separate_track_shower timing: is_shower_trajectory took " << t_shower_trajectory.count() << " ms" << std::endl;
  if (flag_timing) std::cout << "separate_track_shower timing: TOTAL took " << std::chrono::duration_cast<STS_ms>(STS_Clock::now()-sts_t_total).count() << " ms" << std::endl;
}

void WCPPID::NeutrinoID::separate_track_shower(){
  //  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
  //  WCPPID::ProtoSegment *sg = it->first;
  //  sg->is_shower_trajectory();
  //  sg->is_shower_topology();
  //  std::cout <<" C: " << sg->get_cluster_id() << " " << sg->get_id() << " " << sg->get_flag_shower_trajectory() << " " << sg->get_flag_shower_topology() << std::endl;
  //}
  
  PR3DClusterSelection all_clusters = other_clusters;
  all_clusters.push_back(main_cluster);
  for (auto it = all_clusters.begin(); it!=all_clusters.end(); it++){
    WCPPID::PR3DCluster* temp_cluster = *it;

    std::map<int, WCPPID::ProtoSegment*> map_id_seg;
    for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
      WCPPID::ProtoSegment *sg = it->first;
      if (sg->get_cluster_id() != temp_cluster->get_cluster_id()) continue;
      map_id_seg[sg->get_id()] = sg;
      //std::cout << temp_cluster->get_cluster_id() << " " << sg->get_id() << std::endl;
    }
    
    std::vector<int>& point_sub_cluster_ids = temp_cluster->get_point_sub_cluster_ids();
    std::vector<bool>& point_flag_showers = temp_cluster->get_point_flag_showers();
    point_flag_showers.resize(point_sub_cluster_ids.size(), false);

        
    for (size_t i=0;i!=point_sub_cluster_ids.size();i++){ 
      if (point_sub_cluster_ids.at(i) == -1) continue;
      if (map_id_seg.find(point_sub_cluster_ids.at(i)) == map_id_seg.end()) continue;
      //    std::cout << point_sub_cluster_ids.at(i) << " " << map_id_seg[point_sub_cluster_ids.at(i)] << std::endl; 
      if ( map_id_seg[point_sub_cluster_ids.at(i)]->get_flag_shower()) point_flag_showers.at(i) = true; 
    } 
  }
  
}



void WCPPID::NeutrinoID::determine_direction(WCPPID::PR3DCluster* temp_cluster){

  
  
  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != temp_cluster->get_cluster_id()) continue;

    WCPPID::ProtoVertex *start_v=0, *end_v=0;
    for (auto it = map_segment_vertices[sg].begin(); it!=map_segment_vertices[sg].end(); it++){
      if ((*it)->get_wcpt().index == sg->get_wcpt_vec().front().index) start_v = *it;
      if ((*it)->get_wcpt().index == sg->get_wcpt_vec().back().index) end_v = *it;
    }
    if (start_v==0 || end_v==0){
      std::cout << "Error in finding vertices for a segment " << std::endl; 
    }

    bool flag_print = false;
    //if (sg->get_cluster_id() == main_cluster->get_cluster_id()) flag_print = true;
    //if (sg->get_cluster_id()==62) flag_print = true;
    
    if (sg->get_flag_shower_trajectory()){
      // trajectory shower
      sg->determine_dir_shower_trajectory(map_vertex_segments[start_v].size(), map_vertex_segments[end_v].size(), flag_print);
    }else if (sg->get_flag_shower_topology()){
      // topology /* shower */
      sg->determine_dir_shower_topology(map_vertex_segments[start_v].size(), map_vertex_segments[end_v].size(), flag_print);
    }else{
      // track ...
      sg->determine_dir_track(map_vertex_segments[start_v].size(), map_vertex_segments[end_v].size(), flag_print);
    }
    // std::cout << sg->get_id() << " " << sg->get_particle_type() << std::endl;
    
  }
}

void WCPPID::NeutrinoID::improve_maps_one_in(WCPPID::PR3DCluster* temp_cluster, bool flag_strong_check){
  bool flag_update = true;
  std::set<WCPPID::ProtoVertex* > used_vertices;
  std::set<WCPPID::ProtoSegment* > used_segments;
  while(flag_update){
    flag_update = false;
    for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end();it++){
      WCPPID::ProtoVertex *vtx = it->first;
      if (vtx->get_cluster_id() != temp_cluster->get_cluster_id()) continue;
      if (it->second.size()==1) continue;
      if (used_vertices.find(vtx)!=used_vertices.end()) continue;
      
      int n_in = 0;
      std::map<WCPPID::ProtoSegment*, bool> map_sg_dir;
      for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	WCPPID::ProtoSegment *sg = (*it1);
	if (used_segments.find(sg) != used_segments.end()) continue;
	bool flag_start;
	if (sg->get_wcpt_vec().front().index == vtx->get_wcpt().index)
	  flag_start = true;
	else if (sg->get_wcpt_vec().back().index == vtx->get_wcpt().index)
	  flag_start = false;
	
	if ((flag_start && sg->get_flag_dir()==-1 || (!flag_start) && sg->get_flag_dir()==1)){
	  if (flag_strong_check){
	    if (!sg->is_dir_weak()) n_in ++;
	  }else{
	    n_in ++;
	  }
	}
	if (sg->get_flag_dir()==0 || sg->is_dir_weak())
	  map_sg_dir[sg] = flag_start;
      }
      
      if (map_sg_dir.size() == 0 ) used_vertices.insert(vtx); // no outside to change ...
      
      if (n_in>0){
	for (auto it1 = map_sg_dir.begin(); it1!=map_sg_dir.end(); it1++){
	  WCPPID::ProtoSegment *sg = it1->first;
	  bool flag_start = it1->second;
	  if (flag_start){
	    sg->set_flag_dir(1);
	  }else{
	    sg->set_flag_dir(-1);
	  }
	  sg->cal_4mom();
	  sg->set_dir_weak(true);
	  used_segments.insert(sg); // change direction ...
	  flag_update = true;
	}
	used_vertices.insert(vtx); // change vertices ...
      }
      //	if (flag_update) break; // is this the best?
    }
  } // keep updating
}


bool WCPPID::NeutrinoID::examine_maps(WCPPID::ProtoVertex *temp_vertex){
  return examine_maps(temp_vertex->get_cluster_id());
}




bool WCPPID::NeutrinoID::examine_maps(int temp_cluster_id){
  bool flag_return = true;
  for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end();it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() != temp_cluster_id) continue;
    if (it->second.size()==1) continue;

    int n_in = 0;
    int n_in_shower = 0;
    int n_out_tracks = 0;
    
    for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
      WCPPID::ProtoSegment *sg = (*it1);
      bool flag_start;
      if (sg->get_wcpt_vec().front().index == vtx->get_wcpt().index)
	flag_start = true;
      else if (sg->get_wcpt_vec().back().index == vtx->get_wcpt().index)
	flag_start = false;
      
      if (flag_start && sg->get_flag_dir()==-1 || (!flag_start) && sg->get_flag_dir()==1){
	n_in ++;
	if (sg->get_flag_shower()) n_in_shower ++;
      }
      if (flag_start && sg->get_flag_dir()==1 || (!flag_start) && sg->get_flag_dir()==-1){
	if (!sg->get_flag_shower()) n_out_tracks ++;
      }
    }

    if (n_in > 1 && n_in != n_in_shower) {
      std::cout << "Wrong: Multiple (" << n_in << ") particles into a vertex! " << std::endl;
      print_segs_info(temp_cluster_id, vtx);
      flag_return = false;
    }
    if (n_in_shower >0 && n_out_tracks > 0) {
      std::cout << "Wrong: " << n_in_shower << " showers in and " << n_out_tracks << " tracks out! " << std::endl;
      print_segs_info(temp_cluster_id, vtx);
      flag_return = false;
    }
  }
  return flag_return;
}

bool WCPPID::NeutrinoID::examine_maps(WCPPID::PR3DCluster* temp_cluster){
  return examine_maps(temp_cluster->get_cluster_id());
}

void WCPPID::NeutrinoID::examine_good_tracks(int temp_cluster_id){
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
      WCPPID::ProtoSegment *sg = it->first;
      if (sg->get_cluster_id() != temp_cluster_id) continue;
      if (sg->get_flag_shower()) continue;
      if (sg->get_flag_dir()==0 || sg->is_dir_weak() ) continue;

      auto pair_vertices = find_vertices(sg);
      WCPPID::ProtoVertex *start_vertex = 0, *end_vertex = 0;
      if (sg->get_flag_dir()==1){
	if (pair_vertices.first->get_wcpt().index == sg->get_wcpt_vec().front().index){
	  start_vertex = pair_vertices.first;
	  end_vertex = pair_vertices.second;
	}else{
	  start_vertex = pair_vertices.second;
	  end_vertex = pair_vertices.first;
	}
      }else if (sg->get_flag_dir()==-1){
	if (pair_vertices.first->get_wcpt().index == sg->get_wcpt_vec().front().index){
	   start_vertex = pair_vertices.second;
	  end_vertex = pair_vertices.first;
	}else{
	  start_vertex = pair_vertices.first;
	  end_vertex = pair_vertices.second;
	}
      }
      auto result_pair = calculate_num_daughter_showers(start_vertex, sg);
      int num_daughter_showers = result_pair.first;
      double length_daughter_showers = result_pair.second;
      double max_angle = 0;
      TVector3 dir1 = sg->cal_dir_3vector(end_vertex->get_fit_pt(), 15*units::cm);
      TVector3 drift_dir(1,0,0);
      double min_para_angle = 1e9;
      for (auto it1 = map_vertex_segments[end_vertex].begin(); it1!=map_vertex_segments[end_vertex].end(); it1++){
	WCPPID::ProtoSegment *sg1 = *it1;
	if (sg1 == sg) continue;
	TVector3 dir2 = sg1->cal_dir_3vector(end_vertex->get_fit_pt(), 15*units::cm);
	double angle = dir1.Angle(dir2)/3.1415926*180.;
	if (angle > max_angle) max_angle = angle;
	angle = fabs(drift_dir.Angle(dir2)/3.1415926*180.-90);
	if (angle < min_para_angle) min_para_angle = angle;
      }

      if ((num_daughter_showers >=4 || length_daughter_showers > 50*units::cm && num_daughter_showers >=2) && (max_angle > 155 || fabs(drift_dir.Angle(dir1)/3.1415926*180.-90.) < 15 &&  min_para_angle < 15 && min_para_angle +  fabs(drift_dir.Angle(dir1)/3.1415926*180.-90.) < 25) && sg->get_length() < 15*units::cm){
	sg->set_particle_type(11);
	TPCParams& mp = Singleton<TPCParams>::Instance();
	sg->set_particle_mass(mp.get_mass_electron());
	sg->set_flag_dir(0);
	sg->set_dir_weak(1);
      }
      
      //      std::cout << sg->get_id() << " " << sg->get_particle_type() << " " << num_daughter_showers << " " << sg->get_length()/units::cm << " " << max_angle << " " << min_para_angle << " " << fabs(drift_dir.Angle(dir1)/3.1415926*180.-90.) << std::endl;

       /* TVector3 dir1 = sg->cal_dir_3vector(pair_vertices.first->get_fit_pt(), 15*units::cm); */
       /* 	    for (auto it1 = map_vertex_segments[pair_vertices.first].begin(); it1!= map_vertex_segments[pair_vertices.first].end(); it1++){ */
       /* 	      if (*it1 == sg) continue; */
       /* 	      TVector3 dir2 = (*it1)->cal_dir_3vector(pair_vertices.first->get_fit_pt(), 15*units::cm); */
       /* 	      if ( (*it1)->get_flag_shower()){ */
       /* 		double  angle = dir1.Angle(dir2)/3.1415926*180; */
       /* 		if (max_angle1 < angle) max_angle1 = angle; */
       /* 		num_s1 += calculate_num_daughter_showers(pair_vertices.first, *it1); */
       /* 	      } */
       /* 	    } */
       /* 	    dir1 = sg->cal_dir_3vector(pair_vertices.second->get_fit_pt(), 10*units::cm); */
  }
}

void WCPPID::NeutrinoID::judge_no_dir_tracks_close_to_showers(int temp_cluster_id){
  std::set<WCPPID::ProtoSegment* > shower_set;
  std::set<WCPPID::ProtoSegment* > no_dir_track_set;
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
      WCPPID::ProtoSegment *sg = it->first;
      if (sg->get_cluster_id() != temp_cluster_id) continue;
      if (sg->get_flag_shower()){
	shower_set.insert(sg);
      }else{
	if (sg->get_flag_dir()==0) no_dir_track_set.insert(sg);
      }
  }

  
  //  std::cout <<  no_dir_track_set.size() << " " << shower_set.size() << std::endl;
  for (auto it = no_dir_track_set.begin(); it != no_dir_track_set.end(); it++){
    WCPPID::ProtoSegment *sg = *it;
    bool flag_change = true;
    PointVector& pts = sg->get_point_vec();
    
    for (size_t i=0; i!= pts.size(); i++){
      double min_u_dis = 1e9, min_v_dis = 1e9, min_w_dis = 1e9;
      for (auto it1 = shower_set.begin(); it1 != shower_set.end(); it1++){
	auto results = (*it1)->get_closest_2d_dis(pts.at(i));
	if (std::get<0>(results) < min_u_dis) min_u_dis = std::get<0>(results);
	if (std::get<1>(results) < min_v_dis) min_v_dis = std::get<1>(results);
	if (std::get<2>(results) < min_w_dis) min_w_dis = std::get<2>(results);
      }
      if (min_u_dis > 0.6*units::cm || min_v_dis > 0.6*units::cm || min_w_dis > 0.6*units::cm){
	flag_change = false;
      }
    }
    if (flag_change){
      sg->set_particle_type(11);
      TPCParams& mp = Singleton<TPCParams>::Instance();
      sg->set_particle_mass(mp.get_mass_electron());
    }
    //    std::cout << sg->get_id() << " " << flag_change << std::endl;
  }
  
}

void WCPPID::NeutrinoID::improve_maps_no_dir_tracks(int temp_cluster_id){

  TVector3 drift_dir(1,0,0);
  bool flag_update = true;
  while(flag_update){
    flag_update = false;
    for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
      WCPPID::ProtoSegment *sg = it->first;
      if (sg->get_cluster_id() != temp_cluster_id) continue;
      if (sg->get_flag_shower()) continue;
      double length = sg->get_length();
      
      if (sg->get_flag_dir()==0 || sg->is_dir_weak() || fabs(sg->get_particle_type())==2212) {
	std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> two_vertices = find_vertices(sg);
	int nshowers[2]={0,0};
	int n_in[2] = {0,0};
	int nmuons[2] ={0,0};
	int nprotons[2] = {0,0};
	
	for (auto it1 = map_vertex_segments[two_vertices.first].begin(); it1!=map_vertex_segments[two_vertices.first].end(); it1++){
	  if ((*it1)->get_flag_shower()) 	  nshowers[0] ++;
	  bool flag_start;
	  if ((*it1)->get_wcpt_vec().front().index == two_vertices.first->get_wcpt().index)
	    flag_start = true;
	  else if ((*it1)->get_wcpt_vec().back().index == two_vertices.first->get_wcpt().index)
	    flag_start = false;
	  if ((flag_start && (*it1)->get_flag_dir()==-1 || (!flag_start) && (*it1)->get_flag_dir()==1)) n_in[0]++;
	  if (fabs((*it1)->get_particle_type())==13)	  nmuons[0] ++;
	  if (fabs((*it1)->get_particle_type())==2212)	  nprotons[0] ++;
	}
	for (auto it1 = map_vertex_segments[two_vertices.second].begin(); it1!=map_vertex_segments[two_vertices.second].end(); it1++){
	  if ((*it1)->get_flag_shower()) 	  nshowers[1] ++;
	  bool flag_start;
	  if ((*it1)->get_wcpt_vec().front().index == two_vertices.second->get_wcpt().index)
	    flag_start = true;
	  else if ((*it1)->get_wcpt_vec().back().index == two_vertices.second->get_wcpt().index)
	    flag_start = false;
	  if ((flag_start && (*it1)->get_flag_dir()==-1 || (!flag_start) && (*it1)->get_flag_dir()==1)) n_in[1]++;
	  if (fabs((*it1)->get_particle_type())==13 )	  nmuons[1] ++;
	  if (fabs((*it1)->get_particle_type())==2212)	  nprotons[1] ++;
	}
	
	bool flag_print = false;

	//	std::cout << sg->get_id() << " " << nshowers[0] << " " << nshowers[1] << " " << map_vertex_segments[two_vertices.first].size() << " " << map_vertex_segments[two_vertices.second].size() << " " << sg->get_particle_type()  << " " << nmuons[0] << " " << nmuons[1] << " " << nprotons[0] << " " << nprotons[1] << std::endl;
	
	if (nshowers[0] + nshowers[1] >2 && length<5*units::cm ||
	    nshowers[0]+1 == map_vertex_segments[two_vertices.first].size() && (nshowers[1]+1 == map_vertex_segments[two_vertices.second].size()) && nshowers[0]>0 && nshowers[1]>0 && length<5*units::cm){ // 2 shower, muon, very short track ...
	  if (flag_print)	  std::cout << "A: " << sg->get_id() << std::endl;
	  sg->set_particle_type(11);
	  TPCParams& mp = Singleton<TPCParams>::Instance();
	  sg->set_particle_mass(mp.get_mass_electron());
	  if (sg->get_particle_4mom(3)>0) sg->cal_4mom();
	  flag_update = true;
	
	  
	}else if ( nshowers[0]+1 == map_vertex_segments[two_vertices.first].size() && nshowers[0]>=2 && sg->get_particle_type() == 2212){
	  TVector3 v1 = sg->cal_dir_3vector(two_vertices.first->get_fit_pt(), 5*units::cm);
	  double min_angle = 180;
	  for (auto it2 = map_vertex_segments[two_vertices.first].begin(); it2 != map_vertex_segments[two_vertices.first].end(); it2++){
	    if (*it2 == sg) continue;
	    TVector3 v2 = (*it2)->cal_dir_3vector(two_vertices.first->get_fit_pt(), 5*units::cm);
	    double angle = fabs(v1.Angle(v2)/3.1415926*180.-180.);
	    if (angle < min_angle) min_angle = angle;
	  }
	  double dQ_dx_rms = sg->get_rms_dQ_dx();

	  //	  std::cout << sg->get_id() << " " << dQ_dx_rms/(43e3/units::cm) << " " << min_angle << std::endl;
	  
	  if ( dQ_dx_rms > 1.0 * (43e3/units::cm) && min_angle < 40 ||
	      dQ_dx_rms > 0.75 * (43e3/units::cm) && min_angle < 30 ||
	       dQ_dx_rms > 0.4 *(43e3/units::cm) && min_angle < 15){
	    bool flag_start;
	    if (sg->get_wcpt_vec().front().index == two_vertices.first->get_wcpt().index)
	      flag_start = true;
	    else if (sg->get_wcpt_vec().back().index == two_vertices.first->get_wcpt().index)
	      flag_start = false;
	    if (flag_start)
	      sg->set_flag_dir(-1);
	    else
	      sg->set_flag_dir(1);
	    if (flag_print) std::cout << "C: " << sg->get_id() << std::endl;
	    sg->set_particle_type(11);
	    TPCParams& mp = Singleton<TPCParams>::Instance();
	    sg->set_particle_mass(mp.get_mass_electron());
	    if (sg->get_particle_4mom(3)>0) sg->cal_4mom();
	    flag_update = true;
	  }
	}else if ( nshowers[1]+1 == map_vertex_segments[two_vertices.second].size() && nshowers[1]>=2 && sg->get_particle_type() == 2212){
	  TVector3 v1 = sg->cal_dir_3vector(two_vertices.second->get_fit_pt(), 5*units::cm);
	  double min_angle = 180;
	  for (auto it2 = map_vertex_segments[two_vertices.second].begin(); it2 != map_vertex_segments[two_vertices.second].end(); it2++){
	    if (*it2 == sg) continue;
	    TVector3 v2 = (*it2)->cal_dir_3vector(two_vertices.second->get_fit_pt(), 5*units::cm);
	    double angle = fabs(v1.Angle(v2)/3.1415926*180.-180.);
	    if (angle < min_angle) min_angle = angle;
	  }
	  double dQ_dx_rms = sg->get_rms_dQ_dx();

	  //	  std::cout << sg->get_id() << " " << dQ_dx_rms/(43e3/units::cm) << " " << min_angle << std::endl;
	  
	  if ( dQ_dx_rms > 1.0 * (43e3/units::cm) && min_angle < 40 ||
	       dQ_dx_rms > 0.75 * (43e3/units::cm) && min_angle < 30 ||
	       dQ_dx_rms > 0.4 *(43e3/units::cm) && min_angle < 15){
	    bool flag_start;
	    if (sg->get_wcpt_vec().front().index == two_vertices.second->get_wcpt().index)
	      flag_start = true;
	    else if (sg->get_wcpt_vec().back().index == two_vertices.second->get_wcpt().index)
	      flag_start = false;
	    if (flag_start)
	      sg->set_flag_dir(-1);
	    else
	      sg->set_flag_dir(1);
	    if (flag_print) std::cout << "D: " << sg->get_id() << std::endl;
	    sg->set_particle_type(11);
	    TPCParams& mp = Singleton<TPCParams>::Instance();
	    sg->set_particle_mass(mp.get_mass_electron());
	    if (sg->get_particle_4mom(3)>0) sg->cal_4mom();
	    flag_update = true;
	  }
	}else if (fabs(sg->get_particle_type())==13 &&
		  (nprotons[0]>=0 && nmuons[0] >= 1 && nshowers[1]+1 == map_vertex_segments[two_vertices.second].size() && nshowers[1]>=2 ||
		   nprotons[1]>=0 && nmuons[1] >= 1 && nshowers[0]+1 == map_vertex_segments[two_vertices.first].size() && nshowers[0]>=2 ||
		   (nprotons[0]>=0 && nmuons[0] >= 1 && nshowers[1]+1 == map_vertex_segments[two_vertices.second].size() && nshowers[1]>=1 ||
		    nprotons[1]>=0 && nmuons[1] >= 1 && nshowers[0]+1 == map_vertex_segments[two_vertices.first].size() && nshowers[0]>=1)
		   && (sg->get_flag_dir()==0 || sg->is_dir_weak()))){
	 
	  
	  double direct_length = sg->get_direct_length();

	  
	  
	  if ( direct_length < 34*units::cm && direct_length < 0.93 * length
	       || length < 5*units::cm && (nprotons[0] + nshowers[0] ==0 && nshowers[1] >=2
					   || nprotons[1] + nshowers[1]==0 && nshowers[0]>=2)
	       ){
	    if (flag_print) std::cout << "E: " << sg->get_id() << std::endl;
	    sg->set_particle_type(11);
	    TPCParams& mp = Singleton<TPCParams>::Instance();
	    sg->set_particle_mass(mp.get_mass_electron());
	    if (sg->get_particle_4mom(3)>0) sg->cal_4mom();
	    //	    std::cout << sg->get_id() << " " << length << " " << direct_length << " " << sg->get_particle_type() << std::endl;
	    flag_update = true;
	  }else if (((nshowers[0]+nshowers[1]>=2 && (nprotons[0]+nmuons[0] + nshowers[0]==1 || nprotons[1]+nmuons[1]+nshowers[1]==1))
		     ||(nshowers[0]+nshowers[1]>=1 && (nprotons[0]+nmuons[0] + nshowers[0]>1 || nprotons[1]+nmuons[1]+nshowers[1]>1))
		     )&& length < 40*units::cm){
	    // calculate the number of daughters of daughters ...
	    std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> pair_vertices = find_vertices(sg);
	    int num_s1 = 0, num_s2 = 0;
	    double length_s1 = 0, length_s2 = 0;
	    double max_angle1 = 0, max_angle2 = 0;
	    TVector3 dir1 = sg->cal_dir_3vector(pair_vertices.first->get_fit_pt(), 15*units::cm);
	    for (auto it1 = map_vertex_segments[pair_vertices.first].begin(); it1!= map_vertex_segments[pair_vertices.first].end(); it1++){
	      if (*it1 == sg) continue;
	      TVector3 dir2 = (*it1)->cal_dir_3vector(pair_vertices.first->get_fit_pt(), 15*units::cm);
	      if ( (*it1)->get_flag_shower()){
		double  angle = dir1.Angle(dir2)/3.1415926*180;
		if (max_angle1 < angle) max_angle1 = angle;
		auto pair_result = calculate_num_daughter_showers(pair_vertices.first, *it1);
		num_s1 += pair_result.first;
		length_s1 += pair_result.second;
	      }
	    }
	    dir1 = sg->cal_dir_3vector(pair_vertices.second->get_fit_pt(), 10*units::cm);
	    for (auto it1 = map_vertex_segments[pair_vertices.second].begin(); it1!= map_vertex_segments[pair_vertices.second].end(); it1++){
	      if (*it1 == sg) continue;
	      TVector3 dir2 = (*it1)->cal_dir_3vector(pair_vertices.second->get_fit_pt(), 15*units::cm);
	      if ((*it1)->get_flag_shower()){
		
		double  angle = dir1.Angle(dir2)/3.1415926*180;
		if (max_angle2 < angle) max_angle2 = angle;
		auto pair_result = calculate_num_daughter_showers(pair_vertices.second, *it1);
		num_s2 += pair_result.first;
		length_s2 += pair_result.second;
	      }
	    }
	    //	    std::cout << sg->get_id() << " " << sg->get_particle_type() << " " << length/units::cm << " " << max_angle1 << " " << max_angle2 << " " << num_s1 << " " << num_s2 << " " << length_s1/units::cm << " " << length_s2/units::cm << std::endl;
	    //std::cout << max_angle1 << " " << max_angle2 << std::endl;
	    if ((num_s1 >= 4 || length_s1 > 50*units::cm && num_s1 >=2) && max_angle1 > 150
		|| (num_s2 >= 4 || length_s2 > 50*units::cm) && max_angle2 > 150
		|| length<6*units::cm && (num_s1 >=4 && length_s1>20*units::cm || // short track ...
						 num_s2 >=4 && length_s2>20*units::cm)){
	      if (flag_print) std::cout << "F: " << sg->get_id() << std::endl;
	      sg->set_particle_type(11);
	      TPCParams& mp = Singleton<TPCParams>::Instance();
	      sg->set_particle_mass(mp.get_mass_electron());
	      if (sg->get_particle_4mom(3)>0) sg->cal_4mom();
	      flag_update = true;
	    }
	  }
	} else if (fabs(sg->get_particle_type())==13 && (sg->get_flag_dir()==0 || sg->is_dir_weak()) &&
		   (nmuons[0]+nprotons[0]+nshowers[0]==1 || nmuons[1]+nprotons[1]+nshowers[1]==1) &&
		   (nshowers[0] + nshowers[1] > 0 || sg->get_medium_dQ_dx() < 1.3*43e3/units::cm)){
	  auto pair_vertices = find_vertices(sg);
	  bool flag_change = false;


	  //	  std::cout << sg->get_direct_length()/units::cm << " " << sg->get_length()/units::cm << " " << sg->get_medium_dQ_dx() << std::endl;
	  
	  if (map_vertex_segments[pair_vertices.first].size()==2){
	    WCPPID::ProtoSegment *tmp_sg = *map_vertex_segments[pair_vertices.first].begin();
	    if (tmp_sg == sg) tmp_sg = *map_vertex_segments[pair_vertices.first].rbegin();
	    if (tmp_sg->get_particle_type()==13 && tmp_sg->get_length() > 4*length && length < 8*units::cm){
	      flag_change = true;
	    }
	      //	    std::cout << tmp_sg->get_length()/units::cm << " " << tmp_sg->get_id() << std::endl;
	  }else if (map_vertex_segments[pair_vertices.second].size()==2){
	    WCPPID::ProtoSegment *tmp_sg = *map_vertex_segments[pair_vertices.second].begin();
	    if (tmp_sg == sg) tmp_sg = *map_vertex_segments[pair_vertices.second].rbegin();
	    if (tmp_sg->get_particle_type()==13 && tmp_sg->get_length() > 4* length && length < 8*units::cm){
	      flag_change = true;
	    }
	    //std::cout << tmp_sg->get_length()/units::cm << " " << tmp_sg->get_id() << std::endl;
	  }
	  if (flag_change){
	    if (flag_print) std::cout << "G: " << sg->get_id() << std::endl;
	    sg->set_particle_type(11);
	    TPCParams& mp = Singleton<TPCParams>::Instance();
	    sg->set_particle_mass(mp.get_mass_electron());
	    if (sg->get_particle_4mom(3)>0) sg->cal_4mom();
	    flag_update = true;
	  }
	}

	
	if (  (nshowers[0]+1 == map_vertex_segments[two_vertices.first].size() || nshowers[0]>0) && // one shower or nothing
	      (nshowers[1]+1 == map_vertex_segments[two_vertices.second].size() || nshowers[1] >0) && // one shower or nothing
	      (nshowers[0] + nshowers[1] >2) && // 3 showers in total ...
	      (nshowers[0]+1 == map_vertex_segments[two_vertices.first].size() && nshowers[0] >0 ||
	       nshowers[1]+1 == map_vertex_segments[two_vertices.second].size() && nshowers[1] > 0) // one side all showers
	      ){
	  
	  if (length < 25*units::cm && sg->get_particle_type()!=11 || sg->get_flag_dir()==0){ // too long ...
	    bool flag_start;
	    if (sg->get_wcpt_vec().front().index == two_vertices.first->get_wcpt().index)
	      flag_start = true;
	    else if (sg->get_wcpt_vec().back().index == two_vertices.first->get_wcpt().index)
	      flag_start = false;
	    if (flag_start){
	      if (nshowers[1]==0){
		sg->set_flag_dir(-1);
	      }else if (nshowers[0]==0){
		sg->set_flag_dir(1);
	      }
	    }else{
	      if (nshowers[1]==0){
		sg->set_flag_dir(1);
	      }else if (nshowers[0]==0){
		sg->set_flag_dir(-1);
	      }
	    }
	    sg->set_dir_weak(true);
	    if (flag_print) std::cout << "B: " << sg->get_id() << std::endl;
	    sg->set_particle_type(11);
	    TPCParams& mp = Singleton<TPCParams>::Instance();
	    sg->set_particle_mass(mp.get_mass_electron());
	    if (sg->get_particle_4mom(3)>0) sg->cal_4mom();
	    flag_update = true;
	  }	  
	}else if (sg->get_particle_type()==0 && length < 12*units::cm && nshowers[0]  + nshowers[1] > 0 && sg->get_medium_dQ_dx()/(43e3/units::cm) > 1.2){

	  bool flag_change = false;
	  
	  auto pair_result1 = calculate_num_daughter_showers(two_vertices.second, sg);
	  auto pair_result2 = calculate_num_daughter_showers(two_vertices.first, sg);
	  
	  if (pair_result1.first >2){
	    TVector3 v1 = sg->cal_dir_3vector(two_vertices.first->get_fit_pt(), 10*units::cm);
	    double min_angle = 180;
	    double para_angle = 90;
	    for (auto it2 = map_vertex_segments[two_vertices.first].begin(); it2 != map_vertex_segments[two_vertices.first].end(); it2++){
	      if (*it2 == sg || (!(*it2)->get_flag_shower())) continue;
	      TVector3 v2 = (*it2)->cal_dir_3vector(two_vertices.first->get_fit_pt(), 10*units::cm);
	      double angle = fabs(v1.Angle(v2)/3.1415926*180.-180.);
	      if (angle < min_angle) {
		//std::cout << sg->get_id() << " " << (*it2)->get_id() << " " << angle << std::endl;
		min_angle = angle;
		para_angle = fabs(v2.Angle(drift_dir)/3.1415926*180.-90);
	      }
	    }
	    
	    if (min_angle < 25 || fabs(v1.Angle(drift_dir)/3.1415926*180.-90)< 10 && para_angle < 30 && min_angle < 45) flag_change = true; 

	    /* std::cout << sg->get_id() << " "<< sg->get_length()/units::cm << " " << pair_result1.first << " "  << pair_result1.second/units::cm << " " << sg->get_medium_dQ_dx()/(43e3/units::cm) << " " << min_angle << " " << fabs(v1.Angle(drift_dir)/3.1415926*180.-90) << " " << para_angle << " " << std::endl; */
	  }
	  
	  if (!flag_change){
	  
	     if (pair_result2.first >2){
	       TVector3 v1 = sg->cal_dir_3vector(two_vertices.second->get_fit_pt(), 10*units::cm);
	       double min_angle = 180;
	       double para_angle = 90;
	       for (auto it2 = map_vertex_segments[two_vertices.second].begin(); it2 != map_vertex_segments[two_vertices.second].end(); it2++){
		 if (*it2 == sg || (!(*it2)->get_flag_shower())) continue;
		 TVector3 v2 = (*it2)->cal_dir_3vector(two_vertices.second->get_fit_pt(), 10*units::cm);
		 double angle = fabs(v1.Angle(v2)/3.1415926*180.-180.);
		 if (angle < min_angle) {
		   min_angle = angle;
		   para_angle = fabs(v2.Angle(drift_dir)/3.1415926*180.-90);
		 }
	       }
	       if (min_angle < 25 || fabs(v1.Angle(drift_dir)/3.1415926*180.-90)<10 && para_angle < 10 && min_angle < 45) flag_change = true;

	       //    std::cout << sg->get_id() << " "<< sg->get_length()/units::cm << " " << pair_result1.first << " "  << pair_result1.second/units::cm << " " << sg->get_medium_dQ_dx()/(43e3/units::cm) << " " << min_angle << " " << fabs(v1.Angle(drift_dir)/3.1415926*180.-90) << " " << para_angle << " " << std::endl; 
	       
	     }
	  }
	
	  //	  std::cout << sg->get_id() << " "<< sg->get_length()/units::cm << " " << pair_result1.first << " " << pair_result2.first << " " << pair_result1.second/units::cm << " " << pair_result2.second/units::cm << " " << sg->get_medium_dQ_dx()/(43e3/units::cm) << " " << flag_change << std::endl;

	  if (flag_change){
	    if (flag_print) std::cout << "H: " << sg->get_id() << std::endl;
	    sg->set_particle_type(11);
	    TPCParams& mp = Singleton<TPCParams>::Instance();
	    sg->set_particle_mass(mp.get_mass_electron());
	    if (sg->get_particle_4mom(3)>0) sg->cal_4mom();
	    flag_update = true;
	  }
	  
	} // end of judgement ...


	
      } // no direction or weak ...
    } // loop over all segments
  } // keep updating
  

 
}


void WCPPID::NeutrinoID::change_daughter_type(WCPPID::ProtoVertex *vtx, WCPPID::ProtoSegment *sg, int particle_type, double mass){
  WCPPID::ProtoVertex *other_vtx = find_other_vertex(sg, vtx);
  TVector3 dir1 = sg->cal_dir_3vector(other_vtx->get_fit_pt(), 15*units::cm);
  
  for (auto it = map_vertex_segments[other_vtx].begin(); it != map_vertex_segments[other_vtx].end(); it++){
    WCPPID::ProtoSegment *sg1 = *it;
    if (sg1 == sg) continue;
    if (sg1->get_particle_type() == particle_type) continue;
    if ( sg1->get_flag_shower_trajectory() || (!sg1->is_dir_weak()) && sg1->get_flag_dir() !=0) continue;
    if (sg1->get_flag_shower_topology() && sg1->get_flag_dir() == 0 && sg1->get_length() > 40*units::cm){
      TVector3 dir2 = sg1->cal_dir_3vector(other_vtx->get_fit_pt(), 40*units::cm);
      //      std::cout <<dir1.Angle(dir2)/3.1415926*180. << " " << sg1->get_id() << std::endl;
      if (dir1.Angle(dir2)/3.1415926*180. > 170){
	sg1->set_particle_type(particle_type);
	sg1->set_particle_mass(mass);
	sg1->set_flag_shower_topology(false);
	change_daughter_type(other_vtx, sg1, particle_type, mass);
	change_daughter_type(find_other_vertex(sg1, other_vtx), sg1, particle_type, mass);
      }
    }else continue;
    
    if (sg1->get_length() > 10*units::cm){
      TVector3 dir2 = sg1->cal_dir_3vector(other_vtx->get_fit_pt(), 15*units::cm);
      if (dir1.Angle(dir2)/3.1415926*180. > 165){
	sg1->set_particle_type(particle_type);
	sg1->set_particle_mass(mass);
	change_daughter_type(other_vtx, sg1, particle_type, mass);
	change_daughter_type(find_other_vertex(sg1, other_vtx), sg1, particle_type, mass);
      }
      //      std::cout << dir1.Angle(dir2)/3.1415926*180. << std::endl;
    }
  }
}

std::pair<int, double> WCPPID::NeutrinoID::calculate_num_daughter_showers(WCPPID::ProtoVertex *vtx, WCPPID::ProtoSegment *sg, bool flag_count_shower){
  int number_showers = 0;
  double acc_length = 0;
  
  std::set<WCPPID::ProtoVertex* > used_vertices;
  std::set<WCPPID::ProtoSegment* > used_segments;

  std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > segments_to_be_examined;
  segments_to_be_examined.push_back(std::make_pair(vtx, sg));
  used_vertices.insert(vtx);

  while(segments_to_be_examined.size()>0){
    std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > temp_segments;
    for (auto it = segments_to_be_examined.begin(); it!= segments_to_be_examined.end(); it++){
      WCPPID::ProtoVertex *prev_vtx = it->first;
      WCPPID::ProtoSegment *current_sg = it->second;
      if (used_segments.find(current_sg)!=used_segments.end()) continue; // looked at it before ...
      if (current_sg->get_flag_shower() || (!flag_count_shower)){
	  number_showers ++;
	  acc_length += current_sg->get_length();
      }
      used_segments.insert(current_sg);

      WCPPID::ProtoVertex* curr_vertex = find_other_vertex(current_sg, prev_vtx);
      if (used_vertices.find(curr_vertex) != used_vertices.end()) continue;
      for (auto it1 = map_vertex_segments[curr_vertex].begin(); it1!= map_vertex_segments[curr_vertex].end(); it1++){
	temp_segments.push_back(std::make_pair(curr_vertex, *it1));
      }
      used_vertices.insert(curr_vertex);
    }
    segments_to_be_examined = temp_segments;
  }

  return std::make_pair(number_showers, acc_length);
}

std::pair<int, double> WCPPID::NeutrinoID::calculate_num_daughter_tracks(WCPPID::ProtoVertex *vtx, WCPPID::ProtoSegment *sg, bool flag_count_shower, double length_cut){
  int number_tracks = 0;
  double acc_length = 0;
  
  std::set<WCPPID::ProtoVertex* > used_vertices;
  std::set<WCPPID::ProtoSegment* > used_segments;

  std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > segments_to_be_examined;
  segments_to_be_examined.push_back(std::make_pair(vtx, sg));
  used_vertices.insert(vtx);

  while(segments_to_be_examined.size()>0){
    std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > temp_segments;
    for (auto it = segments_to_be_examined.begin(); it!= segments_to_be_examined.end(); it++){
      WCPPID::ProtoVertex *prev_vtx = it->first;
      WCPPID::ProtoSegment *current_sg = it->second;
      if (used_segments.find(current_sg)!=used_segments.end()) continue; // looked at it before ...


      if ((!current_sg->get_flag_shower()) || flag_count_shower){
	double length = current_sg->get_length();
	if (length > length_cut){
	  acc_length += length;
	  number_tracks ++;
	}
      }
      used_segments.insert(current_sg);

      WCPPID::ProtoVertex* curr_vertex = find_other_vertex(current_sg, prev_vtx);
      if (used_vertices.find(curr_vertex) != used_vertices.end()) continue;
      for (auto it1 = map_vertex_segments[curr_vertex].begin(); it1!= map_vertex_segments[curr_vertex].end(); it1++){
	temp_segments.push_back(std::make_pair(curr_vertex, *it1));
      }
      used_vertices.insert(curr_vertex);
    }
    segments_to_be_examined = temp_segments;
  }

  return std::make_pair(number_tracks, acc_length);
}


void WCPPID::NeutrinoID::fix_maps_multiple_tracks_in(int temp_cluster_id){
  for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end();it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() != temp_cluster_id) continue;
    if (it->second.size()==1) continue;

    int n_in = 0;
    int n_in_shower = 0;
    std::vector<WCPPID::ProtoSegment* > in_tracks;
    
    for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
      WCPPID::ProtoSegment *sg = (*it1);
      bool flag_start;
      if (sg->get_wcpt_vec().front().index == vtx->get_wcpt().index)
	flag_start = true;
      else if (sg->get_wcpt_vec().back().index == vtx->get_wcpt().index)
	flag_start = false;
      
      if (flag_start && sg->get_flag_dir()==-1 || (!flag_start) && sg->get_flag_dir()==1){
	n_in ++;
	if (sg->get_flag_shower()) n_in_shower ++;
	else in_tracks.push_back(sg);
      }
    }

    if (n_in > 1 && n_in != n_in_shower) {
      for (auto it1 = in_tracks.begin(); it1 != in_tracks.end(); it1++){
	(*it1)->set_flag_dir(0);
	(*it1)->set_dir_weak(true);
      }
    }
  }
}

void WCPPID::NeutrinoID::fix_maps_shower_in_track_out(int temp_cluster_id){

  for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end();it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() != temp_cluster_id) continue;
    if (it->second.size()==1) continue;
    
    std::vector<WCPPID::ProtoSegment*> in_showers;
    bool flag_turn_shower_dir = false;
    
    for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
      WCPPID::ProtoSegment *sg = (*it1);
      bool flag_start;
      if (sg->get_wcpt_vec().front().index == vtx->get_wcpt().index)
	flag_start = true;
      else if (sg->get_wcpt_vec().back().index == vtx->get_wcpt().index)
	flag_start = false;
      if (flag_start && sg->get_flag_dir()==-1 || (!flag_start) && sg->get_flag_dir()==1){
	if (sg->get_flag_shower() ) in_showers.push_back(sg);
      }else if (flag_start && sg->get_flag_dir()==1 || (!flag_start) && sg->get_flag_dir()==-1 ){
	if ((!sg->get_flag_shower()) ) {
	  if (!sg->is_dir_weak()) flag_turn_shower_dir = true;
	}
      }
    } // loop segment

    if (flag_turn_shower_dir){
      for (auto it1 = in_showers.begin(); it1!= in_showers.end(); it1++){
	(*it1)->set_flag_dir((*it1)->get_flag_dir() * (-1));
	(*it1)->set_dir_weak(true);
      }
    }
    
  } // loop over all vertices
}

void WCPPID::NeutrinoID::improve_maps_multiple_tracks_in(int temp_cluster_id){
  bool flag_update = true;
  std::set<WCPPID::ProtoVertex* > used_vertices;
  std::set<WCPPID::ProtoSegment* > used_segments;
  while(flag_update){
    flag_update = false;
  
    for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end();it++){
      WCPPID::ProtoVertex *vtx = it->first;
      if (vtx->get_cluster_id() != temp_cluster_id) continue;
      if (it->second.size()==1) continue;
      if (used_vertices.find(vtx)!=used_vertices.end()) continue;
      
      int n_in = 0;
      int n_in_shower = 0;
      std::vector<ProtoSegment* > in_tracks;
      for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	WCPPID::ProtoSegment *sg = (*it1);
	bool flag_start;
	if (sg->get_wcpt_vec().front().index == vtx->get_wcpt().index)
	  flag_start = true;
	else if (sg->get_wcpt_vec().back().index == vtx->get_wcpt().index)
	  flag_start = false;
	if (flag_start && sg->get_flag_dir()==-1 || (!flag_start) && sg->get_flag_dir()==1){
	  n_in ++;
	  if (sg->get_flag_shower() ) n_in_shower ++;
	  else in_tracks.push_back(sg);
	}
      }
      
      
      if (n_in > 1 && n_in !=n_in_shower){
	for (auto it1 = in_tracks.begin(); it1!=in_tracks.end(); it1++){
	  WCPPID::ProtoSegment *sg1 = *it1;
	  sg1->set_particle_type(11);
	  TPCParams& mp = Singleton<TPCParams>::Instance();
	  sg1->set_particle_mass(mp.get_mass_electron());
	  if (sg1->get_particle_4mom(3)>0) sg1->cal_4mom();
	  flag_update = true;
	}
      }
      used_vertices.insert(vtx);
      
    } // loop over everything
  } // while
}

void WCPPID::NeutrinoID::improve_maps_shower_in_track_out(int temp_cluster_id, bool flag_strong_check){
  bool flag_update = true;
  std::set<WCPPID::ProtoVertex* > used_vertices;
  std::set<WCPPID::ProtoSegment* > used_segments;
  while(flag_update){
    flag_update = false;
  
    for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end();it++){
      WCPPID::ProtoVertex *vtx = it->first;
      if (vtx->get_cluster_id() != temp_cluster_id) continue;
      if (it->second.size()==1) continue;
      if (used_vertices.find(vtx)!=used_vertices.end()) continue;
      
      int n_in = 0;
      int n_in_shower = 0;
      std::vector<WCPPID::ProtoSegment*> out_tracks;
      std::map<WCPPID::ProtoSegment*, bool> map_no_dir_segments;

      //      bool flag_print = false;
      for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	WCPPID::ProtoSegment *sg = (*it1);
	bool flag_start;
	if (sg->get_wcpt_vec().front().index == vtx->get_wcpt().index)
	  flag_start = true;
	else if (sg->get_wcpt_vec().back().index == vtx->get_wcpt().index)
	  flag_start = false;
	if (flag_start && sg->get_flag_dir()==-1 || (!flag_start) && sg->get_flag_dir()==1){
	  n_in ++;
	  if (sg->get_flag_shower() ) {
	    n_in_shower ++;
	  }
	}else if (flag_start && sg->get_flag_dir()==1 || (!flag_start) && sg->get_flag_dir()==-1 ){
	  if ((!sg->get_flag_shower()) ) {
	    if (sg->is_dir_weak() || (sg->get_particle_type()==0 && (!flag_strong_check)) ) out_tracks.push_back(sg);
	  }
	}else if (sg->get_flag_dir()==0){
	  map_no_dir_segments[sg] = flag_start;
	}
	//if (sg->get_id()==55) flag_print = true;
      }

      //      if (flag_print)
      // std::cout  << n_in_shower << " " << out_tracks.size() << " " << map_no_dir_segments.size() << std::endl;
      
      if (n_in_shower >0 && (out_tracks.size() >0 || map_no_dir_segments.size() >0 )) {
	for (auto it1 = out_tracks.begin(); it1!=out_tracks.end(); it1++){
	  WCPPID::ProtoSegment *sg1 = *it1;
	  sg1->set_particle_type(11);
	  sg1->set_flag_dir(0);
	  TPCParams& mp = Singleton<TPCParams>::Instance();
	  sg1->set_particle_mass(mp.get_mass_electron());
	  if (sg1->get_particle_4mom(3)>0) sg1->cal_4mom();
	  flag_update = true;
	}
	for (auto it1 = map_no_dir_segments.begin(); it1 != map_no_dir_segments.end(); it1++){
	  WCPPID::ProtoSegment *sg1 = it1->first;
	  if (used_segments.find(sg1)!=used_segments.end()) continue;
	  //bool flag_start = it1->second;
	  //if (flag_start){
	    //	    sg1->set_flag_dir(1);
	  // }else{
	    // sg1->set_flag_dir(-1);
	  //}
	  if (!sg1->get_flag_shower()){
	    sg1->set_particle_type(11);
	    TPCParams& mp = Singleton<TPCParams>::Instance();
	    sg1->set_particle_mass(mp.get_mass_electron());
	  }
	  if (sg1->get_particle_4mom(3)>0) sg1->cal_4mom();
	  sg1->set_dir_weak(true);
	  used_segments.insert(sg1);
	  flag_update = true;
	}
      } // correct other directions
      used_vertices.insert(vtx);
      
    } // loop over everything
  } // while
}

void WCPPID::NeutrinoID::print_segs_info(int temp_cluster_id, WCPPID::ProtoVertex* spec_vertex){
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != temp_cluster_id) continue;
    if (spec_vertex != 0){
      if (map_vertex_segments[spec_vertex].find(sg)==map_vertex_segments[spec_vertex].end()) continue;
    }
    // print in and out ...
    int in_vertex = 0; // no direction, -1 in, 1 out ...

    

    if (spec_vertex !=0){
      bool flag_start;
      if (sg->get_wcpt_vec().front().index == spec_vertex->get_wcpt().index)
	flag_start = true;
      else if (sg->get_wcpt_vec().back().index == spec_vertex->get_wcpt().index)
	flag_start = false;
      else std::cout << "Something messed up!" << std::endl;
      if (flag_start && sg->get_flag_dir()==-1  || (!flag_start)&& sg->get_flag_dir()==1){
	in_vertex = -1;
      }else if (flag_start && sg->get_flag_dir()==1  || (!flag_start)&& sg->get_flag_dir()==-1){
	in_vertex = 1;
      }
    }
    
    
    if (sg->get_flag_shower_topology()){
      std::cout << sg->get_id() << " " << sg->get_length()/units::cm << " S_topo "  << sg->get_flag_dir() << " " << sg->get_particle_type() << " " << sg->get_particle_mass()/units::MeV << " " << (sg->get_particle_4mom(3)-sg->get_particle_mass())/units::MeV << " " << sg->is_dir_weak() << " " << in_vertex << std::endl;
    }else if (sg->get_flag_shower_trajectory()){
      std::cout << sg->get_id() << " " << sg->get_length()/units::cm << " S_traj "  << sg->get_flag_dir() << " " << sg->get_particle_type() << " " << sg->get_particle_mass()/units::MeV << " " << (sg->get_particle_4mom(3)-sg->get_particle_mass())/units::MeV << " " << sg->is_dir_weak()<< " " << in_vertex << std::endl;
    }else{
      std::cout << sg->get_id() << " " << sg->get_length()/units::cm << " Track  "  << sg->get_flag_dir() << " " << sg->get_particle_type() << " " << sg->get_particle_mass()/units::MeV << " " << (sg->get_particle_4mom(3)-sg->get_particle_mass())/units::MeV << " " << sg->is_dir_weak() << " " << in_vertex <<  std::endl;
    }
  }
}

void WCPPID::NeutrinoID::print_segs_info(WCPPID::ProtoVertex* temp_vertex){
  print_segs_info(temp_vertex->get_cluster_id());
}
void WCPPID::NeutrinoID::print_segs_info(WCPPID::PR3DCluster* temp_cluster){
  print_segs_info(temp_cluster->get_cluster_id());
}

void WCPPID::NeutrinoID::examine_all_showers(WCPPID::PR3DCluster* temp_cluster){

  //  print_segs_info(temp_cluster);


  TPCParams& mp = Singleton<TPCParams>::Instance();
  int n_good_tracks = 0, n_tracks = 0, n_showers = 0;
  double length_good_tracks = 0, length_tracks = 0, length_showers = 0;
  double tracks_score = 0;
  WCPPID::ProtoSegment *good_track = 0;

  double maximal_length = 0;
  WCPPID::ProtoSegment *maximal_length_track = 0;
  
  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != temp_cluster->get_cluster_id()) continue;
    double length = sg->get_length();
    if (sg->get_flag_shower() ){
      n_showers ++;
      length_showers += length;
    }else{
      if (sg->get_flag_dir()!=0 && (!sg->is_dir_weak())){
	good_track = sg;
	n_good_tracks ++;
	length_good_tracks += length;
      }else{
	n_tracks ++;
	length_tracks += length;
	if (length > maximal_length){
	  maximal_length = length;
	  maximal_length_track = sg;
	}
	if (sg->get_particle_score()!=100) tracks_score += sg->get_particle_score();
      }
    }
  }

  if (n_good_tracks + n_tracks + n_showers == 1) return;
  
  // if there is only one good track ...
  if (n_good_tracks ==1 && (length_good_tracks < 0.15 * (length_showers + length_tracks) ) && length_good_tracks < 10*units::cm){
    auto pair_vertices = find_vertices(good_track);

    int num_s1 = 0, num_s2 = 0;
    double length_s1 = 0, length_s2 = 0;
    auto pair_result1 = calculate_num_daughter_showers(pair_vertices.first, good_track);
    auto pair_result2 = calculate_num_daughter_showers(pair_vertices.second, good_track);
    num_s1 = pair_result1.first;
    length_s1 = pair_result1.second;
    num_s2 = pair_result2.first;
    length_s2 = pair_result2.second;
    
    if (num_s1 > 0 && length_s1 > length_good_tracks){
      double max_angle = 0;
      TVector3 dir1 = good_track->cal_dir_3vector(pair_vertices.second->get_fit_pt(), 15*units::cm);
      for (auto it1 = map_vertex_segments[pair_vertices.second].begin(); it1!= map_vertex_segments[pair_vertices.second].end(); it1++){
	if (*it1 == good_track) continue;
	//if (! (*it1)->get_flag_shower()) continue;
	TVector3 dir2 = (*it1)->cal_dir_3vector(pair_vertices.second->get_fit_pt(), 15*units::cm);
	double  angle = dir1.Angle(dir2)/3.1415926*180;
	if (max_angle < angle) max_angle = angle;
      }

      //      std::cout << max_angle << " " << length_good_tracks/units::cm << " " << length_showers/units::cm << std::endl;

      if (max_angle > 165 || max_angle > 150 && length_good_tracks < 3.0*units::cm && length_good_tracks < 0.1 * length_showers ) {
	n_good_tracks = 0;
	length_tracks += length_good_tracks;
      }
    }

    if (num_s2 >0 && length_s2 > length_good_tracks && n_good_tracks >0){
      double max_angle = 0;
      TVector3 dir1 = good_track->cal_dir_3vector(pair_vertices.first->get_fit_pt(), 15*units::cm);
      for (auto it1 = map_vertex_segments[pair_vertices.first].begin(); it1!= map_vertex_segments[pair_vertices.first].end(); it1++){
	if (*it1 == good_track) continue;
	//if (! (*it1)->get_flag_shower()) continue;
	TVector3 dir2 = (*it1)->cal_dir_3vector(pair_vertices.first->get_fit_pt(), 15*units::cm);
	double  angle = dir1.Angle(dir2)/3.1415926*180;
	if (max_angle < angle) max_angle = angle;
      }
      // std::cout << max_angle << std::endl;
      if (max_angle > 165) {
	n_good_tracks = 0;
	length_tracks += length_good_tracks;
      }
    }

    if (map_vertex_segments[pair_vertices.first].size() == 1 && map_vertex_segments[pair_vertices.second].size() > 1){
      double max_length = 0;
      WCPPID::ProtoSegment *max_segment = 0;
      for (auto it = map_vertex_segments[pair_vertices.second].begin(); it != map_vertex_segments[pair_vertices.second].end(); it++){
	WCPPID::ProtoSegment *sg = *it;
	if (sg->get_flag_shower()){
	  double length = sg->get_length();
	  if (length > max_length){
	    max_length = length;
	    max_segment = sg;
	  }
	}
      }
      if (max_segment !=0 && max_length > 5*units::cm){
	TVector3 dir = max_segment->cal_dir_3vector(pair_vertices.second->get_fit_pt(), 15*units::cm);
	TVector3 beam_dir(0,0,1);
	if (beam_dir.Angle(dir)/3.1415926*180. > 90) {
	  n_good_tracks = 0;
	  length_tracks += length_good_tracks;
	}
	//	std::cout << beam_dir.Angle(dir)/3.1415926*180. << " " << max_length/units::cm << std::endl;
      }
    }else if ( map_vertex_segments[pair_vertices.first].size() > 1 && map_vertex_segments[pair_vertices.second].size() == 1){

      double max_length = 0;
      WCPPID::ProtoSegment *max_segment = 0;
      for (auto it = map_vertex_segments[pair_vertices.first].begin(); it != map_vertex_segments[pair_vertices.first].end(); it++){
	WCPPID::ProtoSegment *sg = *it;
	if (sg->get_flag_shower()){
	  double length = sg->get_length();
	  if (length > max_length){
	    max_length = length;
	    max_segment = sg;
	  }
	}
      }
      if (max_segment !=0 && max_length > 5*units::cm){
	TVector3 dir = max_segment->cal_dir_3vector(pair_vertices.first->get_fit_pt(), 15*units::cm);
	TVector3 beam_dir(0,0,1);
	if (beam_dir.Angle(dir)/3.1415926*180. > 90) {
	  n_good_tracks = 0;
	  length_tracks += length_good_tracks;
	}
	//	std::cout << beam_dir.Angle(dir)/3.1415926*180. << " " << max_length/units::cm << std::endl;
      }
    }
  }else if (n_good_tracks == 0 && (n_tracks == 2 && length_tracks <= 35*units::cm) ){
    //    std::cout << temp_cluster->get_cluster_id() << " " << maximal_length_track << " " << n_tracks << " " << length_tracks << std::endl;
    if (maximal_length_track !=0){
      // change score ...
      auto pair_vertices = find_vertices(maximal_length_track);
      
      if (map_vertex_segments[pair_vertices.first].size() < map_vertex_segments[pair_vertices.second].size()){
	
	TVector3 dir = maximal_length_track->cal_dir_3vector(pair_vertices.second->get_fit_pt(), 15*units::cm);
	TVector3 beam_dir(0,0,1);
	if (beam_dir.Angle(dir)/3.1415926*180. > 100) {
	  n_tracks --;
	  length_tracks -= maximal_length;
	  n_showers ++;
	  length_showers += maximal_length;
	}
	//std::cout << beam_dir.Angle(dir)/3.1415926*180.  << std::endl;
      	
      }else if ( map_vertex_segments[pair_vertices.first].size() > map_vertex_segments[pair_vertices.second].size()){
	TVector3 dir = maximal_length_track->cal_dir_3vector(pair_vertices.first->get_fit_pt(), 15*units::cm);
	TVector3 beam_dir(0,0,1);
	if (beam_dir.Angle(dir)/3.1415926*180. > 90) {
	  n_tracks --;
	  length_tracks -= maximal_length;
	  n_showers ++;
	  length_showers += maximal_length;
	}
	//       std::cout << beam_dir.Angle(dir)/3.1415926*180. << " " << maximal_length << " " << pair_vertices.second->get_fit_pt() << " " << pair_vertices.first->get_fit_pt() << std::endl;
      }
    }
     
     
  }


  
  
  

  
  bool flag_change_showers = false;
  
  if (n_good_tracks == 0){
    if ( length_tracks < 1./3.*length_showers || length_tracks < 2./3.*length_showers && n_tracks == 1){
      if ( (length_showers + length_tracks) < 40*units::cm ){
	flag_change_showers = true;
      }else if (length_tracks < 0.18 * length_showers && ((length_showers + length_tracks) < 60*units::cm || length_tracks < 12*units::cm)){
	flag_change_showers = true;
      }else if (length_tracks < 0.25 *length_showers && (tracks_score ==0 && length_tracks < 30*units::cm || length_tracks < 10*units::cm)){
	flag_change_showers = true;
      }else if (n_tracks == 1 && tracks_score == 0 && length_tracks < 15*units::cm && length_tracks < 1./3. * length_showers){
	flag_change_showers = true;
      }
    }else if ( length_tracks < 35*units::cm &&  length_tracks + length_showers  < 50*units::cm && length_showers < 15*units::cm
	       && (temp_cluster != main_cluster ||
		   temp_cluster == main_cluster
		   && (length_showers > 0.5*length_tracks
		       || length_showers > 0.3*length_tracks && n_showers >=2
		       || n_showers ==1 && n_tracks==1 && length_showers > length_tracks * 0.3
		       || tracks_score == 0) )
	       ){
      flag_change_showers = true;
      if (length_showers == 0 && n_tracks<=2 && (temp_cluster == main_cluster || length_tracks >15*units::cm))	flag_change_showers = false;      
    }else if ( length_tracks < 35*units::cm &&  length_tracks + length_showers  < 50*units::cm && length_showers < 15*units::cm){
      flag_change_showers = true;
      for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
	WCPPID::ProtoSegment *sg = it->first;
	if (sg->get_cluster_id() != temp_cluster->get_cluster_id()) continue;
	if (!sg->get_flag_shower()){
	  auto pair_vertices = find_vertices(sg);
	  bool flag_shower = false;
	  for (auto it1 = map_vertex_segments[pair_vertices.first].begin(); it1!= map_vertex_segments[pair_vertices.first].end(); it1++){
	    if ( (*it1)->get_flag_shower()) flag_shower = true;
	  }
	  for (auto it1 = map_vertex_segments[pair_vertices.second].begin(); it1!= map_vertex_segments[pair_vertices.second].end(); it1++){
	    if ( (*it1)->get_flag_shower()) flag_shower = true;
	  }
	  if (!flag_shower) flag_change_showers = false;
	  //std::cout << flag_shower << std::endl;
	}
      } // loop over segment ...
    } // else if 
  } // no good tracks 


  
 
  
  //if (!flag_change_showers)
  //  std::cout << temp_cluster->get_cluster_id() << " " << n_good_tracks << " " << n_tracks << " " << n_showers << " " << length_good_tracks/units::cm << " " << length_tracks/units::cm << " " << length_showers/units::cm << " " << flag_change_showers << " " << tracks_score << std::endl;

  if (flag_change_showers){
    for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
      WCPPID::ProtoSegment *sg = it->first;
      if (sg->get_cluster_id() != temp_cluster->get_cluster_id()) continue;
      if (!sg->get_flag_shower()){
	sg->set_particle_type(11);
	sg->set_particle_mass(mp.get_mass_electron());
      }
    }
  }
  

  
}


void WCPPID::NeutrinoID::determine_main_vertex(WCPPID::PR3DCluster* temp_cluster, bool flag_print){
  const bool flag_timing = true;
  using DMV_Clock = std::chrono::high_resolution_clock;
  using DMV_ms = std::chrono::duration<double, std::milli>;
  auto dmv_t_total = DMV_Clock::now();
  auto dmv_t0 = DMV_Clock::now();
  std::cout << "[determine_main_vertex] Cluster: " << temp_cluster->get_cluster_id() << " Start -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;

  // update directions ...
  // improve_maps_one_in(temp_cluster);
  // examination ...
  // examine_maps(temp_cluster);
  // print ...

  //  std::cout << "Information after initial logic examination: " << std::endl;
  // print_segs_info(temp_cluster);


  // find the main vertex ...
  bool flag_save_only_showers = true;
  for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() != temp_cluster->get_cluster_id()) continue;
    auto results = examine_main_vertex_candidate(vtx);
    bool flag_in = std::get<0>(results);
    int ntracks = std::get<1>(results), nshowers = std::get<2>(results);
    if (!flag_in){
      if (ntracks > 0) {
	flag_save_only_showers = false;
	break;
      }
    }
  }
  if (flag_timing) std::cout << "determine_main_vertex timing: scan for flag_save_only_showers took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;

  if ((!flag_save_only_showers) && temp_cluster == main_cluster)  {
    dmv_t0 = DMV_Clock::now();
    improve_vertex(temp_cluster, false);
    if (flag_timing) std::cout << "determine_main_vertex timing: improve_vertex took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;
    // if one shower in and a good track out, reverse the shower ..
    dmv_t0 = DMV_Clock::now();
    fix_maps_shower_in_track_out(temp_cluster->get_cluster_id());
    if (flag_timing) std::cout << "determine_main_vertex timing: fix_maps_shower_in_track_out took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;
  }


  dmv_t0 = DMV_Clock::now();
  std::map<ProtoVertex*, std::pair<int, int> > map_vertex_track_shower;
  WCPPID::ProtoVertexSelection main_vertex_candidates;
  for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() != temp_cluster->get_cluster_id()) continue;

    auto results = examine_main_vertex_candidate(vtx);
    bool flag_in = std::get<0>(results);
    int ntracks = std::get<1>(results), nshowers = std::get<2>(results);

    if (!flag_in){
      map_vertex_track_shower[vtx] = std::make_pair(ntracks, nshowers);
    }
  }
  if (flag_timing) std::cout << "determine_main_vertex timing: build map_vertex_track_shower took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;

  dmv_t0 = DMV_Clock::now();
  if (flag_save_only_showers){
    for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
      if (it->first->get_cluster_id() != temp_cluster->get_cluster_id()) continue;
      if (it->second.size()==1){
	main_vertex_candidates.push_back(it->first);
      }
    }
    for (auto it = map_vertex_track_shower.begin(); it!= map_vertex_track_shower.end(); it++){
      if (find(main_vertex_candidates.begin(), main_vertex_candidates.end(), it->first) == main_vertex_candidates.end())
	main_vertex_candidates.push_back(it->first);
    }
  }else{
    for (auto it = map_vertex_track_shower.begin(); it!= map_vertex_track_shower.end(); it++){
      if (it->second.first >0) main_vertex_candidates.push_back(it->first);
    }
  }
  if (flag_timing) std::cout << "determine_main_vertex timing: build main_vertex_candidates took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;


  map_cluster_main_candidate_vertices[temp_cluster] = main_vertex_candidates;

  dmv_t0 = DMV_Clock::now();
  if (flag_save_only_showers){
    if (main_vertex_candidates.size()>0){
      if (flag_print)
	std::cout << "Determining the main vertex with all showers: " << main_vertex_candidates.size() << " in cluster " << main_vertex_candidates.front()->get_cluster_id() << std::endl;
      main_vertex = compare_main_vertices_all_showers(main_vertex_candidates, temp_cluster);
      std::cout << "[determine_main_vertex] Cluster: " << temp_cluster->get_cluster_id() << " Set main_vertex (all-showers): id=" << main_vertex->get_id() << " at " << main_vertex->get_fit_pt() << std::endl;
      if (flag_timing) std::cout << "determine_main_vertex timing: compare_main_vertices_all_showers took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;
    }else{
      return;
    }
  }else{
    examine_main_vertices(main_vertex_candidates);
    if (flag_timing) std::cout << "determine_main_vertex timing: examine_main_vertices took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;
    if (flag_print){
      //  std::cout << main_vertex_candidates.size() << std::endl;
      for (auto it = main_vertex_candidates.begin(); it!= main_vertex_candidates.end(); it++){
	std::cout << "Candidate main vertex " << (*it)->get_fit_pt() << " connecting to: ";
	for (auto it1 = map_vertex_segments[*it].begin(); it1!=map_vertex_segments[*it].end(); it1++){
	  std::cout << (*it1)->get_id() << ", ";
	}
	std::cout << " in cluster " << (*it)->get_cluster_id() << std::endl;
      }
    }

    dmv_t0 = DMV_Clock::now();
    if (main_vertex_candidates.size()==1){
      main_vertex = main_vertex_candidates.front();
      std::cout << "[determine_main_vertex] Cluster: " << temp_cluster->get_cluster_id() << " Set main_vertex (single candidate): id=" << main_vertex->get_id() << " at " << main_vertex->get_fit_pt() << std::endl;
    }else if (main_vertex_candidates.size()>1){
      main_vertex = compare_main_vertices(main_vertex_candidates);
      std::cout << "[determine_main_vertex] Cluster: " << temp_cluster->get_cluster_id() << " Set main_vertex: id=" << main_vertex->get_id() << " at " << main_vertex->get_fit_pt() << std::endl;
    }else{
      return;
    }
    if (flag_timing) std::cout << "determine_main_vertex timing: compare_main_vertices took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;
  }


  if (!flag_save_only_showers){
    // examine structure before examine directions ??? ...
    dmv_t0 = DMV_Clock::now();
    examine_structure_final(temp_cluster);
    if (flag_timing) std::cout << "determine_main_vertex timing: examine_structure_final took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;
  }

  dmv_t0 = DMV_Clock::now();
  bool flag_check = examine_direction(main_vertex);
  if (flag_timing) std::cout << "determine_main_vertex timing: examine_direction took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t0).count() << " ms" << std::endl;
  if (!flag_check) std::cout << "Wrong: inconsistency for track directions in cluster " << main_vertex->get_cluster_id() << std::endl;
  if (flag_print){
    std::cout << "Main Vertex " << main_vertex->get_fit_pt() << " connecting to: ";
    for (auto it = map_vertex_segments[main_vertex].begin(); it!=map_vertex_segments[main_vertex].end(); it++){
      std::cout << (*it)->get_id() << ", ";
    }
    std::cout << std::endl;
    print_segs_info(main_vertex->get_cluster_id(), main_vertex);
  }

  if (flag_timing) std::cout << "determine_main_vertex timing: TOTAL took " << std::chrono::duration_cast<DMV_ms>(DMV_Clock::now()-dmv_t_total).count() << " ms" << std::endl;

  // std::cout << "Information after main vertex determination: " << std::endl;
  // print_segs_info(main_vertex);

}

std::tuple<bool, int, int> WCPPID::NeutrinoID::examine_main_vertex_candidate(WCPPID::ProtoVertex *vertex){
  bool flag_in = false;
  int ntracks =0;
  int nshowers =0;
  WCPPID::ProtoSegment *shower_cand = 0;
  WCPPID::ProtoSegment *track_cand = 0;
  
  for (auto it1 = map_vertex_segments[vertex].begin(); it1!=map_vertex_segments[vertex].end(); it1++){
    WCPPID::ProtoSegment *sg = *it1;
    if (sg->get_flag_shower()) {
      nshowers ++;
      shower_cand = sg;
    } else{
      ntracks ++;
      track_cand = sg;
    }
    
    bool flag_start;
    if (sg->get_wcpt_vec().front().index == vertex->get_wcpt().index)
      flag_start = true;
    else if (sg->get_wcpt_vec().back().index == vertex->get_wcpt().index)
      flag_start = false;
    else std::cout << "Something messed up!" << std::endl;
    if (flag_start && sg->get_flag_dir()==-1  && (!sg->is_dir_weak()) ){
      flag_in = true;
      break;
    }else if ((!flag_start)&& sg->get_flag_dir()==1 &&(!sg->is_dir_weak()) ){
      flag_in = true;
      break;
    }
  }

  // check Michel electron case ...
  if (map_vertex_segments[vertex].size()==2 && ntracks ==1 && nshowers == 1){
    // calculate the number of daughter showers
    auto pair_result = calculate_num_daughter_showers(vertex, shower_cand);
    if (pair_result.first <=3 && pair_result.second < 30*units::cm){
      bool flag_start;
      if (track_cand->get_wcpt_vec().front().index == vertex->get_wcpt().index)
	flag_start = true;
      else if (track_cand->get_wcpt_vec().back().index == vertex->get_wcpt().index)
	flag_start = false;
      if (flag_start && track_cand->get_flag_dir()==-1 || (!flag_start)&& track_cand->get_flag_dir()==1)
	flag_in = true;
    }
  }
  
  return std::make_tuple(flag_in, ntracks, nshowers);
}


WCPPID::ProtoVertex* WCPPID::NeutrinoID::compare_main_vertices_all_showers(WCPPID::ProtoVertexSelection& vertex_candidates, WCPPID::PR3DCluster *temp_cluster){

  WCPPID::ProtoVertex *temp_main_vertex = vertex_candidates.front();
  PointVector pts;
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != temp_main_vertex->get_cluster_id()) continue;
    if (sg->get_point_vec().size()<=2){
    }else{
      for(size_t i = 1; i+1 < sg->get_point_vec().size(); i++){
	pts.push_back(sg->get_point_vec().at(i));
      }
    }
  }
  for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() != temp_main_vertex->get_cluster_id()) continue;
    pts.push_back(vtx->get_fit_pt());
  }
  
  if (pts.size() >3){
    auto pair_result = calc_PCA_main_axis(pts);
    TVector3 dir = pair_result.second;
    Point center = pair_result.first;
    dir = dir.Unit();

    double min_val = 1e9, max_val = -1e9;
    WCPPID::ProtoVertex *min_vtx = 0, *max_vtx = 0;
    for (auto it = vertex_candidates.begin(); it!=vertex_candidates.end(); it++){
      double val = ((*it)->get_fit_pt().x - center.x) * dir.X() + ((*it)->get_fit_pt().y - center.y) * dir.Y() + ((*it)->get_fit_pt().z - center.z) * dir.Z();
      if (map_vertex_segments[*it].size()==1 && (*map_vertex_segments[*it].begin())->get_length()<1*units::cm){
	if (val >0) val -= 0.5*units::cm;
	else if (val <0) val += 0.5*units::cm;
	//	std::cout << "gaga " << std::endl;
      }
      //  std::cout << (*it)->get_fit_pt() << " " << map_vertex_segments[*it].size() << std::endl;
	
      if (val > max_val){
	max_val = val;
	max_vtx = *it;
      }
      if (val < min_val){
	min_val = val;
	min_vtx = *it;
      }
    }

    //    std::cout << temp_cluster->get_point_cloud_steiner()->get_cloud().pts.size() << " " << max_vtx->get_wcpt().index << " " << min_vtx->get_wcpt().index << std::endl;
    ToyPointCloud* pcloud_steiner = temp_cluster->get_point_cloud_steiner();
    if (pcloud_steiner->get_cloud().pts.size()>2 && min_vtx != max_vtx){
      // Now create a fake segment and two fake vertices for track fitting ...
      //  std::cout << max_vtx << " " << min_vtx << " " << max_vtx->get_fit_pt() << " " << min_vtx->get_fit_pt() << std::endl;
      
      auto max_wcp = pcloud_steiner->get_closest_wcpoint(max_vtx->get_fit_pt());
      auto min_wcp = pcloud_steiner->get_closest_wcpoint(min_vtx->get_fit_pt());
      temp_cluster->dijkstra_shortest_paths(max_wcp,2);
      temp_cluster->cal_shortest_path(min_wcp,2);
      if ( temp_cluster->get_path_wcps().size() > 2){
	WCPPID::ProtoVertex *tmp_v1 = new WCPPID::ProtoVertex(-1, max_wcp, temp_cluster->get_cluster_id());
	WCPPID::ProtoVertex *tmp_v2 = new WCPPID::ProtoVertex(-2, min_wcp, temp_cluster->get_cluster_id());
	WCPPID::ProtoSegment *tmp_sg = new WCPPID::ProtoSegment(-1, temp_cluster->get_path_wcps(), temp_cluster->get_cluster_id());
	Map_Proto_Vertex_Segments tmp_map_vertex_segments;
	Map_Proto_Segment_Vertices tmp_map_segment_vertices;
	tmp_map_vertex_segments[tmp_v1].insert(tmp_sg);
	tmp_map_vertex_segments[tmp_v2].insert(tmp_sg);
	tmp_map_segment_vertices[tmp_sg].insert(tmp_v1);
	tmp_map_segment_vertices[tmp_sg].insert(tmp_v2);
	temp_cluster->do_multi_tracking(tmp_map_vertex_segments, tmp_map_segment_vertices, *ct_point_cloud, global_wc_map, flash_time*units::microsecond, true, true, true);
	tmp_sg->build_pcloud_fit();
	// associate points
	WCP::WCPointCloud<double>& cloud = temp_cluster->get_point_cloud()->get_cloud();
	WCP::WC2DPointCloud<double>& cloud_u = temp_cluster->get_point_cloud()->get_cloud_u();
	WCP::WC2DPointCloud<double>& cloud_v = temp_cluster->get_point_cloud()->get_cloud_v();
	WCP::WC2DPointCloud<double>& cloud_w = temp_cluster->get_point_cloud()->get_cloud_w();
	for (size_t i=0;i!=cloud.pts.size();i++){
	  tmp_sg->add_associate_point(cloud.pts[i], cloud_u.pts[i], cloud_v.pts[i], cloud_w.pts[i]);
	}
	ToyPointCloud *pcloud_associate = tmp_sg->get_associated_pcloud();
	if (pcloud_associate !=0) pcloud_associate->build_kdtree_index();
	
	// determine direction as topology ...
	tmp_sg->determine_shower_direction();

	
	//	std::cout << max_vtx->get_cluster_id() << " " << max_vtx->get_fit_pt() << " " << min_vtx->get_fit_pt() << " " << tmp_sg->get_flag_dir() << " " << tmp_sg->get_length()/units::cm << std::endl;
	
	if (tmp_sg->get_flag_dir()==1){
	  temp_main_vertex = max_vtx;
	}else if (tmp_sg->get_flag_dir()==-1){
	  temp_main_vertex = min_vtx;
	}else{
	  if (max_vtx->get_fit_pt().z < min_vtx->get_fit_pt().z){ // pick the forward one ...
	    temp_main_vertex = max_vtx;
	  }else{
	    temp_main_vertex = min_vtx;
	  }
	}

	// for a large shower, anyway front vertex ...
	if (tmp_sg->get_length() > 80*units::cm && fabs(max_vtx->get_fit_pt().z - min_vtx->get_fit_pt().z) > 40*units::cm){
	  if (max_vtx->get_fit_pt().z < min_vtx->get_fit_pt().z){ // pick the forward one ...
	    temp_main_vertex = max_vtx;
	  }else{
	    temp_main_vertex = min_vtx;
	  }
	}

	// hack
	//	if (min_vtx->get_cluster_id()==45) temp_main_vertex = max_vtx;
	
	//    std::cout << tmp_sg->get_id() << " " << tmp_sg->get_length()/units::cm << " " << tmp_sg->get_flag_shower_topology() << " " << tmp_sg->get_flag_dir() << " " << tmp_sg->is_dir_weak() << std::endl;
	
	delete tmp_v1;
	delete tmp_v2;
	delete tmp_sg;
      }else{
	if (max_vtx->get_fit_pt().z < min_vtx->get_fit_pt().z){ // pick the forward one ...
	  temp_main_vertex = max_vtx;
	}else{
	  temp_main_vertex = min_vtx;
	}
      }
    }else{
      if (max_vtx->get_fit_pt().z < min_vtx->get_fit_pt().z){ // pick the forward one ...
	temp_main_vertex = max_vtx;
      }else{
	temp_main_vertex = min_vtx;
      }
    }
  }



  std::cout << "[compare_main_vertices_all_showers] Cluster: " << temp_main_vertex->get_cluster_id() << " Selected vertex id=" << temp_main_vertex->get_id() << " at " << temp_main_vertex->get_fit_pt() << " from " << vertex_candidates.size() << " candidates" << std::endl;
  return temp_main_vertex;
}


WCPPID::ProtoVertex* WCPPID::NeutrinoID::compare_main_vertices(WCPPID::ProtoVertexSelection& vertex_candidates){
  std::map<WCPPID::ProtoVertex*, double> map_vertex_num;
  for (auto it = vertex_candidates.begin(); it!=vertex_candidates.end(); it++){
    map_vertex_num[*it] = 0;
  }

  bool flag_print = false;

  WCPPID::ProtoSegment* max_length_muon = 0;
  double max_length = 0;
 
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (it->first->get_cluster_id() != vertex_candidates.front()->get_cluster_id()) continue;
    if (sg->get_flag_shower()) continue;
    if (sg->get_particle_type()==2212) continue;
    double length = sg->get_length();
    if (length > max_length){
      max_length = length;
      max_length_muon = sg;
    }
  }
  
  

  
  // find the proton in and out ...
  for (auto it = vertex_candidates.begin(); it!=vertex_candidates.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    int n_proton_in = 0;
    int n_proton_out = 0;
    for (auto it1 = map_vertex_segments[vtx].begin(); it1 != map_vertex_segments[vtx].end(); it1++){
      WCPPID::ProtoSegment *sg = (*it1);
      if ((sg->is_dir_weak() || sg->get_flag_dir() == 0) && fabs(sg->get_particle_type())==2212){
	WCPPID::ProtoVertex *other_vertex = find_other_vertex(sg, vtx);

	if (map_vertex_segments[other_vertex].size()>1)
	  for (auto it2 = map_vertex_segments[other_vertex].begin(); it2!=map_vertex_segments[other_vertex].end(); it2++){
	    bool flag_start;
	    if ((*it2)->get_wcpt_vec().front().index == other_vertex->get_wcpt().index)
	      flag_start = true;
	    else if ((*it2)->get_wcpt_vec().back().index == other_vertex->get_wcpt().index)
	    flag_start = false;
	    
	    if ((flag_start && (*it2)->get_flag_dir()==1 || (!flag_start) && (*it2)->get_flag_dir()==-1) && (!(*it2)->is_dir_weak()) && fabs((*it2)->get_particle_type())==2212 ){
	      n_proton_out ++;
	    }
	    
	    if ((flag_start && (*it2)->get_flag_dir()==-1 || (!flag_start) && (*it2)->get_flag_dir()==1) && (!(*it2)->is_dir_weak()) && fabs((*it2)->get_particle_type())==2212){
	      n_proton_in ++;
	    }
	    if (((*it2)->is_dir_weak() || (*it2)->get_flag_dir() == 0) && fabs((*it2)->get_particle_type())==2212)
	      n_proton_in ++;
	  }
	
      }
    }
    
    // positive is good ...
    if (n_proton_in > n_proton_out){
      map_vertex_num[vtx] -= (n_proton_in - n_proton_out)/4.;   // proton information ...
    }else{
      map_vertex_num[vtx] -= (n_proton_in - n_proton_out)/4. - (n_proton_in+n_proton_out)/8.;   // proton information ...
    }
    if (flag_print) std::cout << "A: " << map_vertex_num[vtx] << " " << n_proton_in << " " << n_proton_out << std::endl;
  }

  // whether the vertex is at beginning or not ...
  double min_z = 1e9;
  for (auto it = vertex_candidates.begin(); it!=vertex_candidates.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    if (vtx->get_fit_pt().z < min_z) min_z = vtx->get_fit_pt().z;
  }
  for (auto it = vertex_candidates.begin(); it!=vertex_candidates.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    // std::cout << vtx->get_fit_pt().z << std::endl;
    map_vertex_num[vtx] -= (vtx->get_fit_pt().z - min_z)/(200*units::cm);   // position information
    //    std::cout << map_vertex_segments[vtx].size() << std::endl;
    // number of tracks, more is good
    for (auto it1 = map_vertex_segments[vtx].begin(); it1!= map_vertex_segments[vtx].end(); it1++){
      WCPPID::ProtoSegment *sg = (*it1);
      //  std::cout << sg->get_id() << " " << sg->get_flag_shower() << " " << sg->get_particle_type() << " " << sg->get_flag_dir() << " " << sg->is_dir_weak() << std::endl;
      if (sg->get_flag_shower()){
	map_vertex_num[vtx] += 1/4./2.; // number of showers

	auto pair_results =  calculate_num_daughter_showers(vtx, sg);
	if (pair_results.second > 45*units::cm)
	  map_vertex_num[vtx] += 1/4./2.; 
      }else{
	map_vertex_num[vtx] += 1/4.; // number of tracks
      }
      if (sg->get_particle_type()==2212 && sg->get_flag_dir()!=0 && (!sg->is_dir_weak()))
	map_vertex_num[vtx] += 1/4.; // has a clear proton ...
      else if (sg->get_flag_dir()!=0 && (!sg->get_flag_shower()))
	map_vertex_num[vtx] += 1/4./2.; // has a direction with track ..

      if (max_length > 35*units::cm && sg == max_length_muon)
	map_vertex_num[vtx] += 1/4./2.; // long muon add some weight ...
      //std::cout << sg->get_length() << std::endl;
    }

    
    if (flag_print) std::cout << "B: " << map_vertex_num[vtx] << " " << (vtx->get_fit_pt().z - min_z)/(200*units::cm) << " " << map_vertex_segments[vtx].size()/4. << std::endl;
  }
  
  // whether the vetex is at boundary or not ...
  for (auto it = vertex_candidates.begin(); it!=vertex_candidates.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    if (fid->inside_fiducial_volume(vtx->get_fit_pt(),offset_x))
      map_vertex_num[vtx] +=0.5; // good      // fiducial volume ..
    if (flag_print) std::cout << "C: " << map_vertex_num[vtx] << " " << fid->inside_fiducial_volume(vtx->get_fit_pt(),offset_x) << std::endl;
  }

  for (auto it = vertex_candidates.begin(); it != vertex_candidates.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;

    double num_conflicts = calc_conflict_maps(vtx);
    map_vertex_num[vtx] -= num_conflicts/4.;
    if (flag_print)    std::cout << "D: " << map_vertex_num[vtx] << " " << num_conflicts << " " << map_vertex_num[vtx] << std::endl;
  }
  
  double max_val = -1e9; WCPPID::ProtoVertex* max_vertex = 0;
  for (auto it = vertex_candidates.begin(); it!=vertex_candidates.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    if (map_vertex_num[vtx] > max_val){
      max_val = map_vertex_num[vtx];
      max_vertex = vtx;
    }
  }

  //  std::cout << (max_vertex->get_fit_pt().z-min_z)/(400*units::cm) << std::endl;
  std::cout << "[compare_main_vertices] Cluster: " << max_vertex->get_cluster_id() << " Selected vertex id=" << max_vertex->get_id() << " at " << max_vertex->get_fit_pt() << " score=" << max_val << " from " << vertex_candidates.size() << " candidates" << std::endl;
  return max_vertex;
}


float WCPPID::NeutrinoID::calc_conflict_maps(WCPPID::ProtoVertex *temp_vertex){
  // assume temp_vertex is true, and then calculate the conflict in the system ...
  float num_conflicts = 0;

  std::map<WCPPID::ProtoSegment*, std::pair<ProtoVertex*, ProtoVertex*> > map_seg_dir;
  std::set<WCPPID::ProtoVertex* > used_vertices;

  // start ...
  std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > segments_to_be_examined;
  for (auto it = map_vertex_segments[temp_vertex].begin(); it != map_vertex_segments[temp_vertex].end(); it++){
    segments_to_be_examined.push_back(std::make_pair(temp_vertex, *it));
  }
  used_vertices.insert(temp_vertex);
  
  while(segments_to_be_examined.size()>0){
    std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > temp_segments;
    for (auto it = segments_to_be_examined.begin(); it!= segments_to_be_examined.end(); it++){
      WCPPID::ProtoVertex *prev_vtx = it->first;
      WCPPID::ProtoSegment *current_sg = it->second;
      
      if (map_seg_dir.find(current_sg) != map_seg_dir.end()) continue; //looked at it before ...
      WCPPID::ProtoVertex* curr_vertex = find_other_vertex(current_sg, prev_vtx);

      map_seg_dir[current_sg] = std::make_pair(prev_vtx, curr_vertex);
      if (used_vertices.find(curr_vertex) != used_vertices.end()) continue;
      for (auto it1 = map_vertex_segments[curr_vertex].begin(); it1!= map_vertex_segments[curr_vertex].end(); it1++){
	temp_segments.push_back(std::make_pair(curr_vertex, *it1));
      }
      used_vertices.insert(curr_vertex);
    }
    segments_to_be_examined = temp_segments;
  }

  //  std::cout << used_vertices.size() << " " << map_seg_dir.size() << " " << std::endl;
  // check segments
  for (auto it = map_seg_dir.begin(); it!= map_seg_dir.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    WCPPID::ProtoVertex *start_vtx = it->second.first;
    bool flag_start;
    if (sg->get_wcpt_vec().front().index == start_vtx->get_wcpt().index)
      flag_start = true;
    else if (sg->get_wcpt_vec().back().index == start_vtx->get_wcpt().index)
      flag_start = false;

    if (sg->get_flag_dir()!=0 && (sg->get_flag_shower() && sg->get_length()>5*units::cm || (!sg->get_flag_shower()))){
      if (flag_start && sg->get_flag_dir()==-1 ||
	  (!flag_start) && sg->get_flag_dir() == 1){
	if ( !sg->is_dir_weak() ) 	num_conflicts ++;
	else num_conflicts += 0.5;
	//std::cout << " A: " << it->first->get_id() << " " << it->second.first->get_id() << " " << it->second.second->get_id() << std::endl;
      }
    }
    //
  }
  //  std::cout << num_conflicts << " ";

  TVector3 dir_beam(0,0,1);
  
  // now calculate conflicts based on vertices // two things in, one track in one shower out
  for (auto it = used_vertices.begin(); it != used_vertices.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    if (map_vertex_segments[vtx].size()==1) continue;

    int n_in = 0;
    int n_in_shower = 0;
    int n_out_tracks = 0;
    int n_out_showers = 0;
    
    std::map<WCPPID::ProtoSegment*, TVector3> map_in_segment_dirs;
    std::map<WCPPID::ProtoSegment*, TVector3> map_out_segment_dirs;
    
    
    for (auto it1 = map_vertex_segments[vtx].begin(); it1 != map_vertex_segments[vtx].end(); it1++){
      WCPPID::ProtoSegment *sg = (*it1);
      WCPPID::ProtoVertex *vtx1 = map_seg_dir[sg].first;
      if (vtx!=vtx1){
	//	std::cout << sg->get_id() << " " << flag_start << " " << vtx->get_id() << " " << vtx1->get_id() << " " << map_seg_dir[sg].second->get_id() << " " << sg->get_flag_dir() << std::endl;
	n_in ++;
	if (sg->get_flag_shower()) n_in_shower ++;
	map_in_segment_dirs[sg] = sg->cal_dir_3vector(vtx->get_fit_pt(), 10*units::cm);
      }else if (vtx==vtx1){
	if (!sg->get_flag_shower()) n_out_tracks ++;
	else n_out_showers ++;
	map_out_segment_dirs[sg] = sg->cal_dir_3vector(vtx->get_fit_pt(),10*units::cm);
      }
    }

    if (map_in_segment_dirs.size() >0 && map_out_segment_dirs.size()>0){
      double max_angle = -1;
      WCPPID::ProtoSegment *sg1 = 0;
      WCPPID::ProtoSegment *sg2 = 0;
      for (auto it1 = map_in_segment_dirs.begin(); it1!=map_in_segment_dirs.end(); it1++){
	for (auto it2 = map_out_segment_dirs.begin(); it2!= map_out_segment_dirs.end(); it2++){
	  double angle = it1->second.Angle(it2->second)/3.1415926*180.;
	  if (angle > max_angle) {
	    max_angle = angle;
	    sg1 = it1->first;
	    sg2 = it2->first;
	  }
	}
      }
      
      bool flag_check = true;
      if (sg2->get_flag_shower_trajectory() || sg1->get_flag_shower() && sg2->get_flag_shower())
	flag_check = false;

      double angle_beam = map_in_segment_dirs[sg1].Angle(dir_beam)/3.1415926*180.;

      //std::cout << max_angle << " " << sg1->get_id() << " " << sg2->get_id() << " " << sg1->get_particle_type() << " " << sg2->get_particle_type() << " " << sg1->get_flag_shower() << " " << sg2->get_flag_shower() << " " << angle_beam << std::endl;

      
      if (max_angle >=0 && flag_check){
	if (max_angle < 35) {
	  num_conflicts += 5;
	}else if (max_angle < 70) {
	  num_conflicts += 3; // angle does not look right ...
	}else if (max_angle < 85) {
	  num_conflicts += 1;
	}else if (max_angle < 110) {
	  num_conflicts += 0.25;
	}
	
	// go back ward ...
	if (angle_beam < 60 && max_angle < 110){
	  num_conflicts += 1;
	}else if (angle_beam < 45 && max_angle < 70){
	 num_conflicts += 3;
	}
	
      }
    }
    
    if (n_in > 1) {
      if (n_in != n_in_shower){
	num_conflicts += (n_in-1);
      }else{
	num_conflicts += (n_in-1)/2.;
      }
      //      std::cout << "Wrong: Multiple (" << n_in << ") particles into a vertex! " << std::endl;
    }
    if (n_in_shower >0 && n_out_tracks > 0) {
      num_conflicts += std::min(n_in_shower, n_out_tracks);
      //      std::cout << "Wrong: " << n_in_shower << " showers in and " << n_out_tracks << " tracks out! " << std::endl;
    }
  }
  //  std::cout << num_conflicts << std::endl;
  
  return num_conflicts;
}


bool WCPPID::NeutrinoID::examine_direction(WCPPID::ProtoVertex* temp_vertex, bool flag_final){
  std::cout << "[examine_direction] Cluster: " << temp_vertex->get_cluster_id() << " Start vtx id=" << temp_vertex->get_id() << " at " << temp_vertex->get_fit_pt() << " -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;

  double max_vtx_length = 0;
  double min_vtx_length = 1e9;
  int num_total_segments = 0;
  bool flag_only_showers = true;
  {
    //print_segs_info(temp_vertex);
    
    for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
      WCPPID::ProtoVertex *vtx = it->first;
      if (vtx->get_cluster_id() != temp_vertex->get_cluster_id()) continue;
      //      std::cout << temp_vertex << " " << vtx << " " << temp_vertex->get_cluster_id() << " " << vtx->get_cluster_id() << " " << temp_vertex->get_id() << " " << vtx->get_id() << std::endl;
      if (vtx == temp_vertex){
	for (auto it1 = it->second.begin(); it1 != it->second.end();it1++){
	  double length = (*it1)->get_length();
	  // std::cout << length/units::cm << std::endl;
	  if (length > max_vtx_length) {
	    max_vtx_length = length;
	  }
	  if (length < min_vtx_length){
	    min_vtx_length = length;
	  }
	}
      }
      auto results = examine_main_vertex_candidate(vtx);
      bool flag_in = std::get<0>(results);
      int ntracks = std::get<1>(results), nshowers = std::get<2>(results);    
      if (!flag_in){
	if (ntracks > 0) flag_only_showers = false;
      }
    }

    for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
      if (it->first->get_cluster_id() ==  temp_vertex->get_cluster_id())
	num_total_segments ++;
    }
  }

 
  if (map_vertex_segments[temp_vertex].size()==2 && (max_vtx_length > 30*units::cm || min_vtx_length > 15*units::cm)
      || map_vertex_segments[temp_vertex].size()>2 &&  num_total_segments > 4
      || map_vertex_segments[temp_vertex].size()>3 
      ) flag_only_showers = false;
  
  //std::cout << temp_vertex->get_cluster_id() << " " << map_vertex_segments[temp_vertex].size() << " " << num_total_segments << " " << flag_only_showers << " " << max_vtx_length/units::cm << " " << min_vtx_length/units::cm << std::endl;
  
  /* if (flag_final){ */
  /*   for (auto it = map_vertex_segments[temp_vertex].begin(); it!= map_vertex_segments[temp_vertex].end(); it++){ */
  /*     WCPPID::ProtoSegment *sg = (*it); */
  /*     std::pair<int, double> pair_result = calculate_num_daughter_showers(temp_vertex, sg, false); */
  /*     if (pair_result.first <=2 && sg->get_flag_shower_trajectory()){ */
  /* 	if (!sg->is_shower_trajectory()){ */
  /* 	  WCPPID::ProtoVertex *start_v=0, *end_v=0; */
  /* 	  for (auto it = map_segment_vertices[sg].begin(); it!=map_segment_vertices[sg].end(); it++){ */
  /* 	    if ((*it)->get_wcpt().index == sg->get_wcpt_vec().front().index) start_v = *it; */
  /* 	    if ((*it)->get_wcpt().index == sg->get_wcpt_vec().back().index) end_v = *it; */
  /* 	  } */
  /* 	  sg->determine_dir_track(map_vertex_segments[start_v].size(), map_vertex_segments[end_v].size(), false); */
  /* 	} */
  /* 	//	std::cout << sg->get_id() << " " << sg->is_shower_trajectory() << std::endl; */
  /*     } */
  /*   } */
  /* } */
  
  TVector3 drift_dir(1,0,0);
  TPCParams& mp = Singleton<TPCParams>::Instance();
  std::set<WCPPID::ProtoVertex* > used_vertices;
  std::set<WCPPID::ProtoSegment* > used_segments;

  std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > segments_to_be_examined;
  for (auto it = map_vertex_segments[temp_vertex].begin(); it != map_vertex_segments[temp_vertex].end(); it++){
    segments_to_be_examined.push_back(std::make_pair(temp_vertex, *it));
  }
  used_vertices.insert(temp_vertex);

  while(segments_to_be_examined.size()>0){
    std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > temp_segments;
    for (auto it = segments_to_be_examined.begin(); it!= segments_to_be_examined.end(); it++){
      WCPPID::ProtoVertex *prev_vtx = it->first;
      bool flag_shower_in = false;

      std::vector<ProtoSegment*> in_showers;
      for (auto it1 = map_vertex_segments[prev_vtx].begin(); it1 != map_vertex_segments[prev_vtx].end(); it1++){
	WCPPID::ProtoSegment *sg = (*it1);
	bool flag_start;
	if (sg->get_wcpt_vec().front().index == prev_vtx->get_wcpt().index)
	  flag_start = true;
	else if (sg->get_wcpt_vec().back().index == prev_vtx->get_wcpt().index)
	  flag_start = false;
	if (flag_start && sg->get_flag_dir()==-1 || (!flag_start) && sg->get_flag_dir()==1){
	  if (sg->get_flag_shower() ){
	    flag_shower_in = true;
	    in_showers.push_back(sg);
	    break;
	  }
	}
      }


      WCPPID::ProtoSegment *current_sg = it->second;
      // std::cout << prev_vtx->get_id() << " V " << current_sg->get_id() << std::endl;
      if (used_segments.find(current_sg)!=used_segments.end()) continue; // looked at it before ...
      double length = current_sg->get_length();
      //      std::cout << current_sg->get_id() << " " << current_sg->get_particle_type() << " " << flag_shower_in << std::endl;
      
      if (current_sg->get_flag_dir() ==0 || current_sg->is_dir_weak() || current_sg->get_flag_shower() || flag_final){ // change direction ...
	bool flag_start;
	if (current_sg->get_wcpt_vec().front().index == prev_vtx->get_wcpt().index)
	  flag_start = true;
	else if (current_sg->get_wcpt_vec().back().index == prev_vtx->get_wcpt().index)
	  flag_start = false;
	int old_dir_ed = current_sg->get_flag_dir();
	int old_ptype_ed = current_sg->get_particle_type();
	if (flag_start) current_sg->set_flag_dir(1);
	else current_sg->set_flag_dir(-1);
	if (old_dir_ed != current_sg->get_flag_dir())
	  std::cout << "[examine_direction] Cluster: " << temp_vertex->get_cluster_id() << " Seg " << current_sg->get_id() << " dir: " << old_dir_ed << " -> " << current_sg->get_flag_dir() << " from vtx " << prev_vtx->get_id() << std::endl;
	
	//   if (current_sg->get_id()==19) std::cout << current_sg->get_id() << " " << flag_start << " " << map_vertex_segments[prev_vtx].size() << " " << prev_vtx->get_wcpt().index << " " << current_sg->get_wcpt_vec().front().index << " " << current_sg->get_wcpt_vec().back().index << std::endl;

	if (flag_shower_in && current_sg->get_flag_dir()==0 && (!current_sg->get_flag_shower())){
	  current_sg->set_particle_type(11);
	  current_sg->set_particle_mass(mp.get_mass_electron());
	}else if (flag_shower_in && length<2.0*units::cm && (!current_sg->get_flag_shower())){
	  current_sg->set_particle_type(11);
	  current_sg->set_particle_mass(mp.get_mass_electron());
	}else if (flag_shower_in && (fabs(current_sg->get_particle_type())==13 || current_sg->get_particle_type()==0 )){
	  current_sg->set_particle_type(11);
	  current_sg->set_particle_mass(mp.get_mass_electron());
	}else{
	  auto pair_result = calculate_num_daughter_showers(prev_vtx, current_sg);
	  auto pair_result1 = calculate_num_daughter_showers(prev_vtx, current_sg, false);
	  int num_daughter_showers = pair_result.first;
	  double length_daughter_showers = pair_result.second;

	  //	  std::cout <<  current_sg->get_id() << " " <<  pair_result.second/units::cm << " " << pair_result1.second/units::cm << std::endl;
	  
	  if (current_sg->get_particle_type()!=11 && (num_daughter_showers >=4 || length_daughter_showers > 50*units::cm && num_daughter_showers>=2) && pair_result.second > pair_result1.second - length - pair_result.second){
	    // check angle ...
	    bool flag_change = false;
	    WCPPID::ProtoVertex* next_vertex = find_other_vertex(current_sg, prev_vtx);
	    TVector3 tmp_dir1 = current_sg->cal_dir_3vector(next_vertex->get_fit_pt(), 15*units::cm);
	    
	    for (auto it5 = map_vertex_segments[next_vertex].begin(); it5!=map_vertex_segments[next_vertex].end(); it5++){
	      if (*it5 == current_sg) continue;
	      
	      TVector3 tmp_dir2 = (*it5)->cal_dir_3vector(next_vertex->get_fit_pt(), 15*units::cm);

	      //	      std::cout <<  current_sg->get_id() << " " << tmp_dir1.Angle(tmp_dir2)/3.1415926*180. << " " << drift_dir.Angle(tmp_dir1)/3.1415926*180. << " " << drift_dir.Angle(tmp_dir2)/3.1415926*180. << " " << (*it5)->get_length()/units::cm << " " << pair_result.second/units::cm << " " << pair_result1.second/units::cm << std::endl;
	      
	      if (tmp_dir1.Angle(tmp_dir2)/3.1415926*180. > 155
		  || tmp_dir1.Angle(tmp_dir2)/3.1415926*180. > 135 && fabs(drift_dir.Angle(tmp_dir1)/3.1415926*180.-90)<10 && fabs(drift_dir.Angle(tmp_dir2)/3.1415926*180.-90)<10
		  || tmp_dir1.Angle(tmp_dir2)/3.1415926*180. > 135 &&  (*it5)->get_length() < 6*units::cm
		  ) { // 25 degrees ...
		flag_change = true;
		break;
	      }

	    }
	    if (flag_change){
	      current_sg->set_particle_type(11);
	      current_sg->set_particle_mass(mp.get_mass_electron());
	    }
	    //	    std::cout << current_sg->get_id() << " " << current_sg->get_particle_type() << " " << num_daughter_showers << std::endl;
	  }else if (current_sg->get_particle_type()==11 && num_daughter_showers<=2 && (!flag_shower_in) && (!current_sg->get_flag_shower_topology()) && (!current_sg->get_flag_shower_trajectory()) && length > 10*units::cm && (!flag_only_showers)){

	    //std::cout << current_sg->get_id() << " " << current_sg->get_particle_type() << " " << num_daughter_showers << " " << current_sg->get_length()/units::cm << " " << current_sg->get_flag_shower_topology() << " " << current_sg->get_flag_shower_trajectory() << " " << " " << current_sg->get_particle_score() << std::endl;
	    if (current_sg->get_particle_score()<=100){	      
	      double direct_length = current_sg->get_direct_length();
	      if (direct_length >= 34*units::cm || direct_length < 34*units::cm && direct_length > 0.93 * length
		  //&& current_sg->get_particle_score()!=100 // no good ...
		  ){
		//	       std::cout << direct_length << " " << length << std::endl;
		current_sg->set_particle_type(13);
		current_sg->set_particle_mass(mp.get_mass_muon());
	      }
	    }
	  }else if (current_sg->get_particle_type()==11 && current_sg->get_flag_shower_trajectory() && num_daughter_showers == 1 && (!flag_only_showers)){
	    auto pair_result1 = calculate_num_daughter_showers(prev_vtx, current_sg, false);
	    if (pair_result1.second>3*length && pair_result1.second - length > 12*units::cm){
	      current_sg->set_flag_shower_trajectory(false);
	      current_sg->set_particle_type(13);
	      current_sg->set_particle_mass(mp.get_mass_muon());
	      //	      std::cout << current_sg->get_id() << " " << current_sg->get_particle_type() << " " << num_daughter_showers << " " << current_sg->get_length()/units::cm << " " << current_sg->get_flag_shower_topology() << " " << current_sg->get_flag_shower_trajectory() << " " << pair_result1.first << " " << pair_result1.second << std::endl;
	    }
	  }
	}

	// still particle type is not determined ...
	if (temp_vertex == main_vertex){
	  if (flag_only_showers){
	    if (current_sg->get_particle_type()==0 && (!current_sg->get_flag_shower())){
	      current_sg->set_particle_type(11);
	      current_sg->set_particle_mass(mp.get_mass_electron());
	    }
	  }else{
	    if (current_sg->get_particle_type()==0 && (!current_sg->get_flag_shower())){
	      if (current_sg->get_medium_dQ_dx()/(43e3/units::cm) > 1.4){
		current_sg->set_particle_type(2212);
		current_sg->set_particle_mass(mp.get_mass_proton());
	      }else{
		current_sg->set_particle_type(13);
		current_sg->set_particle_mass(mp.get_mass_muon());
	      }
	      //	      std::cout << current_sg->get_id() << " " << current_sg->get_particle_type() << " " << current_sg->get_medium_dQ_dx()/(43e3/units::cm) << std::endl;
	    }
	  }
	}
	
	
	current_sg->cal_4mom();
	current_sg->set_dir_weak(true);
	if (old_ptype_ed != current_sg->get_particle_type())
	  std::cout << "[examine_direction] Cluster: " << temp_vertex->get_cluster_id() << " Seg " << current_sg->get_id() << " particle type: " << old_ptype_ed << " -> " << current_sg->get_particle_type() << " len=" << length/units::cm << "cm from vtx " << prev_vtx->get_id() << std::endl;
      }else if (current_sg->get_flag_dir() !=0 && (!current_sg->is_dir_weak())){
	auto pair_result = calculate_num_daughter_showers(prev_vtx, current_sg);
	int num_daughter_showers = pair_result.first;
	double length_daughter_showers = pair_result.second;
	//std::cout << current_sg->get_id() << " " << current_sg->get_particle_type() << " " << flag_shower_in << " " << num_daughter_showers << " " << std::endl;
	if (current_sg->get_particle_type()==2212 && flag_shower_in && num_daughter_showers == 0){
	  for (auto it1 = in_showers.begin(); it1!= in_showers.end(); it1++){
	    int old_ptype_in_shower = (*it1)->get_particle_type();
	    if ((*it1)->get_medium_dQ_dx()/(43e3/units::cm) > 1.3){
	      (*it1)->set_particle_type(2212);
	      (*it1)->set_flag_shower_trajectory(false);
	      (*it1)->set_flag_shower_topology(false);
	      (*it1)->set_particle_mass(mp.get_mass_proton());
	      if ((*it1)->get_particle_4mom(3)>0)
		(*it1)->cal_4mom();
	    }else{
	      (*it1)->set_particle_type(211);
	      (*it1)->set_flag_shower_trajectory(false);
	      (*it1)->set_flag_shower_topology(false);
	      (*it1)->set_particle_mass(mp.get_mass_pion());
	      if ((*it1)->get_particle_4mom(3)>0)
		(*it1)->cal_4mom();
	    }
	    if (old_ptype_in_shower != (*it1)->get_particle_type())
	      std::cout << "[examine_direction] Cluster: " << temp_vertex->get_cluster_id() << " In-shower seg " << (*it1)->get_id() << " particle type: " << old_ptype_in_shower << " -> " << (*it1)->get_particle_type() << " (proton/pion reclassification)" << std::endl;
	  }
	}
      } // good track ...

      
      used_segments.insert(current_sg);

      WCPPID::ProtoVertex* curr_vertex = find_other_vertex(current_sg, prev_vtx);

      if (used_vertices.find(curr_vertex) != used_vertices.end()) continue;
      //std::cout << map_vertex_segments[curr_vertex].size() << std::endl;
      for (auto it1 = map_vertex_segments[curr_vertex].begin(); it1!= map_vertex_segments[curr_vertex].end(); it1++){
	//	std::cout << current_sg->get_id() << " " << (*it1)->get_id() << std::endl;
	temp_segments.push_back(std::make_pair(curr_vertex, *it1));
      }
      used_vertices.insert(curr_vertex);
    }
    segments_to_be_examined = temp_segments;
  }

  bool flag_fill_long_muon = true;
  for (auto it = segments_in_long_muon.begin(); it != segments_in_long_muon.end(); it++){
    WCPPID::ProtoSegment *sg = *it;
    if (sg->get_cluster_id() == temp_vertex->get_cluster_id())
      flag_fill_long_muon = false;
  }
  
  // find the long muon candidate ...
  if (flag_fill_long_muon){
    for (auto it = map_vertex_segments[temp_vertex].begin(); it != map_vertex_segments[temp_vertex].end(); it++){
      WCPPID::ProtoSegment *sg = (*it);
      WCPPID::ProtoVertex *vtx = find_other_vertex(sg, temp_vertex);
      if (sg->get_medium_dQ_dx()/(43e3/units::cm) > 1.3) continue;
      std::vector<WCPPID::ProtoSegment* > acc_segments;
      std::vector<WCPPID::ProtoVertex* > acc_vertices;
      acc_segments.push_back(sg);
      acc_vertices.push_back(vtx);

      //      std::cout << sg->get_flag_shower_topology() << " " << sg->get_flag_shower_trajectory() << " " << sg->get_length()/units::cm << " " << " " << sg->get_medium_dQ_dx()/(43e3/units::cm) << std::endl;
      
      auto results = find_cont_muon_segment(sg, vtx);
      while(results.first !=0){
	//	std::cout << results.first->get_id() << " " << results.second->get_id() << std::endl;
	acc_segments.push_back(results.first);
	acc_vertices.push_back(results.second);
	results = find_cont_muon_segment(results.first, results.second);
      }
      double total_length = 0, max_length = 0;
      for (auto it1 = acc_segments.begin(); it1!=acc_segments.end(); it1++){
	double length = (*it1)->get_length();
	total_length += length;
	if (length > max_length) max_length = length;
      }
      //      std::cout << sg->get_id() << " " << max_length/units::cm << " " << total_length/units::cm << std::endl;
      
      if (total_length > 45*units::cm && max_length > 35*units::cm && acc_segments.size()>1){
	for (auto it1 = acc_segments.begin(); it1!=acc_segments.end(); it1++){
	  // change to muon ...
	  //	  std::cout << (*it1)->get_id() << std::endl;

	  (*it1)->set_particle_type(13);
	  (*it1)->set_flag_shower_trajectory(false);
	  (*it1)->set_flag_shower_topology(false);
	  (*it1)->set_particle_mass(mp.get_mass_muon());
	  if ((*it1)->get_particle_4mom(3)>0)
	    (*it1)->cal_4mom();
	  segments_in_long_muon.insert(*it1);
	}
	for (auto it1 = acc_vertices.begin(); it1!=acc_vertices.end(); it1++){
	  vertices_in_long_muon.insert(*it1);
	}
      }
    }
  }
  

  
  
  // std::cout << used_vertices.size() << " " << used_segments.size() << std::endl;

  
  // find the muon candidate, make the other ones connecting to main vertex to be pion
  {
    WCPPID::ProtoSegment *muon_sg = 0;
    double muon_length = 0;
    WCPPID::ProtoSegmentSelection pion_sgs;
    for (auto it = map_vertex_segments[temp_vertex].begin(); it!=map_vertex_segments[temp_vertex].end(); it++){
      WCPPID::ProtoSegment *sg = *it;
      if (abs(sg->get_particle_type()) == 13){
	if (segments_in_long_muon.find(sg)!= segments_in_long_muon.end()) continue;
	WCPPID::ProtoVertex *other_vertex = find_other_vertex(sg, temp_vertex);
	int n_proton = 0;
	for (auto it1 = map_vertex_segments[other_vertex].begin(); it1 != map_vertex_segments[other_vertex].end(); it1++){
	  if (abs((*it1)->get_particle_type())==2212){
	    n_proton ++;
	  }
	}
	//	std::cout << sg->get_id() << " " << sg->get_particle_type() << " " << sg->get_length()/units::cm << " " << muon_length/units::cm << " " << n_proton << std::endl;
	if (sg->get_length() > muon_length && n_proton == 0){
	  muon_length = sg->get_length();
	  muon_sg = sg;
	}
	pion_sgs.push_back(sg);
      }else if (sg->get_particle_type()==0){
      	WCPPID::ProtoVertex *other_vertex = find_other_vertex(sg, temp_vertex);
      	int n_proton = 0;
      	for (auto it1 = map_vertex_segments[other_vertex].begin(); it1 != map_vertex_segments[other_vertex].end(); it1++){
      	  if (abs((*it1)->get_particle_type())==2212){
      	    n_proton ++;
      	  }
      	}
      	if (n_proton >0){
      	  if (sg->get_medium_dQ_dx()/(43e3/units::cm) > 1.3){
      	    (*it)->set_particle_type(2212);
      	    (*it)->set_particle_mass(mp.get_mass_proton());
      	    if ((*it)->get_particle_4mom(3)>0)
      	      (*it)->cal_4mom();
      	  }else{
      	    (*it)->set_particle_type(211);
      	    (*it)->set_particle_mass(mp.get_mass_pion());
      	    if ((*it)->get_particle_4mom(3)>0)
      	      (*it)->cal_4mom();
      	  }
      	}
      }
    }
    for (auto it = pion_sgs.begin(); it!= pion_sgs.end(); it++){
      if (*it == muon_sg) continue;
      (*it)->set_particle_type(211);
      (*it)->set_particle_mass(mp.get_mass_pion());
      if ((*it)->get_particle_4mom(3)>0)
	(*it)->cal_4mom();
    }
  }
  
  // find the Michel electron 
  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != temp_vertex->get_cluster_id()) continue;
    if (sg->get_particle_4mom(3)==0 && sg->get_particle_mass() > 0&& (!sg->get_flag_shower_topology())){
      if (!sg->is_dir_weak() ){  // weak direction and not shower
	//      std::cout << sg->get_id() << std::endl;
	sg->cal_4mom();
      } else{  
      	std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> vertices = find_vertices(sg);
      	WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
      	if (vertices.first->get_wcpt().index == sg->get_wcpt_vec().front().index){
      	  start_v = vertices.first;
      	  end_v = vertices.second;
      	}else{
      	  start_v = vertices.second;
      	  end_v = vertices.first;
      	}

      	if (sg->get_flag_dir()==1  && map_vertex_segments[end_v].size()==1  && fid->inside_fiducial_volume(end_v->get_fit_pt(),offset_x) ||
	    sg->get_flag_dir()==-1 && map_vertex_segments[start_v].size()==1 && fid->inside_fiducial_volume(start_v->get_fit_pt(),offset_x))
      	  sg->cal_4mom();
	else if (map_vertex_segments[end_v].size()==2){
	  bool flag_Michel = false;
	  for (auto it1 = map_vertex_segments[end_v].begin(); it1 != map_vertex_segments[end_v].end(); it1++){
	    if ((*it1) == sg) continue;
	    if ( (*it1)->get_flag_shower_trajectory() || (*it1)->get_flag_shower_dQdx() ) flag_Michel = true;
	  }
	  if (flag_Michel) sg->cal_4mom();
	}else if (map_vertex_segments[start_v].size()==2){
	  bool flag_Michel = false;
	  for (auto it1 = map_vertex_segments[start_v].begin(); it1 != map_vertex_segments[start_v].end(); it1++){
	    if ((*it1) == sg) continue;
	    if ( (*it1)->get_flag_shower_trajectory() || (*it1)->get_flag_shower_dQdx() ) flag_Michel = true;
	  }
	  if (flag_Michel) sg->cal_4mom();
	}
      }
    }
  }

  
  
  // print ...
   
  // examination ...
  return examine_maps(temp_vertex);
  
  
}


std::pair<WCPPID::ProtoSegment*, WCPPID::ProtoVertex* > WCPPID::NeutrinoID::find_cont_muon_segment(WCPPID::ProtoSegment* sg, WCPPID::ProtoVertex* vtx, bool flag_ignore_dQ_dx){
  WCPPID::ProtoSegment *sg1 = 0;
  double max_length = 0;
  double max_angle = 0;
  double max_ratio = 0;
  WCPPID::ProtoVertex *vtx1 = 0;

  bool flag_cont = false;

  double max_ratio1 = 0;
  double max_ratio1_length = 0;

  double sg_length = sg->get_length();
  
  for (auto it = map_vertex_segments[vtx].begin(); it!= map_vertex_segments[vtx].end(); it++){
    WCPPID::ProtoSegment* sg2 = *it;
    if (sg2 == sg) continue;
    WCPPID::ProtoVertex* vtx2 = find_other_vertex(sg2, vtx);

    // check ...
    TVector3 dir1 = sg->cal_dir_3vector(vtx->get_fit_pt(), 15*units::cm);
    TVector3 dir2 = sg2->cal_dir_3vector(vtx->get_fit_pt(), 15*units::cm);

    double length = sg2->get_length();
    double angle = (3.1415926-dir1.Angle(dir2))/3.1415926*180.;
    double ratio = sg2->get_medium_dQ_dx()/(43e3/units::cm);

    double angle1 = angle;
    if (length > 50*units::cm){
      TVector3 dir3 = sg->cal_dir_3vector(vtx->get_fit_pt(), 50*units::cm);
      TVector3 dir4 = sg2->cal_dir_3vector(vtx->get_fit_pt(), 50*units::cm);
      angle1 = (3.1415926-dir3.Angle(dir4))/3.1415926*180.;
    }
    
    //    std::cout << "A: " << angle << " " << angle1 << " " << length/units::cm << " " << ratio << " " << sg->get_id() << " " << sg2->get_id() << std::endl;
    
    if ( (angle < 10. || angle1 < 10 || sg_length < 6*units::cm && (angle < 15 || angle1 < 15)) &&  (ratio < 1.3 || flag_ignore_dQ_dx)){
      flag_cont = true;
      if (length *cos(angle/180.*3.1415926) > max_length){
	max_length = length *cos(angle/180.*3.1415926) ;
	max_angle = angle;
	max_ratio = ratio;
	sg1 = sg2;
	vtx1 = vtx2;
      }
    }else{
      if (ratio > max_ratio1){
	max_ratio1 = ratio;
	max_ratio1_length = length;
      }
    }
  }
  //  if (max_ratio1 > 1.2 && max_ratio1_length > 5*units::cm)
  //  flag_cont = false;
    //    std::cout << flag_cont << " " << max_ratio1 << " " << max_ratio1_length << std::endl;

  if (flag_cont){
    //    std::cout << max_angle << " " << max_length/units::cm << " " << max_ratio << std::endl;
    return std::make_pair(sg1, vtx1);
  }else{
    sg1 = 0;
    vtx1 = 0;
    return std::make_pair(sg1, vtx1);
  }
  
}


std::pair<WCPPID::ProtoSegment*, WCPPID::ProtoVertex* > WCPPID::NeutrinoID::find_cont_muon_segment_nue(WCPPID::ProtoSegment* sg, WCPPID::ProtoVertex* vtx, bool flag_ignore_dQ_dx){
  WCPPID::ProtoSegment *sg1 = 0;
  double max_length = 0;
  double max_angle = 0;
  double max_ratio = 0;
  WCPPID::ProtoVertex *vtx1 = 0;

  bool flag_cont = false;

  double max_ratio1 = 0;
  double max_ratio1_length = 0;

  double sg_length = sg->get_length();
  
  for (auto it = map_vertex_segments[vtx].begin(); it!= map_vertex_segments[vtx].end(); it++){
    WCPPID::ProtoSegment* sg2 = *it;
    if (sg2 == sg) continue;
    WCPPID::ProtoVertex* vtx2 = find_other_vertex(sg2, vtx);

    // check ...
    TVector3 dir1 = sg->cal_dir_3vector(vtx->get_fit_pt(), 15*units::cm);
    TVector3 dir2 = sg2->cal_dir_3vector(vtx->get_fit_pt(), 15*units::cm);

    double length = sg2->get_length();
    double angle = (3.1415926-dir1.Angle(dir2))/3.1415926*180.;
    double ratio = sg2->get_medium_dQ_dx()/(43e3/units::cm);

    double angle1 = angle;
    //  double angle2 = angle;
    //double angle3 = angle;
    if (length > 30*units::cm || sg_length > 30*units::cm){
      TVector3 dir3 = sg->cal_dir_3vector(vtx->get_fit_pt(), 30*units::cm);
      TVector3 dir4 = sg2->cal_dir_3vector(vtx->get_fit_pt(), 30*units::cm);
      angle1 = (3.1415926-dir3.Angle(dir4))/3.1415926*180.;
      //angle2 = (3.1415926-dir3.Angle(dir2))/3.1415926*180.;
      //angle3 = (3.1415926-dir1.Angle(dir4))/3.1415926*180.;
    }
    
    //    std::cout << "A: " << angle << " " << angle1 << " "  << " " << length/units::cm << " " << ratio << " " << sg->get_id() << " " << sg2->get_id() << std::endl;
    
    if ( (angle < 12.5 || angle1 < 12.5 || sg_length < 6*units::cm && (angle < 15 || angle1 < 15)) &&  (ratio < 1.3 || flag_ignore_dQ_dx)){
      flag_cont = true;
      if (length *cos(angle/180.*3.1415926) > max_length){
	max_length = length *cos(angle/180.*3.1415926) ;
	max_angle = angle;
	max_ratio = ratio;
	sg1 = sg2;
	vtx1 = vtx2;
      }
    }else{
      if (ratio > max_ratio1){
	max_ratio1 = ratio;
	max_ratio1_length = length;
      }
    }
  }
  //  if (max_ratio1 > 1.2 && max_ratio1_length > 5*units::cm)
  //  flag_cont = false;
    //    std::cout << flag_cont << " " << max_ratio1 << " " << max_ratio1_length << std::endl;

  if (flag_cont){
    //    std::cout << max_angle << " " << max_length/units::cm << " " << max_ratio << std::endl;
    return std::make_pair(sg1, vtx1);
  }else{
    sg1 = 0;
    vtx1 = 0;
    return std::make_pair(sg1, vtx1);
  }
  
}
