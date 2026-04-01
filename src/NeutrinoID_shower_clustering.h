
struct cluster_point_info{
  WCPPID::PR3DCluster *cluster;
  double min_angle;
  double min_dis;
  Point min_point;
  WCPPID::ProtoVertex *min_vertex;
};

bool sortbydis(const cluster_point_info &a, const cluster_point_info &b){
  return (a.min_dis < b.min_dis);
}


void WCPPID::NeutrinoID::shower_determing_in_main_cluster(WCPPID::PR3DCluster *temp_cluster){
  const bool flag_timing = true;
  using SDMC_Clock = std::chrono::high_resolution_clock;
  using SDMC_ms = std::chrono::duration<double, std::milli>;
  auto sdmc_t_total = SDMC_Clock::now();
  auto sdmc_t0 = SDMC_Clock::now();

  //hack for now
  /* for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){ */
  /*   WCPPID::ProtoSegment *sg = it->first; */
  /*   if (sg->get_cluster_id() != main_cluster->get_cluster_id()) continue; */
  /*   if (sg->get_id()==21 || sg->get_id()==29 || sg->get_id()==31){ */
  /*     sg->set_particle_type(11); */
  /*     TPCParams& mp = Singleton<TPCParams>::Instance(); */
  /*     sg->set_particle_mass(mp.get_mass_electron()); */
  /*     sg->set_flag_dir(0); */
  /*     sg->set_dir_weak(1); */
  /*   } */
  /* } */

  examine_good_tracks(temp_cluster->get_cluster_id());
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: examine_good_tracks took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;


  // if multiple tracks in, make them undetermined ...
  sdmc_t0 = SDMC_Clock::now();
  fix_maps_multiple_tracks_in(temp_cluster->get_cluster_id());
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: fix_maps_multiple_tracks_in took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;

  // if one shower in and a good track out, reverse the shower ..
  sdmc_t0 = SDMC_Clock::now();
  fix_maps_shower_in_track_out(temp_cluster->get_cluster_id());
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: fix_maps_shower_in_track_out (1st) took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;


  // if there is one good track in, turn everything else to out
  sdmc_t0 = SDMC_Clock::now();
  improve_maps_one_in(temp_cluster); // one in and many out ...
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: improve_maps_one_in took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;

  //  print_segs_info(temp_cluster->get_cluster_id());
  // if one shower in and a track out, change the track to shower
  sdmc_t0 = SDMC_Clock::now();
  improve_maps_shower_in_track_out(temp_cluster->get_cluster_id()); // use shower information to determine the rest ...
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: improve_maps_shower_in_track_out (1st) took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;
  //  print_segs_info(temp_cluster->get_cluster_id());

  // help to change tracks around shower to showers
  sdmc_t0 = SDMC_Clock::now();
  improve_maps_no_dir_tracks(temp_cluster->get_cluster_id());
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: improve_maps_no_dir_tracks took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;

  //print_segs_info(temp_cluster->get_cluster_id());

  // if one shower in and a track out, change the track to shower
  sdmc_t0 = SDMC_Clock::now();
  improve_maps_shower_in_track_out(temp_cluster->get_cluster_id(), false); // use shower information to determine the rest ...
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: improve_maps_shower_in_track_out (2nd) took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;



  // if multiple tracks in, change track to shower
  sdmc_t0 = SDMC_Clock::now();
  improve_maps_multiple_tracks_in(temp_cluster->get_cluster_id());
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: improve_maps_multiple_tracks_in took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;

  //  print_segs_info(temp_cluster->get_cluster_id());

  // if one shower in and a good track out, reverse the shower ..
  sdmc_t0 = SDMC_Clock::now();
  fix_maps_shower_in_track_out(temp_cluster->get_cluster_id());
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: fix_maps_shower_in_track_out (2nd) took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;

  // judgement ...
  sdmc_t0 = SDMC_Clock::now();
  judge_no_dir_tracks_close_to_showers(temp_cluster->get_cluster_id());
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: judge_no_dir_tracks_close_to_showers took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;

  // examine map ...
  sdmc_t0 = SDMC_Clock::now();
  examine_maps(temp_cluster);
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: examine_maps took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;

  sdmc_t0 = SDMC_Clock::now();
  examine_all_showers(temp_cluster);
  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: examine_all_showers took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t0).count() << " ms" << std::endl;

  if (flag_timing) std::cout << "shower_determing_in_main_cluster timing: TOTAL took " << std::chrono::duration_cast<SDMC_ms>(SDMC_Clock::now()-sdmc_t_total).count() << " ms" << std::endl;

  /* print_segs_info(main_cluster->get_cluster_id()); */
  /* std::cout << std::endl << std::endl; */
  /* for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){ */
  /*   WCPPID::ProtoSegment *sg = it->first; */
  /*   if (sg->get_particle_4mom(3) > sg->get_particle_mass() && (!sg->get_flag_shower())) */
  /*     std::cout << sg->get_id() << " " << sg->get_length()/units::cm << " Track  "  << sg->get_flag_dir() << " " << sg->get_particle_type() << " " << sg->get_particle_mass()/units::MeV << " " << (sg->get_particle_4mom(3)-sg->get_particle_mass())/units::MeV << " " << sg->is_dir_weak() << std::endl; */
  /* } */

}

void WCPPID::NeutrinoID::shower_clustering_connecting_to_main_vertex(){
  std::set<WCPPID::ProtoSegment*>  used_segments;
  for (auto it = showers.begin(); it != showers.end(); it++){
    WCPPID::WCShower *shower = *it;
    if (shower->get_start_vertex().first == main_vertex){
      Map_Proto_Segment_Vertices& map_seg_vtxs = shower->get_map_seg_vtxs();
      for (auto it1 = map_seg_vtxs.begin(); it1 != map_seg_vtxs.end(); it1++){
	used_segments.insert(it1->first);
      }
      // 7017_941_47091
      if (used_segments.size()==1 && (*used_segments.begin())->get_length() < 8*units::cm ) used_segments.clear();
    }
  }

 
  
  if (used_segments.size()==0){
    std::set<WCPPID::WCShower* > del_showers;
    std::map<WCPPID::WCShower*, WCPPID::ProtoSegment* > map_shower_max_sg;
    WCPPID::WCShower* max_shower = 0;
    double max_length = 0;
    
    for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end(); it++){
      WCPPID::ProtoSegment *sg = *it;

      
      std::pair<int, double> pair_result = calculate_num_daughter_tracks(main_vertex, sg, true);

      //      std::cout << sg->get_id() << " " << sg->get_particle_type()<< " " << sg->get_medium_dQ_dx()/(43e3/units::cm) << " " << pair_result.first << std::endl;
      if (map_segment_in_shower.find(sg) != map_segment_in_shower.end()) continue;
      // 6583_144_7201
      double medium_dQ_dx_1 = sg->get_medium_dQ_dx()/(43e3/units::cm);
      if (sg->get_particle_type()==11 ||sg->get_particle_type()==2212 && (medium_dQ_dx_1 > 1.45 && pair_result.first <=3 || medium_dQ_dx_1 > 2.7) || sg->get_particle_type()== 211 && medium_dQ_dx_1 > 2.0) continue;
      
      WCPPID::WCShower *shower = new WCPPID::WCShower();
      shower->set_start_vertex(main_vertex, 1);
      shower->set_start_segment(sg);
      shower->complete_structure_with_start_segment(map_vertex_segments, map_segment_vertices, used_segments);
      Map_Proto_Segment_Vertices& map_seg_vtxs = shower->get_map_seg_vtxs();
      Map_Proto_Vertex_Segments& map_vtx_segs = shower->get_map_vtx_segs();
      double max_length = 0;
      WCPPID::ProtoSegment *max_sg = 0;
      int n_tracks = 0;
      int n_showers = 0;
      double total_length = 0;
      bool flag_good_track = false;
      for (auto it1 = map_seg_vtxs.begin(); it1 != map_seg_vtxs.end(); it1++){
	WCPPID::ProtoSegment *sg1 = it1->first;
	double length = sg1->get_length();
	double medium_dQ_dx = sg1->get_medium_dQ_dx()/(43e3/units::cm);
	// 7054_1409_70465
	if ((!sg1->is_dir_weak()) && (sg1->get_length() > 3.6*units::cm || sg1->get_length() > 2.4*units::cm && medium_dQ_dx > 2.5)) {
	  auto pair_vertices = find_vertices(sg1);
	  WCPPID::ProtoVertex *end_vertex = 0;
	  if (sg1->get_flag_dir()==1){
	    if (sg1->get_wcpt_vec().back().index == pair_vertices.first->get_wcpt().index){
	      end_vertex = pair_vertices.first;
	    }else{
	      end_vertex = pair_vertices.second;
	    }
	  }else{
	    if (sg1->get_wcpt_vec().front().index == pair_vertices.first->get_wcpt().index){
	      end_vertex = pair_vertices.first;
	    }else{
	      end_vertex = pair_vertices.second;
	    }
	  }
	  if (map_vertex_segments[end_vertex].size()>1){
	    bool flag_non_ele = false;
	    for (auto it2 = map_vertex_segments[end_vertex].begin(); it2 != map_vertex_segments[end_vertex].end(); it2++){
	      WCPPID::ProtoSegment *sg2 = *it2;
	      if(sg2 == sg1) continue;
	      if (!sg2->get_flag_shower()) flag_non_ele = true;
	    }
	    
	    if ((!flag_non_ele) && map_vertex_segments[end_vertex].size()<=3) flag_good_track = true;
	    //std::cout << flag_non_ele << " " << map_vertex_segments[end_vertex].size() << " " << flag_good_track << std::endl;
	  }else{
	    flag_good_track = true;
	  }

	  //std::cout << flag_good_track << std::endl;
	}
	
	if (sg1->get_flag_shower()) n_showers ++;
	n_tracks ++;
	total_length += length;
	if (max_length < length) {
	  max_length = length;
	  max_sg = sg1;
	}

	//	std::cout << sg1->get_length()/units::cm << " " << sg1->get_medium_dQ_dx()/(43e3/units::cm) << " " << sg1->is_dir_weak() << " " << sg1->get_flag_shower() << std::endl;
      }
      int n_multi_vtx = 0;
      int n_two_vtx  = 0;
      for (auto it1 = map_vtx_segs.begin(); it1 != map_vtx_segs.end(); it1++){
	WCPPID::ProtoVertex *vtx = it1->first;
	if (it1->second.size()==2) n_two_vtx ++;
	else if (it1->second.size() >2) n_multi_vtx ++; 
      }
      
      
      //      std::cout << "kaka: " << flag_good_track << " " << n_tracks << " " << n_showers << " " << total_length/units::cm << " " << max_length/units::cm << " " << total_length/n_tracks/units::cm << " " << n_multi_vtx << " " << n_two_vtx << std::endl;

      //7003_550_27509	
      if ((!flag_good_track) && n_multi_vtx >0 && max_length < 65*units::cm && (total_length < n_tracks * 27*units::cm && total_length < 85*units::cm || total_length < n_tracks * 18*units::cm && total_length < 95*units::cm ) && n_two_vtx < 3){
	map_shower_max_sg[shower] = max_sg;
	if (shower->get_total_length() > max_length){
	  max_shower = shower;
	  max_length = shower->get_total_length();
	}

	
      }else{
	delete shower;
      }
    }

    for (auto it1 = map_shower_max_sg.begin(); it1 != map_shower_max_sg.end(); it1++){
      WCPPID::WCShower *shower = it1->first;
      WCPPID::ProtoSegment *max_sg = it1->second;
      if (shower == max_shower){
	std::cout << "[shower_clustering_connecting_to_main_vertex] Convert EM shower " << shower->get_start_segment()->get_id() << std::endl;
	shower->get_start_segment()->set_particle_type(11);
	showers.push_back(shower);
	max_sg->set_flag_avoid_muon_check(true);
	Map_Proto_Vertex_Segments& map_vtx_segs = shower->get_map_vtx_segs();
	for (auto it1 = showers.begin(); it1!=showers.end(); it1++){
	  WCPPID::WCShower *shower1 = *it1;
	  if (shower == shower1) continue;
	  if (shower1->get_start_vertex().second==1){
	    if (map_vtx_segs.find(shower1->get_start_vertex().first) != map_vtx_segs.end()){
	      del_showers.insert(shower1);
	    }
	  }
	}
      }else{
	delete shower;
      }
    }
    
    for (auto it1 = del_showers.begin(); it1!=del_showers.end(); it1 ++){
      WCPPID::WCShower *shower1 = *it1;
      if (shower1->get_start_vertex().first != main_vertex){
	showers.erase(find(showers.begin(), showers.end(), shower1));
	delete shower1;
      }
    }
    update_shower_maps();  
  }
  
}

void WCPPID::NeutrinoID::shower_clustering_with_nv(){
  using SCNV_Clock = std::chrono::high_resolution_clock;
  using SCNV_ms = std::chrono::duration<double, std::milli>;
  auto scnv_t_total = SCNV_Clock::now();
  auto scnv_t0 = SCNV_Clock::now();

  // connect to the main cluster ...
  shower_clustering_with_nv_in_main_cluster();
  auto scnv_dt_in_main_cluster = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();

  // examine things connecting to the main vertex
  shower_clustering_connecting_to_main_vertex();
  auto scnv_dt_connecting_to_main_vertex = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();

  //std::cout << showers.size() << std::endl;
  shower_clustering_with_nv_from_main_cluster();
  auto scnv_dt_from_main_cluster = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();
  //  std::cout << showers.size() << std::endl;
  shower_clustering_with_nv_from_vertices();
  auto scnv_dt_from_vertices = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();
  //  std::cout << showers.size() << std::endl;
  calculate_shower_kinematics();
  auto scnv_dt_calc_kine_1 = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();
  examine_merge_showers();
  auto scnv_dt_examine_merge = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();

  // check remaining clusters ...
  shower_clustering_in_other_clusters(true);
  auto scnv_dt_in_other_clusters = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();

  calculate_shower_kinematics();
  auto scnv_dt_calc_kine_2 = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();

  // examine shower trunk and added to shower
  examine_showers();
  auto scnv_dt_examine_showers = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();

  id_pi0_with_vertex();
  auto scnv_dt_id_pi0_with_vertex = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count(); scnv_t0 = SCNV_Clock::now();

  id_pi0_without_vertex();
  auto scnv_dt_id_pi0_without_vertex = std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t0).count();

  // Debug summary: showers and pi0s
  {
    std::cout << "[shower_clustering_with_nv] " << showers.size() << " shower(s)" << std::endl;
    int idx = 0;
    for (auto shower : showers) {
      if (!shower) continue;
      std::set<int> shower_clusters;
      for (auto it = map_segment_in_shower.begin(); it != map_segment_in_shower.end(); it++) {
        if (it->second == shower && it->first)
          shower_clusters.insert(it->first->get_cluster_id());
      }
      WCP::Point sp = shower->get_start_point();
      TVector3 dir = shower->get_init_dir();
      std::cout << "[shower_clustering_with_nv]   shower[" << idx++ << "]"
                << " pdg=" << shower->get_particle_type()
                << " conn=" << shower->get_start_vertex().second
                << " flag_shower=" << shower->get_flag_shower()
                << " nseg=" << shower->get_num_segments()
                << " ncls=" << shower_clusters.size()
                << " kine_charge=" << shower->get_kine_charge()/units::MeV << "MeV"
                << " start=(" << sp.x/units::cm << "," << sp.y/units::cm << "," << sp.z/units::cm << ")cm"
                << " dir=(" << dir.X() << "," << dir.Y() << "," << dir.Z() << ")"
                << std::endl;
    }
    std::cout << "[shower_clustering_with_nv] " << map_pio_id_showers.size() << " pi0(s)" << std::endl;
    for (auto it = map_pio_id_showers.begin(); it != map_pio_id_showers.end(); it++) {
      int pio_id = it->first;
      auto& pi0_shower_vec = it->second;
      auto mass_it = map_pio_id_mass.find(pio_id);
      double mass = (mass_it != map_pio_id_mass.end()) ? mass_it->second.first : 0.0;
      int    flag = (mass_it != map_pio_id_mass.end()) ? mass_it->second.second : 0;
      std::cout << "[shower_clustering_with_nv]   pi0[id=" << pio_id << "]"
                << " flag=" << flag
                << " mass=" << mass/units::MeV << "MeV"
                << std::endl;
      for (size_t i = 0; i < pi0_shower_vec.size(); i++) {
        auto s = pi0_shower_vec[i];
        if (!s) continue;
        WCP::Point sp = s->get_start_point();
        TVector3 dir = s->get_init_dir();
        std::cout << "[shower_clustering_with_nv]     pi0[id=" << pio_id << "] shower[" << i << "]"
                  << " pdg=" << s->get_particle_type()
                  << " nseg=" << s->get_num_segments()
                  << " kine_charge=" << s->get_kine_charge()/units::MeV << "MeV"
                  << " start=(" << sp.x/units::cm << "," << sp.y/units::cm << "," << sp.z/units::cm << ")cm"
                  << " dir=(" << dir.X() << "," << dir.Y() << "," << dir.Z() << ")"
                  << std::endl;
      }
    }
  }

  std::cout << " shower_clustering_with_nv timing:"
            << " in_main_cluster=" << scnv_dt_in_main_cluster << "ms"
            << " connecting_to_main_vertex=" << scnv_dt_connecting_to_main_vertex << "ms"
            << " from_main_cluster=" << scnv_dt_from_main_cluster << "ms"
            << " from_vertices=" << scnv_dt_from_vertices << "ms"
            << " calc_kine_1=" << scnv_dt_calc_kine_1 << "ms"
            << " examine_merge=" << scnv_dt_examine_merge << "ms"
            << " in_other_clusters=" << scnv_dt_in_other_clusters << "ms"
            << " calc_kine_2=" << scnv_dt_calc_kine_2 << "ms"
            << " examine_showers=" << scnv_dt_examine_showers << "ms"
            << " id_pi0_with_vertex=" << scnv_dt_id_pi0_with_vertex << "ms"
            << " id_pi0_without_vertex=" << scnv_dt_id_pi0_without_vertex << "ms"
            << std::endl;
  std::cout << "[shower_clustering_with_nv] Done -- total took " << std::chrono::duration_cast<SCNV_ms>(SCNV_Clock::now()-scnv_t_total).count() << " ms" << std::endl;

}


void WCPPID::NeutrinoID::examine_merge_showers(){
  auto it = map_vertex_to_shower.find(main_vertex);
  if (it != map_vertex_to_shower.end()){
    std::set<WCPPID::WCShower*> used_showers;
    std::map<WCPPID::WCShower*, std::set<WCPPID::WCShower*> > map_shower_merge_showers;
    for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
      WCPPID::WCShower *shower1 = *it1;
      if (shower1->get_start_segment()->get_particle_type()==13) continue;
      if (shower1->get_start_vertex().second != 1) continue;
      TVector3 dir1 = shower1->cal_dir_3vector(shower1->get_start_point(), 100*units::cm);
      for (auto it2 = it->second.begin(); it2 != it->second.end(); it2++){
	WCPPID::WCShower *shower2 = *it2;
	if (shower2->get_start_vertex().second != 2) continue;
	if (used_showers.find(shower2) != used_showers.end()) continue;
	if (shower2->get_start_segment()->get_particle_type()==13) continue;
	TVector3 dir2 = shower2->cal_dir_3vector(shower2->get_start_point(), 100*units::cm);
	//	std::cout << shower1 << " " << shower2 << " " << dir1.Angle(dir2)/3.1415926*180. << " " << dir1.Mag() << " " << dir2.Mag() << std::endl;
	if (dir1.Angle(dir2)/3.1415926*180. < 10){
	  map_shower_merge_showers[shower1].insert(shower2);
	  used_showers.insert(shower2);
	}
      }
    }

    for (auto it1 = map_shower_merge_showers.begin(); it1 != map_shower_merge_showers.end(); it1++){
      WCPPID::WCShower *shower1 = it1->first;
      for (auto it2 = it1->second.begin(); it2 != it1->second.end(); it2++){
	WCPPID::WCShower *shower2 = *it2;
	shower1->add_shower(shower2);
      }
      shower1->update_particle_type();
      shower1->calculate_kinematics();
      double kine_charge = cal_kine_charge(shower1);
      shower1->set_kine_charge(kine_charge);
      shower1->set_flag_kinematics(true);
    }

    //std::cout << used_showers.size() << std::endl;
    
    for (auto it1 = used_showers.begin(); it1 != used_showers.end(); it1++){
      showers.erase(find(showers.begin(), showers.end(), *it1));
    }
    if (used_showers.size() >0){
      update_shower_maps();
    }
  }
}

void WCPPID::NeutrinoID::id_pi0_without_vertex(){

  //  std::cout << (*map_vertex_segments[main_vertex].begin())->get_cluster_id() << " " << (*map_vertex_segments[main_vertex].begin())->get_id() << " " << (*map_vertex_segments[main_vertex].rbegin())->get_cluster_id() << " " << (*map_vertex_segments[main_vertex].rbegin())->get_id() << std::endl;


  
  // test main vertex ...
  if (map_vertex_segments[main_vertex].size()>2) return; // more than one shower
  if (map_segment_in_shower.find(*map_vertex_segments[main_vertex].begin()) == map_segment_in_shower.end() &&
      map_segment_in_shower.find(*map_vertex_segments[main_vertex].rbegin()) == map_segment_in_shower.end()
      || segments_in_long_muon.find(*map_vertex_segments[main_vertex].begin()) != segments_in_long_muon.end()
      || segments_in_long_muon.find(*map_vertex_segments[main_vertex].rbegin()) != segments_in_long_muon.end() 
      ) return; // not a shower connecting to main vertex ...

  std::set<WCPPID::WCShower* > good_showers;
  {
    auto it = map_vertex_to_shower.find(main_vertex);
    if (it != map_vertex_to_shower.end()){
      for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	if (pi0_showers.find(*it1) != pi0_showers.end()) return;
	if ((*it1)->get_start_vertex().second == 1) {
	  //	  std::cout <<  (*it1)->get_kine_charge() << std::endl;
	  good_showers.insert(*it1);
	}
      }
    }
    if (good_showers.size()>1){
      WCPPID::WCShower *max_shower = 0;
      double max_energy = 0;
      for (auto it1 = good_showers.begin(); it1 != good_showers.end(); it1++){
	WCPPID::WCShower *shower1 = *it1;
	double energy = shower1->get_kine_charge();
	if(energy > max_energy){
	  max_energy = energy;
	  max_shower = shower1;
	}
      }
      good_showers.clear();
      if (max_shower !=0) good_showers.insert(max_shower);
    }
  }

  
  
  
  if (map_vertex_segments[main_vertex].size()==2){
    bool flag_return = true;
    int num_showers = 0;
    for (auto it = map_vertex_segments[main_vertex].begin(); it!= map_vertex_segments[main_vertex].end(); it++){
      WCPPID::ProtoSegment *sg =*it;
      if (map_segment_in_shower.find(sg) == map_segment_in_shower.end()){
	if (sg->get_length() < 1.2*units::cm && (sg->get_flag_dir()==0 || sg->is_dir_weak())){	  
	  flag_return = false;
	}
	// std::cout << (*map_vertex_segments[main_vertex].begin())->get_length()/units::cm << " " << (*map_vertex_segments[main_vertex].rbegin())->get_length()/units::cm << std::endl;
      }else{
	//	if (segments_in_long_muon.find(sg) != segments_in_long_muon.end()) return;
	num_showers ++;
      }
    }
    if (flag_return && num_showers == map_vertex_segments[main_vertex].size()) flag_return = false;
    if (flag_return) return;
  }

  
  std::map<WCPPID::WCShower*, WCP::Line *> map_shower_line;
  //std::cout << map_vertex_to_shower[main_vertex].size() << std::endl;
  for (auto it = map_vertex_to_shower[main_vertex].begin(); it!= map_vertex_to_shower[main_vertex].end(); it++){
    // std::cout << (*it)->get_start_segment()->get_cluster_id() << " " << (*it)->get_start_segment()->get_particle_type() << std::endl;
    if ((*it)->get_start_segment()->get_particle_type()==13) continue; // a muon
    if ((*it)->get_total_length() < 3*units::cm) continue; // too short ...
    if (pi0_showers.find(*it) != pi0_showers.end()) continue; // cannot be already in a pi0
    //    std::cout << /units::cm << std::endl;
    Point test_p = (*it)->get_start_point();
    TVector3 dir = (*it)->cal_dir_3vector(test_p, 15*units::cm);
    WCP::Line *line = new Line(test_p, dir);
    map_shower_line[*it] = line;
    //std::cout << (*it)->get_start_segment()->get_cluster_id() << " " << std::endl;
  }
  for (auto it = map_vertex_to_shower.begin(); it!= map_vertex_to_shower.end(); it++){
    if (it->first == main_vertex) continue;
    
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      if ((*it1)->get_start_segment()->get_particle_type()==13) continue; // a muon 
      if ((*it1)->get_total_length() < 3*units::cm) continue; // too short ... 
      if (pi0_showers.find(*it1) != pi0_showers.end()) continue; // cannot be already in a pi0
      if ((*it1)->get_start_vertex().second != 3) continue;
      if (!(*it1)->get_start_segment()->get_flag_shower()) continue;
      
      //      std::cout << main_vertex->get_fit_pt() << " A " << (*it1) << std::endl;
      
      Point test_p = (*it1)->get_closest_point(main_vertex->get_fit_pt()).second; 
      TVector3 dir = (*it1)->cal_dir_3vector(test_p, 15*units::cm); 
      WCP::Line *line = new Line(test_p, dir); 
      map_shower_line[*it1] = line; 
      //std::cout << (*it1)->get_start_segment()->get_cluster_id() << " " << (*it1)->get_start_segment()->get_particle_type() << " " << (*it1)->get_start_segment()->get_length()/units::cm << " " << (*it1)->get_start_vertex().second << std::endl;
    }
  }
  //  std::cout << map_shower_line.size() << std::endl;
  
  if (map_shower_line.size() > 1){
    // save information ... 
    std::map<std::pair<WCPPID::WCShower*, WCPPID::WCShower*>, std::pair<double, Point> > map_shower_pair_mass_point;
    
    for (auto it = map_shower_line.begin(); it!=map_shower_line.end(); it++){
      WCPPID::WCShower *shower_1 = it->first;
      WCP::Line *l1 = it->second;
      double length_1 = shower_1->get_total_length();
      for (auto it1 = it; it1 != map_shower_line.end(); it1++){
	WCPPID::WCShower *shower_2 = it1->first;
	if (shower_1 == shower_2) continue;
	WCP::Line *l2 = it1->second;
	if (l1->get_p1() == l2->get_p1()) continue;
	double length_2 = shower_2->get_total_length();

	auto pair_points = l1->closest_dis_points(*l2);
	Point center(0,0,0);

	if (l1->get_dir().Angle(l2->get_dir())  ==0 ) continue;
	//	std::cout << l1->get_p1() << " " << l2->get_p1() << " " << l1->get_dir().Angle(l2->get_dir()) << std::endl;
	//	std::cout << length_1/units::cm << " " << length_2/units::cm << std::endl;
	
	if (length_1 > 15*units::cm && length_2 > 15*units::cm){
	  
	  center.x = (pair_points.first.x + pair_points.second.x)/2.;
	  center.y = (pair_points.first.y + pair_points.second.y)/2.;
	  center.z = (pair_points.first.z + pair_points.second.z)/2.;
	  //calculate mass ...
	  TVector3 dir1(l1->get_p1().x - center.x, l1->get_p1().y - center.y, l1->get_p1().z - center.z);
	  TVector3 dir2(l2->get_p1().x - center.x, l2->get_p1().y - center.y, l2->get_p1().z - center.z);
	  
	  if (dir1.Mag() < 3*units::cm) dir1 = l1->get_dir();
	  if (dir2.Mag() < 3*units::cm) dir2 = l2->get_dir();

	  
	  if (dir1.Angle(l1->get_dir())/3.1415926*180.>25 || dir2.Angle(l2->get_dir())/3.1415926*180. >25) continue;
	  double angle = dir1.Angle(dir2);
	  double mass_pio = sqrt(4*shower_1->get_kine_charge()* shower_2->get_kine_charge()*pow(sin(angle/2.),2));
	  map_shower_pair_mass_point[std::make_pair(shower_1,shower_2)] = std::make_pair(mass_pio, center);
	  //	  std::cout << dir1.Mag()/units::cm << " A " << dir2.Mag()/units::cm << " " << dir1.Angle(l1->get_dir())/3.1415926*180. << " " << dir2.Angle(l2->get_dir())/3.1415926*180. << " " << l1->closest_dis(*l2)/units::cm << " " << mass_pio/units::MeV << std::endl;
	  
	}else if (length_1 > 15*units::cm || length_2 > 15*units::cm){

	  TVector3 dir1, dir2;
	  if (length_1 > length_2){
	    center.x = (pair_points.first.x + pair_points.second.x)/2.;
	    center.y = (pair_points.first.y + pair_points.second.y)/2.;
	    center.z = (pair_points.first.z + pair_points.second.z)/2.;

	    //	    std::cout << center << " B" << std::endl;
	    
	    // update directional information for the small one ...
	    Point test_p = shower_2->get_closest_point(center).second;
	    TVector3 dir3 = shower_2->cal_dir_3vector(test_p, 15*units::cm);
	    WCP::Line l3(test_p, dir3);
	    pair_points = l1->closest_dis_points(l3);
	    center = pair_points.first;
	    dir1.SetXYZ(l1->get_p1().x - center.x, l1->get_p1().y - center.y, l1->get_p1().z - center.z);
	    dir2.SetXYZ(test_p.x - center.x, test_p.y - center.y, test_p.z - center.z);
	    if (dir1.Angle(l1->get_dir())/3.1415926*180.>25 || dir2.Angle(l3.get_dir())/3.1415926*180. >25) continue;
	    double angle = dir1.Angle(dir2);
	    double mass_pio = sqrt(4*shower_1->get_kine_charge()* shower_2->get_kine_charge()*pow(sin(angle/2.),2));
	    map_shower_pair_mass_point[std::make_pair(shower_1,shower_2)] = std::make_pair(mass_pio, center);
	    //	    std::cout << dir1.Mag()/units::cm << " B " << dir2.Mag()/units::cm << " " << dir1.Angle(l1->get_dir())/3.1415926*180. << " " << dir2.Angle(l3.get_dir())/3.1415926*180. << " " << l1->closest_dis(*l2)/units::cm << " " << mass_pio/units::MeV << std::endl;
	  }else{
	     center.x = (pair_points.first.x + pair_points.second.x)/2.;
	     center.y = (pair_points.first.y + pair_points.second.y)/2.;
	     center.z = (pair_points.first.z + pair_points.second.z)/2.;

	     //	     std::cout << center << " C" << std::endl;
	     
	     Point test_p = shower_1->get_closest_point(center).second;
	     TVector3 dir3 = shower_1->cal_dir_3vector(test_p, 15*units::cm);
	     WCP::Line l3(test_p, dir3);
	     pair_points = l3.closest_dis_points(*l2);
	     center = pair_points.second;
	     dir2.SetXYZ(l2->get_p1().x - center.x, l2->get_p1().y - center.y, l2->get_p1().z - center.z);
	     dir1.SetXYZ(test_p.x - center.x, test_p.y - center.y, test_p.z - center.z);
	     if (dir1.Angle(l3.get_dir())/3.1415926*180.>25 || dir2.Angle(l2->get_dir())/3.1415926*180. >25) continue;
	     double angle = dir1.Angle(dir2);
	     double mass_pio = sqrt(4*shower_1->get_kine_charge()* shower_2->get_kine_charge()*pow(sin(angle/2.),2));
	     map_shower_pair_mass_point[std::make_pair(shower_1,shower_2)] = std::make_pair(mass_pio, center);
	     //	     std::cout << dir1.Mag()/units::cm << " C " << dir2.Mag()/units::cm << " " << dir1.Angle(l3.get_dir())/3.1415926*180. << " " << dir2.Angle(l2->get_dir())/3.1415926*180. << " " << l1->closest_dis(*l2)/units::cm << " " << angle/3.1415926*180. << " " << mass_pio/units::MeV << std::endl;
	  }
	  
	}else{
	  break;
	}


      } // loop over showers
    } // loop over showers


    // save pi0 ...
    double max_energy = kine_pio_energy_1 + kine_pio_energy_2;
    
    for (auto it = map_shower_pair_mass_point.begin(); it!= map_shower_pair_mass_point.end(); it++){
      WCPPID::WCShower* shower_1 = it->first.first;
      WCPPID::WCShower* shower_2 = it->first.second;

      float mass_save = it->second.first;
      Point vtx_point = it->second.second;

      float energy_1 = shower_1->get_kine_charge();
      float energy_2 = shower_2->get_kine_charge();
      
      if (energy_1 + energy_2 >= max_energy){
	kine_pio_flag = 2;
	kine_pio_mass = mass_save;
	kine_pio_vtx_dis = sqrt(pow(vtx_point.x - main_vertex->get_fit_pt().x,2) + pow(vtx_point.y - main_vertex->get_fit_pt().y,2) + pow(vtx_point.z - main_vertex->get_fit_pt().z,2) );
	
	kine_pio_energy_1 = energy_1;
	kine_pio_energy_2 = energy_2;

	kine_pio_dis_1 = sqrt(pow(vtx_point.x - shower_1->get_start_point().x,2) + pow(vtx_point.y - shower_1->get_start_point().y,2) + pow(vtx_point.z - shower_1->get_start_point().z,2));
	kine_pio_dis_2 = sqrt(pow(vtx_point.x - shower_2->get_start_point().x,2) + pow(vtx_point.y - shower_2->get_start_point().y,2) + pow(vtx_point.z - shower_2->get_start_point().z,2));

	TVector3 dir1;
	dir1.SetXYZ(shower_1->get_start_point().x - vtx_point.x, shower_1->get_start_point().y - vtx_point.y, shower_1->get_start_point().z - vtx_point.z);
	
	TVector3 dir2;
	dir2.SetXYZ(shower_2->get_start_point().x - vtx_point.x, shower_2->get_start_point().y - vtx_point.y, shower_2->get_start_point().z - vtx_point.z);
	
	
	kine_pio_theta_1 = dir1.Theta();
	kine_pio_theta_2 = dir2.Theta();

	kine_pio_phi_1 = dir1.Phi();
	kine_pio_phi_2 = dir2.Phi();

	kine_pio_angle = dir1.Angle(dir2);
	
      }
    }
    //

    
    double mass_diff = 1e9;
    double mass_save = 0;
    WCPPID::WCShower *shower_1 = 0;
    WCPPID::WCShower *shower_2 = 0;
    double mass_offset = 10*units::MeV;
    Point vtx_point;
    for (auto it = map_shower_pair_mass_point.begin(); it!= map_shower_pair_mass_point.end(); it++){
     
      if (fabs(it->second.first - 135*units::MeV + mass_offset) < mass_diff){ // hack pi0 mass to a slightly lower value ...
	if (good_showers.find(it->first.first) == good_showers.end() && good_showers.find(it->first.second) == good_showers.end()) continue;
	shower_1 = it->first.first;
	shower_2 = it->first.second;

	mass_diff = fabs(it->second.first - 135*units::MeV + mass_offset);// hack pi0 mass to a slightly lower value ...
	mass_save = it->second.first;
	vtx_point = it->second.second;
      }
    }
    
    if (mass_diff < 60*units::MeV){
      pi0_showers.insert(shower_1);
      pi0_showers.insert(shower_2);
      int pio_id = acc_segment_id; acc_segment_id ++;
      map_shower_pio_id[shower_1] = pio_id;
      map_shower_pio_id[shower_2] = pio_id;
      map_pio_id_mass[pio_id] = std::make_pair(mass_save,2);
      map_pio_id_showers[pio_id].push_back(shower_1);
      map_pio_id_showers[pio_id].push_back(shower_2);

      
      // hack main vertex 
      main_vertex->set_fit_pt(vtx_point);
      main_vertex->set_dQ(0);

      if (shower_1->get_start_vertex().first == main_vertex && shower_1->get_start_vertex().second==1){
	for (auto it = map_vertex_segments[main_vertex].begin(); it!= map_vertex_segments[main_vertex].end(); it++){
	  WCPPID::ProtoSegment *sg =*it;
	  if (sg == shower_1->get_start_segment()) continue;
	  shower_1->add_segment(sg, map_segment_vertices);
	}
      }
      if (shower_2->get_start_vertex().first == main_vertex && shower_2->get_start_vertex().second==1){
	for (auto it = map_vertex_segments[main_vertex].begin(); it!= map_vertex_segments[main_vertex].end(); it++){
	  WCPPID::ProtoSegment *sg =*it;
	  if (sg == shower_2->get_start_segment()) continue;
	  shower_2->add_segment(sg, map_segment_vertices);
	}
      }
      
      shower_1->set_start_vertex(main_vertex, 2);
      shower_1->calculate_kinematics();

      shower_2->set_start_vertex(main_vertex, 2);
      shower_2->calculate_kinematics();
      
      update_shower_maps();
      std::cout << "[id_pi0_without_vertex] Pi0 (displaced vertex) found with mass: " << " " << mass_save/units::MeV << " MeV with " << shower_1->get_kine_charge()/units::MeV << " MeV + " << shower_2->get_kine_charge()/units::MeV << " MeV" << std::endl;
    }
        
  } // more than one shower

  for (auto it = map_shower_line.begin(); it!=map_shower_line.end(); it++){
    WCP::Line *l1 = it->second;
    delete l1;
  }
  
}


void WCPPID::NeutrinoID::id_pi0_with_vertex(){

  // figure out all unconnected showers ...
  std::set<WCPPID::WCShower* > disconnected_showers;
  std::map<WCShower*, TVector3> map_shower_dir;
  for (auto it = map_vertex_to_shower.begin(); it!= map_vertex_to_shower.end(); it++){
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      if ((*it1)->get_start_vertex().second ==2 && fabs((*it1)->get_particle_type())!=13){ // no muon ...
	disconnected_showers.insert(*it1);
	map_shower_dir[*it1] = (*it1)->cal_dir_3vector((*it1)->get_start_point(), 15*units::cm);
      }else if ((*it1)->get_start_vertex().second == 1){
  	map_shower_dir[*it1] = (*it1)->cal_dir_3vector((*it1)->get_start_vertex().first->get_fit_pt(), 15*units::cm);
      }
    }
  }
  //  std::cout << disconnected_showers.size() <<  " " << map_shower_dir.size() << std::endl;

  std::set<WCPPID::ProtoVertex*> candidate_vertices;
  candidate_vertices.insert(main_vertex);
  for (auto it = map_vertex_to_shower.begin(); it!= map_vertex_to_shower.end(); it++){
    bool flag_add = true;
    if (map_vertex_in_shower.find(it->first) != map_vertex_in_shower.end()){
      flag_add = false;
      if (it->first == map_vertex_in_shower[it->first]->get_start_vertex().first)         flag_add = true; 
    }
    if (flag_add) candidate_vertices.insert(it->first);
  }
  //std::cout << candidate_vertices.size() << std::endl;

  std::map<std::pair<WCPPID::WCShower*, WCPPID::WCShower*>, std::vector<std::pair<double, WCPPID::ProtoVertex*> > > map_shower_pair_mass_vertex;
  
  for (auto it = candidate_vertices.begin(); it!= candidate_vertices.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    
    std::vector<WCPPID::WCShower*> tmp_showers;
    std::map<WCPPID::WCShower*, TVector3> local_dirs;
    
    auto it2 = map_vertex_to_shower.find(vtx);
    
    if (it2 != map_vertex_to_shower.end()){
      for (auto it1 = it2->second.begin(); it1!=it2->second.end(); it1++){
	if ((*it1)->get_start_vertex().second ==1 && fabs((*it1)->get_particle_type())!=13){ // no muon ...
	  tmp_showers.push_back(*it1);
	  local_dirs[*it1] = (*it1)->get_init_dir();
	}
      }
    }
    for (auto it1 = disconnected_showers.begin(); it1 != disconnected_showers.end(); it1++){
      TVector3 dir1 = map_shower_dir[*it1];
      TVector3 dir2((*it1)->get_start_point().x - vtx->get_fit_pt().x,
		    (*it1)->get_start_point().y - vtx->get_fit_pt().y,
		    (*it1)->get_start_point().z - vtx->get_fit_pt().z);

     
      
      //      std::cout << dir1.Angle(dir2)/3.1415926*180. << std::endl;
      if ((*it1)->get_start_vertex().first == vtx){
	tmp_showers.push_back(*it1);
	local_dirs[*it1] = (*it1)->get_init_dir();
      }else if ( dir1.Angle(dir2)/3.1415926*180. < 30){
	tmp_showers.push_back(*it1);
	local_dirs[*it1] = dir2;
      }
    }
    
    if (tmp_showers.size()>1){
      for (size_t i=0;i!= tmp_showers.size();i++){
	WCPPID::WCShower *shower_1 = tmp_showers.at(i);
	TVector3 dir1 = local_dirs[shower_1];
	for (size_t j=i+1; j<tmp_showers.size();j++){
	  WCPPID::WCShower *shower_2 = tmp_showers.at(j);
	  TVector3 dir2 = local_dirs[shower_2];
	  double angle = dir1.Angle(dir2);
	  double mass_pio = sqrt(4*shower_1->get_kine_charge()* shower_2->get_kine_charge()*pow(sin(angle/2.),2));
	  if (shower_1->get_start_vertex().second==1 && shower_2->get_start_vertex().second==1) continue;
	  
	  map_shower_pair_mass_vertex[std::make_pair(shower_1, shower_2)].push_back(std::make_pair(mass_pio, vtx));

	  //	  if (shower_1->get_kine_charge() > 80*units::MeV && shower_2->get_kine_charge() > 80*units::MeV)
	  /* std::cout << vtx->get_fit_pt().x + dir1.X() * 10*units::cm << " " */
	  /* 	    << vtx->get_fit_pt().y + dir1.Y() * 10*units::cm << " " */
	  /* 	    << vtx->get_fit_pt().z + dir1.Z() * 10*units::cm << " " */
	  /* 	    << vtx->get_fit_pt().x + dir2.X() * 10*units::cm << " " */
	  /* 	    << vtx->get_fit_pt().y + dir2.Y() * 10*units::cm << " " */
	  /* 	    << vtx->get_fit_pt().z + dir2.Z() * 10*units::cm << " " */
	  /* 	    << std::endl; */
	  
	   
	  //	  std::cout << vtx->get_id() << " " << i << " " << j << " " << shower_1->get_kine_charge()/units::MeV << " " << shower_2->get_kine_charge()/units::MeV << " " << dir1.Mag() << " " << dir2.Mag() << " " << angle/3.1415926*180. << " " << sqrt(4*shower_1->get_kine_charge()* shower_2->get_kine_charge()*pow(sin(angle/2.),2))/units::MeV<< std::endl;
	} // loop shower
      } // loop shower
    } // more than 1 shower
  } // candidate vertices ...


  // store the pi0 in general for energy
  float max_energy = 0;
  for (auto it = map_shower_pair_mass_vertex.begin(); it != map_shower_pair_mass_vertex.end(); it++){
    WCPPID::WCShower *shower_1 = it->first.first;
    WCPPID::WCShower *shower_2 = it->first.second;
    float energy_1 = shower_1->get_kine_charge();
    float energy_2 = shower_2->get_kine_charge();

    if (energy_1 + energy_2 > max_energy){
      kine_pio_vtx_dis = 1000*units::cm;
      
      for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	double mass_pio = it1->first;
	WCPPID::ProtoVertex *vtx = it1->second;

	Point vtx_point = vtx->get_fit_pt();
	double temp_dis = sqrt(pow(vtx_point.x - main_vertex->get_fit_pt().x,2) + pow(vtx_point.y - main_vertex->get_fit_pt().y,2) + pow(vtx_point.z - main_vertex->get_fit_pt().z,2) );
	
	if (vtx != shower_1->get_start_vertex().first && vtx != shower_2->get_start_vertex().first) continue;

	//	std::cout << mass_pio/units::MeV << " " << vtx << " " << shower_1->get_start_vertex().first << " " << shower_2->get_start_vertex().first << std::endl;
	
	if (temp_dis < kine_pio_vtx_dis){
	  kine_pio_vtx_dis = temp_dis;
	  
	  max_energy = energy_1 + energy_2;

	  kine_pio_flag = 1;
	  kine_pio_mass = mass_pio;

	  //	std::cout <<  kine_pio_vtx_dis << " Xin " << std::endl;
	  
	  kine_pio_energy_1 = energy_1;
	  kine_pio_energy_2 = energy_2;
	  
	  kine_pio_dis_1 = sqrt(pow(vtx->get_fit_pt().x - shower_1->get_start_point().x,2) + pow(vtx->get_fit_pt().y - shower_1->get_start_point().y,2) + pow(vtx->get_fit_pt().z - shower_1->get_start_point().z,2));
	  kine_pio_dis_2 = sqrt(pow(vtx->get_fit_pt().x - shower_2->get_start_point().x,2) + pow(vtx->get_fit_pt().y - shower_2->get_start_point().y,2) + pow(vtx->get_fit_pt().z - shower_2->get_start_point().z,2));
	  
	  TVector3 dir1;
	  if (kine_pio_dis_1 < 3*units::cm){
	    dir1 = shower_1->cal_dir_3vector(shower_1->get_start_vertex().first->get_fit_pt(), 15*units::cm);
	  }else{
	    dir1.SetXYZ(shower_1->get_start_point().x - vtx->get_fit_pt().x, shower_1->get_start_point().y - vtx->get_fit_pt().y, shower_1->get_start_point().z - vtx->get_fit_pt().z);
	  }
	  TVector3 dir2;
	  if (kine_pio_dis_2 < 3*units::cm){
	    dir2 = shower_2->cal_dir_3vector(shower_2->get_start_vertex().first->get_fit_pt(), 15*units::cm);
	}else{
	    dir2.SetXYZ(shower_2->get_start_point().x - vtx->get_fit_pt().x, shower_2->get_start_point().y - vtx->get_fit_pt().y, shower_2->get_start_point().z - vtx->get_fit_pt().z);
	  }
	  
	  kine_pio_theta_1 = dir1.Theta();
	  kine_pio_theta_2 = dir2.Theta();
	  
	  kine_pio_phi_1 = dir1.Phi();
	  kine_pio_phi_2 = dir2.Phi();
	  
	  kine_pio_angle = dir1.Angle(dir2);
	}
      }
    }
    
  }
  // store the pi0 for energy ...
  

  // pi0 for the pattern recognition ...
  while(map_shower_pair_mass_vertex.size()>0){
    double mass_diff = 1e9;
    double mass_save = 0;
    WCPPID::WCShower *shower_1 = 0;
    WCPPID::WCShower *shower_2 = 0;
    double mass_offset = 10*units::MeV;
    WCPPID::ProtoVertex* vtx;
    double mass_penalty = 0;
    double tmp_mass_penalty = 0;
    for (auto it = map_shower_pair_mass_vertex.begin(); it!= map_shower_pair_mass_vertex.end(); it++){
      for (auto it1 = it->second.begin(); it1!= it->second.end(); it1++){
	// 7004_365_18260
	if (it->first.first->get_start_vertex().second == 2 && it->first.second->get_start_vertex().second == 2){
	  tmp_mass_penalty = 6*units::MeV;
	} else{
	  tmp_mass_penalty = 0;
	}

	//	std::cout << (*it1).first << " " << it->first.first->get_start_vertex().second << " " << it->first.second->get_start_vertex().second << std::endl;

	if ((*it1).first - 135*units::MeV + mass_offset < 35*units::MeV && (*it1).first - 135*units::MeV + mass_offset > -25*units::MeV){
	  if (fabs((*it1).first - 135*units::MeV + mass_offset) - tmp_mass_penalty  < fabs(mass_diff) - mass_penalty){ // hack pi0 mass to a slightly lower value ...
	    mass_diff = (*it1).first - 135*units::MeV + mass_offset;// hack pi0 mass to a slightly lower value ...
	    mass_penalty = tmp_mass_penalty;
	    mass_save = (*it1).first;
	    shower_1 = it->first.first;
	    shower_2 = it->first.second;
	    vtx = (*it1).second;
	  }
	}
      }
    }
    
    if (mass_diff < 35*units::MeV && mass_diff > -25*units::MeV){
      pi0_showers.insert(shower_1);
      pi0_showers.insert(shower_2);
      int pio_id = acc_segment_id; acc_segment_id ++;
      map_shower_pio_id[shower_1] = pio_id;
      map_shower_pio_id[shower_2] = pio_id;
      map_pio_id_mass[pio_id] = std::make_pair(mass_save,1);
      map_pio_id_showers[pio_id].push_back(shower_1);
      map_pio_id_showers[pio_id].push_back(shower_2);

      if (shower_1->get_start_vertex().first != vtx){
	shower_1->set_start_vertex(vtx, 2);
	shower_1->calculate_kinematics();
      }
      if (shower_2->get_start_vertex().first != vtx){
	shower_2->set_start_vertex(vtx, 2);
	shower_2->calculate_kinematics();
      }
      
      std::cout << "[id_pi0_with_vertex] Pi0 found with mass: " << " " << mass_save/units::MeV << " MeV with " << shower_1->get_kine_charge()/units::MeV << " MeV + " << shower_2->get_kine_charge()/units::MeV << " MeV" << std::endl;
    }else{
      break;
    }
    
    std::vector<std::pair<WCPPID::WCShower*, WCPPID::WCShower*> > to_be_removed;
    for (auto it = map_shower_pair_mass_vertex.begin(); it!= map_shower_pair_mass_vertex.end(); it++){
      if (it->first.first == shower_1 || it->first.first == shower_2 || it->first.second == shower_1 || it->first.second == shower_2)
	to_be_removed.push_back(it->first);
    }
    for (auto it = to_be_removed.begin(); it != to_be_removed.end(); it++){
      map_shower_pair_mass_vertex.erase(*it);
    }
  }
  
  
  // find the pi0 vertex, and the other track coming in, cannot be a muon, change to pion ...
  std::set<WCPPID::ProtoVertex* > pi0_vertices;
  for (auto it = pi0_showers.begin(); it!= pi0_showers.end(); it++){
    pi0_vertices.insert( (*it)->get_start_vertex().first);
  }
  for (auto it = pi0_vertices.begin(); it!= pi0_vertices.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    for (auto it1 = map_vertex_segments[vtx].begin(); it1 != map_vertex_segments[vtx].end(); it1++){
      WCPPID::ProtoSegment *sg = *it1;
      bool flag_start;
      if (sg->get_wcpt_vec().front().index == vtx->get_wcpt().index)
	flag_start = true;
      else if (sg->get_wcpt_vec().back().index == vtx->get_wcpt().index)
	flag_start = false;
      
      if ((flag_start && sg->get_flag_dir()==-1 || (!flag_start) && sg->get_flag_dir()==1) && (fabs(sg->get_particle_type())==13 || sg->get_particle_type()==0)){ // muon
	// in
	// change to pion ...
	TPCParams& mp = Singleton<TPCParams>::Instance();
	sg->set_particle_type(211);
	sg->set_particle_mass(mp.get_mass_pion());
	if (sg->get_particle_4mom(3)>0)
	  sg->cal_4mom();
      }
    }
  }
  //  std::cout << pi0_vertices.size() << std::endl;
  
}

void WCPPID::NeutrinoID::shower_clustering_with_nv_from_vertices(){
  // first map the sg vs. cluster existing ...
  //  std::cout << map_cluster_segments.size() << " " << other_clusters.size() << " " << map_segment_cluster.size() << " " << map_segment_vertices.size() << std::endl;
  std::map<PR3DCluster*, std::pair<WCP::Point, double> > map_cluster_center_point;
  
  for (auto it = other_clusters.begin(); it!=other_clusters.end(); it++){
    WCPPID::PR3DCluster *cluster = *it;
    auto it1 = map_cluster_segments.find(cluster);
    if (it1  == map_cluster_segments.end()) continue;
    double acc_length = 0;
    double acc_length1 = 0;
    Point p(0,0,0);
    Int_t np = 0;
    for (auto it2 = it1->second.begin(); it2!=it1->second.end(); it2++){
      WCPPID::ProtoSegment *seg = *it2;
      if (map_segment_in_shower.find(seg) != map_segment_in_shower.end()) continue;
      // if (seg->get_cluster_id()==112) std::cout << seg->get_id() << " " << seg->get_flag_shower() << " " << seg->get_particle_type() << std::endl;
      if (seg->get_flag_shower()==1 || seg->get_particle_type()==0 || (abs(seg->get_particle_type())==13 || abs(seg->get_particle_type())==211) && seg->is_dir_weak()){
	acc_length += seg->get_length();
	PointVector& pts = seg->get_point_vec();
	for (size_t i=0; i!= pts.size(); i++){
	  p.x += pts.at(i).x;
	  p.y += pts.at(i).y;
	  p.z += pts.at(i).z;
	  np ++;
	}
      }
      if (seg->get_particle_type()!=11 && (!seg->get_flag_shower()))
	acc_length1 += seg->get_length();
      //      std::cout << seg->get_particle_type() << " " << seg->get_length()/units::cm << " " << seg->is_dir_weak() << std::endl;
    }

    //   std::cout << cluster->get_cluster_id() << " " << acc_length/units::cm << " " << acc_length1/units::cm << std::endl;
    if (acc_length > 1.0*units::cm && acc_length >= acc_length1 || acc_length > 10*units::cm){
      p.x /= np;
      p.y /= np;
      p.z /= np;
      map_cluster_center_point[cluster] = std::make_pair(p, acc_length);
    }
  }


  
  // list the main vertices ...
  std::vector<WCPPID::ProtoVertex*> main_cluster_vertices;
  for (auto it = map_vertex_segments.begin(); it != map_vertex_segments.end(); it++){
    if (it->first->get_cluster_id()==main_cluster->get_cluster_id()){
      if (it->first != main_vertex){
	if (vertices_in_long_muon.find(it->first) != vertices_in_long_muon.end()) continue;
	if (map_vertex_in_shower.find(it->first) != map_vertex_in_shower.end()) continue;
      }
      main_cluster_vertices.push_back(it->first);
    }
  }

 
  std::map<WCPPID::PR3DCluster*, cluster_point_info > map_cluster_pi;
  
  // start to check with each of clusters in main cluster ...
  for (auto it = map_cluster_center_point.begin(); it!= map_cluster_center_point.end(); it++){
    WCPPID::PR3DCluster *cluster = it->first;
    Point center_p = it->second.first;
    PointVector total_pts;
    for (auto it1 = map_cluster_segments[cluster].begin(); it1!= map_cluster_segments[cluster].end(); it1++){
      PointVector& pts = (*it1)->get_point_vec();
      total_pts.insert(total_pts.end(), pts.begin(), pts.end());
    }
    WCP::ToyPointCloud pcloud;
    pcloud.AddPoints(total_pts);
    pcloud.build_kdtree_index();

    cluster_point_info min_pi;
    min_pi.cluster = cluster;
    min_pi.min_angle = 90;
    min_pi.min_dis = 1e9;
    min_pi.min_vertex = 0;
    cluster_point_info main_pi;
    main_pi.cluster = cluster;
    main_pi.min_vertex = main_vertex;
    
    for (size_t i=0;i!=main_cluster_vertices.size();i++){
      std::pair<double, WCP::Point> result = pcloud.get_closest_point(main_cluster_vertices.at(i)->get_fit_pt());
      TVector3 v1(result.second.x - main_cluster_vertices.at(i)->get_fit_pt().x,
		  result.second.y - main_cluster_vertices.at(i)->get_fit_pt().y,
		  result.second.z - main_cluster_vertices.at(i)->get_fit_pt().z);
      TVector3 v2(center_p.x - result.second.x,
		  center_p.y - result.second.y,
		  center_p.z - result.second.z);
      Point near_center = pcloud.get_center_point_radius(result.second, 2*units::cm);
      TVector3 v3(near_center.x - result.second.x,
		  near_center.y - result.second.y,
		  near_center.z - result.second.z);
      double angle = v1.Angle(v2)/3.1415926*180.;
      if (angle < 30 || result.first < 5*units::cm && angle < 45)
	angle = std::min(angle , v1.Angle(v3)/3.1415926*180.);
      if (angle < 7.5){
	if (result.first*sin(angle/180.*3.1415926) < min_pi.min_dis*sin(min_pi.min_angle/180.*3.1415926) && angle < 90){
	  min_pi.min_angle = angle;
	  min_pi.min_dis = result.first;
	  min_pi.min_vertex = main_cluster_vertices.at(i);
	  min_pi.min_point = result.second;
	}
      }else{
	if (angle < min_pi.min_angle){
	  min_pi.min_angle = angle;
	  min_pi.min_dis = result.first;
	  min_pi.min_vertex = main_cluster_vertices.at(i);
	  min_pi.min_point = result.second;
	}
      }
      
      if (main_cluster_vertices.at(i) == main_vertex) {
	main_pi.min_angle = angle;
	main_pi.min_dis = result.first;
	main_pi.min_point = result.second;
      }
      
      // std::cout << i << " " << v1.Angle(v2)/3.1415926*180. << " " << v1.Angle(v3)/3.1415926*180. << " " << result.first/units::cm << std::endl;
    }
    if (min_pi.min_vertex ==0){
      min_pi.min_angle = main_pi.min_angle;
      min_pi.min_vertex = main_vertex;
      min_pi.min_point = main_pi.min_point;
      min_pi.min_dis = main_pi.min_dis;
    }
    //std::cout << main_vertex << " " << min_pi.min_vertex << " " << main_pi.min_dis/units::cm << " " << min_pi.min_dis/units::cm << std::endl;
    
    double vtx_dis = sqrt(pow(main_vertex->get_fit_pt().x - min_pi.min_vertex->get_fit_pt().x,2) + pow(main_vertex->get_fit_pt().y - min_pi.min_vertex->get_fit_pt().y,2) + pow(main_vertex->get_fit_pt().z - min_pi.min_vertex->get_fit_pt().z,2));

    // std::cout << cluster->get_cluster_id() << " " << main_pi.min_point  << " " << main_vertex->get_fit_pt() << " " <<  main_pi.min_angle << " " << min_pi.min_angle << " " << " " << main_pi.min_dis/units::cm << " " << min_pi.min_dis/units::cm << " " << vtx_dis/units::cm << std::endl;
    
    if (main_pi.min_angle < min_pi.min_angle + 3  && main_pi.min_dis < min_pi.min_dis * 1.2 &&
	(min_pi.min_angle > 0.9 * main_pi.min_angle || vtx_dis < 1.5*units::cm)
	){
      map_cluster_pi[cluster] = main_pi;
    }else{
      map_cluster_pi[cluster] = min_pi;
    }
    // hack for now ...
    //    std::cout << cluster->get_cluster_id() << " " << map_cluster_pi[cluster].min_angle << std::endl;
    // map_cluster_pi[cluster] = main_pi;
  }
  


  std::vector<cluster_point_info > vec_pi;
  for (auto it = map_cluster_pi.begin(); it!=map_cluster_pi.end(); it++){
    vec_pi.push_back(it->second);
  }
  std::sort(vec_pi.begin(), vec_pi.end(), sortbydis);
  //for (size_t i=0;i!=vec_pi.size();i++){
  // std::cout << vec_pi.at(i).min_dis << std::endl;
  //}

  std::map<WCPPID::PR3DCluster*, WCPPID::ProtoVertex* > map_cluster_associated_vertex;
  for (size_t i=0;i!=vec_pi.size();i++){
    if (vec_pi.at(i).min_angle < 10)
      map_cluster_associated_vertex[vec_pi.at(i).cluster] = vec_pi.at(i).min_vertex;
  }

  /* for (auto it = map_segment_in_shower.begin(); it!= map_segment_in_shower.end(); it++){ */
  /*   std::cout << it->first->get_id() << " "; */
  /* } */
  /* std::cout << std::endl; */


  
  
  for (size_t i=0;i!=vec_pi.size();i++){
    //    if (i>=2) continue;
    
    // find the segment and/or create the new vertices ...
    WCPPID::PR3DCluster *cluster = vec_pi.at(i).cluster;
    WCPPID::ProtoVertex *vertex = vec_pi.at(i).min_vertex;
    WCP::Point point = vec_pi.at(i).min_point;
    WCPPID::ProtoSegment* sg1 = 0;
    double angle = vec_pi.at(i).min_angle;

    //    std::cout <<  angle << " " << vec_pi.at(i).min_dis/units::cm << std::endl;
    
    if (angle > 50 && vec_pi.at(i).min_dis > 6*units::cm || angle > 60) continue;
    
    for (auto it = map_cluster_segments[cluster].begin(); it!=map_cluster_segments[cluster].end();it++){
      WCPPID::ProtoSegment *sg = *it;
      if (map_segment_in_shower.find(sg) != map_segment_in_shower.end() ) continue;
      std::pair<double, WCP::Point> result = sg->get_closest_point(point);
      if (result.first < 0.01*units::cm){
	sg1 = sg;
	break;
      }
    }
    if (sg1 == 0 ) continue;

    // create the new shower ...
    WCPPID::WCShower *shower = new WCPPID::WCShower();
    //    std::cout << shower << " " << angle << std::endl;
    shower->set_start_vertex(vertex, 2); // second type ...
    showers.push_back(shower); 

    //std::cout << sg1->get_id() << " " << sqrt(pow(sg1->get_point_vec().front().x-point.x,2) + pow(sg1->get_point_vec().front().y-point.y,2) + pow(sg1->get_point_vec().front().z-point.z,2))/units::cm << " " <<  sqrt(pow(sg1->get_point_vec().back().x-point.x,2) + pow(sg1->get_point_vec().back().y-point.y,2) + pow(sg1->get_point_vec().back().z-point.z,2))/units::cm << std::endl;
    
    if (sqrt(pow(sg1->get_point_vec().front().x-point.x,2) + pow(sg1->get_point_vec().front().y-point.y,2) + pow(sg1->get_point_vec().front().z-point.z,2)) < 0.01*units::cm || sqrt(pow(sg1->get_point_vec().back().x-point.x,2) + pow(sg1->get_point_vec().back().y-point.y,2) + pow(sg1->get_point_vec().back().z-point.z,2)) < 0.01*units::cm){
      shower->set_start_segment(sg1, map_segment_vertices);
    }else{
      // break the segment ...
      // std::cout << sg1 << " " << sg1->get_wcpt_vec().size() << " " << sg1->get_point_vec().size() << std::endl;
      // create two segments with filled information , and find the correct segments ...
      std::tuple<WCPPID::ProtoSegment*, WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> result1 = sg1->break_segment_at_point(point, acc_segment_id, acc_vertex_id);
      //      std::cout << "haha " << std::endl;
      
      if (std::get<1>(result1) == 0){
	shower->set_start_segment(sg1, map_segment_vertices);
      }else{
	 std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> result2 = find_vertices(sg1);
	 TVector3 v3(point.x - vertex->get_fit_pt().x, point.y - vertex->get_fit_pt().y, point.z - vertex->get_fit_pt().z);
	 //std::cout << result2.first->get_wcpt().index << " " << result2.second->get_wcpt().index << " " << std::get<0>(result1)->get_wcpt_vec().front().index << " " << std::get<2>(result1)->get_wcpt_vec().back().index << std::endl;
	 if (result2.first->get_wcpt().index == std::get<0>(result1)->get_wcpt_vec().front().index){
	   add_proto_connection(result2.first, std::get<0>(result1), cluster);
	   add_proto_connection(result2.second, std::get<2>(result1), cluster);
	 }else{
	   add_proto_connection(result2.first, std::get<2>(result1), cluster);
	   add_proto_connection(result2.second, std::get<0>(result1), cluster);
	 }
	 add_proto_connection(std::get<1>(result1), std::get<0>(result1), cluster);
	 add_proto_connection(std::get<1>(result1), std::get<2>(result1), cluster);
	 del_proto_segment(sg1);

	 TVector3 v1 = std::get<0>(result1)->cal_dir_3vector(point, 5*units::cm);
	 TVector3 v2 = std::get<2>(result1)->cal_dir_3vector(point, 5*units::cm);
	
	 //std::cout << v3.Angle(v1) << " " << v3.Angle(v2) << std::endl;
	 if ( v3.Angle(v1) <v3.Angle(v2)) {
	   shower->set_start_segment(std::get<0>(result1), map_segment_vertices);
	 }else{
	   shower->set_start_segment(std::get<2>(result1), map_segment_vertices);
	 }
      } // break segment to create a new vertices ...
    } // segment ...


    
    
    //    if (shower->get_start_segment()->get_flag_dir()==0){
    // examine vertices 
    double dis1 = sqrt(pow(shower->get_start_vertex().first->get_fit_pt().x - shower->get_start_segment()->get_point_vec().front().x,2) + pow(shower->get_start_vertex().first->get_fit_pt().y - shower->get_start_segment()->get_point_vec().front().y,2) + pow(shower->get_start_vertex().first->get_fit_pt().z - shower->get_start_segment()->get_point_vec().front().z,2));
    double dis2 = sqrt(pow(shower->get_start_vertex().first->get_fit_pt().x - shower->get_start_segment()->get_point_vec().back().x,2) + pow(shower->get_start_vertex().first->get_fit_pt().y - shower->get_start_segment()->get_point_vec().back().y,2) + pow(shower->get_start_vertex().first->get_fit_pt().z - shower->get_start_segment()->get_point_vec().back().z,2));
    
    if (dis1 < dis2){
      shower->get_start_segment()->set_flag_dir(1);
    }else{
      shower->get_start_segment()->set_flag_dir(-1);
    }
    //}
    
    //    std::cout << shower->get_start_segment()->get_particle_type() << " " << std::endl;
    if (shower->get_start_segment()->get_particle_type()==0 || fabs(shower->get_start_segment()->get_particle_type())==13){
      shower->get_start_segment()->set_particle_type(11);
      TPCParams& mp = Singleton<TPCParams>::Instance();
      shower->get_start_segment()->set_particle_mass(mp.get_mass_electron());
      shower->get_start_segment()->cal_4mom();
    }
    // cluster the other things ...
    std::set<WCPPID::ProtoSegment* > used_segments; 
    shower->complete_structure_with_start_segment(map_vertex_segments, map_segment_vertices, used_segments);
      
    TVector3 dir_shower = shower->get_start_segment()->cal_dir_3vector(point, 15*units::cm);
    TVector3 dir_main(point.x - shower->get_start_vertex().first->get_fit_pt().x,
    		      point.y - shower->get_start_vertex().first->get_fit_pt().y,
    		      point.z - shower->get_start_vertex().first->get_fit_pt().z);
    if (dir_shower.Angle(dir_main)/3.1415926*180. > 30){
      Point test_p = shower->get_closest_point(shower->get_start_vertex().first->get_fit_pt()).second;
      dir_shower = shower->cal_dir_3vector(test_p,30*units::cm);
    
      //     std::cout << shower->get_start_segment()->get_id() << " " << dir_shower.Angle(dir_main)/3.1415926*180. << " " << dir_shower.Mag() << " " << dir_main.Mag() << std::endl;
    }
    if (dir_shower.Mag()<0.001) dir_shower = dir_main;
 
 
    
    for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){ 
      WCPPID::ProtoSegment *seg1 = it->first; 
      if (seg1->get_cluster_id() == main_cluster->get_cluster_id()) continue; 
      if (map_segment_in_shower.find(seg1) != map_segment_in_shower.end()) continue;
      if (seg1->get_cluster_id() == shower->get_start_segment()->get_cluster_id()) continue;
      auto it1 = map_cluster_associated_vertex.find(map_segment_cluster[seg1]);
      
      // find the closest point 
      // judge the detection ...
      
      std::pair<double, WCP::Point> pair_dis_point = seg1->get_closest_point(shower->get_start_vertex().first->get_fit_pt()); 
      TVector3 v1(pair_dis_point.second.x - shower->get_start_vertex().first->get_fit_pt().x, pair_dis_point.second.y - shower->get_start_vertex().first->get_fit_pt().y, pair_dis_point.second.z - shower->get_start_vertex().first->get_fit_pt().z);
      TVector3 v2(pair_dis_point.second.x - point.x,
		  pair_dis_point.second.y - point.y,
		  pair_dis_point.second.z - point.z);
      double angle = dir_shower.Angle(v1);
      double angle1 = dir_shower.Angle(v2);

      double tmp_shower_dis = seg1->get_closest_point(shower->get_start_segment()->get_point_vec().front()).first;
      double close_shower_dis = shower->get_closest_dis(seg1); 
      
      
      //std::cout << seg1->get_cluster_id() << " " << seg1->get_id() << " " << angle1/3.1415926*180. << " " << angle/3.1415926*180. << " " << dis1/units::cm <<" " << pair_dis_point.first/units::cm << " " << tmp_shower_dis/units::cm << " " << close_shower_dis/units::cm << std::endl;
      
      if (angle1/3.1415926*180. > 30) continue;
      
      if ((angle/3.1415926*180.< 25 && (pair_dis_point.first < 80*units::cm || close_shower_dis < 25*units::cm) || angle1/3.1415926*180. < 25 && (tmp_shower_dis < 40*units::cm || close_shower_dis < 25*units::cm))  ||
	  (angle/3.1415926*180 < 12.5 && (pair_dis_point.first < 120*units::cm  || close_shower_dis < 40*units::cm)|| angle1/3.1415926*180. < 12.5 && (tmp_shower_dis < 80*units::cm || close_shower_dis < 40*units::cm))  ){
	if (it1 != map_cluster_associated_vertex.end() && seg1->get_cluster_id() != shower->get_start_segment()->get_cluster_id()){
	  if (it1->second != vertex){ //continue;
	    double dis1 = shower->get_dis(seg1);
	    if (dis1 > 25*units::cm && dis1 > pair_dis_point.first * 0.4){
	      //  std::cout << seg1->get_cluster_id() << " " << seg1->get_id() << " " << angle1/3.1415926*180. << " " << angle1/3.1415926*180. << " " << dis1/units::cm <<" " << pair_dis_point.first/units::cm << std::endl;
	      continue;
	    }
	  }
	}
	//	std::cout << angle1/3.1415926*180. << std::endl;
	//	double dis = pow(pair_dis_point.first*cos(angle),2)/pow(40*units::cm,2) + pow(pair_dis_point.first*sin(angle),2)/pow(5*units::cm,2);
	//	std::cout << seg1->get_cluster_id()*1000 + seg1->get_id() << " " << angle/3.1415926*180. << " " << pair_dis_point.first/units::cm << std::endl;
	//      min_dis = dis;
	shower->add_segment(seg1, map_segment_vertices);
      }
    }





    
    shower->update_particle_type();
    bool tmp_flag = (shower->get_start_vertex().first == main_vertex);
    
    std::cout << "[shower_clustering_with_nv_from_vertices] Separated shower: " <<  shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id() << " " << shower->get_start_segment()->get_particle_type() << " " << shower->get_num_segments() << " " << tmp_flag << " " << vec_pi.at(i).min_dis/units::cm << std::endl;
    
    // udate the map
    update_shower_maps();

    {
      std::map<WCPPID::WCShower*, double> map_shower_length;
      std::map<WCPPID::WCShower*, TVector3> map_shower_dir;
      for (auto it = showers.begin(); it!= showers.end(); it++){
	map_shower_length[*it] = (*it)->get_total_length();
	Point test_p = (*it)->get_closest_point((*it)->get_start_vertex().first->get_fit_pt()).second;
	map_shower_dir[*it] = (*it)->cal_dir_3vector(test_p,30*units::cm);
      }

      bool flag_continue = true;
      while(flag_continue){
	flag_continue = false;
	for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){ 
	  WCPPID::ProtoSegment *seg1 = it->first; 
	  if (seg1->get_cluster_id() == main_cluster->get_cluster_id()) continue; 
	  if (map_segment_in_shower.find(seg1) != map_segment_in_shower.end()) continue;
	  double min_dis = 1e9;
	  WCPPID::WCShower *min_shower = 0;
	  for (size_t i=0; i!= showers.size();i++){ 
	    if (seg1->get_length() > 0.75 * map_shower_length[showers.at(i)] ) continue;
	    Point test_p = seg1->get_closest_point(showers.at(i)->get_start_point()).second;
	    Point test_p1 = showers.at(i)->get_closest_point(showers.at(i)->get_start_vertex().first->get_fit_pt()).second;
	    TVector3 tmp_dir(test_p.x - test_p1.x,
			     test_p.y - test_p1.y,
			     test_p.z - test_p1.z);
	    double angle = tmp_dir.Angle(map_shower_dir[showers.at(i)])/3.1415926*180.;
	    
	    double dis = showers.at(i)->get_closest_dis(seg1);
	    //	    std::cout << angle << " " << dis/units::cm << std::endl;
	    if (dis < min_dis && angle < 45){
	      min_dis = dis;
	      min_shower = showers.at(i);
	    }
	    
	  }
	  if (min_shower !=0 && min_dis < 3.5*units::cm){
	    min_shower->add_segment(seg1, map_segment_vertices);
	    map_shower_length[min_shower] = min_shower->get_total_length();
	    flag_continue = true;
	  }
	}
	update_shower_maps();
      }
      
    }
    
  }

  
  std::cout << "[shower_clustering_with_nv_from_vertices] With separated-cluster shower: " << showers.size() << std::endl;

 

  
}

void WCPPID::NeutrinoID::update_shower_maps(){

  map_vertex_to_shower.clear();
  map_vertex_in_shower.clear();
  map_segment_in_shower.clear();
  used_shower_clusters.clear();
  
  for (auto it = showers.begin(); it!=showers.end(); it++){
    WCPPID::WCShower* shower = *it;
    map_vertex_to_shower[shower->get_start_vertex().first].insert(shower);
    shower->fill_maps(map_vertex_in_shower, map_segment_in_shower);
  }
  for (auto it = map_segment_in_shower.begin(); it!=map_segment_in_shower.end();it++){
    used_shower_clusters.insert(it->first->get_cluster_id());
  }  
}



void WCPPID::NeutrinoID::calculate_shower_kinematics(){
  for (size_t i=0;i!=showers.size();i++){
    WCPPID::WCShower *shower = showers.at(i);
    if (!shower->get_flag_kinematics()){
      if (fabs(shower->get_particle_type())!=13){
	shower->calculate_kinematics();
	double kine_charge = cal_kine_charge(shower);
	shower->set_kine_charge(kine_charge);
	shower->set_flag_kinematics(true);
      }else{
	// long muon ...
	shower->calculate_kinematics_long_muon(segments_in_long_muon);
	double kine_charge = cal_kine_charge(shower);

	//	std::cout << "haha: " << kine_charge << std::endl;
	
	shower->set_kine_charge(kine_charge);
	shower->set_flag_kinematics(true);
      }
    }
  }
}


std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> WCPPID::NeutrinoID::get_start_end_vertices(WCPPID::ProtoSegment* sg){
  WCPPID::ProtoVertex *start_v=0, *end_v=0;
  for (auto it = map_segment_vertices[sg].begin(); it!=map_segment_vertices[sg].end(); it++){
    if ((*it)->get_wcpt().index == sg->get_wcpt_vec().front().index) start_v = *it;
    if ((*it)->get_wcpt().index == sg->get_wcpt_vec().back().index) end_v = *it;
  }
  return std::make_pair(start_v, end_v);
}


// place holder ...
void WCPPID::NeutrinoID::shower_clustering_in_other_clusters(bool flag_save){
  // vertices in main cluster as well as existing showers ...
  std::vector<WCPPID::ProtoVertex* > vertices;
  for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end(); it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() == main_cluster->get_cluster_id() || map_vertex_in_shower.find(vtx) != map_vertex_in_shower.end()){
      vertices.push_back(vtx);
    }
  }
  //  std::cout << vertices.size() << std::endl;

  
  for (auto it = map_cluster_main_vertices.begin(); it!= map_cluster_main_vertices.end(); it++){
    WCPPID::PR3DCluster* cluster = it->first;
    WCPPID::ProtoVertex* vertex = it->second;
    if (used_shower_clusters.find(cluster->get_cluster_id()) != used_shower_clusters.end()) continue;
    if (map_cluster_length[cluster] < 4*units::cm) continue;

    // check against the main cluster first ...
    ToyPointCloud *pcloud = cluster->get_point_cloud();
    double min_dis = 1e9;
    WCPPID::ProtoVertex *min_vertex = 0;
    double main_dis;
    for (size_t i=0;i!=vertices.size();i++){
      double dis = pcloud->get_closest_dis(vertices.at(i)->get_fit_pt());
      if (dis < min_dis){
	min_dis = dis;
	min_vertex = vertices.at(i);
      }
      if (vertices.at(i) == main_vertex)
	main_dis = dis;
    }
    
    // std::cout << min_dis/units::cm << " " << main_dis/units::cm << " " << min_dis/main_dis << std::endl;
    if (min_dis > 0.8 * main_dis){
      min_dis = main_dis;
      min_vertex = main_vertex;
    }
    
    WCPPID::ProtoSegment *sg = 0;
    for (auto it1 = map_vertex_segments[vertex].begin(); it1!= map_vertex_segments[vertex].end(); it1++){
      sg = *it1;
      if(sg->get_flag_shower()){
	break;
      }else{
	sg = 0 ;
      }
    }
    int connection_type = 3;
    
    if (sg != 0){
      WCPPID::WCShower *shower = new WCPPID::WCShower();
      shower->set_start_vertex(min_vertex, connection_type);
      shower->set_start_segment(sg);
      shower->set_start_point(vertex->get_fit_pt());
      double dis1 = sqrt(pow(vertex->get_fit_pt().x - sg->get_point_vec().front().x,2) + pow(vertex->get_fit_pt().y - sg->get_point_vec().front().y,2) + pow(vertex->get_fit_pt().z - sg->get_point_vec().front().z,2));
      double dis2 = sqrt(pow(vertex->get_fit_pt().x - sg->get_point_vec().back().x,2) + pow(vertex->get_fit_pt().y - sg->get_point_vec().back().y,2) + pow(vertex->get_fit_pt().z - sg->get_point_vec().back().z,2));
      
      if (dis1 < dis2){
	sg->set_flag_dir(1);
      }else{
	sg->set_flag_dir(-1);
      }
      std::set<WCPPID::ProtoSegment* > used_segments; 
      shower->complete_structure_with_start_segment(map_vertex_segments, map_segment_vertices, used_segments); 

      
       // cluster with the rest ... 
      TVector3 dir_shower = shower->cal_dir_3vector(vertex->get_fit_pt(), 15*units::cm);
      
      for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){ 
	WCPPID::ProtoSegment *seg1 = it->first; 
	if (seg1->get_cluster_id() == main_cluster->get_cluster_id()) continue; 
	if (map_segment_in_shower.find(seg1) != map_segment_in_shower.end()) continue;
	if (seg1->get_cluster_id() == shower->get_start_segment()->get_cluster_id()) continue;
      
	// find the closest point 
	// judge the detection ...
      
	std::pair<double, WCP::Point> pair_dis_point = seg1->get_closest_point(vertex->get_fit_pt()); 
	TVector3 v1(pair_dis_point.second.x - vertex->get_fit_pt().x, pair_dis_point.second.y - vertex->get_fit_pt().y, pair_dis_point.second.z - vertex->get_fit_pt().z);
	double angle = dir_shower.Angle(v1);
            
	if ((angle/3.1415926*180.< 25 ) && pair_dis_point.first < 80*units::cm ||
	    (angle/3.1415926*180 < 12.5 ) && pair_dis_point.first < 120*units::cm ){
	  shower->add_segment(seg1, map_segment_vertices);
	}
      }

      shower->update_particle_type();

      std::vector<WCPPID::WCShower*> showers_to_be_removed;
      // check with other showers ...
      for (size_t i=0;i!=showers.size();i++){
	TVector3 dir_shower1 = showers.at(i)->cal_dir_3vector(showers.at(i)->get_start_point(), 15*units::cm);
	TVector3 dir2(showers.at(i)->get_start_point().x - vertex->get_fit_pt().x,
		      showers.at(i)->get_start_point().y - vertex->get_fit_pt().y,
		      showers.at(i)->get_start_point().z - vertex->get_fit_pt().z);
	double angle = dir_shower.Angle(dir_shower1)/3.1415926*180.;
	double angle1 = dir_shower.Angle(dir2)/3.1415926*180.;
	if ((angle < 25 && angle1 < 15) && dir2.Mag() < 80*units::cm ||
	    (angle< 12.5 && angle1 < 7.5) && dir2.Mag() < 120*units::cm ){
	  shower->add_shower(showers.at(i));
	  showers_to_be_removed.push_back(showers.at(i));
	}
	//	std::cout << i << " " << dir_shower.Angle(dir_shower1)/3.1415926*180. << " " << dir_shower.Angle(dir2)/3.1415926*180. << " " << dir2.Mag()/units::cm << std::endl;
      }
      for (auto it1 = showers_to_be_removed.begin(); it1 != showers_to_be_removed.end(); it1++){
	auto it2 = find(showers.begin(), showers.end(), *it1);
	showers.erase(it2);
	delete *it1;
      }

      shower->update_particle_type();
      shower->calculate_kinematics();

      showers.push_back(shower);
    }
  }  
  update_shower_maps();  
  //std::cout << showers.size() << std::endl;
    

  
  for (auto it = other_clusters.begin(); it != other_clusters.end(); it++){
    WCPPID::PR3DCluster *cluster = *it;
    if (used_shower_clusters.find(cluster->get_cluster_id()) != used_shower_clusters.end()) continue;

    // check against the main cluster first ...
    ToyPointCloud *pcloud = cluster->get_point_cloud();
    double min_dis = 1e9;
    WCPPID::ProtoVertex *min_vertex = 0;
    double main_dis;
    for (size_t i=0;i!=vertices.size();i++){
      double dis = pcloud->get_closest_dis(vertices.at(i)->get_fit_pt());
      if (dis < min_dis){
	min_dis = dis;
	min_vertex = vertices.at(i);
      }
      if (vertices.at(i) == main_vertex)
	main_dis = dis;
    }
    
    // std::cout << min_dis/units::cm << " " << main_dis/units::cm << " " << min_dis/main_dis << std::endl;
    if (min_dis > 0.8 * main_dis){
      min_dis = main_dis;
      min_vertex = main_vertex;
    }
    
    WCPPID::ProtoSegment *sg = 0;
    for (auto it1 = map_segment_vertices.begin(); it1 != map_segment_vertices.end(); it1++){
      if (it1->first->get_cluster_id() != cluster->get_cluster_id()) continue;
      sg = it1->first;
      break;
    }

    int connection_type = 3;
    if (min_dis > 80*units::cm){
      connection_type = 4;
    }

    if (!flag_save)
      connection_type = 4;
    
    
    if (sg != 0){
      WCPPID::WCShower *shower = new WCPPID::WCShower();
      shower->set_start_vertex(min_vertex, connection_type);
      shower->set_start_segment(sg);
      
      if (sg->get_flag_dir()==0){
	auto tmp_vertices = get_start_end_vertices(sg);
	if (map_vertex_segments[tmp_vertices.first].size()==1 && map_vertex_segments[tmp_vertices.second].size()>1){
	  sg->set_flag_dir(1);
	}else if (map_vertex_segments[tmp_vertices.first].size()>1 && map_vertex_segments[tmp_vertices.second].size()==1){
	  sg->set_flag_dir(-1);
	}else{
	  // examine vertices 
	  double dis1 = sqrt(pow(main_vertex->get_fit_pt().x - sg->get_point_vec().front().x,2) + pow(main_vertex->get_fit_pt().y - sg->get_point_vec().front().y,2) + pow(main_vertex->get_fit_pt().z - sg->get_point_vec().front().z,2));
	  double dis2 = sqrt(pow(main_vertex->get_fit_pt().x - sg->get_point_vec().back().x,2) + pow(main_vertex->get_fit_pt().y - sg->get_point_vec().back().y,2) + pow(main_vertex->get_fit_pt().z - sg->get_point_vec().back().z,2));
	    
	  if (dis1 < dis2){
	    sg->set_flag_dir(1);
	  }else{
	    sg->set_flag_dir(-1);
	  }
	}
      }

      //      std::cout << sg->get_cluster_id() << " " << sg->get_particle_type() << sg->get_length()/units::cm << std::endl;
      
      //      std::cout << sg->get_particle_type() << " " << std::endl;
      if (sg->get_particle_type()==0 || fabs(sg->get_particle_type())==13
	  && sg->get_length() < 40*units::cm && sg->is_dir_weak()
	  ){
	sg->set_particle_type(11);
	TPCParams& mp = Singleton<TPCParams>::Instance();
	sg->set_particle_mass(mp.get_mass_electron());
	sg->cal_4mom();
      }

      
      std::set<WCPPID::ProtoSegment* > used_segments; 
      shower->complete_structure_with_start_segment(map_vertex_segments, map_segment_vertices, used_segments); 
      showers.push_back(shower);
    }
    
  }
  update_shower_maps();  
}


void WCPPID::NeutrinoID::shower_clustering_with_nv_in_main_cluster(){
  // search from main vertex ...
  // search trees, if find an electron, then the rest are all added to it ...
  

  // main_vertex figure out the daughters and mother ... 
  std::set<WCPPID::ProtoSegment* > used_segments; 

  std::vector<std::pair<WCPPID::ProtoSegment*, WCPPID::ProtoVertex*> > segments_to_be_examined; // current_segment, daughter vertex
  for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end(); it++){ 
    // parent are all zero now ...
    WCPPID::ProtoVertex *other_vertex = find_other_vertex(*it, main_vertex);
    // std::cout << *it << " " << other_vertex << std::endl;
    segments_to_be_examined.push_back(std::make_pair(*it, other_vertex)); 
  } 
  
  while(segments_to_be_examined.size()>0){ 
    std::vector<std::pair<WCPPID::ProtoSegment*, WCPPID::ProtoVertex*> > temp_segments; 
    for (auto it = segments_to_be_examined.begin(); it!= segments_to_be_examined.end(); it++){ 
      WCPPID::ProtoSegment *curr_sg = it->first;
      WCPPID::ProtoVertex *daughter_vtx = it->second;
      if (used_segments.find(curr_sg) != used_segments.end()) continue;
      used_segments.insert(curr_sg);

      //      std::cout << curr_sg->get_id() << " " << curr_sg->get_particle_type() << " " << segments_to_be_examined.size() << std::endl;
      
      if (curr_sg->get_flag_shower() || segments_in_long_muon.find(curr_sg) != segments_in_long_muon.end()){
	WCPPID::ProtoVertex *parent_vtx = find_other_vertex(curr_sg, daughter_vtx);
	WCPPID::WCShower *shower = new WCPPID::WCShower();
	shower->set_start_vertex(parent_vtx, 1);
	shower->set_start_segment(curr_sg);
	bool tmp_flag = (parent_vtx == main_vertex);
	showers.push_back(shower);

	if (fabs(curr_sg->get_particle_type()==13)){
	  shower->set_particle_type(curr_sg->get_particle_type());
	  std::cout << "[shower_clustering_with_nv_in_main_cluster] Main-cluster long muon " << showers.size() << " : " << curr_sg->get_cluster_id()*1000 + curr_sg->get_id() << " " << curr_sg->get_particle_type() << " " << tmp_flag << " " << curr_sg->get_flag_shower_topology() << std::endl;
	}else{
	  std::cout << "[shower_clustering_with_nv_in_main_cluster] Main-cluster shower " << showers.size() << " : " << curr_sg->get_cluster_id()*1000 + curr_sg->get_id() << " " << curr_sg->get_particle_type() << " " << tmp_flag << " " << curr_sg->get_flag_shower_topology() << " " << std::endl;
	}
      }else{
	// keep searching its daughter
	for (auto it1 = map_vertex_segments[daughter_vtx].begin(); it1 != map_vertex_segments[daughter_vtx].end(); it1++){
	  WCPPID::ProtoSegment *next_sg = *it1;
	  if (used_segments.find(next_sg)!=used_segments.end()) continue;
	  WCPPID::ProtoVertex *other_vertex = find_other_vertex(next_sg, daughter_vtx);
	  temp_segments.push_back(std::make_pair(next_sg, other_vertex));
	}
      }
    } 
    segments_to_be_examined = temp_segments; 
  }

  // complete the shower construction
  for (size_t i=0;i!=showers.size();i++){
    showers.at(i)->complete_structure_with_start_segment(map_vertex_segments, map_segment_vertices, used_segments); 
  }

  // check if a certain long muon shower should be converted to an EM shower ...
  std::set<WCPPID::ProtoSegment* > tmp_segments;
  std::set<WCPPID::ProtoVertex* > tmp_vertices;
  for (size_t i=0;i!=showers.size();i++){
    if (showers.at(i)->get_start_segment()->get_particle_type()!=13) continue;
    Map_Proto_Segment_Vertices& map_seg_vtxs = showers.at(i)->get_map_seg_vtxs();
    double n_muons = 0;
    double length_muons = 0;
    double n_others = 0;
    double length_others = 0;
    double max_muon_length = 0;
    WCPPID::ProtoSegment *max_sg = 0;
    for (auto it1 = map_seg_vtxs.begin(); it1 != map_seg_vtxs.end(); it1++){
      WCPPID::ProtoSegment *sg1 = it1->first;
      double length = sg1->get_length();
      if (segments_in_long_muon.find(sg1) != segments_in_long_muon.end()){
	n_muons ++;
	length_muons += length;
	if (length > max_muon_length) {
	  max_muon_length = length;
	  max_sg = sg1;
	}
      }else{
	n_others ++;
	length_others += length;
      }
    }
    
    if (n_others >= 2 * n_muons && length_others > 0.33 * length_muons && n_muons >0 && max_muon_length < 60*units::cm){
      std::cout << "[shower_clustering_with_nv_in_main_cluster] Long muon converted to EM shower " << std::endl;
      for (auto it1 = map_seg_vtxs.begin(); it1 != map_seg_vtxs.end(); it1++){
	WCPPID::ProtoSegment *sg1 = it1->first;
	//	std::cout << sg1->get_length()/units::cm << " " << sg1->get_flag_shower_topology() << " " << sg1->get_flag_shower_trajectory() << " " << std::endl;
	if (sg1 == max_sg) max_sg->set_flag_avoid_muon_check(true);
	sg1->set_particle_type(11);
	TPCParams& mp = Singleton<TPCParams>::Instance();
	sg1->set_particle_mass(mp.get_mass_electron());
	tmp_segments.insert(sg1);
	auto pair_vertices = find_vertices(sg1);
	tmp_vertices.insert(pair_vertices.first);
	tmp_vertices.insert(pair_vertices.second);
      }
    }
  }
  // need to update the long muon containers ...
  // clean up long muons ...
 
 
  for (auto it = tmp_segments.begin(); it!=tmp_segments.end();it++){
    segments_in_long_muon.erase(*it);
  }
  for (auto it = tmp_vertices.begin(); it!=tmp_vertices.end(); it++){
    vertices_in_long_muon.erase(*it);
  }
  
  

  update_shower_maps();  
  //std::cout << showers.size() << " " << used_segments.size() << std::endl;
}



void WCPPID::NeutrinoID::shower_clustering_with_nv_from_main_cluster(){
  std::map<WCPPID::WCShower*, TVector3> map_shower_dir;
  std::map<WCPPID::WCShower*, double> map_shower_angle_offset;
  TVector3 drift_dir(1,0,0);
  WCPPID::ProtoSegment* max_length_segment = 0;
  {
    double max_length = 0;
    for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
      WCPPID::ProtoSegment *seg = it->first;
      if (seg->get_cluster_id() != main_cluster->get_cluster_id()) continue;
      double length = seg->get_length();
      if (length > max_length && length > 6*units::cm){
	max_length = length;
	max_length_segment = seg;
      }
    }
  }
  
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *seg = it->first;
    if (seg->get_cluster_id() != main_cluster->get_cluster_id()) continue;
    if (map_segment_in_shower.find(seg) == map_segment_in_shower.end()) continue;
    WCPPID::WCShower *shower = map_segment_in_shower[seg];

    // long muon ...
    if (fabs(shower->get_particle_type())==13) continue;
    //    std::cout << shower->get_particle_type() << std::endl;

    double total_length = shower->get_total_length();
    if (seg == shower->get_start_segment()){   
      //      std::cout << seg->get_flag_shower_topology() << " " << shower->get_num_segments() << " " <<  seg->get_medium_dQ_dx(0,100)/(43e3/units::cm) << " " << seg->get_length()/units::cm << std::endl;
      if ( seg->get_flag_shower_topology() || shower->get_num_segments()>2 || seg->get_medium_dQ_dx(0,100) > 43e3/units::cm * 1.5 && seg->get_length() > 0){
	if (seg->get_length() > 10*units::cm){
	  TVector3 dir_shower = seg->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 15*units::cm);
	  map_shower_dir[shower] = dir_shower;
	}else{
	  TVector3 dir_shower = shower->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 15*units::cm);
	  map_shower_dir[shower] = dir_shower;
	}
      }else if (shower->get_num_segments()<=2){

	if (total_length > 30*units::cm){
	  if (seg->get_length()>10*units::cm){
	    TVector3 dir_shower = seg->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 15*units::cm);
	    map_shower_dir[shower] = dir_shower;
	  }else{
	    TVector3 dir_shower = shower->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 15*units::cm);
	    map_shower_dir[shower] = dir_shower;
	  }
	}
      }

      // very large shower ...
      if (total_length > 100*units::cm){
	TVector3 dir_shower = shower->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 60*units::cm);
	map_shower_dir[shower] = dir_shower;
      }
      
      if (seg == max_length_segment && map_shower_dir.find(shower) == map_shower_dir.end()){
	TVector3 dir_shower = shower->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 15*units::cm);
	map_shower_dir[shower] = dir_shower;
      }else if (map_shower_dir.find(shower) == map_shower_dir.end() && map_segment_vertices[seg].find(main_vertex) != map_segment_vertices[seg].end()){
	if (seg->get_length() > 5*units::cm){
	  TVector3 dir_shower = shower->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 15*units::cm);
	  map_shower_dir[shower] = dir_shower;
	}
      }
      
      if (map_shower_dir.find(shower) != map_shower_dir.end()){
	map_shower_angle_offset[shower] = 0;
	if (fabs(map_shower_dir[shower].Angle(drift_dir)/3.1415926*180.-90)<5){
	  map_shower_dir[shower] = shower->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 50*units::cm);
	  map_shower_angle_offset[shower] = 5;
	}
      }

      //std::cout << shower->get_start_vertex().first->get_fit_pt() << " " << map_shower_dir[shower].Mag() << " " << map_shower_dir[shower].Angle(drift_dir)/3.1415926*180. << " " << shower->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 15*units::cm).Mag() << " " << shower->get_start_segment()->cal_dir_3vector(shower->get_start_vertex().first->get_fit_pt(), 15*units::cm).Mag() << std::endl;
      
    }
  }

  //std::cout << showers.size() << " " << map_shower_dir.size() << std::endl;

  if (map_shower_dir.size() == 0){
    std::map<WCPPID::WCShower*, double> map_shower_length;
    for (auto it = showers.begin(); it!= showers.end(); it++){
      map_shower_length[*it] = (*it)->get_total_length();
    }


    bool flag_continue = true;
    while(flag_continue){
      flag_continue = false;
      for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){ 
	WCPPID::ProtoSegment *seg1 = it->first; 
	if (seg1->get_cluster_id() == main_cluster->get_cluster_id()) continue; 
	if (map_segment_in_shower.find(seg1) != map_segment_in_shower.end()) continue;
	double min_dis = 1e9;
	WCPPID::WCShower *min_shower = 0;
	for (size_t i=0; i!= showers.size();i++){
	  if (showers.at(i)->get_start_segment()->get_particle_type()==13) continue;
	  if (seg1->get_length() > 0.75 * map_shower_length[showers.at(i)] ) continue;
	  double dis = showers.at(i)->get_closest_dis(seg1);
	  if (dis < min_dis){
	    min_dis = dis;
	    min_shower = showers.at(i);
	  }
	}
	if (min_shower !=0 && min_dis < 3.5*units::cm){
	  //	  std::cout << seg1->get_cluster_id() << " " << seg1->get_id() << " " << min_shower->get_start_segment()->get_cluster_id() << " " << min_shower->get_start_segment()->get_id() << " " << min_shower->get_closest_dis(seg1)/units::cm << std::endl;
	  min_shower->add_segment(seg1, map_segment_vertices);
	  map_shower_length[min_shower] = min_shower->get_total_length();
	  flag_continue = true;
	}
      }
      update_shower_maps();
    }
    
    
  }
  
  if (map_shower_dir.size() == 0) return;

  // examine other segments first ... 
  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){ 
    WCPPID::ProtoSegment *seg1 = it->first;
    if (seg1->get_cluster_id() == main_cluster->get_cluster_id()) continue; 
    if (map_segment_in_shower.find(seg1) != map_segment_in_shower.end()) continue;
    
    // find the closest point 
    // judge the detection ...
    double min_dis = 1e9;
    WCPPID::WCShower* min_shower = 0;
    for (auto it1 = map_shower_dir.begin(); it1 != map_shower_dir.end(); it1++){
      WCPPID::WCShower *shower = it1->first;
      //  std::cout << shower->get_start_segment()->get_id() << std::endl;
      std::pair<double, WCP::Point> pair_dis_point = seg1->get_closest_point(shower->get_start_vertex().first->get_fit_pt()); 
      TVector3 v1(pair_dis_point.second.x - shower->get_start_vertex().first->get_fit_pt().x, pair_dis_point.second.y - shower->get_start_vertex().first->get_fit_pt().y, pair_dis_point.second.z - shower->get_start_vertex().first->get_fit_pt().z);
      double angle = it1->second.Angle(v1);
      double angle_offset = 0;
      if (map_shower_angle_offset.find(shower) != map_shower_angle_offset.end())
	angle_offset = map_shower_angle_offset[shower];
      if (angle/3.1415926*180.<25. + angle_offset  && pair_dis_point.first < 80*units::cm ||
	  angle/3.1415926*180 < 12.5 + angle_offset*8/5 && pair_dis_point.first < 130*units::cm ||
	  angle/3.1415926*180 < 5 + angle_offset*2 && pair_dis_point.first < 200*units::cm ){
	double dis = pow(pair_dis_point.first*cos(angle),2)/pow(40*units::cm,2) + pow(pair_dis_point.first*sin(angle),2)/pow(5*units::cm,2);
	if (dis < min_dis){
	  min_dis = dis;
	  min_shower = shower;
	}
      }else{
	//	std::cout << seg1->get_cluster_id()*1000 + seg1->get_id() << " " << pair_dis_point.first/units::cm << " " << angle/3.1415926*180. << " " << seg1->get_length()/units::cm << " " << v1.X() << " " << v1.Y() << " " << v1.Z() << " " << it1->second.X() << " " << it1->second.Y() << " " << it1->second.Z() << std::endl;
      }
    }
    
    //    if (seg1->get_cluster_id()==51 && seg1->get_id()==74) std::cout << min_shower << " " << std::endl;
    
    if (min_shower!=0){
      min_shower->add_segment(seg1, map_segment_vertices);
    }
  }
  
  update_shower_maps();

}
