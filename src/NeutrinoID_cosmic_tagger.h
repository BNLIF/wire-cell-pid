bool WCPPID::NeutrinoID::cosmic_tagger(){
  TVector3 dir_beam(0,0,1);
  TVector3 dir_drift(1,0,0);
  TVector3 dir_vertical(0,1,0);

  double tmp_dis = sqrt(pow(main_vertex->get_fit_pt().x  - main_vertex->get_wcpt().x,2) + pow(main_vertex->get_fit_pt().y  - main_vertex->get_wcpt().y,2) + pow(main_vertex->get_fit_pt().z  - main_vertex->get_wcpt().z,2));
  //  std::cout << tmp_dis/units::cm << std::endl;
  // if main vertex is outside the fiducial volume
  Point test_p = main_vertex->get_fit_pt();
  if (tmp_dis > 5*units::cm){
    test_p.x = main_vertex->get_wcpt().x;
    test_p.y = main_vertex->get_wcpt().y;
    test_p.z = main_vertex->get_wcpt().z;
  }
  
  if (!fid->inside_fiducial_volume(test_p, offset_x)){
    neutrino_type |= 1UL << 1;
    // std::cout << "C: " << test_p << " " << true << std::endl;
    return true;
  }


  // single muon, nothing there ...
  {
    double max_length = 0;
    WCPPID::ProtoSegment *muon = 0;
    Int_t valid_tracks = 0;
    for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end() ;it++){
      WCPPID::ProtoSegment *sg = *it;
      double length = sg->get_length();
      double medium_dQ_dx = sg->get_medium_dQ_dx();
      double dQ_dx_cut = 0.8866+0.9533 *pow(18*units::cm/length, 0.4234);
      
      if (sg->get_particle_type() == 13 || (!sg->get_flag_shower()) && medium_dQ_dx < dQ_dx_cut * 1.05 * 43e3/units::cm){
	if (length > max_length){
	  max_length = length;
	  muon = sg;
	}
      }
      //  std::cout << "Xin_3: " << length/units::cm << " " << map_vertex_segments[main_vertex].size() << " " << sg->get_particle_type() << " " << sg->is_dir_weak() << std::endl;
      if (length > 2.5*units::cm || length > 1.2*units::cm && sg->get_particle_type()==2212 || (!sg->is_dir_weak())) {
	//	std::cout << "Xin_3: " << sg->get_medium_dQ_dx()/(43e3/units::cm) << " " << length/units::cm << " " << sg->get_particle_type() << " " << sg->is_dir_weak() << std::endl;
	if (length < 15*units::cm && sg->get_medium_dQ_dx()/(43e3/units::cm)  < 0.75 && sg->is_dir_weak()) continue;
	valid_tracks ++;
      }
    }

    int connected_showers = 0;
    // high energy showers ...
    for (auto it = map_vertex_to_shower[main_vertex].begin(); it != map_vertex_to_shower[main_vertex].end(); it++){
    //    for (auto it = showers.begin(); it != showers.end(); it++){
      WCPPID::WCShower *shower = *it;
      double Eshower = 0;
      if (shower->get_kine_best() != 0){ 
	Eshower = shower->get_kine_best();
      }else{
	Eshower = shower->get_kine_charge();
      }
      if (shower->get_start_vertex().second >2) continue;
      if (Eshower > 150*units::MeV && (!bad_reconstruction(shower)) )
	valid_tracks ++;
      if (shower->get_start_vertex().second == 1 && Eshower > 70*units::MeV && (!bad_reconstruction(shower)) &&
	  shower->get_start_segment()->get_particle_type()!=13)
	connected_showers ++;
      //      std::cout << Eshower/units::MeV << " " << bad_reconstruction(shower) << " " << shower->get_start_vertex().second << " " << shower->get_start_segment()->get_cluster_id() << std::endl;
    }
    //    std::cout << " " << map_vertex_to_shower[main_vertex].size() << std::endl;
    
    if (valid_tracks == 1 && muon != 0){
      WCPPID::ProtoSegment *sg = muon;
      WCPPID::ProtoVertex *other_vtx = find_other_vertex(sg, main_vertex);
      Point test_p1 = other_vtx->get_fit_pt();
      TVector3 dir = sg->cal_dir_3vector(main_vertex->get_fit_pt(), 15*units::cm);
      bool flag_inside = fid->inside_fiducial_volume(test_p1, offset_x);

      double dQ_dx_front = 0;
      double dQ_dx_end = 0;
      if (sg->get_wcpt_vec().front().index==main_vertex->get_wcpt().index){
	dQ_dx_front = sg->get_medium_dQ_dx(0,10);
	dQ_dx_end =sg->get_medium_dQ_dx(int(sg->get_point_vec().size())-10,sg->get_point_vec().size());
      }else{
	dQ_dx_end = sg->get_medium_dQ_dx(0,10);
	dQ_dx_front = sg->get_medium_dQ_dx(int(sg->get_point_vec().size())-10,sg->get_point_vec().size());
      }

      
      //      std::cout << "Xin_1: " << sg->get_length()/units::cm << " " << sg->get_particle_type() << " " << fid->inside_fiducial_volume(test_p1, offset_x) << " " << sg->is_dir_weak() << " " << dir.Angle(dir_beam)/3.1415926*180. << " " << dir.Angle(dir_vertical)/3.1415926*180. << " " << dir.Angle(dir_drift)/3.1415926*180. << " " << dQ_dx_front/(43e3/units::cm) << " " << dQ_dx_end/(43e3/units::cm) << std::endl;
      
      if (sg->get_particle_type() == 13 && ( (!flag_inside) && dir.Angle(dir_beam)/3.1415926*180. > 40
					     || (flag_inside && 
						 (sg->is_dir_weak() && (!(dQ_dx_end > 1.4 *43e3/units::cm && dQ_dx_end > 1.2 * dQ_dx_front)) || dir.Angle(dir_beam)/3.1415926*180. > 60))
					     )){
	neutrino_type |= 1UL << 1;
	// std::cout << "C: " << test_p << " " << true << std::endl;
	return true;
      }/* else if (sg->get_particle_type()==13 && flag_inside && (!sg->is_dir_weak()) && */
      /* 		(dir.Angle(dir_drift)/3.1415926*180. > 165     // prolonged towards anode ... */
      /* 		 || dir.Angle(dir_vertical)/3.1415926*180. > 150) // vertical down ... */
      /* 		){ */
      /* 	neutrino_type |= 1UL << 1; */
      /* 	return true; */
      /* } */
           
    }

    if (muon != 0){
      WCPPID::ProtoSegment *sg = muon;
      WCPPID::ProtoVertex *other_vtx = find_other_vertex(sg, main_vertex);
      Point test_p1 = other_vtx->get_fit_pt();
      TVector3 dir = sg->cal_dir_3vector(main_vertex->get_fit_pt(), 15*units::cm);
      bool flag_inside = fid->inside_fiducial_volume(test_p1, offset_x);

      std::cout << "Xin: " << sg->get_length()/units::cm << " " << flag_inside << " " << dir.Angle(dir_beam)/3.1415926*180. << " " << connected_showers << std::endl;
      if ( (!flag_inside) && dir.Angle(dir_beam)/3.1415926*180. > 100 && connected_showers ==0){
	neutrino_type |= 1UL << 1; 
     	return true; 
      }
    }
  }
  
  // stopped muon with a  Michel electron ...
  {
    WCPPID::WCShower *michel_ele = 0;
    WCPPID::ProtoSegment *muon = 0;
    WCPPID::ProtoSegment *muon_2nd = 0;
    Int_t valid_tracks = 0;
    
    for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end(); it++){
      WCPPID::ProtoSegment *sg = *it;
      double length = sg->get_length();
      
      
      if (sg->get_flag_shower()){
	for (auto it1 = showers.begin(); it1!=showers.end(); it1++){
	  WCPPID::WCShower *shower = *it1;
	  // calculate energy 
	  if (shower->get_start_segment() == sg){
	    michel_ele = shower;
	  }
	}
      }else{
	if (sg->get_particle_type()==13){
	  if (muon == 0){
	    muon = sg;
	  }else{
	    if (sg->get_length() > muon->get_length()){
	      muon_2nd = muon;
	      muon = sg;
	    }
	  }
	}
      }
    }

    for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end(); it++){
      WCPPID::ProtoSegment *sg = *it;
      double length = sg->get_length();
      if (sg == muon) continue;
      if (michel_ele != 0)
      	if (sg == michel_ele->get_start_segment()) continue;
      
      if (length > 2.5*units::cm || length > 1.2*units::cm && sg->get_particle_type()==2212 || (!sg->is_dir_weak())) {
      	if (length < 15*units::cm && sg->get_medium_dQ_dx()/(43e3/units::cm)  < 0.75 && sg->is_dir_weak()) continue;
	if (sg->get_flag_shower()) continue;
	
      	valid_tracks ++;
      }
    }
    
    

    for (auto it = map_vertex_to_shower[main_vertex].begin(); it != map_vertex_to_shower[main_vertex].end(); it++){
      WCPPID::WCShower *shower = *it;
      if (shower == michel_ele) continue;
      double Eshower = 0;
      if (shower->get_kine_best() != 0){ 
	Eshower = shower->get_kine_best();
      }else{
	Eshower = shower->get_kine_charge();
      }
      if (shower->get_start_vertex().second >2) continue;
      if (Eshower > 150*units::MeV && (!bad_reconstruction(shower))) valid_tracks ++;
      //      std::cout << Eshower/units::MeV << " " << bad_reconstruction(shower) << " " << shower->get_start_vertex().second << " " << shower->get_start_segment()->get_cluster_id() << std::endl;
    }

    std::cout << muon << " " << michel_ele << " " << valid_tracks << std::endl;

    if (muon != 0 && michel_ele != 0 && valid_tracks == 0){
      double Eshower = 0;
      if (michel_ele->get_kine_best() != 0){ 
	Eshower = michel_ele->get_kine_best();
      }else{
	Eshower = michel_ele->get_kine_charge();
      }
      
      WCPPID::ProtoVertex *other_vtx = find_other_vertex(muon, main_vertex);
      Point test_p1 = other_vtx->get_fit_pt();
      TVector3 dir = muon->cal_dir_3vector(main_vertex->get_fit_pt(), 15*units::cm);
      bool flag_inside = fid->inside_fiducial_volume(test_p1, offset_x);

      double dQ_dx_front = 0;
      double dQ_dx_end = 0;
      if (muon->get_wcpt_vec().front().index==main_vertex->get_wcpt().index){
	dQ_dx_front = muon->get_medium_dQ_dx(0,10);
	dQ_dx_end = muon->get_medium_dQ_dx(int(muon->get_point_vec().size())-10,muon->get_point_vec().size());
      }else{
	dQ_dx_end = muon->get_medium_dQ_dx(0,10);
	dQ_dx_front = muon->get_medium_dQ_dx(int(muon->get_point_vec().size())-10,muon->get_point_vec().size());
      }
      
      //      std::cout << "Xin_2: " << Eshower/units::MeV << " " << muon->get_length()/units::cm << " " << muon->get_particle_type() << " " << fid->inside_fiducial_volume(test_p1, offset_x) << " " << muon->is_dir_weak() << " " << dir.Angle(dir_beam)/3.1415926*180. << " " << dir.Angle(dir_vertical)/3.1415926*180. << " " << dir.Angle(dir_drift)/3.1415926*180. << " " << dQ_dx_front/(43e3/units::cm) << " " << dQ_dx_end/(43e3/units::cm) << std::endl;
      
      if (Eshower < 70*units::MeV){
	if (muon->get_particle_type() == 13 && ( (!flag_inside) && dir.Angle(dir_beam)/3.1415926*180. > 40
						 || (flag_inside  && (muon->is_dir_weak() && (!(dQ_dx_end > 1.4 *43e3/units::cm && dQ_dx_end > 1.2 * dQ_dx_front)) || dir.Angle(dir_beam)/3.1415926*180. > 60)))){
	  neutrino_type |= 1UL << 1;
	  return true;
	}/* else if (muon->get_particle_type()==13 && flag_inside && (!muon->is_dir_weak()) && */
	/* 	  (dir.Angle(dir_drift)/3.1415926*180. > 165     // prolonged towards anode ... */
	/* 	   || dir.Angle(dir_vertical)/3.1415926*180. > 150) // vertical down ... */
	/* 	  ){ */
	/*   neutrino_type |= 1UL << 1; */
	/*   return true; */
	/* } */
      }
           
    } // two things ...
  }
  

  

  
  {
    bool flag_main_cluster = true;
    
    std::map<int, PointVector> map_cluster_id_points;
    std::map<int, Point> map_cluster_id_high_point;
    std::map<int, int> map_cluster_id_shower_points;
    std::map<int, double> map_cluster_id_length;
    std::set<int> cluster_id_set;
    
    // calculate PCA for large clusters ...
    int num_small_pieces = 0;
    double acc_small_length = 0;
    for (auto it = map_cluster_length.begin(); it != map_cluster_length.end(); it++){
      map_cluster_id_length[it->first->get_cluster_id()] = it->second;
      if (it->second > 3*units::cm){
	cluster_id_set.insert(it->first->get_cluster_id());
	map_cluster_id_shower_points[it->first->get_cluster_id()] = 0;
      }else{
	if (it->first->get_point_cloud()->get_cloud().pts.front().y > 50*units::cm){
	  num_small_pieces ++;
	  acc_small_length += it->second;
	}
      }
    }

    for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
      WCPPID::ProtoSegment *sg = it->first;
      if (cluster_id_set.find(sg->get_cluster_id()) == cluster_id_set.end()) continue;
      PointVector& pts = sg->get_point_vec();
      
      if (pts.size()<=2) continue;
      if (map_cluster_id_high_point.find(sg->get_cluster_id()) == map_cluster_id_high_point.end()) map_cluster_id_high_point[sg->get_cluster_id()] = pts.front();
      for (size_t i=1; i+1<pts.size();i++){
	map_cluster_id_points[sg->get_cluster_id()].push_back(pts.at(i));
	if (pts.at(i).y > map_cluster_id_high_point[sg->get_cluster_id()].y) map_cluster_id_high_point[sg->get_cluster_id()] = pts.at(i);
	  
      }
      if (sg->get_flag_shower())
	map_cluster_id_shower_points[sg->get_cluster_id()] += pts.size();
    }
    for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end(); it++){
      WCPPID::ProtoVertex *vtx = it->first;
      if (cluster_id_set.find(vtx->get_cluster_id()) == cluster_id_set.end()) continue;
      if (vtx->get_fit_pt().y > map_cluster_id_high_point[vtx->get_cluster_id()].y) map_cluster_id_high_point[vtx->get_cluster_id()] = vtx->get_fit_pt();
      map_cluster_id_points[vtx->get_cluster_id()].push_back(vtx->get_fit_pt());
    }

    int num_cosmic = 0;
    double acc_cosmic_length = 0;
    double acc_total_length = 0;
    double highest_y = -100*units::cm;
    double max_length = 0;
    int num_showers = 0;
    for (auto it = map_cluster_id_points.begin(); it != map_cluster_id_points.end(); it++){
      double angle_cosmic, angle_beam;
       
      if (it->first == main_vertex->get_cluster_id() &&
	  (map_cluster_id_shower_points[it->first] * 1./it->second.size() > 0.7 && map_cluster_id_length[it->first] < 45 *units::cm
	   ||   map_cluster_id_shower_points[it->first] * 1./it->second.size() <0.7 && map_cluster_id_length[it->first] > 40 *units::cm )
	  || map_cluster_id_length[it->first] > 60 *units::cm
	  ){
	Point vector(0,0,0);
	for (size_t i=0; i!= it->second.size();i++){
	  vector.x += it->second.at(i).x - main_vertex->get_fit_pt().x;
	  vector.y += it->second.at(i).y - main_vertex->get_fit_pt().y;
	  vector.z += it->second.at(i).z - main_vertex->get_fit_pt().z;
	}
	TVector3 dir(vector.x, vector.y, vector.z);
	angle_beam = dir.Angle(dir_beam)/3.1415926*180.;
	if (angle_beam > 90) angle_beam = 180 - angle_beam;

	angle_cosmic = 180 - dir.Angle(dir_vertical)/3.1415926*180.;
      }else{
	main_cluster->Calc_PCA(it->second);
	auto vector = main_cluster->get_PCA_axis(0);
	TVector3 dir(vector.x, vector.y, vector.z);
	angle_cosmic = dir.Angle(dir_vertical)/3.1415926*180.;
	angle_beam = dir.Angle(dir_beam)/3.1415926*180.;
	
	if (angle_cosmic > 90) angle_cosmic = 180 - angle_cosmic;
	if (angle_beam > 90) angle_beam = 180 - angle_beam;
      }
      
      // main track is not cosmic like ...
      if ( it->first == main_vertex->get_cluster_id() && map_cluster_id_shower_points[it->first] * 1./it->second.size() <0.3 && map_cluster_id_length[it->first] > 20*units::cm && angle_cosmic > 40){
	flag_main_cluster = false;
      }else if (it->first == main_vertex->get_cluster_id() && map_cluster_id_length[it->first] > 80*units::cm && angle_cosmic > 25){
	flag_main_cluster = false;
      }
      
      //      std::cout << angle_cosmic << " " << angle_beam << " " << map_cluster_id_length[it->first]/units::cm << " " << map_cluster_id_shower_points[it->first] * 1./it->second.size() <<  " " << map_cluster_id_high_point[it->first] << std::endl;
      if (map_cluster_id_high_point[it->first].y > highest_y) highest_y = map_cluster_id_high_point[it->first].y;
      
      if (map_cluster_id_shower_points[it->first] * 1./it->second.size() > 0.7){
	if (angle_cosmic < 30 && angle_beam > 30){
	  acc_cosmic_length += map_cluster_id_length[it->first];
	  num_cosmic ++;
	  if (max_length < map_cluster_id_length[it->first]) max_length = map_cluster_id_length[it->first];
	}else if (angle_cosmic < 35 && angle_beam > 40){
	  acc_cosmic_length += map_cluster_id_length[it->first];
	  num_cosmic ++;
	  if (max_length < map_cluster_id_length[it->first]) max_length = map_cluster_id_length[it->first];
	}
      }else{
	if (angle_cosmic < 20 || angle_cosmic < 30 && highest_y > 100*units::cm && it->first == main_vertex->get_cluster_id() ){
	  acc_cosmic_length += map_cluster_id_length[it->first];
	  num_cosmic ++;
	  if (max_length < map_cluster_id_length[it->first]) max_length = map_cluster_id_length[it->first];
	}
      }
      acc_total_length += map_cluster_id_length[it->first];
    }
    
    //    std::cout << num_cosmic << " Xin " << acc_cosmic_length/units::cm << " " << acc_small_length/units::cm << " " << acc_total_length/units::cm << " " << main_vertex->get_fit_pt() << " " << showers.size() << " " << highest_y/units::cm << " " << max_length/units::cm << " " << flag_main_cluster << std::endl;   
    
    if ( (num_cosmic >2 && acc_cosmic_length + acc_small_length  > 0.55 * (acc_total_length)||
	  num_cosmic >=2 && acc_cosmic_length + acc_small_length  > 0.7 * (acc_total_length) ||
	  num_cosmic ==1 && acc_cosmic_length + acc_small_length  > 0.625 * (acc_total_length) && highest_y > 102*units::cm
	   ) && main_vertex->get_fit_pt().y > 0 && flag_main_cluster && highest_y > 80*units::cm 
	 ) {
      neutrino_type |= 1UL << 1;
      //      std::cout << "A: " << true << std::endl;
      
      return true;
    }
  } // else

  // test front end ...
  for (auto it = map_vertex_segments.begin(); it != map_vertex_segments.end(); it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() != main_cluster->get_cluster_id()) continue;

    if (!fid->inside_fiducial_volume(vtx->get_fit_pt(), offset_x) && vtx->get_fit_pt().z < 15*units::cm){
      for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	WCPPID::ProtoSegment *sg = *it1;
	TVector3 dir = sg->cal_dir_3vector(vtx->get_fit_pt(),15*units::cm);

	double angle_beam = dir.Angle(dir_beam)/3.1415926*180.;
	if (angle_beam > 90) angle_beam = 180 - angle_beam;
	double length = sg->get_length();
	
	if ((!sg->get_flag_shower()) && sg->is_dir_weak() && angle_beam < 25 && length > 10*units::cm){
	  neutrino_type |= 1UL << 1;
	  //      std::cout << true << std::endl;
	  //	  std::cout << "B: " << true << std::endl;
	  return true;
	}
	//	std::cout << dir.Angle(dir_beam)/3.1415926*180. << " " << sg->get_flag_dir() << " " << sg->is_dir_weak() << std::endl;
      }
    }
    //    std::cout << vtx->get_fit_pt() << " " << fid->inside_fiducial_volume(vtx->get_fit_pt(), offset_x) << std::endl;
  }
  
  return false;
}
