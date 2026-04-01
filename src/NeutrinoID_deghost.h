#include "WCPData/DynamicToyPointCloud.h"

bool sortbysec(const std::pair<WCPPID::PR3DCluster*,double> &a,
	       const std::pair<WCPPID::PR3DCluster*,double> &b){
  return (a.second > b.second);
}

bool sortbysec1(const std::pair<WCPPID::ProtoSegment*,double> &a,
	       const std::pair<WCPPID::ProtoSegment*,double> &b){
  return (a.second > b.second);
}

void WCPPID::NeutrinoID::deghosting(){
  const bool flag_timing = true;
  using DG_Clock = std::chrono::high_resolution_clock;
  using DG_ms = std::chrono::duration<double, std::milli>;
  auto dg_t_total = DG_Clock::now();
  auto dg_t0 = DG_Clock::now();
  std::cout << "[deghosting] Start -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;

  deghost_clusters();
  if (flag_timing) std::cout << "[deghosting] timing: deghost_clusters took " << std::chrono::duration_cast<DG_ms>(DG_Clock::now()-dg_t0).count() << " ms" << std::endl;

  dg_t0 = DG_Clock::now();
  deghost_segments();
  if (flag_timing) std::cout << "[deghosting] timing: deghost_segments took " << std::chrono::duration_cast<DG_ms>(DG_Clock::now()-dg_t0).count() << " ms" << std::endl;

  dg_t0 = DG_Clock::now();
  std::set<WCPPID::PR3DCluster*> temp_clusters;
  for (auto it = map_cluster_main_vertices.begin(); it!= map_cluster_main_vertices.end(); it++){
    WCPPID::PR3DCluster *cluster = it->first;
    WCPPID::ProtoVertex *vertex = it->second;
    if (map_vertex_segments.find(vertex)==map_vertex_segments.end()){
      temp_clusters.insert(cluster);
    }
  }
  for (auto it = temp_clusters.begin(); it != temp_clusters.end(); it++){
    map_cluster_main_vertices.erase(*it);
  }
  if (flag_timing) std::cout << "[deghosting] timing: cleanup map_cluster_main_vertices took " << std::chrono::duration_cast<DG_ms>(DG_Clock::now()-dg_t0).count() << " ms" << std::endl;

  std::cout << "[deghosting] Done -- total took " << std::chrono::duration_cast<DG_ms>(DG_Clock::now()-dg_t_total).count() << " ms" << std::endl;
}

void WCPPID::NeutrinoID::deghost_segments(){
  // deghost segments ....
  TPCParams& mp = Singleton<TPCParams>::Instance();
  double pitch_u = mp.get_pitch_u();
  double pitch_v = mp.get_pitch_v();
  double pitch_w = mp.get_pitch_w();
  double angle_u = mp.get_angle_u();
  double angle_v = mp.get_angle_v();
  double angle_w = mp.get_angle_w();
  double time_slice_width = mp.get_ts_width();

  DynamicToyPointCloud global_point_cloud(angle_u,angle_v,angle_w);
  DynamicToyPointCloud global_steiner_point_cloud(angle_u,angle_v,angle_w);
  DynamicToyPointCloud global_skeleton_cloud(angle_u,angle_v,angle_w);

  // order cluster from longest to shortest ...
  std::map<int, WCPPID::ProtoSegmentSelection> map_cluster_id_segments;  // ID segments
  std::map<WCPPID::PR3DCluster*, double> cluster_length_map;
  WCPPID::PR3DClusterSelection ordered_clusters;
  order_clusters(ordered_clusters, map_cluster_id_segments, cluster_length_map);


  // std::cout << all_clusters.size() << std::endl;
  for (size_t i=0;i!=all_clusters.size();i++){
    if (cluster_length_map.find(all_clusters.at(i)) != cluster_length_map.end()) continue;
    global_point_cloud.AddPoints(all_clusters.at(i)->get_point_cloud());    
  }
  if (global_point_cloud.get_num_points() ==0) return;
  //  std::cout << global_point_cloud.get_num_points() << std::endl;
  // dist cut ...
  double dis_cut = 1.2*units::cm;
  
  for (size_t i=0;i!= ordered_clusters.size(); i++){
    // order segments from long to low
    WCPPID::ProtoSegmentSelection ordered_segments;
    order_segments(ordered_segments, map_cluster_id_segments[ordered_clusters.at(i)->get_cluster_id()]);


    for (size_t j=0;j!=ordered_segments.size(); j++){
      WCPPID::ProtoSegment *sg = ordered_segments.at(j);
      bool flag_add_seg = true;
      //  std::cout << global_point_cloud.get_num_points() << " " << global_steiner_point_cloud.get_num_points() << " " << global_skeleton_cloud.get_num_points() << std::endl;
      
      std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> pair_vertices = find_vertices(sg);
      double medium_dQ_dx = sg->get_medium_dQ_dx();
      double length = sg->get_length();
      int start_n = map_vertex_segments[pair_vertices.first].size();
      int end_n = map_vertex_segments[pair_vertices.second].size();
      
      int num_dead[3]={0,0,0};
      int num_unique[3]={0,0,0};
      int num_total_points = 0;

      if ((start_n==1 || end_n == 1) && medium_dQ_dx < 1.1 * 43e3/units::cm && length > 3.6*units::cm){
	PointVector& pts = sg->get_point_vec();
	num_total_points += pts.size();
	
	for (size_t k=0;k!=pts.size();k++){
	  Point test_point = pts.at(k);

	  
	  
	  bool flag_dead = ct_point_cloud->get_closest_dead_chs(test_point,0);
	  
	  
	  if (!flag_dead){
	    bool flag_in = false;
	    
	    std::tuple<double, WCP::PR3DCluster*, size_t> results = global_point_cloud.get_closest_2d_point_info(test_point, 0);
	    
	   
	    
	    if (std::get<0>(results)<=dis_cut*2./3.) flag_in = true;
	    if (global_steiner_point_cloud.get_num_points()!=0){
	      results = global_steiner_point_cloud.get_closest_2d_point_info(test_point, 0);
	      if (std::get<0>(results)<=dis_cut*2./3.) flag_in = true;
	    }
	    if ( global_skeleton_cloud.get_num_points()!=0){
	      results = global_skeleton_cloud.get_closest_2d_point_info(test_point, 0);
	      if (std::get<0>(results)> dis_cut*3./4.) flag_in = true;
	    }
	    if (!flag_in) num_unique[0]++;
	  }else{
	    num_dead[0]++;
	  }
	    

	  // V plane ...
	  flag_dead = ct_point_cloud->get_closest_dead_chs(test_point,1);
	  if (!flag_dead){
	    bool flag_in = false;
	    std::tuple<double, WCP::PR3DCluster*, size_t> results = global_point_cloud.get_closest_2d_point_info(test_point, 1);
	    if (std::get<0>(results)<=dis_cut*2./3.) flag_in = true;
	    if (global_steiner_point_cloud.get_num_points()!=0){
	      results = global_steiner_point_cloud.get_closest_2d_point_info(test_point, 1);
	      if (std::get<0>(results)<=dis_cut*2/3.) flag_in = true;
	    }
	    if ( global_skeleton_cloud.get_num_points()!=0){
	      results = global_skeleton_cloud.get_closest_2d_point_info(test_point, 1);
	      if (std::get<0>(results)<=dis_cut*3./4.) flag_in = true;
	    }
	    if (!flag_in) num_unique[1]++;
	  }else{
	    num_dead[1]++;
	  }
	    
	  // W plane ...
	  flag_dead = ct_point_cloud->get_closest_dead_chs(test_point,2);
	  if (!flag_dead){
	    bool flag_in = false;
	    std::tuple<double, WCP::PR3DCluster*, size_t> results = global_point_cloud.get_closest_2d_point_info(test_point, 2);
	    // std::cout << "A: " << std::get<0>(results)/units::cm << std::endl;
	    if (std::get<0>(results)<=dis_cut*2./3.) flag_in = true;
	    if (global_steiner_point_cloud.get_num_points()!=0){
	      results = global_steiner_point_cloud.get_closest_2d_point_info(test_point, 2);
	      //std::cout << "B: " << std::get<0>(results)/units::cm << std::endl;
	      if (std::get<0>(results)<=dis_cut*2./3.) flag_in = true;
	    }
	    if ( global_skeleton_cloud.get_num_points()!=0){
	      results = global_skeleton_cloud.get_closest_2d_point_info(test_point, 2);
	      //std::cout << "C: " << std::get<0>(results)/units::cm << std::endl;
	      if (std::get<0>(results)<=dis_cut*3./4.) flag_in = true;
	    }
	    if (!flag_in) num_unique[2]++;
	  }else{
	    num_dead[2]++;
	  }
	  
	}  
	
	//	std::cout << sg->get_cluster_id() << " " << sg->get_id() << " " <<start_n << " " << end_n << " " << pts.size() << " " << medium_dQ_dx/(43e3/units::cm) << " " << length/units::cm << " " << num_dead[0] << " " << num_dead[1] << " " << num_dead[2] << " " << num_unique[0] << " " << num_unique[1] << " " << num_unique[2] << " " << num_total_points<< std::endl;
	if (num_unique[0] + num_unique[1] + num_unique[2] == 0){
	  flag_add_seg = false;
	}
      }
	
      if (flag_add_seg){
	global_skeleton_cloud.AddPoints(sg->get_point_vec());
      }else{
	// protect main vertex ...
	
	WCPPID::PR3DCluster *tmp_cluster = map_segment_cluster[sg];
	if (map_cluster_main_vertices.find(tmp_cluster) != map_cluster_main_vertices.end()){
	  WCPPID::ProtoVertex *tmp_vtx = map_cluster_main_vertices[tmp_cluster];
	  if (map_vertex_segments[tmp_vtx].find(sg) != map_vertex_segments[tmp_vtx].end() && map_vertex_segments[tmp_vtx].size()==1){
	    flag_add_seg = true;
	  }
	}
	
	if (flag_add_seg){
	  global_skeleton_cloud.AddPoints(sg->get_point_vec());
	}else{
	  std::cout << "[deghost_segments] cluster " << sg->get_cluster_id()
		    << " removing segment id=" << sg->get_id()
		    << " len=" << length/units::cm << "cm"
		    << " dQ/dx=" << medium_dQ_dx/(43e3/units::cm)
		    << " unique=(" << num_unique[0] << "," << num_unique[1] << "," << num_unique[2] << ")"
		    << " dead=(" << num_dead[0] << "," << num_dead[1] << "," << num_dead[2] << ")"
		    << " pts=" << num_total_points
		    << " vtx1(" << pair_vertices.first->get_fit_pt().x/units::cm << "," << pair_vertices.first->get_fit_pt().y/units::cm << "," << pair_vertices.first->get_fit_pt().z/units::cm << ")"
		    << " vtx2(" << pair_vertices.second->get_fit_pt().x/units::cm << "," << pair_vertices.second->get_fit_pt().y/units::cm << "," << pair_vertices.second->get_fit_pt().z/units::cm << ")"
		    << std::endl;
	  // remove segment
	  del_proto_segment(sg);
	}
      }
    }
    
    
    
    // add points in ...
    global_point_cloud.AddPoints(ordered_clusters.at(i)->get_point_cloud());
    global_steiner_point_cloud.AddPoints(ordered_clusters.at(i)->get_point_cloud_steiner());
  }


  
  // remove vertices ...
  WCPPID::ProtoVertexSelection tmp_vertices;
  for (auto it1 = map_vertex_segments.begin(); it1!=map_vertex_segments.end(); it1++){
    if (it1->second.size()==0) tmp_vertices.push_back(it1->first);
  }
  for (auto it1 = tmp_vertices.begin(); it1!=tmp_vertices.end(); it1++){
    del_proto_vertex(*it1);
  }
}


void WCPPID::NeutrinoID::deghost_clusters(){
  std::map<int, WCPPID::ProtoSegmentSelection> map_cluster_id_segments;  // ID segments
  std::map<WCPPID::PR3DCluster*, double> cluster_length_map;
  WCPPID::PR3DClusterSelection ordered_clusters;
  order_clusters(ordered_clusters, map_cluster_id_segments, cluster_length_map);
  
  TPCParams& mp = Singleton<TPCParams>::Instance();
  double pitch_u = mp.get_pitch_u();
  double pitch_v = mp.get_pitch_v();
  double pitch_w = mp.get_pitch_w();
  double angle_u = mp.get_angle_u();
  double angle_v = mp.get_angle_v();
  double angle_w = mp.get_angle_w();
  double time_slice_width = mp.get_ts_width();
  
  DynamicToyPointCloud global_point_cloud(angle_u,angle_v,angle_w);
  DynamicToyPointCloud global_steiner_point_cloud(angle_u,angle_v,angle_w);
  DynamicToyPointCloud global_skeleton_cloud(angle_u,angle_v,angle_w);

  for (size_t i=0;i!=all_clusters.size();i++){
    if (cluster_length_map.find(all_clusters.at(i)) != cluster_length_map.end()) continue;
    global_point_cloud.AddPoints(all_clusters.at(i)->get_point_cloud());
  }
  
  
  std::vector<WCPPID::PR3DCluster*> to_be_removed_clusters;

  for (size_t i=0;i!= ordered_clusters.size(); i++){
    //  std::cout << i << " " << ordered_clusters.size() << std::endl;
    if (i==0){
      // fill anyway ...
      global_point_cloud.AddPoints(ordered_clusters.at(i)->get_point_cloud());
      global_steiner_point_cloud.AddPoints(ordered_clusters.at(i)->get_point_cloud_steiner());
      auto it = map_cluster_id_segments.find(ordered_clusters.at(i)->get_cluster_id());
      if (it != map_cluster_id_segments.end()){
	for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	  global_skeleton_cloud.AddPoints((*it1)->get_point_vec());
	}
      }
    }else{
      WCPPID::PR3DCluster *cluster = ordered_clusters.at(i);
      int num_dead[3]={0,0,0};
      int num_unique[3]={0,0,0};
      int num_total_points = 0;
      // dist cut ...
      double dis_cut = 1.2*units::cm;
      auto it = map_cluster_id_segments.find(ordered_clusters.at(i)->get_cluster_id());

      if (it != map_cluster_id_segments.end()){
	for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	  PointVector& pts = (*it1)->get_point_vec();
	  num_total_points += pts.size();
	  for (size_t j=0;j!=pts.size();j++){
	    Point test_point = pts.at(j);
	    bool flag_dead = ct_point_cloud->get_closest_dead_chs(test_point,0);
	    if (!flag_dead){
	      std::tuple<double, WCP::PR3DCluster*, size_t> results = global_point_cloud.get_closest_2d_point_info(test_point, 0);
	      if (std::get<0>(results)<=dis_cut/2.){
	      }else{
		results = global_steiner_point_cloud.get_closest_2d_point_info(test_point, 0);
		if (std::get<0>(results)<=dis_cut/3.*2.){
		}else{
		  results = global_skeleton_cloud.get_closest_2d_point_info(test_point, 0);
		  if (std::get<0>(results)<=dis_cut*6/4.){
		  }else{
		    num_unique[0]++;
		  }
		}
	      }
	    }else{
	      num_dead[0]++;
	    }

	  
	    // V plane ...
	    flag_dead = ct_point_cloud->get_closest_dead_chs(test_point,1);
	    if (!flag_dead){
	      std::tuple<double, WCP::PR3DCluster*, size_t> results = global_point_cloud.get_closest_2d_point_info(test_point, 1);
	      //std::cout << "A: " << std::get<0>(results)/units::cm << " ";
	      if (std::get<0>(results)<=dis_cut/2.){
	      }else{
		results = global_steiner_point_cloud.get_closest_2d_point_info(test_point, 1);
		//std::cout << std::get<0>(results)/units::cm << " ";
		if (std::get<0>(results)<=dis_cut/3.*2.){
		}else{
		  results = global_skeleton_cloud.get_closest_2d_point_info(test_point, 1);
		  //  std::cout  << std::get<0>(results)/units::cm << " ";
		  if (std::get<0>(results)<=dis_cut*6/4.){
		  }else{
		    num_unique[1]++;
		  }
		}
	      }
	      //	      std::cout << std::endl;
	    }else{
	      num_dead[1]++;
	    }

	    // W plane ...
	    flag_dead = ct_point_cloud->get_closest_dead_chs(test_point,2);
	    //std::cout << test_point.z << " " << flag_dead << std::endl;
	    if (!flag_dead){
	      std::tuple<double, WCP::PR3DCluster*, size_t> results = global_point_cloud.get_closest_2d_point_info(test_point, 2);
	      if (std::get<0>(results)<=dis_cut/2.){
	      }else{
		results = global_steiner_point_cloud.get_closest_2d_point_info(test_point, 2);
		if (std::get<0>(results)<=dis_cut/3.*2.){
		}else{
		  results = global_skeleton_cloud.get_closest_2d_point_info(test_point, 2);
		  if (std::get<0>(results)<=dis_cut*6/4.){
		  }else{
		    num_unique[2]++;
		  }
		}
	      }
	    }else{
	      num_dead[2]++;
	    }
	    
	    
	  }
	} // loop over segments
      }else{
	// no segments identified ...
	
      }
      
      bool flag_add = true;
      
      double unique_percent_u = num_unique[0]*1.0/num_total_points;
      double unique_percent_v = num_unique[1]*1.0/num_total_points;
      double unique_percent_w = num_unique[2]*1.0/num_total_points;

      double dead_percent_u = num_dead[0]*1.0/num_total_points;
      double dead_percent_v = num_dead[1]*1.0/num_total_points;
      double dead_percent_w = num_dead[2]*1.0/num_total_points;
      
      double max_unique_percent = std::max(unique_percent_u, unique_percent_v);
      max_unique_percent = std::max(max_unique_percent, unique_percent_w);
      double min_unique_percent = std::min(unique_percent_u, unique_percent_v);
      min_unique_percent = std::min(min_unique_percent, unique_percent_w);
      double ave_unique_percent = (unique_percent_u + unique_percent_v + unique_percent_w)/3.;
      double max_dead_percent = std::max(dead_percent_u, dead_percent_v);
      max_dead_percent = std::max(max_dead_percent, dead_percent_w);
      
      
      
      if (max_dead_percent >= 0.8 && max_unique_percent <= 0.35 && ave_unique_percent <= 0.16 && min_unique_percent <= 0.08 ||
	  max_unique_percent <= 0.1 && ave_unique_percent <= 0.05 && min_unique_percent <= 0.025 ||
	  max_dead_percent < 0.8 && max_dead_percent >=0.7 && max_unique_percent <= 0.2 && ave_unique_percent <= 0.1 && min_unique_percent <= 0.05)
	flag_add = false;

      //      if(!flag_add)
      if ((num_dead[0]==num_total_points || num_dead[1]==num_total_points || num_dead[2]==num_total_points) // one plane must be dead ...
	  &&(num_unique[0]==0 && num_unique[1]==0 || num_unique[0]==0 && num_unique[2]==0 || num_unique[2]==0 && num_unique[1]==0) // two plane's unique are zero
	  && flag_add && max_unique_percent < 0.75) // the rest is smaller than 75% 
	flag_add = false;
      //	std::cout << flag_add << " " << cluster->get_cluster_id() << " " << num_total_points << " " << num_dead[0] << " " << num_dead[1] << " " << num_dead[2] << " " << num_unique[0] << " " << num_unique[1] << " " << num_unique[2] << " " << max_unique_percent << " " << min_unique_percent << " " << ave_unique_percent << " " << max_dead_percent << std::endl;
      
      if (flag_add){
	global_point_cloud.AddPoints(ordered_clusters.at(i)->get_point_cloud());
	global_steiner_point_cloud.AddPoints(ordered_clusters.at(i)->get_point_cloud_steiner());
	auto it = map_cluster_id_segments.find(ordered_clusters.at(i)->get_cluster_id());
	if (it != map_cluster_id_segments.end()){
	  for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	    global_skeleton_cloud.AddPoints((*it1)->get_point_vec());
	  }
	}
      }else{
	std::cout << "[deghost_clusters] cluster " << cluster->get_cluster_id()
		  << " ghosted len=" << cluster_length_map[cluster]/units::cm << "cm"
		  << " pts=" << num_total_points
		  << " unique=(" << int(unique_percent_u*100+0.5) << "%," << int(unique_percent_v*100+0.5) << "%," << int(unique_percent_w*100+0.5) << "%)"
		  << " dead=(" << int(dead_percent_u*100+0.5) << "%," << int(dead_percent_v*100+0.5) << "%," << int(dead_percent_w*100+0.5) << "%)"
		  << std::endl;
	to_be_removed_clusters.push_back(cluster);
      }
      
    }
  } // loop over cluster


  //  std::cout << to_be_removed_clusters.size() << std::endl;
  
  for (auto it = to_be_removed_clusters.begin(); it!= to_be_removed_clusters.end(); it++){
    auto it2 = map_cluster_id_segments.find((*it)->get_cluster_id());
    if (it2 != map_cluster_id_segments.end()){
      //  std::cout << it2->second.size() << std::endl;
      for (auto it1 = it2->second.begin(); it1 != it2->second.end(); it1++){
    	del_proto_segment(*it1);
      }
    }
  }
  
  
  WCPPID::ProtoVertexSelection tmp_vertices;
  for (auto it1 = map_vertex_segments.begin(); it1!=map_vertex_segments.end(); it1++){
    if (it1->second.size()==0) tmp_vertices.push_back(it1->first);
  }
  //  std::cout << "A: " << tmp_vertices.size() << std::endl;
  for (auto it1 = tmp_vertices.begin(); it1!=tmp_vertices.end(); it1++){
    //std::cout << (*it1)->get_id() << " " << (*it1)->get_cluster_id() << std::endl;
    del_proto_vertex(*it1);
  }
  
}


void WCPPID::NeutrinoID::order_segments(WCPPID::ProtoSegmentSelection& ordered_segments, WCPPID::ProtoSegmentSelection& segments){
  std::vector<std::pair<WCPPID::ProtoSegment*, double> >  temp_pair_vec;
  for (auto it = segments.begin(); it != segments.end(); it++){
    temp_pair_vec.push_back(std::make_pair(*it, (*it)->get_length()));
  }
  sort(temp_pair_vec.begin(), temp_pair_vec.end(), sortbysec1);
  for (auto it = temp_pair_vec.begin(); it!=temp_pair_vec.end();it++){
    ordered_segments.push_back(it->first);
    //    std::cout << it->first->get_cluster_id() << " " << it->first << " " << it->second/units::cm << std::endl;    
  }
  
}


void WCPPID::NeutrinoID::order_clusters(WCPPID::PR3DClusterSelection& ordered_clusters, std::map<int, WCPPID::ProtoSegmentSelection>& map_cluster_id_segments, std::map<WCPPID::PR3DCluster*, double>& map_cluster_total_length){
  
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    double length = sg->get_length();

    //std::cout << sg->get_point_vec().size() << " " << sg->get_point_vec().front() << " " << sg->get_point_vec().back() << std::endl;
    
    if (map_cluster_total_length.find(map_id_cluster[sg->get_cluster_id()])==map_cluster_total_length.end()){
      WCPPID::ProtoSegmentSelection segments;
      segments.push_back(sg);
      map_cluster_id_segments[sg->get_cluster_id()] =segments;
      map_cluster_total_length[map_id_cluster[sg->get_cluster_id()]] = length;
    }else{
      map_cluster_id_segments[sg->get_cluster_id()].push_back(sg);
      map_cluster_total_length[map_id_cluster[sg->get_cluster_id()]] += length;
    }
  }
  
 
  //  std::cout << map_cluster_total_length.size() << " " << map_cluster_id_segments.size() << std::endl;
  std::vector<std::pair<WCPPID::PR3DCluster*, double>> temp_pair_vec;
  for (auto it = map_cluster_total_length.begin(); it!=map_cluster_total_length.end(); it++){
    temp_pair_vec.push_back(std::make_pair(it->first, it->second));
  }
  sort(temp_pair_vec.begin(), temp_pair_vec.end(), sortbysec);
  for (auto it = temp_pair_vec.begin(); it!=temp_pair_vec.end();it++){
    ordered_clusters.push_back(it->first);
    //    std::cout << it->first->get_cluster_id() << " " << map_cluster_total_length[it->first] << " " << map_cluster_id_segments[it->first->get_cluster_id()].size() << std::endl;
  }
}
