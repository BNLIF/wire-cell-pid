#include "WCPPID/ProtectOverClustering.h"
#include "TH1F.h"

using namespace WCP;

void WCPPID::Protect_Over_Clustering(double eventTime, std::vector<std::pair<int, Opflash*> >& to_be_checked, WCPPID::PR3DClusterSelection& live_clusters, std::map<WCPPID::PR3DCluster*, int>& map_cluster_parent_id,  std::map<int, std::vector<WCPPID::PR3DCluster*> >& map_parentid_clusters, WCP::ToyCTPointCloud& ct_point_cloud, bool flag_match_data, int run_no, double time_offset, double nrebin, double time_slice_width, bool flag_timestamp ){
  // update live_clusters,  map_parentid_clusters, map_cluster_parent_id;
  // keep the original id for the main cluster ...

  WCP::Photon_Library *pl = 0;

  
  std::set<int> used_cluster_ids;
  for (auto it = live_clusters.begin(); it!=live_clusters.end(); it++){
    used_cluster_ids.insert((*it)->get_cluster_id());
    //std::cout << "haha: " << (*it)->get_cluster_id() << std::endl;
  }
  
  WCPPID::PR3DClusterSelection temp_live_clusters;
  int acc_cluster_id = 0;
  // std::cout << get_next_cluster_id(acc_cluster_id, used_cluster_ids) << std::endl;
  
  std::set<WCPPID::PR3DCluster*> examined_clusters;
  for (size_t i=0;i!=to_be_checked.size(); i++){

    int curr_main_cluster_id = to_be_checked.at(i).first;
    auto it = map_parentid_clusters.find(curr_main_cluster_id);
    if (it == map_parentid_clusters.end()) continue;
    WCP::Opflash* flash = to_be_checked.at(i).second;
    double offset_x = (to_be_checked.at(i).second->get_time() - time_offset)*2./nrebin*time_slice_width;

    // create a total cluster holding all mcells ...
    //    WCPPID::PR3DCluster *total_cluster = new PR3DCluster(-1);

    std::set<WCPPID::PR3DCluster*> examined_clusters_1;
    for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
      WCPPID::PR3DCluster *temp_cluster = *it1;
      examined_clusters.insert(temp_cluster);
      examined_clusters_1.insert(temp_cluster);
      //WCP::SMGCSelection& mcells = temp_cluster->get_mcells();
      //for (size_t i=0;i!=mcells.size();i++){
      //	total_cluster->AddCell(mcells.at(i), mcells.at(i)->GetTimeSlice());
      //}
      
      map_cluster_parent_id.erase(temp_cluster);
      used_cluster_ids.erase(temp_cluster->get_cluster_id());
    }
    used_cluster_ids.insert(curr_main_cluster_id);
    map_parentid_clusters[curr_main_cluster_id].clear();

    
    // examine these clusters ...
    for (auto it1 = examined_clusters_1.begin(); it1 != examined_clusters_1.end(); it1++){
      WCPPID::PR3DCluster *temp_cluster = *it1;
      std::vector<WCP::SMGCSelection> vec_mcells;
      vec_mcells = temp_cluster->Examine_graph(ct_point_cloud);

      //      std::cout << examined_clusters.size() << " " << vec_mcells.size() << std::endl;
      if (temp_cluster->get_cluster_id() == curr_main_cluster_id){
	// first round examination ...
	int max_number_cells = vec_mcells.front().size();
	int second_max = 0;
	int main_id = 0;
	for (size_t j=1;j<vec_mcells.size();j++){
	  if (vec_mcells.at(j).size() > max_number_cells){
	    main_id = j;
	    second_max = max_number_cells;
	    max_number_cells = vec_mcells.at(j).size();
	  }
	}

	// //	std::cout << second_max << " " << max_number_cells << " " << main_id << std::endl;
	// // select the main cluster according to the compare_pe_pattern  (comment temporarily)
	// if (second_max > 0.7 * max_number_cells){
	//   if (pl == 0 ) pl = new Photon_Library(eventTime, run_no,flag_match_data, false, flag_timestamp);
	//   std::pair<double, double> results = WCPPID::compare_pe_pattern(eventTime, run_no, offset_x, pl, vec_mcells.at(main_id), flash ,flag_match_data, flag_timestamp);
	//   double min_score = results.first;
	//   double min_ratio = results.second;
	  
	//   // second round of examination ...
	//   for (size_t j=0;j<vec_mcells.size();j++){
	//     if (vec_mcells.at(j).size() < 0.7 * max_number_cells || j==main_id) continue;
	//     results = WCPPID::compare_pe_pattern(eventTime, run_no, offset_x, pl, vec_mcells.at(j), flash ,flag_match_data, flag_timestamp);
	//     double score;
	//     if (fabs(1-results.second) > fabs(1-min_ratio)){
	//       score = sqrt(pow(results.first,2) + pow(results.second - min_ratio,2));
	//     }else{
	//       score = sqrt(pow(results.first,2));
	//     }
	//     //	    std::cout << score << " " <<  min_score << " " << results.first << " "<< results.second << " " << min_ratio << std::endl;
	    
	//     if (score < min_score){
	//       min_score = score;
	//       main_id = j;
	//       max_number_cells = vec_mcells.at(j).size();
	//     }
	//   }

	//   //	  std::cout << main_id << " " << max_number_cells << std::endl;
	// }
	
	
	// main cluster replacement 
	WCPPID::PR3DCluster *new_cluster = new WCPPID::PR3DCluster(curr_main_cluster_id);
	for (auto it2 = vec_mcells.at(main_id).begin(); it2!=vec_mcells.at(main_id).end();it2++){
	  new_cluster->AddCell(*it2, (*it2)->GetTimeSlice());
	}
	temp_live_clusters.push_back(new_cluster);
	map_cluster_parent_id[new_cluster] = curr_main_cluster_id;
	map_parentid_clusters[curr_main_cluster_id].push_back(new_cluster);
	
	// rest ...
	for (size_t j = 0; j!=vec_mcells.size(); j++){
	  if (j==main_id) continue;
	  acc_cluster_id = get_next_cluster_id(acc_cluster_id, used_cluster_ids);
	  WCPPID::PR3DCluster *new_cluster = new WCPPID::PR3DCluster(acc_cluster_id);
	  for (auto it2 = vec_mcells.at(j).begin(); it2!=vec_mcells.at(j).end();it2++){
	    new_cluster->AddCell(*it2, (*it2)->GetTimeSlice());
	  }
	  used_cluster_ids.insert(acc_cluster_id);
	  temp_live_clusters.push_back(new_cluster);
	  map_cluster_parent_id[new_cluster] = curr_main_cluster_id;
	  map_parentid_clusters[curr_main_cluster_id].push_back(new_cluster);
	}
	
      }else{
	for (size_t j = 0; j!=vec_mcells.size(); j++){
	  acc_cluster_id = get_next_cluster_id(acc_cluster_id, used_cluster_ids);
	  WCPPID::PR3DCluster *new_cluster = new WCPPID::PR3DCluster(acc_cluster_id);
	  for (auto it2 = vec_mcells.at(j).begin(); it2!=vec_mcells.at(j).end();it2++){
	    new_cluster->AddCell(*it2, (*it2)->GetTimeSlice());
	  }
	  used_cluster_ids.insert(acc_cluster_id);
	  temp_live_clusters.push_back(new_cluster);
	  map_cluster_parent_id[new_cluster] = curr_main_cluster_id;
	  map_parentid_clusters[curr_main_cluster_id].push_back(new_cluster);
	}
      } // main or other ...
    } // examine each cluster
    
    //    delete total_cluster;
    
  } // to be checked loop
  

  
  for (auto it = live_clusters.begin(); it != live_clusters.end(); it++){
    if (examined_clusters.find(*it)==examined_clusters.end())
      temp_live_clusters.push_back(*it);
  }
  
  for (auto it = examined_clusters.begin(); it!=examined_clusters.end(); it++){
    delete (*it);
  }
  
  live_clusters = temp_live_clusters;

  if (pl!=0) delete pl;
   
}



int WCPPID::get_next_cluster_id(int acc_cluster_id, std::set<int>& used_cluster_ids){
  acc_cluster_id ++;
  while (used_cluster_ids.find(acc_cluster_id)!=used_cluster_ids.end()) acc_cluster_id++;
  return acc_cluster_id;
}



std::pair<double, double> WCPPID::compare_pe_pattern(double eventTime, int run_no, double offset_x, WCP::Photon_Library *pl, WCP::SMGCSelection& mcells, WCP::Opflash* flash, bool flag_match_data, bool flag_timestamp){

  //  std::cout << "ZXin_2: " << eventTime << " " << flag_timestamp << std::endl;
  
  std::vector<double> pred_pmt_light(32,0);
  double rel_light_yield_err = pl->rel_light_yield_err;
  double scaling_light_mag = pl->scaling_light_mag;
  std::map<int,int>* map_lib_pmt = &pl->map_lib_pmt;
  std::map<int,int>* map_pmt_lib = &pl->map_pmt_lib;
  std::vector<std::list<std::pair<int,float>>>* photon_library = &pl->library;
  
  double high_x_cut = 256 * units::cm;
  double high_x_cut_ext1 = + 1.2*units::cm;
  double high_x_cut_ext2 = - 2.0*units::cm;
  double low_x_cut = 0*units::cm;
  double low_x_cut_ext1 = - 2*units::cm;
  double low_x_cut_ext2 = + 4.0*units::cm;
  
  
  WCPPID::PR3DCluster *main_cluster = new WCPPID::PR3DCluster(0);
  for (auto it2 = mcells.begin(); it2!=mcells.end();it2++){
    main_cluster->AddCell(*it2, (*it2)->GetTimeSlice());
  }
  
  
  double first_pos_x = (*((main_cluster->get_time_cells_set_map().begin())->second.begin()))->get_sampling_points().front().x;
  double last_pos_x = (*((main_cluster->get_time_cells_set_map().rbegin())->second.begin()))->get_sampling_points().front().x;

  bool flag_spec_end = false;
  
  // improve the position code ... 
  if (first_pos_x - offset_x <= low_x_cut + low_x_cut_ext1 &&
      first_pos_x - offset_x > low_x_cut - 120*units::cm ){
    
    std::map<int,SMGCSet>& time_cells_set_map = main_cluster->get_time_cells_set_map();
    int num_mcells_outside = 0;
    int num_time_slices_outside = 0;
    
    int num_mcells_def_outside = 0;
    
    double prev_pos_x= first_pos_x;
    double current_pos_x = first_pos_x;
    for (auto it3 = time_cells_set_map.begin(); it3 != time_cells_set_map.end(); it3++){
      current_pos_x = (*(it3->second.begin()))->get_sampling_points().front().x;
      
      
      // if (flash->get_flash_id()==35&&abs(main_cluster->get_cluster_id()-5)<=0)
      // std::cout << num_time_slices_outside<< " " << num_mcells_outside << "  " << (first_pos_x - offset_x)/units::cm << " " <<
      // 	(prev_pos_x-offset_x)/units::cm << " " << (current_pos_x-offset_x)/units::cm << std::endl;
      
      if (current_pos_x -offset_x > low_x_cut + low_x_cut_ext1 && current_pos_x - prev_pos_x > 0.75*units::cm)
	break;
      if (num_time_slices_outside > 60) break;
      
      if (current_pos_x -offset_x < low_x_cut + low_x_cut_ext1) num_mcells_def_outside += it3->second.size();
      
      num_time_slices_outside += 1;
      num_mcells_outside += it3->second.size();
      prev_pos_x = current_pos_x;
    }
    if (num_time_slices_outside <=36 && num_mcells_outside < 0.05*main_cluster->get_num_mcells()){
      first_pos_x = current_pos_x;
      if (num_time_slices_outside > 10 && fabs(current_pos_x - prev_pos_x)<10*units::cm)
	flag_spec_end = true;
    }else if (num_time_slices_outside <=60 && num_mcells_outside < 0.06*main_cluster->get_num_mcells() && fabs(current_pos_x - prev_pos_x)>10*units::cm){
      first_pos_x = current_pos_x;
    }else if (num_time_slices_outside <=25 && num_mcells_outside < 0.12 * main_cluster->get_num_mcells() && fabs(current_pos_x - prev_pos_x)>20*units::cm){
      first_pos_x = current_pos_x;
    }
    
    if (num_mcells_def_outside < 0.0015 * main_cluster->get_num_mcells()&&num_mcells_def_outside>0)
      first_pos_x = offset_x;
    
    //  if (flash->get_flash_id()==61&&main_cluster->get_cluster_id()==1)
    // std::cout << num_mcells_outside << " " << main_cluster->get_num_mcells() << "  A " << (first_pos_x - offset_x)/units::cm << " " <<
    //  (prev_pos_x-offset_x)/units::cm << " " << (current_pos_x-offset_x)/units::cm << " " << num_mcells_def_outside << std::endl;
    
  }
  if (last_pos_x - offset_x >= high_x_cut + high_x_cut_ext1 &&
      last_pos_x - offset_x < high_x_cut + 120*units::cm){
    std::map<int,SMGCSet>& time_cells_set_map = main_cluster->get_time_cells_set_map();
    int num_mcells_outside = 0;
    int num_time_slices_outside = 0;
    int num_mcells_def_outside = 0;
    double prev_pos_x= last_pos_x;
    double current_pos_x = last_pos_x;
    
    for (auto it3 = time_cells_set_map.rbegin(); it3 != time_cells_set_map.rend(); it3++){
      current_pos_x = (*(it3->second.begin()))->get_sampling_points().front().x;
      if (current_pos_x -offset_x<high_x_cut + high_x_cut_ext1 && fabs(current_pos_x - prev_pos_x) > 0.75*units::cm)
	break;
      if (num_time_slices_outside > 60) break;
      
      
      if (current_pos_x -offset_x>high_x_cut + high_x_cut_ext1) num_mcells_def_outside +=it3->second.size();
      
      num_time_slices_outside += 1;
      num_mcells_outside += it3->second.size();
      prev_pos_x = current_pos_x;
    }
    if (num_time_slices_outside <=36 && num_mcells_outside < 0.05*main_cluster->get_num_mcells()){
      last_pos_x = current_pos_x;
      if (num_time_slices_outside > 10 && fabs(current_pos_x - prev_pos_x)<10*units::cm)
	flag_spec_end = true;
    }else if (num_time_slices_outside <=60 && num_mcells_outside < 0.06*main_cluster->get_num_mcells() && fabs(current_pos_x - prev_pos_x)>10*units::cm){
      last_pos_x = current_pos_x;
    }else if (num_time_slices_outside <=25 && num_mcells_outside < 0.12 * main_cluster->get_num_mcells() && fabs(current_pos_x - prev_pos_x)>20*units::cm){
      last_pos_x = current_pos_x;
    }
    
    if (num_mcells_def_outside < 0.0015 * main_cluster->get_num_mcells()&&num_mcells_def_outside>0)
      last_pos_x = offset_x+high_x_cut;
    
    // if (flash->get_flash_id()==19&&main_cluster->get_cluster_id()==19)
    //   std::cout << flash->get_flash_id() << " "<< main_cluster->get_cluster_id() << " " << (first_pos_x-offset_x)/units::cm << " " << (last_pos_x-offset_x)/units::cm << " " << num_time_slices_outside << " " << num_mcells_outside << " " << main_cluster->get_num_mcells() << " " << fabs(current_pos_x - prev_pos_x)/units::cm << std::endl;
    
  }
  
  // if (flash->get_flash_id()==16 && main_cluster->get_cluster_id()==13 )
  //   std::cout << flash->get_flash_id() << " "<< main_cluster->get_cluster_id() << " " << (first_pos_x-offset_x)/units::cm << " " << (last_pos_x-offset_x)/units::cm << std::endl;
  
  // if (flash->get_flash_id()==39)
  //   std::cout << flash->get_flash_id() << " " << main_cluster->get_cluster_id() << " " << offset_x/units::cm << " " << (first_pos_x-offset_x)/units::cm << " " << (last_pos_x-offset_x)/units::cm << " " << std::endl;
  
  if (first_pos_x-offset_x > low_x_cut + low_x_cut_ext1 -1.0*units::cm &&
      last_pos_x-offset_x > low_x_cut &&
      last_pos_x-offset_x < high_x_cut + high_x_cut_ext1 &&
      first_pos_x-offset_x < high_x_cut){
    
    //bundle->set_spec_end_flag(flag_spec_end);
    
    //if (first_pos_x-offset_x <=low_x_cut + low_x_cut_ext2 && first_pos_x-offset_x > low_x_cut + low_x_cut_ext1 - 1.0*units::cm ){
    // bundle->set_flag_close_to_PMT(true);
    //   bundle->set_flag_at_x_boundary(true);
    // }else if (first_pos_x-offset_x <= low_x_cut + low_x_cut_ext1 && first_pos_x-offset_x > low_x_cut + low_x_cut_ext1 -1.0*units::cm){
    //bundle->set_flag_close_to_PMT(true);
    //bundle->set_flag_at_x_boundary(true);
    //}
    //if (last_pos_x-offset_x >= high_x_cut + high_x_cut_ext2 && last_pos_x-offset_x < high_x_cut + high_x_cut_ext1){
    //bundle->set_flag_at_x_boundary(true);
    //}
    
    WCPPID::PR3DClusterSelection temp_clusters;
    temp_clusters.push_back(main_cluster);
    
    for (auto it3 = temp_clusters.begin(); it3!=temp_clusters.end(); it3++){
      SMGCSelection& mcells = (*it3)->get_mcells();
      bool flag_save = true;
      
      if ((*it3) == main_cluster)  flag_save = false;
      
      for (auto it4 = mcells.begin(); it4!=mcells.end(); it4++){
	SlimMergeGeomCell *mcell = (*it4);
	if (mcell->get_q()>0){
	  PointVector& pts = mcell->get_sampling_points();
	  if (pts.at(0).x-offset_x < low_x_cut+low_x_cut_ext1 ||
	      pts.at(0).x-offset_x > high_x_cut+high_x_cut_ext1){
	    flag_save = false;
	    continue;
	  }
	  
	  float charge = mcell->get_q()/pts.size();
	  Point p;
	  for (size_t i=0;i!=pts.size();i++){
	    p.x = pts.at(i).x - offset_x;
	    p.y = pts.at(i).y;
	    p.z = pts.at(i).z;
	    
	    int voxel_id = WCPPID::convert_xyz_voxel_id(p);
	    std::list<std::pair<int,float>>& pmt_list = photon_library->at(voxel_id);
	    
	    for (auto it5 = pmt_list.begin(); it5!=pmt_list.end(); it5++){
	      pred_pmt_light[(*map_lib_pmt)[it5->first]] += charge * it5->second;
	    }
	  }
	}
      }
      //if (flag_save)
      //  more_clusters->push_back(*it3);
      
    } // loop over all clusters within this TPC object...
    
    // veto PMT 18 ([17])
    double norm_factor[32];
    for (int i=0;i!=32;i++){
      norm_factor[i] = 1;
    }
    if (flag_match_data){
      if ((run_no >= 12809 && (!flag_timestamp)) || (flag_timestamp && eventTime >= 1505170407))
	norm_factor[17] = 0;
    }
    //std::cout << "kaka1 " << run_no << " " << eventTime << " " << flag_timestamp << " " << eventTime - 1505170407 << " " << norm_factor[17] << std::endl;
    
    double sum1 = 0, sum2 = 0, max_pe = 0;
    for (size_t i=0;i!=32;i++){
      pred_pmt_light[i] *= scaling_light_mag * norm_factor[i];
      
      sum1 += flash->get_PE(i);
      sum2 += pred_pmt_light[i];
      if (pred_pmt_light[i] > max_pe)
	max_pe = pred_pmt_light[i];
    }
    
    // if (sum2 < sum1 * 3){ // three times allowrance ... 
    //flag_good_bundle = true;
    // }
  }

  delete main_cluster;
  
  //  return pred_pmt_light;

  double ks_score=0;
  double norm_ratio=1;

  double sum_pred_light = 0;
  TH1F *h1_meas = new TH1F("h1_meas", "h1_meas", 32, 0, 32);
  TH1F *h1_pred = new TH1F("h1_pred", "h1_pred", 32, 0, 32);
  for (size_t i=0;i!=pred_pmt_light.size();i++){
    sum_pred_light += pred_pmt_light.at(i);
    h1_pred->SetBinContent(i+1,pred_pmt_light.at(i));
    h1_meas->SetBinContent(i+1,flash->get_PE(i));
  }
  double sum_obsv_light = flash->get_total_PE();
  norm_ratio = sum_pred_light/sum_obsv_light;
  ks_score = h1_meas->KolmogorovTest(h1_pred,"M");
  delete h1_meas;
  delete h1_pred;
  
  return std::make_pair(ks_score, norm_ratio);
}


//Helper Function
int WCPPID::convert_xyz_voxel_id(WCP::Point &p){
  // int voxel_x_id = std::round((p.x/units::cm+64.825-5.14667/2.)/5.14667);
  // int voxel_y_id = std::round((p.y/units::cm+193-5.14667/2.)/5.14667);
  // int voxel_z_id = std::round((p.z/units::cm+128.243-3.23122/2.)/3.23122);
  
  int voxel_x_id = std::round((p.x/units::cm+63.435-5.1096/2.)/5.1095);
  int voxel_y_id = std::round((p.y/units::cm+191.61-5.1096/2.)/5.1096);
  int voxel_z_id = std::round((p.z/units::cm+92.375-3.05437/2.)/3.05437);
  //fit
  // int voxel_y_id = std::round((p.y/units::cm+188.416-5.11698/2.)/5.11698);
  // int voxel_z_id = std::round((p.z/units::cm+88.3554-3.05768/2.)/3.05768);
  
  if (voxel_x_id<0) voxel_x_id=0;
  if (voxel_x_id>=75) voxel_x_id=74;
  if (voxel_y_id<0) voxel_y_id=0;
  if (voxel_y_id>=75) voxel_y_id=74;
  if (voxel_z_id<0) voxel_z_id=0;
  if (voxel_z_id>=400) voxel_z_id=399;

  int voxel_id = voxel_z_id*75*75 + voxel_y_id*75 + voxel_x_id;
  return voxel_id;
} 
