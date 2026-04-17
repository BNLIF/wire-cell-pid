#include "WCPPID/NeutrinoID.h"

#include "WCPData/TPCParams.h"
#include "WCPData/Singleton.h"

#include <boost/graph/connected_components.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>

#include "WCPPaal/graph_metrics.h"
#include "WCPPaal/steiner_tree_greedy.h"

#include "WCPData/Line.h"


using namespace WCP;

#include "NeutrinoID_proto_vertex.h"
#include "NeutrinoID_final_structure.h"
#include "NeutrinoID_improve_vertex.h"
#include "NeutrinoID_examine_structure.h"
#include "NeutrinoID_deghost.h"
#include "NeutrinoID_track_shower.h"
#include "NeutrinoID_energy_reco.h"
#include "NeutrinoID_shower_clustering.h"
#include "NeutrinoID_em_shower.h"

#include "NeutrinoID_cosmic_tagger.h"
#include "NeutrinoID_numu_tagger.h"
#include "NeutrinoID_nue_tagger.h"
#include "NeutrinoID_nue_functions.h"
#include "NeutrinoID_pio_tagger.h"
#include "NeutrinoID_singlephoton_tagger.h"

#include "NeutrinoID_ssm_tagger.h"

#include "NeutrinoID_numu_bdts.h"
#include "NeutrinoID_nue_bdts.h"
#include "NeutrinoID_DL.h"

#include "NeutrinoID_kine.h"

WCPPID::NeutrinoID::NeutrinoID(WCPPID::PR3DCluster *main_cluster1, std::vector<WCPPID::PR3DCluster*>& other_clusters1, std::vector<WCPPID::PR3DCluster*>& all_clusters1, WCPPID::ToyFiducial* fid, WCPSst::GeomDataSource& gds, int nrebin, int frame_length, float unit_dis, ToyCTPointCloud* ct_point_cloud, std::map<int,std::map<const GeomWire*, SMGCSelection > >& global_wc_map, double flash_time, double offset_x, int flag_neutrino_id_process, int flag_bdt, bool flag_dl_vtx, double dl_vtx_cut, float match_isFC, bool is_neutrino_candidate, const std::set<std::string>& debug_funcs_init)
  : acc_vertex_id(0)
  , acc_segment_id(0)
  , debug_funcs(debug_funcs_init)
  , main_cluster(main_cluster1)
  , other_clusters(other_clusters1)
  , all_clusters(all_clusters1)
  , fid(fid)
  , ct_point_cloud(ct_point_cloud)
  , global_wc_map(global_wc_map)
  , flash_time(flash_time)
  , offset_x(offset_x)
  , flag_neutrino_id_process(flag_neutrino_id_process)
  , type(0)
  , main_vertex(0)
  , main_cluster_initial_pair_vertices(std::make_pair((WCPPID::ProtoVertex*)0, (WCPPID::ProtoVertex*)0))
  , neutrino_type(0)
  , flag_bdt(flag_bdt)
  , flag_dl_vtx(flag_dl_vtx)
  , dl_vtx_cut(dl_vtx_cut)
  , match_isFC(match_isFC)
{
  bool flag_other_clusters = true;
  bool flag_main_cluster = true;
  bool flag_tagger = true;

  // hack the main cluster
  // for (auto it = other_clusters.begin(); it != other_clusters.end(); it++){
  //   WCPPID::PR3DCluster *cluster = *it;    
  //   if (cluster->get_cluster_id()==67) {
  //     swap_main_cluster(cluster);
  //     break;
  //   }
  // }
  // form id vs. cluster ...
  const bool flag_timing_csg = true;
  using CSGClock = std::chrono::high_resolution_clock;
  using CSGms = std::chrono::duration<double, std::milli>;
  auto csg_t_total = CSGClock::now();
  auto csg_t0 = CSGClock::now();

  map_id_cluster[main_cluster->get_cluster_id()] = main_cluster;
  main_cluster->create_steiner_graph(*ct_point_cloud, gds, nrebin, frame_length, unit_dis);
  {
      std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> two_wcps = main_cluster->get_two_boundary_wcps();
      map_cluster_length[main_cluster] = sqrt(pow(two_wcps.first.x - two_wcps.second.x, 2) + pow(two_wcps.first.y - two_wcps.second.y, 2) + pow(two_wcps.first.z - two_wcps.second.z, 2));
  }
  if (flag_timing_csg) std::cout << "create_steiner_graph timing: main_cluster took " << std::chrono::duration_cast<CSGms>(CSGClock::now()-csg_t0).count() << " ms" << std::endl;


  if (flag_other_clusters){
    for (auto it = other_clusters.begin(); it!=other_clusters.end(); it++){
      csg_t0 = CSGClock::now();
      map_id_cluster[(*it)->get_cluster_id()] = *it;
      (*it)->create_steiner_graph(*ct_point_cloud, gds, nrebin, frame_length, unit_dis);
      std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> two_wcps = (*it)->get_two_boundary_wcps();
      double length = sqrt(pow(two_wcps.first.x - two_wcps.second.x, 2) + pow(two_wcps.first.y - two_wcps.second.y, 2) + pow(two_wcps.first.z - two_wcps.second.z, 2));
      map_cluster_length[*it] = length;
      if (flag_timing_csg) std::cout << "create_steiner_graph timing: other_cluster id=" << (*it)->get_cluster_id() << " took " << std::chrono::duration_cast<CSGms>(CSGClock::now()-csg_t0).count() << " ms" << std::endl;

    //      std::cout << (*it)->get_cluster_id() << " " << length/units::cm << std::endl;
    }
  }
  if (flag_timing_csg) std::cout << "create_steiner_graph timing: TOTAL took " << std::chrono::duration_cast<CSGms>(CSGClock::now()-csg_t_total).count() << " ms" << std::endl;

  if (is_neutrino_candidate){
    

    init_tagger_info();

  // std::cout << main_cluster->get_num_mcells() << " " << map_cluster_length[main_cluster]/units::cm << std::endl;
  // for  (auto it = other_clusters.begin(); it!=other_clusters.end(); it++){
  //  std::cout << (*it)->get_num_mcells() << " " << map_cluster_length[*it]/units::cm << std::endl;
  // }
  // hack switch 
  // {
  //   auto it = find(other_clusters.begin(), other_clusters.end(), max_length_cluster);
  //   other_clusters.erase(it);
  //   other_clusters.push_back(main_cluster);
  //   main_cluster = max_length_cluster;
  // }

  //std::cout << "haha " << " " << map_cluster_length[main_cluster]/units::cm << std::endl;
  
    if (map_cluster_length[main_cluster] < 3*units::cm) return;


  
  
    if (flag_main_cluster){
      
      // find the proto vertex ...
      find_proto_vertex(main_cluster, true, 2);

    
      // deal with shower ...
      clustering_points(main_cluster);
      separate_track_shower(main_cluster);

      determine_direction(main_cluster);

      shower_determing_in_main_cluster(main_cluster);
      determine_main_vertex(main_cluster);
      //improve_vertex(main_cluster, false); // do not search for vertex activities ...

   
      if (main_vertex !=0){
        map_cluster_main_vertices[main_cluster] = main_vertex;
        main_vertex = 0;
      }

      print_segs_info(main_cluster->get_cluster_id(), main_vertex);

    
    }

  
    // loop over other clusters ...
    if (flag_other_clusters){
      for (auto it = other_clusters.begin(); it!=other_clusters.end(); it++){
        //if (skip_clusters.find(*it) != skip_clusters.end()) continue;
        //if ((*it)->get_cluster_id()!=75) continue;
        //std::cout << (*it)->get_cluster_id() << " " << map_cluster_length[*it]/units::cm << std::endl;
      
        if (map_cluster_length[*it] > 6*units::cm){
        //if (map_cluster_length[*it] > 4*units::cm){
	      // find the proto vertex ...
	        find_proto_vertex(*it, true, 2);
	        // deal with shower ...
	        clustering_points(*it);
          separate_track_shower(*it);
          determine_direction(*it);    
          shower_determing_in_main_cluster(*it);
          determine_main_vertex(*it, false);
          //improve_vertex(main_cluster, false); // do not search for vertex activity
          
          if (main_vertex !=0){
            map_cluster_main_vertices[*it] = main_vertex;
            main_vertex = 0;
          }
       }else{
        if (!find_proto_vertex(*it, false, 1)) init_point_segment(*it);

        clustering_points(*it);
        separate_track_shower(*it);
        determine_direction(*it);
        shower_determing_in_main_cluster(*it);
        determine_main_vertex(*it, false);
        if (main_vertex !=0){
          map_cluster_main_vertices[*it] = main_vertex;
          main_vertex = 0;
        }
      }
    
    }
      //  deghost ...
    deghosting();
  }
 


  if (flag_main_cluster){
    if (flag_dl_vtx){
      bool flag_change = determine_overall_main_vertex_DL();
      if (!flag_change) determine_overall_main_vertex();
    }else{
      determine_overall_main_vertex();
    }
  }

  
  
    if (flag_main_cluster && main_vertex !=0){
      
      
      // fit the vertex in 3D 
      improve_vertex(main_cluster, true, true);

      std::cout << "After improve_vertex " << std::endl;
      print_segs_info(main_vertex->get_cluster_id(), main_vertex);

    
      clustering_points(main_cluster);
      examine_direction(main_vertex);
      
      std::cout << "After examine_direction " << std::endl;
      print_segs_info(main_vertex->get_cluster_id(), main_vertex);
      
      std::cout << "Overall main Vertex " << main_vertex->get_fit_pt() << " connecting to: ";
      for (auto it = map_vertex_segments[main_vertex].begin(); it!=map_vertex_segments[main_vertex].end(); it++){
        std::cout << (*it)->get_id() << ", ";
      }
      std::cout << " in cluster " << main_vertex->get_cluster_id() << std::endl;
      print_segs_info(main_vertex->get_cluster_id(), main_vertex);
      
      
      // overall
      separate_track_shower();
      // for charge based on calculation ...
      collect_2D_charges();


      
      // cluster E&M ...
      shower_clustering_with_nv();

      
    }

    if (flag_tagger){

      fill_kine_tree(kine_info);
      
      bool flag_cosmic = cosmic_tagger();


      // set the cosmic flag anyway ...
      // if (flag_cosmic) tagger_info.cosmic_flag = false;
      
      auto results = numu_tagger();
      bool flag_long_muon = results.first;

      bool flag_ssm = ssm_tagger(); 
      if(flag_ssm) std::cout<<"SSM Spotted"<<std::endl;
      
      nue_tagger(results.second);

      bool flag_sp = singlephoton_tagger(results.second);
      std::cout<<"NeutrinoID.cxx line 241"<<std::endl;
      if (flag_sp){tagger_info.photon_flag = true;}

      
    }

    if (flag_bdt >0)
        tagger_info.numu_score = cal_numu_bdts_xgboost();

    
    if (flag_bdt == 1){
      // Xgboost training ...
      tagger_info.nue_score = cal_bdts_xgboost();
          
    }else if (flag_bdt == 2){
      // TMVA training ...
      tagger_info.nue_score = cal_bdts();
      // numuCC TMVA training ...
    }
  }else{
      
    if (flag_main_cluster){
      if (map_cluster_length[main_cluster] > 3*units::cm){
        // find the proto vertex ...
       find_proto_vertex(main_cluster, true, 2);
      }else{
        if (!find_proto_vertex(main_cluster, false, 1)) init_point_segment(main_cluster);
      }
    }

  
    // loop over other clusters ...
     if (flag_other_clusters){ 
      for (auto it = other_clusters.begin(); it!=other_clusters.end(); it++){
        if (map_cluster_length[*it] > 6*units::cm){
	        find_proto_vertex(*it, true, 2);
        }else{
          if (!find_proto_vertex(*it, false, 1)) init_point_segment(*it);
        }

      }
      //  deghost ...
      deghosting();
    }

  }


  
  //std::cout << "Final Information: " << std::endl;
  //print_segs_info(main_vertex);

  // for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
  //   auto pair_vertices = find_vertices(it->first);
  //   if (pair_vertices.first->get_cluster_id() != pair_vertices.second->get_cluster_id())
  //     std::cout << pair_vertices.first->get_cluster_id() << " " << pair_vertices.second->get_cluster_id() << std::endl;
  // }
  
  // prepare output ...
  fill_fit_parameters();
}


void WCPPID::NeutrinoID::determine_overall_main_vertex(){
  const bool flag_timing = true;
  using DOMV_Clock = std::chrono::high_resolution_clock;
  using DOMV_ms = std::chrono::duration<double, std::milli>;
  auto domv_t_total = DOMV_Clock::now();
  auto domv_t0 = DOMV_Clock::now();
  std::cout << "[determine_overall_main_vertex] Start -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;

  PR3DCluster* max_length_cluster = 0;
  double max_length = 0;
  for (auto it = map_cluster_length.begin(); it!= map_cluster_length.end(); it++){
    double length = it->second;
    if (length > max_length){
      max_length = length;
      max_length_cluster = it->first;
    }
  }

  // check main vertices ...
  domv_t0 = DOMV_Clock::now();
  examine_main_vertices();
  if (flag_timing) std::cout << "[determine_overall_main_vertex] timing: examine_main_vertices took " << std::chrono::duration_cast<DOMV_ms>(DOMV_Clock::now()-domv_t0).count() << " ms" << std::endl;

  domv_t0 = DOMV_Clock::now();
  if (flag_neutrino_id_process==1){
    // development chain ...
    check_switch_main_cluster();
  }else{
    // frozen chain
    auto it_frozen = map_cluster_main_vertices.find(main_cluster);
    if (max_length > map_cluster_length[main_cluster] * 0.8 && it_frozen != map_cluster_main_vertices.end())
      check_switch_main_cluster(it_frozen->second, max_length_cluster);
  }
  if (flag_timing) std::cout << "[determine_overall_main_vertex] timing: check_switch_main_cluster took " << std::chrono::duration_cast<DOMV_ms>(DOMV_Clock::now()-domv_t0).count() << " ms" << std::endl;

  auto it_mv = map_cluster_main_vertices.find(main_cluster);
  main_vertex = (it_mv != map_cluster_main_vertices.end()) ? it_mv->second : nullptr;

  if (main_vertex == 0){
    std::cout << "[determine_overall_main_vertex] No main vertex found for main_cluster, returning." << std::endl;
    return;
  }

  // examine the track connected to it ...
  for (auto it = map_vertex_segments[main_vertex].begin(); it!= map_vertex_segments[main_vertex].end();it++){
    WCPPID::ProtoSegment *sg = *it;
    auto pair_results = calculate_num_daughter_showers(main_vertex, sg, false);
    if (pair_results.first==1 && sg->get_length() < 1.5*units::cm && sg->get_medium_dQ_dx()/(43e3/units::cm) > 1.6){
      TPCParams& mp = Singleton<TPCParams>::Instance();
      sg->set_particle_type(2212);
      sg->set_particle_mass(mp.get_mass_proton());
      sg->cal_4mom();
      //    std::cout << pair_results.first << " " << sg->get_length()/units::cm << " " << sg->get_medium_dQ_dx()/(43e3/units::cm) << std::endl;
      //    if (sg->get_length()<1*units::cm && pair_results.
    }
  }

  
  
  
  // clean up long muons ...
  {
    std::set<WCPPID::ProtoSegment* > tmp_segments;
    std::set<WCPPID::ProtoVertex* > tmp_vertices;
    for (auto it = segments_in_long_muon.begin(); it!= segments_in_long_muon.end(); it++){
      if ((*it)->get_cluster_id() != main_vertex->get_cluster_id()) tmp_segments.insert(*it);
    }
    for (auto it = vertices_in_long_muon.begin(); it!= vertices_in_long_muon.end(); it++){
      if ((*it)->get_cluster_id() != main_vertex->get_cluster_id()) tmp_vertices.insert(*it);
    }
    for (auto it = tmp_segments.begin(); it!=tmp_segments.end();it++){
      segments_in_long_muon.erase(*it);
    }
    for (auto it = tmp_vertices.begin(); it!=tmp_vertices.end(); it++){
      vertices_in_long_muon.erase(*it);
    }
  }

  std::cout << "[determine_overall_main_vertex] Done -- total took " << std::chrono::duration_cast<DOMV_ms>(DOMV_Clock::now()-domv_t_total).count() << " ms" << std::endl;
}



void WCPPID::NeutrinoID::examine_main_vertices(){
 

  double cluster_length_cut = std::min(map_cluster_length[main_cluster]*0.6, 6*units::cm);
  WCP::ToyPointCloud* pcloud = main_cluster->get_point_cloud_steiner();
    
  WCPPID::PR3DClusterSelection clusters_to_be_removed;
  for (auto it = map_cluster_main_vertices.begin(); it!= map_cluster_main_vertices.end(); it++){
    WCPPID::PR3DCluster* cluster = it->first;
    WCPPID::ProtoVertex* vertex = it->second;
    double length = map_cluster_length[cluster];
    //    std::cout << cluster->get_cluster_id() << " " << length/units::cm << std::endl;
    if (length < cluster_length_cut){
      bool flag_removed = true;
      for (auto it1 = map_vertex_segments[vertex].begin(); it1 != map_vertex_segments[vertex].end(); it1++){
	WCPPID::ProtoSegment *seg = *it1;
	//	std::cout << seg->get_cluster_id() << " " << seg->get_id() << " " << seg->get_flag_shower() << " " << seg->get_flag_dir() << " " << seg->is_dir_weak() << " " << seg->get_particle_type() << " " << length/units::cm << " " << seg->get_medium_dQ_dx()/(43e3/units::cm) << std::endl;
	if ((!seg->get_flag_shower()) && seg->get_flag_dir()!=0 && ((!seg->is_dir_weak()) || seg->get_medium_dQ_dx()/(43e3/units::cm)> 2 ) ){
	 
	  flag_removed = false;
	  break;
	}
      }
      if (!flag_removed){
	if (length < 5*units::cm && pcloud->get_closest_dis(vertex->get_fit_pt()) > 100*units::cm)
	  flag_removed = true;

	//std::cout << cluster->get_cluster_id() << " " << length/units::cm << " " << pcloud->get_closest_dis(vertex->get_fit_pt()) << std::endl;
      }
      if (flag_removed)
	clusters_to_be_removed.push_back(cluster);
    }else{
      if (pcloud->get_closest_dis(vertex->get_fit_pt()) > 200*units::cm)
	clusters_to_be_removed.push_back(cluster);

      //   std::cout << cluster->get_cluster_id() << " " << pcloud->get_closest_dis(vertex->get_fit_pt())/units::cm << std::endl;
    }
  }
  for (auto it = clusters_to_be_removed.begin(); it!= clusters_to_be_removed.end(); it++){
    map_cluster_main_vertices.erase(*it);
  }
  clusters_to_be_removed.clear();

  // additional cut to remove global main vertex candidates ...
  if (map_cluster_main_vertices.find(main_cluster) != map_cluster_main_vertices.end() ){
    std::map<int, bool> map_cluster_id_shower;
    for (auto it = map_cluster_main_vertices.begin(); it != map_cluster_main_vertices.end(); it++){
      map_cluster_id_shower[it->first->get_cluster_id()] = true;
    }

    bool flag_main_vertex_all_showers = true;
    for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
      //      if (it->first->get_cluster_id() != main_cluster->get_cluster_id()) continue;
      WCPPID::ProtoSegment *sg = it->first;
      if (map_cluster_id_shower.find(sg->get_cluster_id())  == map_cluster_id_shower.end()) continue; 
      if (!sg->get_flag_shower()){
	map_cluster_id_shower[sg->get_cluster_id()] = false;
	//flag_main_vertex_all_showers = false;
      }
    }
    flag_main_vertex_all_showers = map_cluster_id_shower[main_cluster->get_cluster_id()];

    //    std::cout << main_cluster->get_cluster_id() << " " << flag_main_vertex_all_showers << std::endl;
    if (flag_main_vertex_all_showers){
      TVector3 dir_main = calc_dir_cluster(main_cluster->get_cluster_id(), map_cluster_main_vertices[main_cluster]->get_fit_pt(), 15*units::cm);
      ToyPointCloud *pcloud = main_cluster->get_point_cloud_steiner();
      for (auto it = map_cluster_main_vertices.begin(); it != map_cluster_main_vertices.end(); it++){
	if (it->first == main_cluster) continue;
	double closest_dis = pcloud->get_closest_dis(it->second->get_fit_pt());
	TVector3 dir1(it->second->get_fit_pt().x - map_cluster_main_vertices[main_cluster]->get_fit_pt().x,
		     it->second->get_fit_pt().y - map_cluster_main_vertices[main_cluster]->get_fit_pt().y,
		     it->second->get_fit_pt().z - map_cluster_main_vertices[main_cluster]->get_fit_pt().z);
	double angle = dir_main.Angle(dir1)/3.14115926*180. ;

	//std::cout << it->first->get_cluster_id() << " " << closest_dis/units::cm << " " << dir_main.Angle(dir1)/3.14115926*180. << " " << map_cluster_length[it->first]/units::cm << std::endl;

	if (angle < 10){
	  if (map_cluster_length[it->first] < 15*units::cm && closest_dis < 40*units::cm){
	    clusters_to_be_removed.push_back(it->first);
	  }else if (map_cluster_length[it->first] < 7*units::cm && closest_dis < 60*units::cm){
	    clusters_to_be_removed.push_back(it->first);
	  }
	}else if (angle > 160){
	  if (map_cluster_id_shower[it->first->get_cluster_id()] && map_cluster_length[it->first] > 10*units::cm && map_cluster_length[it->first] > 0.5 * map_cluster_length[main_cluster]){
	    TVector3 dir2 = calc_dir_cluster(it->first->get_cluster_id(), it->second->get_fit_pt(), 15*units::cm);
	    double closest_dis = std::get<2>(it->first->get_point_cloud()->get_closest_points(main_cluster->get_point_cloud()));
	    double angle = dir2.Angle(dir_main)/3.1415926*180.;
	    if (closest_dis < 10*units::cm && angle < 25){
	      //	      clusters_to_be_removed.push_back(main_cluster);
	      swap_main_cluster(it->first);
	    }
	    //	    std::cout << dir2.Angle(dir_main)/3.1415926*180. << " " << closest_dis/units::cm << std::endl;
	  }
	}
      }
    }

    for (auto it = clusters_to_be_removed.begin(); it!= clusters_to_be_removed.end(); it++){
      map_cluster_main_vertices.erase(*it);
    }
    clusters_to_be_removed.clear();

  }
  
  // for (auto it = map_cluster_main_vertices.begin(); it!= map_cluster_main_vertices.end(); it++){
  //   WCPPID::PR3DCluster* cluster = it->first;
  //   WCPPID::ProtoVertex* vertex = it->second;
  //   double length = map_cluster_length[cluster];
    //    std::cout << cluster->get_cluster_id() << " " << length/units::cm << " " << vertex->get_fit_pt() << std::endl;
  // }
}


void WCPPID::NeutrinoID::examine_main_vertices(WCPPID::ProtoVertexSelection& vertices){
  
  
  if (vertices.size()==1) return;

  double max_length = 0;
  TPCParams& mp = Singleton<TPCParams>::Instance();
  std::set<WCPPID::ProtoVertex*> tmp_vertices; 


  for (auto it = vertices.begin(); it != vertices.end(); it++){
    WCPPID::ProtoVertex* vtx = *it;
    if (map_vertex_segments[vtx].size()==1){
      tmp_vertices.insert(vtx);
    }else{
      std::set<WCPPID::ProtoSegment*> used_segments;
      for (auto it1 = map_vertex_segments[vtx].begin(); it1 != map_vertex_segments[vtx].end(); it1++){
	WCPPID::ProtoSegment *sg1 = *it1;
	if (sg1->get_length() < 10*units::cm) continue;
	TVector3 dir1 = sg1->cal_dir_3vector(vtx->get_fit_pt(), 15*units::cm);
	TVector3 dir3 = sg1->cal_dir_3vector(vtx->get_fit_pt(), 30*units::cm);
	if (sg1->get_length() > max_length) max_length = sg1->get_length();
	for (auto it2 = it1; it2 != map_vertex_segments[vtx].end(); it2++){
	  WCPPID::ProtoSegment *sg2 = *it2;
	  if (sg1 == sg2) continue;
	  if (sg2->get_length()<10*units::cm) continue;
	  TVector3 dir2 = sg2->cal_dir_3vector(vtx->get_fit_pt(), 15*units::cm);
	  TVector3 dir4 = sg2->cal_dir_3vector(vtx->get_fit_pt(), 30*units::cm);

	  // std::cout << sg1->get_length()/units::cm << " " << sg2->get_length()/units::cm << " " << dir1.Angle(dir2)/3.1415926*180. << " " << dir3.Angle(dir4)/3.1415926*180. << " " << sg1->get_particle_type() << " " << sg2->get_particle_type() << std::endl;
	  
	  if ( (dir1.Angle(dir2)/3.1415926*180. > 165 || dir3.Angle(dir4)/3.1415926*180. > 165)&& (sg1->get_particle_type()==13 || sg2->get_particle_type()==13 ) && (sg1->get_length() > 30*units::cm|| sg2->get_length() > 30*units::cm )){

	    // std::cout << "Xin: " << sg1->get_id() << " " << sg1->get_particle_type() << " " << sg1->get_length()/units::cm << std::endl;
	    // std::cout << "Xin: " << sg2->get_id() << " " << sg2->get_particle_type() << " " << sg2->get_length()/units::cm << std::endl;
	    
	    used_segments.insert(sg1);
	    used_segments.insert(sg2);
	  }else if ((dir1.Angle(dir2)/3.1415926*180. > 170 || dir3.Angle(dir4)/3.1415926*180. > 170)&& (sg1->get_particle_type()==2212 && (sg2->get_particle_type()==0 || sg2->get_particle_type()==2212) || sg2->get_particle_type()==2212 && (sg1->get_particle_type()==0 || sg1->get_particle_type()==2212)) && (sg1->get_length() > 20*units::cm &&  sg2->get_length() > 20*units::cm )){
	    // std::cout << "Xin: " << sg1->get_id() << " " << sg1->get_particle_type() << " " << sg1->get_length()/units::cm << std::endl;
	    // std::cout << "Xin: " << sg2->get_id() << " " << sg2->get_particle_type() << " " << sg2->get_length()/units::cm << std::endl;
	    
	    used_segments.insert(sg1);
	    used_segments.insert(sg2);
	  }

	  
	} // loop second track
      } // loop first track

      if (used_segments.size()>0){
	bool flag_skip = true;
	for (auto it1 = map_vertex_segments[vtx].begin(); it1 != map_vertex_segments[vtx].end(); it1++){
	  WCPPID::ProtoSegment *sg1 = *it1;
	  if (used_segments.find(sg1) != used_segments.end()) continue;
	  double length = sg1->get_length();
	  if (sg1->get_flag_shower()){ // shower
	    auto pair_result = calculate_num_daughter_showers(vtx, sg1, false); // count all
	    if (pair_result.second > 35*units::cm){
	      flag_skip = false;
	      break;
	    }
	  }else{ // not shower 
	    if ((!sg1->is_dir_weak()) && length > 6*units::cm){
	      flag_skip = false;
	      break;
	    }
	  }
	  //std::cout << sg1->get_id() << " " << sg1->get_length()/units::cm << " " << sg1->is_dir_weak() << " " << sg1->get_particle_type() << std::endl;
	}
	if (!flag_skip) tmp_vertices.insert(vtx);
	else{
	  std::cout << "[examine_main_vertices] Cluster: " << vtx->get_cluster_id() << " Remove back-to-back vertex id=" << vtx->get_id() << " at " << vtx->get_fit_pt() << " from candidates -- # of Vertices: " << map_vertex_segments.size() << "; # of Segments: " << map_segment_vertices.size() << std::endl;
	  // change direction ...
	  for (auto it1 = used_segments.begin(); it1!=used_segments.end(); it1++){
	    WCPPID::ProtoSegment *sg1 = *it1;
	    // std::cout << sg1->get_id() << " " << sg1->get_length()/units::cm << std::endl;
	    if ( sg1->get_flag_shower_trajectory()) continue;
	    else if (sg1->get_flag_shower_topology()){
	      if (sg1->get_length() > 40*units::cm && sg1->get_flag_dir()==0){
		sg1->set_particle_type(13);
		sg1->set_particle_mass(mp.get_mass_muon());
		change_daughter_type(vtx, sg1, 13, mp.get_mass_muon());
		change_daughter_type(find_other_vertex(sg1, vtx), sg1, 13, mp.get_mass_muon());
		sg1->set_flag_shower_topology(false);
	      }else continue;
	    }else if (sg1->get_particle_type()!=13){
	      sg1->set_particle_type(13);
	      sg1->set_particle_mass(mp.get_mass_muon());
	      change_daughter_type(vtx, sg1, 13, mp.get_mass_muon());
	      change_daughter_type(find_other_vertex(sg1, vtx), sg1, 13, mp.get_mass_muon());
	    }

	    
	    std::vector<WCPPID::ProtoVertex*> acc_vertices;
	    auto results = find_cont_muon_segment(sg1, vtx);
	    while(results.first !=0){
	      acc_vertices.push_back(results.second);
	      results = find_cont_muon_segment(results.first, results.second);
	    }
	    if (acc_vertices.size()>0 && acc_vertices.back()!=0)
	      tmp_vertices.insert(acc_vertices.back());
	    
	  }
	}
      }else{
	tmp_vertices.insert(vtx);
      }
    } // else
  } // loop vertices ...

  //print_segs_info(vertices.front()->get_cluster_id());
  
  
  if (tmp_vertices.size()==0) return;
  std::cout << "[examine_main_vertices] Cluster: " << vertices.front()->get_cluster_id() << " Candidates: " << vertices.size() << " -> " << tmp_vertices.size() << std::endl;
  vertices.clear();
  vertices.resize(tmp_vertices.size());
  std::copy(tmp_vertices.begin(), tmp_vertices.end(), vertices.begin());



  /*
  if (max_length > 40*units::cm){
    if (vertices.size()==1) return;
    
    tmp_vertices.clear();
    for (auto it = vertices.begin(); it!= vertices.end(); it++){
      WCPPID::ProtoVertex *vtx = *it;
      if (map_vertex_segments[vtx].size()>1){
	tmp_vertices.push_back(vtx);
      }else{
	WCPPID::ProtoSegment *sg = (*map_vertex_segments[vtx].begin());
	WCPPID::ProtoVertex *other_vtx = find_other_vertex(sg, vtx);
	//      std::cout << sg->get_length()/units::cm << std::endl;
	TVector3 dir1 = sg->cal_dir_3vector(other_vtx->get_fit_pt(), 15*units::cm);
	double max_angle = 0;
	double length = sg->get_length();
	if (length < 1*units::cm){
	}else if (length < 5*units::cm){
	  for (auto it1 = map_vertex_segments[other_vtx].begin(); it1 != map_vertex_segments[other_vtx].end(); it1++){
	    WCPPID::ProtoSegment *sg1 = *it1;
	    if (sg1 == sg) continue;
	    TVector3 dir2 = sg1->cal_dir_3vector(other_vtx->get_fit_pt(), 15*units::cm);
	    double angle = dir1.Angle(dir2)/3.1415926*180.;
	    if (angle > max_angle) max_angle = angle;
	  }
	  if (max_angle > 155) tmp_vertices.push_back(vtx);
	  //	std::cout << max_angle << " " << sg->get_length()/units::cm << std::endl;
	}else{
	  tmp_vertices.push_back(vtx);
	}
      }
    }
    
    if (tmp_vertices.size() == 0) return;
    vertices = tmp_vertices;
  }
  */
}


TVector3 WCPPID::NeutrinoID::calc_dir_cluster(int tmp_cluster_id, WCP::Point& orig_p, double dis_cut){
  Point ave_p(0,0,0);
  int num = 0;

  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != tmp_cluster_id) continue;
    WCP::PointVector& pts = sg->get_point_vec();
    for (size_t i=1;i+1<pts.size();i++){
      double dis = sqrt(pow(pts.at(i).x - orig_p.x,2) + pow(pts.at(i).y - orig_p.y,2) + pow(pts.at(i).z - orig_p.z,2));
      if (dis < dis_cut){
	ave_p.x += pts.at(i).x;
	ave_p.y += pts.at(i).y;
	ave_p.z += pts.at(i).z;
	num ++;
      }
    }
  }
  for (auto it = map_vertex_segments.begin(); it != map_vertex_segments.end(); it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() != tmp_cluster_id) continue;
    double dis = sqrt(pow(vtx->get_fit_pt().x - orig_p.x,2) + pow(vtx->get_fit_pt().y - orig_p.y,2) + pow(vtx->get_fit_pt().z - orig_p.z,2));
    if (dis < dis_cut){
      ave_p.x += vtx->get_fit_pt().x;
      ave_p.y += vtx->get_fit_pt().y;
      ave_p.z += vtx->get_fit_pt().z;
      num ++;
    }
  }

  TVector3 dir(0,0,0);
  
  if (num >0){
    ave_p.x /= num;
    ave_p.y /= num;
    ave_p.z /= num;

    dir.SetXYZ(ave_p.x - orig_p.x, ave_p.y - orig_p.y, ave_p.z - orig_p.z);
    dir = dir.Unit();
  }
  return dir;
}


void WCPPID::NeutrinoID::swap_main_cluster(WCPPID::PR3DCluster *new_main_cluster){
  other_clusters.push_back(main_cluster);
  main_cluster = new_main_cluster;
  auto it1 = find(other_clusters.begin(), other_clusters.end(), main_cluster);
  other_clusters.erase(it1);
}


void WCPPID::NeutrinoID::check_switch_main_cluster(){
  bool flag_switch = false;
  bool flag_all_showers = false;

  bool flag_print = true;

 
  WCPPID::ProtoVertex *temp_main_vertex = 0;

  if (map_cluster_main_vertices.find(main_cluster) != map_cluster_main_vertices.end()){
    temp_main_vertex = map_cluster_main_vertices[main_cluster];
    int n_showers = 0;
    
    //    std::cout << main_cluster->get_cluster_id() << " " << main_cluster << " " << temp_main_vertex << std::endl;
  
    for (auto it = map_vertex_segments[temp_main_vertex].begin(); it!= map_vertex_segments[temp_main_vertex].end(); it++){
      if ((*it)->get_flag_shower()) n_showers ++;
    }
    if (n_showers == map_vertex_segments[temp_main_vertex].size()) flag_all_showers = true;
  }else{
    flag_all_showers = true;
  }
  
  if (flag_all_showers){
    WCPPID::ProtoVertexSelection vertex_candidates;
    for (auto it = map_cluster_main_vertices.begin(); it!= map_cluster_main_vertices.end(); it++){
      vertex_candidates.push_back(it->second);
    }

    if (flag_print){
      for (auto it = vertex_candidates.begin(); it!= vertex_candidates.end(); it++){
	std::cout << "[check_switch_main_cluster] Candidate main vertex " << (*it)->get_cluster_id() << " " << (*it)->get_fit_pt() << " connecting to: ";
	for (auto it1 = map_vertex_segments[*it].begin(); it1!=map_vertex_segments[*it].end(); it1++){
	  std::cout << (*it1)->get_id() << ", ";
	}
	std::cout << std::endl;
      }
    }
    
    WCPPID::ProtoVertex *temp_main_vertex_1 = compare_main_vertices_global(vertex_candidates);
    if (temp_main_vertex_1 != temp_main_vertex){
      if (temp_main_vertex != 0){
	std::cout << "[check_switch_main_cluster] Switch Main Cluster " << temp_main_vertex->get_cluster_id() << " to " << temp_main_vertex_1->get_cluster_id() << std::endl;
      }else{
	std::cout << "[check_switch_main_cluster] Switch Main Cluster  to " << temp_main_vertex_1->get_cluster_id() << std::endl;
      }
      for (auto it = map_cluster_main_vertices.begin(); it!= map_cluster_main_vertices.end(); it++){
	if (it->second == temp_main_vertex_1){
	  swap_main_cluster(it->first);
	  break;
	}
      }
    } // switch
  } // all showers
}



WCPPID::ProtoVertex* WCPPID::NeutrinoID::compare_main_vertices_global(WCPPID::ProtoVertexSelection& vertex_candidates){
  bool flag_print = false;


  std::map<WCPPID::ProtoVertex*, double> map_vertex_num;
  for (auto it = vertex_candidates.begin(); it!=vertex_candidates.end(); it++){
    map_vertex_num[*it] = 0;
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
      // std::cout << sg->get_id() << " " << sg->get_flag_shower() << " " << sg->get_particle_type() << " " << sg->get_flag_dir() << " " << sg->is_dir_weak() << " " << sg->get_length()/units::cm << std::endl;
      if (sg->get_flag_shower()){
	map_vertex_num[vtx] += 1/4./2.; // number of showers
      }else{
	map_vertex_num[vtx] += 1/4.; // number of tracks
      }
      if (sg->get_particle_type()==2212 && sg->get_flag_dir()!=0 && (!sg->is_dir_weak()))
	map_vertex_num[vtx] += 1/4.; // has a clear proton ...
      else if (sg->get_flag_dir()!=0 && (!sg->get_flag_shower()))
	map_vertex_num[vtx] += 1/4./2.; // has a direction with track ..
    }
    // add to main vertex ...
    if (vtx->get_cluster_id() == main_cluster->get_cluster_id()) map_vertex_num[vtx] += 0.25;
    if (flag_print)
      std::cout << "A: " << map_vertex_num[vtx] << " " << (vtx->get_fit_pt().z - min_z)/(200*units::cm) << " " << map_vertex_segments[vtx].size()/4. << " " << std::endl;
  }
  
  // whether the vetex is at boundary or not ...
  for (auto it = vertex_candidates.begin(); it!=vertex_candidates.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    if (fid->inside_fiducial_volume(vtx->get_fit_pt(),offset_x) || vtx->get_cluster_id() == main_cluster->get_cluster_id())  map_vertex_num[vtx] +=0.5; // good      // fiducial volume ..
    if (flag_print) std::cout << "B: " << map_vertex_num[vtx] << " " << fid->inside_fiducial_volume(vtx->get_fit_pt(),offset_x) << std::endl;
  }

  //  for (auto it = vertex_candidates.begin(); it != vertex_candidates.end(); it++){
  // WCPPID::ProtoVertex *vtx = *it;

  //double num_conflicts = calc_conflict_maps(vtx);
  //  map_vertex_num[vtx] -= num_conflicts/4.;
    //std::cout << "C: " << map_vertex_num[vtx] << " " << num_conflicts << " " << map_vertex_num[vtx] << std::endl;
  //}


  // Now compare with all other vertices ...
  std::map<WCPPID::ProtoVertex*, TVector3> map_vertex_dir;
  // for each vertices, calculate a direction first ...
  for (auto it = vertex_candidates.begin(); it != vertex_candidates.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    map_vertex_dir[vtx] = get_dir(vtx);
  }
  
  for (auto it = vertex_candidates.begin(); it!= vertex_candidates.end(); it++){
    WCPPID::ProtoVertex *vtx = *it;
    double delta = 0;
    for (auto it1 = vertex_candidates.begin(); it1!=vertex_candidates.end(); it1++){
      WCPPID::ProtoVertex *vtx1 = *it1;
      if (vtx1 == vtx) continue;
      TVector3 dir(vtx1->get_fit_pt().x -vtx->get_fit_pt().x, vtx1->get_fit_pt().y -vtx->get_fit_pt().y, vtx1->get_fit_pt().z -vtx->get_fit_pt().z);
      TVector3 dir1 = map_vertex_dir[vtx1];
      if (dir.Angle(dir1)/3.1415926*180. < 15){
	map_vertex_num[vtx] += 0.25;
	delta ++;
      }else if (dir.Angle(dir1)/3.1415926*180.< 30){
	map_vertex_num[vtx] += 0.25/2.;
	delta ++;
      }
      // if (flag_print) std::cout << "D: " << vtx->get_cluster_id() << " " << map_vertex_num[vtx] << " " << dir.Angle(dir1)/3.1415926*180. << " " << main_cluster->get_cluster_id() << " " << dir.Mag()/units::cm << std::endl;
    }
    
    // nothing point to this one ...
    if (delta == 0){
      double total_length = 0;
      int num_tracks = 0;
      for (auto it1 = map_segment_vertices.begin(); it1!= map_segment_vertices.end(); it1++){
	if (it1->first->get_cluster_id() != vtx->get_cluster_id()) continue;
	total_length += it1->first->get_length();
	num_tracks ++;
	//std::cout << delta << std::endl;
      }
      if (vtx->get_cluster_id() != main_cluster->get_cluster_id() &&  total_length < 6*units::cm){
	map_vertex_num[vtx] -= 0.25 * num_tracks;
      }
   
    }
    if (flag_print) std::cout << "E: " << map_vertex_num[vtx] << " "  << std::endl;
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
  
  return max_vertex;
}

TVector3 WCPPID::NeutrinoID::get_dir(WCPPID::ProtoVertex *vtx, double dis_cut){
  Point center(0,0,0);
  int ncount = 0;

  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != vtx->get_cluster_id()) continue;
    PointVector& pts = sg->get_point_vec();
    for (size_t i=1;i+1<pts.size();i++){
      double dis = sqrt(pow(pts.at(i).x - vtx->get_fit_pt().x,2)+pow(pts.at(i).y - vtx->get_fit_pt().y,2)+pow(pts.at(i).z - vtx->get_fit_pt().z,2));
      if (dis < dis_cut){
	center.x += pts.at(i).x;
	center.y += pts.at(i).y;
	center.z += pts.at(i).z;
	ncount ++;
      }
    }
  }
  for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
    WCPPID::ProtoVertex *vertex = it->first;
    if (vertex->get_cluster_id() != vtx->get_cluster_id()) continue;
    double dis = sqrt(pow(vertex->get_fit_pt().x - vtx->get_fit_pt().x,2)+pow(vertex->get_fit_pt().y - vtx->get_fit_pt().y,2)+pow(vertex->get_fit_pt().z - vtx->get_fit_pt().z,2));
    if (dis < dis_cut){
      center.x += vertex->get_fit_pt().x;
      center.y += vertex->get_fit_pt().y;
      center.z += vertex->get_fit_pt().z;
      ncount ++;
    }
  }

  center.x /= ncount;
  center.y /= ncount;
  center.z /= ncount;

  TVector3 dir(center.x - vtx->get_fit_pt().x,
	      center.y - vtx->get_fit_pt().y,
	      center.z - vtx->get_fit_pt().z);
  dir = dir.Unit();
  return dir;
}



void WCPPID::NeutrinoID::check_switch_main_cluster(WCPPID::ProtoVertex *temp_main_vertex, WCPPID::PR3DCluster *max_length_cluster ){
  
  bool flag_switch = false;

  int n_showers = 0;
  for (auto it = map_vertex_segments[temp_main_vertex].begin(); it!= map_vertex_segments[temp_main_vertex].end(); it++){
    if ((*it)->get_flag_shower()) n_showers ++;
  }
  if (n_showers == map_vertex_segments[temp_main_vertex].size()) flag_switch = true;

  if (flag_switch){
    std::cout << "[check_switch_main_cluster] Switch Main Cluster " << main_cluster->get_cluster_id() << " to " << max_length_cluster->get_cluster_id() << std::endl;
    swap_main_cluster(max_length_cluster);
  }
}
  

std::pair<int, int> WCPPID::NeutrinoID::count_num_tracks_showers(WCPPID::PR3DCluster* temp_cluster){
  int num[2] = {0,0};
  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_flag_shower()) num[1] ++;
    else num[0] ++;
  }
  return std::make_pair(num[0], num[1]);
}



void WCPPID::NeutrinoID::fill_fit_parameters(){
  organize_vertices_segments();
  std::set<WCPPID::PR3DCluster*> clusters_set;
  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    WCPPID::PR3DCluster *cluster = map_id_cluster[sg->get_cluster_id()];
    clusters_set.insert(cluster);
  }
  for (auto it = clusters_set.begin(); it!=clusters_set.end(); it++){
    (*it)->set_fit_parameters(proto_vertices, proto_segments, map_vertex_segments, map_segment_vertices);
  }

  PR3DClusterSelection all_clusters = other_clusters;
  all_clusters.push_back(main_cluster);
  for (auto it = all_clusters.begin(); it!=all_clusters.end(); it++){
    WCPPID::PR3DCluster* temp_cluster = *it;
    std::vector<int>& point_sub_cluster_ids = temp_cluster->get_point_sub_cluster_ids();
    std::vector<bool>& point_flag_showers = temp_cluster->get_point_flag_showers();
    if (point_flag_showers.size()==0)
      point_flag_showers.resize(point_sub_cluster_ids.size(), false);
  }
}

void WCPPID::NeutrinoID::clustering_points(WCPPID::PR3DCluster* temp_cluster){
  const bool flag_timing = true;
  using CP_Clock = std::chrono::high_resolution_clock;
  using CP_ms = std::chrono::duration<double, std::milli>;
  auto cp_t_total = CP_Clock::now();
  auto cp_t0 = CP_Clock::now();

  temp_cluster->clustering_points_master(map_vertex_segments, map_segment_vertices, *ct_point_cloud);
  if (flag_timing) std::cout << "clustering_points timing: clustering_points_master took " << std::chrono::duration_cast<CP_ms>(CP_Clock::now()-cp_t0).count() << " ms" << std::endl;

  cp_t0 = CP_Clock::now();
  std::map<int, WCPPID::ProtoSegment*> map_id_seg;
  std::map<WCPPID::ProtoSegment*, int> map_seg_id;
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != temp_cluster->get_cluster_id()) continue;
    map_id_seg[sg->get_id()] = sg;
    map_seg_id[sg] = sg->get_id();
    // std::cout << "A: " << sg->get_id() << std::endl;
    sg->reset_associate_points();
  }
  if (flag_timing) std::cout << "clustering_points timing: build map and reset associate points took " << std::chrono::duration_cast<CP_ms>(CP_Clock::now()-cp_t0).count() << " ms" << std::endl;

  cp_t0 = CP_Clock::now();
  {
    // find the relevant point clouds ...
    WCP::WCPointCloud<double>& cloud = temp_cluster->get_point_cloud()->get_cloud();
    WCP::WC2DPointCloud<double>& cloud_u = temp_cluster->get_point_cloud()->get_cloud_u();
    WCP::WC2DPointCloud<double>& cloud_v = temp_cluster->get_point_cloud()->get_cloud_v();
    WCP::WC2DPointCloud<double>& cloud_w = temp_cluster->get_point_cloud()->get_cloud_w();

    std::vector<int>& point_sub_cluster_ids = temp_cluster->get_point_sub_cluster_ids();


    for (size_t i=0;i!=point_sub_cluster_ids.size();i++){
      //      std::cout << point_sub_cluster_ids.at(i) << std::endl;
      if (point_sub_cluster_ids.at(i) == -1) continue;
      if (map_id_seg.find(point_sub_cluster_ids.at(i))==map_id_seg.end()) continue;
      map_id_seg[point_sub_cluster_ids.at(i)]->add_associate_point(cloud.pts[i], cloud_u.pts[i], cloud_v.pts[i], cloud_w.pts[i]);
    }
  }
  if (flag_timing) std::cout << "clustering_points timing: assign points from point cloud took " << std::chrono::duration_cast<CP_ms>(CP_Clock::now()-cp_t0).count() << " ms" << std::endl;

  cp_t0 = CP_Clock::now();
  {
    WCP::WCPointCloud<double>& cloud = temp_cluster->get_point_cloud_steiner()->get_cloud();
    std::vector<int>& point_steiner_sub_cluster_ids = temp_cluster->get_point_steiner_sub_cluster_ids();
    for (size_t i=0;i!=point_steiner_sub_cluster_ids.size();i++){
      if (point_steiner_sub_cluster_ids.at(i) == -1) continue;
      if (map_id_seg.find(point_steiner_sub_cluster_ids.at(i))==map_id_seg.end()) continue;
      map_id_seg[point_steiner_sub_cluster_ids.at(i)]->add_associate_point_steiner(cloud.pts[i]);
    }
  }
  if (flag_timing) std::cout << "clustering_points timing: assign steiner points took " << std::chrono::duration_cast<CP_ms>(CP_Clock::now()-cp_t0).count() << " ms" << std::endl;

  cp_t0 = CP_Clock::now();
  // build kdtree
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != temp_cluster->get_cluster_id()) continue;
    ToyPointCloud *pcloud_associate = sg->get_associated_pcloud();
    if (pcloud_associate !=0) pcloud_associate->build_kdtree_index();
    ToyPointCloud *pcloud_associate_steiner = sg->get_associated_pcloud_steiner();
    if (pcloud_associate_steiner !=0) pcloud_associate_steiner->build_kdtree_index();
  }
  if (flag_timing) std::cout << "clustering_points timing: build kdtree took " << std::chrono::duration_cast<CP_ms>(CP_Clock::now()-cp_t0).count() << " ms" << std::endl;
  if (flag_timing) std::cout << "clustering_points timing: TOTAL took " << std::chrono::duration_cast<CP_ms>(CP_Clock::now()-cp_t_total).count() << " ms" << std::endl;

}



WCPPID::NeutrinoID::~NeutrinoID(){
  for (auto it = proto_vertices.begin(); it != proto_vertices.end(); it++){
    delete (*it);
  }
  for (auto it = proto_segments.begin(); it != proto_segments.end(); it++){
    delete (*it);
  }
  for (auto it = showers.begin(); it!= showers.end(); it++){
    delete (*it);
  }
}




  
int WCPPID::NeutrinoID::get_num_segments(WCPPID::ProtoVertex *pv){
  if (map_vertex_segments.find(pv) != map_vertex_segments.end()){
    return map_vertex_segments[pv].size();
  }else{
    return 0;
  }
}



void WCPPID::NeutrinoID::organize_vertices_segments(){
  WCPPID::ProtoVertexSelection temp_vertices = proto_vertices;
  WCPPID::ProtoSegmentSelection temp_segments = proto_segments;
  proto_vertices.clear();
  proto_segments.clear();
  WCPPID::ProtoVertexSet temp_vset;
  WCPPID::ProtoSegmentSet temp_sset;
  WCPPID::ProtoSegmentSet temp_delset;
  for (auto it = temp_vertices.begin(); it!=temp_vertices.end(); it++){
    if (map_vertex_segments.find(*it)!=map_vertex_segments.end() &&
	temp_vset.find(*it)==temp_vset.end()){
      proto_vertices.push_back(*it);
      temp_vset.insert(*it);
      //std::cout << "1: " << *it << std::endl;
      // std::cout <<"1: " << (*it)->get_wcpt().index << std::endl;
    }
  }
  for (auto it = temp_segments.begin(); it!=temp_segments.end(); it++){
    
    if (map_segment_vertices.find(*it)!=map_segment_vertices.end()){
      if (temp_sset.find(*it)==temp_sset.end()){
	proto_segments.push_back(*it);
	temp_sset.insert(*it);
	//std::cout << (*it)->get_wcpt_vec().front().index << " " << (*it)->get_wcpt_vec().back().index << " " << (*it)->get_point_vec().front() << " " << (*it)->get_wcpt_vec().front().x << " " << (*it)->get_wcpt_vec().front().y << " " << (*it)->get_wcpt_vec().front().z << " " << (*it)->get_point_vec().back() << " " << (*it)->get_wcpt_vec().back().x << " " << (*it)->get_wcpt_vec().back().y << " " << (*it)->get_wcpt_vec().back().z << std::endl;
	//std::cout << "2: " << *it << std::endl;
      }
    }else{
      temp_delset.insert(*it);
    }
  }

  for (auto it = temp_delset.begin(); it!=temp_delset.end();it++){
    delete *it;
  }
}


void WCPPID::NeutrinoID::process_main_cluster(){ 

  WCPPID::PR3DCluster *temp_cluster = main_cluster;
  if (temp_cluster->get_point_cloud_steiner()!=0){
    if (temp_cluster->get_point_cloud_steiner()->get_num_points() >= 2){
      std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> wcps = temp_cluster->get_two_boundary_wcps(2); 
      temp_cluster->dijkstra_shortest_paths(wcps.first,2); 
      temp_cluster->cal_shortest_path(wcps.second,2);
    }
    if (temp_cluster->get_path_wcps().size()>=2){
      temp_cluster->collect_charge_trajectory(*ct_point_cloud);
      temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond);
    }
  }
  
}


void WCPPID::NeutrinoID::process_other_clusters(){
  for (auto it1 =other_clusters.begin(); it1!=other_clusters.end();it1++){
    WCPPID::PR3DCluster *temp_cluster = (*it1);
    if (temp_cluster->get_point_cloud_steiner()!=0){
      if (temp_cluster->get_point_cloud_steiner()->get_num_points() >= 2){
    	std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> wcps = temp_cluster->get_two_boundary_wcps(2); 
    	temp_cluster->dijkstra_shortest_paths(wcps.first,2); 
    	temp_cluster->cal_shortest_path(wcps.second,2);
      }
      if (temp_cluster->get_path_wcps().size()>=2){
    	temp_cluster->collect_charge_trajectory(*ct_point_cloud);
    	temp_cluster->do_tracking(*ct_point_cloud, global_wc_map, flash_time*units::microsecond);
      }
    }
  }
}

void WCPPID::NeutrinoID::fill_reco_tree(WCPPID::ProtoSegment* sg, WCRecoTree& rtree){
  rtree.mc_id[rtree.mc_Ntrack]  = sg->get_cluster_id()*1000 + sg->get_id();
  // particle code
  // e- 11  e+ -11
  // muon- 13  muon+ -13
  // gamma 22
  // pi+ 211, pi0 111, pi- -211
  // kaon+ 321, K- -321
  // p  2212
  // n 2112

  
  
  //    std::cout << sg->cal_kine_range()/units::MeV << " " << sg->cal_kine_dQdx()/units::MeV << std::endl;
  rtree.mc_included[rtree.mc_Ntrack] = 1; // always included
  
  rtree.mc_pdg[rtree.mc_Ntrack] = sg->get_particle_type(); // all muons for now ...
  rtree.mc_process[rtree.mc_Ntrack] = 0;
  rtree.mc_mother[rtree.mc_Ntrack] = 0; 
  rtree.mc_dir_weak[rtree.mc_Ntrack] = sg->is_dir_weak();
  rtree.mc_kine_range[rtree.mc_Ntrack] = sg->cal_kine_range()/units::MeV;
  rtree.mc_kine_dQdx[rtree.mc_Ntrack] = sg->cal_kine_dQdx()/units::MeV;
  rtree.mc_kine_charge[rtree.mc_Ntrack] = cal_kine_charge(sg)/units::MeV;
  rtree.mc_length[rtree.mc_Ntrack] = sg->get_length()/units::cm;
  // std::cout << rtree.mc_id[rtree.mc_Ntrack] << " " << rtree.mc_dir_weak[rtree.mc_Ntrack] << " " << rtree.mc_kine_range[rtree.mc_Ntrack] << " " << rtree.mc_kine_dQdx[rtree.mc_Ntrack] << " " << rtree.mc_kine_charge[rtree.mc_Ntrack] << std::endl;
  
  sg->set_kine_charge( rtree.mc_kine_charge[rtree.mc_Ntrack] * units::MeV);
  //    std::cout << rtree.mc_kine_range[rtree.mc_Ntrack] << " " << rtree.mc_kine_dQdx[rtree.mc_Ntrack] << " " << rtree.mc_kine_charge[rtree.mc_Ntrack] << std::endl;
 
 // std::cout<<"id "<<sg->get_cluster_id()*1000 + sg->get_id()<<"  pdg "<<sg->get_particle_type()<<"  KE "<<sg->get_particle_4mom(3)-sg->get_particle_mass()<<"  x "<<sg->get_point_vec().front().x/units::cm<<"  y  "<<sg->get_point_vec().front().y/units::cm<<"  z "<<sg->get_point_vec().front().z/units::cm<<std::endl;

 
  if (sg->get_flag_dir()==0) return; // no direction not plot 

  else if (sg->get_flag_dir()==1){
    rtree.mc_startXYZT[rtree.mc_Ntrack][0] = sg->get_point_vec().front().x/units::cm;
    rtree.mc_startXYZT[rtree.mc_Ntrack][1] = sg->get_point_vec().front().y/units::cm;
    rtree.mc_startXYZT[rtree.mc_Ntrack][2] = sg->get_point_vec().front().z/units::cm;
    rtree.mc_startXYZT[rtree.mc_Ntrack][3] = 0;
    
    rtree.mc_endXYZT[rtree.mc_Ntrack][0] = sg->get_point_vec().back().x/units::cm;
    rtree.mc_endXYZT[rtree.mc_Ntrack][1] = sg->get_point_vec().back().y/units::cm;
    rtree.mc_endXYZT[rtree.mc_Ntrack][2] = sg->get_point_vec().back().z/units::cm;
    rtree.mc_endXYZT[rtree.mc_Ntrack][3] = 0;
  }else if (sg->get_flag_dir()==-1){
    rtree.mc_startXYZT[rtree.mc_Ntrack][0] = sg->get_point_vec().back().x/units::cm;
    rtree.mc_startXYZT[rtree.mc_Ntrack][1] = sg->get_point_vec().back().y/units::cm;
    rtree.mc_startXYZT[rtree.mc_Ntrack][2] = sg->get_point_vec().back().z/units::cm;
    rtree.mc_startXYZT[rtree.mc_Ntrack][3] = 0;
    
    rtree.mc_endXYZT[rtree.mc_Ntrack][0] = sg->get_point_vec().front().x/units::cm;
    rtree.mc_endXYZT[rtree.mc_Ntrack][1] = sg->get_point_vec().front().y/units::cm;
    rtree.mc_endXYZT[rtree.mc_Ntrack][2] = sg->get_point_vec().front().z/units::cm;
    rtree.mc_endXYZT[rtree.mc_Ntrack][3] = 0;
  }
  
  if (sg->get_particle_4mom(3)>0){
    //      std::cout << sg->get_particle_4mom(0)/units::MeV << " " << sg->get_particle_4mom(1)/units::MeV << " " << sg->get_particle_4mom(2)/units::MeV << " " << sg->get_particle_4mom(3)/units::MeV << " " << sg->get_particle_type() << " " << sg->get_particle_mass() << std::endl;
    rtree.mc_startMomentum[rtree.mc_Ntrack][0] = sg->get_particle_4mom(0)/units::GeV;
    rtree.mc_startMomentum[rtree.mc_Ntrack][1] = sg->get_particle_4mom(1)/units::GeV;
    rtree.mc_startMomentum[rtree.mc_Ntrack][2] = sg->get_particle_4mom(2)/units::GeV;
    rtree.mc_startMomentum[rtree.mc_Ntrack][3] = sg->get_particle_4mom(3)/units::GeV;
    
    rtree.mc_endMomentum[rtree.mc_Ntrack][0] = 0;
    rtree.mc_endMomentum[rtree.mc_Ntrack][1] = 0;
    rtree.mc_endMomentum[rtree.mc_Ntrack][2] = 0;
    rtree.mc_endMomentum[rtree.mc_Ntrack][3] = sg->get_particle_mass()/units::GeV;
  }else{
    rtree.mc_startMomentum[rtree.mc_Ntrack][0] = 0;
    rtree.mc_startMomentum[rtree.mc_Ntrack][1] = 0;
    rtree.mc_startMomentum[rtree.mc_Ntrack][2] = 0;
    rtree.mc_startMomentum[rtree.mc_Ntrack][3] = 0;
    
    rtree.mc_endMomentum[rtree.mc_Ntrack][0] = 0;
    rtree.mc_endMomentum[rtree.mc_Ntrack][1] = 0;
    rtree.mc_endMomentum[rtree.mc_Ntrack][2] = 0;
    rtree.mc_endMomentum[rtree.mc_Ntrack][3] = 0;
  }
  rtree.mc_Ntrack++;
  rtree.mc_daughters->resize(rtree.mc_Ntrack);
}

void WCPPID::NeutrinoID::fill_reco_tree(WCPPID::WCShower* shower, WCRecoTree& rtree){
  //  fill_reco_tree(shower->get_start_segment(), rtree);

  WCPPID::ProtoSegment *sg1 = shower->get_start_segment();
  
  rtree.mc_id[rtree.mc_Ntrack]  = sg1->get_cluster_id()*1000 + sg1->get_id();
  rtree.mc_pdg[rtree.mc_Ntrack] = shower->get_particle_type();
  rtree.mc_process[rtree.mc_Ntrack] = 0;

  if (shower->get_start_vertex().second <=2){
    rtree.mc_included[rtree.mc_Ntrack] = 1;
  }else{
    rtree.mc_included[rtree.mc_Ntrack] = shower->get_start_vertex().second; // 3 or 4 ...
  }

  
  rtree.mc_kine_range[rtree.mc_Ntrack] = shower->get_kine_range()/units::MeV;
  rtree.mc_kine_dQdx[rtree.mc_Ntrack] = shower->get_kine_dQdx()/units::MeV;
  rtree.mc_kine_charge[rtree.mc_Ntrack] = shower->get_kine_charge()/units::MeV;
  rtree.mc_length[rtree.mc_Ntrack] = shower->get_total_length()/units::cm;
  
  rtree.mc_startXYZT[rtree.mc_Ntrack][0] = shower->get_start_point().x/units::cm;
  rtree.mc_startXYZT[rtree.mc_Ntrack][1] = shower->get_start_point().y/units::cm;
  rtree.mc_startXYZT[rtree.mc_Ntrack][2] = shower->get_start_point().z/units::cm;
  rtree.mc_startXYZT[rtree.mc_Ntrack][3] = 0;
  
  rtree.mc_endXYZT[rtree.mc_Ntrack][0] = shower->get_end_point().x/units::cm;
  rtree.mc_endXYZT[rtree.mc_Ntrack][1] = shower->get_end_point().y/units::cm;
  rtree.mc_endXYZT[rtree.mc_Ntrack][2] = shower->get_end_point().z/units::cm;
  rtree.mc_endXYZT[rtree.mc_Ntrack][3] = 0;

  
  std::pair<ProtoVertex*, int> pair_start_vertex = shower->get_start_vertex();
  if (pair_start_vertex.second ==4 ){
    rtree.mc_dir_weak[rtree.mc_Ntrack] = 1;
    return;
  }else{
    rtree.mc_dir_weak[rtree.mc_Ntrack] = 0;
  }

  double kine_best = shower->get_kine_best();
  if (kine_best ==0 ) kine_best = shower->get_kine_charge();
  rtree.mc_startMomentum[rtree.mc_Ntrack][3] = (kine_best + sg1->get_particle_mass())/units::GeV;
  double momentum = sqrt(pow(kine_best + sg1->get_particle_mass(),2) - pow(sg1->get_particle_mass(),2));
  TVector3 init_dir = shower->get_init_dir();
  init_dir = init_dir.Unit();
  rtree.mc_startMomentum[rtree.mc_Ntrack][0] = momentum * init_dir.X()/units::GeV;
  rtree.mc_startMomentum[rtree.mc_Ntrack][1] = momentum * init_dir.Y()/units::GeV;
  rtree.mc_startMomentum[rtree.mc_Ntrack][2] = momentum * init_dir.Z()/units::GeV;
  
 // std::cout<<"id "<<sg1->get_cluster_id()*1000 + sg1->get_id()<<"  pdg "<<shower->get_particle_type()<<"  KE "<<kine_best<<"  x "<<shower->get_start_point().x/units::cm<<"  y  "<<shower->get_start_point().y/units::cm<<"  z "<<shower->get_start_point().z/units::cm<<std::endl;

  rtree.mc_mother[rtree.mc_Ntrack] = 0;
  rtree.mc_Ntrack++;
  rtree.mc_daughters->resize(rtree.mc_Ntrack);
  
}

std::pair<int, int> WCPPID::NeutrinoID::fill_pi0_reco_tree(WCPPID::WCShower* shower, WCRecoTree& rtree){
  // not calculation energy ....
  rtree.mc_included[rtree.mc_Ntrack] = 0;
  
  if (map_pio_id_saved_pair.find( map_shower_pio_id[shower]) == map_pio_id_saved_pair.end()){
    WCPPID::ProtoSegment *sg1 = shower->get_start_segment();
    rtree.mc_id[rtree.mc_Ntrack]  = sg1->get_cluster_id()*1000 + map_shower_pio_id[shower];
    rtree.mc_pdg[rtree.mc_Ntrack] = 111;
 // std::cout<<"id "<<sg1->get_cluster_id()*1000 + map_shower_pio_id[shower]<<"  pdg 111"<<"  KE "<<map_pio_id_mass[map_shower_pio_id[shower]].first<<"  x "<<shower->get_start_vertex().first->get_fit_pt().x/units::cm<<"  y "<<shower->get_start_vertex().first->get_fit_pt().y/units::cm<<"  z "<<shower->get_start_vertex().first->get_fit_pt().z/units::cm<<std::endl;    
    rtree.mc_process[rtree.mc_Ntrack] = 0;  
    rtree.mc_kine_range[rtree.mc_Ntrack] = 0;
    rtree.mc_kine_dQdx[rtree.mc_Ntrack] = 0;
    rtree.mc_kine_charge[rtree.mc_Ntrack] = 0;
    rtree.mc_length[rtree.mc_Ntrack] = 0;
    rtree.mc_stopped[rtree.mc_Ntrack-1] = 0;
    
    rtree.mc_startXYZT[rtree.mc_Ntrack][0] = shower->get_start_vertex().first->get_fit_pt().x/units::cm;
    rtree.mc_startXYZT[rtree.mc_Ntrack][1] = shower->get_start_vertex().first->get_fit_pt().y/units::cm;
    rtree.mc_startXYZT[rtree.mc_Ntrack][2] = shower->get_start_vertex().first->get_fit_pt().z/units::cm;
    rtree.mc_startXYZT[rtree.mc_Ntrack][3] = 0;
    
    rtree.mc_endXYZT[rtree.mc_Ntrack][0] = shower->get_start_vertex().first->get_fit_pt().x/units::cm;
    rtree.mc_endXYZT[rtree.mc_Ntrack][1] = shower->get_start_vertex().first->get_fit_pt().y/units::cm;
    rtree.mc_endXYZT[rtree.mc_Ntrack][2] = shower->get_start_vertex().first->get_fit_pt().z/units::cm;
    rtree.mc_endXYZT[rtree.mc_Ntrack][3] = 0;
    
    std::pair<ProtoVertex*, int> pair_start_vertex = shower->get_start_vertex();
    
    rtree.mc_dir_weak[rtree.mc_Ntrack] = 0;
    
    rtree.mc_startMomentum[rtree.mc_Ntrack][0] = sqrt(pow(map_pio_id_mass[map_shower_pio_id[shower]].first+135*units::MeV,2) - pow(135*units::MeV,2))/units::GeV;
    rtree.mc_startMomentum[rtree.mc_Ntrack][1] = 0;
    rtree.mc_startMomentum[rtree.mc_Ntrack][2] = 0;
    rtree.mc_startMomentum[rtree.mc_Ntrack][3] = (map_pio_id_mass[map_shower_pio_id[shower]].first+135*units::MeV)/units::GeV;
    
    rtree.mc_mother[rtree.mc_Ntrack] = 0;
    rtree.mc_Ntrack++;
    rtree.mc_daughters->resize(rtree.mc_Ntrack);
    
    map_pio_id_saved_pair[ map_shower_pio_id[shower] ] = std::make_pair(rtree.mc_Ntrack-1, rtree.mc_id[rtree.mc_Ntrack-1]);
    return std::make_pair(rtree.mc_Ntrack-1, rtree.mc_id[rtree.mc_Ntrack-1]);
  }else{
    return map_pio_id_saved_pair[ map_shower_pio_id[shower] ];
  }
}



int WCPPID::NeutrinoID::fill_psuedo_reco_tree(WCPPID::WCShower* shower, WCRecoTree& rtree){
  
  WCPPID::ProtoSegment *sg1 = shower->get_start_segment();
  
  rtree.mc_id[rtree.mc_Ntrack]  = sg1->get_cluster_id()*1000 + acc_segment_id; acc_segment_id ++;
  if (fabs(shower->get_particle_type())==11 || fabs(shower->get_particle_type())==22){
    rtree.mc_pdg[rtree.mc_Ntrack] = 22;
  }else{
    rtree.mc_pdg[rtree.mc_Ntrack] = 2112;
  }
  rtree.mc_process[rtree.mc_Ntrack] = 0;

  rtree.mc_included[rtree.mc_Ntrack] = 0;
  
  rtree.mc_kine_range[rtree.mc_Ntrack] = 0;
  rtree.mc_kine_dQdx[rtree.mc_Ntrack] = 0;
  rtree.mc_kine_charge[rtree.mc_Ntrack] = 0;
  rtree.mc_length[rtree.mc_Ntrack] = 0;
  rtree.mc_stopped[rtree.mc_Ntrack-1] = 0;
  
  rtree.mc_startXYZT[rtree.mc_Ntrack][0] = shower->get_start_vertex().first->get_fit_pt().x/units::cm;
  rtree.mc_startXYZT[rtree.mc_Ntrack][1] = shower->get_start_vertex().first->get_fit_pt().y/units::cm;
  rtree.mc_startXYZT[rtree.mc_Ntrack][2] = shower->get_start_vertex().first->get_fit_pt().z/units::cm;
  rtree.mc_startXYZT[rtree.mc_Ntrack][3] = 0;
  
  rtree.mc_endXYZT[rtree.mc_Ntrack][0] = shower->get_start_point().x/units::cm;
  rtree.mc_endXYZT[rtree.mc_Ntrack][1] = shower->get_start_point().y/units::cm;
  rtree.mc_endXYZT[rtree.mc_Ntrack][2] = shower->get_start_point().z/units::cm;
  rtree.mc_endXYZT[rtree.mc_Ntrack][3] = 0;

  std::pair<ProtoVertex*, int> pair_start_vertex = shower->get_start_vertex();

  rtree.mc_dir_weak[rtree.mc_Ntrack] = 0;
  

  double kine_best = shower->get_kine_best();
  if (kine_best ==0 ) kine_best = shower->get_kine_charge();
  double momentum;
  if (fabs(shower->get_particle_type())==11 || fabs(shower->get_particle_type())==22){
    rtree.mc_startMomentum[rtree.mc_Ntrack][3] = (kine_best)/units::GeV;
    momentum = kine_best;
  }else{
    TPCParams& mp = Singleton<TPCParams>::Instance();
    double mass_neutron = mp.get_mass_neutron();
    rtree.mc_startMomentum[rtree.mc_Ntrack][3] = (kine_best + mass_neutron)/units::GeV;
    momentum = sqrt(pow(kine_best + mass_neutron,2) - pow(mass_neutron,2));
  }
  
  TVector3 init_dir(  rtree.mc_endXYZT[rtree.mc_Ntrack][0] - rtree.mc_startXYZT[rtree.mc_Ntrack][0], rtree.mc_endXYZT[rtree.mc_Ntrack][1] - rtree.mc_startXYZT[rtree.mc_Ntrack][1] , rtree.mc_endXYZT[rtree.mc_Ntrack][2] - rtree.mc_startXYZT[rtree.mc_Ntrack][2]);
  init_dir = init_dir.Unit();
  if (init_dir.Mag()==0) init_dir = shower->get_start_segment()->cal_dir_3vector();
  rtree.mc_startMomentum[rtree.mc_Ntrack][0] = momentum * init_dir.X()/units::GeV;
  rtree.mc_startMomentum[rtree.mc_Ntrack][1] = momentum * init_dir.Y()/units::GeV;
  rtree.mc_startMomentum[rtree.mc_Ntrack][2] = momentum * init_dir.Z()/units::GeV;
  
  //std::cout<<"id "<<sg1->get_cluster_id()*1000 + acc_segment_id<<"  pdg "<<rtree.mc_pdg[rtree.mc_Ntrack]<<"  KE "<<kine_best<<"  x "<<shower->get_start_vertex().first->get_fit_pt().x/units::cm<<"  y "<<shower->get_start_vertex().first->get_fit_pt().y/units::cm<<"  z "<<shower->get_start_vertex().first->get_fit_pt().z/units::cm<<std::endl;

  rtree.mc_mother[rtree.mc_Ntrack] = 0;
  rtree.mc_Ntrack++;
  rtree.mc_daughters->resize(rtree.mc_Ntrack);


  return rtree.mc_id[rtree.mc_Ntrack-1];
}


void WCPPID::NeutrinoID::fill_reco_simple_tree(WCPPID::WCRecoTree& rtree){
  
  //start to fill  
  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment* sg= it->first;
    // only save the main cluster
    if (sg->get_cluster_id() != main_cluster->get_cluster_id()) continue;
   
    fill_reco_tree(sg, rtree);
    std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> pair_vertices = find_vertices(sg);
    if (map_vertex_segments[pair_vertices.first].size()==1 || map_vertex_segments[pair_vertices.second].size()==1 ){
      rtree.mc_stopped[rtree.mc_Ntrack-1] = 1;
    }else{
      rtree.mc_stopped[rtree.mc_Ntrack-1] = 0;
    }
  }
  // std::cout << rtree.mc_Ntrack << std::endl;
}




void WCPPID::NeutrinoID::fill_particle_tree(WCPPID::WCRecoTree& rtree){
  if (main_vertex ==0 ) return;

  // helpers for diagnostic prints
  auto fpt_pdgname = [](int pdg) -> std::string {
    if (pdg==11)   return "e-";
    if (pdg==-11)  return "e+";
    if (pdg==13)   return "mu-";
    if (pdg==-13)  return "mu+";
    if (pdg==2212) return "proton";
    if (pdg==2112) return "neutron";
    if (pdg==211)  return "pi+";
    if (pdg==-211) return "pi-";
    if (pdg==111)  return "pi0";
    if (pdg==22)   return "gamma";
    return std::to_string(pdg);
  };
  auto fpt_ke_mev = [](const WCPPID::WCRecoTree& rt, int idx) -> float {
    int pdg = rt.mc_pdg[idx];
    float mass = 0;
    if (abs(pdg)==13)           mass = 0.1057f;
    if (abs(pdg)==211)          mass = 0.1396f;
    if (pdg==111)               mass = 0.1350f;
    if (pdg==2212 || pdg==2112) mass = 0.9383f;
    return (rt.mc_startMomentum[idx][3] - mass) * 1000.f;
  };

  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment* sg= it->first;
    if (map_segment_in_shower.find(sg)!=map_segment_in_shower.end()) continue;
    if (sg->get_cluster_id() != main_vertex->get_cluster_id() ) continue;
    fill_reco_tree(sg, rtree);
    std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> pair_vertices = find_vertices(sg);
    if (map_vertex_segments[pair_vertices.first].size()==1 || map_vertex_segments[pair_vertices.second].size()==1 ){
      rtree.mc_stopped[rtree.mc_Ntrack-1] = 1;
    }else{
      rtree.mc_stopped[rtree.mc_Ntrack-1] = 0;
    }
    { int n = rtree.mc_Ntrack-1;
      std::cout << "[fill_particle_tree] ADD track-node"
                << "  seg=" << rtree.mc_id[n]
                << "  parent=ROOT"
                << "  name=" << fpt_pdgname(rtree.mc_pdg[n])
                << "  ke=" << fpt_ke_mev(rtree,n) << " MeV"
                << "  cluster=" << sg->get_cluster_id()
                << "  in_main_cluster=1  is_shower_seg=0"
                << "  start=(" << rtree.mc_startXYZT[n][0] << "," << rtree.mc_startXYZT[n][1] << "," << rtree.mc_startXYZT[n][2] << ") cm"
                << "  end=(" << rtree.mc_endXYZT[n][0] << "," << rtree.mc_endXYZT[n][1] << "," << rtree.mc_endXYZT[n][2] << ") cm"
                << std::endl;
    }
        //std::cout << "kak " << sg->get_cluster_id() << " " << sg->get_id() << std::endl;
  }
//std::cout<<std::endl;
//std::cout<<"Passed over tracks"<<std::endl;
for(int i=0; i<rtree.mc_Ntrack; i++){
double mass = 0;
if(rtree.mc_pdg[i]==13) mass = 0.1057;
if(abs(rtree.mc_pdg[i])==211) mass = 0.140;
if(rtree.mc_pdg[i]==111) mass = 0.135;
if(rtree.mc_pdg[i]==2212 || rtree.mc_pdg[i]==2112) mass = 0.938;
double ke = rtree.mc_startMomentum[i][3]-mass;
//std::cout<<"id "<<rtree.mc_id[i]<<"  pdg "<<rtree.mc_pdg[i]<<"  mom "<<rtree.mc_mother[i]<<"  inc "<<rtree.mc_included[i]<<"  KE "<<ke*1000<<"  Start:  x "<<rtree.mc_startXYZT[i][0]<<"  y "<<rtree.mc_startXYZT[i][1]<<"  z "<<rtree.mc_startXYZT[i][2]<<"  End:  x "<<rtree.mc_endXYZT[i][0]<<"  y "<<rtree.mc_endXYZT[i][1]<<"  z "<<rtree.mc_endXYZT[i][2]<<std::endl;
}
//std::cout<<std::endl;

  for (auto it = showers.begin(); it!=showers.end();it++){
    WCPPID::WCShower *fpt_sh = *it;
    std::pair<WCPPID::ProtoVertex*, int> fpt_svtx = fpt_sh->get_start_vertex();
    int fpt_conn = fpt_svtx.second;
    double fpt_ke_s = fpt_sh->get_kine_best();
    if (fpt_ke_s == 0) fpt_ke_s = fpt_sh->get_kine_charge();
    fpt_ke_s /= units::MeV;
    int fpt_pdg_s = fpt_sh->get_particle_type();
    int fpt_clus_s = fpt_sh->get_start_segment() ? fpt_sh->get_start_segment()->get_cluster_id() : -1;
    if (fpt_conn == 4) {
      std::cout << "[fill_particle_tree] SKIP shower (conn_type=4)"
                << "  pdg=" << fpt_pdg_s
                << "  ke=" << fpt_ke_s << " MeV"
                << "  cluster=" << fpt_clus_s << std::endl;
    } else {
      std::string fpt_reason;
      if (fpt_svtx.first == main_vertex)
        fpt_reason = "reason=start_vtx==main_vertex";
      else if (map_vertex_in_shower.find(fpt_svtx.first) != map_vertex_in_shower.end())
        fpt_reason = "(via root-reachable shower vtx)";
      else
        fpt_reason = "reason=via_parent_segment";
      std::cout << "[fill_particle_tree] ROOT shower  conn_type=" << fpt_conn
                << "  " << fpt_reason
                << "  pdg=" << fpt_pdg_s
                << "  ke=" << fpt_ke_s << " MeV"
                << "  cluster=" << fpt_clus_s
                << "  start=(" << fpt_sh->get_start_point().x/units::cm
                << "," << fpt_sh->get_start_point().y/units::cm
                << "," << fpt_sh->get_start_point().z/units::cm << ") cm" << std::endl;
    }
    fill_reco_tree(*it, rtree);
    rtree.mc_stopped[rtree.mc_Ntrack-1] = 0;
    { int n = rtree.mc_Ntrack-1;
      std::cout << "[fill_particle_tree] ADD shower-leaf"
                << "  id=" << rtree.mc_id[n]
                << "  pdg=" << rtree.mc_pdg[n]
                << "  conn_type=" << fpt_conn
                << "  ke=" << fpt_ke_s << " MeV"
                << "  cluster=" << fpt_clus_s
                << "  has_start_vtx=" << (fpt_svtx.first != nullptr ? 1 : 0) << std::endl;
    }
  }

    // id vs. rtree id
  std::map<int, int> map_sgid_rtid;
  std::map<int, int> map_rtid_sgid;
  for (int i=0;i!=rtree.mc_Ntrack;i++){
    int sgid = rtree.mc_id[i];
    map_sgid_rtid[sgid] = i;
    map_rtid_sgid[i] = sgid;
  }
  
  // id vs. sg 
  std::map<int, WCPPID::ProtoSegment*> map_sgid_sg;
  std::map<WCPPID::ProtoSegment*, int> map_sg_sgid;
  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment* sg= it->first;
    // only save the main cluster
    int sgid = sg->get_cluster_id()*1000 + sg->get_id();
    map_sg_sgid[sg] = sgid;
    map_sgid_sg[sgid] = sg;
  }
  
  // main_vertex figure out the daughters and mother ...
  std::set<WCPPID::ProtoVertex* > used_vertices;
  std::set<WCPPID::ProtoSegment* > used_segments;

  for (auto it = showers.begin(); it!=showers.end();it++){
    (*it)->fill_sets(used_vertices, used_segments, false);
  }
  
  
  std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > segments_to_be_examined;
  for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end(); it++){
    // parent are all zero now ...
    used_segments.insert(*it);
    WCPPID::ProtoVertex *other_vertex = find_other_vertex(*it, main_vertex);
    segments_to_be_examined.push_back(std::make_pair(other_vertex, *it));
  }
  used_vertices.insert(main_vertex);
  //std::cout << segments_to_be_examined.size() << " " << used_vertices.size() << " " << used_segments.size() << std::endl;
  
  while(segments_to_be_examined.size()>0){
    std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > temp_segments;
    for (auto it = segments_to_be_examined.begin(); it!= segments_to_be_examined.end(); it++){
      WCPPID::ProtoVertex *curr_vtx = it->first;
      WCPPID::ProtoSegment *prev_sg = it->second;
      if (used_vertices.find(curr_vtx)!=used_vertices.end()) continue;

      for (auto it1 = map_vertex_segments[curr_vtx].begin(); it1!=map_vertex_segments[curr_vtx].end(); it1++){
	WCPPID::ProtoSegment *curr_sg = *it1;
	if (used_segments.find(curr_sg)!=used_segments.end()) continue;
	used_segments.insert(curr_sg);
	// set mother ...
	rtree.mc_mother[map_sgid_rtid[map_sg_sgid[curr_sg]]] = map_sg_sgid[prev_sg];
	// set daughters ...
	rtree.mc_daughters->at(map_sgid_rtid[map_sg_sgid[prev_sg]]).push_back(map_sg_sgid[curr_sg]);
	WCPPID::ProtoVertex *other_vertex = find_other_vertex(curr_sg, curr_vtx);
	if (used_vertices.find(other_vertex) == used_vertices.end())
	  temp_segments.push_back(std::make_pair(other_vertex, curr_sg));
      }
      used_vertices.insert(curr_vtx);
    }
    segments_to_be_examined = temp_segments;
  }

//std::cout<<std::endl;
//std::cout<<"Passed over tracks+ some showers"<<std::endl;
for(int i=0; i<rtree.mc_Ntrack; i++){
double mass = 0;
if(rtree.mc_pdg[i]==13) mass = 0.1057;
if(abs(rtree.mc_pdg[i])==211) mass = 0.140;
if(rtree.mc_pdg[i]==111) mass = 0.135;
if(rtree.mc_pdg[i]==2212 || rtree.mc_pdg[i]==2112) mass = 0.938;
double ke = rtree.mc_startMomentum[i][3]-mass;
//std::cout<<"id "<<rtree.mc_id[i]<<"  pdg "<<rtree.mc_pdg[i]<<"  mom "<<rtree.mc_mother[i]<<"  inc "<<rtree.mc_included[i]<<"  KE "<<ke*1000<<"  Start:  x "<<rtree.mc_startXYZT[i][0]<<"  y "<<rtree.mc_startXYZT[i][1]<<"  z "<<rtree.mc_startXYZT[i][2]<<"  End:  x "<<rtree.mc_endXYZT[i][0]<<"  y "<<rtree.mc_endXYZT[i][1]<<"  z "<<rtree.mc_endXYZT[i][2]<<std::endl;
}

  // Now the showers
  for (auto it = showers.begin(); it!= showers.end(); it++){
    WCPPID::WCShower *shower = *it;
    WCPPID::ProtoSegment* curr_sg = shower->get_start_segment();
    std::pair<ProtoVertex*, int> pair_vertex = shower->get_start_vertex();

    //    std::cout << pair_vertex.second << std::endl;

    if (pi0_showers.find(shower)==pi0_showers.end()){ // not pi0
      if (pair_vertex.second == 1){ // direct connection
	if (pair_vertex.first == main_vertex){
	  rtree.mc_mother[ map_sgid_rtid[map_sg_sgid[curr_sg]] ] = 0;
	}else{
	  WCPPID::ProtoSegment* prev_sg = 0;
	  if (map_vertex_in_shower.find(pair_vertex.first) != map_vertex_in_shower.end()){
	    prev_sg = map_vertex_in_shower[pair_vertex.first]->get_start_segment();
	  }else{
	    prev_sg = find_incoming_segment(pair_vertex.first);
	  }
	  // set mother ...
	  rtree.mc_mother[map_sgid_rtid[map_sg_sgid[curr_sg]]] = map_sg_sgid[prev_sg];
	  // set daughters ...
	  rtree.mc_daughters->at(map_sgid_rtid[map_sg_sgid[prev_sg]]).push_back(map_sg_sgid[curr_sg]);
	}
      }else if (pair_vertex.second == 2 || pair_vertex.second == 3){
	int psuedo_particle_id = fill_psuedo_reco_tree(shower, rtree);
        { int n = rtree.mc_Ntrack-1;
          std::cout << "[fill_particle_tree] ADD pseudo-gamma"
                    << "  id=" << rtree.mc_id[n]
                    << "  ke=" << rtree.mc_startMomentum[n][3]*1000.f << " MeV"
                    << "  child_pdg=" << shower->get_particle_type() << std::endl;
        }
	if (pair_vertex.first == main_vertex){
	  rtree.mc_mother[rtree.mc_Ntrack-1] = 0;
	  rtree.mc_daughters->at(rtree.mc_Ntrack-1).push_back(map_sg_sgid[curr_sg]);
	  rtree.mc_mother[ map_sgid_rtid[map_sg_sgid[curr_sg]] ] = psuedo_particle_id;
	}else{
	  WCPPID::ProtoSegment* prev_sg = 0;
	  if (map_vertex_in_shower.find(pair_vertex.first) != map_vertex_in_shower.end()){
	    prev_sg = map_vertex_in_shower[pair_vertex.first]->get_start_segment();
	  }else{
	    prev_sg = find_incoming_segment(pair_vertex.first);
	  }
	  

	  rtree.mc_mother[rtree.mc_Ntrack-1] = map_sg_sgid[prev_sg];
	  rtree.mc_daughters->at(rtree.mc_Ntrack-1).push_back(map_sg_sgid[curr_sg]);
	  rtree.mc_mother[ map_sgid_rtid[map_sg_sgid[curr_sg]] ] = psuedo_particle_id;
	  rtree.mc_daughters->at(map_sgid_rtid[map_sg_sgid[prev_sg]]).push_back(psuedo_particle_id);
	}
      }
    }else{

       std::pair<ProtoVertex*, int> pair_vertex = shower->get_start_vertex();
       // pi0 ...
       // create a pi0 ...
       std::pair<int, int> pio_info_pair = fill_pi0_reco_tree(shower, rtree);
       
       int psuedo_particle_id = fill_psuedo_reco_tree(shower, rtree);
       { int n = rtree.mc_Ntrack-1;
         std::cout << "[fill_particle_tree] ADD pseudo-gamma (pi0)"
                   << "  id=" << rtree.mc_id[n]
                   << "  ke=" << rtree.mc_startMomentum[n][3]*1000.f << " MeV"
                   << "  child_pdg=" << shower->get_particle_type() << std::endl;
       }
       //       std::cout << pio_info_pair.first << " " << pio_info_pair.second << " " << pair_vertex.first << " " << main_vertex << std::endl;
       
       if (pair_vertex.first == main_vertex){
	 // pio
	 rtree.mc_mother[pio_info_pair.first] = 0;
	 rtree.mc_daughters->at(pio_info_pair.first).push_back(psuedo_particle_id);
	 // gamma
	 rtree.mc_mother[rtree.mc_Ntrack-1] = pio_info_pair.second;
	 rtree.mc_daughters->at(rtree.mc_Ntrack-1).push_back(map_sg_sgid[curr_sg]);
	 // electron 
	 rtree.mc_mother[ map_sgid_rtid[map_sg_sgid[curr_sg]] ] = psuedo_particle_id;
       }else{
	 WCPPID::ProtoSegment* prev_sg = 0;
	 if (map_vertex_in_shower.find(pair_vertex.first) != map_vertex_in_shower.end()){
	   prev_sg = map_vertex_in_shower[pair_vertex.first]->get_start_segment();
	 }else{
	   prev_sg = find_incoming_segment(pair_vertex.first);
	 }
	 //	 std::cout << map_sg_sgid[prev_sg] << " " <<  psuedo_particle_id << " " <<  pio_info_pair.first << " " << pio_info_pair.second << std::endl;


	 // pio
	 rtree.mc_mother[pio_info_pair.first] = map_sg_sgid[prev_sg];
	 rtree.mc_daughters->at(pio_info_pair.first).push_back(psuedo_particle_id);
	 // gamma
	 rtree.mc_mother[rtree.mc_Ntrack-1] =  pio_info_pair.second;
	 rtree.mc_daughters->at(rtree.mc_Ntrack-1).push_back(map_sg_sgid[curr_sg]);
	 // electron
	 rtree.mc_mother[ map_sgid_rtid[map_sg_sgid[curr_sg]] ] = psuedo_particle_id;
	 if (find(rtree.mc_daughters->at(map_sgid_rtid[map_sg_sgid[prev_sg]]).begin(), rtree.mc_daughters->at(map_sgid_rtid[map_sg_sgid[prev_sg]]).end(), pio_info_pair.second) == rtree.mc_daughters->at(map_sgid_rtid[map_sg_sgid[prev_sg]]).end())
	   rtree.mc_daughters->at(map_sgid_rtid[map_sg_sgid[prev_sg]]).push_back(pio_info_pair.second);
	 //	 rtree.mc_daughters->at(map_sgid_rtid[map_sg_sgid[prev_sg]]).push_back(psuedo_particle_id);
       }
       
      // create the second psuedo particle

      // fill the final daughter ...

      
    }
  }

//std::cout<<std::endl;
//std::cout<<"Passed over everything"<<std::endl;
for(int i=0; i<rtree.mc_Ntrack; i++){
double mass = 0;
if(rtree.mc_pdg[i]==13) mass = 0.1057;
if(abs(rtree.mc_pdg[i])==211) mass = 0.140;
if(rtree.mc_pdg[i]==111) mass = 0.135;
if(rtree.mc_pdg[i]==2212 || rtree.mc_pdg[i]==2112) mass = 0.938;
double ke = rtree.mc_startMomentum[i][3]-mass;
//std::cout<<"id "<<rtree.mc_id[i]<<"  pdg "<<rtree.mc_pdg[i]<<"  mom "<<rtree.mc_mother[i]<<"  inc "<<rtree.mc_included[i]<<"  KE "<<ke*1000<<"  Start:  x "<<rtree.mc_startXYZT[i][0]<<"  y "<<rtree.mc_startXYZT[i][1]<<"  z "<<rtree.mc_startXYZT[i][2]<<"  End:  x "<<rtree.mc_endXYZT[i][0]<<"  y "<<rtree.mc_endXYZT[i][1]<<"  z "<<rtree.mc_endXYZT[i][2]<<std::endl;
}

}

WCPPID::ProtoSegment* WCPPID::NeutrinoID::find_incoming_segment(WCPPID::ProtoVertex *vtx){
  WCPPID::ProtoSegment* sg = 0;
  // find the first segment ...
  for ( auto it = map_vertex_segments[vtx].begin(); it != map_vertex_segments[vtx].end(); it++){
    WCPPID::ProtoSegment* current_sg = (*it);
    bool flag_start;
    if (current_sg->get_wcpt_vec().front().index == vtx->get_wcpt().index)
      flag_start = true;
    else if (current_sg->get_wcpt_vec().back().index == vtx->get_wcpt().index)
      flag_start = false;

    if (flag_start && current_sg->get_flag_dir()==-1
	|| (!flag_start) && current_sg->get_flag_dir()==1 ){
      sg = current_sg;
      break;
    }
    
  }
  
  return sg;
}


void WCPPID::NeutrinoID::fill_proto_main_tree(WCPPID::WCRecoTree& rtree){
  fill_reco_simple_tree(rtree);

  // id vs. rtree id
  std::map<int, int> map_sgid_rtid;
  std::map<int, int> map_rtid_sgid;
  for (int i=0;i!=rtree.mc_Ntrack;i++){
    int sgid = rtree.mc_id[i];
    map_sgid_rtid[sgid] = i;
    map_rtid_sgid[i] = sgid;
  }
  
  // id vs. sg 
  std::map<int, WCPPID::ProtoSegment*> map_sgid_sg;
  std::map<WCPPID::ProtoSegment*, int> map_sg_sgid;
  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment* sg= it->first;
    // only save the main cluster
    if (sg->get_cluster_id() != main_cluster->get_cluster_id()) continue;
    int sgid = sg->get_cluster_id()*1000 + sg->get_id();

    map_sg_sgid[sg] = sgid;
    map_sgid_sg[sgid] = sg;
  }

  
  // main_vertex figure out the daughters and mother ...
  std::set<WCPPID::ProtoVertex* > used_vertices;
  std::set<WCPPID::ProtoSegment* > used_segments;

  std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > segments_to_be_examined;
  for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end(); it++){
    // parent are all zero now ...
    used_segments.insert(*it);
    WCPPID::ProtoVertex *other_vertex = find_other_vertex(*it, main_vertex);
    segments_to_be_examined.push_back(std::make_pair(other_vertex, *it));
  }
  used_vertices.insert(main_vertex);
  //std::cout << segments_to_be_examined.size() << " " << used_vertices.size() << " " << used_segments.size() << std::endl;
  
  while(segments_to_be_examined.size()>0){
    std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > temp_segments;
    for (auto it = segments_to_be_examined.begin(); it!= segments_to_be_examined.end(); it++){
      WCPPID::ProtoVertex *curr_vtx = it->first;
      WCPPID::ProtoSegment *prev_sg = it->second;
      if (used_vertices.find(curr_vtx)!=used_vertices.end()) continue;

      for (auto it1 = map_vertex_segments[curr_vtx].begin(); it1!=map_vertex_segments[curr_vtx].end(); it1++){
	WCPPID::ProtoSegment *curr_sg = *it1;
	if (used_segments.find(curr_sg)!=used_segments.end()) continue;
	used_segments.insert(curr_sg);
	// set mother ...
	rtree.mc_mother[map_sgid_rtid[map_sg_sgid[curr_sg]]] = map_sg_sgid[prev_sg];
	// set daughters ...
	rtree.mc_daughters->at(map_sgid_rtid[map_sg_sgid[prev_sg]]).push_back(map_sg_sgid[curr_sg]);
	WCPPID::ProtoVertex *other_vertex = find_other_vertex(curr_sg, curr_vtx);
	if (used_vertices.find(other_vertex) == used_vertices.end())
	  temp_segments.push_back(std::make_pair(other_vertex, curr_sg));
      }
      used_vertices.insert(curr_vtx);
    }
    segments_to_be_examined = temp_segments;
  }
}


void WCPPID::NeutrinoID::fill_skeleton_info_magnify(int mother_cluster_id, WCPPID::WCPointTree& ptree, TTree *T, double dQdx_scale, double dQdx_offset, bool flag_skip_vertex){

  WCPPID::PR3DClusterSelection clusters_vec;
  clusters_vec.push_back(main_cluster);
  for (auto it = other_clusters.begin(); it!=other_clusters.end(); it++){
    clusters_vec.push_back(*it);
  }
  
  ptree.reco_chi2 = 1;
  ptree.reco_mother_cluster_id = mother_cluster_id;
  

  for (auto it0 = clusters_vec.begin(); it0!=clusters_vec.end(); it0++){
    WCPPID::PR3DCluster* cluster = *it0;
    
    if (!flag_skip_vertex){
      // fill vertex information
      ptree.reco_flag_track_shower = 0;
      for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
	WCPPID::ProtoVertex *vtx = it->first;
	if (vtx->get_cluster_id() != cluster->get_cluster_id()) continue;
	if (map_vertex_in_shower.find(vtx)!=map_vertex_in_shower.end() && vtx != main_vertex) continue;
	ptree.reco_cluster_id = vtx->get_cluster_id();
	ptree.reco_proto_cluster_id = -1;
	ptree.reco_particle_id = -1;
	ptree.reco_ndf = vtx->get_cluster_id();
	ptree.reco_flag_vertex = 1;
      
	ptree.reco_x = vtx->get_fit_pt().x/units::cm;
	ptree.reco_y = vtx->get_fit_pt().y/units::cm;
	ptree.reco_z = vtx->get_fit_pt().z/units::cm;
	ptree.reco_dQ = vtx->get_dQ() * dQdx_scale + dQdx_offset;
	ptree.reco_dx = vtx->get_dx()/units::cm;
	ptree.reco_pu = vtx->get_pu();
	ptree.reco_pv = vtx->get_pv();
	ptree.reco_pw = vtx->get_pw();
	ptree.reco_pt = vtx->get_pt();    
	ptree.reco_reduced_chi2 = vtx->get_reduced_chi2();
	ptree.reco_rr = -1; // no residual range

	T->Fill();

	//std::cout<<" check vtx #### "<<"  "<<ptree.reco_dx<<", x/y/z "<<ptree.reco_x<<"\t"<<ptree.reco_y<<std::endl;

	
	// std::cout<<" check vtx #### "<<"  "
	// 	 <<ptree.reco_x<<"\t"<<ptree.reco_y<<"\t dx and dQ: "
	// 	 <<vtx->get_dx()/units::cm<<"\t"
	// 	 <<vtx->get_dQ()
	// 	 <<std::endl;
      }
    }


    for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
      WCPPID::ProtoSegment *seg = it->first;
      if (seg->get_cluster_id() != cluster->get_cluster_id()) continue;
      std::vector<WCP::Point>& pts = seg->get_point_vec();
      std::vector<double>& dQ_vec = seg->get_dQ_vec();
      std::vector<double>& dx_vec = seg->get_dx_vec();
      std::vector<double>& pu_vec = seg->get_pu_vec();
      std::vector<double>& pv_vec = seg->get_pv_vec();
      std::vector<double>& pw_vec = seg->get_pw_vec();
      std::vector<double>& pt_vec = seg->get_pt_vec();
      std::vector<double>& reduced_chi2_vec = seg->get_reduced_chi2_vec();
      
      ptree.reco_cluster_id = seg->get_cluster_id();
      ptree.reco_ndf = seg->get_cluster_id();
      ptree.reco_proto_cluster_id = seg->get_cluster_id()*1000 + seg->get_id();

      // calculate rr ...
      std::vector<double> rr_vec(pts.size(),0);
      {
	WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
	if ( (*it->second.begin())->get_wcpt().index == it->first->get_wcpt_vec().front().index){
	  start_v = (*it->second.begin());
	  end_v = (*it->second.rbegin());
	}else{
	  end_v = (*it->second.begin());
	  start_v = (*it->second.rbegin());
	}
	//	for (auto it1 = map_segment_vertices[it->first].begin(); it1!=map_segment_vertices[it->first].end(); it1++){
	//  if ((*it1)->get_wcpt().index == it->first->get_wcpt_vec().front().index) start_v = *it1;
	// if ((*it1)->get_wcpt().index == it->first->get_wcpt_vec().back().index) end_v = *it1;
	//}
	
	int start_n = map_vertex_segments[start_v].size();
	int end_n = map_vertex_segments[end_v].size();

	std::vector<double> L(pts.size(),0);
	double acc_length = 0;
	for (size_t i=0;i+1<pts.size();i++){
	  acc_length += sqrt(pow(pts.at(i+1).x-pts.at(i).x,2) + pow(pts.at(i+1).y - pts.at(i).y ,2) + pow(pts.at(i+1).z - pts.at(i).z,2)); 
	  L.at(i+1) = acc_length;
	}
	if ((seg)->get_flag_dir()==1){// forward direction
	  for (size_t i=0;i!=pts.size();i++){
	    rr_vec.at(pts.size()-1-i) = L.back() - L.at(pts.size()-1-i);
	  }
	}else if ((seg)->get_flag_dir()==-1){ // reverse direction
	  rr_vec = L;
	}else{
	  rr_vec = L; // quick
	}
	if (start_n>1) rr_vec.front() = -1;
	if (end_n >1) rr_vec.back() = -1;
	
      }
      
      // hack for now ...
      //      ptree.reco_particle_id = seg->get_cluster_id()*1000 + seg->get_id();
      if (seg->get_flag_shower()){
	ptree.reco_flag_track_shower = 1;
      }else{
	ptree.reco_flag_track_shower = 0;
      }

      if (ptree.reco_flag_track_shower==1){
	ptree.reco_particle_id =1;
      }else{
	ptree.reco_particle_id =4;
      }
      
      ptree.reco_flag_vertex = 0;
      for (size_t i=0;i!=pts.size();i++){
	ptree.reco_x = pts.at(i).x/units::cm;
	ptree.reco_y = pts.at(i).y/units::cm;
	ptree.reco_z = pts.at(i).z/units::cm;
	ptree.reco_dQ = dQ_vec.at(i) * dQdx_scale + dQdx_offset;
	ptree.reco_dx = dx_vec.at(i)/units::cm;
	ptree.reco_pu = pu_vec.at(i);
	ptree.reco_pv = pv_vec.at(i);
	ptree.reco_pw = pw_vec.at(i);
	ptree.reco_pt = pt_vec.at(i);    
	ptree.reco_reduced_chi2 = reduced_chi2_vec.at(i);
	ptree.reco_rr = rr_vec.at(i)/units::cm; // no residual range
	T->Fill();

	//std::cout<<" check pts ---> "<<i<<"  "<<ptree.reco_dx<<", x/y/z "<<ptree.reco_x<<"\t"<<ptree.reco_y<<std::endl;

	// std::cout<<" check pts ---> "<<i<<"  "
	// 	 <<ptree.reco_x<<"\t"<<ptree.reco_y<<"\t dx and dQ: "
	// 	 <<dx_vec.at(i)/units::cm<<"\t"
	// 	 <<dQ_vec.at(i)
	// 	 <<std::endl;
      }
    }
  }
  
}


void WCPPID::NeutrinoID::fill_skeleton_info(int mother_cluster_id, WCPPID::WCPointTree& ptree, TTree *T, double dQdx_scale, double dQdx_offset, bool flag_skip_vertex){
  
  ptree.reco_chi2 = 1;
  ptree.reco_mother_cluster_id = mother_cluster_id;
  
    
  if (!flag_skip_vertex){
    // fill vertex information
    ptree.reco_flag_track_shower = 0;
    for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
      WCPPID::ProtoVertex *vtx = it->first;
      ptree.reco_cluster_id = vtx->get_cluster_id();
      ptree.reco_proto_cluster_id = -1;
      ptree.reco_particle_id = -1;

      ptree.reco_ndf = vtx->get_cluster_id();
      ptree.reco_flag_vertex = 1;
      ptree.reco_x = vtx->get_fit_pt().x/units::cm;
      ptree.reco_y = vtx->get_fit_pt().y/units::cm;
      ptree.reco_z = vtx->get_fit_pt().z/units::cm;
      ptree.reco_dQ = vtx->get_dQ() * dQdx_scale + dQdx_offset;
      ptree.reco_dx = vtx->get_dx()/units::cm;
      ptree.reco_pu = vtx->get_pu();
      ptree.reco_pv = vtx->get_pv();
      ptree.reco_pw = vtx->get_pw();
      ptree.reco_pt = vtx->get_pt();    
      ptree.reco_reduced_chi2 = vtx->get_reduced_chi2();

      if ( vtx->get_dQ()!=0)
	T->Fill();
    }
  }
  
  
  for (auto it = map_vertex_to_shower.begin(); it!=map_vertex_to_shower.end(); it++){
    WCPPID::ProtoVertex* vtx = it->first;
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      WCPPID::WCShower *shower = (*it1);
    
      if (map_vertex_segments[vtx].find(shower->get_start_segment())!=map_vertex_segments[vtx].end()){
	ptree.reco_cluster_id = vtx->get_cluster_id();
	
	ptree.reco_proto_cluster_id = shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id();
	ptree.reco_particle_id = shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id();
	
	ptree.reco_ndf = vtx->get_cluster_id();
	ptree.reco_flag_vertex = 1;
	
	ptree.reco_x = vtx->get_fit_pt().x/units::cm;
	ptree.reco_y = vtx->get_fit_pt().y/units::cm;
	ptree.reco_z = vtx->get_fit_pt().z/units::cm;
	ptree.reco_dQ = vtx->get_dQ() * dQdx_scale + dQdx_offset;
	ptree.reco_dx = vtx->get_dx()/units::cm;
	ptree.reco_pu = vtx->get_pu();
	ptree.reco_pv = vtx->get_pv();
	ptree.reco_pw = vtx->get_pw();
	ptree.reco_pt = vtx->get_pt();    
	ptree.reco_reduced_chi2 = vtx->get_reduced_chi2();

	if ( vtx->get_dQ()!=0)
	  T->Fill();
      }
    }
  }
  
  for (auto it = map_vertex_in_shower.begin(); it != map_vertex_in_shower.end(); it++){
     WCPPID::ProtoVertex *vtx = it->first;
     WCPPID::WCShower *shower = it->second;

     ptree.reco_proto_cluster_id = shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id();
     if (shower->get_flag_shower()){
       ptree.reco_flag_track_shower = 1;
     }else{
       ptree.reco_flag_track_shower = 0;
     }
     ptree.reco_particle_id = shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id();
     ptree.reco_flag_vertex = 0;

     ptree.reco_x = vtx->get_fit_pt().x/units::cm;
     ptree.reco_y = vtx->get_fit_pt().y/units::cm;
     ptree.reco_z = vtx->get_fit_pt().z/units::cm;
     ptree.reco_dQ = vtx->get_dQ() * dQdx_scale + dQdx_offset;
     ptree.reco_dx = vtx->get_dx()/units::cm;
     ptree.reco_pu = vtx->get_pu();
     ptree.reco_pv = vtx->get_pv();
     ptree.reco_pw = vtx->get_pw();
     ptree.reco_pt = vtx->get_pt();    
     ptree.reco_reduced_chi2 = vtx->get_reduced_chi2();

     
     // std::cout << vtx->get_cluster_id() << " " << vtx->get_id() << " " << vtx->get_fit_pt() << " " << shower->get_start_segment()->get_cluster_id() << " " << shower->get_start_segment()->get_id() << std::endl;
     
     // std::cout <<  ptree.reco_proto_cluster_id << std::endl;
     // std::cout << shower->get_start_segment()->get_length()/units::cm << " " << vtx << " " << shower << std::endl;
     if ( vtx->get_dQ()!=0)
       T->Fill();
  }
  
  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *seg = it->first;

    std::vector<WCP::Point>& pts = seg->get_point_vec();
    std::vector<double>& dQ_vec = seg->get_dQ_vec();
    std::vector<double>& dx_vec = seg->get_dx_vec();
    std::vector<double>& pu_vec = seg->get_pu_vec();
    std::vector<double>& pv_vec = seg->get_pv_vec();
    std::vector<double>& pw_vec = seg->get_pw_vec();
    std::vector<double>& pt_vec = seg->get_pt_vec();
    std::vector<double>& reduced_chi2_vec = seg->get_reduced_chi2_vec();

    ptree.reco_cluster_id = seg->get_cluster_id();
    ptree.reco_ndf = seg->get_cluster_id();
    
    auto it1 = map_segment_in_shower.find(seg);
    if (it1!=map_segment_in_shower.end()){
      WCPPID::WCShower *shower = it1->second;
      
      ptree.reco_proto_cluster_id = shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id();
      if (shower->get_flag_shower()){
	ptree.reco_flag_track_shower = 1;
      }else{
	ptree.reco_flag_track_shower = 0;
      }
      ptree.reco_particle_id = shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id();
      ptree.reco_flag_vertex = 0;

      for (size_t i=1; i+1<pts.size();i++){
	ptree.reco_x = pts.at(i).x/units::cm;
	ptree.reco_y = pts.at(i).y/units::cm;
	ptree.reco_z = pts.at(i).z/units::cm;
	ptree.reco_dQ = dQ_vec.at(i) * dQdx_scale + dQdx_offset;
	ptree.reco_dx = dx_vec.at(i)/units::cm;
	ptree.reco_pu = pu_vec.at(i);
	ptree.reco_pv = pv_vec.at(i);
	ptree.reco_pw = pw_vec.at(i);
	ptree.reco_pt = pt_vec.at(i);    
	ptree.reco_reduced_chi2 = reduced_chi2_vec.at(i);
	T->Fill();
      }

          
    }else{
      
      ptree.reco_proto_cluster_id = seg->get_cluster_id()*1000 + seg->get_id();

      if (seg->get_flag_shower()){
	ptree.reco_flag_track_shower = 1;
      }else{
	ptree.reco_flag_track_shower = 0;
      }
      
      ptree.reco_particle_id = ptree.reco_particle_id = seg->get_cluster_id()*1000 + seg->get_id();
      
      ptree.reco_flag_vertex = 0;
      for (size_t i=0;i!=pts.size();i++){
	ptree.reco_x = pts.at(i).x/units::cm;
	ptree.reco_y = pts.at(i).y/units::cm;
	ptree.reco_z = pts.at(i).z/units::cm;
	ptree.reco_dQ = dQ_vec.at(i) * dQdx_scale + dQdx_offset;
	ptree.reco_dx = dx_vec.at(i)/units::cm;
	ptree.reco_pu = pu_vec.at(i);
	ptree.reco_pv = pv_vec.at(i);
	ptree.reco_pw = pw_vec.at(i);
	ptree.reco_pt = pt_vec.at(i);    
	ptree.reco_reduced_chi2 = reduced_chi2_vec.at(i);
	T->Fill();
      }
    }
    
  }
  
  
}


void WCPPID::NeutrinoID::fill_point_info(int mother_cluster_id, WCPPID::WCPointTree& ptree, TTree *T){
  
  ptree.reco_mother_cluster_id = mother_cluster_id;
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *seg = it->first;

    auto it1 = map_segment_in_shower.find(seg);
    if (it1 != map_segment_in_shower.end()){
      ptree.reco_proto_cluster_id = (it1->second)->get_start_segment()->get_cluster_id() * 1000 + (it1->second)->get_start_segment()->get_id();
      if (it1->second->get_flag_shower()){
	ptree.reco_flag_track_shower = 1;
	ptree.reco_flag_track_shower_charge = 15000;
      }else{
	ptree.reco_flag_track_shower = 0;
	ptree.reco_flag_track_shower_charge = 0;
      }
    }else{
      ptree.reco_proto_cluster_id = seg->get_cluster_id() * 1000 + seg->get_id();
      if (seg->get_flag_shower()){
	ptree.reco_flag_track_shower = 1;
	ptree.reco_flag_track_shower_charge = 15000;
      }else{
	ptree.reco_flag_track_shower = 0;
	ptree.reco_flag_track_shower_charge = 0;
      }
    }
    ptree.reco_cluster_id = seg->get_cluster_id();
    
    ToyPointCloud *pcloud =  seg->get_associated_pcloud();
    if (pcloud != 0){
      auto cloud = pcloud->get_cloud();
      //    std::cout << cloud.pts.size() << std::endl;
      for (size_t i=0;i!=cloud.pts.size();i++){
	ptree.reco_x = cloud.pts.at(i).x/units::cm;
	ptree.reco_y = cloud.pts.at(i).y/units::cm;
	ptree.reco_z = cloud.pts.at(i).z/units::cm;
	T->Fill();
      }
    }
    
  }
}
    


void WCPPID::NeutrinoID::init_tagger_info(){
  // initialize cosmic one
  tagger_info.cosmic_flag = true ;
  tagger_info.cosmic_n_solid_tracks = 0;
  tagger_info.cosmic_energy_main_showers = 0;
  tagger_info.cosmic_energy_direct_showers = 0;
  tagger_info.cosmic_energy_indirect_showers = 0;
  tagger_info.cosmic_n_direct_showers = 0;
  tagger_info.cosmic_n_indirect_showers = 0;
  tagger_info.cosmic_n_main_showers = 0;
  tagger_info.cosmic_filled = 0;

  // std::cout << "haha: " << tagger_info.cosmic_flag << std::endl;
  
  // shower gap identification
  tagger_info.gap_flag = true;
  tagger_info.gap_flag_prolong_u = false;
  tagger_info.gap_flag_prolong_v = false;
  tagger_info.gap_flag_prolong_w = false;
  tagger_info.gap_flag_parallel = false;
  tagger_info.gap_n_points = 0;
  tagger_info.gap_n_bad = 0;
  tagger_info.gap_energy = 0;
  tagger_info.gap_num_valid_tracks = 0;
  tagger_info.gap_flag_single_shower = false;
  tagger_info.gap_filled = 0;

  // shower MIP quality
  tagger_info.mip_quality_flag = true;
  tagger_info.mip_quality_energy = 0;
  tagger_info.mip_quality_overlap = false;
  tagger_info.mip_quality_n_showers = 0;
  tagger_info.mip_quality_n_tracks = 0;
  tagger_info.mip_quality_flag_inside_pi0 = false;
  tagger_info.mip_quality_n_pi0_showers = 0;
  tagger_info.mip_quality_shortest_length = 0;
  tagger_info.mip_quality_acc_length = 0;
  tagger_info.mip_quality_shortest_angle = 0;
  tagger_info.mip_quality_flag_proton = false;
  tagger_info.mip_quality_filled = 0;

  // mip identification
  tagger_info.mip_flag = true;
  tagger_info.mip_n_first_non_mip_2 = 19;
  tagger_info.mip_n_first_mip = 0;
  tagger_info.mip_n_end_reduction = 0;
  tagger_info.mip_n_first_non_mip_1 = 19;
  tagger_info.mip_n_first_non_mip = 19;
  tagger_info.mip_energy = 0;
  tagger_info.mip_max_dQ_dx_sample = 1;
  tagger_info.mip_vec_dQ_dx_0 = 1;
  tagger_info.mip_vec_dQ_dx_1 = 1;
  tagger_info.mip_n_below_threshold = 19;
  tagger_info.mip_n_good_tracks = 0;
  tagger_info.mip_n_vertex = 1;
  tagger_info.mip_angle_beam = 0;
  tagger_info.mip_flag_all_above = false;
  tagger_info.mip_length_main = 1;
  tagger_info.mip_length_total = 1;
  tagger_info.mip_min_dQ_dx_5 = 1;
  tagger_info.mip_lowest_dQ_dx = 1;
  tagger_info.mip_iso_angle = 0;
  tagger_info.mip_n_below_zero = 0;
  tagger_info.mip_highest_dQ_dx = 1;
  tagger_info.mip_n_lowest = 1;
  tagger_info.mip_n_highest = 1;
  tagger_info.mip_stem_length = 1;
  tagger_info.mip_E_indirect_max_energy = 0;
  tagger_info.mip_flag_stem_trajectory = 0;
  tagger_info.mip_min_dis = 0;
  tagger_info.mip_n_other_vertex = 2;
  tagger_info.mip_n_stem_size = 20;
  tagger_info.mip_medium_dQ_dx = 1;
  tagger_info.mip_filled = 0;

  tagger_info.mip_vec_dQ_dx_2 = 0;
  tagger_info.mip_vec_dQ_dx_3 = 0;
  tagger_info.mip_vec_dQ_dx_4 = 0;
  tagger_info.mip_vec_dQ_dx_5 = 0;
  tagger_info.mip_vec_dQ_dx_6 = 0; 
  tagger_info.mip_vec_dQ_dx_7 = 0; 
  tagger_info.mip_vec_dQ_dx_8 = 0; 
  tagger_info.mip_vec_dQ_dx_9 = 0; 
  tagger_info.mip_vec_dQ_dx_10 = 0; 
  tagger_info.mip_vec_dQ_dx_11 = 0; 
  tagger_info.mip_vec_dQ_dx_12 = 0; 
  tagger_info.mip_vec_dQ_dx_13 = 0; 
  tagger_info.mip_vec_dQ_dx_14 = 0; 
  tagger_info.mip_vec_dQ_dx_15 = 0; 
  tagger_info.mip_vec_dQ_dx_16 = 0; 
  tagger_info.mip_vec_dQ_dx_17 = 0; 
  tagger_info.mip_vec_dQ_dx_18 = 0; 
  tagger_info.mip_vec_dQ_dx_19 = 0; 

  //single track kdar tagger
  tagger_info.ssm_flag_st_kdar = false;
  tagger_info.ssm_Nsm = -999;//number of short straight muons
  tagger_info.ssm_Nsm_wivtx = -999;//number of short straight muons with vertex activity
  tagger_info.ssm_dq_dx_fwd_1 = -999;
  tagger_info.ssm_dq_dx_fwd_2 = -999;
  tagger_info.ssm_dq_dx_fwd_3 = -999;
  tagger_info.ssm_dq_dx_fwd_4 = -999;
  tagger_info.ssm_dq_dx_fwd_5 = -999;
  tagger_info.ssm_dq_dx_bck_1 = -999;
  tagger_info.ssm_dq_dx_bck_2 = -999;
  tagger_info.ssm_dq_dx_bck_3 = -999;
  tagger_info.ssm_dq_dx_bck_4 = -999;
  tagger_info.ssm_dq_dx_bck_5 = -999;
  tagger_info.ssm_d_dq_dx_fwd_12 = -999;
  tagger_info.ssm_d_dq_dx_fwd_23 = -999;
  tagger_info.ssm_d_dq_dx_fwd_34 = -999;
  tagger_info.ssm_d_dq_dx_fwd_45 = -999;
  tagger_info.ssm_d_dq_dx_bck_12 = -999;
  tagger_info.ssm_d_dq_dx_bck_23 = -999;
  tagger_info.ssm_d_dq_dx_bck_34 = -999;
  tagger_info.ssm_d_dq_dx_bck_45 = -999;
  tagger_info.ssm_max_dq_dx_fwd_3 = -999;
  tagger_info.ssm_max_dq_dx_fwd_5 = -999;
  tagger_info.ssm_max_dq_dx_bck_3 = -999;
  tagger_info.ssm_max_dq_dx_bck_5 = -999;
  tagger_info.ssm_max_d_dq_dx_fwd_3 = -999;
  tagger_info.ssm_max_d_dq_dx_fwd_5 = -999;
  tagger_info.ssm_max_d_dq_dx_bck_3 = -999;
  tagger_info.ssm_max_d_dq_dx_bck_5 = -999;
  tagger_info.ssm_medium_dq_dx = -999;
  tagger_info.ssm_medium_dq_dx_bp = -999;
      //angluar info
  tagger_info.ssm_angle_to_z = -999;
  tagger_info.ssm_angle_to_target = -999;
  tagger_info.ssm_angle_to_absorber = -999;
  tagger_info.ssm_angle_to_vertical = -999;
      //directional info
  tagger_info.ssm_x_dir = -999;
  tagger_info.ssm_y_dir = -999;
  tagger_info.ssm_z_dir = -999;
      //energy info
  tagger_info.ssm_kine_energy = -999;
  tagger_info.ssm_kine_energy_reduced = -999;
      //general properties
  tagger_info.ssm_vtx_activity = -999;
  tagger_info.ssm_pdg = -999;
  tagger_info.ssm_dQ_dx_cut = -999;
  tagger_info.ssm_score_mu_fwd = -999;
  tagger_info.ssm_score_p_fwd = -999;
  tagger_info.ssm_score_e_fwd = -999;
  tagger_info.ssm_score_mu_bck = -999;
  tagger_info.ssm_score_p_bck = -999;
  tagger_info.ssm_score_e_bck = -999;
  tagger_info.ssm_score_mu_fwd_bp = -999;
  tagger_info.ssm_score_p_fwd_bp = -999;
  tagger_info.ssm_score_e_fwd_bp = -999;
      //track "straighness"
  tagger_info.ssm_length = -999;
  tagger_info.ssm_direct_length = -999;
  tagger_info.ssm_length_ratio = -999;
  tagger_info.ssm_max_dev = -999;
    //number of other particles
  tagger_info.ssm_n_prim_tracks_1 = -999;
  tagger_info.ssm_n_prim_tracks_3 = -999;
  tagger_info.ssm_n_prim_tracks_5 = -999;
  tagger_info.ssm_n_prim_tracks_8 = -999;
  tagger_info.ssm_n_prim_tracks_11 = -999;
  tagger_info.ssm_n_all_tracks_1 = -999;
  tagger_info.ssm_n_all_tracks_3 = -999;
  tagger_info.ssm_n_all_tracks_5 = -999;
  tagger_info.ssm_n_all_tracks_8 = -999;
  tagger_info.ssm_n_all_tracks_11 = -999;
  tagger_info.ssm_n_daughter_tracks_1 = -999;
  tagger_info.ssm_n_daughter_tracks_3 = -999;
  tagger_info.ssm_n_daughter_tracks_5 = -999;
  tagger_info.ssm_n_daughter_tracks_8 = -999;
  tagger_info.ssm_n_daughter_tracks_11 = -999;
  tagger_info.ssm_n_daughter_all_1 = -999;
  tagger_info.ssm_n_daughter_all_3 = -999;
  tagger_info.ssm_n_daughter_all_5 = -999;
  tagger_info.ssm_n_daughter_all_8 = -999;
  tagger_info.ssm_n_daughter_all_11 = -999;
    //properties of leading other primary track
  tagger_info.ssm_prim_track1_pdg = -999;
  tagger_info.ssm_prim_track1_score_mu_fwd = -999;
  tagger_info.ssm_prim_track1_score_p_fwd = -999;
  tagger_info.ssm_prim_track1_score_e_fwd = -999;
  tagger_info.ssm_prim_track1_score_mu_bck = -999;
  tagger_info.ssm_prim_track1_score_p_bck = -999;
  tagger_info.ssm_prim_track1_score_e_bck = -999;
  tagger_info.ssm_prim_track1_length = -999;
  tagger_info.ssm_prim_track1_direct_length = -999;
  tagger_info.ssm_prim_track1_length_ratio = -999;
  tagger_info.ssm_prim_track1_max_dev = -999;
  tagger_info.ssm_prim_track1_kine_energy_range = -999;
  tagger_info.ssm_prim_track1_kine_energy_range_mu = -999;
  tagger_info.ssm_prim_track1_kine_energy_range_p = -999;
  tagger_info.ssm_prim_track1_kine_energy_range_e = -999;
  tagger_info.ssm_prim_track1_kine_energy_cal = -999;
  tagger_info.ssm_prim_track1_medium_dq_dx = -999;
  tagger_info.ssm_prim_track1_x_dir = -999;
  tagger_info.ssm_prim_track1_y_dir = -999;
  tagger_info.ssm_prim_track1_z_dir = -999;
  tagger_info.ssm_prim_track1_add_daught_track_counts_1 = -999;
  tagger_info.ssm_prim_track1_add_daught_all_counts_1 = -999;
  tagger_info.ssm_prim_track1_add_daught_track_counts_5 = -999;
  tagger_info.ssm_prim_track1_add_daught_all_counts_5 = -999; 
  tagger_info.ssm_prim_track1_add_daught_track_counts_11 = -999;
  tagger_info.ssm_prim_track1_add_daught_all_counts_11 = -999;
  //properties of sub-leading other primary track
  tagger_info.ssm_prim_track2_pdg = -999;
  tagger_info.ssm_prim_track2_score_mu_fwd = -999;
  tagger_info.ssm_prim_track2_score_p_fwd = -999;
  tagger_info.ssm_prim_track2_score_e_fwd = -999;
  tagger_info.ssm_prim_track2_score_mu_bck = -999;
  tagger_info.ssm_prim_track2_score_p_bck = -999;
  tagger_info.ssm_prim_track2_score_e_bck = -999;
  tagger_info.ssm_prim_track2_length = -999;
  tagger_info.ssm_prim_track2_direct_length = -999;
  tagger_info.ssm_prim_track2_length_ratio = -999;
  tagger_info.ssm_prim_track2_max_dev = -999;
  tagger_info.ssm_prim_track2_kine_energy_range = -999;
  tagger_info.ssm_prim_track2_kine_energy_range_mu = -999;
  tagger_info.ssm_prim_track2_kine_energy_range_p = -999;
  tagger_info.ssm_prim_track2_kine_energy_range_e = -999;
  tagger_info.ssm_prim_track2_kine_energy_cal = -999;
  tagger_info.ssm_prim_track2_medium_dq_dx = -999;
  tagger_info.ssm_prim_track2_x_dir = -999;
  tagger_info.ssm_prim_track2_y_dir = -999;
  tagger_info.ssm_prim_track2_z_dir = -999;
  tagger_info.ssm_prim_track2_add_daught_track_counts_1 = -999;
  tagger_info.ssm_prim_track2_add_daught_all_counts_1 = -999;
  tagger_info.ssm_prim_track2_add_daught_track_counts_5 = -999;
  tagger_info.ssm_prim_track2_add_daught_all_counts_5 = -999;
  tagger_info.ssm_prim_track2_add_daught_track_counts_11 = -999;
  tagger_info.ssm_prim_track2_add_daught_all_counts_11 = -999;
  //properties of leading daughter track
  tagger_info.ssm_daught_track1_pdg = -999;
  tagger_info.ssm_daught_track1_score_mu_fwd = -999;
  tagger_info.ssm_daught_track1_score_p_fwd = -999;
  tagger_info.ssm_daught_track1_score_e_fwd = -999;
  tagger_info.ssm_daught_track1_score_mu_bck = -999;
  tagger_info.ssm_daught_track1_score_p_bck = -999;
  tagger_info.ssm_daught_track1_score_e_bck = -999;
  tagger_info.ssm_daught_track1_length = -999;
  tagger_info.ssm_daught_track1_direct_length = -999;
  tagger_info.ssm_daught_track1_length_ratio = -999;
  tagger_info.ssm_daught_track1_max_dev = -999;
  tagger_info.ssm_daught_track1_kine_energy_range = -999;
  tagger_info.ssm_daught_track1_kine_energy_range_mu = -999;
  tagger_info.ssm_daught_track1_kine_energy_range_p = -999;
  tagger_info.ssm_daught_track1_kine_energy_range_e = -999;
  tagger_info.ssm_daught_track1_kine_energy_cal = -999;
  tagger_info.ssm_daught_track1_medium_dq_dx = -999;
  tagger_info.ssm_daught_track1_x_dir = -999;
  tagger_info.ssm_daught_track1_y_dir = -999;
  tagger_info.ssm_daught_track1_z_dir = -999;
  tagger_info.ssm_daught_track1_add_daught_track_counts_1 = -999;
  tagger_info.ssm_daught_track1_add_daught_all_counts_1 = -999;
  tagger_info.ssm_daught_track1_add_daught_track_counts_5 = -999;
  tagger_info.ssm_daught_track1_add_daught_all_counts_5 = -999;
  tagger_info.ssm_daught_track1_add_daught_track_counts_11 = -999;
  tagger_info.ssm_daught_track1_add_daught_all_counts_11 = -999;
  //properties of sub-leading daughter track
  tagger_info.ssm_daught_track2_pdg = -999;
  tagger_info.ssm_daught_track2_score_mu_fwd = -999;
  tagger_info.ssm_daught_track2_score_p_fwd = -999;
  tagger_info.ssm_daught_track2_score_e_fwd = -999;
  tagger_info.ssm_daught_track2_score_mu_bck = -999;
  tagger_info.ssm_daught_track2_score_p_bck = -999;
  tagger_info.ssm_daught_track2_score_e_bck = -999;
  tagger_info.ssm_daught_track2_length = -999;
  tagger_info.ssm_daught_track2_direct_length = -999;
  tagger_info.ssm_daught_track2_length_ratio = -999;
  tagger_info.ssm_daught_track2_max_dev = -999;
  tagger_info.ssm_daught_track2_kine_energy_range = -999;
  tagger_info.ssm_daught_track2_kine_energy_range_mu = -999;
  tagger_info.ssm_daught_track2_kine_energy_range_p = -999;
  tagger_info.ssm_daught_track2_kine_energy_range_e = -999;
  tagger_info.ssm_daught_track2_kine_energy_cal = -999;
  tagger_info.ssm_daught_track2_medium_dq_dx = -999;
  tagger_info.ssm_daught_track2_x_dir = -999;
  tagger_info.ssm_daught_track2_y_dir = -999;
  tagger_info.ssm_daught_track2_z_dir = -999;
  tagger_info.ssm_daught_track2_add_daught_track_counts_1 = -999;
  tagger_info.ssm_daught_track2_add_daught_all_counts_1 = -999;
  tagger_info.ssm_daught_track2_add_daught_track_counts_5 = -999;
  tagger_info.ssm_daught_track2_add_daught_all_counts_5 = -999;
  tagger_info.ssm_daught_track2_add_daught_track_counts_11 = -999;
  tagger_info.ssm_daught_track2_add_daught_all_counts_11 = -999;
  //properties of leading other primary shower
  tagger_info.ssm_prim_shw1_pdg = -999;
  tagger_info.ssm_prim_shw1_score_mu_fwd = -999;
  tagger_info.ssm_prim_shw1_score_p_fwd = -999;
  tagger_info.ssm_prim_shw1_score_e_fwd = -999;
  tagger_info.ssm_prim_shw1_score_mu_bck = -999;
  tagger_info.ssm_prim_shw1_score_p_bck = -999;
  tagger_info.ssm_prim_shw1_score_e_bck = -999;
  tagger_info.ssm_prim_shw1_length = -999;
  tagger_info.ssm_prim_shw1_direct_length = -999;
  tagger_info.ssm_prim_shw1_length_ratio = -999;
  tagger_info.ssm_prim_shw1_max_dev = -999;
  tagger_info.ssm_prim_shw1_kine_energy_range = -999;
  tagger_info.ssm_prim_shw1_kine_energy_range_mu = -999;
  tagger_info.ssm_prim_shw1_kine_energy_range_p = -999;
  tagger_info.ssm_prim_shw1_kine_energy_range_e = -999;
  tagger_info.ssm_prim_shw1_kine_energy_cal = -999;
  tagger_info.ssm_prim_shw1_kine_energy_best = -999;
  tagger_info.ssm_prim_shw1_medium_dq_dx = -999;
  tagger_info.ssm_prim_shw1_x_dir = -999;
  tagger_info.ssm_prim_shw1_y_dir = -999;
  tagger_info.ssm_prim_shw1_z_dir = -999;
  tagger_info.ssm_prim_shw1_add_daught_track_counts_1 = -999;
  tagger_info.ssm_prim_shw1_add_daught_all_counts_1 = -999;
  tagger_info.ssm_prim_shw1_add_daught_track_counts_5 = -999;
  tagger_info.ssm_prim_shw1_add_daught_all_counts_5 = -999;
  tagger_info.ssm_prim_shw1_add_daught_track_counts_11 = -999;
  tagger_info.ssm_prim_shw1_add_daught_all_counts_11 = -999;
    //properties of sub-leading other primary shower
  tagger_info.ssm_prim_shw2_pdg = -999;
  tagger_info.ssm_prim_shw2_score_mu_fwd = -999;
  tagger_info.ssm_prim_shw2_score_p_fwd = -999;
  tagger_info.ssm_prim_shw2_score_e_fwd = -999;
  tagger_info.ssm_prim_shw2_score_mu_bck = -999;
  tagger_info.ssm_prim_shw2_score_p_bck = -999;
  tagger_info.ssm_prim_shw2_score_e_bck = -999;
  tagger_info.ssm_prim_shw2_length = -999;
  tagger_info.ssm_prim_shw2_direct_length = -999;
  tagger_info.ssm_prim_shw2_length_ratio = -999;
  tagger_info.ssm_prim_shw2_max_dev = -999;
  tagger_info.ssm_prim_shw2_kine_energy_range = -999;
  tagger_info.ssm_prim_shw2_kine_energy_range_mu = -999;
  tagger_info.ssm_prim_shw2_kine_energy_range_p = -999;
  tagger_info.ssm_prim_shw2_kine_energy_range_e = -999;
  tagger_info.ssm_prim_shw2_kine_energy_cal = -999;
  tagger_info.ssm_prim_shw2_kine_energy_best = -999;
  tagger_info.ssm_prim_shw2_medium_dq_dx = -999;
  tagger_info.ssm_prim_shw2_x_dir = -999;
  tagger_info.ssm_prim_shw2_y_dir = -999;
  tagger_info.ssm_prim_shw2_z_dir = -999;
  tagger_info.ssm_prim_shw2_add_daught_track_counts_1 = -999;
  tagger_info.ssm_prim_shw2_add_daught_all_counts_1 = -999;
  tagger_info.ssm_prim_shw2_add_daught_track_counts_5 = -999;
  tagger_info.ssm_prim_shw2_add_daught_all_counts_5 = -999;
  tagger_info.ssm_prim_shw2_add_daught_track_counts_11 = -999;
  tagger_info.ssm_prim_shw2_add_daught_all_counts_11 = -999;
  //properties of leading daughter shower
  tagger_info.ssm_daught_shw1_pdg = -999;
  tagger_info.ssm_daught_shw1_score_mu_fwd = -999;
  tagger_info.ssm_daught_shw1_score_p_fwd = -999;
  tagger_info.ssm_daught_shw1_score_e_fwd = -999;
  tagger_info.ssm_daught_shw1_score_mu_bck = -999;
  tagger_info.ssm_daught_shw1_score_p_bck = -999;
  tagger_info.ssm_daught_shw1_score_e_bck = -999;
  tagger_info.ssm_daught_shw1_length = -999;
  tagger_info.ssm_daught_shw1_direct_length = -999;
  tagger_info.ssm_daught_shw1_length_ratio = -999;
  tagger_info.ssm_daught_shw1_max_dev = -999;
  tagger_info.ssm_daught_shw1_kine_energy_range = -999;
  tagger_info.ssm_daught_shw1_kine_energy_range_mu = -999;
  tagger_info.ssm_daught_shw1_kine_energy_range_p = -999;
  tagger_info.ssm_daught_shw1_kine_energy_range_e = -999;
  tagger_info.ssm_daught_shw1_kine_energy_cal = -999;
  tagger_info.ssm_daught_shw1_kine_energy_best = -999;
  tagger_info.ssm_daught_shw1_medium_dq_dx = -999;
  tagger_info.ssm_daught_shw1_x_dir = -999;
  tagger_info.ssm_daught_shw1_y_dir = -999;
  tagger_info.ssm_daught_shw1_z_dir = -999;
  tagger_info.ssm_daught_shw1_add_daught_track_counts_1 = -999;
  tagger_info.ssm_daught_shw1_add_daught_all_counts_1 = -999;
  tagger_info.ssm_daught_shw1_add_daught_track_counts_5 = -999;
  tagger_info.ssm_daught_shw1_add_daught_all_counts_5 = -999;
  tagger_info.ssm_daught_shw1_add_daught_track_counts_11 = -999;
  tagger_info.ssm_daught_shw1_add_daught_all_counts_11 = -999;
    //properties of sub-leading daughter shower
  tagger_info.ssm_daught_shw2_pdg = -999;
  tagger_info.ssm_daught_shw2_score_mu_fwd = -999;
  tagger_info.ssm_daught_shw2_score_p_fwd = -999;
  tagger_info.ssm_daught_shw2_score_e_fwd = -999;
  tagger_info.ssm_daught_shw2_score_mu_bck = -999;
  tagger_info.ssm_daught_shw2_score_p_bck = -999;
  tagger_info.ssm_daught_shw2_score_e_bck = -999;
  tagger_info.ssm_daught_shw2_length = -999;
  tagger_info.ssm_daught_shw2_direct_length = -999;
  tagger_info.ssm_daught_shw2_length_ratio = -999;
  tagger_info.ssm_daught_shw2_max_dev = -999;
  tagger_info.ssm_daught_shw2_kine_energy_range = -999;
  tagger_info.ssm_daught_shw2_kine_energy_range_mu = -999;
  tagger_info.ssm_daught_shw2_kine_energy_range_p = -999;
  tagger_info.ssm_daught_shw2_kine_energy_range_e = -999;
  tagger_info.ssm_daught_shw2_kine_energy_cal = -999;
  tagger_info.ssm_daught_shw2_kine_energy_best = -999;
  tagger_info.ssm_daught_shw2_medium_dq_dx = -999;
  tagger_info.ssm_daught_shw2_x_dir = -999;
  tagger_info.ssm_daught_shw2_y_dir = -999;
  tagger_info.ssm_daught_shw2_z_dir = -999;
  tagger_info.ssm_daught_shw2_add_daught_track_counts_1 = -999;
  tagger_info.ssm_daught_shw2_add_daught_all_counts_1 = -999;
  tagger_info.ssm_daught_shw2_add_daught_track_counts_5 = -999;
  tagger_info.ssm_daught_shw2_add_daught_all_counts_5 = -999;
  tagger_info.ssm_daught_shw2_add_daught_track_counts_11 = -999;
  tagger_info.ssm_daught_shw2_add_daught_all_counts_11 = -999;
  //event level properties
  tagger_info.ssm_nu_angle_z = -999;
  tagger_info.ssm_nu_angle_target = -999;
  tagger_info.ssm_nu_angle_absorber = -999;
  tagger_info.ssm_nu_angle_vertical = -999;
  tagger_info.ssm_con_nu_angle_z = -999;
  tagger_info.ssm_con_nu_angle_target = -999;
  tagger_info.ssm_con_nu_angle_absorber = -999;
  tagger_info.ssm_con_nu_angle_vertical = -999;
  tagger_info.ssm_prim_nu_angle_z = -999;
  tagger_info.ssm_prim_nu_angle_target = -999;
  tagger_info.ssm_prim_nu_angle_absorber = -999;
  tagger_info.ssm_prim_nu_angle_vertical = -999;
  tagger_info.ssm_track_angle_z = -999;
  tagger_info.ssm_track_angle_target = -999;
  tagger_info.ssm_track_angle_absorber = -999;
  tagger_info.ssm_track_angle_vertical = -999;
  tagger_info.ssm_vtxX = -999;
  tagger_info.ssm_vtxY = -999;
  tagger_info.ssm_vtxZ = -999;
  //off vertex stuff
  tagger_info.ssm_offvtx_length  =  -999;
  tagger_info.ssm_offvtx_energy  =  -999;
  tagger_info.ssm_n_offvtx_tracks_1  =  -999;
  tagger_info.ssm_n_offvtx_tracks_3  =  -999;
  tagger_info.ssm_n_offvtx_tracks_5  =  -999;
  tagger_info.ssm_n_offvtx_tracks_8  =  -999;
  tagger_info.ssm_n_offvtx_tracks_11  =  -999;
  tagger_info.ssm_n_offvtx_showers_1  =  -999;
  tagger_info.ssm_n_offvtx_showers_3  =  -999;
  tagger_info.ssm_n_offvtx_showers_5  =  -999;
  tagger_info.ssm_n_offvtx_showers_8  =  -999;
  tagger_info.ssm_n_offvtx_showers_11  =  -999;
   //properties of leading off vertex track
  tagger_info.ssm_offvtx_track1_pdg  =  -999;
  tagger_info.ssm_offvtx_track1_score_mu_fwd  =  -999;
  tagger_info.ssm_offvtx_track1_score_p_fwd  =  -999;
  tagger_info.ssm_offvtx_track1_score_e_fwd  =  -999;
  tagger_info.ssm_offvtx_track1_score_mu_bck  =  -999;
  tagger_info.ssm_offvtx_track1_score_p_bck  =  -999;
  tagger_info.ssm_offvtx_track1_score_e_bck  =  -999;
  tagger_info.ssm_offvtx_track1_length  =  -999;
  tagger_info.ssm_offvtx_track1_direct_length  =  -999;
  tagger_info.ssm_offvtx_track1_max_dev  =  -999;
  tagger_info.ssm_offvtx_track1_kine_energy_range  =  -999;
  tagger_info.ssm_offvtx_track1_kine_energy_range_mu  = -999;
  tagger_info.ssm_offvtx_track1_kine_energy_range_p  =  -999;
  tagger_info.ssm_offvtx_track1_kine_energy_range_e  =  -999;
  tagger_info.ssm_offvtx_track1_kine_energy_cal  =  -999;
  tagger_info.ssm_offvtx_track1_medium_dq_dx  =  -999;
  tagger_info.ssm_offvtx_track1_x_dir  =  -999;
  tagger_info.ssm_offvtx_track1_y_dir  =  -999;
  tagger_info.ssm_offvtx_track1_z_dir  =  -999;
  tagger_info.ssm_offvtx_track1_dist_mainvtx  =  -999;
   //properties of leading off vertex shower
  tagger_info.ssm_offvtx_shw1_pdg_offvtx  =  -999;
  tagger_info.ssm_offvtx_shw1_score_mu_fwd  = -999;
  tagger_info.ssm_offvtx_shw1_score_p_fwd  =  -999;
  tagger_info.ssm_offvtx_shw1_score_e_fwd  =  -999;
  tagger_info.ssm_offvtx_shw1_score_mu_bck  =  -999;
  tagger_info.ssm_offvtx_shw1_score_p_bck  =  -999;
  tagger_info.ssm_offvtx_shw1_score_e_bck  =  -999;
  tagger_info.ssm_offvtx_shw1_length  =  -999;
  tagger_info.ssm_offvtx_shw1_direct_length  =  -999;
  tagger_info.ssm_offvtx_shw1_max_dev  =  -999;
  tagger_info.ssm_offvtx_shw1_kine_energy_best  =  -999;
  tagger_info.ssm_offvtx_shw1_kine_energy_range  =  -999;
  tagger_info.ssm_offvtx_shw1_kine_energy_range_mu  =  -999;
  tagger_info.ssm_offvtx_shw1_kine_energy_range_p  =  -999;
  tagger_info.ssm_offvtx_shw1_kine_energy_range_e  =  -999;
  tagger_info.ssm_offvtx_shw1_kine_energy_cal  =  -999;
  tagger_info.ssm_offvtx_shw1_medium_dq_dx  =  -999;
  tagger_info.ssm_offvtx_shw1_x_dir  =  -999;
  tagger_info.ssm_offvtx_shw1_y_dir  =  -999;
  tagger_info.ssm_offvtx_shw1_z_dir  =  -999;
  tagger_info.ssm_offvtx_shw1_dist_mainvtx  =  -999;
  // Spacepoints
  tagger_info.ssmsp_Ntrack = 0;
  tagger_info.ssmsp_Nsp_tot = 0;
  // Kine vars
  tagger_info.ssm_kine_reco_Enu = -999; // kinetic energy  + additional energy ...
  tagger_info.ssm_kine_reco_add_energy = -999;  // mass, binding energy ...
  tagger_info.ssm_kine_pio_mass = -999; // mass
  tagger_info.ssm_kine_pio_flag = -999; // 0 not filled, 1, with vertex, 2 without vertex
  tagger_info.ssm_kine_pio_vtx_dis = -999;
  tagger_info.ssm_kine_pio_energy_1 = -999;
  tagger_info.ssm_kine_pio_theta_1 = -999;
  tagger_info.ssm_kine_pio_phi_1 = -999;
  tagger_info.ssm_kine_pio_dis_1 = -999;
  tagger_info.ssm_kine_pio_energy_2 = -999;
  tagger_info.ssm_kine_pio_theta_2 = -999;
  tagger_info.ssm_kine_pio_phi_2 = -999;
  tagger_info.ssm_kine_pio_dis_2 = -999;
  tagger_info.ssm_kine_pio_angle = -999;

  // single photon shower identification
  tagger_info.shw_sp_flag = true;
  tagger_info.shw_sp_num_mip_tracks = 0;
  tagger_info.shw_sp_num_muons = 0;
  tagger_info.shw_sp_num_pions = 0;
  tagger_info.shw_sp_num_protons = 0;
  tagger_info.shw_sp_proton_length_1 = -1;
  tagger_info.shw_sp_proton_dqdx_1 = -1;
  tagger_info.shw_sp_proton_energy_1 = -1;
  tagger_info.shw_sp_proton_length_2 = -1;
  tagger_info.shw_sp_proton_dqdx_2 = -1;
  tagger_info.shw_sp_proton_energy_2 = -1;
  tagger_info.shw_sp_n_good_showers = 0;
  tagger_info.shw_sp_n_20mev_showers = 0;
  tagger_info.shw_sp_n_br1_showers = 0;
  tagger_info.shw_sp_n_br2_showers = 0;
  tagger_info.shw_sp_n_br3_showers = 0;
  tagger_info.shw_sp_n_br4_showers = 0;
  tagger_info.shw_sp_n_20mev_showers = 0;
  tagger_info.shw_sp_n_br1_showers = 0;
  tagger_info.shw_sp_n_br2_showers = 0;
  tagger_info.shw_sp_n_br3_showers = 0;
  tagger_info.shw_sp_n_br4_showers = 0;
  tagger_info.shw_sp_n_20br1_showers = 0;
  tagger_info.shw_sp_shw_vtx_dis = -1;
  tagger_info.shw_sp_max_shw_dis = -1;
  tagger_info.shw_sp_energy = 0;
  tagger_info.shw_sp_max_dQ_dx_sample = 1;
  tagger_info.shw_sp_vec_dQ_dx_0 = 1;
  tagger_info.shw_sp_vec_dQ_dx_1 = 1;
  tagger_info.shw_sp_n_below_threshold = 19;
  tagger_info.shw_sp_n_good_tracks = 0;
  tagger_info.shw_sp_n_vertex = 1;
  tagger_info.shw_sp_angle_beam = 0;
  tagger_info.shw_sp_flag_all_above = false;
  tagger_info.shw_sp_length_main = 1;
  tagger_info.shw_sp_length_total = 1;
  tagger_info.shw_sp_min_dQ_dx_5 = 1;
  tagger_info.shw_sp_lowest_dQ_dx = 1;
  tagger_info.shw_sp_iso_angle = 0;
  tagger_info.shw_sp_n_below_zero = 0;
  tagger_info.shw_sp_highest_dQ_dx = 1;
  tagger_info.shw_sp_n_lowest = 1;
  tagger_info.shw_sp_n_highest = 1;
  tagger_info.shw_sp_stem_length = 1;
  tagger_info.shw_sp_E_indirect_max_energy = 0;
  tagger_info.shw_sp_flag_stem_trajectory = 0;
  tagger_info.shw_sp_min_dis = 0;
  tagger_info.shw_sp_n_other_vertex = 2;
  tagger_info.shw_sp_n_stem_size = 20;
  tagger_info.shw_sp_medium_dQ_dx = 1;
  tagger_info.shw_sp_filled = 0;

  tagger_info.shw_sp_vec_dQ_dx_2 = 0;
  tagger_info.shw_sp_vec_dQ_dx_3 = 0;
  tagger_info.shw_sp_vec_dQ_dx_4 = 0;
  tagger_info.shw_sp_vec_dQ_dx_5 = 0;
  tagger_info.shw_sp_vec_dQ_dx_6 = 0;
  tagger_info.shw_sp_vec_dQ_dx_7 = 0;
  tagger_info.shw_sp_vec_dQ_dx_8 = 0;
  tagger_info.shw_sp_vec_dQ_dx_9 = 0;
  tagger_info.shw_sp_vec_dQ_dx_10 = 0;
  tagger_info.shw_sp_vec_dQ_dx_11 = 0;
  tagger_info.shw_sp_vec_dQ_dx_12 = 0;
  tagger_info.shw_sp_vec_dQ_dx_13 = 0;
  tagger_info.shw_sp_vec_dQ_dx_14 = 0;
  tagger_info.shw_sp_vec_dQ_dx_15 = 0;
  tagger_info.shw_sp_vec_dQ_dx_16 = 0;
  tagger_info.shw_sp_vec_dQ_dx_17 = 0;
  tagger_info.shw_sp_vec_dQ_dx_18 = 0;
  tagger_info.shw_sp_vec_dQ_dx_19 = 0;
  tagger_info.shw_sp_vec_median_dedx = 0;
  tagger_info.shw_sp_vec_mean_dedx = 0;

  // shower pi0 identification
  tagger_info.shw_sp_pio_flag = true;
  tagger_info.shw_sp_pio_mip_id = 0;
  tagger_info.shw_sp_pio_filled = 0;
  tagger_info.shw_sp_pio_flag_pio = 0;

  tagger_info.shw_sp_pio_1_flag = true;
  tagger_info.shw_sp_pio_1_mass = 0;
  tagger_info.shw_sp_pio_1_pio_type = 0;
  tagger_info.shw_sp_pio_1_energy_1 = 0;
  tagger_info.shw_sp_pio_1_energy_2 = 0;
  tagger_info.shw_sp_pio_1_dis_1 = 0;
  tagger_info.shw_sp_pio_1_dis_2 = 0;

  // bad reconstruction
  tagger_info.shw_sp_br_filled = 0;

  tagger_info.shw_sp_br1_flag = true;
  // br1_1
  tagger_info.shw_sp_br1_1_flag = true;
  tagger_info.shw_sp_br1_1_shower_type = 0;
  tagger_info.shw_sp_br1_1_vtx_n_segs = 0;
  tagger_info.shw_sp_br1_1_energy = 0;
  tagger_info.shw_sp_br1_1_n_segs = 0;
  tagger_info.shw_sp_br1_1_flag_sg_topology = 0;
  tagger_info.shw_sp_br1_1_flag_sg_trajectory = 0;
  tagger_info.shw_sp_br1_1_sg_length = 0;

  //br1_2
  tagger_info.shw_sp_br1_2_flag = true;
  tagger_info.shw_sp_br1_2_energy = 0;
  tagger_info.shw_sp_br1_2_n_connected = 0;
  tagger_info.shw_sp_br1_2_max_length = 0;
  tagger_info.shw_sp_br1_2_n_connected_1 = 0;
  tagger_info.shw_sp_br1_2_vtx_n_segs = 0;
  tagger_info.shw_sp_br1_2_n_shower_segs = 0;
  tagger_info.shw_sp_br1_2_max_length_ratio = 1;
  tagger_info.shw_sp_br1_2_shower_length = 0;

  //br1_3
  tagger_info.shw_sp_br1_3_flag = true;
  tagger_info.shw_sp_br1_3_energy = 0;
  tagger_info.shw_sp_br1_3_n_connected_p = 0;
  tagger_info.shw_sp_br1_3_max_length_p = 0;
  tagger_info.shw_sp_br1_3_n_shower_segs = 0;
  tagger_info.shw_sp_br1_3_flag_sg_topology = 0;
  tagger_info.shw_sp_br1_3_flag_sg_trajectory = 0;
  tagger_info.shw_sp_br1_3_n_shower_main_segs = 0;
  tagger_info.shw_sp_br1_3_sg_length = 0;

  // br2
  tagger_info.shw_sp_br2_flag = true;
  tagger_info.shw_sp_br2_flag_single_shower = 0;
  tagger_info.shw_sp_br2_num_valid_tracks = 0;
  tagger_info.shw_sp_br2_energy = 0;
  tagger_info.shw_sp_br2_angle1 = 0;
  tagger_info.shw_sp_br2_angle2 = 0;
  tagger_info.shw_sp_br2_angle = 0;
  tagger_info.shw_sp_br2_angle3 = 0;
  tagger_info.shw_sp_br2_n_shower_main_segs = 0;
  tagger_info.shw_sp_br2_max_angle = 0;
  tagger_info.shw_sp_br2_sg_length = 0;
  tagger_info.shw_sp_br2_flag_sg_trajectory = 0;

  // low-energy overlap
  tagger_info.shw_sp_lol_flag = true;
  tagger_info.shw_sp_lol_3_flag = true;
  tagger_info.shw_sp_lol_3_angle_beam = 0;
  tagger_info.shw_sp_lol_3_min_angle = 0;
  tagger_info.shw_sp_lol_3_n_valid_tracks = 0;
  tagger_info.shw_sp_lol_3_vtx_n_segs = 0;
  tagger_info.shw_sp_lol_3_energy = 0;
  tagger_info.shw_sp_lol_3_shower_main_length = 0;
  tagger_info.shw_sp_lol_3_n_sum = 0;
  tagger_info.shw_sp_lol_3_n_out = 0;


  // br3
  tagger_info.shw_sp_br3_1_energy = 0;
  tagger_info.shw_sp_br3_1_n_shower_segments = 0;
  tagger_info.shw_sp_br3_1_sg_flag_trajectory = 0;
  tagger_info.shw_sp_br3_1_sg_direct_length = 0;
  tagger_info.shw_sp_br3_1_sg_length = 0;
  tagger_info.shw_sp_br3_1_total_main_length = 0;
  tagger_info.shw_sp_br3_1_total_length = 0;
  tagger_info.shw_sp_br3_1_iso_angle = 0;
  tagger_info.shw_sp_br3_1_sg_flag_topology = 0;
  tagger_info.shw_sp_br3_1_flag = true;

  tagger_info.shw_sp_br3_2_n_ele = 0;
  tagger_info.shw_sp_br3_2_n_other = 0;
  tagger_info.shw_sp_br3_2_energy = 0;
  tagger_info.shw_sp_br3_2_total_main_length = 0;
  tagger_info.shw_sp_br3_2_total_length = 0;
  tagger_info.shw_sp_br3_2_other_fid = 0;
  tagger_info.shw_sp_br3_2_flag = true;

  tagger_info.shw_sp_br3_4_acc_length = 0;
  tagger_info.shw_sp_br3_4_total_length = 0;
  tagger_info.shw_sp_br3_4_energy = 0;
  tagger_info.shw_sp_br3_4_flag = true;

  tagger_info.shw_sp_br3_7_energy = 0;
  tagger_info.shw_sp_br3_7_min_angle = 0;
  tagger_info.shw_sp_br3_7_sg_length = 0;
  tagger_info.shw_sp_br3_7_shower_main_length = 0;
  tagger_info.shw_sp_br3_7_flag = true;

  tagger_info.shw_sp_br3_8_max_dQ_dx = 0;
  tagger_info.shw_sp_br3_8_energy = 0;
  tagger_info.shw_sp_br3_8_n_main_segs = 0;
  tagger_info.shw_sp_br3_8_shower_main_length = 0;
  tagger_info.shw_sp_br3_8_shower_length = 0;
  tagger_info.shw_sp_br3_8_flag = true;

  tagger_info.shw_sp_br3_flag = true;


  tagger_info.shw_sp_br4_1_shower_main_length = 0;
  tagger_info.shw_sp_br4_1_shower_total_length = 0;
  tagger_info.shw_sp_br4_1_min_dis = 0;
  tagger_info.shw_sp_br4_1_energy = 0;
  tagger_info.shw_sp_br4_1_flag_avoid_muon_check = 0;
  tagger_info.shw_sp_br4_1_n_vtx_segs = 0;
  tagger_info.shw_sp_br4_1_n_main_segs = 0;
  tagger_info.shw_sp_br4_1_flag = true;

  tagger_info.shw_sp_br4_2_ratio_45 = 1;
  tagger_info.shw_sp_br4_2_ratio_35 = 1;
  tagger_info.shw_sp_br4_2_ratio_25 = 1;
  tagger_info.shw_sp_br4_2_ratio_15 = 1;
  tagger_info.shw_sp_br4_2_energy = 0;
  tagger_info.shw_sp_br4_2_ratio1_45 = 1;
  tagger_info.shw_sp_br4_2_ratio1_35 = 1;
  tagger_info.shw_sp_br4_2_ratio1_25 = 1;
  tagger_info.shw_sp_br4_2_ratio1_15 = 1;
  tagger_info.shw_sp_br4_2_iso_angle = 0;
  tagger_info.shw_sp_br4_2_iso_angle1 = 0;
  tagger_info.shw_sp_br4_2_angle = 0;
  tagger_info.shw_sp_br4_2_flag = true;

  tagger_info.shw_sp_br4_flag = true;


  tagger_info.shw_sp_hol_1_n_valid_tracks = 0;
  tagger_info.shw_sp_hol_1_min_angle = 0;
  tagger_info.shw_sp_hol_1_energy = 0;
  tagger_info.shw_sp_hol_1_flag_all_shower = 0;
  tagger_info.shw_sp_hol_1_min_length = 0;
  tagger_info.shw_sp_hol_1_flag = true;

  tagger_info.shw_sp_hol_2_min_angle = 0;
  tagger_info.shw_sp_hol_2_medium_dQ_dx = 1;
  tagger_info.shw_sp_hol_2_ncount = 0;
  tagger_info.shw_sp_hol_2_energy = 0;
  tagger_info.shw_sp_hol_2_flag = true;

  tagger_info.shw_sp_hol_flag = true;

  tagger_info.shw_sp_lem_shower_total_length = 0;
  tagger_info.shw_sp_lem_shower_main_length = 0;
  tagger_info.shw_sp_lem_n_3seg = 0;
  tagger_info.shw_sp_lem_e_charge = 0;
  tagger_info.shw_sp_lem_e_dQdx = 0;
  tagger_info.shw_sp_lem_shower_num_segs = 0;
  tagger_info.shw_sp_lem_shower_num_main_segs = 0;
  tagger_info.shw_sp_lem_flag = true;


  
  // shower pi0 identification
  tagger_info.pio_flag = true;
  tagger_info.pio_mip_id = 0;
  tagger_info.pio_filled = 0;
  tagger_info.pio_flag_pio = 0;

  tagger_info.pio_1_flag = true;
  tagger_info.pio_1_mass = 0;
  tagger_info.pio_1_pio_type = 0;
  tagger_info.pio_1_energy_1 = 0;
  tagger_info.pio_1_energy_2 = 0;
  tagger_info.pio_1_dis_1 = 0;
  tagger_info.pio_1_dis_2 = 0;

  // stem direction
  tagger_info.stem_dir_flag = true;
  tagger_info.stem_dir_flag_single_shower = 0;
  tagger_info.stem_dir_filled = 0;
  tagger_info.stem_dir_angle = 0;
  tagger_info.stem_dir_energy = 0;
  tagger_info.stem_dir_angle1 = 0;
  tagger_info.stem_dir_angle2 = 0;
  tagger_info.stem_dir_angle3 = 0;
  tagger_info.stem_dir_ratio = 1;

  // bad reconstruction
  tagger_info.br_filled = 0;

  tagger_info.br1_flag = true;
  // br1_1
  tagger_info.br1_1_flag = true;
  tagger_info.br1_1_shower_type = 0;
  tagger_info.br1_1_vtx_n_segs = 0;
  tagger_info.br1_1_energy = 0;
  tagger_info.br1_1_n_segs = 0;
  tagger_info.br1_1_flag_sg_topology = 0;
  tagger_info.br1_1_flag_sg_trajectory = 0;
  tagger_info.br1_1_sg_length = 0;

  //br1_2
  tagger_info.br1_2_flag = true;
  tagger_info.br1_2_energy = 0;
  tagger_info.br1_2_n_connected = 0;
  tagger_info.br1_2_max_length = 0;
  tagger_info.br1_2_n_connected_1 = 0;
  tagger_info.br1_2_vtx_n_segs = 0;
  tagger_info.br1_2_n_shower_segs = 0;
  tagger_info.br1_2_max_length_ratio = 1;
  tagger_info.br1_2_shower_length = 0;

  //br1_3
  tagger_info.br1_3_flag = true;
  tagger_info.br1_3_energy = 0;
  tagger_info.br1_3_n_connected_p = 0;
  tagger_info.br1_3_max_length_p = 0;
  tagger_info.br1_3_n_shower_segs = 0;
  tagger_info.br1_3_flag_sg_topology = 0;
  tagger_info.br1_3_flag_sg_trajectory = 0;
  tagger_info.br1_3_n_shower_main_segs = 0;
  tagger_info.br1_3_sg_length = 0;

  // br2
  tagger_info.br2_flag = true;
  tagger_info.br2_flag_single_shower = 0;
  tagger_info.br2_num_valid_tracks = 0;
  tagger_info.br2_energy = 0;
  tagger_info.br2_angle1 = 0;
  tagger_info.br2_angle2 = 0;
  tagger_info.br2_angle = 0;
  tagger_info.br2_angle3 = 0;
  tagger_info.br2_n_shower_main_segs = 0;
  tagger_info.br2_max_angle = 0;
  tagger_info.br2_sg_length = 0;
  tagger_info.br2_flag_sg_trajectory = 0;
  
  // low-energy overlap
  tagger_info.lol_flag = true;
  tagger_info.lol_3_flag = true;
  tagger_info.lol_3_angle_beam = 0;
  tagger_info.lol_3_min_angle = 0;
  tagger_info.lol_3_n_valid_tracks = 0;
  tagger_info.lol_3_vtx_n_segs = 0;
  tagger_info.lol_3_energy = 0;
  tagger_info.lol_3_shower_main_length = 0;
  tagger_info.lol_3_n_sum = 0;
  tagger_info.lol_3_n_out = 0;


  // br3
  tagger_info.br3_1_energy = 0;
  tagger_info.br3_1_n_shower_segments = 0;
  tagger_info.br3_1_sg_flag_trajectory = 0;
  tagger_info.br3_1_sg_direct_length = 0;
  tagger_info.br3_1_sg_length = 0;
  tagger_info.br3_1_total_main_length = 0;
  tagger_info.br3_1_total_length = 0;
  tagger_info.br3_1_iso_angle = 0;
  tagger_info.br3_1_sg_flag_topology = 0;
  tagger_info.br3_1_flag = true;
  
  tagger_info.br3_2_n_ele = 0;
  tagger_info.br3_2_n_other = 0;
  tagger_info.br3_2_energy = 0;
  tagger_info.br3_2_total_main_length = 0;
  tagger_info.br3_2_total_length = 0;
  tagger_info.br3_2_other_fid = 0;
  tagger_info.br3_2_flag = true;
  
  tagger_info.br3_4_acc_length = 0;
  tagger_info.br3_4_total_length = 0;
  tagger_info.br3_4_energy = 0;
  tagger_info.br3_4_flag = true;
  
  tagger_info.br3_7_energy = 0;
  tagger_info.br3_7_min_angle = 0;
  tagger_info.br3_7_sg_length = 0;
  tagger_info.br3_7_shower_main_length = 0;
  tagger_info.br3_7_flag = true;
  
  tagger_info.br3_8_max_dQ_dx = 0;
  tagger_info.br3_8_energy = 0;
  tagger_info.br3_8_n_main_segs = 0;
  tagger_info.br3_8_shower_main_length = 0;
  tagger_info.br3_8_shower_length = 0;
  tagger_info.br3_8_flag = true;
  
  tagger_info.br3_flag = true;


  tagger_info.br4_1_shower_main_length = 0;
  tagger_info.br4_1_shower_total_length = 0;
  tagger_info.br4_1_min_dis = 0;
  tagger_info.br4_1_energy = 0;
  tagger_info.br4_1_flag_avoid_muon_check = 0;
  tagger_info.br4_1_n_vtx_segs = 0;
  tagger_info.br4_1_n_main_segs = 0;
  tagger_info.br4_1_flag = true;
  
  tagger_info.br4_2_ratio_45 = 1;
  tagger_info.br4_2_ratio_35 = 1;
  tagger_info.br4_2_ratio_25 = 1;
  tagger_info.br4_2_ratio_15 = 1;
  tagger_info.br4_2_energy = 0;
  tagger_info.br4_2_ratio1_45 = 1;
  tagger_info.br4_2_ratio1_35 = 1;
  tagger_info.br4_2_ratio1_25 = 1;
  tagger_info.br4_2_ratio1_15 = 1;
  tagger_info.br4_2_iso_angle = 0;
  tagger_info.br4_2_iso_angle1 = 0;
  tagger_info.br4_2_angle = 0;
  tagger_info.br4_2_flag = true;
  
  tagger_info.br4_flag = true;


  tagger_info.hol_1_n_valid_tracks = 0;
  tagger_info.hol_1_min_angle = 0;
  tagger_info.hol_1_energy = 0;
  tagger_info.hol_1_flag_all_shower = 0;
  tagger_info.hol_1_min_length = 0;
  tagger_info.hol_1_flag = true;
  
  tagger_info.hol_2_min_angle = 0;
  tagger_info.hol_2_medium_dQ_dx = 1;
  tagger_info.hol_2_ncount = 0;
  tagger_info.hol_2_energy = 0;
  tagger_info.hol_2_flag = true;
  
  tagger_info.hol_flag = true;


  // vertex inside shower
  tagger_info.vis_1_filled = false;
  tagger_info.vis_1_n_vtx_segs = 0;
  tagger_info.vis_1_energy = 0;
  tagger_info.vis_1_num_good_tracks = 0;
  tagger_info.vis_1_max_angle = 0;
  tagger_info.vis_1_max_shower_angle = 0;
  tagger_info.vis_1_tmp_length1 = 0;
  tagger_info.vis_1_tmp_length2 = 0;
  tagger_info.vis_1_particle_type = 0;
  tagger_info.vis_1_flag = true;
  
  tagger_info.vis_2_filled = false;
  tagger_info.vis_2_n_vtx_segs = 0;
  tagger_info.vis_2_min_angle = 0;
  tagger_info.vis_2_min_weak_track = 0;
  tagger_info.vis_2_angle_beam = 0;
  tagger_info.vis_2_min_angle1 = 0;
  tagger_info.vis_2_iso_angle1 = 0;
  tagger_info.vis_2_min_medium_dQ_dx= 0;
  tagger_info.vis_2_min_length = 0;
  tagger_info.vis_2_sg_length = 0;
  tagger_info.vis_2_max_angle = 0;
  tagger_info.vis_2_max_weak_track = 0;
  tagger_info.vis_2_flag = true;
  
  tagger_info.vis_flag = true;

  
  tagger_info.stem_len_energy = 0;
  tagger_info.stem_len_length = 0;
  tagger_info.stem_len_flag_avoid_muon_check = 0;
  tagger_info.stem_len_num_daughters = 0;
  tagger_info.stem_len_daughter_length = 0;
  tagger_info.stem_len_flag = true;

  tagger_info.brm_n_mu_segs = 0;
  tagger_info.brm_Ep = 0;
  tagger_info.brm_energy = 0;
  tagger_info.brm_acc_length = 0;
  tagger_info.brm_shower_total_length = 0;
  tagger_info.brm_connected_length = 0;
  tagger_info.brm_n_size = 0;
  tagger_info.brm_acc_direct_length = 0;
  tagger_info.brm_n_shower_main_segs = 0;
  tagger_info.brm_n_mu_main = 0;
  tagger_info.brm_flag = true;

  tagger_info.cme_mu_energy = 0;
  tagger_info.cme_energy = 0;
  tagger_info.cme_mu_length = 0;
  tagger_info.cme_length = 0;
  tagger_info.cme_angle_beam = 0;
  tagger_info.cme_flag = true;

  tagger_info.anc_energy = 0;
  tagger_info.anc_angle = 0;
  tagger_info.anc_max_angle = 0;
  tagger_info.anc_max_length = 0;
  tagger_info.anc_acc_forward_length = 0;
  tagger_info.anc_acc_backward_length = 0;
  tagger_info.anc_acc_forward_length1 = 0;
  tagger_info.anc_shower_main_length = 0;
  tagger_info.anc_shower_total_length = 0;
  tagger_info.anc_flag_main_outside = 0;
  tagger_info.anc_flag = true;

  tagger_info.lem_shower_total_length = 0;
  tagger_info.lem_shower_main_length = 0;
  tagger_info.lem_n_3seg = 0;
  tagger_info.lem_e_charge = 0;
  tagger_info.lem_e_dQdx = 0;
  tagger_info.lem_shower_num_segs = 0;
  tagger_info.lem_shower_num_main_segs = 0;
  tagger_info.lem_flag = true;

  tagger_info.stw_1_energy = 0;
  tagger_info.stw_1_dis = 0;
  tagger_info.stw_1_dQ_dx = 1;
  tagger_info.stw_1_flag_single_shower = 0;
  tagger_info.stw_1_n_pi0 = 0;
  tagger_info.stw_1_num_valid_tracks = 0;
  tagger_info.stw_1_flag = true;
  tagger_info.stw_flag = true;

  tagger_info.spt_flag_single_shower = 0;
  tagger_info.spt_energy = 0;
  tagger_info.spt_shower_main_length = 0;
  tagger_info.spt_shower_total_length = 0;
  tagger_info.spt_angle_beam = 0;
  tagger_info.spt_angle_vertical = 0;
  tagger_info.spt_max_dQ_dx = 1;
  tagger_info.spt_angle_beam_1 = 0;
  tagger_info.spt_angle_drift = 0;
  tagger_info.spt_angle_drift_1 = 0;
  tagger_info.spt_num_valid_tracks = 0;
  tagger_info.spt_n_vtx_segs = 0;
  tagger_info.spt_max_length = 0;
  tagger_info.spt_flag = true;

  tagger_info.mgo_energy = 0;
  tagger_info.mgo_max_energy = 0;
  tagger_info.mgo_total_energy = 0;
  tagger_info.mgo_n_showers = 0;
  tagger_info.mgo_max_energy_1 = 0;
  tagger_info.mgo_max_energy_2 = 0;
  tagger_info.mgo_total_other_energy = 0;
  tagger_info.mgo_n_total_showers = 0;
  tagger_info.mgo_total_other_energy_1 = 0;
  tagger_info.mgo_flag = true;

  tagger_info.mgt_flag_single_shower = 0;
  tagger_info.mgt_max_energy = 0;
  tagger_info.mgt_energy = 0;
  tagger_info.mgt_total_other_energy = 0;
  tagger_info.mgt_max_energy_1 = 0;
  tagger_info.mgt_e_indirect_max_energy = 0;
  tagger_info.mgt_e_direct_max_energy = 0;
  tagger_info.mgt_n_direct_showers = 0;
  tagger_info.mgt_e_direct_total_energy = 0;
  tagger_info.mgt_e_indirect_total_energy = 0;
  tagger_info.mgt_flag_indirect_max_pio = 0;
  tagger_info.mgt_flag = true;

  tagger_info.sig_flag = true;


  tagger_info.tro_3_stem_length = 0;
  tagger_info.tro_3_n_muon_segs = 0;
  tagger_info.tro_3_energy = 0;
  tagger_info.tro_3_flag = true;
  tagger_info.tro_flag = true;


  // cosmic tagger
  tagger_info.cosmict_flag_1 = false;
  tagger_info.cosmict_flag_2 = false;
  tagger_info.cosmict_flag_3 = false; 
  tagger_info.cosmict_flag_4 = false;
  tagger_info.cosmict_flag_5 = false;
  tagger_info.cosmict_flag_6 = false;
  tagger_info.cosmict_flag_7 = false;
  tagger_info.cosmict_flag_8 = false;
  tagger_info.cosmict_flag_9 = false;
  tagger_info.cosmict_flag = false;

  // single muon
  tagger_info.cosmict_2_filled = 0;
  tagger_info.cosmict_2_particle_type = 0;
  tagger_info.cosmict_2_n_muon_tracks = 0;
  tagger_info.cosmict_2_total_shower_length = 0;
  tagger_info.cosmict_2_flag_inside = 1;
  tagger_info.cosmict_2_angle_beam = 0;
  tagger_info.cosmict_2_flag_dir_weak = 0;
  tagger_info.cosmict_2_dQ_dx_end = 0;
  tagger_info.cosmict_2_dQ_dx_front = 0;
  tagger_info.cosmict_2_theta = 0;
  tagger_info.cosmict_2_phi = 0;
  tagger_info.cosmict_2_valid_tracks = 0;
  
  // single muon (long)
  tagger_info.cosmict_3_filled = 0;
  tagger_info.cosmict_3_flag_inside = 0;
  tagger_info.cosmict_3_angle_beam = 0;
  tagger_info.cosmict_3_flag_dir_weak = 0;
  tagger_info.cosmict_3_dQ_dx_front = 0;
  tagger_info.cosmict_3_dQ_dx_end = 0;
  tagger_info.cosmict_3_theta = 0;
  tagger_info.cosmict_3_phi = 0;
  tagger_info.cosmict_3_valid_tracks = 0;

  // kinematics muon
  tagger_info.cosmict_4_filled = 0;
  tagger_info.cosmict_4_flag_inside = 0;
  tagger_info.cosmict_4_angle_beam = 0;
  tagger_info.cosmict_4_connected_showers = 0;
  
  // kinematics muon (long)
  tagger_info.cosmict_5_filled = 0;
  tagger_info.cosmict_5_flag_inside = 0;
  tagger_info.cosmict_5_angle_beam = 0;
  tagger_info.cosmict_5_connected_showers = 0;

  // special
  tagger_info.cosmict_6_filled = 0;
  tagger_info.cosmict_6_flag_dir_weak = 0;
  tagger_info.cosmict_6_flag_inside = 1; 
  tagger_info.cosmict_6_angle = 0;

  //muon + michel
  tagger_info.cosmict_7_filled = 0;
  tagger_info.cosmict_7_flag_sec = 0;
  tagger_info.cosmict_7_n_muon_tracks = 0;
  tagger_info.cosmict_7_total_shower_length = 0;
  tagger_info.cosmict_7_flag_inside = 1;
  tagger_info.cosmict_7_angle_beam =  0;
  tagger_info.cosmict_7_flag_dir_weak = 0;
  tagger_info.cosmict_7_dQ_dx_end = 0;
  tagger_info.cosmict_7_dQ_dx_front = 0;
  tagger_info.cosmict_7_theta = 0;
  tagger_info.cosmict_7_phi = 0;

  // muon + michel + special 
  tagger_info.cosmict_8_filled = 0;
  tagger_info.cosmict_8_flag_out = 0;
  tagger_info.cosmict_8_muon_length = 0;
  tagger_info.cosmict_8_acc_length = 0;

  // numu tagger
  tagger_info.numu_cc_flag_3 = 0;
  tagger_info.numu_cc_3_particle_type = 0;
  tagger_info.numu_cc_3_max_length = 0;
  tagger_info.numu_cc_3_acc_track_length = 0;
  tagger_info.numu_cc_3_max_length_all = 0;
  tagger_info.numu_cc_3_max_muon_length = 0;
  tagger_info.numu_cc_3_n_daughter_tracks = 0;
  tagger_info.numu_cc_3_n_daughter_all = 0;

  tagger_info.numu_cc_flag = 0;


  // numu BDTs
  tagger_info.cosmict_2_4_score = 0;
  tagger_info.cosmict_3_5_score = 0;
  tagger_info.cosmict_6_score = 0;
  tagger_info.cosmict_7_score = 0;
  tagger_info.cosmict_8_score = 0;
  tagger_info.cosmict_10_score = 0;

  tagger_info.numu_1_score = 0;
  tagger_info.numu_2_score = 0;
  tagger_info.numu_3_score = 0;

  // total score, place holder for cosmict ...
  tagger_info.cosmict_score = 0;
  tagger_info.numu_score = 0;

  // nue BDTs
  tagger_info.mipid_score=0;
  tagger_info.gap_score=0;
  tagger_info.hol_lol_score=0;
  tagger_info.cme_anc_score=0;
  tagger_info.mgo_mgt_score=0;
  tagger_info.br1_score=0;
  tagger_info.br3_score=0;
  tagger_info.br3_3_score=0;
  tagger_info.br3_5_score=0;
  tagger_info.br3_6_score=0;
  tagger_info.stemdir_br2_score=0;
  tagger_info.trimuon_score=0;
  tagger_info.br4_tro_score=0;
  tagger_info.mipquality_score=0;
  tagger_info.pio_1_score=0;
  tagger_info.pio_2_score=0;
  tagger_info.stw_spt_score=0;
  tagger_info.vis_1_score=0;
  tagger_info.vis_2_score=0;
  tagger_info.stw_2_score=0;
  tagger_info.stw_3_score=0;
  tagger_info.stw_4_score=0;
  tagger_info.sig_1_score=0;
  tagger_info.sig_2_score=0;
  tagger_info.lol_1_score=0;
  tagger_info.lol_2_score=0;
  tagger_info.tro_1_score=0;
  tagger_info.tro_2_score=0;
  tagger_info.tro_4_score=0;
  tagger_info.tro_5_score=0;
  tagger_info.nue_score = 0;
  tagger_info.photon_flag=false;
  
  kine_pio_flag = 0;
  kine_pio_mass = 0;
  kine_pio_vtx_dis = 1000*units::cm; // 10 m
  
  kine_pio_energy_1 = 0;
  kine_pio_theta_1 = 0;
  kine_pio_phi_1 = 0;
  kine_pio_dis_1 = 0;
  
  kine_pio_energy_2 = 0;
  kine_pio_theta_2 = 0;
  kine_pio_phi_2 = 0;
  kine_pio_dis_2 = 0;

  kine_pio_angle = 0;
}
