#include "WCPPID/ImprovePR3DCluster.h"
#include "WCP2dToy/LowmemTiling.h"
#include "WCPPID/CalcPoints.h"

#include <boost/graph/connected_components.hpp>

using namespace WCP;

WCPPID::PR3DCluster* WCPPID::Improve_PR3DCluster_2(WCPPID::PR3DCluster* cluster, ToyCTPointCloud& ct_point_cloud,WCPSst::GeomDataSource& gds, WCP2dToy::WCPHolder *holder, int nrebin, int frame_length, double unit_dis){
  cluster->Create_graph(ct_point_cloud);

  // {
  //   MCUGraph *graph = cluster->get_graph();
  //   std::cout << num_vertices(*graph) << " " << num_edges(*graph) << std::endl;
  // }

  std::cout << "ImproveCluster_2 " << " Orig Graph: " << num_vertices(*cluster->get_graph()) << " " << num_edges(*cluster->get_graph()) << std::endl;

  cluster->establish_same_mcell_steiner_edges(gds);
  
  // {
  //   MCUGraph *graph = cluster->get_graph();
  //   std::cout << num_vertices(*graph) << " " << num_edges(*graph) << std::endl;
  // }
  
  std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> wcps1 = cluster->get_two_boundary_wcps();
  cluster->dijkstra_shortest_paths(wcps1.first);
  cluster->cal_shortest_path(wcps1.second);

  std::cout << "ImproveCluster_2 " << " Origi Shortest path indices: " << cluster->get_path_wcps().size() << " ; Graphvertices: " << num_vertices(*cluster->get_graph()) << " edges: " << num_edges(*cluster->get_graph()) << std::endl;
  // {
  //   std::cout << cluster->get_path_wcps().size() << std::endl;
  // }
  
  cluster->remove_same_mcell_steiner_edges();
  std::cout << "ImproveCluster_2 " << " Orig Graph: " << num_vertices(*cluster->get_graph()) << " " << num_edges(*cluster->get_graph()) << std::endl;

  cluster->Del_graph();


  // {
  //   PointVector tmp_pts;
  //   for (auto it = cluster->get_path_wcps().begin(); it!= cluster->get_path_wcps().end(); it++){
  //     Point tmp_p((*it).x, (*it).y, (*it).z);
  //     tmp_pts.push_back(tmp_p);
  //   }
  //   double tmp_length = 0;
  //   for (size_t i=0;i+1<tmp_pts.size();i++){
  //     tmp_length += sqrt(pow(tmp_pts.at(i+1).x - tmp_pts.at(i).x,2)+pow(tmp_pts.at(i+1).y - tmp_pts.at(i).y,2)+pow(tmp_pts.at(i+1).z - tmp_pts.at(i).z,2));
  //   }
    
  //   std::cout << wcps1.first.x << " " << wcps1.first.y << " " << wcps1.first.z << "; " <<  wcps1.second.x << " " << wcps1.second.y << " " << wcps1.second.z << "; " << fabs(wcps1.first.x - wcps1.second.x)/(2.22*units::mm) + fabs(wcps1.first.index_u - wcps1.second.index_u)  + fabs(wcps1.first.index_v - wcps1.second.index_v) + fabs(wcps1.first.index_w - wcps1.second.index_w) << " " << cluster->get_path_wcps().size() << " " << tmp_length/units::cm << std::endl;
  // }
  
  
  // include dead channels ...
  WCP2dToy::WCPHolder *temp_holder1 = new WCP2dToy::WCPHolder();
  WCPPID::PR3DCluster *temp_cluster = WCPPID::Improve_PR3DCluster_1(cluster,ct_point_cloud, gds,temp_holder1);
  WCPPID::calc_sampling_points(gds,temp_cluster,nrebin, frame_length, unit_dis,false);

  
  ToyPointCloud* ref_point_cloud = cluster->get_point_cloud();
  temp_cluster->Create_point_cloud();
  temp_cluster->Create_graph(ct_point_cloud, ref_point_cloud);

  std::cout << "ImproveCluster_1: " << temp_cluster->get_point_cloud()->get_num_points() << " points created " << temp_cluster->get_mcells().size() << std::endl;

  std::cout << "ImproveCluster_2: " << " Temp Graph: " << num_vertices(*temp_cluster->get_graph()) << " " << num_edges(*temp_cluster->get_graph()) << std::endl;

  // {
  //   MCUGraph *graph = temp_cluster->get_graph();
  //   std::cout << num_vertices(*graph) << " " << num_edges(*graph) << std::endl;
  // }
  
  // WCPPID::PR3DCluster* temp_cluster = cluster;
  // temp_cluster->Create_graph(ct_point_cloud);
  
  //std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> wcps = temp_cluster->get_highest_lowest_wcps();
  std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> wcps = temp_cluster->get_two_boundary_wcps();

  
  temp_cluster->establish_same_mcell_steiner_edges(gds,false);
  temp_cluster->dijkstra_shortest_paths(wcps.first);
  temp_cluster->cal_shortest_path(wcps.second);

  std::cout << "ImproveCluster_2: " << " Temp Shortest path indices: " << temp_cluster->get_path_wcps().size() << " ; Graph vertices: " << num_vertices(*temp_cluster->get_graph()) << " edges: " << num_edges(*temp_cluster->get_graph()) << std::endl;


  temp_cluster->remove_same_mcell_steiner_edges();

  std::cout << "ImproveCluster_2: " << " Temp Graph: " << num_vertices(*temp_cluster->get_graph()) << " " << num_edges(*temp_cluster->get_graph()) << std::endl;

  temp_cluster->Del_graph();
  temp_cluster->Del_point_cloud();


  //WCPPID::PR3DCluster *new_cluster = Improve_PR3DCluster(temp_cluster, ct_point_cloud, gds, holder);

  // {
  //   PointVector tmp_pts;
  //   for (auto it = temp_cluster->get_path_wcps().begin(); it!= temp_cluster->get_path_wcps().end(); it++){
  //     Point tmp_p((*it).x, (*it).y, (*it).z);
  //     tmp_pts.push_back(tmp_p);
  //     //std::cout << "{" << tmp_p.x << ", " << tmp_p.y << ", " << tmp_p.z << ", " << (*it).index << "}, " << std::endl;
  //   }
  //   double tmp_length = 0;
  //   for (size_t i=0;i+1<tmp_pts.size();i++){
  //     tmp_length += sqrt(pow(tmp_pts.at(i+1).x - tmp_pts.at(i).x,2)+pow(tmp_pts.at(i+1).y - tmp_pts.at(i).y,2)+pow(tmp_pts.at(i+1).z - tmp_pts.at(i).z,2));
  //   }
  
  //   std::cout << wcps.first.x << " " << wcps.first.y << " " << wcps.first.z << " " << wcps.first.index << "; " <<  wcps.second.x << " " << wcps.second.y << " " << wcps.second.z << " " << wcps.second.index << "; "  << temp_cluster->get_path_wcps().size() << " " << tmp_length/units::cm << std::endl;
  // }

   


      
   // include original inefficient channels ... 
   WCPPID::PR3DCluster *new_cluster = Improve_PR3DCluster(cluster, temp_cluster, ct_point_cloud, gds, holder);

   delete temp_cluster;
   delete temp_holder1;
     
   return new_cluster;
   //return temp_cluster;
}

WCPPID::PR3DCluster* WCPPID::Improve_PR3DCluster_1(WCPPID::PR3DCluster* cluster, ToyCTPointCloud& ct_point_cloud,WCPSst::GeomDataSource& gds, WCP2dToy::WCPHolder *holder){

  std::map<int,std::set<int>> u_time_chs; // time chs
  std::map<int,std::set<int>> v_time_chs; // time chs
  std::map<int,std::set<int>> w_time_chs; // time chs
  std::map<std::pair<int,int>,double> time_ch_charge_map;
  std::map<std::pair<int,int>,double> time_ch_charge_err_map;

  int min_time = 1e9;
  int max_time = -1e9;
  int min_uch = 1e9;
  int max_uch = -1e9;
  int min_vch = 1e9;
  int max_vch = -1e9;
  int min_wch = 1e9;
  int max_wch = -1e9;
  
  
  // fill all map according to existing mcells
  SMGCSelection& old_mcells = cluster->get_mcells();
  for (auto it=old_mcells.begin(); it!=old_mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    GeomWireSelection& uwires = mcell->get_uwires();
    GeomWireSelection& vwires = mcell->get_vwires();
    GeomWireSelection& wwires = mcell->get_wwires();
    int time_slice = mcell->GetTimeSlice();

    if (time_slice < min_time) min_time = time_slice;
    if (time_slice > max_time) max_time = time_slice;
    
    if (u_time_chs.find(time_slice)==u_time_chs.end()){
      std::set<int> uchs;
      u_time_chs[time_slice] = uchs;
      std::set<int> vchs;
      v_time_chs[time_slice] = vchs;
      std::set<int> wchs;
      w_time_chs[time_slice] = wchs;
    }
    

    for (auto it1 = uwires.begin(); it1!=uwires.end(); it1++){
      const GeomWire *wire = (*it1); 
      double charge = mcell->Get_Wire_Charge(wire);
      double charge_err = mcell->Get_Wire_Charge_Err(wire);
      u_time_chs[time_slice].insert(wire->channel());
      time_ch_charge_map[std::make_pair(time_slice,wire->channel())] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,wire->channel())] = charge_err;

      if (wire->channel() < min_uch) min_uch = wire->channel();
      if (wire->channel() > max_uch) max_uch = wire->channel();
      // std::cout << time_slice << " " << wire->channel() << " " << charge << " " << charge_err << std::endl;
    }

    for (auto it1 = vwires.begin(); it1!=vwires.end(); it1++){
      const GeomWire *wire = (*it1);
      double charge = mcell->Get_Wire_Charge(wire);
      double charge_err = mcell->Get_Wire_Charge_Err(wire);
      v_time_chs[time_slice].insert(wire->channel());
      time_ch_charge_map[std::make_pair(time_slice,wire->channel())] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,wire->channel())] = charge_err;

      if (wire->channel() < min_vch) min_vch = wire->channel();
      if (wire->channel() > max_vch) max_vch = wire->channel();
      // std::cout << time_slice << " " << wire->channel() << " " << charge << " " << charge_err << std::endl;
    }

    for (auto it1 = wwires.begin(); it1!=wwires.end(); it1++){
      const GeomWire *wire = (*it1);
      double charge = mcell->Get_Wire_Charge(wire);
      double charge_err = mcell->Get_Wire_Charge_Err(wire);
      w_time_chs[time_slice].insert(wire->channel());
      time_ch_charge_map[std::make_pair(time_slice,wire->channel())] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,wire->channel())] = charge_err;

      if (wire->channel() < min_wch) min_wch = wire->channel();
      if (wire->channel() > max_wch) max_wch = wire->channel();
      // std::cout << time_slice << " " << wire->channel() << " " << charge << " " << charge_err << std::endl;
    }
  }

  WCP::ToyPointCloud *orig_point_cloud = cluster->get_point_cloud();
  double dis_cut = 20*units::cm;

  // fill in the dead channels ...
  std::vector<std::pair<int, int> > dead_uch_ranges = ct_point_cloud.get_overlap_dead_chs(min_time, max_time, min_uch, max_uch, 0, false);
  std::vector<std::pair<int, int> > dead_vch_ranges = ct_point_cloud.get_overlap_dead_chs(min_time, max_time, min_vch, max_vch, 1, false);
  std::vector<std::pair<int, int> > dead_wch_ranges = ct_point_cloud.get_overlap_dead_chs(min_time, max_time, min_wch, max_wch, 2,false);

  // {
  //   std::cout << min_wch << " " << max_wch << std::endl;
  //   for (auto it = dead_wch_ranges.begin(); it!= dead_wch_ranges.end(); it++){
  //     std::cout << "final W: " <<  it->first << " " << it->second << std::endl;
  //   }
    
  // }
  
  for (int time_slice = min_time; time_slice <= max_time; time_slice ++){
    // U plane 
    if (u_time_chs.find(time_slice)==u_time_chs.end()){
      std::set<int> uchs;
      for (auto it = dead_uch_ranges.begin(); it!=dead_uch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
	  double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
	  if (temp_min_dis > dis_cut) continue;
	  uchs.insert(ch);
	  time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	  time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	}
      }
      u_time_chs[time_slice] = uchs;
    }else{
      for (auto it = dead_uch_ranges.begin(); it!=dead_uch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  if (u_time_chs[time_slice].find(ch)==u_time_chs[time_slice].end()){
	    std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
	    double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
	    if (temp_min_dis > dis_cut) continue;
	    
	    u_time_chs[time_slice].insert(ch);
	    time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	  }
	}
      }
    }

    // V plane
     if (v_time_chs.find(time_slice)==v_time_chs.end()){
      std::set<int> vchs;
      for (auto it = dead_vch_ranges.begin(); it!=dead_vch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
	  double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
	  if (temp_min_dis > dis_cut) continue;
	  
	  vchs.insert(ch);
	  time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	  time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	}
      }
      v_time_chs[time_slice] = vchs;
    }else{
      for (auto it = dead_vch_ranges.begin(); it!=dead_vch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  if (v_time_chs[time_slice].find(ch)==v_time_chs[time_slice].end()){
	    std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
	    double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
	    if (temp_min_dis > dis_cut) continue;
	  
	    v_time_chs[time_slice].insert(ch);
	    time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	  }
	}
      }
    }

     // W plane
     if (w_time_chs.find(time_slice)==w_time_chs.end()){
      std::set<int> wchs;
      for (auto it = dead_wch_ranges.begin(); it!=dead_wch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
	  double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
	  if (temp_min_dis > dis_cut) continue;
	  
	  wchs.insert(ch);
	  time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	  time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	}
      }
      w_time_chs[time_slice] = wchs;
    }else{
      for (auto it = dead_wch_ranges.begin(); it!=dead_wch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  if (w_time_chs[time_slice].find(ch)==w_time_chs[time_slice].end()){
	    std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
	    double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
	    if (temp_min_dis > dis_cut) continue;
	    w_time_chs[time_slice].insert(ch);
	    time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	  }
	}
      }
    }
  }

  // std::cout << cluster->get_cluster_id() << " " << min_time << " " << max_time << " " << min_uch << " " << max_uch << " "  << min_vch << " " << max_vch << " " << min_wch << " " << max_wch << std::endl;
  // if (cluster->get_cluster_id()==2){
  //   for (auto it  = u_time_chs.begin(); it!=u_time_chs.end(); it++){
  //     if (it->first >=2298 && it->first <=2308){
  // 	std::cout << "U: " << it->first << std::endl;
  // 	int prev_ch = -1;
  // 	for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  // 	  if ( (*it1)-prev_ch!=1){
  // 	    std::cout << prev_ch << "; " << (*it1) << " ";
  // 	  }
  // 	  prev_ch = (*it1);
  // 	  //	  std::cout << (*it1) << " " ;
  // 	}
  // 	std::cout << std::endl;
  //     }
  //   }

    
  // }
  
 
  
  // deal with the good channels using WCPoint Cloud ...
  std::map<std::pair<int,int>, std::pair<double,double> > map_u_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_uch, max_uch, 0);
  std::map<std::pair<int,int>, std::pair<double,double> > map_v_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_vch, max_vch, 1);
  std::map<std::pair<int,int>, std::pair<double,double> > map_w_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_wch, max_wch, 2);
  //std::cout << map_u_tcc.size() << " " << map_v_tcc.size() << " " << map_w_tcc.size() << std::endl;
  // Form a connected graph for each view ??? 

  // U plane 
  for (auto it = map_u_tcc.begin(); it!=map_u_tcc.end(); it++){
    int time_slice = (*it).first.first;
    int ch = (*it).first.second;
    double charge = (*it).second.first;
    double charge_err = (*it).second.second;
    if (charge < 0 ) continue;
    if (u_time_chs.find(time_slice)==u_time_chs.end()){
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
      if (temp_min_dis > dis_cut ) continue;
	  
      std::set<int> uchs;
      uchs.insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
      u_time_chs[time_slice] = uchs;
    }else{
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
      if (temp_min_dis > dis_cut) continue;
      
      u_time_chs[time_slice].insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
    }
  }
  // V plane
  for (auto it = map_v_tcc.begin(); it!=map_v_tcc.end(); it++){
    int time_slice = (*it).first.first;
    int ch = (*it).first.second;
    double charge = (*it).second.first;
    double charge_err = (*it).second.second;
    if (charge < 0 ) continue;
    if (v_time_chs.find(time_slice)==v_time_chs.end()){
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
      if (temp_min_dis > dis_cut) continue;
	  
      std::set<int> vchs;
      vchs.insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
      v_time_chs[time_slice] = vchs;
    }else{
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
      if (temp_min_dis > dis_cut) continue;
	  
      v_time_chs[time_slice].insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
    }
  }

  // W plane
  for (auto it = map_w_tcc.begin(); it!=map_w_tcc.end(); it++){
    int time_slice = (*it).first.first;
    int ch = (*it).first.second;
    double charge = (*it).second.first;
    double charge_err = (*it).second.second;
    if (charge < 0 ) continue;
    if (w_time_chs.find(time_slice)==w_time_chs.end()){
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
      if (temp_min_dis > dis_cut) continue;
	  
      std::set<int> wchs;
      wchs.insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
      w_time_chs[time_slice] = wchs;
    }else{
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
      if (temp_min_dis > dis_cut) continue;
	  
      w_time_chs[time_slice].insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
    }
  }
  
  //if (cluster->get_cluster_id()==2){
    // for (auto it  = u_time_chs.begin(); it!=u_time_chs.end(); it++){
    //   if (it->first >=2298 && it->first <=2308){
    // 	std::cout << "U: " << it->first << std::endl;
    // 	int prev_ch = -1;
    // 	for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
    // 	  if ( (*it1)-prev_ch!=1){
    // 	    std::cout << prev_ch << "; " << (*it1) << " ";
    // 	  }
    // 	  prev_ch = (*it1);
    // 	  //	  std::cout << (*it1) << " " ;
    // 	}
    // 	std::cout << std::endl;
    //   }
    // }

    // for (auto it  = v_time_chs.begin(); it!=v_time_chs.end(); it++){
    //   if (it->first >=2298 && it->first <=2308){
    // 	std::cout << "V: " << it->first << std::endl;
    // 	int prev_ch = -1;
    // 	for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
    // 	  if ( (*it1)-prev_ch!=1){
    // 	    std::cout << prev_ch << "; " << (*it1) << " ";
    // 	  }
    // 	  prev_ch = (*it1);
    // 	}
    // 	std::cout << std::endl;
    //   }
    // }

    // for (auto it  = w_time_chs.begin(); it!=w_time_chs.end(); it++){
    //   if (it->first >=2298 && it->first <=2308){
    // 	std::cout << "W: " << it->first << std::endl;
    // 	int prev_ch = -1;
    // 	for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
    // 	  if ( (*it1)-prev_ch!=1){
    // 	    std::cout << prev_ch << "; " << (*it1) << " ";
    // 	  }
    // 	  prev_ch = (*it1);
    // 	}
    // 	std::cout << std::endl;
    //   }
    // }
  // }
  
  
  //  WCP2dToy::WCPHolder *WCholder = new WCP2dToy::WCPHolder();
  for (auto it = u_time_chs.begin(); it!= u_time_chs.end(); it++){
    int time_slice = it->first;
    WCP2dToy::LowmemTiling tiling(time_slice,gds,*holder);
    // recreate the merged wires
    // recreate the merge cells
    tiling.init_good_cells_with_charge(u_time_chs, v_time_chs, w_time_chs, time_ch_charge_map, time_ch_charge_err_map);  
  }
  std::cout << "ImproveCluster_1: " <<  cluster->get_mcells().size() << " " << holder->get_cells().size() << " cells created." << std::endl;
  
  // examine the newly create merged cells
  std::map<int,SMGCSelection> old_time_mcells_map;
  for (auto it = old_mcells.begin(); it!=old_mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    int time_slice = mcell->GetTimeSlice();
    if (old_time_mcells_map.find(time_slice)==old_time_mcells_map.end()){
      SMGCSelection mcells;
      mcells.push_back(mcell);
      old_time_mcells_map[time_slice] = mcells;
    }else{
      old_time_mcells_map[time_slice].push_back(mcell);
    }
  }
  // order the new mcells
  std::map<int, SMGCSelection> new_time_mcells_map;
  std::map<int,SlimMergeGeomCell*> map_index_mcell;
  std::map<SlimMergeGeomCell*,int> map_mcell_index;
  GeomCellSelection& temp_cells = holder->get_cells();
  int index = 0;
  for (auto it = temp_cells.begin(); it!=temp_cells.end(); it++){
    SlimMergeGeomCell *mcell = (SlimMergeGeomCell*)(*it);

    map_index_mcell[index] = mcell;
    map_mcell_index[mcell] = index;
    index++;
    
    int time_slice = mcell->GetTimeSlice();
    if (new_time_mcells_map.find(time_slice)==new_time_mcells_map.end()){
      SMGCSelection mcells;
      mcells.push_back(mcell);
      new_time_mcells_map[time_slice] = mcells;
    }else{
      new_time_mcells_map[time_slice].push_back(mcell);
    }
  }

  // graph ...
  {
    // create a graph and establish connected components ...
    const int N = map_mcell_index.size();
    boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
    	boost::no_property, boost::property<boost::edge_weight_t, double>>
    	temp_graph(N);
    for (auto it=new_time_mcells_map.begin(); it!=new_time_mcells_map.end(); it++){
      int time_slice = it->first;
      if (new_time_mcells_map.find(time_slice+1)!=new_time_mcells_map.end()){
	for (int j=0;j!=it->second.size();j++){
	  SlimMergeGeomCell *mcell1 = it->second.at(j);
	  for (int k=0;k!=new_time_mcells_map[time_slice+1].size();k++){
	    SlimMergeGeomCell *mcell2 = new_time_mcells_map[time_slice+1].at(k);
	    int index1 = map_mcell_index[mcell1];
	    int index2 = map_mcell_index[mcell2];
	    if (mcell1->Overlap_fast(mcell2)){
	      auto edge = add_edge(index1,index2, 1, temp_graph);
	    }
	  }
	}
      }
    }
    std::vector<int> component(num_vertices(temp_graph));
    const int num = connected_components(temp_graph,&component[0]);
    //std::cout << num << std::endl;
    if (num > 1){
      SMGCSet new_mcells_set;
      std::set<int> good_sets;
      // examine each connected component and its mcell ...
      std::vector<int>::size_type i;
      for (i=0;i!=component.size(); ++i){
	if (good_sets.find(component[i])!=good_sets.end()) continue;
	SlimMergeGeomCell *mcell = map_index_mcell[i];
	int time_slice = mcell->GetTimeSlice();
	bool flag_good = false;
	if (!flag_good){
	  // -1 time slice
	  if (old_time_mcells_map.find(time_slice-1) != old_time_mcells_map.end()){
	    for (auto it1 = old_time_mcells_map[time_slice-1].begin(); it1!=old_time_mcells_map[time_slice-1].end(); it1++){
	      if (mcell->Overlap_fast((*it1))){
		flag_good = true;
		break;
	      }
	    }
	  }
	}
	if (!flag_good){
	  // same time_slice
	  if (old_time_mcells_map.find(time_slice) != old_time_mcells_map.end()){
	    for (auto it1 = old_time_mcells_map[time_slice].begin(); it1!=old_time_mcells_map[time_slice].end(); it1++){
	      if (mcell->Overlap_fast((*it1))){
		flag_good = true;
		break;
	      }
	    }
	  }
	}
	if (!flag_good){
	  // +1 time_slice
	  if (old_time_mcells_map.find(time_slice+1) != old_time_mcells_map.end()){
	    for (auto it1 = old_time_mcells_map[time_slice+1].begin(); it1!=old_time_mcells_map[time_slice+1].end(); it1++){
	      if (mcell->Overlap_fast((*it1))){
		flag_good = true;
		break;
	      }
	    }
	  }
	}
	//   // if (!flag_good){
	//   //   // -2 time_slice
	//   //   if (old_time_mcells_map.find(time_slice-2) != old_time_mcells_map.end()){
	//   // 	for (auto it1 = old_time_mcells_map[time_slice-2].begin(); it1!=old_time_mcells_map[time_slice-2].end(); it1++){
	//   // 	  if (mcell->Overlap_fast((*it1))){
	//   // 	    flag_good = true;
	//   // 	    break;
	//   // 	  }
	//   // 	}
	//   //   }
	//   // }
	//   // if (!flag_good){
	//   //   // +2 time_slice
	//   //   if (old_time_mcells_map.find(time_slice+2) != old_time_mcells_map.end()){
	//   // 	for (auto it1 = old_time_mcells_map[time_slice+2].begin(); it1!=old_time_mcells_map[time_slice+2].end(); it1++){
	//   // 	  if (mcell->Overlap_fast((*it1))){
	//   // 	    flag_good = true;
	//   // 	    break;
	//   // 	  }
	//   // 	}
	//   //   }
	//   // }
	if (flag_good){
	  good_sets.insert(component[i]);
	}
      }
      
      // added or delete
      new_time_mcells_map.clear();
      
      for (i=0;i!=component.size(); ++i){
	SlimMergeGeomCell *mcell = map_index_mcell[i];
	if (good_sets.find(component[i])!=good_sets.end()){
	  int time_slice = mcell->GetTimeSlice();
	  if (new_time_mcells_map.find(time_slice)==new_time_mcells_map.end()){
	    SMGCSelection mcells;
	    mcells.push_back(mcell);
	    new_time_mcells_map[time_slice] = mcells;
	  }else{
	    new_time_mcells_map[time_slice].push_back(mcell);
	  }
	}else{
	  //	  delete mcell;
	}
      }
      
    }
    
   
  }


  
  

  // create a new cluster ...
  WCPPID::PR3DCluster *new_cluster = new WCPPID::PR3DCluster(cluster->get_cluster_id());
  for (auto it = new_time_mcells_map.begin(); it!= new_time_mcells_map.end(); it++){
    int time_slice = it->first;
    SMGCSelection& temp_mcells = it->second;
    for (auto it1 = temp_mcells.begin(); it1!=temp_mcells.end(); it1++){
      SlimMergeGeomCell *mcell = (*it1);
      new_cluster->AddCell(mcell,time_slice);
      //      std::cout << time_slice << " " << mcell->get_q() << std::endl;
    }
  }

  std::cout << "ImproveCluster_1: " << holder->get_cells().size() - new_cluster->get_mcells().size() << " cells removed." << std::endl;
  
  //std::cout << cluster->get_cluster_id() << " " << old_mcells.size() << " " << u_time_chs.size() << " " << WCholder->get_ncell() << " " << WCholder->get_nwire() << " " << new_mcells_set.size() << std::endl;
  //Point p(150*units::cm, 35*units::cm, 532*units::cm);
  //std::vector<int> results = ct_point_cloud.convert_3Dpoint_time_ch(p);
  //std::cout << results.at(0) << " " << results.at(1) << " " << results.at(2) << " " << results.at(3) << std::endl;
  
  return new_cluster;
  
  // std::cout << dead_uch_ranges.size() << std::endl;
  // for (size_t i=0;i!=dead_uch_ranges.size();i++){
  //   std::cout << i << " " << dead_uch_ranges.at(i).first << " " << dead_uch_ranges.at(i).second << std::endl;
  // }
  
  
}

WCPPID::PR3DCluster* WCPPID::Improve_PR3DCluster(WCPPID::PR3DCluster* cluster, ToyCTPointCloud& ct_point_cloud,WCPSst::GeomDataSource& gds, WCP2dToy::WCPHolder *holder){

  std::map<int,std::set<int>> u_time_chs; // time chs
  std::map<int,std::set<int>> v_time_chs; // time chs
  std::map<int,std::set<int>> w_time_chs; // time chs
  std::map<std::pair<int,int>,double> time_ch_charge_map;
  std::map<std::pair<int,int>,double> time_ch_charge_err_map;

  int min_time = 1e9;
  int max_time = -1e9;
  int min_uch = 1e9;
  int max_uch = -1e9;
  int min_vch = 1e9;
  int max_vch = -1e9;
  int min_wch = 1e9;
  int max_wch = -1e9;
  
  // fill all map according to existing mcells
  SMGCSelection& old_mcells = cluster->get_mcells();
  for (auto it=old_mcells.begin(); it!=old_mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    GeomWireSelection& uwires = mcell->get_uwires();
    GeomWireSelection& vwires = mcell->get_vwires();
    GeomWireSelection& wwires = mcell->get_wwires();
    int time_slice = mcell->GetTimeSlice();

    if (u_time_chs.find(time_slice)==u_time_chs.end()){
      std::set<int> uchs;
      u_time_chs[time_slice] = uchs;
      std::set<int> vchs;
      v_time_chs[time_slice] = vchs;
      std::set<int> wchs;
      w_time_chs[time_slice] = wchs;
    }

    if (time_slice < min_time) min_time = time_slice;
    if (time_slice > max_time) max_time = time_slice;

    for (auto it1 = uwires.begin(); it1!=uwires.end(); it1++){
      const GeomWire *wire = (*it1); 
      double charge = mcell->Get_Wire_Charge(wire);
      double charge_err = mcell->Get_Wire_Charge_Err(wire);
      u_time_chs[time_slice].insert(wire->channel());
      time_ch_charge_map[std::make_pair(time_slice,wire->channel())] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,wire->channel())] = charge_err;
      if (wire->channel() < min_uch) min_uch = wire->channel();
      if (wire->channel() > max_uch) max_uch = wire->channel();
      // std::cout << time_slice << " " << wire->channel() << " " << charge << " " << charge_err << std::endl;
    }

    for (auto it1 = vwires.begin(); it1!=vwires.end(); it1++){
      const GeomWire *wire = (*it1);
      double charge = mcell->Get_Wire_Charge(wire);
      double charge_err = mcell->Get_Wire_Charge_Err(wire);
      v_time_chs[time_slice].insert(wire->channel());
      time_ch_charge_map[std::make_pair(time_slice,wire->channel())] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,wire->channel())] = charge_err;
      if (wire->channel() < min_vch) min_vch = wire->channel();
      if (wire->channel() > max_vch) max_vch = wire->channel();
      // std::cout << time_slice << " " << wire->channel() << " " << charge << " " << charge_err << std::endl;
    }

    for (auto it1 = wwires.begin(); it1!=wwires.end(); it1++){
      const GeomWire *wire = (*it1);
      double charge = mcell->Get_Wire_Charge(wire);
      double charge_err = mcell->Get_Wire_Charge_Err(wire);
      w_time_chs[time_slice].insert(wire->channel());
      time_ch_charge_map[std::make_pair(time_slice,wire->channel())] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,wire->channel())] = charge_err;
      if (wire->channel() < min_wch) min_wch = wire->channel();
      if (wire->channel() > max_wch) max_wch = wire->channel();
      // std::cout << time_slice << " " << wire->channel() << " " << charge << " " << charge_err << std::endl;
    }
  }

   WCP::ToyPointCloud *orig_point_cloud = cluster->get_point_cloud();
  double dis_cut = 20*units::cm;

  // fill in the dead channels ...
  std::vector<std::pair<int, int> > dead_uch_ranges = ct_point_cloud.get_overlap_dead_chs(min_time, max_time, min_uch, max_uch, 0, false);
  std::vector<std::pair<int, int> > dead_vch_ranges = ct_point_cloud.get_overlap_dead_chs(min_time, max_time, min_vch, max_vch, 1, false);
  std::vector<std::pair<int, int> > dead_wch_ranges = ct_point_cloud.get_overlap_dead_chs(min_time, max_time, min_wch, max_wch, 2, false);


  // std::cout << "dead_uch_ranges: ";
  // for (const auto& range : dead_uch_ranges) {
  //   std::cout << "[" << range.first << ", " << range.second << "] ";
  // }
  // std::cout << std::endl;

  
  for (int time_slice = min_time; time_slice <= max_time; time_slice ++){
    // U plane 
    if (u_time_chs.find(time_slice)==u_time_chs.end()){
      std::set<int> uchs;
      for (auto it = dead_uch_ranges.begin(); it!=dead_uch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
	  double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
	  if (temp_min_dis > dis_cut) continue;
	  uchs.insert(ch);
	  time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	  time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	}
      }
      u_time_chs[time_slice] = uchs;
    }else{
      for (auto it = dead_uch_ranges.begin(); it!=dead_uch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  if (u_time_chs[time_slice].find(ch)==u_time_chs[time_slice].end()){
	    std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
	    double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
	    if (temp_min_dis > dis_cut) continue;
	    
	    u_time_chs[time_slice].insert(ch);
	    time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	  }
	}
      }
    }

    // V plane
     if (v_time_chs.find(time_slice)==v_time_chs.end()){
      std::set<int> vchs;
      for (auto it = dead_vch_ranges.begin(); it!=dead_vch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
	  double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
	  if (temp_min_dis > dis_cut) continue;
	  
	  vchs.insert(ch);
	  time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	  time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	}
      }
      v_time_chs[time_slice] = vchs;
    }else{
      for (auto it = dead_vch_ranges.begin(); it!=dead_vch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  if (v_time_chs[time_slice].find(ch)==v_time_chs[time_slice].end()){
	    std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
	    double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
	    if (temp_min_dis > dis_cut) continue;
	  
	    v_time_chs[time_slice].insert(ch);
	    time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	  }
	}
      }
    }

     // W plane
     if (w_time_chs.find(time_slice)==w_time_chs.end()){
      std::set<int> wchs;
      for (auto it = dead_wch_ranges.begin(); it!=dead_wch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
	  double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
	  if (temp_min_dis > dis_cut) continue;
	  
	  wchs.insert(ch);
	  time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	  time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	}
      }
      w_time_chs[time_slice] = wchs;
    }else{
      for (auto it = dead_wch_ranges.begin(); it!=dead_wch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  if (w_time_chs[time_slice].find(ch)==w_time_chs[time_slice].end()){
	    std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
	    double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
	    if (temp_min_dis > dis_cut) continue;
	    w_time_chs[time_slice].insert(ch);
	    time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	  }
	}
      }
    }
  }

  
  // deal with the good channels using WCPoint Cloud ...
  std::map<std::pair<int,int>, std::pair<double,double> > map_u_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_uch, max_uch, 0);
  std::map<std::pair<int,int>, std::pair<double,double> > map_v_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_vch, max_vch, 1);
  std::map<std::pair<int,int>, std::pair<double,double> > map_w_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_wch, max_wch, 2);
  //std::cout << map_u_tcc.size() << " " << map_v_tcc.size() << " " << map_w_tcc.size() << std::endl;
  // Form a connected graph for each view ??? 

  // U plane 
  for (auto it = map_u_tcc.begin(); it!=map_u_tcc.end(); it++){
    int time_slice = (*it).first.first;
    int ch = (*it).first.second;
    double charge = (*it).second.first;
    double charge_err = (*it).second.second;
    if (charge < 0 ) continue;
    if (u_time_chs.find(time_slice)==u_time_chs.end()){
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
      if (temp_min_dis > dis_cut) continue;
	  
      std::set<int> uchs;
      uchs.insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
      u_time_chs[time_slice] = uchs;
    }else{
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
      if (temp_min_dis > dis_cut) continue;
      
      u_time_chs[time_slice].insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
    }
  }
  // V plane
  for (auto it = map_v_tcc.begin(); it!=map_v_tcc.end(); it++){
    int time_slice = (*it).first.first;
    int ch = (*it).first.second;
    double charge = (*it).second.first;
    double charge_err = (*it).second.second;
    if (charge < 0 ) continue;
    if (v_time_chs.find(time_slice)==v_time_chs.end()){
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
      if (temp_min_dis > dis_cut) continue;
	  
      std::set<int> vchs;
      vchs.insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
      v_time_chs[time_slice] = vchs;
    }else{
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
      if (temp_min_dis > dis_cut) continue;
	  
      v_time_chs[time_slice].insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
    }
  }

  // W plane
  for (auto it = map_w_tcc.begin(); it!=map_w_tcc.end(); it++){
    int time_slice = (*it).first.first;
    int ch = (*it).first.second;
    double charge = (*it).second.first;
    double charge_err = (*it).second.second;
    if (charge < 0 ) continue;
    if (w_time_chs.find(time_slice)==w_time_chs.end()){
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
      if (temp_min_dis > dis_cut) continue;
	  
      std::set<int> wchs;
      wchs.insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
      w_time_chs[time_slice] = wchs;
    }else{
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
      if (temp_min_dis > dis_cut) continue;
	  
      w_time_chs[time_slice].insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
    }
  }
  

  

  
  // for (auto it = time_ch_charge_map.begin(); it!= time_ch_charge_map.end(); it++){
  //   std::cout << it->first.first << " " << it->first.second << std::endl;
  // }
  
  
  // //  std::cout << u_time_chs.size() << " Xin1: " << v_time_chs.size() << " " << w_time_chs.size() << " " << time_ch_charge_map.size() << std::endl;

  std::list<WCPointCloud<double>::WCPoint>& wcps = cluster->get_path_wcps();

  // if (!wcps.empty()) {
  //   std::cout << "Path check: " << wcps.size()
  //             << " first: (" << wcps.front().x << ", " << wcps.front().y << ", " << wcps.front().z << ")"
  //             << " last: (" << wcps.back().x << ", " << wcps.back().y << ", " << wcps.back().z << ")"
  //             << std::endl;
  // }

  if (wcps.size()>1){
    // add in missing pieces based on trajectory points

    std::vector<Point> path_pts;
    double low_dis_limit = 0.3*units::cm;
    for (auto it = wcps.begin(); it!=wcps.end(); it++){
      Point p((*it).x,(*it).y,(*it).z);
      if (path_pts.size()==0){
	path_pts.push_back(p);
      }else{
	double dis = sqrt(pow(p.x-path_pts.back().x,2) +
			  pow(p.y-path_pts.back().y,2) +
			  pow(p.z-path_pts.back().z,2) );
	if (dis < low_dis_limit ){
	  path_pts.push_back(p);
	}else{
	  int ncount = int(dis/low_dis_limit)+1;
	  
	  for (int i=0; i != ncount; i++){
	    Point p1;
	    p1.x = path_pts.back().x + (p.x - path_pts.back().x) * (i+1)/ncount;
	    p1.y = path_pts.back().y + (p.y - path_pts.back().y) * (i+1)/ncount;
	    p1.z = path_pts.back().z + (p.z - path_pts.back().z) * (i+1)/ncount;
	    path_pts.push_back(p1);
	  }
	}
      }
    }

    std::pair<int,int> uch_limits = ct_point_cloud.get_uch_limits();
    std::pair<int,int> vch_limits = ct_point_cloud.get_vch_limits();
    std::pair<int,int> wch_limits = ct_point_cloud.get_wch_limits();

    std::vector<bool> path_pts_flag;
    // std::vector<int> path_pts_counts;
    for (auto it=path_pts.begin(); it!=path_pts.end(); it++){
      std::vector<int> results = ct_point_cloud.convert_3Dpoint_time_ch((*it));

      int nu = 0;
      int nv = 0;
      int nw = 0;

      if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(1)))!=time_ch_charge_map.end()) nu +=2;
      if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(1)-1))!=time_ch_charge_map.end()) nu ++;
      if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(1)+1))!=time_ch_charge_map.end()) nu ++;

      if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(2)))!=time_ch_charge_map.end()) nv +=2;
      if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(2)-1))!=time_ch_charge_map.end()) nv++;
      if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(2)+1))!=time_ch_charge_map.end()) nv++;
      
      if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(3)))!=time_ch_charge_map.end()) nw+=2;
      if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(3)-1))!=time_ch_charge_map.end()) nw ++;
      if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(3)+1))!=time_ch_charge_map.end()) nw ++;

      if (nu>=2&&nv>=2&&nw>=2){
	path_pts_flag.push_back(true);
      }else{
	path_pts_flag.push_back(false);
      }
      // path_pts_counts.push_back(nu+nv+nw);
      
      // std::cout << "Path: " << (*it).x/units::cm << " " << (*it).y/units::cm << " " << (*it).z/units::cm << " " << path_pts_flag.back() << std::endl;
    }

    // std::cout << path_pts.size() << " " << path_pts_flag.size() << " " << cluster->get_cluster_id() << std::endl;
    
    for (size_t i=0;i!=path_pts.size();i++){
      std::vector<int> results = ct_point_cloud.convert_3Dpoint_time_ch(path_pts.at(i));

      if (i==0){
	if (path_pts_flag.at(i) && path_pts_flag.at(i+1)) continue;
      }else if (i+1==path_pts.size()){
	if (path_pts_flag.at(i) && path_pts_flag.at(i-1)) continue;
      }else{
	if (path_pts_flag.at(i-1) && path_pts_flag.at(i) && path_pts_flag.at(i+1)) continue;
      }
      

      int range = 3;
      
      for (int time_slice = results.at(0)-range; time_slice<=results.at(0)+range; time_slice ++){
	if (time_slice <0) continue;
	
	if (u_time_chs.find(time_slice)==u_time_chs.end()){
	  std::set<int> uchs;
	  u_time_chs[time_slice] = uchs;
	  std::set<int> vchs;
	  v_time_chs[time_slice] = vchs;
	  std::set<int> wchs;
	  w_time_chs[time_slice] = wchs;
	}
	for (int ch = results.at(1)-range; ch<=results.at(1)+range; ch++){
	  if (ch < uch_limits.first || ch > uch_limits.second ||
	      pow(ch - results.at(1),2) + pow(time_slice -results.at(0),2)> range*range)
	    continue;
	  u_time_chs[time_slice].insert(ch);
	  if (time_ch_charge_map.find(std::make_pair(time_slice,ch))==time_ch_charge_map.end()){
	    time_ch_charge_map[std::make_pair(time_slice,ch)]=0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)]=0;
	    //std::cout << time_slice << " " << ch << std::endl;
	  }
	}
	for (int ch = results.at(2)-range; ch<=results.at(2)+range; ch++){
	  if (ch < vch_limits.first || ch > vch_limits.second ||
	      pow(ch - results.at(2),2) + pow(time_slice -results.at(0),2)>range*range)
	    continue;
	  v_time_chs[time_slice].insert(ch);
	  if (time_ch_charge_map.find(std::make_pair(time_slice,ch))==time_ch_charge_map.end()){
	    time_ch_charge_map[std::make_pair(time_slice,ch)]=0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)]=0;
	    //std::cout << time_slice << " " << ch << std::endl;
	  }
	}
	for (int ch = results.at(3)-range; ch<=results.at(3)+range; ch++){
	  if (ch < wch_limits.first || ch > wch_limits.second ||
	      pow(ch - results.at(3),2) + pow(time_slice -results.at(0),2)>range*range)
	    continue;
	  w_time_chs[time_slice].insert(ch);
	  if (time_ch_charge_map.find(std::make_pair(time_slice,ch))==time_ch_charge_map.end()){
	    time_ch_charge_map[std::make_pair(time_slice,ch)]=0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)]=0;
	    //std::cout << time_slice << " " << ch << std::endl;
	  }
	}	
      }
    }
    //std::cout << path_pts.size() << std::endl;
    //   std::cout << u_time_chs.size() << " Xin2: " << v_time_chs.size() << " " << w_time_chs.size() << " " << time_ch_charge_map.size() << std::endl;
  }

  
  //  WCP2dToy::WCPHolder *WCholder = new WCP2dToy::WCPHolder();
  for (auto it = u_time_chs.begin(); it!= u_time_chs.end(); it++){
    int time_slice = it->first;
    WCP2dToy::LowmemTiling tiling(time_slice,gds,*holder);
    // recreate the merged wires
    // recreate the merge cells
    tiling.init_good_cells_with_charge(u_time_chs, v_time_chs, w_time_chs, time_ch_charge_map, time_ch_charge_err_map);  
 
    // //  debug ...
    // std::cout << "Slice: [" << time_slice * 4 << ", " << (time_slice+1)*4 << ") ";
    // std::cout << "Layer 2 ";
    // if (!it->second.empty()) {
    //   auto ch_it = it->second.begin();
    //   int range_start = *ch_it;
    //   int prev_ch = *ch_it;
    //   ++ch_it;
    //   for (; ch_it != it->second.end(); ++ch_it) {
    //     if (*ch_it != prev_ch + 1) {
    //       std::cout << "[" << range_start << ", " << prev_ch << ") ";
    //       range_start = *ch_it;
    //     }
    //     prev_ch = *ch_it;
    //   }
    //   std::cout << "[" << range_start << ", " << prev_ch << ") ";
    // }
    // std::cout << "Layer 3 ";
    // auto it1 = v_time_chs.find(time_slice);
    // if (it1 != v_time_chs.end() && !it1->second.empty()) {
    //   auto ch_it = it1->second.begin();
    //   int range_start = *ch_it;
    //   int prev_ch = *ch_it;
    //   ++ch_it;
    //   for (; ch_it != it1->second.end(); ++ch_it) {
    //     if (*ch_it != prev_ch + 1) {
    //       std::cout << "[" << range_start-2400 << ", " << prev_ch-2400 << ") ";
    //       range_start = *ch_it;
    //     }
    //     prev_ch = *ch_it;
    //   }
    //   std::cout << "[" << range_start-2400 << ", " << prev_ch-2400 << ") ";
    // }
    // std::cout << "Layer 4 ";
    // auto it2 = w_time_chs.find(time_slice);
    // if (it2 != w_time_chs.end() && !it2->second.empty()) {
    //   auto ch_it = it2->second.begin();
    //   int range_start = *ch_it;
    //   int prev_ch = *ch_it;
    //   ++ch_it;
    //   for (; ch_it != it2->second.end(); ++ch_it) {
    //     if (*ch_it != prev_ch + 1) {
    //       std::cout << "[" << range_start-4800 << ", " << prev_ch-4800 << ") ";
    //       range_start = *ch_it;
    //     }
    //     prev_ch = *ch_it;
    //   }
    //   std::cout << "[" << range_start-4800 << ", " << prev_ch-4800 << ") ";
    // }
    // std::cout << std::endl;
  }

 
  
  std::cout << "ImproveCluster_2: new cluster " << holder->get_cells().size() << " cells" << std::endl;



  // order the original mcells
  std::map<int,SMGCSelection> old_time_mcells_map;
  for (auto it = old_mcells.begin(); it!=old_mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    int time_slice = mcell->GetTimeSlice();
    if (old_time_mcells_map.find(time_slice)==old_time_mcells_map.end()){
      SMGCSelection mcells;
      mcells.push_back(mcell);
      old_time_mcells_map[time_slice] = mcells;
    }else{
      old_time_mcells_map[time_slice].push_back(mcell);
    }
  }

  // order the new mcells
  std::map<int, SMGCSelection> new_time_mcells_map;
  std::map<int,SlimMergeGeomCell*> map_index_mcell;
  std::map<SlimMergeGeomCell*,int> map_mcell_index;
  GeomCellSelection& temp_cells = holder->get_cells();
  int index = 0;
  for (auto it = temp_cells.begin(); it!=temp_cells.end(); it++){
    SlimMergeGeomCell *mcell = (SlimMergeGeomCell*)(*it);

    map_index_mcell[index] = mcell;
    map_mcell_index[mcell] = index;
    index++;
    
    int time_slice = mcell->GetTimeSlice();
    if (new_time_mcells_map.find(time_slice)==new_time_mcells_map.end()){
      SMGCSelection mcells;
      mcells.push_back(mcell);
      new_time_mcells_map[time_slice] = mcells;
    }else{
      new_time_mcells_map[time_slice].push_back(mcell);
    }
  }

  // graph ...
  {
    // create a graph and establish connected components ...
    const int N = map_mcell_index.size();
    boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
    	boost::no_property, boost::property<boost::edge_weight_t, double>>
    	temp_graph(N);
    for (auto it=new_time_mcells_map.begin(); it!=new_time_mcells_map.end(); it++){
      int time_slice = it->first;
      if (new_time_mcells_map.find(time_slice+1)!=new_time_mcells_map.end()){
	for (int j=0;j!=it->second.size();j++){
	  SlimMergeGeomCell *mcell1 = it->second.at(j);
	  for (int k=0;k!=new_time_mcells_map[time_slice+1].size();k++){
	    SlimMergeGeomCell *mcell2 = new_time_mcells_map[time_slice+1].at(k);
	    int index1 = map_mcell_index[mcell1];
	    int index2 = map_mcell_index[mcell2];
	    if (mcell1->Overlap_fast(mcell2)){
	      auto edge = add_edge(index1,index2, 1, temp_graph);
	    }
	  }
	}
      }
    }
    std::vector<int> component(num_vertices(temp_graph));
    const int num = connected_components(temp_graph,&component[0]);
    //std::cout << num << std::endl;
    if (num > 1){
      SMGCSet new_mcells_set;
      std::set<int> good_sets;
      // examine each connected component and its mcell ...
      std::vector<int>::size_type i;
      for (i=0;i!=component.size(); ++i){
	if (good_sets.find(component[i])!=good_sets.end()) continue;
	SlimMergeGeomCell *mcell = map_index_mcell[i];
	int time_slice = mcell->GetTimeSlice();
	bool flag_good = false;
	if (!flag_good){
	  // -1 time slice
	  if (old_time_mcells_map.find(time_slice-1) != old_time_mcells_map.end()){
	    for (auto it1 = old_time_mcells_map[time_slice-1].begin(); it1!=old_time_mcells_map[time_slice-1].end(); it1++){
	      if (mcell->Overlap_fast((*it1))){
		flag_good = true;
		break;
	      }
	    }
	  }
	}
	if (!flag_good){
	  // same time_slice
	  if (old_time_mcells_map.find(time_slice) != old_time_mcells_map.end()){
	    for (auto it1 = old_time_mcells_map[time_slice].begin(); it1!=old_time_mcells_map[time_slice].end(); it1++){
	      if (mcell->Overlap_fast((*it1))){
		flag_good = true;
		break;
	      }
	    }
	  }
	}
	if (!flag_good){
	  // +1 time_slice
	  if (old_time_mcells_map.find(time_slice+1) != old_time_mcells_map.end()){
	    for (auto it1 = old_time_mcells_map[time_slice+1].begin(); it1!=old_time_mcells_map[time_slice+1].end(); it1++){
	      if (mcell->Overlap_fast((*it1))){
		flag_good = true;
		break;
	      }
	    }
	  }
	}
	//   // if (!flag_good){
	//   //   // -2 time_slice
	//   //   if (old_time_mcells_map.find(time_slice-2) != old_time_mcells_map.end()){
	//   // 	for (auto it1 = old_time_mcells_map[time_slice-2].begin(); it1!=old_time_mcells_map[time_slice-2].end(); it1++){
	//   // 	  if (mcell->Overlap_fast((*it1))){
	//   // 	    flag_good = true;
	//   // 	    break;
	//   // 	  }
	//   // 	}
	//   //   }
	//   // }
	//   // if (!flag_good){
	//   //   // +2 time_slice
	//   //   if (old_time_mcells_map.find(time_slice+2) != old_time_mcells_map.end()){
	//   // 	for (auto it1 = old_time_mcells_map[time_slice+2].begin(); it1!=old_time_mcells_map[time_slice+2].end(); it1++){
	//   // 	  if (mcell->Overlap_fast((*it1))){
	//   // 	    flag_good = true;
	//   // 	    break;
	//   // 	  }
	//   // 	}
	//   //   }
	//   // }
	if (flag_good){
	  good_sets.insert(component[i]);
	}
      }
      
      // added or delete
      new_time_mcells_map.clear();
      
      for (i=0;i!=component.size(); ++i){
	SlimMergeGeomCell *mcell = map_index_mcell[i];
	if (good_sets.find(component[i])!=good_sets.end()){
	  int time_slice = mcell->GetTimeSlice();
	  if (new_time_mcells_map.find(time_slice)==new_time_mcells_map.end()){
	    SMGCSelection mcells;
	    mcells.push_back(mcell);
	    new_time_mcells_map[time_slice] = mcells;
	  }else{
	    new_time_mcells_map[time_slice].push_back(mcell);
	  }
	}else{
	  //	  delete mcell;
	}
      }
      
    }
    
   
  }
  // form cluster ...
  // create a new cluster ...
  WCPPID::PR3DCluster *new_cluster = new WCPPID::PR3DCluster(cluster->get_cluster_id());
  for (auto it = new_time_mcells_map.begin(); it!= new_time_mcells_map.end(); it++){
    int time_slice = it->first;
    SMGCSelection& temp_mcells = it->second;
    for (auto it1 = temp_mcells.begin(); it1!=temp_mcells.end(); it1++){
      SlimMergeGeomCell *mcell = (*it1);
      new_cluster->AddCell(mcell,time_slice);
    }
  }
 
  std::cout << "ImproveCluster_2: " << holder->get_cells().size() - new_cluster->get_mcells().size() << " cells removed." << std::endl;


  
  //std::cout << cluster->get_cluster_id() << " " << old_mcells.size() << " " << u_time_chs.size() << " " << WCholder->get_ncell() << " " << WCholder->get_nwire() << " " << new_mcells_set.size() << std::endl;
  //Point p(150*units::cm, 35*units::cm, 532*units::cm);
  //std::vector<int> results = ct_point_cloud.convert_3Dpoint_time_ch(p);
  //std::cout << results.at(0) << " " << results.at(1) << " " << results.at(2) << " " << results.at(3) << std::endl;
  
  return new_cluster;
}






WCPPID::PR3DCluster* WCPPID::Improve_PR3DCluster(WCPPID::PR3DCluster* cluster1, WCPPID::PR3DCluster* cluster2, ToyCTPointCloud& ct_point_cloud,WCPSst::GeomDataSource& gds, WCP2dToy::WCPHolder *holder){

  std::map<int,std::set<int>> u_time_chs; // time chs
  std::map<int,std::set<int>> v_time_chs; // time chs
  std::map<int,std::set<int>> w_time_chs; // time chs
  std::map<std::pair<int,int>,double> time_ch_charge_map;
  std::map<std::pair<int,int>,double> time_ch_charge_err_map;

  int min_time = 1e9;
  int max_time = -1e9;
  int min_uch = 1e9;
  int max_uch = -1e9;
  int min_vch = 1e9;
  int max_vch = -1e9;
  int min_wch = 1e9;
  int max_wch = -1e9;
  
  // fill all map according to existing mcells
  SMGCSelection& old_mcells = cluster2->get_mcells();
  for (auto it=old_mcells.begin(); it!=old_mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    GeomWireSelection& uwires = mcell->get_uwires();
    GeomWireSelection& vwires = mcell->get_vwires();
    GeomWireSelection& wwires = mcell->get_wwires();
    int time_slice = mcell->GetTimeSlice();

    if (time_slice < min_time) min_time = time_slice;
    if (time_slice > max_time) max_time = time_slice;
    
    if (u_time_chs.find(time_slice)==u_time_chs.end()){
      std::set<int> uchs;
      u_time_chs[time_slice] = uchs;
      std::set<int> vchs;
      v_time_chs[time_slice] = vchs;
      std::set<int> wchs;
      w_time_chs[time_slice] = wchs;
    }
    

    for (auto it1 = uwires.begin(); it1!=uwires.end(); it1++){
      const GeomWire *wire = (*it1); 
      double charge = mcell->Get_Wire_Charge(wire);
      double charge_err = mcell->Get_Wire_Charge_Err(wire);
      u_time_chs[time_slice].insert(wire->channel());
      time_ch_charge_map[std::make_pair(time_slice,wire->channel())] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,wire->channel())] = charge_err;
      // std::cout << time_slice << " " << wire->channel() << " " << charge << " " << charge_err << std::endl;
      if (wire->channel() < min_uch) min_uch = wire->channel();
      if (wire->channel() > max_uch) max_uch = wire->channel();
    }

    for (auto it1 = vwires.begin(); it1!=vwires.end(); it1++){
      const GeomWire *wire = (*it1);
      double charge = mcell->Get_Wire_Charge(wire);
      double charge_err = mcell->Get_Wire_Charge_Err(wire);
      v_time_chs[time_slice].insert(wire->channel());
      time_ch_charge_map[std::make_pair(time_slice,wire->channel())] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,wire->channel())] = charge_err;
      // std::cout << time_slice << " " << wire->channel() << " " << charge << " " << charge_err << std::endl;
      if (wire->channel() < min_vch) min_vch = wire->channel();
      if (wire->channel() > max_vch) max_vch = wire->channel();
    }

    for (auto it1 = wwires.begin(); it1!=wwires.end(); it1++){
      const GeomWire *wire = (*it1);
      double charge = mcell->Get_Wire_Charge(wire);
      double charge_err = mcell->Get_Wire_Charge_Err(wire);
      w_time_chs[time_slice].insert(wire->channel());
      time_ch_charge_map[std::make_pair(time_slice,wire->channel())] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,wire->channel())] = charge_err;
      // std::cout << time_slice << " " << wire->channel() << " " << charge << " " << charge_err << std::endl;
      if (wire->channel() < min_wch) min_wch = wire->channel();
      if (wire->channel() > max_wch) max_wch = wire->channel();
    }
  }

  // for (auto it = time_ch_charge_map.begin(); it!= time_ch_charge_map.end(); it++){
  //   std::cout << it->first.first << " " << it->first.second << std::endl;
  // }
  

  WCP::ToyPointCloud *orig_point_cloud = cluster1->get_point_cloud();
  double dis_cut = 20*units::cm;

  // fill in the dead channels ...
  std::vector<std::pair<int, int> > dead_uch_ranges = ct_point_cloud.get_overlap_dead_chs(min_time, max_time, min_uch, max_uch, 0, false);
  std::vector<std::pair<int, int> > dead_vch_ranges = ct_point_cloud.get_overlap_dead_chs(min_time, max_time, min_vch, max_vch, 1, false);
  std::vector<std::pair<int, int> > dead_wch_ranges = ct_point_cloud.get_overlap_dead_chs(min_time, max_time, min_wch, max_wch, 2, false);

  
  for (int time_slice = min_time; time_slice <= max_time; time_slice ++){
    // U plane 
    if (u_time_chs.find(time_slice)==u_time_chs.end()){
      std::set<int> uchs;
      for (auto it = dead_uch_ranges.begin(); it!=dead_uch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
	  double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
	  if (temp_min_dis > dis_cut) continue;
	  uchs.insert(ch);
	  time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	  time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	}
      }
      u_time_chs[time_slice] = uchs;
    }else{
      for (auto it = dead_uch_ranges.begin(); it!=dead_uch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  if (u_time_chs[time_slice].find(ch)==u_time_chs[time_slice].end()){
	    std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
	    double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
	    if (temp_min_dis > dis_cut) continue;
	    
	    u_time_chs[time_slice].insert(ch);
	    time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	  }
	}
      }
    }

    // V plane
     if (v_time_chs.find(time_slice)==v_time_chs.end()){
      std::set<int> vchs;
      for (auto it = dead_vch_ranges.begin(); it!=dead_vch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
	  double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
	  if (temp_min_dis > dis_cut) continue;
	  
	  vchs.insert(ch);
	  time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	  time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	}
      }
      v_time_chs[time_slice] = vchs;
    }else{
      for (auto it = dead_vch_ranges.begin(); it!=dead_vch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  if (v_time_chs[time_slice].find(ch)==v_time_chs[time_slice].end()){
	    std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
	    double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
	    if (temp_min_dis > dis_cut) continue;
	  
	    v_time_chs[time_slice].insert(ch);
	    time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	  }
	}
      }
    }

     // W plane
     if (w_time_chs.find(time_slice)==w_time_chs.end()){
      std::set<int> wchs;
      for (auto it = dead_wch_ranges.begin(); it!=dead_wch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
	  double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
	  if (temp_min_dis > dis_cut) continue;
	  
	  wchs.insert(ch);
	  time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	  time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	}
      }
      w_time_chs[time_slice] = wchs;
    }else{
      for (auto it = dead_wch_ranges.begin(); it!=dead_wch_ranges.end(); it++){
	for (int ch = (*it).first; ch<=(*it).second; ch++){
	  if (w_time_chs[time_slice].find(ch)==w_time_chs[time_slice].end()){
	    std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
	    double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
	    if (temp_min_dis > dis_cut) continue;
	    w_time_chs[time_slice].insert(ch);
	    time_ch_charge_map[std::make_pair(time_slice,ch)] = 0;
	    time_ch_charge_err_map[std::make_pair(time_slice,ch)] = 0;
	  }
	}
      }
    }
  }

  
  // deal with the good channels using WCPoint Cloud ...
  std::map<std::pair<int,int>, std::pair<double,double> > map_u_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_uch, max_uch, 0);
  std::map<std::pair<int,int>, std::pair<double,double> > map_v_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_vch, max_vch, 1);
  std::map<std::pair<int,int>, std::pair<double,double> > map_w_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_wch, max_wch, 2);
  //std::cout << map_u_tcc.size() << " " << map_v_tcc.size() << " " << map_w_tcc.size() << std::endl;
  // Form a connected graph for each view ??? 

  // U plane 
  for (auto it = map_u_tcc.begin(); it!=map_u_tcc.end(); it++){
    int time_slice = (*it).first.first;
    int ch = (*it).first.second;
    double charge = (*it).second.first;
    double charge_err = (*it).second.second;
    if (charge < 0 ) continue;
    if (u_time_chs.find(time_slice)==u_time_chs.end()){
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
      if (temp_min_dis > dis_cut) continue;
	  
      std::set<int> uchs;
      uchs.insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
      u_time_chs[time_slice] = uchs;
    }else{
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 0);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 0);
      if (temp_min_dis > dis_cut) continue;
      
      u_time_chs[time_slice].insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
    }
  }
  // V plane
  for (auto it = map_v_tcc.begin(); it!=map_v_tcc.end(); it++){
    int time_slice = (*it).first.first;
    int ch = (*it).first.second;
    double charge = (*it).second.first;
    double charge_err = (*it).second.second;
    if (charge < 0 ) continue;
    if (v_time_chs.find(time_slice)==v_time_chs.end()){
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
      if (temp_min_dis > dis_cut) continue;
	  
      std::set<int> vchs;
      vchs.insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
      v_time_chs[time_slice] = vchs;
    }else{
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 1);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 1);
      if (temp_min_dis > dis_cut) continue;
	  
      v_time_chs[time_slice].insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
    }
  }

  // W plane
  for (auto it = map_w_tcc.begin(); it!=map_w_tcc.end(); it++){
    int time_slice = (*it).first.first;
    int ch = (*it).first.second;
    double charge = (*it).second.first;
    double charge_err = (*it).second.second;
    if (charge < 0 ) continue;
    if (w_time_chs.find(time_slice)==w_time_chs.end()){
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
      if (temp_min_dis > dis_cut) continue;
	  
      std::set<int> wchs;
      wchs.insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
      w_time_chs[time_slice] = wchs;
    }else{
      std::pair<double,double> temp_2d_pos = ct_point_cloud.convert_time_ch_2Dpoint(time_slice, ch, 2);
      double temp_min_dis = orig_point_cloud->get_closest_2d_dis(temp_2d_pos.first, temp_2d_pos.second, 2);
      if (temp_min_dis > dis_cut) continue;
	  
      w_time_chs[time_slice].insert(ch);
      time_ch_charge_map[std::make_pair(time_slice,ch)] = charge;
      time_ch_charge_err_map[std::make_pair(time_slice,ch)] = charge_err;
    }
  }
  

  
  
  //  std::cout << u_time_chs.size() << " Xin1: " << v_time_chs.size() << " " << w_time_chs.size() << " " << time_ch_charge_map.size() << std::endl;

  

  {
    std::list<WCPointCloud<double>::WCPoint>& wcps = cluster2->get_path_wcps();
    if (wcps.size()>1){ 
      std::vector<Point> path_pts;
      // add in missing pieces based on trajectory points
      double low_dis_limit = 0.3*units::cm;
      for (auto it = wcps.begin(); it!=wcps.end(); it++){
	Point p((*it).x,(*it).y,(*it).z);
	if (path_pts.size()==0){
	  path_pts.push_back(p);
	}else{
	  double dis = sqrt(pow(p.x-path_pts.back().x,2) +
			    pow(p.y-path_pts.back().y,2) +
			    pow(p.z-path_pts.back().z,2) );
	  if (dis < low_dis_limit ){
	    path_pts.push_back(p);
	  }else{
	    int ncount = int(dis/low_dis_limit)+1;
	    
	    for (int i=0; i != ncount; i++){
	      Point p1;
	      p1.x = path_pts.back().x + (p.x - path_pts.back().x) * (i+1)/ncount;
	      p1.y = path_pts.back().y + (p.y - path_pts.back().y) * (i+1)/ncount;
	      p1.z = path_pts.back().z + (p.z - path_pts.back().z) * (i+1)/ncount;
	      path_pts.push_back(p1);
	    }
	  }
	}
      }
      
      std::pair<int,int> uch_limits = ct_point_cloud.get_uch_limits();
      std::pair<int,int> vch_limits = ct_point_cloud.get_vch_limits();
      std::pair<int,int> wch_limits = ct_point_cloud.get_wch_limits();
      
      std::vector<bool> path_pts_flag;
      for (auto it=path_pts.begin(); it!=path_pts.end(); it++){
      	std::vector<int> results = ct_point_cloud.convert_3Dpoint_time_ch((*it));
	
      	int nu = 0;
      	int nv = 0;
      	int nw = 0;
	
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(1)))!=time_ch_charge_map.end()) nu +=2;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(1)-1))!=time_ch_charge_map.end()) nu ++;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(1)+1))!=time_ch_charge_map.end()) nu ++;
	
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(2)))!=time_ch_charge_map.end()) nv +=2;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(2)-1))!=time_ch_charge_map.end()) nv++;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(2)+1))!=time_ch_charge_map.end()) nv++;
	
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(3)))!=time_ch_charge_map.end()) nw+=2;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(3)-1))!=time_ch_charge_map.end()) nw ++;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(3)+1))!=time_ch_charge_map.end()) nw ++;
	
      	if (nu>=2&&nv>=2&&nw>=2){
      	  path_pts_flag.push_back(true);
      	}else{
      	  path_pts_flag.push_back(false);
      	}
      	//	if (fabs((*it).x)<10*units::cm){
      	// std::cout << "Path: " << (*it).x/units::cm << " " << (*it).y/units::cm << " " << (*it).z/units::cm << " " << path_pts_flag.back() << " " << nu << " " << nv << " " << nw << std::endl;
      	// }
      }
      
      // std::cout << path_pts.size() << " " << path_pts_flag.size() << " " << cluster->get_cluster_id() << std::endl;
      
      for (size_t i=0;i!=path_pts.size();i++){
	std::vector<int> results = ct_point_cloud.convert_3Dpoint_time_ch(path_pts.at(i));
	
	if (i==0){
	  if (path_pts_flag.at(i) && path_pts_flag.at(i+1)) continue;
	}else if (i+1==path_pts.size()){
	  if (path_pts_flag.at(i) && path_pts_flag.at(i-1)) continue;
	}else{
	  if (path_pts_flag.at(i-1) && path_pts_flag.at(i) && path_pts_flag.at(i+1)) continue;
	}
	
	// if (fabs(path_pts.at(i).x) < 3*units::cm)
	//   std::cout << results.at(0) << " " << results.at(1) << " " << results.at(2) << " " << results.at(3) << std::endl;
	
	int range = 3;
	
	for (int time_slice = results.at(0)-range; time_slice<=results.at(0)+range; time_slice ++){
	  if (time_slice <0) continue;
	  
	  if (u_time_chs.find(time_slice)==u_time_chs.end()){
	    std::set<int> uchs;
	    u_time_chs[time_slice] = uchs;
	    std::set<int> vchs;
	    v_time_chs[time_slice] = vchs;
	    std::set<int> wchs;
	    w_time_chs[time_slice] = wchs;
	  }
	  for (int ch = results.at(1)-range; ch<=results.at(1)+range; ch++){
	    if (ch < uch_limits.first || ch > uch_limits.second ||
		pow(ch - results.at(1),2) + pow(time_slice -results.at(0),2)> range*range)
	      continue;
	    u_time_chs[time_slice].insert(ch);
	    if (time_ch_charge_map.find(std::make_pair(time_slice,ch))==time_ch_charge_map.end()){
	      time_ch_charge_map[std::make_pair(time_slice,ch)]=0;
	      time_ch_charge_err_map[std::make_pair(time_slice,ch)]=0;
	      // if (fabs(path_pts.at(i).x) < 3*units::cm)
	      // 	std::cout << time_slice << " U " << ch << std::endl;
	    }
	  }
	  for (int ch = results.at(2)-range; ch<=results.at(2)+range; ch++){
	    if (ch < vch_limits.first || ch > vch_limits.second ||
		pow(ch - results.at(2),2) + pow(time_slice -results.at(0),2)>range*range)
	      continue;
	    v_time_chs[time_slice].insert(ch);
	    if (time_ch_charge_map.find(std::make_pair(time_slice,ch))==time_ch_charge_map.end()){
	      time_ch_charge_map[std::make_pair(time_slice,ch)]=0;
	      time_ch_charge_err_map[std::make_pair(time_slice,ch)]=0;
	      // if (fabs(path_pts.at(i).x) < 3*units::cm)
	      // 	std::cout << time_slice << " V " << ch << std::endl;
	    }
	  }
	  for (int ch = results.at(3)-range; ch<=results.at(3)+range; ch++){
	    if (ch < wch_limits.first || ch > wch_limits.second ||
		pow(ch - results.at(3),2) + pow(time_slice -results.at(0),2)>range*range)
	      continue;
	    w_time_chs[time_slice].insert(ch);
	    if (time_ch_charge_map.find(std::make_pair(time_slice,ch))==time_ch_charge_map.end()){
	      time_ch_charge_map[std::make_pair(time_slice,ch)]=0;
	      time_ch_charge_err_map[std::make_pair(time_slice,ch)]=0;
	      //if (fabs(path_pts.at(i).x) < 3*units::cm)
	      //std::cout << time_slice << " W " << ch << std::endl;
	    }
	  }	
	}
      }

      
      //std::cout << path_pts.size() << std::endl;
      //   std::cout << u_time_chs.size() << " Xin2: " << v_time_chs.size() << " " << w_time_chs.size() << " " << time_ch_charge_map.size() << std::endl;
    }
  }



  
  {
    std::list<WCPointCloud<double>::WCPoint>& wcps = cluster1->get_path_wcps();
    if (wcps.size()>1){ 
      std::vector<Point> path_pts;
      // add in missing pieces based on trajectory points
      double low_dis_limit = 0.3*units::cm;
      for (auto it = wcps.begin(); it!=wcps.end(); it++){
	Point p((*it).x,(*it).y,(*it).z);
	if (path_pts.size()==0){
	  path_pts.push_back(p);
	}else{
	  double dis = sqrt(pow(p.x-path_pts.back().x,2) +
			    pow(p.y-path_pts.back().y,2) +
			    pow(p.z-path_pts.back().z,2) );
	  if (dis < low_dis_limit ){
	    path_pts.push_back(p);
	  }else{
	    int ncount = int(dis/low_dis_limit)+1;
	    
	    for (int i=0; i != ncount; i++){
	      Point p1;
	      p1.x = path_pts.back().x + (p.x - path_pts.back().x) * (i+1)/ncount;
	      p1.y = path_pts.back().y + (p.y - path_pts.back().y) * (i+1)/ncount;
	      p1.z = path_pts.back().z + (p.z - path_pts.back().z) * (i+1)/ncount;
	      path_pts.push_back(p1);
	    }
	  }
	}
      }
      
      std::pair<int,int> uch_limits = ct_point_cloud.get_uch_limits();
      std::pair<int,int> vch_limits = ct_point_cloud.get_vch_limits();
      std::pair<int,int> wch_limits = ct_point_cloud.get_wch_limits();
      
      std::vector<bool> path_pts_flag;
      for (auto it=path_pts.begin(); it!=path_pts.end(); it++){
      	std::vector<int> results = ct_point_cloud.convert_3Dpoint_time_ch((*it));
	
      	int nu = 0;
      	int nv = 0;
      	int nw = 0;
	
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(1)))!=time_ch_charge_map.end()) nu +=2;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(1)-1))!=time_ch_charge_map.end()) nu ++;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(1)+1))!=time_ch_charge_map.end()) nu ++;
	
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(2)))!=time_ch_charge_map.end()) nv +=2;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(2)-1))!=time_ch_charge_map.end()) nv++;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(2)+1))!=time_ch_charge_map.end()) nv++;
	
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(3)))!=time_ch_charge_map.end()) nw+=2;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(3)-1))!=time_ch_charge_map.end()) nw ++;
      	if (time_ch_charge_map.find(std::make_pair(results.at(0),results.at(3)+1))!=time_ch_charge_map.end()) nw ++;
	
      	if (nu>=2&&nv>=2&&nw>=2){
      	  path_pts_flag.push_back(true);
      	}else{
      	  path_pts_flag.push_back(false);
      	}
	
      	//  std::cout << "Path: " << (*it).x/units::cm << " " << (*it).y/units::cm << " " << (*it).z/units::cm << " " << path_pts_flag.back() << std::endl;
      }
      
      // std::cout << path_pts.size() << " " << path_pts_flag.size() << " " << cluster->get_cluster_id() << std::endl;
      
      for (size_t i=0;i!=path_pts.size();i++){
	std::vector<int> results = ct_point_cloud.convert_3Dpoint_time_ch(path_pts.at(i));
	
	if (i==0){
	  if (path_pts_flag.at(i) && path_pts_flag.at(i+1)) continue;
	}else if (i+1==path_pts.size()){
	  if (path_pts_flag.at(i) && path_pts_flag.at(i-1)) continue;
	}else{
	  if (path_pts_flag.at(i-1) && path_pts_flag.at(i) && path_pts_flag.at(i+1)) continue;
	}
	
	
	int range = 3;
	
	for (int time_slice = results.at(0)-range; time_slice<=results.at(0)+range; time_slice ++){
	  if (time_slice <0) continue;
	  
	  if (u_time_chs.find(time_slice)==u_time_chs.end()){
	    std::set<int> uchs;
	    u_time_chs[time_slice] = uchs;
	    std::set<int> vchs;
	    v_time_chs[time_slice] = vchs;
	    std::set<int> wchs;
	    w_time_chs[time_slice] = wchs;
	  }
	  for (int ch = results.at(1)-range; ch<=results.at(1)+range; ch++){
	    if (ch < uch_limits.first || ch > uch_limits.second ||
		pow(ch - results.at(1),2) + pow(time_slice -results.at(0),2)> range*range)
	      continue;
	    u_time_chs[time_slice].insert(ch);
	    if (time_ch_charge_map.find(std::make_pair(time_slice,ch))==time_ch_charge_map.end()){
	      time_ch_charge_map[std::make_pair(time_slice,ch)]=0;
	      time_ch_charge_err_map[std::make_pair(time_slice,ch)]=0;
	      //std::cout << time_slice << " " << ch << std::endl;
	    }
	  }
	  for (int ch = results.at(2)-range; ch<=results.at(2)+range; ch++){
	    if (ch < vch_limits.first || ch > vch_limits.second ||
		pow(ch - results.at(2),2) + pow(time_slice -results.at(0),2)>range*range)
	      continue;
	    v_time_chs[time_slice].insert(ch);
	    if (time_ch_charge_map.find(std::make_pair(time_slice,ch))==time_ch_charge_map.end()){
	      time_ch_charge_map[std::make_pair(time_slice,ch)]=0;
	      time_ch_charge_err_map[std::make_pair(time_slice,ch)]=0;
	      //std::cout << time_slice << " " << ch << std::endl;
	    }
	  }
	  for (int ch = results.at(3)-range; ch<=results.at(3)+range; ch++){
	    if (ch < wch_limits.first || ch > wch_limits.second ||
		pow(ch - results.at(3),2) + pow(time_slice -results.at(0),2)>range*range)
	      continue;
	    w_time_chs[time_slice].insert(ch);
	    if (time_ch_charge_map.find(std::make_pair(time_slice,ch))==time_ch_charge_map.end()){
	      time_ch_charge_map[std::make_pair(time_slice,ch)]=0;
	      time_ch_charge_err_map[std::make_pair(time_slice,ch)]=0;
	      //std::cout << time_slice << " " << ch << std::endl;
	    }
	  }	
	}
      }
      //std::cout << path_pts.size() << std::endl;
      //   std::cout << u_time_chs.size() << " Xin2: " << v_time_chs.size() << " " << w_time_chs.size() << " " << time_ch_charge_map.size() << std::endl;
    }
  }
  
  // for (auto it = w_time_chs.begin(); it!= w_time_chs.end(); it++){
  //   int time_slice = it->first;
  //   for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  //     //std::cout << "W: " << time_slice << " " << (*it1) << " " << time_ch_charge_map[std::make_pair(time_slice, *it1)] << std::endl;
  //     if (time_ch_charge_map[std::make_pair(time_slice, *it1)] <0)
  // 	time_ch_charge_map[std::make_pair(time_slice, *it1)] = 0;
      
  //   }
  // }
  // for (auto it = u_time_chs.begin(); it!= u_time_chs.end(); it++){
  //   int time_slice = it->first;
  //   for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  //     // std::cout << "U: " << time_slice << " " << (*it1) << " " << time_ch_charge_map[std::make_pair(time_slice, *it1)] << std::endl;
  //     if (time_ch_charge_map[std::make_pair(time_slice, *it1)] <0)
  // 	time_ch_charge_map[std::make_pair(time_slice, *it1)] = 0;
      
  //   }
  // }
  // for (auto it = v_time_chs.begin(); it!= v_time_chs.end(); it++){
  //   int time_slice = it->first;
  //   for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  //     //std::cout << "V: " << time_slice << " " << (*it1) << " " << time_ch_charge_map[std::make_pair(time_slice, *it1)] << std::endl;
  //     if (time_ch_charge_map[std::make_pair(time_slice, *it1)] <0)
  // 	time_ch_charge_map[std::make_pair(time_slice, *it1)] = 0;
      
  //   }
  // }
  

  
    //  WCP2dToy::WCPHolder *WCholder = new WCP2dToy::WCPHolder();
  for (auto it = u_time_chs.begin(); it!= u_time_chs.end(); it++){
    int time_slice = it->first;
    // if (time_slice == 800){
    //   for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
    // 	std::cout << (*it1) << std::endl;
    //   }
    // }
    
    WCP2dToy::LowmemTiling tiling(time_slice,gds,*holder);
    // recreate the merged wires
    // recreate the merge cells
    tiling.init_good_cells_with_charge(u_time_chs, v_time_chs, w_time_chs, time_ch_charge_map, time_ch_charge_err_map);  
  }

  // order the original mcells
  std::map<int,SMGCSelection> old_time_mcells_map;
  for (auto it = old_mcells.begin(); it!=old_mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    int time_slice = mcell->GetTimeSlice();
    if (old_time_mcells_map.find(time_slice)==old_time_mcells_map.end()){
      SMGCSelection mcells;
      mcells.push_back(mcell);
      old_time_mcells_map[time_slice] = mcells;
    }else{
      old_time_mcells_map[time_slice].push_back(mcell);
    }
  }

  // order the new mcells
  std::map<int, SMGCSelection> new_time_mcells_map;
  std::map<int,SlimMergeGeomCell*> map_index_mcell;
  std::map<SlimMergeGeomCell*,int> map_mcell_index;
  GeomCellSelection& temp_cells = holder->get_cells();
  int index = 0;
  for (auto it = temp_cells.begin(); it!=temp_cells.end(); it++){
    SlimMergeGeomCell *mcell = (SlimMergeGeomCell*)(*it);
    //if (mcell->GetTimeSlice()==800){
    //  std::cout << "haha " << std::endl;
    // }

    map_index_mcell[index] = mcell;
    map_mcell_index[mcell] = index;
    index++;
    
    int time_slice = mcell->GetTimeSlice();
    if (new_time_mcells_map.find(time_slice)==new_time_mcells_map.end()){
      SMGCSelection mcells;
      mcells.push_back(mcell);
      new_time_mcells_map[time_slice] = mcells;
    }else{
      new_time_mcells_map[time_slice].push_back(mcell);
    }
  }

  // graph ...
  {
    // create a graph and establish connected components ...
    const int N = map_mcell_index.size();
    boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
    	boost::no_property, boost::property<boost::edge_weight_t, double>>
    	temp_graph(N);
    for (auto it=new_time_mcells_map.begin(); it!=new_time_mcells_map.end(); it++){
      int time_slice = it->first;
      if (new_time_mcells_map.find(time_slice+1)!=new_time_mcells_map.end()){
	for (int j=0;j!=it->second.size();j++){
	  SlimMergeGeomCell *mcell1 = it->second.at(j);
	  for (int k=0;k!=new_time_mcells_map[time_slice+1].size();k++){
	    SlimMergeGeomCell *mcell2 = new_time_mcells_map[time_slice+1].at(k);
	    int index1 = map_mcell_index[mcell1];
	    int index2 = map_mcell_index[mcell2];
	    if (mcell1->Overlap_fast(mcell2)){
	      auto edge = add_edge(index1,index2, 1, temp_graph);
	    }
	  }
	}
      }
    }
    std::vector<int> component(num_vertices(temp_graph));
    const int num = connected_components(temp_graph,&component[0]);
    //std::cout << num << std::endl;
    if (num > 1){
      SMGCSet new_mcells_set;
      std::set<int> good_sets;
      // examine each connected component and its mcell ...
      std::vector<int>::size_type i;
      for (i=0;i!=component.size(); ++i){
	if (good_sets.find(component[i])!=good_sets.end()) continue;
	SlimMergeGeomCell *mcell = map_index_mcell[i];
	int time_slice = mcell->GetTimeSlice();
	bool flag_good = false;
	if (!flag_good){
	  // -1 time slice
	  if (old_time_mcells_map.find(time_slice-1) != old_time_mcells_map.end()){
	    for (auto it1 = old_time_mcells_map[time_slice-1].begin(); it1!=old_time_mcells_map[time_slice-1].end(); it1++){
	      if (mcell->Overlap_fast((*it1))){
		flag_good = true;
		break;
	      }
	    }
	  }
	}
	if (!flag_good){
	  // same time_slice
	  if (old_time_mcells_map.find(time_slice) != old_time_mcells_map.end()){
	    for (auto it1 = old_time_mcells_map[time_slice].begin(); it1!=old_time_mcells_map[time_slice].end(); it1++){
	      if (mcell->Overlap_fast((*it1))){
		flag_good = true;
		break;
	      }
	    }
	  }
	}
	if (!flag_good){
	  // +1 time_slice
	  if (old_time_mcells_map.find(time_slice+1) != old_time_mcells_map.end()){
	    for (auto it1 = old_time_mcells_map[time_slice+1].begin(); it1!=old_time_mcells_map[time_slice+1].end(); it1++){
	      if (mcell->Overlap_fast((*it1))){
		flag_good = true;
		break;
	      }
	    }
	  }
	}

	if (flag_good){
	  good_sets.insert(component[i]);
	}
      }
      
      // added or delete
      new_time_mcells_map.clear();
      
      for (i=0;i!=component.size(); ++i){
	SlimMergeGeomCell *mcell = map_index_mcell[i];
	if (good_sets.find(component[i])!=good_sets.end()){
	  int time_slice = mcell->GetTimeSlice();
	  if (new_time_mcells_map.find(time_slice)==new_time_mcells_map.end()){
	    SMGCSelection mcells;
	    mcells.push_back(mcell);
	    new_time_mcells_map[time_slice] = mcells;
	  }else{
	    new_time_mcells_map[time_slice].push_back(mcell);
	  }
	}else{
	  //	  std::cout << mcell->GetTimeSlice() << " h" << std::endl; 
	  //	  delete mcell;
	}
      }
      
    }
    
   
  }
  // form cluster ...
  // create a new cluster ...
  WCPPID::PR3DCluster *new_cluster = new WCPPID::PR3DCluster(cluster2->get_cluster_id());
  for (auto it = new_time_mcells_map.begin(); it!= new_time_mcells_map.end(); it++){
    int time_slice = it->first;
    SMGCSelection& temp_mcells = it->second;
    for (auto it1 = temp_mcells.begin(); it1!=temp_mcells.end(); it1++){
      SlimMergeGeomCell *mcell = (*it1);
      new_cluster->AddCell(mcell,time_slice);
    }
  }
  
  return new_cluster;
}
