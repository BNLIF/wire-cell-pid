#include "WCPPID/PR3DCluster.h" 
#include "WCPData/TPCParams.h"
#include "WCPData/Singleton.h"

#include "TMatrixDEigen.h"
#include "TVector3.h"
#include "TH2F.h"
#include <chrono>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>


#include <Eigen/IterativeLinearSolvers>

using namespace WCP;

#include "PR3DCluster_graph.h"
#include "PR3DCluster_graph_overclustering.h"
#include "PR3DCluster_steiner.h"
#include "PR3DCluster_path.h"
#include "PR3DCluster_trajectory_fit.h"
#include "PR3DCluster_dQ_dx_fit.h"
#include "PR3DCluster_crawl.h"
#include "PR3DCluster_pattern_recognition.h"

#include "PR3DCluster_nu_proto_vertex.h"
#include "PR3DCluster_multi_track_fitting.h"
#include "PR3DCluster_multi_dQ_dx_fit.h"

#include "PR3DCluster_point_clustering.h"

void WCPPID::PR3DCluster::do_tracking(WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const GeomWire*, SMGCSelection > >& global_wc_map, double time, bool flag_dQ_dx_fit_reg, bool flag_dQ_dx_fit){
  const bool flag_timing = false; // set to true to enable timing printouts
  using DSClock = std::chrono::high_resolution_clock;
  using DSms = std::chrono::duration<double, std::milli>;
  auto ds_t0 = DSClock::now();

  fine_tracking_path.clear();
  dQ.clear();
  dx.clear();
  pu.clear();
  pv.clear();
  pw.clear();
  pt.clear();
  reduced_chi2.clear();
  
  
  
  bool flag_1st_tracking = true;
  bool flag_2nd_tracking = true;
  bool flag_dQ_dx = flag_dQ_dx_fit;
 
   // prepare the data for the fit, do not contain everything ...
  // form from the cluster ...
  std::map<std::pair<int,int>,std::tuple<double,double, int> > map_2D_ut_charge;
  std::map<std::pair<int,int>,std::tuple<double,double, int> > map_2D_vt_charge;
  std::map<std::pair<int,int>,std::tuple<double,double, int> > map_2D_wt_charge;
  ds_t0 = DSClock::now();
  prepare_data(ct_point_cloud, global_wc_map, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge);
  if (flag_timing) std::cout << "do_single_tracking timing: prepare_data took " << std::chrono::duration_cast<DSms>(DSClock::now()-ds_t0).count() << " ms" << std::endl;

  //  for (auto it = map_2D_wt_charge.begin(); it!=map_2D_wt_charge.end(); it++){
  //  if (it->first.first + 4800 > 8256) std::cout << "abc " << it->first.first + 4800 << std::endl;
  // }
  
  // std::cout << "Total Measurements: " << map_2D_ut_charge.size() + map_2D_vt_charge.size()  + map_2D_wt_charge.size() << std::endl;
  // for (const auto& it : map_2D_vt_charge) {
  //   int time = it.first.second*4;
  //   int channel = it.first.first;
  //   double charge = std::get<0>(it.second);
  //   double charge_err = std::get<1>(it.second);
  //   int flag = std::get<2>(it.second);
  //   std::cout << "CoordReadout: (APA=0, Time=" << time << ", Channel=" << channel << ") -> Charge=" << static_cast<int>(charge)
  //         << ", ChargeErr=" << static_cast<int>(charge_err) << ", Flag=" << flag << std::endl;
  // }


  // first round of organizing the path from the path_wcps (shortest path)
  double low_dis_limit = 1.2*units::cm;
  double end_point_limit = 0.6*units::cm;
  //std::cout << path_wcps.size() << std::endl;
  ds_t0 = DSClock::now();
  PointVector pts = organize_wcps_path(ct_point_cloud, path_wcps,low_dis_limit, end_point_limit); 
  if (pts.size()==0) return;
  else if (pts.size()==1){
    if (sqrt(pow(path_wcps.back().x - pts.back().x,2)+pow(path_wcps.back().y - pts.back().y,2)+pow(path_wcps.back().z - pts.back().z,2))<0.01*units::cm ){
      return;
    }else{
      Point p2(path_wcps.back().x, path_wcps.back().y, path_wcps.back().z);
      pts.push_back(p2);
    }
  }
 
  // std::cout << "After organization " << pts.size() << std::endl;
  if (flag_timing) std::cout << "do_single_tracking timing: organize path " << std::chrono::duration_cast<DSms>(DSClock::now()-ds_t0).count() << " ms" << std::endl;


  // for (size_t i=0;i+1!=pts.size();i++){
  //   std::cout << i << " " << pts.at(i) << " " << sqrt(pow(pts.at(i+1).x-pts.at(i).x,2)+pow(pts.at(i+1).y - pts.at(i).y,2)+pow(pts.at(i+1).z-pts.at(i).z,2))<< std::endl;
  // }
  
  // form association ...
  // map 3D index to set of 2D points
  std::map<int,std::pair<std::set<std::pair<int,int>>, float> > map_3D_2DU_set;
  std::map<int,std::pair<std::set<std::pair<int,int>>, float> > map_3D_2DV_set;
  std::map<int,std::pair<std::set<std::pair<int,int>>, float> > map_3D_2DW_set;
  // map 2D points to 3D indices
  std::map<std::pair<int,int>,std::set<int>> map_2DU_3D_set;
  std::map<std::pair<int,int>,std::set<int>> map_2DV_3D_set;
  std::map<std::pair<int,int>,std::set<int>> map_2DW_3D_set;
  
  if (flag_1st_tracking){
    ds_t0 = DSClock::now();
    form_map(ct_point_cloud, pts,
	     map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,
	     map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set,
	     map_2DU_3D_set, map_2DV_3D_set, map_2DW_3D_set);
    
    // for (size_t i=0;i!=pts.size();i++){
    //   std::cout << i << " " << pts.at(i) << " " << map_3D_2DU_set[i].first.size() << " " << map_3D_2DV_set[i].first.size() << " " << map_3D_2DW_set[i].first.size()  << " " << map_3D_2DU_set[i].second << " " <<  map_3D_2DV_set[i].second << " " << map_3D_2DW_set[i].second << std::endl; 
    //   //   std::cout << i << " " << pts.at(i) << " " << sqrt(pow(pts.at(i+1).x-pts.at(i).x,2)+pow(pts.at(i+1).y - pts.at(i).y,2)+pow(pts.at(i+1).z-pts.at(i).z,2))/units::cm << " " << map_3D_2DU_set[i].first.size() << " " << map_3D_2DV_set[i].first.size() << " " << map_3D_2DW_set[i].first.size() << std::endl;
    //  }
    
    trajectory_fit(pts, map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set,
		   map_2DU_3D_set, map_2DV_3D_set, map_2DW_3D_set,
		   map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge);
    if (flag_timing) std::cout << "do_single_tracking timing: 1st trajectory_fit took " << std::chrono::duration_cast<DSms>(DSClock::now()-ds_t0).count() << " ms" << std::endl;
  }
  
  // for (size_t i=0;i+1!=pts.size();i++){
  //   std::cout << i << " " << pts.at(i) << " " << sqrt(pow(pts.at(i+1).x-pts.at(i).x,2)+pow(pts.at(i+1).y - pts.at(i).y,2)+pow(pts.at(i+1).z-pts.at(i).z,2))<< std::endl;
  // }
  //  std::cout << "Second round fit " << std::endl;
  if (pts.size()==2){
    if (sqrt(pow(pts.front().x - pts.back().x,2) + pow(pts.front().y - pts.back().y,2) + pow(pts.front().z - pts.back().z,2)) < 0.1*units::cm){
      pts.clear();
      Point p1(path_wcps.front().x, path_wcps.front().y, path_wcps.front().z);
      pts.push_back(p1);
      Point p2(path_wcps.back().x, path_wcps.back().y, path_wcps.back().z);
      pts.push_back(p2);
    }
  }

  if (pts.size()<=1) return;
  

  if (flag_2nd_tracking){
    // second round trajectory fit ...
    low_dis_limit = 0.6*units::cm;
    end_point_limit = 0.3*units::cm;

   


    ds_t0 = DSClock::now();
    organize_ps_path(ct_point_cloud, pts, low_dis_limit, end_point_limit); 
    if (flag_timing) std::cout << "do_single_tracking timing: organize path " << std::chrono::duration_cast<DSms>(DSClock::now()-ds_t0).count() << " ms" << std::endl;
    
    // std::cout << pts.size() << std::endl;
    //  for (size_t i = 0; i < pts.size(); ++i) {
    //   std::cout << "pts[" << i << "] = (" << pts[i].x << ", " << pts[i].y << ", " << pts[i].z << ")" << std::endl;
    // }
    
    map_3D_2DU_set.clear();
    map_3D_2DV_set.clear();
    map_3D_2DW_set.clear();
    // map 2D points to 3D indices
    map_2DU_3D_set.clear();
    map_2DV_3D_set.clear();
    map_2DW_3D_set.clear();
    
    ds_t0 = DSClock::now();
    form_map(ct_point_cloud, pts,
	     map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,
	     map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set,
	     map_2DU_3D_set, map_2DV_3D_set, map_2DW_3D_set);
    
    // for (size_t i=0;i!=pts.size();i++){
    //   std::cout << i << " " << pts.at(i) << " " << map_3D_2DU_set[i].first.size() << " " << map_3D_2DV_set[i].first.size() << " " << map_3D_2DW_set[i].first.size() << std::endl;
    // }
    
    //      std::cout << pts.size() <<  " " << map_3D_2DU_set.size() << " " << map_3D_2DW_set.size() << " " << map_3D_2DV_set.size() << " " << map_2DU_3D_set.size() << " " << map_2DV_3D_set.size() << " " << map_2DW_3D_set.size() << " " << map_2D_ut_charge.size() << " " << map_2D_vt_charge.size() << " " << map_2D_wt_charge.size() << std::endl;
    
    trajectory_fit(pts, map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set,
		   map_2DU_3D_set, map_2DV_3D_set, map_2DW_3D_set,
		   map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge, 2, 0.6*units::cm);
    if (flag_timing) std::cout << "do_single_tracking timing: 2nd trajectory_fit took " << std::chrono::duration_cast<DSms>(DSClock::now()-ds_t0).count() << " ms" << std::endl;
    
    //  std::cout << pts.back() << std::endl;
    
    // std::cout << pts.size() << std::endl;
    // for (size_t i = 0; i < pts.size(); ++i) {
    //   std::cout << "pts[" << i << "] = ("
    //             << pts[i].x << ", "
    //             << pts[i].y << ", "
    //             << pts[i].z << ")" << std::endl;
    // }
    
    // examine trajectory ... // no angle at the moment ...
    //std::cout << pts.size() << std::endl;
    ds_t0 = DSClock::now();
    organize_ps_path(ct_point_cloud, pts, low_dis_limit, 0);
    if (flag_timing) std::cout << "do_single_tracking timing: organize path " << std::chrono::duration_cast<DSms>(DSClock::now()-ds_t0).count() << " ms" << std::endl;
    //std::cout << pts.size() << std::endl;
    // std::cout << "dQ/dx fit " << pts.size() << std::endl;
  }

 
  // for (size_t i=0;i+1!=pts.size();i++){
  //   std::cout << i << " " << pts.at(i) << " " << sqrt(pow(pts.at(i+1).x-pts.at(i).x,2)+pow(pts.at(i+1).y - pts.at(i).y,2)+pow(pts.at(i+1).z-pts.at(i).z,2))<< std::endl;
  // }
   if (pts.size()==2){
    if (sqrt(pow(pts.front().x - pts.back().x,2) + pow(pts.front().y - pts.back().y,2) + pow(pts.front().z - pts.back().z,2)) < 0.1*units::cm){
      pts.clear();
      Point p1(path_wcps.front().x, path_wcps.front().y, path_wcps.front().z);
      pts.push_back(p1);
      Point p2(path_wcps.back().x, path_wcps.back().y, path_wcps.back().z);
      pts.push_back(p2);
    }
  }
   //  std::cout << pts.size() << " " << pts.front() << " " << pts.back() << std::endl;
  
  double ds_dt_organize_data = 0, ds_dt_dqdx = 0, ds_dt_fill = 0;

  if (flag_dQ_dx){
    ds_t0 = DSClock::now();
    fine_tracking_path = pts;
    ds_dt_organize_data = std::chrono::duration_cast<DSms>(DSClock::now()-ds_t0).count();
    
    // std::cout << fine_tracking_path.size() << std::endl;
    // for (size_t i = 0; i < fine_tracking_path.size(); ++i) {
    //   std::cout << "fine_tracking_path[" << i << "] = ("
    //         << fine_tracking_path[i].x << ", "
    //         << fine_tracking_path[i].y << ", "
    //         << fine_tracking_path[i].z << ")" << std::endl;
    // }

    // std::cout << pts.size() << std::endl;
    // for (size_t i=0;i+1!=pts.size();i++){
    //  std::cout << i << " " << pts.at(i) << " " << sqrt(pow(pts.at(i+1).x-pts.at(i).x,2)+pow(pts.at(i+1).y - pts.at(i).y,2)+pow(pts.at(i+1).z-pts.at(i).z,2))<< std::endl;
    // }
    
    //    std::cout << map_2D_ut_charge.size() << " " << map_2D_vt_charge.size() << " " << map_2D_wt_charge.size() << " " << pts.size() << std::endl;
    
    // first round of dQ/dx fit ...
    ds_t0 = DSClock::now();
    dQ_dx_fit(global_wc_map, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge, time, end_point_limit, flag_dQ_dx_fit_reg);
    ds_dt_dqdx = std::chrono::duration_cast<DSms>(DSClock::now()-ds_t0).count();
    
    // std::vector<int> indices;
    // indices.push_back(86);
    // fill_data_map_trajectory(indices, map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set,  map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge);
  }else{
    // fill the data ...
    ds_t0 = DSClock::now();
    dQ_dx_fill(end_point_limit);
    ds_dt_fill = std::chrono::duration_cast<DSms>(DSClock::now()-ds_t0).count();
  }

  if (flag_timing){
    std::cout << "do_single_tracking timing: organize data " << ds_dt_organize_data << " ms" << std::endl;
    std::cout << "do_single_tracking timing: dQ_dx fit/fill took " << ds_dt_dqdx << " ms" << std::endl;
    std::cout << "do_single_tracking timing: fill data " << ds_dt_fill << " ms" << std::endl;
  }
}

void WCPPID::PR3DCluster::calc_num_components(){
  std::cout << cluster_id << " " << graph << " " << graph_steiner << std::endl;
  if (graph==0 || graph_steiner==0) return;
  {
    std::vector<int> component(num_vertices(*graph)); 
    const int num = connected_components(*graph,&component[0]); 
    std::cout << cluster_id << " " << num << " ";
  }
  {
    std::vector<int> component(num_vertices(*graph_steiner)); 
    const int num = connected_components(*graph_steiner,&component[0]); 
    std::cout << num << " " << std::endl;
  }
}


WCPPID::PR3DCluster::PR3DCluster(int cluster_id)
  : cluster_id(cluster_id)
  , flag_PCA(false)
{
  point_cloud = 0;
  graph = 0;
  point_cloud_steiner = 0;
  graph_steiner = 0;
  point_cloud_steiner_terminal = 0;
  source_wcp_index = -1;
  // flag_fine_tracking = false;
}

void WCPPID::PR3DCluster::clear_fit_tracks(){
  for (auto it = fit_tracks.begin(); it!=fit_tracks.end(); it++){
    delete *it;
  }
  fit_tracks.clear();
}

WCPPID::PR3DCluster::~PR3DCluster(){
  if (point_cloud!=(ToyPointCloud*)0)
    delete point_cloud;
  if (graph!=(WCPPID::MCUGraph*)0)
    delete graph;

  if (point_cloud_steiner != (ToyPointCloud*)0)
    delete point_cloud_steiner;
  if (point_cloud_steiner_terminal !=(ToyPointCloud*)0)
    delete point_cloud_steiner_terminal;
  if (graph_steiner!=(WCPPID::MCUGraph*)0)
    delete graph_steiner;

  clear_fit_tracks();
}




void WCPPID::PR3DCluster::AddCell(SlimMergeGeomCell* mcell, int time_slice){
  if (cell_times_set_map.find(mcell)==cell_times_set_map.end()){
    std::set<int> times;
    times.insert(time_slice);
    cell_times_set_map[mcell]=times;
    mcells.push_back(mcell);
  }else{
    std::set<int>& times = cell_times_set_map[mcell];
    //if (find(times.begin(),times.end(),time_slice)==times.end()){
    times.insert(time_slice);
    // }
  }
  
  if (time_cells_set_map.find(time_slice)==time_cells_set_map.end()){
    SMGCSet mcells_1;
    mcells_1.insert(mcell);
    time_cells_set_map[time_slice] = mcells_1;
  }else{
    SMGCSet& mcells_1 = time_cells_set_map[time_slice];
    //if (find(mcells_1.begin(),mcells_1.end(), mcell) == mcells_1.end()){
    mcells_1.insert(mcell);
    //}
  }
}

void WCPPID::PR3DCluster::Del_point_cloud(){
  if (point_cloud!=(ToyPointCloud*)0)
    delete point_cloud;
  point_cloud = 0;
}

void WCPPID::PR3DCluster::Create_point_cloud(WCP::ToyPointCloud *global_point_cloud){
  if (point_cloud!=(ToyPointCloud*)0)
    return;

  TPCParams& mp = Singleton<TPCParams>::Instance();
  double angle_u = mp.get_angle_u();
  double angle_v = mp.get_angle_v();
  double angle_w = mp.get_angle_w();
  
  
  point_cloud = new ToyPointCloud(angle_u, angle_v, angle_w);
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    PointVector pts = mcell->get_sampling_points();

    if (global_point_cloud!=(ToyPointCloud*)0)
      global_point_cloud->AddPoints(pts,mcell->get_sampling_points_wires(),mcell);  
    
    point_cloud->AddPoints(pts,mcell->get_sampling_points_wires(),mcell);
  }
  point_cloud->build_kdtree_index();
  //  std::cout << point_cloud->get_num_points() << std::endl;

}

Point WCPPID::PR3DCluster::calc_ave_pos(Point&p, int N){
  std::map<WCP::SlimMergeGeomCell*, Point> pts = point_cloud->get_closest_mcell(p,N);
  Point pt(0,0,0);
  double charge = 0;
  //std::cout << pts.size() << std::endl;
  for (auto it = pts.begin(); it!= pts.end(); it++){
    SlimMergeGeomCell *mcell = (*it).first;
    Point pc = mcell->center();
    double q = mcell->get_q();
    pt.x += pc.x * q;
    pt.y += pc.y * q;
    pt.z += pc.z * q;
    charge += q;
    // std::cout << pc.x/units::cm << " " << pc.y/units::cm << " " << pc.z/units::cm << " " << q << " " <<
    //  sqrt(pow(pc.x-p.x,2)+pow(pc.y-p.y,2)+pow(pc.z-p.z,2))/units::cm << std::endl;
  }
  if (charge!=0){
    pt.x/=charge;
    pt.y/=charge;
    pt.z/=charge;
  }
  return pt;
}

Point WCPPID::PR3DCluster::calc_ave_pos(Point& p, double dis){
  std::map<WCP::SlimMergeGeomCell*, Point> pts = point_cloud->get_closest_mcell(p,dis);
  Point pt(0,0,0);
  double charge = 0;
  //std::cout << pts.size() << std::endl;
  for (auto it = pts.begin(); it!= pts.end(); it++){
    SlimMergeGeomCell *mcell = (*it).first;
    Point pc = mcell->center();
    double q = mcell->get_q();
    pt.x += pc.x * q;
    pt.y += pc.y * q;
    pt.z += pc.z * q;
    charge += q;
    // std::cout << pc.x/units::cm << " " << pc.y/units::cm << " " << pc.z/units::cm << " " << q << " " <<
    //  sqrt(pow(pc.x-p.x,2)+pow(pc.y-p.y,2)+pow(pc.z-p.z,2))/units::cm << std::endl;
  }
  if (charge!=0){
    pt.x/=charge;
    pt.y/=charge;
    pt.z/=charge;
  }
  return pt;
    
}

int WCPPID::PR3DCluster::get_num_points(Point& p_test, double dis){
  return point_cloud->get_closest_points(p_test, dis).size();
}


std::vector<float> WCPPID::PR3DCluster::get_time_ch_range(){
  double min_ts=1e9, max_ts=-1e9;
  double min_uch=1e9, max_uch=-1e9;
  double min_vch=1e9, max_vch=-1e9;
  double min_wch=1e9, max_wch=-1e9;
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    int time_slice = mcell->GetTimeSlice();
    GeomWireSelection& uwires = mcell->get_uwires();
    GeomWireSelection& vwires = mcell->get_vwires();
    GeomWireSelection& wwires = mcell->get_wwires();
    if (time_slice > max_ts) max_ts = time_slice;
    if (time_slice < min_ts) min_ts = time_slice;
    if (uwires.back()->channel() > max_uch) max_uch = uwires.back()->channel();
    if (uwires.front()->channel() < min_uch) min_uch = uwires.front()->channel();
    if (vwires.back()->channel() > max_vch) max_vch = vwires.back()->channel();
    if (vwires.front()->channel() < min_vch) min_vch = vwires.front()->channel();
    if (wwires.back()->channel() > max_wch) max_wch = wwires.back()->channel();
    if (wwires.front()->channel() < min_wch) min_wch = wwires.front()->channel();
  }
  std::vector<float> results(8,0);
  results.at(0) = min_ts;
  results.at(1) = max_ts;
  results.at(2) = min_uch;
  results.at(3) = max_uch;
  results.at(4) = min_vch;
  results.at(5) = max_vch;
  results.at(6) = min_wch;
  results.at(7) = max_wch;
  return results;
}

void WCPPID::PR3DCluster::fill_2d_charge_dead_chs(std::map<std::pair<int,int>, std::pair<double,double> >& charge_2d_u, std::map<std::pair<int,int>, std::pair<double,double> >& charge_2d_v, std::map<std::pair<int,int>, std::pair<double,double> >& charge_2d_w){
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = *it;
    int time_slice = mcell->GetTimeSlice();
    GeomWireSelection& uwires = mcell->get_uwires();
    GeomWireSelection& vwires = mcell->get_vwires();
    GeomWireSelection& wwires = mcell->get_wwires();
    
    std::vector<WirePlaneType_t> bad_planes = mcell->get_bad_planes();
    //    if (bad_planes.size() >0) std::cout << bad_planes.size() << std::endl;
    // U plane
    if (find(bad_planes.begin(),bad_planes.end(),WirePlaneType_t(0))!=bad_planes.end()){
      double charge = mcell->get_q()*1.0/uwires.size();
      double charge_err = sqrt(pow(charge*0.1,2)+pow(600,2)); // assume 30% error
      for (auto it1 = uwires.begin(); it1!=uwires.end(); it1++){
	auto it2 = charge_2d_u.find(std::make_pair(time_slice, (*it1)->channel()));
	if (it2 != charge_2d_u.end()){
	  it2->second.first += charge;
	  it2->second.second = sqrt(pow(charge_err,2) + pow(it2->second.second,2));
	}else{
	  charge_2d_u[std::make_pair(time_slice, (*it1)->channel())] = std::make_pair(charge, charge_err);
	}
      }
    }

    if (find(bad_planes.begin(),bad_planes.end(),WirePlaneType_t(1))!=bad_planes.end()){
      double charge = mcell->get_q()*1.0/vwires.size();
      double charge_err = sqrt(pow(charge*0.1,2)+pow(600,2)); // assume 30% error
      for (auto it1 = vwires.begin(); it1!=vwires.end(); it1++){
	auto it2 = charge_2d_v.find(std::make_pair(time_slice, (*it1)->channel()));
	if (it2 != charge_2d_v.end()){
	  it2->second.first += charge;
	  it2->second.second = sqrt(pow(charge_err,2) + pow(it2->second.second,2));
	}else{
	  charge_2d_v[std::make_pair(time_slice, (*it1)->channel())] = std::make_pair(charge, charge_err);
	}
      }
    }

    if (find(bad_planes.begin(),bad_planes.end(),WirePlaneType_t(2))!=bad_planes.end()){
      double charge = mcell->get_q()*1.0/wwires.size();
      double charge_err = sqrt(pow(charge*0.1,2)+pow(600,2)); // assume 30% error
      for (auto it1 = wwires.begin(); it1!=wwires.end(); it1++){
	auto it2 = charge_2d_w.find(std::make_pair(time_slice, (*it1)->channel()));
	if (it2 != charge_2d_w.end()){
	  it2->second.first += charge;
	  it2->second.second = sqrt(pow(charge_err,2) + pow(it2->second.second,2));
	}else{
	  charge_2d_w[std::make_pair(time_slice, (*it1)->channel())] = std::make_pair(charge, charge_err);
	}
      }
    }
    
    
  }
  
}


void WCPPID::PR3DCluster::Calc_PCA(){
  if (flag_PCA) return;
  flag_PCA = true;
  
  center.x=0; center.y=0; center.z=0;
  int nsum = 0;
  for (auto it = mcells.begin(); it!=mcells.end();it++){
    PointVector ps = (*it)->get_sampling_points();
    for (int k=0;k!=ps.size();k++){
      center.x += ps.at(k).x;
      center.y += ps.at(k).y;
      center.z += ps.at(k).z;
      nsum ++;
    }
  }

  for (int i=0;i!=3;i++){
    PCA_axis[i].x = 0;
    PCA_axis[i].y = 0;
    PCA_axis[i].z = 0;
  }
  
  if (nsum>=3){
    center.x /=nsum;
    center.y /=nsum;
    center.z /=nsum;
  }else{
    return;
  }
  TMatrixD cov_matrix(3,3);

  for (int i=0;i!=3;i++){
    for (int j=i;j!=3;j++){
      cov_matrix(i,j)=0;
      for (auto it = mcells.begin(); it!=mcells.end();it++){
	PointVector ps = (*it)->get_sampling_points();
	for (int k=0;k!=ps.size();k++){
	  if (i==0 && j==0){
	    cov_matrix(i,j) += (ps.at(k).x - center.x) * (ps.at(k).x - center.x);
	  }else if (i==0 && j==1){
	    cov_matrix(i,j) += (ps.at(k).x - center.x) * (ps.at(k).y - center.y);
	  }else if (i==0 && j==2){
	    cov_matrix(i,j) += (ps.at(k).x - center.x) * (ps.at(k).z - center.z);
	  }else if (i==1 && j==1){
	    cov_matrix(i,j) += (ps.at(k).y - center.y) * (ps.at(k).y - center.y);
	  }else if (i==1 && j==2){
	    cov_matrix(i,j) += (ps.at(k).y - center.y) * (ps.at(k).z - center.z);
	  }else if (i==2 && j==2){
	    cov_matrix(i,j) += (ps.at(k).z - center.z) * (ps.at(k).z - center.z);
	  }
	}
      }
    }
  }
  cov_matrix(1,0) = cov_matrix(0,1);
  cov_matrix(2,0) = cov_matrix(0,2);
  cov_matrix(2,1) = cov_matrix(1,2);
  
  TMatrixDEigen eigen(cov_matrix);
  TMatrixD eigen_values = eigen.GetEigenValues();
  TMatrixD eigen_vectors = eigen.GetEigenVectors();

  PCA_values[0] = eigen_values(0,0) ;
  PCA_values[1] = eigen_values(1,1) ;
  PCA_values[2] = eigen_values(2,2) ;
  
  // std::cout << eigen_values(0,0) << " " << eigen_values(1,1) << " " << eigen_values(2,2) << std::endl;
  //std::cout << eigen_vectors(0,0) << " " << eigen_vectors(0,1) << " " << eigen_vectors(0,2) << std::endl;
  for (int i=0;i!=3;i++){
    PCA_axis[i].x = eigen_vectors(0,i)/sqrt(eigen_vectors(0,i)*eigen_vectors(0,i) + eigen_vectors(1,i)*eigen_vectors(1,i) + eigen_vectors(2,i)*eigen_vectors(2,i));
    PCA_axis[i].y = eigen_vectors(1,i)/sqrt(eigen_vectors(0,i)*eigen_vectors(0,i) + eigen_vectors(1,i)*eigen_vectors(1,i) + eigen_vectors(2,i)*eigen_vectors(2,i));
    PCA_axis[i].z = eigen_vectors(2,i)/sqrt(eigen_vectors(0,i)*eigen_vectors(0,i) + eigen_vectors(1,i)*eigen_vectors(1,i) + eigen_vectors(2,i)*eigen_vectors(2,i));
  }
  
  
  //std::cout << mean_x << " " << mean_y << " " << mean_z << std::endl;
}


void WCPPID::PR3DCluster::Calc_PCA(PointVector& points){
  
  center.x=0; center.y=0; center.z=0;
  int nsum = 0;
  for (auto it = mcells.begin(); it!=mcells.end();it++){
    for (int k=0;k!=points.size();k++){
      center.x += points.at(k).x;
      center.y += points.at(k).y;
      center.z += points.at(k).z;
      nsum ++;
    }
  }

  for (int i=0;i!=3;i++){
    PCA_axis[i].x = 0;
    PCA_axis[i].y = 0;
    PCA_axis[i].z = 0;
  }
  
  if (nsum>=3){
    center.x /=nsum;
    center.y /=nsum;
    center.z /=nsum;
  }else{
    return;
  }
  TMatrixD cov_matrix(3,3);

  for (int i=0;i!=3;i++){
    for (int j=i;j!=3;j++){
      cov_matrix(i,j)=0;
      for (auto it = mcells.begin(); it!=mcells.end();it++){
	PointVector& ps = points;
	for (int k=0;k!=ps.size();k++){
	  if (i==0 && j==0){
	    cov_matrix(i,j) += (ps.at(k).x - center.x) * (ps.at(k).x - center.x);
	  }else if (i==0 && j==1){
	    cov_matrix(i,j) += (ps.at(k).x - center.x) * (ps.at(k).y - center.y);
	  }else if (i==0 && j==2){
	    cov_matrix(i,j) += (ps.at(k).x - center.x) * (ps.at(k).z - center.z);
	  }else if (i==1 && j==1){
	    cov_matrix(i,j) += (ps.at(k).y - center.y) * (ps.at(k).y - center.y);
	  }else if (i==1 && j==2){
	    cov_matrix(i,j) += (ps.at(k).y - center.y) * (ps.at(k).z - center.z);
	  }else if (i==2 && j==2){
	    cov_matrix(i,j) += (ps.at(k).z - center.z) * (ps.at(k).z - center.z);
	  }
	}
      }
    }
  }
  cov_matrix(1,0) = cov_matrix(0,1);
  cov_matrix(2,0) = cov_matrix(0,2);
  cov_matrix(2,1) = cov_matrix(1,2);
  
  TMatrixDEigen eigen(cov_matrix);
  TMatrixD eigen_values = eigen.GetEigenValues();
  TMatrixD eigen_vectors = eigen.GetEigenVectors();

  PCA_values[0] = eigen_values(0,0) ;
  PCA_values[1] = eigen_values(1,1) ;
  PCA_values[2] = eigen_values(2,2) ;
  
  // std::cout << eigen_values(0,0) << " " << eigen_values(1,1) << " " << eigen_values(2,2) << std::endl;
  //std::cout << eigen_vectors(0,0) << " " << eigen_vectors(0,1) << " " << eigen_vectors(0,2) << std::endl;
  for (int i=0;i!=3;i++){
    PCA_axis[i].x = eigen_vectors(0,i)/sqrt(eigen_vectors(0,i)*eigen_vectors(0,i) + eigen_vectors(1,i)*eigen_vectors(1,i) + eigen_vectors(2,i)*eigen_vectors(2,i));
    PCA_axis[i].y = eigen_vectors(1,i)/sqrt(eigen_vectors(0,i)*eigen_vectors(0,i) + eigen_vectors(1,i)*eigen_vectors(1,i) + eigen_vectors(2,i)*eigen_vectors(2,i));
    PCA_axis[i].z = eigen_vectors(2,i)/sqrt(eigen_vectors(0,i)*eigen_vectors(0,i) + eigen_vectors(1,i)*eigen_vectors(1,i) + eigen_vectors(2,i)*eigen_vectors(2,i));

  }
}

TVector3 WCPPID::PR3DCluster::calc_PCA_dir(Point&p, PointVector& ps){
  Point center1 = p;
  
  TMatrixD cov_matrix(3,3);
  for (int i=0;i!=3;i++){
    for (int j=i;j!=3;j++){
      cov_matrix(i,j)=0;
      
      for (int k=0;k!=ps.size();k++){
	if (i==0 && j==0){
	  cov_matrix(i,j) += (ps.at(k).x - center1.x) * (ps.at(k).x - center1.x);//*q*q/ps.size()/ps.size();
	}else if (i==0 && j==1){
	  cov_matrix(i,j) += (ps.at(k).x - center1.x) * (ps.at(k).y - center1.y);//*q*q/ps.size()/ps.size();
	}else if (i==0 && j==2){
	  cov_matrix(i,j) += (ps.at(k).x - center1.x) * (ps.at(k).z - center1.z);//*q*q/ps.size()/ps.size();
	}else if (i==1 && j==1){
	  cov_matrix(i,j) += (ps.at(k).y - center1.y) * (ps.at(k).y - center1.y);//*q*q/ps.size()/ps.size();
	}else if (i==1 && j==2){
	  cov_matrix(i,j) += (ps.at(k).y - center1.y) * (ps.at(k).z - center1.z);//*q*q/ps.size()/ps.size();
	}else if (i==2 && j==2){
	  cov_matrix(i,j) += (ps.at(k).z - center1.z) * (ps.at(k).z - center1.z);//*q*q/ps.size()/ps.size();
	}
      }
    }
  }

  cov_matrix(1,0) = cov_matrix(0,1);
  cov_matrix(2,0) = cov_matrix(0,2);
  cov_matrix(2,1) = cov_matrix(1,2);
  
  TMatrixDEigen eigen(cov_matrix);
  TMatrixD eigen_values = eigen.GetEigenValues();
  TMatrixD eigen_vectors = eigen.GetEigenVectors();
  TVector3 dir(eigen_vectors(0,0)/sqrt(eigen_vectors(0,0)*eigen_vectors(0,0) + eigen_vectors(1,0)*eigen_vectors(1,0) + eigen_vectors(2,0)*eigen_vectors(2,0)),
	       eigen_vectors(1,0)/sqrt(eigen_vectors(0,0)*eigen_vectors(0,0) + eigen_vectors(1,0)*eigen_vectors(1,0) + eigen_vectors(2,0)*eigen_vectors(2,0)),
	       eigen_vectors(2,0)/sqrt(eigen_vectors(0,0)*eigen_vectors(0,0) + eigen_vectors(1,0)*eigen_vectors(1,0) + eigen_vectors(2,0)*eigen_vectors(2,0)));
  return dir;
}

TVector3 WCPPID::PR3DCluster::calc_PCA_dir(Point& p, double dis){
  std::map<WCP::SlimMergeGeomCell*, Point> pts = point_cloud->get_closest_mcell(p,dis);
  Point center1(0,0,0);
  // double charge=0;
  // for (auto it = pts.begin(); it!= pts.end(); it++){
  //   SlimMergeGeomCell *mcell = (*it).first;
  //   Point point = mcell->center();
  //   double q = mcell->get_q();
  //   center1.x += point.x * q;
  //   center1.y += point.y * q;
  //   center1.z += point.z * q;
  //   charge+=q;
  // }
  // center1.x/=charge;
  // center1.y/=charge;
  // center1.z/=charge;
  center1.x = p.x;
  center1.y = p.y;
  center1.z = p.z;

  TMatrixD cov_matrix(3,3);
  for (int i=0;i!=3;i++){
    for (int j=i;j!=3;j++){
      cov_matrix(i,j)=0;
      for (auto it = pts.begin(); it!= pts.end(); it++){
	SlimMergeGeomCell *mcell = (*it).first;
	double q = mcell->get_q();
	PointVector ps = mcell->get_sampling_points();
	for (int k=0;k!=ps.size();k++){
	  if (i==0 && j==0){
	    cov_matrix(i,j) += (ps.at(k).x - center1.x) * (ps.at(k).x - center1.x);//*q*q/ps.size()/ps.size();
	  }else if (i==0 && j==1){
	    cov_matrix(i,j) += (ps.at(k).x - center1.x) * (ps.at(k).y - center1.y);//*q*q/ps.size()/ps.size();
	  }else if (i==0 && j==2){
	    cov_matrix(i,j) += (ps.at(k).x - center1.x) * (ps.at(k).z - center1.z);//*q*q/ps.size()/ps.size();
	  }else if (i==1 && j==1){
	    cov_matrix(i,j) += (ps.at(k).y - center1.y) * (ps.at(k).y - center1.y);//*q*q/ps.size()/ps.size();
	  }else if (i==1 && j==2){
	    cov_matrix(i,j) += (ps.at(k).y - center1.y) * (ps.at(k).z - center1.z);//*q*q/ps.size()/ps.size();
	  }else if (i==2 && j==2){
	    cov_matrix(i,j) += (ps.at(k).z - center1.z) * (ps.at(k).z - center1.z);//*q*q/ps.size()/ps.size();
	  }
	}
      }
    }
  }

  cov_matrix(1,0) = cov_matrix(0,1);
  cov_matrix(2,0) = cov_matrix(0,2);
  cov_matrix(2,1) = cov_matrix(1,2);
  
  TMatrixDEigen eigen(cov_matrix);
  TMatrixD eigen_values = eigen.GetEigenValues();
  TMatrixD eigen_vectors = eigen.GetEigenVectors();
  TVector3 dir(eigen_vectors(0,0)/sqrt(eigen_vectors(0,0)*eigen_vectors(0,0) + eigen_vectors(1,0)*eigen_vectors(1,0) + eigen_vectors(2,0)*eigen_vectors(2,0)),
	       eigen_vectors(1,0)/sqrt(eigen_vectors(0,0)*eigen_vectors(0,0) + eigen_vectors(1,0)*eigen_vectors(1,0) + eigen_vectors(2,0)*eigen_vectors(2,0)),
	       eigen_vectors(2,0)/sqrt(eigen_vectors(0,0)*eigen_vectors(0,0) + eigen_vectors(1,0)*eigen_vectors(1,0) + eigen_vectors(2,0)*eigen_vectors(2,0)));
  return dir;
  
}


//Hough Transformation 
TVector3 WCPPID::PR3DCluster::VHoughTrans(Point&p, double dis, ToyPointCloud *point_cloud1, bool flag_print){
  double theta, phi;
  std::pair<double,double> angles_1 = HoughTrans(p,dis, point_cloud1, flag_print);
  theta = angles_1.first;
  phi = angles_1.second;
  TVector3 temp(sin(theta)*cos(phi),sin(theta)*sin(phi),cos(theta));
  return temp;
}

std::pair<double,double> WCPPID::PR3DCluster::HoughTrans(Point&p , double dis, ToyPointCloud *point_cloud1, bool flag_print){
  double theta, phi;
  TH2F *hough = new TH2F("","",180,0.,3.1415926,360,-3.1415926,3.1415926);
  double x0 = p.x;
  double y0 = p.y;
  double z0 = p.z;
  
  std::vector<std::pair<WCP::SlimMergeGeomCell*,Point>>pts = point_cloud1->get_closest_points(p,dis);

  
    
  double x,y,z,q;
  for (size_t i=0; i!=pts.size(); i++){
    x = pts.at(i).second.x;
    y = pts.at(i).second.y;
    z = pts.at(i).second.z;
    q = pts.at(i).first->get_q()/pts.at(i).first->get_sampling_points().size();

    // if (flag_print)
    //   std::cout << i << " " << pts.at(i).first->get_q() << " " << pts.at(i).first->get_sampling_points().size() << std::endl;

    if (q<=0) continue;

    //  for (int i1=0; i1!=5; i1++){
    //  for (int j1=0; j1!=5; j1++){
    //	for (int k1=0; k1!=5; k1++){
    TVector3 vec(x-x0 ,y-y0 ,z-z0 );
    hough->Fill(vec.Theta(),vec.Phi(), q );
    
  }
  int maxbin = hough->GetMaximumBin();
  int a,b,c;
  hough->GetBinXYZ(maxbin,a,b,c);

  if (flag_print)
    std::cout << "Num " <<  pts.size() << " " << maxbin << " " << a << " " << b << " " << c << " " << hough->GetSum() << std::endl;
  
  theta = hough->GetXaxis()->GetBinCenter(a);
  phi = hough->GetYaxis()->GetBinCenter(b);

  // std::cout << hough->GetSum() << " " << hough->GetBinContent(a,b)<< std::endl;
  
  delete hough;
  return std::make_pair(theta,phi);
}

TVector3 WCPPID::PR3DCluster::VHoughTrans(Point&p, double dis){
  double theta, phi;
  std::pair<double,double> angles_1 = HoughTrans(p,dis);
  theta = angles_1.first;
  phi = angles_1.second;
  TVector3 temp(sin(theta)*cos(phi),sin(theta)*sin(phi),cos(theta));
  return temp;
}

std::pair<double,double> WCPPID::PR3DCluster::HoughTrans(Point&p , double dis){
  double theta, phi;
  TH2F *hough = new TH2F("","",180,0.,3.1415926,360,-3.1415926,3.1415926);
  double x0 = p.x;
  double y0 = p.y;
  double z0 = p.z;
  
  std::vector<std::pair<WCP::SlimMergeGeomCell*,Point>>pts = point_cloud->get_closest_points(p,dis);

  // std::cout << "Num " <<  pts.size() << std::endl;
    
  double x,y,z,q;
  for (size_t i=0; i!=pts.size(); i++){
    x = pts.at(i).second.x;
    y = pts.at(i).second.y;
    z = pts.at(i).second.z;
    q = pts.at(i).first->get_q()/pts.at(i).first->get_sampling_points().size();
    if (q<=0) continue;

    //  for (int i1=0; i1!=5; i1++){
    //  for (int j1=0; j1!=5; j1++){
    //	for (int k1=0; k1!=5; k1++){
    TVector3 vec(x-x0 ,y-y0 ,z-z0 );
    hough->Fill(vec.Theta(),vec.Phi(), q );
  }
  int maxbin = hough->GetMaximumBin();
  int a,b,c;
  hough->GetBinXYZ(maxbin,a,b,c);
  theta = hough->GetXaxis()->GetBinCenter(a);
  phi = hough->GetYaxis()->GetBinCenter(b);

  // std::cout << hough->GetSum() << " " << hough->GetBinContent(a,b)<< std::endl;
  
  delete hough;
  return std::make_pair(theta,phi);
}



void WCPPID::PR3DCluster::get_projection(std::vector<int>& proj_channel, std::vector<int>& proj_timeslice, std::vector<int>& proj_charge, std::vector<int>& proj_charge_err, std::vector<int>& proj_flag, std::map<int,std::map<const GeomWire*, SMGCSelection > >& global_wc_map){
  // std::vector<int> proj_charge_err;
  
  std::set<SlimMergeGeomCell*> cluster_mcells_set;
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = *it;
    cluster_mcells_set.insert(mcell);
  }

  std::set<std::pair<int,int>> saved_time_channel;
   
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = *it;
    int time_slice = mcell->GetTimeSlice();

    std::map<const GeomWire*, SMGCSelection >& timeslice_wc_map = global_wc_map[time_slice];

    GeomWireSelection& uwires = mcell->get_uwires();
    GeomWireSelection& vwires = mcell->get_vwires();
    GeomWireSelection& wwires = mcell->get_wwires();

    bool flag_reg_save = true;
    bool flag_bad_plane = false;
    
    std::vector<WirePlaneType_t> bad_planes = mcell->get_bad_planes();
    if (find(bad_planes.begin(),bad_planes.end(),WirePlaneType_t(0))==bad_planes.end()){
      int num_shared_wires = 0;
      for (int i=0;i!=uwires.size();i++){
        const GeomWire *wire = uwires.at(i);
        
        if (timeslice_wc_map[wire].size()>1){
          for (auto it1 = timeslice_wc_map[wire].begin(); it1!=timeslice_wc_map[wire].end(); it1++){
            SlimMergeGeomCell *mcell1 = *it1;
            if (cluster_mcells_set.find(mcell1)==cluster_mcells_set.end())
              num_shared_wires ++;
          }
          //
        }
      }
      if (num_shared_wires >0.15*uwires.size()&&num_shared_wires>1) flag_reg_save = false;
    }else{
      flag_reg_save = false;
      flag_bad_plane = true;
    }

    if (flag_reg_save){
      for (int i=0;i!=uwires.size();i++){
        const GeomWire *wire = uwires.at(i);
        int ch = wire->channel();
        // regular cases ... 
        int charge = mcell->Get_Wire_Charge(wire);
        int charge_err = mcell->Get_Wire_Charge_Err(wire);
        
        if (saved_time_channel.find(std::make_pair(time_slice,ch))==saved_time_channel.end()){
          proj_channel.push_back(ch);
          proj_timeslice.push_back(time_slice);
          proj_charge.push_back(charge);
          proj_charge_err.push_back(charge_err);
          proj_flag.push_back(1);
          saved_time_channel.insert(std::make_pair(time_slice,ch));
        }
      }
    }else{
      for (int i=0;i!=uwires.size();i++){
        const GeomWire *wire = uwires.at(i);
        int ch = wire->channel();
        int temp_flag = 1;
        int charge = mcell->Get_Wire_Charge(wire);
        int charge_err = mcell->Get_Wire_Charge_Err(wire);
        
        if (charge<=0 && flag_bad_plane){
          charge = mcell->get_q()*1.0/uwires.size();
          charge_err = sqrt(pow(charge*0.1,2)+pow(600,2)); // assume 30% error
          temp_flag = 0;
        }

        if (charge <=0) {
          charge = 0;
          charge_err = 1000;
        }
	
        //	if(cluster_id==18)
        //std::cout << ch << " " << time_slice << " " << charge << std::endl;
        //	if (saved_time_channel.find(std::make_pair(time_slice,ch))==saved_time_channel.end()){
        proj_channel.push_back(ch);
        proj_timeslice.push_back(time_slice);
        proj_charge.push_back(charge);
        proj_charge_err.push_back(charge_err);
        proj_flag.push_back(temp_flag);
        // saved_time_channel.insert(std::make_pair(time_slice,ch));
	      //}
      }
    }
    
    flag_reg_save = true;
    flag_bad_plane = false;
    if (find(bad_planes.begin(),bad_planes.end(),WirePlaneType_t(1))==bad_planes.end()){
      int num_shared_wires = 0;
      for (int i=0;i!=vwires.size();i++){
	const GeomWire *wire = vwires.at(i);

	for (auto it1 = timeslice_wc_map[wire].begin(); it1!=timeslice_wc_map[wire].end(); it1++){
	  SlimMergeGeomCell *mcell1 = *it1;
	  if (cluster_mcells_set.find(mcell1)==cluster_mcells_set.end())
	    num_shared_wires ++;
	}
	
	//	if (timeslice_wc_map[wire].size()>1)
	//  num_shared_wires ++;
      }
      if (num_shared_wires >0.15*vwires.size()&&num_shared_wires>1)
	flag_reg_save = false;
    }else{
      flag_reg_save = false;
      flag_bad_plane = true;
    }

    
    if (flag_reg_save){
      for (int i=0;i!=vwires.size();i++){
	const GeomWire *wire = vwires.at(i);
	int ch = wire->channel();
	int charge = mcell->Get_Wire_Charge(wire);
	int charge_err = mcell->Get_Wire_Charge_Err(wire);
	
	if (saved_time_channel.find(std::make_pair(time_slice,ch))==saved_time_channel.end()){
	  proj_channel.push_back(ch);
	  proj_timeslice.push_back(time_slice);
	  proj_charge.push_back(charge);
	  proj_charge_err.push_back(charge_err);
	  proj_flag.push_back(1);
	  saved_time_channel.insert(std::make_pair(time_slice,ch));
	}
      }
    }else{
      for (int i=0;i!=vwires.size();i++){
	const GeomWire *wire = vwires.at(i);
	int ch = wire->channel();
	int temp_flag=1;
	int charge = mcell->Get_Wire_Charge(wire);
	int charge_err = mcell->Get_Wire_Charge_Err(wire);
	if (charge<=0 && flag_bad_plane){
	  charge = mcell->get_q()*1.0/vwires.size();
	  charge_err = sqrt(pow(charge*0.1,2)+pow(600,2));
	  temp_flag= 0;
	}

	if (charge <=0) {
	  charge = 0;
	  charge_err = 1000;
	}
	
	//if (saved_time_channel.find(std::make_pair(time_slice,ch))==saved_time_channel.end()){
	proj_channel.push_back(ch);
	proj_timeslice.push_back(time_slice);
	proj_charge.push_back(charge);
	proj_charge_err.push_back(charge_err);
	proj_flag.push_back(temp_flag);
	//saved_time_channel.insert(std::make_pair(time_slice,ch));
	  //}
      }
    }
    
    flag_reg_save = true;
    flag_bad_plane = false;
    if (find(bad_planes.begin(),bad_planes.end(),WirePlaneType_t(2))==bad_planes.end()){
      int num_shared_wires = 0;
      for (int i=0;i!=wwires.size();i++){
	const GeomWire *wire = wwires.at(i);

	for (auto it1 = timeslice_wc_map[wire].begin(); it1!=timeslice_wc_map[wire].end(); it1++){
	  SlimMergeGeomCell *mcell1 = *it1;
	  if (cluster_mcells_set.find(mcell1)==cluster_mcells_set.end())
	    num_shared_wires ++;
	}
	
	//	if (timeslice_wc_map[wire].size()>1)
	//num_shared_wires ++;
      }
      if (num_shared_wires >0.15*wwires.size()&&num_shared_wires>1)
	flag_reg_save = false;
    }else{
      flag_reg_save = false;
      flag_bad_plane = true;
    }


    if(flag_reg_save){
      for (int i=0;i!=wwires.size();i++){
	const GeomWire *wire = wwires.at(i);
	int ch = wire->channel();
	int charge = mcell->Get_Wire_Charge(wire);
	int charge_err = mcell->Get_Wire_Charge_Err(wire);
	
	if (saved_time_channel.find(std::make_pair(time_slice,ch))==saved_time_channel.end()){
	  proj_channel.push_back(ch);
	  proj_timeslice.push_back(time_slice);
	  proj_charge.push_back(charge);
	  proj_charge_err.push_back(charge_err);
	  proj_flag.push_back(1);
	  saved_time_channel.insert(std::make_pair(time_slice,ch));
	}
      }
    }else{
      for (int i=0;i!=wwires.size();i++){
	const GeomWire *wire = wwires.at(i);
	int ch = wire->channel();
	int temp_flag=1;
	int charge = mcell->Get_Wire_Charge(wire);
	int charge_err = mcell->Get_Wire_Charge_Err(wire);
	if (charge<=0 && flag_bad_plane){
	  charge = mcell->get_q()*1.0/wwires.size();
	  charge_err = sqrt(pow(charge*0.1,2) + pow(100,2));
	  temp_flag = 0;
	}
	if (charge<=0) {
	  charge = 0;
	  charge_err = 1000;
	}
	//	if (saved_time_channel.find(std::make_pair(time_slice,ch))==saved_time_channel.end()){
	proj_channel.push_back(ch);
	proj_timeslice.push_back(time_slice);
	proj_charge.push_back(charge);
	proj_charge_err.push_back(charge_err);
	proj_flag.push_back(temp_flag);
	  //	  saved_time_channel.insert(std::make_pair(time_slice,ch));
	  //	}
      }
    }
  } // loop over mcells

  for (auto it = collected_charge_map.begin(); it!=collected_charge_map.end(); it++){
    int time_slice = it->first.first;
    int ch = it->first.second;
    int charge = it->second.first;
    int charge_err = it->second.second;
    proj_channel.push_back(ch);
    proj_timeslice.push_back(time_slice);
    proj_charge.push_back(charge);
    proj_charge_err.push_back(charge_err);
    proj_flag.push_back(2); // isolated ones ...
  }
  
}



void WCPPID::PR3DCluster::collect_charge_trajectory(ToyCTPointCloud& ct_point_cloud, double dis_cut, double range_cut){
  //clear up ...
  collected_charge_map.clear();
  
  std::set<std::pair<int,int>> existing_tcs;

  //std::cout << "mcells: " << mcells.size() << std::endl;
  
  // form a set cotaining everything inside the cluster
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = (*it);
    int time_slice = mcell->GetTimeSlice();
    GeomWireSelection& uwires = mcell->get_uwires();
    GeomWireSelection& vwires = mcell->get_vwires();
    GeomWireSelection& wwires = mcell->get_wwires();
    for (auto it1 = uwires.begin(); it1!=uwires.end(); it1++){
      const GeomWire *wire = (*it1);
      existing_tcs.insert(std::make_pair(time_slice,wire->channel()));
    }
    for (auto it1 = vwires.begin(); it1!=vwires.end(); it1++){
      const GeomWire *wire = (*it1);
      existing_tcs.insert(std::make_pair(time_slice,wire->channel()));
    }
    for (auto it1 = wwires.begin(); it1!=wwires.end(); it1++){
      const GeomWire *wire = (*it1);
      existing_tcs.insert(std::make_pair(time_slice,wire->channel()));
    }
  }
  
  // form a trajectory according to dis and fine tracking?
  PointVector traj_pts;
  //  PointVector& pts = get_fine_tracking_path();

  // not using fine tracking results ... 
  PointVector pts;
  std::list<WCPointCloud<double>::WCPoint>& path_wcps = get_path_wcps();
  for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
    Point p((*it).x, (*it).y, (*it).z);
    pts.push_back(p);
  }
  
  //std::cout << "trajectory points " << pts.size() << std::endl;

  // if (cluster_id==13){
  //   for (int i=0; i!=pts.size(); i++){
  //     std::cout << i << " " << pts.at(i).x/units::cm << " " << pts.at(i).y/units::cm << " "<< pts.at(i).z/units::cm << std::endl;
  //   }
  // }
  
  for (int i=0; i!=pts.size(); i++){
    if (pts.at(i).y <-120*units::cm || pts.at(i).y > 120*units::cm ||
	pts.at(i).z < -5*units::cm || pts.at(i).z > 1070*units::cm) continue;
    
    if (traj_pts.size()==0){
      traj_pts.push_back(pts.at(i));
    }else{
      double dis = sqrt(pow(pts.at(i).x-pts.at(i-1).x,2) +
			pow(pts.at(i).y-pts.at(i-1).y,2) +
			pow(pts.at(i).z-pts.at(i-1).z,2));
      if (dis <= dis_cut){
	traj_pts.push_back(pts.at(i));
      }else{
	int nseg = dis / dis_cut + 1;
	//	std::cout << i << " " << pts.at(i).x/units::cm << " " << pts.at(i).y/units::cm << " "<< pts.at(i).z/units::cm << " " << dis << " " << dis_cut << " " << nseg << std::endl;
	for (int j=0; j!=nseg;j++){
	  Point temp_pt;
	  temp_pt.x = pts.at(i-1).x + (pts.at(i).x-pts.at(i-1).x) *(j+1.)/nseg;
	  temp_pt.y = pts.at(i-1).y + (pts.at(i).y-pts.at(i-1).y) *(j+1.)/nseg;
	  temp_pt.z = pts.at(i-1).z + (pts.at(i).z-pts.at(i-1).z) *(j+1.)/nseg;
	  traj_pts.push_back(temp_pt);
	}
      }
    }
  }
  
  // collect the nearby points, and compare with existing maps
  for (size_t i=0;i!=traj_pts.size();i++){
    //std::cout << i << " " << traj_pts.at(i).x/units::cm << " " << traj_pts.at(i).y/units::cm << " " << traj_pts.at(i).z/units::cm << " " << range_cut << std::endl;
    WCP::CTPointCloud<double> nearby_points = ct_point_cloud.get_closest_points(traj_pts.at(i),range_cut,0);
    
    //    std::cout << "0 " << nearby_points.pts.size() << std::endl;
    
    for (size_t j=0;j!=nearby_points.pts.size();j++){
      if (existing_tcs.find(std::make_pair(nearby_points.pts.at(j).time_slice,nearby_points.pts.at(j).channel)) == existing_tcs.end()){
	collected_charge_map[std::make_pair(nearby_points.pts.at(j).time_slice,nearby_points.pts.at(j).channel)] = std::make_pair(nearby_points.pts.at(j).charge,nearby_points.pts.at(j).charge_err);
      }
    }
    nearby_points = ct_point_cloud.get_closest_points(traj_pts.at(i),range_cut,1);
    //std::cout << "1 " << nearby_points.pts.size() << std::endl;
    
    for (size_t j=0;j!=nearby_points.pts.size();j++){
      if (existing_tcs.find(std::make_pair(nearby_points.pts.at(j).time_slice,nearby_points.pts.at(j).channel)) == existing_tcs.end()){
	collected_charge_map[std::make_pair(nearby_points.pts.at(j).time_slice,nearby_points.pts.at(j).channel)] = std::make_pair(nearby_points.pts.at(j).charge,nearby_points.pts.at(j).charge_err);
      }
    }
    nearby_points = ct_point_cloud.get_closest_points(traj_pts.at(i),range_cut,2);
    
    //    std::cout << "2 " << nearby_points.pts.size() << std::endl;
    
    for (size_t j=0;j!=nearby_points.pts.size();j++){
      if (existing_tcs.find(std::make_pair(nearby_points.pts.at(j).time_slice,nearby_points.pts.at(j).channel)) == existing_tcs.end()){
	collected_charge_map[std::make_pair(nearby_points.pts.at(j).time_slice,nearby_points.pts.at(j).channel)] = std::make_pair(nearby_points.pts.at(j).charge,nearby_points.pts.at(j).charge_err);
      }
    }
  }

  // std::cout << collected_charge_map.size() << std::endl;
}
