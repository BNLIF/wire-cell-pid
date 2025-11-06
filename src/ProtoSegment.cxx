#include "WCPPID/ProtoSegment.h"
#include "WCPData/TPCParams.h"
#include "WCPData/Singleton.h"
#include "WCPData/Line.h"

#include "TH1F.h"

using namespace WCP;

int WCPPID::ProtoSegment::get_particle_type(){
  if (get_flag_shower()){
    particle_type = 11; // shower are all treated as electron
  }
  return particle_type;
}

WCPPID::ProtoSegment::ProtoSegment(int id, std::list<WCP::WCPointCloud<double>::WCPoint >& path_wcps, int cluster_id )
  : id(id)
  , cluster_id(cluster_id)
  , flag_fit(false)
  , pcloud_fit(0)
  , pcloud_associated(0)
  , pcloud_associated_steiner(0)
  , flag_shower_trajectory(false)
  , flag_shower_topology(false)
  , flag_avoid_muon_check(false)
  , flag_dir(0)
  , dir_weak(false)
  , particle_type(0)
  , particle_score(100)
  , particle_mass(0)
  , kenergy_charge(0)
  , kenergy_best(0)
{
  for (int i=0;i!=4;i++){
    particle_4mom[i] = 0;
  }
  
  for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
    wcpt_vec.push_back(*it);
    Point p;
    p.x = wcpt_vec.back().x;
    p.y = wcpt_vec.back().y;
    p.z = wcpt_vec.back().z;
    fit_pt_vec.push_back(p);
  }

  if (pcloud_fit != (ToyPointCloud*)0) delete pcloud_fit;
  pcloud_fit = new ToyPointCloud();
  pcloud_fit->AddPoints(fit_pt_vec);
  pcloud_fit->build_kdtree_index();
  
}

void WCPPID::ProtoSegment::build_pcloud_fit(){
  if (pcloud_fit != (ToyPointCloud*)0) delete pcloud_fit;
  pcloud_fit = new ToyPointCloud();
  pcloud_fit->AddPoints(fit_pt_vec);
  pcloud_fit->build_kdtree_index();
}

WCPPID::ProtoSegment::~ProtoSegment(){
  if (pcloud_fit != (ToyPointCloud*)0)
    delete pcloud_fit;
  if (pcloud_associated != (ToyPointCloud*)0)
    delete pcloud_associated;
  if (pcloud_associated_steiner != (ToyPointCloud*)0)
    delete pcloud_associated_steiner;
}


bool WCPPID::ProtoSegment::determine_shower_direction(){
  flag_dir = 0;
  // look at the points ...
  std::vector<PointVector > local_points_vec(fit_pt_vec.size());
  std::vector<std::tuple<double, double, double> > vec_rms_vals(fit_pt_vec.size(), std::make_tuple(0,0,0));
  std::vector<double> vec_dQ_dx(fit_pt_vec.size(), 0);
  std::vector<TVector3> vec_dir(fit_pt_vec.size());
  
  if (pcloud_associated == 0 ) return false;
 
  WCP::WCPointCloud<double>& cloud = pcloud_associated->get_cloud();

  for (size_t i = 0; i!= cloud.pts.size(); i++){
    Point test_p(cloud.pts.at(i).x, cloud.pts.at(i).y, cloud.pts.at(i).z);
    std::vector<std::pair<size_t,double>> results = pcloud_fit->get_closest_index(test_p,1);
    local_points_vec.at(results.front().first).push_back(test_p);
  }

  TVector3 drift_dir(1,0,0); // drift direction
  
  for (size_t i=0;i!=local_points_vec.size();i++){
    TVector3 dir_1, dir_2, dir_3;
    Point v1(0,0,0);
    for (size_t j=1;j!=3;j++){
      if (i+j<local_points_vec.size()){
	v1.x += fit_pt_vec.at(i+j).x - fit_pt_vec.at(i).x;
	v1.y += fit_pt_vec.at(i+j).y - fit_pt_vec.at(i).y;
	v1.z += fit_pt_vec.at(i+j).z - fit_pt_vec.at(i).z;
      }
      if (i>=j){
	v1.x += fit_pt_vec.at(i).x - fit_pt_vec.at(i-j).x;
	v1.y += fit_pt_vec.at(i).y - fit_pt_vec.at(i-j).y;
	v1.z += fit_pt_vec.at(i).z - fit_pt_vec.at(i-j).z;
      }
    }
    dir_1.SetXYZ(v1.x, v1.y, v1.z);
    dir_1 = dir_1.Unit();
    vec_dir.at(i) = dir_1;
    
    
    if (dir_1.Angle(drift_dir)/3.1415926*180. < 7.5){
      dir_1.SetXYZ(1,0,0);
      dir_2.SetXYZ(0,1,0);
      dir_3.SetXYZ(0,0,1);
    }else{
      dir_2 = drift_dir.Cross(dir_1);
      dir_2 = dir_2.Unit();
      dir_3 = dir_1.Cross(dir_2);
    }
    
    std::vector<std::tuple<double, double, double> > vec_projs;
    for (size_t j=0;j!=local_points_vec.at(i).size();j++){
      double proj_1 = dir_1.X() * local_points_vec.at(i).at(j).x + dir_1.Y() * local_points_vec.at(i).at(j).y + dir_1.Z() * local_points_vec.at(i).at(j).z;
      double proj_2 = dir_2.X() * local_points_vec.at(i).at(j).x + dir_2.Y() * local_points_vec.at(i).at(j).y + dir_2.Z() * local_points_vec.at(i).at(j).z;
      double proj_3 = dir_3.X() * local_points_vec.at(i).at(j).x + dir_3.Y() * local_points_vec.at(i).at(j).y + dir_3.Z() * local_points_vec.at(i).at(j).z;
      vec_projs.push_back(std::make_tuple(proj_1, proj_2, proj_3));
    }
    std::tuple<double, double, double> means = std::make_tuple(0,0,0);
    int ncount = local_points_vec.at(i).size();
    if (ncount >1){
      std::get<0>(means) = dir_1.X() * fit_pt_vec.at(i).x + dir_1.Y() * fit_pt_vec.at(i).y + dir_1.Z() * fit_pt_vec.at(i).z;
      std::get<1>(means) = dir_2.X() * fit_pt_vec.at(i).x + dir_2.Y() * fit_pt_vec.at(i).y + dir_2.Z() * fit_pt_vec.at(i).z;
      std::get<2>(means) = dir_3.X() * fit_pt_vec.at(i).x + dir_3.Y() * fit_pt_vec.at(i).y + dir_3.Z() * fit_pt_vec.at(i).z;
      
      for (size_t j=0;j!=local_points_vec.at(i).size();j++){
	std::get<0>(vec_rms_vals.at(i)) += pow(std::get<0>(vec_projs.at(j)) - std::get<0>(means),2);
	std::get<1>(vec_rms_vals.at(i)) += pow(std::get<1>(vec_projs.at(j)) - std::get<1>(means),2);
	std::get<2>(vec_rms_vals.at(i)) += pow(std::get<2>(vec_projs.at(j)) - std::get<2>(means),2);
      }

      std::get<0>(vec_rms_vals.at(i)) = sqrt(std::get<0>(vec_rms_vals.at(i))*1./(ncount));
      std::get<1>(vec_rms_vals.at(i)) = sqrt(std::get<1>(vec_rms_vals.at(i))*1./(ncount));
      std::get<2>(vec_rms_vals.at(i)) = sqrt(std::get<2>(vec_rms_vals.at(i))*1./(ncount));
    }
    vec_dQ_dx.at(i) = dQ_vec.at(i)/(dx_vec.at(i)+1e-9)/43e3*units::cm;
  }

  double max_spread = 0;
  double large_spread_length = 0;
  double total_effective_length = 0;

  double max_cont_length = 0;
  double max_cont_weighted_length = 0;
  
  double cont_length = 0;
  double cont_weighted_length = 0;

  bool flag_prev = false;
  double total_length;
  for (size_t i=0;i+1<local_points_vec.size();i++){
    double length = sqrt(pow(fit_pt_vec.at(i+1).x - fit_pt_vec.at(i).x,2) + pow(fit_pt_vec.at(i+1).y - fit_pt_vec.at(i).y,2) + pow(fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z,2));
    //std::cout << i << " " << std::get<2>(vec_rms_vals.at(i))/units::cm << std::endl;
    total_length += length;
    if (std::get<2>(vec_rms_vals.at(i))!=0){
      total_effective_length += length;
      if (std::get<2>(vec_rms_vals.at(i)) > 0.4*units::cm) {
	large_spread_length += length;

	cont_length += length;
	cont_weighted_length += length * std::get<2>(vec_rms_vals.at(i));
	flag_prev = true;
      }else{
	if (flag_prev && cont_length > max_cont_length){
	  max_cont_length = cont_length;
	  max_cont_weighted_length = cont_weighted_length;
	}
	cont_length = 0;
	cont_weighted_length = 0;
	flag_prev = false;
      }
      if (std::get<2>(vec_rms_vals.at(i)) > max_spread) max_spread = std::get<2>(vec_rms_vals.at(i));
    }    
  }
  
            //  std::cout << "Shower Topology Direction: " << max_spread/units::cm << " " << large_spread_length/units::cm << " " << total_effective_length/units::cm << std::endl;


  if (max_spread > 0.7*units::cm && large_spread_length > 0.2 * total_effective_length && total_effective_length > 3*units::cm && total_effective_length < 15*units::cm && ( large_spread_length > 2.7*units::cm || large_spread_length > 0.35 * total_effective_length)
      || max_spread > 0.8*units::cm && large_spread_length > 0.3 * total_effective_length && total_effective_length >= 15*units::cm
      || max_spread > 1.0*units::cm && large_spread_length > 0.4 * total_effective_length) {
    TVector3 drift_dir(1,0,0);
    
    TVector3 main_dir1, main_dir2;
    bool flag_skip_angle1 = false;
    bool flag_skip_angle2 = false;
    if (fit_pt_vec.front().z < fit_pt_vec.back().z){
      main_dir1 = cal_dir_3vector(fit_pt_vec.front(), 15*units::cm);
      main_dir2 = cal_dir_3vector(fit_pt_vec.back(), 6*units::cm);
      if (fabs(main_dir1.Angle(drift_dir)/3.1415926*180.-90)<10) flag_skip_angle1 = true;
    }else{
      main_dir1 = cal_dir_3vector(fit_pt_vec.front(), 6*units::cm);
      main_dir2 = cal_dir_3vector(fit_pt_vec.back(), 15*units::cm);
      if (fabs(main_dir2.Angle(drift_dir)/3.1415926*180.-90)<10) flag_skip_angle2 = true;
    }

   
    //    std::cout << main_dir.Angle(drift_dir)/3.1415926*180. << std::endl;
    
    std::vector<std::tuple<int, int, double> > threshold_segs;
    for(size_t i=0;i!=vec_dQ_dx.size(); i++){
      //      std::cout << i << " A: " << main_dir.Angle(vec_dir.at(i))/3.1415926*180. << " " <<vec_dir.at(i).Angle(drift_dir)/3.1415926*180. << " " << vec_dQ_dx.at(i) << " " << std::get<2>(vec_rms_vals.at(i))/units::cm << std::endl;
      double angle = main_dir1.Angle(vec_dir.at(i))/3.1415926*180.;
      if (threshold_segs.size()==0 && (angle < 30 || flag_skip_angle1 && angle < 60) && (std::get<2>(vec_rms_vals.at(i))/units::cm> 0.4 || vec_dQ_dx.at(i) > 1.6)){
	threshold_segs.push_back(std::make_tuple(i,i,std::get<2>(vec_rms_vals.at(i))));
      }else if ((angle < 30 || flag_skip_angle1 && angle < 60) && (std::get<2>(vec_rms_vals.at(i))/units::cm> 0.4 || vec_dQ_dx.at(i) > 1.6)){
	if (i == std::get<1>(threshold_segs.back())+1 && std::get<2>(threshold_segs.back()) < std::get<2>(vec_rms_vals.at(i)) ){
	  std::get<1>(threshold_segs.back()) = i;
	  std::get<2>(threshold_segs.back()) = std::get<2>(vec_rms_vals.at(i));
	}else{
	  if (i!=std::get<1>(threshold_segs.back())+1)
	    threshold_segs.push_back(std::make_tuple(i,i,std::get<2>(vec_rms_vals.at(i))));
	}
      }
      // loop ...
    }
    
    double total_length1 = 0, max_length1 = 0;
    for (auto it = threshold_segs.begin(); it!= threshold_segs.end(); it++){
      int start_n = std::get<0>(*it);
      if (start_n >0) start_n --;
      int end_n = std::get<1>(*it);
      double tmp_length = 0;
      for (int i=start_n; i!=end_n;i++){
	tmp_length += sqrt(pow(fit_pt_vec.at(i+1).x - fit_pt_vec.at(i).x,2)+pow(fit_pt_vec.at(i+1).y - fit_pt_vec.at(i).y,2)+pow(fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z,2));
      }
      total_length1 += tmp_length;
      if (tmp_length > max_length1) max_length1 = tmp_length;
      //      std::cout << id << " A: " << start_n << " " << end_n << " " << tmp_length/units::cm << std::endl;
    }
    
    
    threshold_segs.clear();
    for(int i=vec_dQ_dx.size()-1;i>=0; i--){
      // std::cout << "B: " << 180-main_dir.Angle(vec_dir.at(i))/3.1415926*180. << std::endl;
      double angle = 180-main_dir2.Angle(vec_dir.at(i))/3.1415926*180.;
      if (threshold_segs.size()==0 && (angle  < 30 || flag_skip_angle2 && angle < 60) && (std::get<2>(vec_rms_vals.at(i))/units::cm> 0.4 || vec_dQ_dx.at(i) > 1.6)){
	threshold_segs.push_back(std::make_tuple(i,i,std::get<2>(vec_rms_vals.at(i))));
      }else if ( (angle < 30 || flag_skip_angle2 && angle < 60) && (std::get<2>(vec_rms_vals.at(i))/units::cm> 0.4 || vec_dQ_dx.at(i) > 1.6)){
	if (i == std::get<1>(threshold_segs.back())-1 && std::get<2>(threshold_segs.back()) < std::get<2>(vec_rms_vals.at(i)) ){
	  std::get<1>(threshold_segs.back()) = i;
	  std::get<2>(threshold_segs.back()) = std::get<2>(vec_rms_vals.at(i));
	}else{
	  if (i!=std::get<1>(threshold_segs.back())-1)
	    threshold_segs.push_back(std::make_tuple(i,i,std::get<2>(vec_rms_vals.at(i))));
	}
      }
      // loop ...
    }
    
    
    double total_length2 = 0, max_length2 = 0;
    for (auto it = threshold_segs.begin(); it!= threshold_segs.end(); it++){
      int start_n = std::get<0>(*it);
      if (start_n <fit_pt_vec.size()-1) start_n ++;
      int end_n = std::get<1>(*it);
      double tmp_length = 0;
      for (int i=start_n; i!=end_n;i--){
	tmp_length+= sqrt(pow(fit_pt_vec.at(i-1).x - fit_pt_vec.at(i).x,2)+pow(fit_pt_vec.at(i-1).y - fit_pt_vec.at(i).y,2)+pow(fit_pt_vec.at(i-1).z - fit_pt_vec.at(i).z,2));
      }
      total_length2 += tmp_length;
      if (tmp_length > max_length2) max_length2 = tmp_length;
      //       std::cout << id << " B: " << start_n << " " << end_n << " " << tmp_length/units::cm << std::endl;
    }
    
    if (total_length1 + max_length1 > 1.1*(total_length2 + max_length2)){
      flag_dir = 1;
    }else if (1.1*(total_length1 + max_length1) < total_length2 + max_length2){
      flag_dir = -1;
    }
    
    //    std::cout << cluster_id << " " << id << " " << total_length1/units::cm << " " << total_length2/units::cm << " " << max_length1/units::cm << " " << max_length2/units::cm << " " << flag_dir << std::endl;
  }else{
    if (total_length < 5*units::cm){
      if (!is_shower_trajectory())
	determine_dir_track(0, fit_pt_vec.size());
    }else{
      
      TVector3 main_dir = cal_dir_3vector(fit_pt_vec.front(),6*units::cm);
      Int_t ncount_front = 0, ncount_back = 0;
      for(size_t i=0;i!=vec_dQ_dx.size(); i++){
	if (main_dir.Angle(vec_dir.at(i))/3.1415926*180. < 30) ncount_front ++;
	  //	std::cout << i << " A: " << main_dir.Angle(vec_dir.at(i))/3.1415926*180. << std::endl;
      }
      main_dir = cal_dir_3vector(fit_pt_vec.back(),6*units::cm);
      for(int i=vec_dQ_dx.size()-1;i>=0; i--){
	if (180-main_dir.Angle(vec_dir.at(i))/3.1415926*180. < 30) ncount_back++;
	//	std::cout << "B: " << 180-main_dir.Angle(vec_dir.at(i))/3.1415926*180. << std::endl;
      }
      if (1.2 * ncount_front  < ncount_back){
	flag_dir = -1;
      }else if (ncount_front > 1.2 * ncount_back){
	flag_dir = 1;
      }
      // std::cout << id << " " << ncount_front << " " << ncount_back << std::endl;
       
    }
  }
  
  if (flag_dir !=0) return true;
  else return false; 
  
  
    
   
}


bool WCPPID::ProtoSegment::is_shower_topology(bool tmp_val){
  flag_shower_topology = tmp_val;
  flag_dir = 0;
  
  // look at the points ...
  std::vector<PointVector > local_points_vec(fit_pt_vec.size());
  std::vector<std::tuple<double, double, double> > vec_rms_vals(fit_pt_vec.size(), std::make_tuple(0,0,0));
  std::vector<double> vec_dQ_dx(fit_pt_vec.size(), 0);

  //  std::cout << fit_pt_vec.size() << " " << pcloud_associated << " " << std::endl;
  if (pcloud_associated == 0 ) return false;
  
  WCP::WCPointCloud<double>& cloud = pcloud_associated->get_cloud();

  for (size_t i = 0; i!= cloud.pts.size(); i++){
    Point test_p(cloud.pts.at(i).x, cloud.pts.at(i).y, cloud.pts.at(i).z);
    std::vector<std::pair<size_t,double>> results = pcloud_fit->get_closest_index(test_p,1);
    local_points_vec.at(results.front().first).push_back(test_p);
  }

  TVector3 drift_dir(1,0,0); // drift direction
  
  for (size_t i=0;i!=local_points_vec.size();i++){
    TVector3 dir_1, dir_2, dir_3;
    Point v1(0,0,0);
    for (size_t j=1;j!=3;j++){
      if (i+j<local_points_vec.size()){
	v1.x += fit_pt_vec.at(i+j).x - fit_pt_vec.at(i).x;
	v1.y += fit_pt_vec.at(i+j).y - fit_pt_vec.at(i).y;
	v1.z += fit_pt_vec.at(i+j).z - fit_pt_vec.at(i).z;
      }
      if (i>=j){
	v1.x += fit_pt_vec.at(i).x - fit_pt_vec.at(i-j).x;
	v1.y += fit_pt_vec.at(i).y - fit_pt_vec.at(i-j).y;
	v1.z += fit_pt_vec.at(i).z - fit_pt_vec.at(i-j).z;
      }
    }
    dir_1.SetXYZ(v1.x, v1.y, v1.z);
    dir_1 = dir_1.Unit();

    if (dir_1.Angle(drift_dir)/3.1415926*180. < 7.5){
      dir_1.SetXYZ(1,0,0);
      dir_2.SetXYZ(0,1,0);
      dir_3.SetXYZ(0,0,1);
    }else{
      dir_2 = drift_dir.Cross(dir_1);
      dir_2 = dir_2.Unit();
      dir_3 = dir_1.Cross(dir_2);
    }
    
    std::vector<std::tuple<double, double, double> > vec_projs;
    for (size_t j=0;j!=local_points_vec.at(i).size();j++){
      double proj_1 = dir_1.X() * local_points_vec.at(i).at(j).x + dir_1.Y() * local_points_vec.at(i).at(j).y + dir_1.Z() * local_points_vec.at(i).at(j).z;
      double proj_2 = dir_2.X() * local_points_vec.at(i).at(j).x + dir_2.Y() * local_points_vec.at(i).at(j).y + dir_2.Z() * local_points_vec.at(i).at(j).z;
      double proj_3 = dir_3.X() * local_points_vec.at(i).at(j).x + dir_3.Y() * local_points_vec.at(i).at(j).y + dir_3.Z() * local_points_vec.at(i).at(j).z;
      vec_projs.push_back(std::make_tuple(proj_1, proj_2, proj_3));
    }
    std::tuple<double, double, double> means = std::make_tuple(0,0,0);
    int ncount = local_points_vec.at(i).size();
    //for (size_t j=0;j!=local_points_vec.at(i).size();j++){
    //  std::get<0>(means) += std::get<0>(vec_projs.at(j));
    //  std::get<1>(means) += std::get<1>(vec_projs.at(j));
    //  std::get<2>(means) += std::get<2>(vec_projs.at(j));
    //  ncount ++;
    //}
    if (ncount >1){
      //std::get<0>(means) /= 1.*ncount;
      //std::get<1>(means) /= 1.*ncount;
      //std::get<2>(means) /= 1.*ncount;

      std::get<0>(means) = dir_1.X() * fit_pt_vec.at(i).x + dir_1.Y() * fit_pt_vec.at(i).y + dir_1.Z() * fit_pt_vec.at(i).z;
      std::get<1>(means) = dir_2.X() * fit_pt_vec.at(i).x + dir_2.Y() * fit_pt_vec.at(i).y + dir_2.Z() * fit_pt_vec.at(i).z;
      std::get<2>(means) = dir_3.X() * fit_pt_vec.at(i).x + dir_3.Y() * fit_pt_vec.at(i).y + dir_3.Z() * fit_pt_vec.at(i).z;
      
      for (size_t j=0;j!=local_points_vec.at(i).size();j++){
	std::get<0>(vec_rms_vals.at(i)) += pow(std::get<0>(vec_projs.at(j)) - std::get<0>(means),2);
	std::get<1>(vec_rms_vals.at(i)) += pow(std::get<1>(vec_projs.at(j)) - std::get<1>(means),2);
	std::get<2>(vec_rms_vals.at(i)) += pow(std::get<2>(vec_projs.at(j)) - std::get<2>(means),2);
      }

      std::get<0>(vec_rms_vals.at(i)) = sqrt(std::get<0>(vec_rms_vals.at(i))*1./(ncount));
      std::get<1>(vec_rms_vals.at(i)) = sqrt(std::get<1>(vec_rms_vals.at(i))*1./(ncount));
      std::get<2>(vec_rms_vals.at(i)) = sqrt(std::get<2>(vec_rms_vals.at(i))*1./(ncount));
    }
    vec_dQ_dx.at(i) = dQ_vec.at(i)/(dx_vec.at(i)+1e-9)/43e3*units::cm;
    //std::cout << dir_1.Angle(drift_dir)/3.1415926*180. << " " << dir_1.Mag() << " " << v1 << std::endl;
    //    std::cout << i << " " << dQ_vec.at(i)/(dx_vec.at(i)+1e-9)/43e3*units::cm << " " << std::get<0>(vec_rms_vals.at(i))/units::cm << " " << std::get<1>(vec_rms_vals.at(i))/units::cm << " " << std::get<2>(vec_rms_vals.at(i))/units::cm << std::endl;
  }

  double max_spread = 0;
  double large_spread_length = 0;
  double total_effective_length = 0;

  double max_cont_length = 0;
  double max_cont_weighted_length = 0;
  
  double cont_length = 0;
  double cont_weighted_length = 0;

  bool flag_prev = false;
  
  for (size_t i=0;i+1<local_points_vec.size();i++){
    double length = sqrt(pow(fit_pt_vec.at(i+1).x - fit_pt_vec.at(i).x,2) + pow(fit_pt_vec.at(i+1).y - fit_pt_vec.at(i).y,2) + pow(fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z,2));
    if (std::get<2>(vec_rms_vals.at(i))!=0){
      total_effective_length += length;
      if (std::get<2>(vec_rms_vals.at(i)) > 0.4*units::cm) {
	large_spread_length += length;

	cont_length += length;
	cont_weighted_length += length * std::get<2>(vec_rms_vals.at(i));
	flag_prev = true;
      }else{
	if (flag_prev && cont_length > max_cont_length){
	  max_cont_length = cont_length;
	  max_cont_weighted_length = cont_weighted_length;
	}
	cont_length = 0;
	cont_weighted_length = 0;
	flag_prev = false;
      }
      if (std::get<2>(vec_rms_vals.at(i)) > max_spread) max_spread = std::get<2>(vec_rms_vals.at(i));
    }
  }

            // std::cout << "Shower Topology Check: " << max_spread/units::cm << " " << large_spread_length/units::cm << " " << total_effective_length/units::cm << std::endl;

  
  if (max_spread > 0.7*units::cm && large_spread_length > 0.2 * total_effective_length && total_effective_length > 3*units::cm && total_effective_length < 15*units::cm && ( large_spread_length > 2.7*units::cm || large_spread_length > 0.35 * total_effective_length)
      || max_spread > 0.8*units::cm && large_spread_length > 0.3 * total_effective_length && total_effective_length >= 15*units::cm
      || max_spread > 0.8*units::cm && large_spread_length > 8*units::cm && large_spread_length > 0.18 * total_effective_length
      || max_spread > 1.0*units::cm && large_spread_length > 0.4 * total_effective_length
      || max_spread > 1.0*units::cm && large_spread_length > 5*units::cm && large_spread_length > 0.23 * total_effective_length
      ){

  
    flag_shower_topology = true;
      
  }
  //  std::cout << cluster_id << " " << id << " " << max_spread/units::cm << " " << large_spread_length/units::cm <<  " " << total_effective_length/units::cm << std::endl;
  
  
  if (flag_shower_topology){
    std::vector<std::tuple<int, int, double> > threshold_segs;
    for(size_t i=0;i!=vec_dQ_dx.size(); i++){
      if (threshold_segs.size()==0 && std::get<2>(vec_rms_vals.at(i))/units::cm> 0.4){
	threshold_segs.push_back(std::make_tuple(i,i,std::get<2>(vec_rms_vals.at(i))));
      }else if (std::get<2>(vec_rms_vals.at(i))/units::cm> 0.4){
	if (i == std::get<1>(threshold_segs.back())+1 && std::get<2>(threshold_segs.back()) < std::get<2>(vec_rms_vals.at(i)) ){
	  std::get<1>(threshold_segs.back()) = i;
	  std::get<2>(threshold_segs.back()) = std::get<2>(vec_rms_vals.at(i));
	}else{
	  if (i!=std::get<1>(threshold_segs.back())+1)
	    threshold_segs.push_back(std::make_tuple(i,i,std::get<2>(vec_rms_vals.at(i))));
	}
      }
      // loop ...
    }

    double total_length1 = 0, max_length1 = 0;
    for (auto it = threshold_segs.begin(); it!= threshold_segs.end(); it++){
      int start_n = std::get<0>(*it);
      if (start_n >0) start_n --;
      int end_n = std::get<1>(*it);
      double tmp_length = 0;
      for (int i=start_n; i!=end_n;i++){
        tmp_length += sqrt(pow(fit_pt_vec.at(i+1).x - fit_pt_vec.at(i).x,2)+pow(fit_pt_vec.at(i+1).y - fit_pt_vec.at(i).y,2)+pow(fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z,2));
      }
      total_length1 += tmp_length;
      if (tmp_length > max_length1) max_length1 = tmp_length;
      //      std::cout << id << " A: " << start_n << " " << end_n << " " << tmp_length/units::cm << std::endl;
    }


    threshold_segs.clear();
    for(int i=vec_dQ_dx.size()-1;i>=0; i--){
      if (threshold_segs.size()==0 && std::get<2>(vec_rms_vals.at(i))/units::cm> 0.4){
	threshold_segs.push_back(std::make_tuple(i,i,std::get<2>(vec_rms_vals.at(i))));
      }else if (std::get<2>(vec_rms_vals.at(i))/units::cm> 0.4){
	if (i == std::get<1>(threshold_segs.back())-1 && std::get<2>(threshold_segs.back()) < std::get<2>(vec_rms_vals.at(i)) ){
	  std::get<1>(threshold_segs.back()) = i;
	  std::get<2>(threshold_segs.back()) = std::get<2>(vec_rms_vals.at(i));
	}else{
	  if (i!=std::get<1>(threshold_segs.back())-1)
	    threshold_segs.push_back(std::make_tuple(i,i,std::get<2>(vec_rms_vals.at(i))));
	}
      }
      // loop ...
    }

    
    double total_length2 = 0, max_length2 = 0;
    for (auto it = threshold_segs.begin(); it!= threshold_segs.end(); it++){
       int start_n = std::get<0>(*it);
       if (start_n <fit_pt_vec.size()-1) start_n ++;
       int end_n = std::get<1>(*it);
       double tmp_length = 0;
       for (int i=start_n; i!=end_n;i--){
	 tmp_length+= sqrt(pow(fit_pt_vec.at(i-1).x - fit_pt_vec.at(i).x,2)+pow(fit_pt_vec.at(i-1).y - fit_pt_vec.at(i).y,2)+pow(fit_pt_vec.at(i-1).z - fit_pt_vec.at(i).z,2));
       }
       total_length2 += tmp_length;
       if (tmp_length > max_length2) max_length2 = tmp_length;
       //       std::cout << id << " B: " << start_n << " " << end_n << " " << tmp_length/units::cm << std::endl;
    }

    if (total_length1 + max_length1 > 1.1*(total_length2 + max_length2)){
      flag_dir = 1;
    }else if (1.1*(total_length1 + max_length1) < total_length2 + max_length2){
      flag_dir = -1;
    }
    double tmp_total_length = get_length();
    if (tmp_total_length > 50*units::cm && total_length1 < 0.25* tmp_total_length && total_length2 < 0.25* tmp_total_length ){
      flag_dir = 0;
      flag_shower_topology = false;
    }
	
    //    std::cout << cluster_id << " " << id << " " << total_length1/units::cm << " " << total_length2/units::cm << " " << max_length1/units::cm << " " << max_length2/units::cm << " " << flag_dir << " " << get_length()/units::cm << std::endl;
    
  }
  
  //  std::cout << cluster_id << " "  << id << " " << max_spread/units::cm << " " << large_spread_length/total_effective_length << " " << max_cont_length/units::cm << " " << total_effective_length/units::cm << " " << flag_shower_trajectory << " " << flag_shower_topology << std::endl;
  
  return flag_shower_topology;
}

bool WCPPID::ProtoSegment::is_shower_trajectory(double step_size){
  flag_shower_trajectory = false;
  double length = get_length();
  // too long
  if (length > 50*units::cm) return flag_shower_trajectory;
  
  int ncount = std::round(length/step_size);
  if (ncount ==0 ) ncount = 1;

  std::vector<std::pair<int,int> > sections(ncount);
  for (size_t i=0;i!=ncount;i++){
    sections.at(i)=std::make_pair(std::round(fit_pt_vec.size()/ncount*i),std::round(fit_pt_vec.size()/ncount*(i+1)));
  }
  sections.back().second = fit_pt_vec.size()-1;

  int n_shower_like = 0;

  TVector3 drift_dir(1,0,0);
  
  // 
  for (size_t j=0;j!=ncount;j++){
    TVector3 dir_1(fit_pt_vec.at(sections.at(j).first).x - fit_pt_vec.at(sections.at(j).second).x, fit_pt_vec.at(sections.at(j).first).y - fit_pt_vec.at(sections.at(j).second).y, fit_pt_vec.at(sections.at(j).first).z - fit_pt_vec.at(sections.at(j).second).z);
    dir_1 = dir_1.Unit();
    double tmp_dQ_dx = get_medium_dQ_dx(sections.at(j).first, sections.at(j).second)/(50000/units::cm);

    //    std::cout << fabs(drift_dir.Angle(dir_1)/3.1415926*180.-90.) << std::endl;
    double angle_diff = fabs(drift_dir.Angle(dir_1)/3.1415926*180.-90.);
    if (angle_diff>10 ){ // not parallel case ...
      double direct_length = get_direct_length(sections.at(j).first, sections.at(j).second);
      double integrated_length = get_length(sections.at(j).first, sections.at(j).second);
      double max_dev = get_max_deviation(sections.at(j).first, sections.at(j).second);

      //      std::cout << max_dev/units::cm << " " << fabs(drift_dir.Angle(dir_1)/3.1415926*180.-90.) << std::endl;
      
      double length_ratio;
      if (direct_length == 0 ) length_ratio = 1;
      else length_ratio = direct_length / integrated_length;
    
      if (tmp_dQ_dx*0.11 + 2*length_ratio < 2.03 && tmp_dQ_dx < 2 && length_ratio < 0.95 && (angle_diff < 60 || integrated_length < 10*units::cm || integrated_length >= 10*units::cm && max_dev > 0.75*units::cm)
	  ) n_shower_like ++;
      //      std::cout << "Xin: " << id << " " << j << " " << sections.at(j).first << " " << sections.at(j).second <<  " " << length_ratio << " " << tmp_dQ_dx << " " << direct_length << " " << drift_dir.Angle(dir_1)/3.1415926*180. << std::endl;
    }else{
      TVector3 dir_2 = drift_dir.Cross(dir_1);
      dir_2 = dir_2.Unit();
      TVector3 dir_3 = dir_1.Cross(dir_2);
      double direct_length = get_direct_length(sections.at(j).first, sections.at(j).second, dir_2);
      double integrated_length = get_length(sections.at(j).first, sections.at(j).second, dir_2);
      double max_dev = get_max_deviation(sections.at(j).first, sections.at(j).second);
      double length_ratio;
      if (direct_length == 0 ) length_ratio = 1;
      else length_ratio = direct_length / integrated_length;
      if (tmp_dQ_dx*0.11 + 2*length_ratio < 2.06 && tmp_dQ_dx < 2 && length_ratio < 0.97 && (integrated_length < 10*units::cm || integrated_length >= 10*units::cm && max_dev > 0.75*units::cm) ) n_shower_like ++;
      // std::cout << "Xin: " << j << " " << sections.at(j).first << " " << sections.at(j).second <<  " " << length_ratio << " " << tmp_dQ_dx << " " << direct_length << " " << drift_dir.Angle(dir_1)/3.1415926*180. << " " << tmp_dQ_dx*0.11 + 2*length_ratio - 2 << std::endl;  
    }
  }

  
  
  //std::cout << "BB " << id << " " << sections.size() << " " << get_length()/units::cm << " " << n_shower_like << std::endl;
  //std::cout << id << " " << get_medium_dQ_dx()/(43e3/units::cm) << std::endl;

  //  double medium_dQ_dx = get_medium_dQ_dx()/(43e3/units::cm);
  // not a good idea to require dQ/dx ...
  if (n_shower_like >=0.5*sections.size()
      //&& medium_dQ_dx < 1.1
      ) flag_shower_trajectory = true;
  
  // calculate direct length, accumulated length, medium dQ/dx in each section ...
  //  std::cout << length/units::cm << " " << ncount << std::endl;
  return flag_shower_trajectory;
}

double WCPPID::ProtoSegment::get_max_deviation(int n1, int n2){
  if (n1 < 0) n1 = 0;  if (n1+1 > fit_pt_vec.size()) n1 = int(fit_pt_vec.size())-1;
  if (n2 < 0) n2 = 0;  if (n2+1 > fit_pt_vec.size()) n2 = int(fit_pt_vec.size())-1;
  
  double max_dev = 0;
  if (n1 != n2){
    WCP::Line l(fit_pt_vec.at(n1), fit_pt_vec.at(n2));
    for (size_t i=n1; i<=n2;i++){
      double dis = l.closest_dis(fit_pt_vec.at(i));
      if (dis > max_dev) max_dev = dis;
    }
  }
  
  return max_dev;
}

double WCPPID::ProtoSegment::get_direct_length(int n1, int n2){
  if (n1 < 0) n1 = 0;  if (n1+1 > fit_pt_vec.size()) n1 = int(fit_pt_vec.size())-1;
  if (n2 < 0) n2 = 0;  if (n2+1 > fit_pt_vec.size()) n2 = int(fit_pt_vec.size())-1;

  double length = sqrt(pow(fit_pt_vec.at(n1).x - fit_pt_vec.at(n2).x,2)+pow(fit_pt_vec.at(n1).y - fit_pt_vec.at(n2).y,2)+pow(fit_pt_vec.at(n1).z - fit_pt_vec.at(n2).z,2));
  return length;
}

double WCPPID::ProtoSegment::get_length(int n1, int n2){
  if (n1 < 0) n1 = 0;  if (n1+1 > fit_pt_vec.size()) n1 = int(fit_pt_vec.size())-1;
  if (n2 < 0) n2 = 0;  if (n2+1 > fit_pt_vec.size()) n2 = int(fit_pt_vec.size())-1;
  double length = 0;
  for (int i=n1;i+1<=n2;i++){
    length += sqrt(pow(fit_pt_vec.at(i+1).x - fit_pt_vec.at(i).x,2)+pow(fit_pt_vec.at(i+1).y - fit_pt_vec.at(i).y,2)+pow(fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z,2));
  }
  return length;
}

double WCPPID::ProtoSegment::get_direct_length(int n1, int n2, TVector3 dir_perp){
  if (n1 < 0) n1 = 0;  if (n1+1 > fit_pt_vec.size()) n1 = int(fit_pt_vec.size())-1;
  if (n2 < 0) n2 = 0;  if (n2+1 > fit_pt_vec.size()) n2 = int(fit_pt_vec.size())-1;

  TVector3 temp_dir(fit_pt_vec.at(n1).x - fit_pt_vec.at(n2).x, fit_pt_vec.at(n1).y - fit_pt_vec.at(n2).y, fit_pt_vec.at(n1).z - fit_pt_vec.at(n2).z);
  double length = sqrt(pow(temp_dir.Mag(),2) - pow(temp_dir.Dot(dir_perp.Unit()),2));
  return length;
}

double WCPPID::ProtoSegment::get_length(int n1, int n2, TVector3 dir_perp){
  if (n1 < 0) n1 = 0;  if (n1+1 > fit_pt_vec.size()) n1 = int(fit_pt_vec.size())-1;
  if (n2 < 0) n2 = 0;  if (n2+1 > fit_pt_vec.size()) n2 = int(fit_pt_vec.size())-1;
  
  double length = 0;
  for (int i=n1;i+1<=n2;i++){
    TVector3 temp_dir(fit_pt_vec.at(i+1).x - fit_pt_vec.at(i).x, fit_pt_vec.at(i+1).y - fit_pt_vec.at(i).y, fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z);
    length += sqrt(pow(temp_dir.Mag(),2)-pow(temp_dir.Dot(dir_perp.Unit()),2));
  }
  return length;
}

double WCPPID::ProtoSegment::get_rms_dQ_dx(){
  std::vector<double> vec_dQ_dx;
  double sum[3]={0,0,0};
  if (dQ_vec.size()<=1) return 0;
  
  for (int i = 0 ; i< dQ_vec.size(); i++){
    vec_dQ_dx.push_back(dQ_vec.at(i)/(dx_vec.at(i)+1e-9));
    sum[0] += pow(vec_dQ_dx.back(),2);
    sum[1] += vec_dQ_dx.back();
    sum[2] += 1;
  }

  return sqrt((sum[0] - pow(sum[1],2)/sum[2])/(sum[2]-1.));
}

double WCPPID::ProtoSegment::get_medium_dQ_dx(){
  return get_medium_dQ_dx(0, fit_pt_vec.size());
}

double WCPPID::ProtoSegment::get_medium_dQ_dx(int n1, int n2){
  if (n1 < 0) n1 = 0;  if (n1+1 > fit_pt_vec.size()) n1 = int(fit_pt_vec.size())-1;
  if (n2 < 0) n2 = 0;  if (n2+1 > fit_pt_vec.size()) n2 = int(fit_pt_vec.size())-1;
  std::vector<double> vec_dQ_dx;
  for (int i = n1 ; i<=n2; i++){
    vec_dQ_dx.push_back(dQ_vec.at(i)/(dx_vec.at(i)+1e-9));
  }
  std::nth_element(vec_dQ_dx.begin(), vec_dQ_dx.begin() + vec_dQ_dx.size()/2, vec_dQ_dx.end());
  return *std::next(vec_dQ_dx.begin(), vec_dQ_dx.size()/2);
}

double WCPPID::ProtoSegment::get_direct_length(){
  double length =sqrt(pow(fit_pt_vec.front().x - fit_pt_vec.back().x ,2) + pow(fit_pt_vec.front().y - fit_pt_vec.back().y,2) + pow(fit_pt_vec.front().z - fit_pt_vec.back().z,2)); 
  
  return length;
}

double WCPPID::ProtoSegment::get_length(){
  double length = 0;
  for (size_t i=0;i+1<fit_pt_vec.size();i++){
    length += sqrt(pow(fit_pt_vec.at(i+1).x - fit_pt_vec.at(i).x ,2) + pow(fit_pt_vec.at(i+1).y - fit_pt_vec.at(i).y,2) + pow(fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z,2));
  }
  return length;
}

std::tuple<WCP::Point, TVector3, TVector3, bool> WCPPID::ProtoSegment::search_kink(Point& start_p){
  // find the first point ...
  //  Point start_p = fit_pt_vec.front();
  std::pair<double, WCP::Point> tmp_results = get_closest_point(start_p);
  Point test_p = tmp_results.second;

  TVector3 drift_dir(1,0,0);

  std::vector<double> refl_angles(fit_pt_vec.size(),0);
  std::vector<double> para_angles(fit_pt_vec.size(),0);
  
  // start the angle search
 
  for (int i=0;i!=fit_pt_vec.size(); i++){
    double angle1 = 0;
    double angle2 = 0;
    
    for (int j=0;j!=6;j++){    
      TVector3 v10(0,0,0);
      TVector3 v20(0,0,0);

      //      std::cout << i << " " << j << std::endl;
      if (i >= (j+1)*2){
	v10.SetXYZ(fit_pt_vec.at(i).x - fit_pt_vec.at(i-(j+1)*2).x,
		   fit_pt_vec.at(i).y - fit_pt_vec.at(i-(j+1)*2).y,
		   fit_pt_vec.at(i).z - fit_pt_vec.at(i-(j+1)*2).z);
      }else{
	v10.SetXYZ(fit_pt_vec.at(i).x - fit_pt_vec.front().x,
		   fit_pt_vec.at(i).y - fit_pt_vec.front().y,
		   fit_pt_vec.at(i).z - fit_pt_vec.front().z);
      }
      
      if (i+(j+1)*2<fit_pt_vec.size()){
	v20.SetXYZ(fit_pt_vec.at(i+(j+1)*2).x - fit_pt_vec.at(i).x,
		   fit_pt_vec.at(i+(j+1)*2).y - fit_pt_vec.at(i).y,
		   fit_pt_vec.at(i+(j+1)*2).z - fit_pt_vec.at(i).z);
      }else{
	v20.SetXYZ(fit_pt_vec.back().x - fit_pt_vec.at(i).x,
		   fit_pt_vec.back().y - fit_pt_vec.at(i).y,
		   fit_pt_vec.back().z - fit_pt_vec.at(i).z);
      }
      
      if (j==0){
	angle1 = v10.Angle(v20)/3.1415926*180.;
	angle2 = std::max(fabs(v10.Angle(drift_dir)/3.1415926*180.-90.),
			  fabs(v20.Angle(drift_dir)/3.1415926*180.-90.));
      }else{
	if (v10.Mag()!=0 && v20.Mag()!=0){

	  // std::cout << i << " " << j << " " << angle1 << " " << angle2 << std::endl;
	  angle1 = std::min(v10.Angle(v20)/3.1415926*180., angle1);
	  angle2 = std::min(std::max(fabs(v10.Angle(drift_dir)/3.1415926*180.-90.),
				     fabs(v20.Angle(drift_dir)/3.1415926*180.-90.)),angle2);
	}
      }
    }
    
    refl_angles.at(i) = angle1;
    para_angles.at(i) = angle2;
  }

  bool flag_check = false;
  int save_i = -1;
  bool flag_switch = false;
  bool flag_search = false;
    
  for (int i=0;i!=fit_pt_vec.size();i++){
    if (sqrt(pow(test_p.x - fit_pt_vec.at(i).x,2) +
	     pow(test_p.y - fit_pt_vec.at(i).y,2) +
	     pow(test_p.z - fit_pt_vec.at(i).z,2) ) < 0.1*units::cm) flag_check = true;

    // not too close and not too far to the begin and end ...
    if (sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.front().x,2) +
	     pow(fit_pt_vec.at(i).y - fit_pt_vec.front().y,2) +
	     pow(fit_pt_vec.at(i).z - fit_pt_vec.front().z,2) ) < 1*units::cm ||
	sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.back().x,2) +
	     pow(fit_pt_vec.at(i).y - fit_pt_vec.back().y,2) +
	     pow(fit_pt_vec.at(i).z - fit_pt_vec.back().z,2) ) < 1*units::cm ||
	sqrt(pow(fit_pt_vec.at(i).x - start_p.x,2) +
	     pow(fit_pt_vec.at(i).y - start_p.y,2) +
	     pow(fit_pt_vec.at(i).z - start_p.z,2) ) < 1*units::cm
	) continue;

    // std::cout << sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.front().x,2) +
    // 	     pow(fit_pt_vec.at(i).y - fit_pt_vec.front().y,2) +
    // 		      pow(fit_pt_vec.at(i).z - fit_pt_vec.front().z,2) )/units::cm << std::endl;
    
    if (flag_check){
      double ave_dQ_dx = 0; int ave_count = 0;
      double max_dQ_dx = dQ_vec.at(i)/dx_vec.at(i);
      for (size_t j = -2;j!=3;j++){
	if (i+j<fit_pt_vec.size() && i+j>=0){
	  ave_dQ_dx += dQ_vec.at(i+j)/dx_vec.at(i+j);
	  ave_count ++;
	  if (dQ_vec.at(i+j)/dx_vec.at(i+j) > max_dQ_dx) max_dQ_dx = dQ_vec.at(i+j)/dx_vec.at(i+j);
	}
      }
      if (ave_count!=0) ave_dQ_dx /= ave_count; 
      
      double sum_angles = 0;
      double nsum = 0;

      double sum_angles1 = 0;
      double nsum1 = 0;
      
      for (int j = -2; j!=3;j++){
	if (i+j>=0 && i+j<fit_pt_vec.size()){
	  if (para_angles.at(i+j)>10){
	    sum_angles += pow(refl_angles.at(i+j),2);
	    nsum ++;
	  }
	  if (para_angles.at(i+j)>7.5){
	    sum_angles1 += pow(refl_angles.at(i+j),2);
	    nsum1 ++;
	  }
	}
      }
      if (nsum!=0) sum_angles=sqrt(sum_angles/nsum);
      if (nsum1!=0) sum_angles1 = sqrt(sum_angles1/nsum1);
      //if (wcpt_vec.front().index >940 && wcpt_vec.front().index < 1200)

      // if (fabs(fit_pt_vec.at(i).x-2121.19)<30 && fabs(fit_pt_vec.at(i).y-218.775) < 30 && fabs(fit_pt_vec.at(i).z-715.347)<30)
      // std::cout << i << " " << max_dQ_dx << " " << para_angles.at(i) << " " << refl_angles.at(i) << " " << sum_angles << " " << sum_angles1<< " " << sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.front().x,2) + pow(fit_pt_vec.at(i).y - fit_pt_vec.front().y,2) + pow(fit_pt_vec.at(i).z - fit_pt_vec.front().z,2) ) /units::cm << " " << sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.back().x,2) + pow(fit_pt_vec.at(i).y - fit_pt_vec.back().y,2) + pow(fit_pt_vec.at(i).z - fit_pt_vec.back().z,2) )/units::cm << " " << fit_pt_vec.at(i) << std::endl;
      
      if (para_angles.at(i) > 10 && refl_angles.at(i) > 30 && sum_angles > 15 ){
	save_i = i;
	break;
      }else if (para_angles.at(i) > 7.5 && refl_angles.at(i) > 45 && sum_angles1 > 25){
	save_i = i;
	break;
      }else if (para_angles.at(i) > 15 && refl_angles.at(i) > 27 && sum_angles > 12.5){
	//std::cout << i << " " << ave_dQ_dx << " " <<max_dQ_dx << " " << para_angles.at(i) << " " << refl_angles.at(i) << " " << sum_angles << " " << sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.front().x,2) + pow(fit_pt_vec.at(i).y - fit_pt_vec.front().y,2) + pow(fit_pt_vec.at(i).z - fit_pt_vec.front().z,2) ) /units::cm << " " << sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.back().x,2) + pow(fit_pt_vec.at(i).y - fit_pt_vec.back().y,2) + pow(fit_pt_vec.at(i).z - fit_pt_vec.back().z,2) )/units::cm << " " << fit_pt_vec.at(i) << std::endl;
	save_i = i;
	break;
      }else if(para_angles.at(i) > 15 && refl_angles.at(i) > 22 && sum_angles > 19 && max_dQ_dx > 43e3/units::cm*1.5 && ave_dQ_dx > 43e3/units::cm){
	//	std::cout << i << " " << ave_dQ_dx << " " << max_dQ_dx << " " << para_angles.at(i) << " " << refl_angles.at(i) << " " << sum_angles << " " << sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.front().x,2) + pow(fit_pt_vec.at(i).y - fit_pt_vec.front().y,2) + pow(fit_pt_vec.at(i).z - fit_pt_vec.front().z,2) ) /units::cm << " " << sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.back().x,2) + pow(fit_pt_vec.at(i).y - fit_pt_vec.back().y,2) + pow(fit_pt_vec.at(i).z - fit_pt_vec.back().z,2) )/units::cm << " " << fit_pt_vec.at(i) << std::endl;
	save_i = i;
	flag_search = true;
	break;
      }
    }
  }

  // return the results ...
  if (save_i>0 && save_i+1 < fit_pt_vec.size() ){
    Point p = fit_pt_vec.at(save_i);
    
    Point prev_p(0,0,0);
    Point next_p(0,0,0);
    int num_p = 0;
    int num_p1 = 0;

    double length1 = 0;
    double length2 = 0;
    Point last_p1, last_p2;
    for (size_t i=1;i!=10;i++){
      if (save_i>=i){
	length1 += sqrt(pow(fit_pt_vec.at(save_i-i).x -fit_pt_vec.at(save_i-i+1).x,2) + pow(fit_pt_vec.at(save_i-i).y -fit_pt_vec.at(save_i-i+1).y,2) + pow(fit_pt_vec.at(save_i-i).z -fit_pt_vec.at(save_i-i+1).z,2));
	  
	prev_p.x += fit_pt_vec.at(save_i-i).x;
	prev_p.y += fit_pt_vec.at(save_i-i).y;
	prev_p.z += fit_pt_vec.at(save_i-i).z;

	last_p1 = fit_pt_vec.at(save_i-i);
	
	num_p ++;
      }
      if (save_i+i<fit_pt_vec.size()){
	length2 += sqrt(pow(fit_pt_vec.at(save_i+i).x - fit_pt_vec.at(save_i+i-1).x,2)+pow(fit_pt_vec.at(save_i+i).y - fit_pt_vec.at(save_i+i-1).y,2)+pow(fit_pt_vec.at(save_i+i).z - fit_pt_vec.at(save_i+i-1).z,2));
	next_p.x += fit_pt_vec.at(save_i+i).x;
	next_p.y += fit_pt_vec.at(save_i+i).y;
	next_p.z += fit_pt_vec.at(save_i+i).z;
	
	last_p2 = fit_pt_vec.at(save_i+i);
	num_p1 ++;
      }
    }
    double length1_1 = sqrt(pow(last_p1.x - fit_pt_vec.at(save_i).x,2) + pow(last_p1.y - fit_pt_vec.at(save_i).y,2) + pow(last_p1.z - fit_pt_vec.at(save_i).z,2));
    double length2_1 = sqrt(pow(last_p2.x - fit_pt_vec.at(save_i).x,2) + pow(last_p2.y - fit_pt_vec.at(save_i).y,2) + pow(last_p2.z - fit_pt_vec.at(save_i).z,2));

    
    if (fabs(length2 - length2_1)< 0.03 * length2_1 && length1 * length2_1 > 1.06 * length2 * length1_1){
      flag_switch = true;
      flag_search = true;
    }else if (fabs(length1 - length1_1)< 0.03 * length1_1 && length2 * length1_1 > 1.06 * length1 * length2_1){
      flag_search = true;
    }
    
    
    //    std::cout << length1/length1_1 << " " << length2/length2_1 << " " << flag_switch << std::endl;
    
    prev_p.x /= num_p;
    prev_p.y /= num_p;
    prev_p.z /= num_p;

    next_p.x /= num_p1;
    next_p.y /= num_p1;
    next_p.z /= num_p1;
    
    TVector3 dir(p.x - prev_p.x, p.y - prev_p.y, p.z - prev_p.z);
    dir = dir.Unit();
    TVector3 dir1(p.x - next_p.x, p.y - next_p.y, p.z - next_p.z);
    dir1 = dir1.Unit();
    
    double sum_dQ = 0, sum_dx = 0;
    for (int i=-2;i!=3;i++){
      if (save_i+i>=0&&save_i+i<fit_pt_vec.size()){
	sum_dQ += dQ_vec.at(save_i+i);
	sum_dx += dx_vec.at(save_i+i);
      }
    }

    //    std::cout << sqrt(pow(p.x-start_p.x,2) + pow(p.y-start_p.y,2) + pow(p.z-start_p.z,2))/units::cm << std::endl;
    //    std::cout << sum_dQ/(sum_dx) << " " << flag_search << std::endl;
    
    if (flag_search){
      if (flag_switch){
	std::cout << "Cluster: " << cluster_id << " Continue Search True Kink in Backward" << std::endl;
	return std::make_tuple(p, dir1, dir, true);
      }else{
	std::cout << "Cluster: " << cluster_id<< " Continue Search True Kink in Forward" << std::endl;
	return std::make_tuple(p, dir, dir1, true);
      }
    }else if (sum_dQ/(sum_dx+1e-9) > 2500 ){
      if (flag_switch){
	return std::make_tuple(p, dir1, dir, false);
      }else{
	return std::make_tuple(p, dir, dir1, false);
      }
    }else{
      if (flag_switch){
	std::cout << "Cluster: " << cluster_id<< " Continue Search True Kink in Backward" << std::endl;
	return std::make_tuple(p, dir1, dir, true);
      }else{
	std::cout << "Cluster: " << cluster_id << " Continue Search True Kink in Forward" << std::endl;
	return std::make_tuple(p, dir, dir1, true);
      }
    }

  }else{

    Point p1 = fit_pt_vec.back();
    TVector3 dir(0,0,0);
    return std::make_tuple(p1,dir, dir,false);

  }
  //  std::cout << save_i << std::endl;
  
}

void WCPPID::ProtoSegment::set_point_vec(std::vector<WCP::Point >& tmp_pt_vec){
  fit_pt_vec = tmp_pt_vec;
  if (pcloud_fit != (ToyPointCloud*)0) delete pcloud_fit;
  pcloud_fit = new ToyPointCloud();
  pcloud_fit->AddPoints(fit_pt_vec);
  pcloud_fit->build_kdtree_index();
}


void WCPPID::ProtoSegment::set_fit_vec(std::vector<WCP::Point >& tmp_fit_pt_vec, std::vector<double>& tmp_dQ_vec, std::vector<double>& tmp_dx_vec, std::vector<double>& tmp_pu_vec, std::vector<double>& tmp_pv_vec, std::vector<double>& tmp_pw_vec, std::vector<double>& tmp_pt_vec, std::vector<double>& tmp_reduced_chi2_vec){
  flag_fit = true;
  
  fit_pt_vec = tmp_fit_pt_vec;
  dQ_vec = tmp_dQ_vec;
  dx_vec = tmp_dx_vec;
  dQ_dx_vec.resize(dQ_vec.size(),0);
  for (size_t i=0;i!=dQ_vec.size();i++){
    dQ_dx_vec.at(i) = dQ_vec.at(i)/(dx_vec.at(i)+1e-9);
  }

  pu_vec = tmp_pu_vec;
  pv_vec = tmp_pv_vec;
  pw_vec = tmp_pw_vec;
  pt_vec = tmp_pt_vec;
  reduced_chi2_vec = tmp_reduced_chi2_vec;

  if (pcloud_fit != (ToyPointCloud*)0) delete pcloud_fit;
  pcloud_fit = new ToyPointCloud();
  pcloud_fit->AddPoints(fit_pt_vec);
  pcloud_fit->build_kdtree_index();
}

void WCPPID::ProtoSegment::set_fit_associate_vec(std::vector<WCP::Point >& tmp_fit_pt_vec, std::vector<int>& tmp_fit_index, std::vector<bool>& tmp_fit_skip){
  fit_pt_vec = tmp_fit_pt_vec;
  fit_index_vec = tmp_fit_index;
  fit_flag_skip = tmp_fit_skip;
  
  if (pcloud_fit != (ToyPointCloud*)0) delete pcloud_fit;
  pcloud_fit = new ToyPointCloud();
  pcloud_fit->AddPoints(fit_pt_vec);
  pcloud_fit->build_kdtree_index();
}

void WCPPID::ProtoSegment::reset_fit_prop(){
  fit_index_vec.resize(fit_pt_vec.size(),-1);
  fit_flag_skip.resize(fit_pt_vec.size(),false);
}

void WCPPID::ProtoSegment::clear_fit(){
  flag_fit = false;

  fit_pt_vec.clear();
  fit_pt_vec.resize(wcpt_vec.size());
  for (size_t i=0;i!=wcpt_vec.size();i++){
    fit_pt_vec.at(i).x = wcpt_vec.at(i).x;
    fit_pt_vec.at(i).y = wcpt_vec.at(i).y;
    fit_pt_vec.at(i).z = wcpt_vec.at(i).z;
  }
  if (pcloud_fit != (ToyPointCloud*)0) delete pcloud_fit;
  pcloud_fit = new ToyPointCloud();
  pcloud_fit->AddPoints(fit_pt_vec);
  pcloud_fit->build_kdtree_index();
  
  
  
  dQ_vec.clear();
  dx_vec.clear();
  dQ_dx_vec.clear();
  pu_vec.clear();
  pv_vec.clear();
  pw_vec.clear();
  pt_vec.clear();
  reduced_chi2_vec.clear();

  fit_index_vec.clear();
  fit_flag_skip.clear();
}

void WCPPID::ProtoSegment::reset_associate_points(){
  if (pcloud_associated != (ToyPointCloud*)0){
    delete pcloud_associated;
    //    flag_good_associated.clear();
  }
  pcloud_associated = 0;
  if (pcloud_associated_steiner != (ToyPointCloud*)0){
    delete pcloud_associated_steiner;
    //    flag_good_associated_steiner.clear();
  }
  pcloud_associated_steiner = 0;
}

void WCPPID::ProtoSegment::add_associate_point_steiner(WCP::WCPointCloud<double>::WCPoint& wcp){
  if (pcloud_associated_steiner == (ToyPointCloud*)0)
    pcloud_associated_steiner = new ToyPointCloud();
  pcloud_associated_steiner->AddPoint(wcp);
  //  flag_good_associated_steiner.push_back(true);
}

void WCPPID::ProtoSegment::add_associate_point(WCPointCloud<double>::WCPoint& wcp, WC2DPointCloud<double>::WC2DPoint& wcp_u, WC2DPointCloud<double>::WC2DPoint& wcp_v, WC2DPointCloud<double>::WC2DPoint& wcp_w){
  //  std::cout << pcloud_associated << std::endl;
  
  if (pcloud_associated == (ToyPointCloud*)0)
    pcloud_associated = new ToyPointCloud();
  pcloud_associated->AddPoint(wcp, wcp_u, wcp_v, wcp_w);
  // flag_good_associated.push_back(true);
  
}

void WCPPID::ProtoSegment::print_dis(){
  for (size_t i=0;i!=wcpt_vec.size(); i++){
    Point p(wcpt_vec.at(i).x, wcpt_vec.at(i).y, wcpt_vec.at(i).z);
    std::pair<double, WCP::Point> results = get_closest_point(p);
    if (results.first > 0.6*units::cm)
      std::cout << i << " " << results.first/units::cm << std::endl;
  }
}

std::pair<double, WCP::Point> WCPPID::ProtoSegment::get_closest_point(WCP::Point &p){
  if (pcloud_fit != (ToyPointCloud*)0) 
    return pcloud_fit->get_closest_point(p);
  else{
    WCP::Point p1(0,0,0);
    return std::make_pair(-1,p1);
  }
}

double WCPPID::ProtoSegment::get_closest_2d_dis(double x, double y, int plane){
  return pcloud_fit->get_closest_2d_dis(x, y, plane);
}

std::tuple<double, double, double> WCPPID::ProtoSegment::get_closest_2d_dis(WCP::Point &p){
  if (pcloud_fit != (ToyPointCloud*)0) {
    std::pair<int, double> results_u = pcloud_fit->get_closest_2d_dis(p, 0);
    std::pair<int, double> results_v = pcloud_fit->get_closest_2d_dis(p, 1);
    std::pair<int, double> results_w = pcloud_fit->get_closest_2d_dis(p, 2);
    
    return std::make_tuple(results_u.second, results_v.second, results_w.second);
  }else{
    return std::make_tuple(-1,-1,-1);
  }
}


WCP::WCPointCloud<double>::WCPoint WCPPID::ProtoSegment::get_closest_wcpt(WCP::Point& test_p){
  double min_dis = 1e9;
  WCP::WCPointCloud<double>::WCPoint min_wcpt = wcpt_vec.front();
  for (auto it = wcpt_vec.begin(); it!=wcpt_vec.end(); it++){
    double dis = sqrt(pow(test_p.x - (*it).x,2) + pow(test_p.y - (*it).y,2) + pow(test_p.z - (*it).z,2));
    if (dis < min_dis){
      min_dis = dis;
      min_wcpt =  *it;
    }
  }
  return min_wcpt;
}

std::vector<double> WCPPID::ProtoSegment::do_track_comp(std::vector<double>& L , std::vector<double>& dQ_dx, double compare_range, double offset_length){
  TPCParams& mp = Singleton<TPCParams>::Instance();
  TGraph *g_muon = mp.get_muon_dq_dx();
  TGraph *g_proton = mp.get_proton_dq_dx();
  TGraph *g_electron = mp.get_electron_dq_dx();
  
  double end_L = L.back() + 0.15*units::cm-offset_length;
  
  
  int ncount = 0;
  std::vector<double> vec_x;
  std::vector<double> vec_y;
    
  for (size_t i=0;i!=L.size(); i++){
    if (end_L - L.at(i) < compare_range && end_L - L.at(i) > 0){ // check up to compared range ...
      vec_x.push_back(end_L-L.at(i));
      vec_y.push_back(dQ_dx.at(i));
      ncount ++;
    }
  }
    
  TH1F *h1 = new TH1F("h1","h1",ncount,0,ncount);
  TH1F *h2 = new TH1F("h2","h2",ncount,0,ncount);
  TH1F *h3 = new TH1F("h3","h3",ncount,0,ncount);
  TH1F *h4 = new TH1F("h4","h4",ncount,0,ncount);
  TH1F *h5 = new TH1F("h5","h5",ncount,0,ncount);
    
    
  for (size_t i=0;i!=ncount;i++){
    // std::cout << i << " " << vec_y.at(i) << std::endl;
    h1->SetBinContent(i+1,vec_y.at(i));
    h2->SetBinContent(i+1,g_muon->Eval((vec_x.at(i))/units::cm)); //muon
    h3->SetBinContent(i+1,50e3); //MIP like ...
    h4->SetBinContent(i+1,g_proton->Eval(vec_x.at(i)/units::cm)); //proton
    h5->SetBinContent(i+1,g_electron->Eval(vec_x.at(i)/units::cm));    // electron
  }
  double ks1 = h2->KolmogorovTest(h1,"M");
  double ratio1 = h2->GetSum()/(h1->GetSum()+1e-9);
  double ks2 = h3->KolmogorovTest(h1,"M");
  double ratio2 = h3->GetSum()/(h1->GetSum()+1e-9);
  double ks3 = h4->KolmogorovTest(h1,"M");
  double ratio3 = h4->GetSum()/(h1->GetSum()+1e-9);
  double ks4 = h5->KolmogorovTest(h1,"M");
  double ratio4 = h5->GetSum()/(h1->GetSum()+1e-9);
  
  delete h1;
  delete h2;
  delete h3;
  delete h4;
  delete h5;
  
  // std::cout <<  ks1 << " " << ratio1 << " " << ks2 << " " << ratio2 << " " << ks3 << " " << ratio3 << " " << ks4 << " " << ratio4 << std::endl;

  std::vector<double> results;
  results.push_back(eval_ks_ratio(ks1, ks2, ratio1, ratio2)); // direction metric
  
  results.push_back(sqrt(pow(ks1,2) + pow(ratio1-1,2))); // muon information
  //  results.push_back(ratio1);

  results.push_back(sqrt(pow(ks3,2) + pow(ratio3-1,2))); // proton information
  //results.push_back(ratio3);

  results.push_back(sqrt(pow(ks4,2) + pow(ratio4-1,2))); // electron information
  //  results.push_back(ratio4);
  
  return results;
  
}

bool WCPPID::ProtoSegment::eval_ks_ratio(double ks1, double ks2, double ratio1, double ratio2){
  //  std::cout << ks1 << " " << ks2 << " " << ratio1 << " " << ratio2 << " " << sqrt(pow(ks2/0.06,2)+pow((ratio2-1)/0.06,2)) << " " << ks1-ks2 + (fabs(ratio1-1)-fabs(ratio2-1))/1.5*0.3 << " " <<  ks1-ks2 + (fabs(ratio1-1)-fabs(ratio2-1))/1.5*0.3 << " " << std::endl;
  if (ks1-ks2 >= 0.0) return false;
  if (sqrt(pow(ks2/0.06,2)+pow((ratio2-1)/0.06,2))< 1.4 && ks1-ks2 + (fabs(ratio1-1)-fabs(ratio2-1))/1.5*0.3 > -0.02) return false;

  if (ks1 - ks2 < -0.02 && (ks2 > 0.09 && fabs(ratio2-1) >0.1 || ratio2 > 1.5 || ks2 > 0.2)) return true;
  if ( ks1-ks2 + (fabs(ratio1-1)-fabs(ratio2-1))/1.5*0.3 < 0) return true;

  return false;
 }


bool WCPPID::ProtoSegment::do_track_pid(std::vector<double>& L , std::vector<double>& dQ_dx, double compare_range , double offset_length, bool flag_force ){
  std::vector<double> rL(L.size(),0);
  std::vector<double> rdQ_dx(L.size(),0);
  // get reverse  vectors ...
  for (size_t i=0;i!=L.size();i++){
    rL.at(i) = L.back() - L.at(L.size()-1-i);
    rdQ_dx.at(i) = dQ_dx.at(L.size()-1-i);
  }

  std::vector<double> result_forward = do_track_comp(L, dQ_dx, compare_range, offset_length);
  std::vector<double> result_backward = do_track_comp(rL, rdQ_dx, compare_range, offset_length);

  // direction determination 
  bool flag_forward = std::round(result_forward.at(0));
  bool flag_backward = std::round(result_backward.at(0));

  double length = get_length();
  double length1 = sqrt(pow(fit_pt_vec.front().x - fit_pt_vec.back().x,2) + pow(fit_pt_vec.front().y - fit_pt_vec.back().y,2) + pow(fit_pt_vec.front().z - fit_pt_vec.back().z,2));
  
  int forward_particle_type = 13; // default muon
  double min_forward_val = result_forward.at(1) ;
  if (result_forward.at(2) < min_forward_val){
    min_forward_val = result_forward.at(2);
    forward_particle_type = 2212;
  }
  
  if (result_forward.at(3) < min_forward_val && length < 20*units::cm){
    min_forward_val = result_forward.at(3);
    forward_particle_type = 11;
  }

  //  std::cout << length/units::cm << " " << length1/units::cm << std::endl;
  
  int backward_particle_type  =13; // default muon
  double min_backward_val = result_backward.at(1);
  if (result_backward.at(2) < min_backward_val){
    min_backward_val = result_backward.at(2);
    backward_particle_type = 2212;
  }
  if (result_backward.at(3) < min_backward_val && length < 20*units::cm){
    min_backward_val = result_backward.at(3);
    backward_particle_type = 11;
  }

  // std::cout << id << " " << get_length()/units::cm << " " << flag_forward << " " << result_forward.at(0) << " m: " << result_forward.at(1)  << " p: " << result_forward.at(2) << " e: " << result_forward.at(3)  << std::endl;
  // std::cout << id << " " << get_length()/units::cm << " " << flag_backward << " " << result_backward.at(0) << " m: " << result_backward.at(1)  << " p: " << result_backward.at(2)  << " e: " << result_backward.at(3) <<  std::endl;

  
  if (flag_forward == 1 && flag_backward == 0){
    flag_dir = 1;
    particle_type = forward_particle_type;
    particle_score = min_forward_val;
    return true;
  }else if (flag_forward == 0 && flag_backward == 1){
    flag_dir = -1;
    particle_type = backward_particle_type;
    particle_score = min_backward_val;
    return true;
  }else if (flag_forward == 1 && flag_backward == 1){
    if (min_forward_val < min_backward_val){
      flag_dir = 1;
      particle_type = forward_particle_type;
      particle_score = min_forward_val;
    }else{
      flag_dir = -1;
      particle_type = backward_particle_type;
      particle_score = min_backward_val;
    }
    return true;
  }else if (flag_forward ==0 && flag_backward == 0 && flag_force){
    if (min_forward_val < min_backward_val){
      particle_score = min_forward_val;
      particle_type = forward_particle_type;
      flag_dir = 1;
    }else{
      particle_score = min_backward_val;
      particle_type = backward_particle_type;
      flag_dir = -1;
    }
    return true;
  }

  // reset before return ...
  flag_dir = 0;
  particle_type  = 0;
  particle_score = 0;
  
  return false;
}

bool WCPPID::ProtoSegment::is_dir_weak(){
  double length = get_length();
  
  
  if (fabs(particle_type)==13 && particle_score > 0.07 && length >= 5*units::cm
      || fabs(particle_type)==13 &&  particle_score > 0.15 && length < 5*units::cm
      || fabs(particle_type)==2212 && particle_score > 0.13 && length >=5*units::cm
      || fabs(particle_type)==2212 && particle_score > 0.27 && length < 5*units::cm
      || dir_weak ) return true;

  
  return false;
}

bool WCPPID::ProtoSegment::get_flag_shower(){
  return flag_shower_trajectory || flag_shower_topology || get_flag_shower_dQdx();
}

bool WCPPID::ProtoSegment::get_flag_shower_dQdx(){
  if (fabs(particle_type)==11) return true;
  return false;
}



double WCPPID::ProtoSegment::cal_kine_dQdx(std::vector<double>& vec_dQ, std::vector<double>& vec_dx){
  double kine_energy =0;
  Double_t alpha = 1.0;
  Double_t beta = 0.255;
  
  for (size_t i=0;i!=vec_dQ.size();i++){
    Double_t dQdx = vec_dQ.at(i)/(vec_dx.at(i) + 1e-9) * units::cm;
    if (dQdx/43e3 > 1000) dQdx = 0;

    
    //std::cout << id << " " << i << " " << dQdx/43e3 << std::endl;
    
    Double_t dEdx = (exp(dQdx * 23.6e-6*beta/1.38/0.273) - alpha)/(beta/1.38/0.273) * units::MeV/units::cm;

    if (dEdx < 0) dEdx = 0;
    if (dEdx > 50*units::MeV/units::cm ) dEdx = 50*units::MeV/units::cm;
    
    kine_energy += dEdx * vec_dx.at(i);

    // std::cout << vec_dQ.at(i) << " " << vec_dx.at(i) << " " << dEdx * vec_dx.at(i) << std::endl;
  }
  
  return kine_energy;
}

double WCPPID::ProtoSegment::cal_kine_dQdx(){
  double kine_energy =0;
  Double_t alpha = 1.0;
  Double_t beta = 0.255;
  for (size_t i=0;i!=fit_pt_vec.size();i++){
    Double_t dQdx = dQ_vec.at(i)/(dx_vec.at(i) + 1e-9) * units::cm;
    if (dQdx/43e3 > 1000) dQdx = 0;


    
    Double_t dEdx = (exp(dQdx * 23.6e-6*beta/1.38/0.273) - alpha)/(beta/1.38/0.273) * units::MeV/units::cm;

    if (dEdx < 0) dEdx = 0;
    if (dEdx > 50*units::MeV/units::cm ) dEdx = 50*units::MeV/units::cm;
    
    //  std::cout << id << " " << i << " " << dQdx/(43e3) << " " << dEdx/(units::MeV/units::cm) << " " << dx_vec.at(i)/units::cm << " " << kine_energy << std::endl;
    
    if (i==0){
      double dis = sqrt(pow(fit_pt_vec.at(1).x - fit_pt_vec.at(0).x,2) + pow(fit_pt_vec.at(1).y - fit_pt_vec.at(0).y,2) + pow(fit_pt_vec.at(1).z - fit_pt_vec.at(0).z,2));
      if (dx_vec.at(i) > dis*1.5){
	kine_energy += dEdx * dis ;
      }else{
	kine_energy += dEdx * dx_vec.at(i);
      }
    }else if (i+1 == fit_pt_vec.size()){
      double dis = sqrt(pow(fit_pt_vec.at(i).x - fit_pt_vec.at(i-1).x,2) + pow(fit_pt_vec.at(i).y - fit_pt_vec.at(i-1).y,2) + pow(fit_pt_vec.at(i).z - fit_pt_vec.at(i-1).z,2));
      if (dx_vec.at(i) > dis*1.5){
	kine_energy += dEdx * dis ;
      }else{
	kine_energy += dEdx * dx_vec.at(i);
      }
    }else{
      kine_energy += dEdx * dx_vec.at(i);
    }
  }
  
  return kine_energy;
}

double WCPPID::ProtoSegment::cal_kine_range(double L){
  TPCParams& mp = Singleton<TPCParams>::Instance();
  TGraph *g_range = 0;
  if (fabs(particle_type)==11)    g_range = mp.get_electron_r2ke();
  else if (fabs(particle_type)==13) g_range = mp.get_muon_r2ke();
  else if (fabs(particle_type)==211) g_range = mp.get_pion_r2ke();
  else if (fabs(particle_type)==321) g_range = mp.get_kaon_r2ke();
  else if (fabs(particle_type)==2212) g_range = mp.get_proton_r2ke();

  double kine_energy = g_range->Eval(L/units::cm) * units::MeV;
  return kine_energy;
}

double WCPPID::ProtoSegment::cal_kine_range(){
  TPCParams& mp = Singleton<TPCParams>::Instance();
  TGraph *g_range = mp.get_muon_r2ke();
  if (fabs(particle_type)==11)    g_range = mp.get_electron_r2ke();
  else if (fabs(particle_type)==13) g_range = mp.get_muon_r2ke();
  else if (fabs(particle_type)==211) g_range = mp.get_pion_r2ke();
  else if (fabs(particle_type)==321) g_range = mp.get_kaon_r2ke();
  else if (fabs(particle_type)==2212) g_range = mp.get_proton_r2ke();

  //  std::cout << g_range << std::endl;
  
  double kine_energy = g_range->Eval(get_length()/units::cm) * units::MeV;
  return kine_energy;
}

double WCPPID::ProtoSegment::cal_kine_range(int pdg){
  TPCParams& mp = Singleton<TPCParams>::Instance();
  TGraph *g_range = mp.get_muon_r2ke();
  if (fabs(pdg)==11)    g_range = mp.get_electron_r2ke();
  else if (fabs(pdg)==13) g_range = mp.get_muon_r2ke();
  else if (fabs(pdg)==211) g_range = mp.get_pion_r2ke();
  else if (fabs(pdg)==321) g_range = mp.get_kaon_r2ke();
  else if (fabs(pdg)==2212) g_range = mp.get_proton_r2ke();
  double kine_energy = g_range->Eval(get_length()/units::cm) * units::MeV;
  return kine_energy;
}

void WCPPID::ProtoSegment::cal_4mom(){
 
  double length = get_length();
  double kine_energy = 0;

  if (length < 4*units::cm){
    kine_energy = cal_kine_dQdx(); // short track 
  }else if (flag_shower_trajectory){
    kine_energy = cal_kine_dQdx();
  }else{
    kine_energy = cal_kine_range();
  }

  kenergy_best = kine_energy;

  
  //std::cout << kine_energy << std::endl;
  particle_4mom[3]= kine_energy + particle_mass;
  double mom = sqrt(pow(particle_4mom[3],2) - pow(particle_mass,2));
  //  direction vector ...

  TVector3 v1 = cal_dir_3vector();
  particle_4mom[0] = mom * v1.X();
  particle_4mom[1] = mom * v1.Y();
  particle_4mom[2] = mom * v1.Z();

  // std::cout << id << " " << cal_kine_dQdx() << " " << cal_kine_range() << " " << particle_mass << " " << particle_type << " " << kine_energy << " " << mom << " " << v1.X() << " " << v1.Y() << " " << v1.Z() << std::endl;

}

TVector3 WCPPID::ProtoSegment::cal_dir_3vector(WCP::Point& p, double dis_cut){
  WCP::Point p1(0,0,0);
  Int_t ncount = 0;
  for (size_t i=0;i!=fit_pt_vec.size();i++){
    double dis = sqrt(pow(fit_pt_vec.at(i).x - p.x,2)+pow(fit_pt_vec.at(i).y - p.y,2)+pow(fit_pt_vec.at(i).z - p.z,2));
    //    std::cout << dis/units::cm << std::endl;
    if (dis < dis_cut){
      p1.x += fit_pt_vec.at(i).x;
      p1.y += fit_pt_vec.at(i).y;
      p1.z += fit_pt_vec.at(i).z;
      ncount ++;
    }
  }
  
  TVector3 v1(p1.x/ncount - p.x, p1.y/ncount - p.y, p1.z/ncount - p.z);
  v1 = v1.Unit();
  return v1;
}

TVector3 WCPPID::ProtoSegment::cal_dir_3vector(){
  WCP::Point p(0,0,0);  
  if (flag_dir == 1){
    for (size_t i=1;i<5;i++){
      if (i>= fit_pt_vec.size()) break;
      p.x += fit_pt_vec.at(i).x - fit_pt_vec.at(0).x;
      p.y += fit_pt_vec.at(i).y - fit_pt_vec.at(0).y;
      p.z += fit_pt_vec.at(i).z - fit_pt_vec.at(0).z;
    }
  }else if (flag_dir == -1){
    for (size_t i=1;i<5;i++){
      if (i+1 > fit_pt_vec.size()) break;
      p.x += fit_pt_vec.at(fit_pt_vec.size()-i-1).x - fit_pt_vec.back().x;
      p.y += fit_pt_vec.at(fit_pt_vec.size()-i-1).y - fit_pt_vec.back().y;
      p.z += fit_pt_vec.at(fit_pt_vec.size()-i-1).z - fit_pt_vec.back().z;
    }
  }
  TVector3 v1(p.x, p.y, p.z);
  v1 = v1.Unit();
  return v1;
}

TVector3 WCPPID::ProtoSegment::cal_dir_3vector(int direction, int num_points, int start){
  WCP::Point p(0,0,0); 
  if (start>= fit_pt_vec.size() || start<=0){ std::cout<<"bad start point in TVector3 WCPPID::ProtoSegment::cal_dir_3vector"<<std::endl; TVector3 v1(p.x, p.y, p.z);  return v1;} 
  if (direction == 1){
    for (size_t i=start;i<start+num_points-1;i++){
      if (i>= fit_pt_vec.size()) break;
      p.x += fit_pt_vec.at(i).x - fit_pt_vec.at(start-1).x;
      p.y += fit_pt_vec.at(i).y - fit_pt_vec.at(start-1).y;
      p.z += fit_pt_vec.at(i).z - fit_pt_vec.at(start-1).z;
    }
  }else if (direction == -1){
    for (size_t i=start;i<start+num_points-1;i++){
      if (i+1 > fit_pt_vec.size()) break;
      p.x += fit_pt_vec.at(fit_pt_vec.size()-i-1).x - fit_pt_vec.at(fit_pt_vec.size()-start).x;
      p.y += fit_pt_vec.at(fit_pt_vec.size()-i-1).y - fit_pt_vec.at(fit_pt_vec.size()-start).y;
      p.z += fit_pt_vec.at(fit_pt_vec.size()-i-1).z - fit_pt_vec.at(fit_pt_vec.size()-start).z;
    }
  }
  TVector3 v1(p.x, p.y, p.z);
  v1 = v1.Unit();
  return v1;
}



void WCPPID::ProtoSegment::determine_dir_track(int start_n, int end_n, bool flag_print){
  flag_dir = 0;
  int npoints = fit_pt_vec.size();
  int start_n1 = 0, end_n1 = npoints - 1;
  // if more than one point, exclude the vertex ...
  if (end_n!=1) {
    end_n1 = npoints - 2;
    npoints -= 1;
  }
  if (start_n!=1) {
    npoints -=1;
    start_n1 = 1;
  }
  // if (start_n == 1 && end_n == 1){
  //   end_n1 = npoints - 2;
  //   npoints -= 1;
    
  //   npoints -=1;
  //   start_n1 = 1;
  // }
  
  // if (id == 1) std::cout << id << " " << npoints << " " << end_n1 << " " << start_n1 << std::endl;
  
  if (npoints ==0 || end_n1 < start_n1) return;
  std::vector<double> L(npoints,0);
  std::vector<double> dQ_dx(npoints,0);
  
  double dis = 0;
  //  std::cout << start_n1 << " " << end_n1 << " " << dQ_vec.size() << " " << fit_pt_vec.size() << std::endl;
  for (size_t i = start_n1; i <= end_n1; i++){
    L.at(i-start_n1) = dis;
    dQ_dx.at(i-start_n1) = dQ_vec.at(i)/(dx_vec.at(i)/units::cm+1e-9);
    if (i+1 < fit_pt_vec.size())
      dis += sqrt(pow(fit_pt_vec.at(i+1).x-fit_pt_vec.at(i).x,2) + pow(fit_pt_vec.at(i+1).y-fit_pt_vec.at(i).y,2) + pow(fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z,2));
  }
    
  if (npoints >=2){ // reasonably long ...
     if (start_n == 1 && end_n == 1 &&  npoints >=15){
      // can use the dQ/dx to do PID and direction ...
       bool tmp_flag_pid = do_track_pid(L, dQ_dx, 35*units::cm, 1*units::cm, true);
       if (!tmp_flag_pid) tmp_flag_pid =do_track_pid(L, dQ_dx, 15*units::cm, 1*units::cm, true);
     }else{
       // can use the dQ/dx to do PID and direction ...
       bool tmp_flag_pid = do_track_pid(L, dQ_dx);
       if (!tmp_flag_pid) tmp_flag_pid =do_track_pid(L, dQ_dx, 15*units::cm);
       // if (!tmp_flag_pid) tmp_flag_pid = do_track_pid(L, dQ_dx, 35*units::cm, 3*units::cm);
       // if (!tmp_flag_pid) tmp_flag_pid =do_track_pid(L, dQ_dx, 15*units::cm, 3*units::cm);
     }
  }

 
  double length = get_length();

  // not a good idea to constraing dQ/dx for electron
  
  // short track what to do???
  if (particle_type == 0){
    // calculate medium dQ/dx
    std::vector<double> vec_dQ_dx = dQ_dx;
    std::nth_element(vec_dQ_dx.begin(), vec_dQ_dx.begin() + vec_dQ_dx.size()/2, vec_dQ_dx.end());
    double medium_dQ_dx = *std::next(vec_dQ_dx.begin(), vec_dQ_dx.size()/2);
    if (medium_dQ_dx > 43e3 * 1.75) particle_type = 2212; // proton
    else if (medium_dQ_dx < 43e3 * 1.2) particle_type = 13; //muon type
    else if (medium_dQ_dx < 43e3 * 1.5 && length < 4*units::cm) particle_type = 13;
    //    std::cout << medium_dQ_dx/(43e3) << std::endl;
  }

  // electron and both end contain stuff
  if (abs(particle_type)==11 && (start_n>1 && end_n >1)){
    dir_weak = true;
    flag_dir = 0;
    if (particle_score < 0.15) particle_type = 13;
  }else if (abs(particle_type)==11 && (start_n >1 && flag_dir == -1 || end_n >1 && flag_dir == 1) ){
    dir_weak = true;
    flag_dir = 0;
    if (particle_score < 0.15) particle_type = 13;
  }else if (length < 1.5*units::cm){
    dir_weak = true;
  }

  
  
  // vertex activities ...
  if (length < 1.5*units::cm && (start_n == 1 || end_n == 1)){
    if (start_n ==1 && end_n > 2){
      flag_dir = -1;
      std::vector<double> vec_dQ_dx = dQ_dx;
      std::nth_element(vec_dQ_dx.begin(), vec_dQ_dx.begin() + vec_dQ_dx.size()/2, vec_dQ_dx.end());
      double medium_dQ_dx = *std::next(vec_dQ_dx.begin(), vec_dQ_dx.size()/2);
      if (medium_dQ_dx > 43e3 * 1.75) particle_type = 2212;
      else if (medium_dQ_dx < 43e3*1.2) particle_type = 211;
    }else if (end_n==1 && start_n>2){
      flag_dir = 1;
      std::vector<double> vec_dQ_dx = dQ_dx;
      std::nth_element(vec_dQ_dx.begin(), vec_dQ_dx.begin() + vec_dQ_dx.size()/2, vec_dQ_dx.end());
      double medium_dQ_dx = *std::next(vec_dQ_dx.begin(), vec_dQ_dx.size()/2);
      if (medium_dQ_dx > 43e3 * 1.75) particle_type = 2212;
      else if (medium_dQ_dx < 43e3*1.2) particle_type = 211;
    }
  }


  // if the particle score is really bad, make it a shower ...
  if (length > 10*units::cm && particle_score > 1.0 && particle_score < 100){
    particle_type = 11;
    particle_score = 200;
    flag_dir = 0;
  }
 
  
  if (particle_type!=0){
    TPCParams& mp = Singleton<TPCParams>::Instance();
    if (fabs(particle_type)==11)    particle_mass = mp.get_mass_electron();
    else if (fabs(particle_type)==13) particle_mass = mp.get_mass_muon();
    else if (fabs(particle_type)==211) particle_mass = mp.get_mass_pion();
    else if (fabs(particle_type)==321) particle_mass = mp.get_mass_kaon();
    else if (fabs(particle_type)==2212) particle_mass = mp.get_mass_proton();
  }

  

  if ((flag_dir==1 && end_n == 1 || flag_dir==-1 && start_n ==1) && particle_type!=0){
    cal_4mom();
  }

  if (flag_print)
    std::cout << id << " " << length/units::cm << " Track " << flag_dir << " " << is_dir_weak() <<  " " << particle_type << " " << particle_mass/units::MeV << " " << (particle_4mom[3]-particle_mass)/units::MeV << " " << particle_score << std::endl;
  
  
}

void WCPPID::ProtoSegment::determine_dir_shower_trajectory(int start_n, int end_n, bool flag_print){
  flag_dir = 0;
  double length = get_length();
 
  // hack for now ...
  particle_type = 11;
  TPCParams& mp = Singleton<TPCParams>::Instance();

  
  if (start_n==1 && end_n >1){
    flag_dir = -1;
  }else if (start_n > 1 && end_n == 1){
    flag_dir = 1;
  }else{
    determine_dir_track(start_n, end_n, false);
    if (particle_type!=11){
      particle_type = 11;
      flag_dir = 0;
    }
  }
  
  particle_mass = mp.get_mass_electron();
  
  cal_4mom();

  if (flag_print)
    std::cout << id << " " << length/units::cm << " S_traj " << flag_dir << " " << is_dir_weak() <<  " " << particle_type << " " << particle_mass/units::MeV << " " << (particle_4mom[3]-particle_mass)/units::MeV << " " << particle_score << std::endl;

}

void WCPPID::ProtoSegment::determine_dir_shower_topology(int start_n, int end_n, bool flag_print){
  
  double length = get_length();
  // hack for now
  particle_type = 11;
  TPCParams& mp = Singleton<TPCParams>::Instance();
  particle_mass = mp.get_mass_electron();
  
  //if (start_n==1 && end_n >1){
  //  flag_dir = -1;
  //}else if (start_n > 1 && end_n == 1){
  //  flag_dir = 1;
  //}
  // WCP::Point center(0,0,0);
  // WCP::WCPointCloud<double>& cloud = pcloud_associated->get_cloud();
  // for (size_t i=0;i!=cloud.pts.size();i++){
  //   center.x += cloud.pts.at(i).x;
  //   center.y += cloud.pts.at(i).y;
  //   center.z += cloud.pts.at(i).z;
  // }
  // center.x /= cloud.pts.size();
  // center.y /= cloud.pts.size();
  // center.z /= cloud.pts.size();

  // double dis1 = sqrt(pow(center.x - fit_pt_vec.front().x ,2) + pow(center.y - fit_pt_vec.front().y,2) + pow(center.z - fit_pt_vec.front().z,2));
  // double dis2 = sqrt(pow(center.x - fit_pt_vec.back().x ,2) + pow(center.y - fit_pt_vec.back().y,2) + pow(center.z - fit_pt_vec.back().z,2));

  // std::cout << dis1/units::cm << " " << dis2/units::cm << std::endl;

  if (flag_print)
    std::cout << id << " " << length/units::cm << " S_topo " << flag_dir << " " << is_dir_weak() <<  " " << particle_type << " " << particle_mass/units::MeV << " " << (particle_4mom[3]-particle_mass)/units::MeV << " " << particle_score << std::endl;
}


std::tuple<WCPPID::ProtoSegment*, WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> WCPPID::ProtoSegment::break_segment_at_point(WCP::Point& p, int& acc_segment_id, int& acc_vertex_id){

  int nbreak = -1;
  double min_dis = 1e9;
  for (size_t i=0;i!=wcpt_vec.size(); i++){
    double dis = sqrt(pow(wcpt_vec.at(i).x-p.x,2) + pow(wcpt_vec.at(i).y-p.y,2) + pow(wcpt_vec.at(i).z-p.z,2));
    if (dis < min_dis){
      min_dis = dis;
      nbreak = i;
    }
  }
  
  int nbreak_fit = -1;
  min_dis = 1e9;
  for (size_t i=0; i!=fit_pt_vec.size(); i++){
    double dis = sqrt(pow(fit_pt_vec.at(i).x-p.x,2) + pow(fit_pt_vec.at(i).y-p.y,2) + pow(fit_pt_vec.at(i).z-p.z,2));
    if (dis < min_dis){
      min_dis = dis;
      nbreak_fit = i;
    }
  }

 
  if (nbreak_fit == 0 || nbreak_fit+1 == fit_pt_vec.size()){
    return std::make_tuple(this, (WCPPID::ProtoVertex*)0, (WCPPID::ProtoSegment*)0);
  }else{
    if (nbreak ==0 ) nbreak ++;
    if (nbreak+1==wcpt_vec.size()) nbreak --;
  }


 
  
  // Now start to construct new segment ...
  std::list<WCP::WCPointCloud<double>::WCPoint > path_wcps1;
  std::list<WCP::WCPointCloud<double>::WCPoint > path_wcps2;
  WCP::WCPointCloud<double>::WCPoint vertex_wcp;
  for (size_t i=0; i!=wcpt_vec.size(); i++){
    if (i==nbreak){
      path_wcps1.push_back(wcpt_vec.at(i));
      path_wcps2.push_back(wcpt_vec.at(i));
      vertex_wcp = wcpt_vec.at(i);
    }else if ( i< nbreak){
      path_wcps1.push_back(wcpt_vec.at(i));
    }else{
      path_wcps2.push_back(wcpt_vec.at(i));
    }
  }
  // create new segment vertex ...
  WCPPID::ProtoSegment *sg1 = new WCPPID::ProtoSegment(acc_segment_id, path_wcps1, cluster_id); acc_segment_id ++;
  WCPPID::ProtoVertex *vtx = new WCPPID::ProtoVertex(acc_vertex_id, vertex_wcp, cluster_id); acc_vertex_id ++;
  WCPPID::ProtoSegment *sg2 = new WCPPID::ProtoSegment(acc_segment_id, path_wcps2, cluster_id); acc_segment_id ++;

  if (fit_index_vec.size()==0) fit_index_vec.resize(dQ_vec.size(),0);
  if (fit_flag_skip.size()==0) fit_flag_skip.resize(dQ_vec.size(),false);
  // std::cout << cluster_id << " " << nbreak_fit << " " << fit_pt_vec.size() << " " << dQ_vec.size() << " " << fit_index_vec.size() << " " << pu_vec.size() << " " << reduced_chi2_vec.size() << " " << nbreak << " " << wcpt_vec.size() << std::endl;

  
  // fill in the vertex
  vtx->set_neutrino_vertex(false);
  vtx->set_fit_pt(fit_pt_vec.at(nbreak_fit) );
  vtx->set_fit_index(fit_index_vec.at(nbreak_fit) );
  vtx->set_flag_fit_fix(false);
  vtx->set_fit_range(1*units::cm);
  vtx->set_dx(dx_vec.at(nbreak_fit) );
  vtx->set_fit_flag(true);
  vtx->set_dQ(dQ_vec.at(nbreak_fit));
  vtx->set_pu(pu_vec.at(nbreak_fit));
  vtx->set_pv(pv_vec.at(nbreak_fit));
  vtx->set_pw(pw_vec.at(nbreak_fit));
  vtx->set_pt(pt_vec.at(nbreak_fit));
  vtx->set_reduced_chi2(reduced_chi2_vec.at(nbreak_fit));

  // std::cout << cluster_id << " " << nbreak_fit << " " << fit_pt_vec.size() << " " << dQ_vec.size() << " " << fit_index_vec.size() << " " << pu_vec.size() << " " << reduced_chi2_vec.size() << " " << nbreak << " " << wcpt_vec.size() << std::endl;
  
  // fill in first and segments
  sg1->set_dir_weak(dir_weak); sg2->set_dir_weak(dir_weak);
  sg1->set_fit_flag(flag_fit); sg2->set_fit_flag(flag_fit);
  sg1->set_flag_dir(flag_dir); sg2->set_flag_dir(flag_dir);
  sg1->set_particle_type(particle_type); sg2->set_particle_type(particle_type);
  sg1->set_particle_mass(particle_mass); sg2->set_particle_mass(particle_mass);
  sg1->set_flag_shower_trajectory(flag_shower_trajectory); sg2->set_flag_shower_trajectory(flag_shower_trajectory);
  sg1->set_flag_shower_topology(flag_shower_topology); sg2->set_flag_shower_topology(flag_shower_topology);
  sg1->set_particle_score(particle_score); sg2->set_particle_score(particle_score);
  
  
  std::vector<WCP::Point >& fit_pt_vec1 = sg1->get_point_vec();
  std::vector<WCP::Point >& fit_pt_vec2 = sg2->get_point_vec();
  std::vector<double>& dQ_vec1 = sg1->get_dQ_vec();
  std::vector<double>& dQ_vec2 = sg2->get_dQ_vec();
  std::vector<double>& dx_vec1 = sg1->get_dx_vec();
  std::vector<double>& dx_vec2 = sg2->get_dx_vec();
  std::vector<double>& dQ_dx_vec1 = sg1->get_dQ_dx_vec();
  std::vector<double>& dQ_dx_vec2 = sg2->get_dQ_dx_vec();
  std::vector<double>& pu_vec1 = sg1->get_pu_vec();
  std::vector<double>& pu_vec2 = sg2->get_pu_vec();
  std::vector<double>& pv_vec1 = sg1->get_pv_vec();
  std::vector<double>& pv_vec2 = sg2->get_pv_vec();
  std::vector<double>& pw_vec1 = sg1->get_pw_vec();
  std::vector<double>& pw_vec2 = sg2->get_pw_vec();
  std::vector<double>& pt_vec1 = sg1->get_pt_vec();
  std::vector<double>& pt_vec2 = sg2->get_pt_vec();
  std::vector<double>& reduced_chi2_vec1 = sg1->get_reduced_chi2_vec();
  std::vector<double>& reduced_chi2_vec2 = sg2->get_reduced_chi2_vec();
  std::vector<int>& fit_index_vec1 = sg1->get_fit_index_vec();
  std::vector<int>& fit_index_vec2 = sg2->get_fit_index_vec();
  std::vector<bool>& fit_flag_skip1 = sg1->get_fit_flag_skip(); // vertex???
  std::vector<bool>& fit_flag_skip2 = sg2->get_fit_flag_skip(); // vertex???

  fit_pt_vec1.clear(); fit_pt_vec2.clear();
  dQ_vec1.clear(); dQ_vec2.clear();
  pu_vec1.clear(); pu_vec2.clear();
  pv_vec1.clear(); pv_vec2.clear();
  pw_vec1.clear(); pw_vec2.clear();
  pt_vec1.clear(); pt_vec2.clear();
  reduced_chi2_vec1.clear(); reduced_chi2_vec2.clear();
  fit_index_vec1.clear(); fit_index_vec2.clear();
  fit_flag_skip1.clear(); fit_flag_skip2.clear();

  //  std::cout << fit_pt_vec.size() << " " << dQ_vec.size() << " " << dx_vec.size() << " " << pu_vec.size() << " " << pv_vec.size() << " " << pw_vec.size() << " " << pt_vec.size() << " " << reduced_chi2_vec.size() << " " << fit_index_vec.size() << " " << fit_flag_skip.size() << std::endl;
  
  for (size_t i=0; i!=fit_pt_vec.size(); i++){
    if (i==nbreak_fit){
      fit_pt_vec1.push_back(fit_pt_vec.at(i)); fit_pt_vec2.push_back(fit_pt_vec.at(i));
      dQ_vec1.push_back(dQ_vec.at(i)); dQ_vec2.push_back(dQ_vec.at(i));
      dx_vec1.push_back(dx_vec.at(i)); dx_vec2.push_back(dx_vec.at(i));
      dQ_dx_vec1.push_back(dQ_dx_vec.at(i));  dQ_dx_vec2.push_back(dQ_dx_vec.at(i));
      pu_vec1.push_back(pu_vec.at(i)); pu_vec2.push_back(pu_vec.at(i));
      pv_vec1.push_back(pv_vec.at(i)); pv_vec2.push_back(pv_vec.at(i));
      pw_vec1.push_back(pw_vec.at(i)); pw_vec2.push_back(pw_vec.at(i));
      pt_vec1.push_back(pt_vec.at(i)); pt_vec2.push_back(pt_vec.at(i));
      reduced_chi2_vec1.push_back(reduced_chi2_vec.at(i)); reduced_chi2_vec2.push_back(reduced_chi2_vec.at(i));
      fit_index_vec1.push_back(fit_index_vec.at(i)); fit_index_vec2.push_back(fit_index_vec.at(i));
      fit_flag_skip1.push_back(fit_flag_skip.at(i)); fit_flag_skip2.push_back(fit_flag_skip.at(i));
    }else if (i< nbreak_fit){
      fit_pt_vec1.push_back(fit_pt_vec.at(i));
      dQ_vec1.push_back(dQ_vec.at(i));
      dx_vec1.push_back(dx_vec.at(i));
      dQ_dx_vec1.push_back(dQ_dx_vec.at(i));
      pu_vec1.push_back(pu_vec.at(i));
      pv_vec1.push_back(pv_vec.at(i)); 
      pw_vec1.push_back(pw_vec.at(i)); 
      pt_vec1.push_back(pt_vec.at(i));
      reduced_chi2_vec1.push_back(reduced_chi2_vec.at(i)); 
      fit_index_vec1.push_back(fit_index_vec.at(i)); 
      fit_flag_skip1.push_back(fit_flag_skip.at(i)); 
    }else{
      fit_pt_vec2.push_back(fit_pt_vec.at(i));
      dQ_vec2.push_back(dQ_vec.at(i));
      dx_vec2.push_back(dx_vec.at(i));
      dQ_dx_vec2.push_back(dQ_dx_vec.at(i));
      pu_vec2.push_back(pu_vec.at(i));
      pv_vec2.push_back(pv_vec.at(i));
      pw_vec2.push_back(pw_vec.at(i));
      pt_vec2.push_back(pt_vec.at(i));
      reduced_chi2_vec2.push_back(reduced_chi2_vec.at(i));
      fit_index_vec2.push_back(fit_index_vec.at(i));
      fit_flag_skip2.push_back(fit_flag_skip.at(i));
    }
  }

  // std::cout << nbreak_fit << " " << fit_pt_vec.size() << std::endl;
  //  std::cout << fit_pt_vec2.size() << " " << dQ_vec2.size() << " " << dx_vec2.size() << " " << pu_vec2.size() << " " << pv_vec2.size() << " " << pw_vec2.size() << " " << pt_vec2.size() << " " << reduced_chi2_vec2.size() << " " << fit_index_vec2.size() << " " << fit_flag_skip2.size() << std::endl;

  
  
  sg1->build_pcloud_fit();
  sg2->build_pcloud_fit();

  //  std::cout << fit_pt_vec2.size() << " " << dQ_vec2.size() << " " << dx_vec2.size() << " " << pu_vec2.size() << " " << pv_vec2.size() << " " << pw_vec2.size() << " " << pt_vec2.size() << " " << reduced_chi2_vec2.size() << " " << fit_index_vec2.size() << " " << fit_flag_skip2.size() << std::endl;

  sg1->cal_4mom();
  sg2->cal_4mom();

  WCP::WCPointCloud<double>& cloud = pcloud_associated->get_cloud();
  WCP::WC2DPointCloud<double>& cloud_u = pcloud_associated->get_cloud_u();
  WCP::WC2DPointCloud<double>& cloud_v = pcloud_associated->get_cloud_v();
  WCP::WC2DPointCloud<double>& cloud_w = pcloud_associated->get_cloud_w();
  Int_t ncount[2]={0,0};
  for (size_t i=0;i!=cloud.pts.size();i++){
    Point p(cloud.pts.at(i).x, cloud.pts.at(i).y, cloud.pts.at(i).z);
    if (sg1->get_closest_point(p).first < sg2->get_closest_point(p).first){
      sg1->add_associate_point(cloud.pts.at(i), cloud_u.pts.at(i), cloud_v.pts.at(i), cloud_w.pts.at(i));
      ncount[0]++;
    }else{
      sg2->add_associate_point(cloud.pts.at(i), cloud_u.pts.at(i), cloud_v.pts.at(i), cloud_w.pts.at(i));
      ncount[1]++;
    }
  }
  //  std::cout << ncount[0] << " " << ncount[1] << std::endl;
  if (ncount[0] !=0)
    sg1->get_associated_pcloud()->build_kdtree_index();
  if (ncount[1] !=0)
    sg2->get_associated_pcloud()->build_kdtree_index();

  
  //  std::cout << nbreak << " " << wcpt_vec.size() << " " << nbreak_fit << " " << fit_pt_vec.size() << std::endl;

  return std::make_tuple(sg1, vtx, sg2);
  
}
