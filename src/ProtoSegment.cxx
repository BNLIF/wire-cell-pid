#include "WCPPID/ProtoSegment.h"

using namespace WCP;

WCPPID::ProtoSegment::ProtoSegment(std::list<WCP::WCPointCloud<double>::WCPoint >& path_wcps )
  : flag_fit(false)
  , pcloud_fit(0)
{
  
  for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){
    wcpt_vec.push_back(*it);
  }
  
}

WCPPID::ProtoSegment::~ProtoSegment(){
  if (pcloud_fit != (ToyPointCloud*)0)
    delete pcloud_fit;
}
std::tuple<WCP::Point, TVector3, bool> WCPPID::ProtoSegment::search_kink(WCP::Point start_p){
  // find the first point ...
  std::pair<double, WCP::Point> tmp_results = get_closest_point(start_p);
  Point test_p = tmp_results.second;

  TVector3 drift_dir(1,0,0);

  std::vector<double> refl_angles(fit_pt_vec.size(),0);
  std::vector<double> para_angles(fit_pt_vec.size(),0);
  
  // start the angle search
 
  for (size_t i=0;i!=fit_pt_vec.size(); i++){
    double angle1 = 0;
    double angle2 = 0;
    
    for (int j=0;j!=6;j++){    
      TVector3 v10(0,0,0);
      TVector3 v20(0,0,0);
      if (i>j)
	v10.SetXYZ(fit_pt_vec.at(i).x - fit_pt_vec.at(i-j-1).x,
		   fit_pt_vec.at(i).y - fit_pt_vec.at(i-j-1).y,
		   fit_pt_vec.at(i).z - fit_pt_vec.at(i-j-1).z);
      
      if (i+j+1<fit_pt_vec.size())
	v20.SetXYZ(fit_pt_vec.at(i+j+1).x - fit_pt_vec.at(i).x,
		   fit_pt_vec.at(i+j+1).y - fit_pt_vec.at(i).y,
		   fit_pt_vec.at(i+j+1).z - fit_pt_vec.at(i).z);
      
      if (j==0){
	angle1 = v10.Angle(v20)/3.1415926*180.;
	angle2 = std::max(fabs(v10.Angle(drift_dir)/3.1415926*180.-90.),
			  fabs(v20.Angle(drift_dir)/3.1415926*180.-90.));
      }else{
	if (v10.Mag()!=0 && v20.Mag()!=0){
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
	     pow(fit_pt_vec.at(i).z - fit_pt_vec.back().z,2) ) < 1*units::cm
	) continue;
    
    if (flag_check){
      double min_dQ_dx = dQ_vec.at(i)/dx_vec.at(i);
      for (size_t j = 1;j!=6;j++){
	if (i+j<fit_pt_vec.size())
	  if (dQ_vec.at(i+j)/dx_vec.at(i+j) < min_dQ_dx)
	    min_dQ_dx = dQ_vec.at(i+j)/dx_vec.at(i+j);
      }
      
      double sum_angles = 0;
      double nsum = 0;
      for (int j = -2; j!=3;j++){
	if (i+j>=0 && i+j<fit_pt_vec.size()){
	  if (para_angles.at(i+j)>10){
	    sum_angles += pow(refl_angles.at(i+j),2);
	    nsum ++;
	  }
	}
      }
      if (nsum!=0) sum_angles=sqrt(sum_angles/nsum);

      //      std::cout << i << " " << min_dQ_dx << " " << para_angles.at(i) << " " << refl_angles.at(i) << " " << sum_angles << std::endl;
      
      if (min_dQ_dx < 1000 && para_angles.at(i) > 10 && refl_angles.at(i) > 25){
	save_i = i;
	break;
      }else if (para_angles.at(i) > 15 && refl_angles.at(i) > 27 && sum_angles > 12.5){
	
	save_i = i;
	break;
      }
    }
  }

  // return the results ...
  if (save_i>=0){
    Point p = fit_pt_vec.at(save_i);
    
    Point prev_p(0,0,0);
    int num_p = 0;
    for (size_t i=1;i!=10;i++){
      if (save_i>=i){
	prev_p.x += fit_pt_vec.at(save_i-i).x;
	prev_p.y += fit_pt_vec.at(save_i-i).y;
	prev_p.z += fit_pt_vec.at(save_i-i).z;
	num_p ++;
      }
    }
    prev_p.x /= num_p;
    prev_p.y /= num_p;
    prev_p.z /= num_p;
    
    TVector3 dir(p.x - prev_p.x, p.y - prev_p.y, p.z - prev_p.z);
    dir = dir.Unit();

    double sum_dQ = 0, sum_dx = 0;
    for (int i=-2;i!=3;i++){
      if (save_i+i>=0&&save_i+i<fit_pt_vec.size()){
	sum_dQ += dQ_vec.at(save_i+i);
	sum_dx += dx_vec.at(save_i+i);
      }
    }
    //    std::cout << sum_dQ/(sum_dx+1e-9) << std::endl;
    if (sum_dQ/(sum_dx+1e-9) > 2500){
      return std::make_tuple(p, dir, false);
    }else{
      return std::make_tuple(p, dir, true);
    }

  }else{

    Point p1 = fit_pt_vec.back();
    TVector3 dir(0,0,0);
    return std::make_tuple(p1,dir, false);

  }
  //  std::cout << save_i << std::endl;
  
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

void WCPPID::ProtoSegment::clear_fit(){
  flag_fit = false;
  fit_pt_vec.clear();
  dQ_vec.clear();
  dx_vec.clear();
  dQ_dx_vec.clear();

  pu_vec.clear();
  pv_vec.clear();
  pw_vec.clear();
  pt_vec.clear();
  reduced_chi2_vec.clear();
  
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