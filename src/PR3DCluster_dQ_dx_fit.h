#include <Eigen/SVD>

std::vector<std::pair<double, double> >  WCPPID::PR3DCluster::cal_compact_matrix(Eigen::SparseMatrix<double>& MW, Eigen::SparseMatrix<double>& RWT, int n_2D_w, int n_3D_pos, double cut_pos)
// W plane ...
{         
  // initial data ...
  std::vector<int> count_2D(n_2D_w,1);
  std::map<int, std::set<int> > map_2D_3D;
  std::map<int, std::set<int> > map_3D_2D;
  std::map<std::pair<int, int>, double> map_pair_val;    
  for (int k=0;k<RWT.outerSize(); ++k){
    int count = 0;
    
    for (Eigen::SparseMatrix<double>::InnerIterator it(RWT,k); it; ++it){

      // std::cout << "Row: " << it.row() << ", Col: " <<  it.col() << ", Value: " << it.value() << std::endl; 
      if (map_2D_3D.find(it.col()) != map_2D_3D.end()){
	      map_2D_3D[it.col()].insert(it.row());
      }else{
        std::set<int> temp_set;
        temp_set.insert(it.row());
        map_2D_3D[it.col()] = temp_set;
      }
      
      if (map_3D_2D.find(it.row())!=map_3D_2D.end()){
	      map_3D_2D[it.row()].insert(it.col());
      }else{
        std::set<int> temp_set;
        temp_set.insert(it.col());
        map_3D_2D[it.row()] = temp_set;
      }
      
      map_pair_val[std::make_pair(it.row(), it.col())] = it.value();
      count ++;
    }
    
    count_2D.at(k) = count;

    /* if (count > 2) */
    /*   std::cout << k << " " << count << std::endl; */
    //    if (count>2){
    // MW.coeffRef(k,k)=pow(1./(count-1.),2);
    // }
    
  }
  
  // figure out the 3D count
  std::vector<std::pair<double,int> > ave_count(n_3D_pos);
  for (auto it = map_3D_2D.begin(); it!=map_3D_2D.end(); it++){
    int row = it->first;
    double sum1 = 0;
    double sum2 = 0;
    int flag = 0;
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      int col = *it1;
      double val = map_pair_val[std::make_pair(row, col)];
      sum1 += count_2D[col] * val;
      sum2 += val;
      if (count_2D[col] > 2) flag = 1;
      // std::cout << row << " " << count_2D[col] << " " << val << std::endl;
    }
    // std::cout << row << " " << sum1 << " " << sum2 << " " << flag << std::endl;

    ave_count.at(row) = std::make_pair(sum1/sum2, flag);
  }
  
  
  // figure out the 2D count ..
  for (auto it = map_2D_3D.begin(); it!=map_2D_3D.end(); it++){
    int col = it->first;
    double sum1 = 0;
    double sum2 = 0;
    int flag = 0;
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      int row = *it1;
      double val = map_pair_val[std::make_pair(row, col)];
      if (ave_count.at(row).second==1) flag = 1;
      sum1 += ave_count.at(row).first * val;
      sum2 += val;
    }
    if (flag==1 && MW.coeffRef(col,col)==1 && sum1 > cut_pos*sum2){
      MW.coeffRef(col,col)=pow(1./(sum1/sum2-cut_pos+1),2);
      //std::cout << col << " " << sum1/sum2 << " " << flag << std::endl;
    }
  }


  std::vector<std::pair<double,double> > results(n_3D_pos, std::make_pair(0,0));
  // figure out the sharing between two nearby points
  for (auto it = map_3D_2D.begin(); it!=map_3D_2D.end(); it++){
    int row = it->first;
    auto it1 = map_3D_2D.find(row-1);
    auto it2 = map_3D_2D.find(row+1);

    double sum[3]={0,0,0};
    for (auto it3 = it->second.begin(); it3!=it->second.end(); it3++){
      int col = *it3;
      // std::cout << col << " " ;
      double val = map_pair_val[std::make_pair(row, col)];
      sum[0] += 1;//val;
    }
    // std::cout << std::endl;
    
    if (it1!=map_3D_2D.end()){
      std::vector<int> common_results(it->second.size());
      {
        auto it3 = std::set_intersection(it->second.begin(), it->second.end(), it1->second.begin(), it1->second.end(), common_results.begin());
        common_results.resize(it3-common_results.begin());
      }
      for (auto it3 = common_results.begin(); it3!=common_results.end(); it3++){
        int col = *it3;
        //	std::cout << col << " ";
        double val = map_pair_val[std::make_pair(row, col)];
        sum[1] += 1;//val;
      }
      // std::cout << std::endl;
    }
    
    if (it2!=map_3D_2D.end()){
      std::vector<int> common_results(it->second.size());
      {
        auto it3 = std::set_intersection(it->second.begin(), it->second.end(), it2->second.begin(), it2->second.end(), common_results.begin());
        common_results.resize(it3-common_results.begin());
      }
      for (auto it3 = common_results.begin(); it3!=common_results.end(); it3++){
        int col = *it3;
        double val = map_pair_val[std::make_pair(row, col)];
        sum[2] += 1;//val;
      }
    }

    // std::cout << row << " " << sum[0] << " " << sum[1] << " " << sum[2] << std::endl;


    results.at(row).first = sum[1]/sum[0];
    results.at(row).second = sum[2]/sum[0];
    //    std::cout << row << " " << sum[1]/sum[0] << " " << sum[2]/sum[0] << std::endl;
  }

  return results;
}

double WCPPID::PR3DCluster::cal_gaus_integral_seg(int tbin, int wbin, std::vector<double>& t_centers, std::vector<double>& t_sigmas, std::vector<double>& w_centers, std::vector<double>& w_sigmas, std::vector<double>& weights, int flag, double nsigma){
  double result = 0;
  double result1 = 0;

  for (size_t i=0;i!=t_centers.size();i++){
    result += cal_gaus_integral(tbin,wbin,t_centers.at(i), t_sigmas.at(i), w_centers.at(i), w_sigmas.at(i),flag,nsigma) * weights.at(i);
    result1 += weights.at(i);
  
    // std::cout << cal_gaus_integral(tbin,wbin,t_centers.at(i), t_sigmas.at(i), w_centers.at(i), w_sigmas.at(i),flag,nsigma) << " " << weights.at(i) << std::endl;
  }

  result /= result1;
  
  return result;
}


double WCPPID::PR3DCluster::cal_gaus_integral(int tbin, int wbin, double t_center, double t_sigma, double w_center, double w_sigma, int flag, double nsigma){
  // flag  = 0 no boundary effect, pure Gaussian, time or collection plane
  // flag  = 1 taking into account boundary effect for induction plane
  double result = 0;
  // time domain ...
  if (fabs((tbin + 0.5)-t_center) <= nsigma * t_sigma &&
      fabs((wbin + 0.5)-w_center) <= nsigma * w_sigma){
    // time dimension ...
    result = 0.5*(std::erf((tbin+1-t_center)/sqrt(2.)/t_sigma)-std::erf((tbin-t_center)/sqrt(2.)/t_sigma));

    if (flag ==0){
      result *= 0.5*(std::erf((wbin+1-w_center)/sqrt(2.)/w_sigma)-std::erf((wbin-w_center)/sqrt(2.)/w_sigma));

      // std::cout << tbin << " " << t_center << " " << t_sigma << " " << (tbin+1-t_center)/sqrt(2.)/t_sigma << " " << (tbin-t_center)/sqrt(2.)/t_sigma << " " << 0.5*(std::erf((tbin+1-t_center)/sqrt(2.)/t_sigma)-std::erf((tbin-t_center)/sqrt(2.)/t_sigma)) << " | " << 0.5*(std::erf((wbin+1-w_center)/sqrt(2.)/w_sigma)-std::erf((wbin-w_center)/sqrt(2.)/w_sigma)) << std::endl;
    }else if (flag==1){

      double x2 = wbin + 1.5;
      double x1 = wbin + 0.5;
      double x0 = w_center;
      double content1 = 0.5*(std::erf((x2-x0)/sqrt(2.)/w_sigma)-std::erf((x1-x0)/sqrt(2.)/w_sigma));
      double w1 = - pow(w_sigma,2)/(-1)/sqrt(2.*3.1415926)/w_sigma
      	*(exp(-pow(x0-x2,2)/2./pow(w_sigma,2))-exp(-pow(x0-x1,2)/2./pow(w_sigma,2)))
      	/(0.5*std::erf((x2-x0)/sqrt(2.)/w_sigma) - 0.5*std::erf((x1-x0)/sqrt(2.)/w_sigma))
      	+ (x0-x2)/(-1);
      //w1 = 0.5;
      // std::cout << w1 << std::endl;
      /* double w1 = (1./2./sqrt(2*3.1415926)/pow(x1-x2,2) * */
      /* 	    (2*exp(-pow(x0-x1,2)/2./pow(w_sigma,2)) * w_sigma *(x0+x1-2*x2) - 2 * exp(-pow(x0-x2,2)/2./pow(w_sigma,2))*w_sigma*(x0-x2)- */
      /* 	     sqrt(2*3.1415926)*(pow(w_sigma,2)+pow(x0-x2,2))*std::erf((x1-x0)/sqrt(2.)/w_sigma) + */
      /* 	     sqrt(2*3.1415926)*(pow(w_sigma,2)+pow(x0-x2,2))*std::erf((x2-x0)/sqrt(2.)/w_sigma)))/content1; */

      x2 = wbin + 0.5;
      x1 = wbin - 0.5;
      double content2 = 0.5*(std::erf((x2-x0)/sqrt(2.)/w_sigma)-std::erf((x1-x0)/sqrt(2.)/w_sigma));
      double w2 = - pow(w_sigma,2)/(-1)/sqrt(2.*3.1415926)/w_sigma
      	*(exp(-pow(x0-x2,2)/2./pow(w_sigma,2))-exp(-pow(x0-x1,2)/2./pow(w_sigma,2)))
      	/(0.5*std::erf((x2-x0)/sqrt(2.)/w_sigma) - 0.5*std::erf((x1-x0)/sqrt(2.)/w_sigma))
      	+ (x0-x2)/(-1);
      // std::cout << w2 << std::endl
      /* double w2 = (1./2./sqrt(2*3.1415926)/pow(x1-x2,2) * */
      /* 	    (2*exp(-pow(x0-x1,2)/2./pow(w_sigma,2)) * w_sigma *(x0+x1-2*x2) - 2 * exp(-pow(x0-x2,2)/2./pow(w_sigma,2))*w_sigma*(x0-x2)- */
      /* 	     sqrt(2*3.1415926)*(pow(w_sigma,2)+pow(x0-x2,2))*std::erf((x1-x0)/sqrt(2.)/w_sigma) + */
      /* 	     sqrt(2*3.1415926)*(pow(w_sigma,2)+pow(x0-x2,2))*std::erf((x2-x0)/sqrt(2.)/w_sigma)))/content2; */

      /* if (w1 > 0.5) { */
      /* 	w1 = sqrt(w1); */
      /* }else if (w1<0.5){ */
      /* 	w1 = 1-sqrt(1-w1); */
      /* } */
      /*  if (w2 > 0.5) { */
      /* 	w2 = sqrt(w1); */
      /*  }else if (w2 < 0.5){ */
      /* 	w2 = 1-sqrt(1-w2); */
      /* } */
      
      result *= (content1 * w1+content2 * (1-w2));
    }else if (flag==2){
      double sum = 0;
      
      double x2 = wbin + 1.0;
      double x1 = wbin + 0.5;
      double x0 = w_center;
      double content1 = 0.5*(std::erf((x2-x0)/sqrt(2.)/w_sigma)-std::erf((x1-x0)/sqrt(2.)/w_sigma));
      double w1 = - pow(w_sigma,2)/(-1)/sqrt(2.*3.1415926)/w_sigma
      	*(exp(-pow(x0-x2,2)/2./pow(w_sigma,2))-exp(-pow(x0-x1,2)/2./pow(w_sigma,2)))
      	/(0.5*std::erf((x2-x0)/sqrt(2.)/w_sigma) - 0.5*std::erf((x1-x0)/sqrt(2.)/w_sigma))
      	+ (x0-x2)/(-1);

      sum += content1 * (0.545 + 0.697*w1);

      x2 = wbin + 1.5;
      x1 = wbin + 1.0;
      content1 = 0.5*(std::erf((x2-x0)/sqrt(2.)/w_sigma)-std::erf((x1-x0)/sqrt(2.)/w_sigma));
      w1 = - pow(w_sigma,2)/(-1)/sqrt(2.*3.1415926)/w_sigma
      	*(exp(-pow(x0-x2,2)/2./pow(w_sigma,2))-exp(-pow(x0-x1,2)/2./pow(w_sigma,2)))
      	/(0.5*std::erf((x2-x0)/sqrt(2.)/w_sigma) - 0.5*std::erf((x1-x0)/sqrt(2.)/w_sigma))
      	+ (x0-x2)/(-1);

      sum+= content1 *(0.11364 + 0.1 * w1);

      x2 = wbin + 0.5;
      x1 = wbin;
      content1 = 0.5*(std::erf((x2-x0)/sqrt(2.)/w_sigma)-std::erf((x1-x0)/sqrt(2.)/w_sigma));
      w1 = - pow(w_sigma,2)/(-1)/sqrt(2.*3.1415926)/w_sigma
      	*(exp(-pow(x0-x2,2)/2./pow(w_sigma,2))-exp(-pow(x0-x1,2)/2./pow(w_sigma,2)))
      	/(0.5*std::erf((x2-x0)/sqrt(2.)/w_sigma) - 0.5*std::erf((x1-x0)/sqrt(2.)/w_sigma))
      	+ (x0-x2)/(-1);

      sum+= content1 *(0.545 + 0.697*(1-w1));

      x2 = wbin;
      x1 = wbin - 0.5;
      content1 = 0.5*(std::erf((x2-x0)/sqrt(2.)/w_sigma)-std::erf((x1-x0)/sqrt(2.)/w_sigma));
      w1 = - pow(w_sigma,2)/(-1)/sqrt(2.*3.1415926)/w_sigma
      	*(exp(-pow(x0-x2,2)/2./pow(w_sigma,2))-exp(-pow(x0-x1,2)/2./pow(w_sigma,2)))
      	/(0.5*std::erf((x2-x0)/sqrt(2.)/w_sigma) - 0.5*std::erf((x1-x0)/sqrt(2.)/w_sigma))
      	+ (x0-x2)/(-1);

      sum+= content1 *(0.11364 + 0.1 * (1-w1));
      
      result *= sum;
    }
  }
  return result;
}

void WCPPID::PR3DCluster::dQ_dx_fill(double dis_end_point_ext){
  if (fine_tracking_path.size()<=1) return;
  pu.resize(fine_tracking_path.size(),0);
  pv.resize(fine_tracking_path.size(),0);
  pw.resize(fine_tracking_path.size(),0);
  pt.resize(fine_tracking_path.size(),0);
  dQ.resize(fine_tracking_path.size(),0);
  dx.resize(fine_tracking_path.size(),0);
  reduced_chi2.resize(fine_tracking_path.size(),0);
  
  // Need to take into account the time, so one can properly calculate X value for diffusion ...
  TPCParams& mp = Singleton<TPCParams>::Instance();
  double time_slice_width = mp.get_ts_width();
  int time_offset = mp.get_time_offset();
  int nrebin = mp.get_nrebin();
  double pitch_u = mp.get_pitch_u();
  double pitch_v = mp.get_pitch_v();
  double pitch_w = mp.get_pitch_w();
  double first_u_dis = mp.get_first_u_dis();
  double first_v_dis = mp.get_first_v_dis();
  double first_w_dis = mp.get_first_w_dis();
  double angle_u = mp.get_angle_u();
  double angle_v = mp.get_angle_v();
  double angle_w = mp.get_angle_w();
  //  convert Z to W ... 
  double slope_zw = 1./pitch_w * cos(angle_w);
  double slope_yw = 1./pitch_w * sin(angle_w);
  
  double slope_yu = -1./pitch_u * sin(angle_u);
  double slope_zu = 1./pitch_u * cos(angle_u);
  double slope_yv = -1./pitch_v * sin(angle_v);
  double slope_zv = 1./pitch_v * cos(angle_v);
  //convert Y,Z to U,V
  double offset_w = -first_w_dis/pitch_w + 0.5;
  double offset_u = -first_u_dis/pitch_u + 0.5;
  double offset_v = -first_v_dis/pitch_v + 0.5;
  
  double first_t_dis = point_cloud->get_cloud().pts[0].mcell->GetTimeSlice()*time_slice_width - point_cloud->get_cloud().pts[0].x;
  double slope_xt = 1./time_slice_width;
  double offset_t =  first_t_dis/time_slice_width + 0.5;

  for (size_t i=0;i!=fine_tracking_path.size(); i++){
    Point curr_rec_pos = fine_tracking_path.at(i);
    Point prev_rec_pos, next_rec_pos;
    if (i==0){
      next_rec_pos.x = (fine_tracking_path.at(i).x+fine_tracking_path.at(i+1).x)/2.;
      next_rec_pos.y = (fine_tracking_path.at(i).y+fine_tracking_path.at(i+1).y)/2.;
      next_rec_pos.z = (fine_tracking_path.at(i).z+fine_tracking_path.at(i+1).z)/2.;
      double length = sqrt(pow(fine_tracking_path.at(i+1).x-fine_tracking_path.at(i).x,2)
			   +pow(fine_tracking_path.at(i+1).y-fine_tracking_path.at(i).y,2)
			   +pow(fine_tracking_path.at(i+1).z-fine_tracking_path.at(i).z,2) );
      
      if (length ==0){
	prev_rec_pos.x = fine_tracking_path.at(i).x;
	prev_rec_pos.y = fine_tracking_path.at(i).y;
	prev_rec_pos.z = fine_tracking_path.at(i).z;
      }else{
	prev_rec_pos.x = fine_tracking_path.at(i).x - (fine_tracking_path.at(i+1).x-fine_tracking_path.at(i).x)/length * dis_end_point_ext;
	prev_rec_pos.y = fine_tracking_path.at(i).y - (fine_tracking_path.at(i+1).y-fine_tracking_path.at(i).y)/length * dis_end_point_ext;
	prev_rec_pos.z = fine_tracking_path.at(i).z - (fine_tracking_path.at(i+1).z-fine_tracking_path.at(i).z)/length * dis_end_point_ext;
      }
    }else if (i+1==fine_tracking_path.size()){
      prev_rec_pos.x = (fine_tracking_path.at(i).x+fine_tracking_path.at(i-1).x)/2.;
      prev_rec_pos.y = (fine_tracking_path.at(i).y+fine_tracking_path.at(i-1).y)/2.;
      prev_rec_pos.z = (fine_tracking_path.at(i).z+fine_tracking_path.at(i-1).z)/2.;
      double length = sqrt(pow(fine_tracking_path.at(i-1).x-fine_tracking_path.at(i).x,2)
			   +pow(fine_tracking_path.at(i-1).y-fine_tracking_path.at(i).y,2)
			   +pow(fine_tracking_path.at(i-1).z-fine_tracking_path.at(i).z,2) );
      if (length ==0){
	next_rec_pos.x = fine_tracking_path.at(i).x;
	next_rec_pos.y = fine_tracking_path.at(i).y;
	next_rec_pos.z = fine_tracking_path.at(i).z;
      }else{
	next_rec_pos.x = fine_tracking_path.at(i).x - (fine_tracking_path.at(i-1).x-fine_tracking_path.at(i).x)/length * dis_end_point_ext;
	next_rec_pos.y = fine_tracking_path.at(i).y - (fine_tracking_path.at(i-1).y-fine_tracking_path.at(i).y)/length * dis_end_point_ext;
	next_rec_pos.z = fine_tracking_path.at(i).z - (fine_tracking_path.at(i-1).z-fine_tracking_path.at(i).z)/length * dis_end_point_ext;
      }
      
    }else{
      prev_rec_pos.x = (fine_tracking_path.at(i).x+fine_tracking_path.at(i-1).x)/2.;
      prev_rec_pos.y = (fine_tracking_path.at(i).y+fine_tracking_path.at(i-1).y)/2.;
      prev_rec_pos.z = (fine_tracking_path.at(i).z+fine_tracking_path.at(i-1).z)/2.;

      next_rec_pos.x = (fine_tracking_path.at(i).x+fine_tracking_path.at(i+1).x)/2.;
      next_rec_pos.y = (fine_tracking_path.at(i).y+fine_tracking_path.at(i+1).y)/2.;
      next_rec_pos.z = (fine_tracking_path.at(i).z+fine_tracking_path.at(i+1).z)/2.;
    }
    
    

    dx.at(i) = (sqrt(pow(curr_rec_pos.x-prev_rec_pos.x,2)+pow(curr_rec_pos.y-prev_rec_pos.y,2)+pow(curr_rec_pos.z-prev_rec_pos.z,2))
		 +sqrt(pow(curr_rec_pos.x-next_rec_pos.x,2)+pow(curr_rec_pos.y-next_rec_pos.y,2)+pow(curr_rec_pos.z-next_rec_pos.z,2)));
    dQ.at(i) = (5000 * dx.at(i));

    pu.at(i) = (offset_u + (slope_yu * curr_rec_pos.y + slope_zu * curr_rec_pos.z));
    pv.at(i) = (offset_v + (slope_yv * curr_rec_pos.y + slope_zv * curr_rec_pos.z)+2400);
    pw.at(i) = (offset_w + (slope_yw * curr_rec_pos.y + slope_zw * curr_rec_pos.z)+4800);
    pt.at(i) = (offset_t + slope_xt * curr_rec_pos.x );
    reduced_chi2.at(i) = (0);
  }
}

void WCPPID::PR3DCluster::dQ_dx_fit(std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,  std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, double flash_time,double dis_end_point_ext, bool flag_dQ_dx_fit_reg){
  if (fine_tracking_path.size()<=1) return;
  pu.clear();
  pv.clear();
  pw.clear();
  pt.clear();
  dQ.clear();
  dx.clear();

  
  // Need to take into account the time, so one can properly calculate X value for diffusion ...
  TPCParams& mp = Singleton<TPCParams>::Instance();
  double time_slice_width = mp.get_ts_width();
  int time_offset = mp.get_time_offset();
  int nrebin = mp.get_nrebin();
  double pitch_u = mp.get_pitch_u();
  double pitch_v = mp.get_pitch_v();
  double pitch_w = mp.get_pitch_w();
  double first_u_dis = mp.get_first_u_dis();
  double first_v_dis = mp.get_first_v_dis();
  double first_w_dis = mp.get_first_w_dis();
  double angle_u = mp.get_angle_u();
  double angle_v = mp.get_angle_v();
  double angle_w = mp.get_angle_w();
  //  convert Z to W ... 
  double slope_zw = 1./pitch_w * cos(angle_w);
  double slope_yw = 1./pitch_w * sin(angle_w);
  
  double slope_yu = -1./pitch_u * sin(angle_u);
  double slope_zu = 1./pitch_u * cos(angle_u);
  double slope_yv = -1./pitch_v * sin(angle_v);
  double slope_zv = 1./pitch_v * cos(angle_v);
  //convert Y,Z to U,V
  double offset_w = -first_w_dis/pitch_w + 0.5;
  double offset_u = -first_u_dis/pitch_u + 0.5;
  double offset_v = -first_v_dis/pitch_v + 0.5;
  
  double first_t_dis = point_cloud->get_cloud().pts[0].mcell->GetTimeSlice()*time_slice_width - point_cloud->get_cloud().pts[0].x;
  double slope_xt = 1./time_slice_width;
  double offset_t =  first_t_dis/time_slice_width + 0.5;
  
  // get the correct flash time matching TPC side
  flash_time = flash_time - time_offset*units::microsecond; // us

  // given an x position value, we want to convert this to a drift time 
  // pos_x/time_slice_width * nrebin * 0.5*units::us // us
  // difference between these two numbers are the time in us ... 
  
  // Now figure out the diffusion coefficients
  // these are current  numbers in WCT, not sure what would be the values for data ... 
  double DL = 6.4 * pow(units::cm,2)/units::second ;
  double DT = 9.8 * pow(units::cm,2)/units::second ;

  // these are the transverse broading due to software filters in the wire dimension
  // these should be quadrature added to the
  // there is some cancellation of this effect with the effect of using average field response in the deconvolution ... 0-
  double col_sigma_w_T = 0.188060 * pitch_w*0.2; // units::mm
  double ind_sigma_u_T = 0.402993 * pitch_u*0.3; // units::mm
  double ind_sigma_v_T = 0.402993 * pitch_v*0.5; // units::mm

  double rel_uncer_ind = 0.075; // original 0.1
  double rel_uncer_col = 0.05; // original 0.035

  double add_uncer_ind = 0.0;
  double add_uncer_col = 300.0; // if 800 can reach balance with induction planes ...
  
  // this is the longitudinal filters in the time dimension ...
  double add_sigma_L = 1.428249  * time_slice_width / nrebin / 0.5; // units::mm

  // Point-like case ... 
  // these should be the expected values:
  // U, V, W, T,  1252.01, 3819.54, 6799.62, 1485.81
  // reco position 1252.02, 3819.63, 6799.67, 1485.79
  // good ...

  //std::set<int> good_channels_set =
  update_data_dQ_dx_fit(global_wc_map, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge);


  std::cout << "dQ/dx: " << map_2D_ut_charge.size() << " " << map_2D_vt_charge.size() << " " << map_2D_wt_charge.size() << std::endl; 
  // for (const auto& item : map_2D_ut_charge) {
  //   std::cout << "U: (" << item.first.second*4 << ", " << item.first.first << ") -> ("
  //         << std::get<0>(item.second) << ", "
  //         << std::get<1>(item.second) << ", "
  //         << std::get<2>(item.second) << ")" << std::endl;
  // }

  int n_3D_pos = fine_tracking_path.size();
  int n_2D_u = map_2D_ut_charge.size();
  int n_2D_v = map_2D_vt_charge.size();
  int n_2D_w = map_2D_wt_charge.size();

  Eigen::VectorXd pos_3D(n_3D_pos), data_u_2D(n_2D_u), data_v_2D(n_2D_v), data_w_2D(n_2D_w), pred_data_u_2D(n_2D_u), pred_data_v_2D(n_2D_v), pred_data_w_2D(n_2D_w);
  
  Eigen::SparseMatrix<double> RU(n_2D_u, n_3D_pos) ;
  Eigen::SparseMatrix<double> RV(n_2D_v, n_3D_pos) ;
  Eigen::SparseMatrix<double> RW(n_2D_w, n_3D_pos) ;
  Eigen::VectorXd pos_3D_init(n_3D_pos);
  std::vector<int> reg_flag_u(n_3D_pos,0), reg_flag_v(n_3D_pos,0), reg_flag_w(n_3D_pos,0);
  // initialize 
  for (int i = 0;i!=n_3D_pos;i++){
    pos_3D_init(i) = 50000;
  }

  // start to fill in the data ...
  {
    int n_u = 0;
    for (auto it = map_2D_ut_charge.begin(); it!= map_2D_ut_charge.end(); it++){
      if (std::get<0>(it->second) > 0)
	data_u_2D(n_u) = std::get<0>(it->second)/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2));
      else
	data_u_2D(n_u) = 0;
      if (std::isnan(data_u_2D(n_u)))
	std::cout << "U: " << data_u_2D(n_u) << " " << std::get<1>(it->second) << " " << std::get<0>(it->second)*rel_uncer_ind << " " << std::endl;

  // std::cout << it->first.second*4 << " " << it->first.first << " " << std::get<0>(it->second) << " " << std::get<1>(it->second) << " " << data_u_2D(n_u) << std::endl;

      n_u ++;
    }
    int n_v = 0;
    for (auto it = map_2D_vt_charge.begin(); it!= map_2D_vt_charge.end(); it++){
      if (std::get<0>(it->second) > 0)
	data_v_2D(n_v) = std::get<0>(it->second)/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2));
      else
	data_v_2D(n_v) = 0;
      if (std::isnan(data_v_2D(n_v)))
	std::cout << "V: " << data_v_2D(n_v) << " " << std::get<1>(it->second) << " " << std::get<0>(it->second)*rel_uncer_ind << " " << std::endl;
      n_v ++;
    }
    int n_w = 0;
    for (auto it = map_2D_wt_charge.begin(); it!= map_2D_wt_charge.end(); it++){
      if (std::get<0>(it->second)>0)
	data_w_2D(n_w) = std::get<0>(it->second)/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_col,2) + pow(add_uncer_col,2));
      else
	data_w_2D(n_w) = 0;
      if (std::isnan(data_w_2D(n_w)))
	std::cout << "W: " << data_w_2D(n_w) << " " << std::get<1>(it->second) << " " << std::get<0>(it->second)*rel_uncer_col << " " << std::endl;


      //      std::cout << "W: " << data_w_2D(n_w) << " " << std::get<1>(it->second) << " " << std::get<0>(it->second)*rel_uncer_col << " " << std::endl;
      
      n_w ++;
    }
  }
  
  for (int i=0;i!=n_3D_pos;i++){
    
    Point prev_rec_pos, next_rec_pos;
    Point curr_rec_pos(fine_tracking_path.at(i).x, fine_tracking_path.at(i).y, fine_tracking_path.at(i).z);

    if (i==0){
      next_rec_pos.x = (fine_tracking_path.at(i).x+fine_tracking_path.at(i+1).x)/2.;
      next_rec_pos.y = (fine_tracking_path.at(i).y+fine_tracking_path.at(i+1).y)/2.;
      next_rec_pos.z = (fine_tracking_path.at(i).z+fine_tracking_path.at(i+1).z)/2.;
      double length = sqrt(pow(fine_tracking_path.at(i+1).x-fine_tracking_path.at(i).x,2)
			   +pow(fine_tracking_path.at(i+1).y-fine_tracking_path.at(i).y,2)
			   +pow(fine_tracking_path.at(i+1).z-fine_tracking_path.at(i).z,2) );
      if (length ==0){
	prev_rec_pos.x = fine_tracking_path.at(i).x;
	prev_rec_pos.y = fine_tracking_path.at(i).y;
	prev_rec_pos.z = fine_tracking_path.at(i).z;
      }else{
	prev_rec_pos.x = fine_tracking_path.at(i).x - (fine_tracking_path.at(i+1).x-fine_tracking_path.at(i).x)/length * dis_end_point_ext;
	prev_rec_pos.y = fine_tracking_path.at(i).y - (fine_tracking_path.at(i+1).y-fine_tracking_path.at(i).y)/length * dis_end_point_ext;
	prev_rec_pos.z = fine_tracking_path.at(i).z - (fine_tracking_path.at(i+1).z-fine_tracking_path.at(i).z)/length * dis_end_point_ext;
      }
    }else if (i==n_3D_pos-1){
      prev_rec_pos.x = (fine_tracking_path.at(i).x+fine_tracking_path.at(i-1).x)/2.;
      prev_rec_pos.y = (fine_tracking_path.at(i).y+fine_tracking_path.at(i-1).y)/2.;
      prev_rec_pos.z = (fine_tracking_path.at(i).z+fine_tracking_path.at(i-1).z)/2.;
      double length = sqrt(pow(fine_tracking_path.at(i-1).x-fine_tracking_path.at(i).x,2)
			   +pow(fine_tracking_path.at(i-1).y-fine_tracking_path.at(i).y,2)
			   +pow(fine_tracking_path.at(i-1).z-fine_tracking_path.at(i).z,2) );
      if (length ==0){
	next_rec_pos.x = fine_tracking_path.at(i).x;
	next_rec_pos.y = fine_tracking_path.at(i).y;
	next_rec_pos.z = fine_tracking_path.at(i).z;
      }else{
	next_rec_pos.x = fine_tracking_path.at(i).x - (fine_tracking_path.at(i-1).x-fine_tracking_path.at(i).x)/length * dis_end_point_ext;
	next_rec_pos.y = fine_tracking_path.at(i).y - (fine_tracking_path.at(i-1).y-fine_tracking_path.at(i).y)/length * dis_end_point_ext;
	next_rec_pos.z = fine_tracking_path.at(i).z - (fine_tracking_path.at(i-1).z-fine_tracking_path.at(i).z)/length * dis_end_point_ext;
      }
      
    }else{
      prev_rec_pos.x = (fine_tracking_path.at(i).x+fine_tracking_path.at(i-1).x)/2.;
      prev_rec_pos.y = (fine_tracking_path.at(i).y+fine_tracking_path.at(i-1).y)/2.;
      prev_rec_pos.z = (fine_tracking_path.at(i).z+fine_tracking_path.at(i-1).z)/2.;

      next_rec_pos.x = (fine_tracking_path.at(i).x+fine_tracking_path.at(i+1).x)/2.;
      next_rec_pos.y = (fine_tracking_path.at(i).y+fine_tracking_path.at(i+1).y)/2.;
      next_rec_pos.z = (fine_tracking_path.at(i).z+fine_tracking_path.at(i+1).z)/2.;
    }

    // not needed ... 
    /* curr_rec_pos.x = (curr_rec_pos.x + 0.6*units::cm)/1.098 * 1.101 - 1 * 0.1101*units::cm ; */
    /* prev_rec_pos.x = (prev_rec_pos.x + 0.6*units::cm)/1.098 * 1.101 - 1 * 0.1101*units::cm ; */
    /* next_rec_pos.x = (next_rec_pos.x + 0.6*units::cm)/1.098 * 1.101 - 1 * 0.1101*units::cm ; */

    //    std::cout << curr_rec_pos << " " << prev_rec_pos << " " << next_rec_pos << std::endl;
    
    dx.push_back(sqrt(pow(curr_rec_pos.x-prev_rec_pos.x,2)+pow(curr_rec_pos.y-prev_rec_pos.y,2)+pow(curr_rec_pos.z-prev_rec_pos.z,2))
		 +sqrt(pow(curr_rec_pos.x-next_rec_pos.x,2)+pow(curr_rec_pos.y-next_rec_pos.y,2)+pow(curr_rec_pos.z-next_rec_pos.z,2)));

    pu.push_back(offset_u + (slope_yu * curr_rec_pos.y + slope_zu * curr_rec_pos.z));
    pv.push_back(offset_v + (slope_yv * curr_rec_pos.y + slope_zv * curr_rec_pos.z)+2400);
    pw.push_back(offset_w + (slope_yw * curr_rec_pos.y + slope_zw * curr_rec_pos.z)+4800);
    pt.push_back(offset_t + slope_xt * curr_rec_pos.x );
    
    
    // std::cout << i << " " << dx.back() << std::endl;

    
    std::vector<double> centers_U ;
    std::vector<double> centers_V ;
    std::vector<double> centers_W ;
    std::vector<double> centers_T ;
    std::vector<double> sigmas_T;
    std::vector<double> sigmas_U;
    std::vector<double> sigmas_V;
    std::vector<double> sigmas_W;
    std::vector<double> weights;


    
    for (int j=0;j!=5;j++){
      Point reco_pos;
      reco_pos.x = prev_rec_pos.x + (curr_rec_pos.x-prev_rec_pos.x)/5.*(j+0.5);
      reco_pos.y = prev_rec_pos.y + (curr_rec_pos.y-prev_rec_pos.y)/5.*(j+0.5);
      reco_pos.z = prev_rec_pos.z + (curr_rec_pos.z-prev_rec_pos.z)/5.*(j+0.5);
      double central_T = offset_t + slope_xt * reco_pos.x ;
      double central_U = offset_u + (slope_yu * reco_pos.y + slope_zu * reco_pos.z);
      double central_V = offset_v + (slope_yv * reco_pos.y + slope_zv * reco_pos.z)+2400;
      double central_W = offset_w + (slope_yw * reco_pos.y + slope_zw * reco_pos.z)+4800;
      double weight = sqrt(pow(prev_rec_pos.x-curr_rec_pos.x,2)+
			   pow(prev_rec_pos.y-curr_rec_pos.y,2)+
			   pow(prev_rec_pos.z-curr_rec_pos.z,2));

      double drift_time = reco_pos.x/time_slice_width * nrebin * 0.5*units::microsecond  - flash_time ;
      if (drift_time <50 *units::microsecond ) drift_time = 50* units::microsecond;
      
      //  std::cout << drift_time/units::microsecond << std::endl;
      double diff_sigma_L = sqrt(2* DL * drift_time);
      double diff_sigma_T = sqrt(2* DT * drift_time);

      double sigma_L = sqrt(pow(diff_sigma_L,2) + pow(add_sigma_L,2))/time_slice_width;
      double sigma_T_u = sqrt(pow(diff_sigma_T,2) + pow(ind_sigma_u_T,2))/pitch_u;
      double sigma_T_v = sqrt(pow(diff_sigma_T,2) + pow(ind_sigma_v_T,2))/pitch_v;
      double sigma_T_w = sqrt(pow(diff_sigma_T,2) + pow(col_sigma_w_T,2))/pitch_w;
      
      centers_U.push_back(central_U);
      centers_V.push_back(central_V);
      centers_W.push_back(central_W);
      centers_T.push_back(central_T);

      weights.push_back(weight);
      
      sigmas_U.push_back(sigma_T_u);
      sigmas_V.push_back(sigma_T_v);
      sigmas_W.push_back(sigma_T_w);
      sigmas_T.push_back(sigma_L);


      reco_pos.x = next_rec_pos.x + (curr_rec_pos.x-next_rec_pos.x)/5.*(j+0.5);
      reco_pos.y = next_rec_pos.y + (curr_rec_pos.y-next_rec_pos.y)/5.*(j+0.5);
      reco_pos.z = next_rec_pos.z + (curr_rec_pos.z-next_rec_pos.z)/5.*(j+0.5);
      central_T = offset_t + slope_xt * reco_pos.x ;
      central_U = offset_u + (slope_yu * reco_pos.y + slope_zu * reco_pos.z);
      central_V = offset_v + (slope_yv * reco_pos.y + slope_zv * reco_pos.z)+2400;
      central_W = offset_w + (slope_yw * reco_pos.y + slope_zw * reco_pos.z)+4800;
      weight = sqrt(pow(next_rec_pos.x-curr_rec_pos.x,2)+
		    pow(next_rec_pos.y-curr_rec_pos.y,2)+
		    pow(next_rec_pos.z-curr_rec_pos.z,2));
      
      drift_time = reco_pos.x/time_slice_width * nrebin * 0.5*units::microsecond  - flash_time ;
      if (drift_time <50 *units::microsecond ) drift_time = 50* units::microsecond;
      //  std::cout << drift_time/units::microsecond << std::endl;
      diff_sigma_L = sqrt(2* DL * drift_time);
      diff_sigma_T = sqrt(2* DT * drift_time);

      // std::cout << drift_time << " " << DL << " " << DT << " " << diff_sigma_L << " " << diff_sigma_T << " " << ind_sigma_u_T << std::endl;


      sigma_L = sqrt(pow(diff_sigma_L,2) + pow(add_sigma_L,2))/time_slice_width;
      sigma_T_u = sqrt(pow(diff_sigma_T,2) + pow(ind_sigma_u_T,2))/pitch_u;
      sigma_T_v = sqrt(pow(diff_sigma_T,2) + pow(ind_sigma_v_T,2))/pitch_v;
      sigma_T_w = sqrt(pow(diff_sigma_T,2) + pow(col_sigma_w_T,2))/pitch_w;
      
      
      centers_U.push_back(central_U);
      centers_V.push_back(central_V);
      centers_W.push_back(central_W);
      centers_T.push_back(central_T);

      weights.push_back(weight);
      
      sigmas_U.push_back(sigma_T_u);
      sigmas_V.push_back(sigma_T_v);
      sigmas_W.push_back(sigma_T_w);
      sigmas_T.push_back(sigma_L);
    }
    
    // std::cout << i << " ";
    //  for (size_t idx = 0; idx < centers_U.size(); ++idx) {
    //         std::cout << centers_U[idx] << " ";
    //     }
    //     std::cout << std::endl;

    // std::cout << i << " V ";
    //     for (size_t idx = 0; idx < centers_V.size(); ++idx) {
    //         std::cout << centers_V[idx] << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << i << " W ";
    //     for (size_t idx = 0; idx < centers_W.size(); ++idx) {
    //         std::cout << centers_W[idx] << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << i << " T ";
    //     for (size_t idx = 0; idx < centers_T.size(); ++idx) {
    //         std::cout << centers_T[idx] << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << i << " Weights ";
    //     for (size_t idx = 0; idx < weights.size(); ++idx) {
    //         std::cout << weights[idx] << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout <<i << " SU ";
    //     for (size_t idx = 0; idx < sigmas_U.size(); ++idx) {
    //         std::cout << sigmas_U[idx] << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << i << " SV ";
    //     for (size_t idx = 0; idx < sigmas_V.size(); ++idx) {
    //         std::cout << sigmas_V[idx] << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << i << " SW ";
    //     for (size_t idx = 0; idx < sigmas_W.size(); ++idx) {
    //         std::cout << sigmas_W[idx] << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << i << " ST ";
    //     for (size_t idx = 0; idx < sigmas_T.size(); ++idx) {
    //         std::cout << sigmas_T[idx] << " ";
    //     }
    //     std::cout << std::endl;

    int n_u = 0;
    double sum_u = 0;
    for (auto it = map_2D_ut_charge.begin(); it!= map_2D_ut_charge.end(); it++){
      if (fabs(it->first.first - centers_U.front()) <= 10 &&
      	  fabs(it->first.second - centers_T.front()) <= 10 ){
            double value = cal_gaus_integral_seg(it->first.second, it->first.first,centers_T, sigmas_T, centers_U, sigmas_U, weights , 0 , 4);
            sum_u += value;
            // near dead channels ...
            if (std::get<2>(it->second)==0 && value > 0) reg_flag_u.at(i) = 1;
            
            if (value > 0 && std::get<0>(it->second) >0 && std::get<2>(it->second)!=0){
              // if (i!=143)
              RU.insert(n_u,i) = value/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2));
              // if (i==143) std::cout << "U: " << it->first.first << " " << it->first.second << " " << value << std::endl;
              // std::cout << n_u << " " << i << " " << it->first.second*4 << " " << it->first.first << " " << i << " " << value / sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2)) << std::endl;
            }
      }
      n_u ++;
    }

    

    int n_v = 0;
    double sum_v = 0;
    for (auto it = map_2D_vt_charge.begin(); it!= map_2D_vt_charge.end(); it++){
      if (fabs(it->first.first +  2400 - centers_V.front()) <= 10 &&
	  fabs(it->first.second - centers_T.front()) <= 10 ){
	double value = cal_gaus_integral_seg(it->first.second, it->first.first + 2400, centers_T, sigmas_T, centers_V, sigmas_V, weights , 0 , 4);
	sum_v += value;
	// near dead channels
	if (std::get<2>(it->second)==0 && value > 0) {
	  // if (i==136) std::cout << it->first.first << " " << it->first.second << std::endl;
	  reg_flag_v.at(i) = 1;
	}

	
	if (value > 0 && std::get<0>(it->second) >0 && std::get<2>(it->second)!=0){
	  //	  if (i==136) std::cout << "V: " << it->first.first << " " << it->first.second << " " << value << std::endl;
	  // if (i!=143)
	  RV.insert(n_v,i) = value/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2));
	}
      }
      n_v ++;
    }

    
    int n_w = 0;
    double sum_w = 0;
    for (auto it = map_2D_wt_charge.begin(); it!= map_2D_wt_charge.end(); it++){
      if (fabs(it->first.first + 4800 - centers_W.front()) <= 10 &&
      	  fabs(it->first.second - centers_T.front()) <= 10 ){
	double value = cal_gaus_integral_seg(it->first.second, it->first.first + 4800,centers_T, sigmas_T, centers_W, sigmas_W, weights , 0 , 4);
	sum_w += value;
	// near dead channels ...
	if (std::get<2>(it->second)==0 && value > 0) reg_flag_w.at(i) = 1;
	
	
		
        // relevant && charge > 0 && not dead channel ...
        if (value > 0 && std::get<0>(it->second) >0 && std::get<2>(it->second)!=0){
          // if (i!=143)
          RW.insert(n_w,i) = value/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_col,2) + pow(add_uncer_col,2));
          //  if (i==147) std::cout << "W: " << it->first.first << " " << it->first.second << " " << value << std::endl;
          // std::cout << n_w << " " << i << " " << it->first.second*4 << " " << it->first.first << " " << i << " " << value / sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_col,2) + pow(add_uncer_col,2)) << std::endl;
        }
      }
      n_w ++;
    }

    // additional checks on the dead channels ...
    if (reg_flag_u.at(i)==0){
      for (size_t kk = 0; kk!=centers_U.size(); kk++){
    	if (map_2D_ut_charge.find(std::make_pair(int(centers_U.at(kk)), int(centers_T.at(kk)) ))==map_2D_ut_charge.end()) {
    	  reg_flag_u.at(i) =1;
    	  break;
    	}
      }
    }
    if (reg_flag_v.at(i)==0){
      for (size_t kk = 0; kk!=centers_V.size(); kk++){
    	if (map_2D_vt_charge.find(std::make_pair(int(centers_V.at(kk)-2400), int(centers_T.at(kk)) ))==map_2D_vt_charge.end()) {
	  //if (i==136)  std::cout << centers_V.at(kk) << " " << centers_T.at(kk) << std::endl;
    	  reg_flag_v.at(i) =1;
    	  break;
    	}
      }
    }
    if (reg_flag_w.at(i)==0){
      for (size_t kk = 0; kk!=centers_W.size(); kk++){
    	if (map_2D_wt_charge.find(std::make_pair(int(centers_W.at(kk)-4800), int(centers_T.at(kk)) ))==map_2D_wt_charge.end()) {
    	  reg_flag_w.at(i) =1;
    	  break;
    	}
      }
    }

    // std::cout << i << " " << reg_flag_u.at(i) << " " << reg_flag_v.at(i) << " " << reg_flag_w.at(i) << std::endl;

  }


  Eigen::SparseMatrix<double> RUT = Eigen::SparseMatrix<double>(RU.transpose());
  Eigen::SparseMatrix<double> RVT = Eigen::SparseMatrix<double>(RV.transpose());
  Eigen::SparseMatrix<double> RWT = Eigen::SparseMatrix<double>(RW.transpose());


  // check the compact view, relax the uncertainties ...
  // A(i,j), i is row, j is column,  One row corresponding to a data point ...

  Eigen::SparseMatrix<double> MU(n_2D_u, n_2D_u), MV(n_2D_v, n_2D_v), MW(n_2D_w, n_2D_w);
  for (int k=0;k!=n_2D_u;k++){
    MU.insert(k,k) = 1;
  }
  for (int k=0;k!=n_2D_v;k++){
    MV.insert(k,k) = 1;
  }
  for (int k=0;k!=n_2D_w;k++){
    MW.insert(k,k) = 1;
  }

  // std::cout << "U: " << std::endl;
  std::vector<std::pair<double, double> > overlap_u = cal_compact_matrix(MU, RUT, n_2D_u, n_3D_pos,3);
  // std::cout << "V: " << std::endl;
  std::vector<std::pair<double, double> > overlap_v = cal_compact_matrix(MV, RVT, n_2D_v, n_3D_pos,3); // three wire sharing ...
  // std::cout << "W: " << std::endl;
  std::vector<std::pair<double, double> > overlap_w = cal_compact_matrix(MW, RWT, n_2D_w, n_3D_pos,2); // two wire sharing  ...

//   for (size_t i=0;i!=n_3D_pos;i++){
//         // std::cout << i << " " << reg_flag_u.at(i) << " " << reg_flag_v.at(i) << " " << reg_flag_w.at(i) << std::endl;
//     std::cout << i << " " << (overlap_u.at(i).first + overlap_u.at(i).second)/2. << " " 
//   	      << (overlap_v.at(i).first + overlap_v.at(i).second)/2. << " " 
//   	      << (overlap_w.at(i).first + overlap_w.at(i).second)/2. << " " << std::endl;
// //  	      << MU.coeffRef(i,i) << " " << MV.coeffRef(i,i) << " " << MW.coeffRef(i,i) << std::endl;
//   } 
  
  // { int n_w = 0;
  //   for (auto it = map_2D_wt_charge.begin(); it!= map_2D_wt_charge.end(); it++){
     
  //     std::cout << n_w << " " << it->first.first << " " << it->first.second*4 << " " << MW.coeffRef(n_w,n_w) << " " << std::get<0>(it->second) << " " << std::get<1>(it->second) << std::endl;
  //     n_w ++;
  //   }
  // }
  
  // add regularization ...
  Eigen::SparseMatrix<double> FMatrix(n_3D_pos, n_3D_pos);
  
  
  double dead_ind_weight = 0.3;
  double dead_col_weight = 0.9;

  double close_ind_weight = 0.15;
  double close_col_weight = 0.45;

  /* for (size_t i=0;i!=n_3D_pos;i++){ */
  /*   std::cout << dx.at(i)/units::cm << std::endl; */
  /* } */
  
  for (size_t i=0;i!=n_3D_pos;i++){
    bool flag_u = reg_flag_u.at(i);
    bool flag_v = reg_flag_v.at(i);
    bool flag_w = reg_flag_w.at(i);
    
    if (n_3D_pos!=1){
      double weight = 0;
      if (i==0){
        if (flag_u) weight += dead_ind_weight;
        if (flag_v) weight += dead_ind_weight;
        if (flag_w) weight += dead_col_weight;

        if (overlap_u.at(i).second > 0.5) weight += close_ind_weight * pow(2*overlap_u.at(i).second-1,2);
        if (overlap_v.at(i).second > 0.5) weight += close_ind_weight * pow(2*overlap_v.at(i).second-1,2);
        if (overlap_w.at(i).second > 0.5) weight += close_col_weight * pow(2*overlap_w.at(i).second-1,2);
        
        FMatrix.insert(0,0) = -weight/((dx.at(0)+0.001*units::cm)/(0.6*units::cm)); 
        FMatrix.insert(0,1) = weight/((dx.at(1)+0.001*units::cm)/(0.6*units::cm));
      }else if (i==n_3D_pos-1){
        if (flag_u) weight += dead_ind_weight;
        if (flag_v) weight += dead_ind_weight;
        if (flag_w) weight += dead_col_weight;

        if (overlap_u.at(i).first > 0.5) weight += close_ind_weight * pow(2*overlap_u.at(i).first-1,2);
        if (overlap_v.at(i).first > 0.5) weight += close_ind_weight * pow(2*overlap_v.at(i).first-1,2);
        if (overlap_w.at(i).first > 0.5) weight += close_col_weight * pow(2*overlap_w.at(i).first-1,2);
        
        FMatrix.insert(i,i) = -weight/((dx.at(i)+0.001*units::cm)/(0.6*units::cm)); 
        FMatrix.insert(i,i-1) = weight/((dx.at(i-1)+0.001*units::cm)/(0.6*units::cm));
      }else{
        if (flag_u) weight += dead_ind_weight;
        if (flag_v) weight += dead_ind_weight;
        if (flag_w) weight += dead_col_weight;

        if (overlap_u.at(i).first + overlap_u.at(i).second > 1) weight += close_ind_weight * pow(overlap_u.at(i).first + overlap_u.at(i).second - 1,2);
        if (overlap_v.at(i).first + overlap_v.at(i).second > 1) weight += close_ind_weight * pow(overlap_v.at(i).first + overlap_v.at(i).second - 1,2);
        if (overlap_w.at(i).first + overlap_w.at(i).second > 1) weight += close_col_weight * pow(overlap_w.at(i).first + overlap_w.at(i).second - 1,2);
        
        FMatrix.insert(i,i)=-2.*weight/((dx.at(i)+0.001*units::cm)/(0.6*units::cm)); 
        FMatrix.insert(i,i+1)=weight/((dx.at(i+1)+0.001*units::cm)/(0.6*units::cm)); 
        FMatrix.insert(i,i-1)=weight/((dx.at(i-1)+0.001*units::cm)/(0.6*units::cm));
      }
      // std::cout << i << " " << flag_u << " " << flag_v << " " << flag_w << " " << overlap_u.at(i).first << " | " << overlap_u.at(i).second << " " << overlap_v.at(i).first << " | " << overlap_v.at(i).second << " " << overlap_w.at(i).first << " | " << overlap_w.at(i).second << " " << weight << " " << ((dx.at(i)+0.001*units::cm)/(0.6*units::cm)) << std::endl;
      // std::cout << i << " " << FMatrix.coeff(i, i) << std::endl;

    }
  }


 
  
  

  double lambda = 0.0005;
  FMatrix *= lambda;

  if (!flag_dQ_dx_fit_reg)
    FMatrix *=0.01;
  
  Eigen::SparseMatrix<double> FMatrixT = Eigen::SparseMatrix<double>(FMatrix.transpose());
  
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
  Eigen::VectorXd b = RUT * MU * data_u_2D + RVT * MV * data_v_2D + RWT * MW * data_w_2D;
  Eigen::SparseMatrix<double> A =  RUT * MU * RU + RVT * MV * RV + RWT * MW * RW + FMatrixT * FMatrix;//
  solver.compute(A);
  
  pos_3D = solver.solveWithGuess(b,pos_3D_init);

  if (std::isnan(solver.error())){
    pos_3D = solver.solve(b);
  }


  //  for (int i = 0; i < b.size(); ++i) {
  //       // Example: print or process each element of b
  //       std::cout << "b[" << i << "] = " << b[i] << " " << A.coeff(i,i) << " " << lambda << " " << flag_dQ_dx_fit_reg << std::endl;
  //       // You can add your processing logic here
  //   }
    
  double sum = 0 ;
  for (int i=0;i!=n_3D_pos;i++){
    /* std::cout << i << " "<< pos_3D(i) << " " << dx.at(i)/units::cm << " " << pos_3D(i)/dx.at(i)*units::cm << std::endl; */
  //   double corr = 1.;
  //   if (mp.get_flag_corr()){
  //     corr = mp.get_corr_factor(fine_tracking_path.at(i), offset_u,  slope_yu,  slope_zu,  offset_v,  slope_yv,  slope_zv,  offset_w,  slope_yw,  slope_zw);
  //   }
    
  //   // correction electron lifetime attenuation
  //   double elifetime_ratio =  mp.get_attenuation_ratio((fine_tracking_path.at(i).x/time_slice_width * nrebin * 0.5*units::microsecond  - flash_time)/units::millisecond);
  //   //    std::cout << fine_tracking_path.at(i).x << " " << (fine_tracking_path.at(i).x/time_slice_width * nrebin * 0.5*units::microsecond  - flash_time)/units::millisecond << " " << elifetime_ratio << std::endl;
    
  //   corr /= elifetime_ratio ; 
    
  //   double central_U = offset_u + (slope_yu * fine_tracking_path.at(i).y + slope_zu * fine_tracking_path.at(i).z);
  //   if (central_U >=296 && central_U <=327 ||
	// central_U >=336 && central_U <=337 ||
	// central_U >=343 && central_U <=351 ||
	// central_U >=376 && central_U <=400 ||
	// central_U >=410 && central_U <=484 ||
	// central_U >=501 && central_U <=524 ||
	// central_U >=536 && central_U <=671)
  //     dQ.push_back(pos_3D(i)/0.7*corr);
  //   else
  //     dQ.push_back(pos_3D(i)*corr);
  
    dQ.push_back(pos_3D(i));

    sum += dQ.back();
  }
  //  std::cout << "Cluster: " << cluster_id << " " << "total: " << sum << std::endl;


  // prediction ...
  pred_data_u_2D = RU * pos_3D;
  pred_data_v_2D = RV * pos_3D;
  pred_data_w_2D = RW * pos_3D;

  int n_u = 0;
  for (auto it = map_2D_ut_charge.begin(); it!=map_2D_ut_charge.end(); it++){
    proj_data_u_map[std::make_pair(it->first.first, it->first.second)] = std::make_tuple(std::get<0>(it->second), std::get<1>(it->second), pred_data_u_2D(n_u) * sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2)+pow(add_uncer_ind,2)));
    n_u++;
  }
  int n_v = 0;
  for (auto it = map_2D_vt_charge.begin(); it!=map_2D_vt_charge.end(); it++){
    proj_data_v_map[std::make_pair(it->first.first+2400, it->first.second)] = std::make_tuple(std::get<0>(it->second), std::get<1>(it->second), pred_data_v_2D(n_v) * sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2)));
    n_v++;
  }
  int n_w = 0;
  for (auto it = map_2D_wt_charge.begin(); it!=map_2D_wt_charge.end(); it++){
    //    if (it->first.first+4800 > 8256) std::cout << "problem " << it->first.first+4800 << std::endl;
    proj_data_w_map[std::make_pair(it->first.first+4800, it->first.second)] = std::make_tuple(std::get<0>(it->second), std::get<1>(it->second), pred_data_w_2D(n_w) * sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_col,2)+pow(add_uncer_col,2)));
    n_w++;
  }

  

  
  // label the data comparison ...
  reduced_chi2.clear();
  for (int k=0;k!=RU.outerSize(); ++k){
    double sum[3] = {0,0,0};
    double sum1[3] = {0,0,0};
    for (Eigen::SparseMatrix<double>::InnerIterator it(RU,k); it; ++it){
      sum[0] += pow(data_u_2D(it.row()) - pred_data_u_2D(it.row()),2) * (it.value() * pos_3D(k) )/pred_data_u_2D(it.row());
      //      std::cout << it.value() << " " << it.row() << " " << it.col() << std::endl;
      sum1[0] += (it.value() * pos_3D(k) )/pred_data_u_2D(it.row());
    }

    for (Eigen::SparseMatrix<double>::InnerIterator it(RV,k); it; ++it){
      sum[1] += pow(data_v_2D(it.row()) - pred_data_v_2D(it.row()),2) * (it.value() * pos_3D(k))/pred_data_v_2D(it.row());
      //      std::cout << it.value() << " " << it.row() << " " << it.col() << std::endl;
      sum1[1] += (it.value() * pos_3D(k))/pred_data_v_2D(it.row());
    }

    for (Eigen::SparseMatrix<double>::InnerIterator it(RW,k); it; ++it){
      sum[2] += pow(data_w_2D(it.row()) - pred_data_w_2D(it.row()),2) * (it.value() * pos_3D(k))/pred_data_w_2D(it.row());
      //      std::cout << it.value() << " " << it.row() << " " << it.col() << std::endl;
      sum1[2] += (it.value()*pos_3D(k))/pred_data_w_2D(it.row());
    }
    reduced_chi2.push_back(sqrt((sum[0] + sum[1] + sum[2]/4.)/(sum1[0]+sum1[1]+sum1[2])));
    //    std::cout << "Xin: " << cluster_id << " " << k << " " << sum[0] << " " << sum[1] << " " << sum[2] << " " << sum1[0] << " " << sum1[1] << " " << sum1[2] << " " <<  (fine_tracking_path.at(k).x - time_slice_width /( nrebin * 0.5*units::microsecond) * flash_time)/units::cm << " " <<  fine_tracking_path.at(k).y/units::cm << " " <<  fine_tracking_path.at(k).z/units::cm << std::endl;
  }
  
  
  
}

void WCPPID::PR3DCluster::update_data_dQ_dx_fit(std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_wt_charge){
  
  std::set<SlimMergeGeomCell*> cluster_mcells_set;
  for (auto it = mcells.begin(); it!=mcells.end(); it++){
    SlimMergeGeomCell *mcell = *it;
    cluster_mcells_set.insert(mcell);
  }

  for (auto it=mcells.begin();it!=mcells.end();it++){
    SlimMergeGeomCell *mcell = (*it);
    int time_slice = mcell->GetTimeSlice();
    std::map<const GeomWire*, SMGCSelection >& timeslice_wc_map = global_wc_map[time_slice];

    WireChargeMap& wire_charge_map = mcell->get_wirecharge_map();
    WireChargeMap& wire_charge_err_map = mcell->get_wirechargeerr_map();
    for (auto it = wire_charge_map.begin(); it!= wire_charge_map.end(); it++){
      const GeomWire* wire = it->first;
      double charge = it->second;
      double charge_err = wire_charge_err_map[wire];
      
      if (charge > 0)
	if (timeslice_wc_map[wire].size()>1){
	  for (auto it1 = timeslice_wc_map[wire].begin(); it1!=timeslice_wc_map[wire].end(); it1++){
	    SlimMergeGeomCell *mcell1 = *it1;
	    if (cluster_mcells_set.find(mcell1)==cluster_mcells_set.end()) charge_err = 8000;
	  }
	}

      if (wire->iplane()==0){
	auto it = map_2D_ut_charge.find(std::make_pair(wire->index(),time_slice));
	if (it !=map_2D_ut_charge.end()){
	  it->second = std::make_tuple(charge, charge_err, std::get<2>(it->second));
	}else{
	  std::cout << "Mismatch U" << std::endl;
	}
      }else if (wire->iplane()==1){
	auto it = map_2D_vt_charge.find(std::make_pair(wire->index(),time_slice));
	if (it !=map_2D_vt_charge.end()){
	  it->second = std::make_tuple(charge, charge_err, std::get<2>(it->second));
	}else{
	  std::cout << "Mismatch V" << std::endl;
	}
      }else{
	auto it = map_2D_wt_charge.find(std::make_pair(wire->index(),time_slice));
	if (it !=map_2D_wt_charge.end()){
	  it->second = std::make_tuple(charge, charge_err, std::get<2>(it->second));
	}else{
	  std::cout << "Mismatch W" << std::endl;
	}
      }

      
    }
  }

  // fill in additional stuff ... 
  /* for (auto it = map_2D_ut_charge.begin(); it!=map_2D_ut_charge.end(); it++){  */
  /*   if (std::get<2>(it->second)==0) continue; */
  /*   if (proj_data_u_map.find(it->first)==proj_data_u_map.end()){ */
  /*     proj_data_u_map[it->first] = std::make_tuple(std::get<0>(it->second),std::get<1>(it->second),0); */
  /*   } */
  /* } */
  /* for (auto it = map_2D_vt_charge.begin(); it!=map_2D_vt_charge.end(); it++){  */
  /*   if (std::get<2>(it->second)==0) continue; */
  /*   if (proj_data_v_map.find(std::make_pair(it->first.first+2400,it->first.second))==proj_data_v_map.end()){ */
  /*     proj_data_v_map[std::make_pair(it->first.first+2400,it->first.second)] = std::make_tuple(std::get<0>(it->second),std::get<1>(it->second),0); */
  /*   } */
  /* } */
  /* for (auto it = map_2D_wt_charge.begin(); it!=map_2D_wt_charge.end(); it++){  */
  /*   if (std::get<2>(it->second)==0) continue; */
  /*   if (proj_data_w_map.find(std::make_pair(it->first.first+4800,it->first.second))==proj_data_w_map.end()){ */
  /*     proj_data_w_map[std::make_pair(it->first.first+4800,it->first.second)] = std::make_tuple(std::get<0>(it->second),std::get<1>(it->second),0); */
  /*   } */
  /* } */
 
  
}
