void WCPPID::PR3DCluster::fill_data_map_trajectory(std::vector<int> indices, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>,  std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge){
  proj_data_u_map.clear();
  proj_data_v_map.clear();
  proj_data_w_map.clear();

  if (indices.size()==0){
    for (auto it = map_2D_ut_charge.begin(); it!=map_2D_ut_charge.end(); it++){
      //good_channels_set.insert(it->first.first);
      proj_data_u_map[it->first] = std::make_tuple(std::get<0>(it->second),std::get<1>(it->second),0);
    }
    for (auto it = map_2D_vt_charge.begin(); it!=map_2D_vt_charge.end(); it++){
      //good_channels_set.insert(it->first.first+2400);
      proj_data_v_map[std::make_pair(it->first.first+2400,it->first.second)] = std::make_tuple(std::get<0>(it->second),std::get<1>(it->second),0);
    }
    for (auto it = map_2D_wt_charge.begin(); it!=map_2D_wt_charge.end(); it++){
      //good_channels_set.insert(it->first.first+4800);
      proj_data_w_map[std::make_pair(it->first.first+4800,it->first.second)] = std::make_tuple(std::get<0>(it->second),std::get<1>(it->second),0);
    }
  }else{
    for (auto it = indices.begin(); it!=indices.end(); it++){
      //      std::cout << pu.at(*it) << " " << pv.at(*it) << " " << pw.at(*it) << " " << pt.at(*it) << std::endl;
      for (auto it1 = map_3D_2DU_set[*it].first.begin(); it1!=map_3D_2DU_set[*it].first.end(); it1++){
	proj_data_u_map[*it1] = std::make_tuple(std::get<0>(map_2D_ut_charge[*it1]),std::get<1>(map_2D_ut_charge[*it1]),0);
	//std::cout << "U: " << it1->first << " " << it1->second << std::endl; 
      }
      for (auto it1 = map_3D_2DV_set[*it].first.begin(); it1!=map_3D_2DV_set[*it].first.end(); it1++){
	proj_data_v_map[std::make_pair(it1->first+2400,it1->second)] = std::make_tuple(std::get<0>(map_2D_vt_charge[*it1]),std::get<1>(map_2D_vt_charge[*it1]),0);
	//std::cout << "V: " << it1->first+2400 << " " << it1->second << std::endl; 
      }
      for (auto it1 = map_3D_2DW_set[*it].first.begin(); it1!=map_3D_2DW_set[*it].first.end(); it1++){
	proj_data_w_map[std::make_pair(it1->first+4800,it1->second)] = std::make_tuple(std::get<0>(map_2D_wt_charge[*it1]),std::get<1>(map_2D_wt_charge[*it1]),0);
	//std::cout << "W: " << it1->first+4800 << " " << it1->second << std::endl; 
      }
    }
    
  }
}

void WCPPID::PR3DCluster::trajectory_fit(WCP::PointVector& ps_vec, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set,std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, int charge_div_method, double div_sigma){
  
  TPCParams& mp = Singleton<TPCParams>::Instance();
  double pitch_u = mp.get_pitch_u();
  double pitch_v = mp.get_pitch_v();
  double pitch_w = mp.get_pitch_w();
  double angle_u = mp.get_angle_u();
  double angle_v = mp.get_angle_v();
  double angle_w = mp.get_angle_w();
  double time_slice_width = mp.get_ts_width();
  double first_u_dis = mp.get_first_u_dis();
  double first_v_dis = mp.get_first_v_dis();
  double first_w_dis = mp.get_first_w_dis();

  /* for (auto it = path_wcps.begin(); it!=path_wcps.end(); it++){ */
  /*   std::cout  << " " << it->mcell << std::endl; */
  /* } */
  
  double slope_x = 1./time_slice_width;
  //mcell
  double first_t_dis = point_cloud->get_cloud().pts[0].mcell->GetTimeSlice()*time_slice_width - point_cloud->get_cloud().pts[0].x;
    //  double first_t_dis = path_wcps.front().mcell->GetTimeSlice()*time_slice_width - path_wcps.front().x;
  double offset_t = first_t_dis / time_slice_width;

  //  convert Z to W ... 
  double slope_zw = 1./pitch_w * cos(angle_w);
  double slope_yw = 1./pitch_w * sin(angle_w);
  
  double slope_yu = -1./pitch_u * sin(angle_u);
  double slope_zu = 1./pitch_u * cos(angle_u);
  double slope_yv = -1./pitch_v * sin(angle_v);
  double slope_zv = 1./pitch_v * cos(angle_v);
  
  //convert Y,Z to U,V
  double offset_w = -first_w_dis/pitch_w;
  double offset_u = -first_u_dis/pitch_u;
  double offset_v = -first_v_dis/pitch_v;

  // // Debug output (optional - can be removed)
  // std::cout << "TrackFitting: Initialized geometry" << std::endl;
  // std::cout << "  Offsets: T=" << offset_t << " U=" << offset_u 
  //       << " V=" << offset_v << " W=" << offset_w << std::endl;
  // std::cout << "  Slopes: T=" << slope_x 
  //       << " U=(" << slope_yu << "," << slope_zu << ")"
  //       << " V=(" << slope_yv << "," << slope_zv << ")"
  //       << " W=(" << slope_yw << "," << slope_zw << ")" << std::endl;


   // 2D pixel and then the 3D index ...
  std::map<std::tuple<int,int, int>, double> map_Udiv_fac;
  std::map<std::tuple<int,int, int>, double> map_Vdiv_fac;
  std::map<std::tuple<int,int, int>, double> map_Wdiv_fac;

    // charge division method ...
  if (charge_div_method==1){
    // equal division
    for (auto it = map_2DU_3D_set.begin(); it!=map_2DU_3D_set.end(); it++){
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  	map_Udiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] = 1./it->second.size();
  	//std::cout << it->first.first << " " << it->first.second << " " << *it1 << std::endl;
      }
    }
    for (auto it = map_2DV_3D_set.begin(); it!=map_2DV_3D_set.end(); it++){
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  	map_Vdiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] = 1./it->second.size();
  	//std::cout << it->first.first << " " << it->first.second << " " << *it1 << std::endl;
      }
    }
    for (auto it = map_2DW_3D_set.begin(); it!=map_2DW_3D_set.end(); it++){
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  	map_Wdiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] = 1./it->second.size();
  	//	std::cout << it->first.first << " " << it->first.second << " " << *it1 << std::endl;
      }
    }
  }else if (charge_div_method == 2){
    // use div_sigma ...
    for (auto it = map_2DU_3D_set.begin(); it!=map_2DU_3D_set.end(); it++){
      double sum = 0;
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
        double central_t = slope_x * ps_vec.at(*it1).x + offset_t;
        double central_ch = slope_yu * ps_vec.at(*it1).y + slope_zu * ps_vec.at(*it1).z + offset_u;
        double factor = exp(-0.5 * (pow((central_t-it->first.second)*time_slice_width,2) + pow((central_ch-it->first.first)*pitch_u,2))/pow(div_sigma,2));

          // std::cout << "U " << it->first.second *4 << " " << it->first.first << " " << central_t*4 << " " << central_ch << " " << factor << std::endl;

        map_Udiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] = factor;
        sum += factor;
  	//std::cout << it->first.first << " " << it->first.second << " " << *it1 << std::endl;
      }
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  	    map_Udiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] /= sum;
        // std::cout << "U " << it->first.second*4 << " " << it->first.first << " " << map_Udiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] << " " << sum << std::endl;
      }
    }

    for (auto it = map_2DV_3D_set.begin(); it!=map_2DV_3D_set.end(); it++){
      double sum = 0;
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  	double central_t = slope_x * ps_vec.at(*it1).x + offset_t;
  	double central_ch = slope_yv * ps_vec.at(*it1).y + slope_zv * ps_vec.at(*it1).z + offset_v;
  	double factor = exp(-0.5 * (pow((central_t-it->first.second)*time_slice_width,2) + pow((central_ch-it->first.first)*pitch_v,2))/pow(div_sigma,2));
  	map_Vdiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] = factor;
  	sum += factor;
	//	if ( it->first.first == 2520 &&  it->first.second ==1)
	//  std::cout << it->first.first << " " << it->first.second << " " << central_t << " " << central_ch << " " << factor << std::endl;
      }
     
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  	map_Vdiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] /= sum;

      }
    }

    
    for (auto it = map_2DW_3D_set.begin(); it!=map_2DW_3D_set.end(); it++){
      double sum = 0;
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  	double central_t = slope_x * ps_vec.at(*it1).x + offset_t;
  	double central_ch = slope_yw * ps_vec.at(*it1).y + slope_zw * ps_vec.at(*it1).z + offset_w;
  	double factor = exp(-0.5 * (pow((central_t-it->first.second)*time_slice_width,2) + pow((central_ch-it->first.first)*pitch_w,2))/pow(div_sigma,2));
  	map_Wdiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] = factor;
  	sum += factor;
  	//std::cout << it->first.first << " " << it->first.second << " " << *it1 << " " << factor << std::endl;
      }
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  	map_Wdiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] /= sum;
      }
    }
  }

  Eigen::VectorXd pos_3D(3*ps_vec.size());
  
  for (size_t i= 0; i!=ps_vec.size(); i++){
    int n_2D_u = 2* map_3D_2DU_set[i].first.size();
    int n_2D_v = 2* map_3D_2DV_set[i].first.size();
    int n_2D_w = 2* map_3D_2DW_set[i].first.size();
    Eigen::VectorXd temp_pos_3D(3),data_u_2D(n_2D_u), data_v_2D(n_2D_v), data_w_2D(n_2D_w);
    Eigen::VectorXd temp_pos_3D_init(3);
    Eigen::SparseMatrix<double> RU(n_2D_u, 3) ;
    Eigen::SparseMatrix<double> RV(n_2D_v, 3) ;
    Eigen::SparseMatrix<double> RW(n_2D_w, 3) ;
    // initialization
    temp_pos_3D_init(0) = ps_vec.at(i).x;
    temp_pos_3D_init(1) = ps_vec.at(i).y;
    temp_pos_3D_init(2) = ps_vec.at(i).z;

    for (size_t i=0;i!=n_2D_u;i++){
      data_u_2D(i) = 0;
    }
    for (size_t i=0;i!=n_2D_v;i++){
      data_v_2D(i) = 0;
    }
    for (size_t i=0;i!=n_2D_w;i++){
      data_w_2D(i) = 0;
    }

    int index = 0;
    for (auto it = map_3D_2DU_set[i].first.begin(); it!=map_3D_2DU_set[i].first.end(); it++){
      double charge, charge_err;
      if (map_2D_ut_charge.find(*it)!=map_2D_ut_charge.end()){
	charge = std::get<0>(map_2D_ut_charge[*it]);
	charge_err = std::get<1>(map_2D_ut_charge[*it]);
      }else{
	charge = 100; // serve as regularization ...
	charge_err = 1000;
      }
      if (charge<100){
	charge = 100;
	charge_err = 1000;
      }
      // induction plane error is large, so they have less weights in the fit to decide X position
      double scaling = charge/charge_err * map_Udiv_fac[std::make_tuple(it->first, it->second, i)];
      
      if (map_3D_2DU_set[i].second < 0.5){
	if (map_3D_2DU_set[i].second!=0)
	  scaling *= pow(map_3D_2DU_set[i].second/0.5,1);
	else
	  scaling *= 0.05;
      }
      
      if (scaling!=0){
	data_u_2D(2*index) =  scaling * (it->first - offset_u);
	data_u_2D(2*index+1) = scaling * (it->second - offset_t);
      
	RU.insert(2*index, 1) = scaling * slope_yu; // Y--> U
	RU.insert(2*index, 2) = scaling * slope_zu; // Z--> U
	RU.insert(2*index+1,0) = scaling * slope_x; // X --> T
      }
      index ++;
    }

    index = 0;
    for (auto it = map_3D_2DV_set[i].first.begin(); it!=map_3D_2DV_set[i].first.end(); it++){
      double charge, charge_err;
      if (map_2D_vt_charge.find(*it)!=map_2D_vt_charge.end()){
	charge = std::get<0>(map_2D_vt_charge[*it]);
	charge_err = std::get<1>(map_2D_vt_charge[*it]);
      }else{
	charge = 100; // serve as regularization ...
	charge_err = 1000;
      }
      if (charge < 100){
	charge = 100;
	charge_err = 1000;
      }
      // induction plane error is large, so they have less weights in the fit to decide X position
      double scaling = charge/charge_err * map_Vdiv_fac[std::make_tuple(it->first, it->second, i)];
      if (map_3D_2DV_set[i].second < 0.5){
	if (map_3D_2DV_set[i].second!=0)
	  scaling *= pow(map_3D_2DV_set[i].second/0.5,1);
	else
	  scaling *= 0.05;
      }
      
      if (scaling!=0){
	data_v_2D(2*index) =  scaling * (it->first - offset_v);
	data_v_2D(2*index+1) = scaling * (it->second - offset_t);
      
	RV.insert(2*index, 1) = scaling * slope_yv; // Y--> V
	RV.insert(2*index, 2) = scaling * slope_zv; // Z--> V
	RV.insert(2*index+1,0) = scaling * slope_x; // X --> T
      }
      index ++;
    }

    index = 0;
    for (auto it = map_3D_2DW_set[i].first.begin(); it!=map_3D_2DW_set[i].first.end(); it++){
      double charge, charge_err;
      if (map_2D_wt_charge.find(*it)!=map_2D_wt_charge.end()){
	charge = std::get<0>(map_2D_wt_charge[*it]);
	charge_err = std::get<1>(map_2D_wt_charge[*it]);
      }else{
	charge = 100; // serve as regularization ...
	charge_err = 1000;
      }
      if (charge<100){
	charge = 100;
	charge_err = 1000;
      }
      // induction plane error is large, so they have less weights in the fit to decide X position
      double scaling = charge/charge_err * map_Wdiv_fac[std::make_tuple(it->first, it->second, i)];
      
      if (map_3D_2DW_set[i].second < 0.5){
	if (map_3D_2DW_set[i].second!=0)
	  scaling *= pow(map_3D_2DW_set[i].second/0.5,1);
	else
	  scaling *= 0.05;
      }
      if (scaling!=0){
	data_w_2D(2*index) =  scaling * (it->first - offset_w);
	data_w_2D(2*index+1) = scaling * (it->second - offset_t);
      
	RW.insert(2*index, 2) = scaling * slope_zw; // Z--> W
	RW.insert(2*index+1,0) = scaling * slope_x; // X --> T
      }
      index ++;
    }
    
     Eigen::SparseMatrix<double> RUT = Eigen::SparseMatrix<double>(RU.transpose());
     Eigen::SparseMatrix<double> RVT = Eigen::SparseMatrix<double>(RV.transpose());
     Eigen::SparseMatrix<double> RWT = Eigen::SparseMatrix<double>(RW.transpose());

     Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
     Eigen::VectorXd b = RUT * data_u_2D + RVT * data_v_2D + RWT * data_w_2D;// + PMatrixT * pos_3D_init ;
     
     Eigen::SparseMatrix<double> A =   RUT * RU + RVT * RV + RWT * RW ;// + PMatrixT*PMatrix;
     solver.compute(A);
     
     temp_pos_3D = solver.solveWithGuess(b,temp_pos_3D_init);

     //  std::cout << i << " " << temp_pos_3D(0) << " " << temp_pos_3D(1) << " " << temp_pos_3D(2) << " " << temp_pos_3D_init(0) << " " << temp_pos_3D_init(1) << " " << temp_pos_3D_init(2) << std::endl;
     
     if (std::isnan(solver.error())){
       pos_3D(3*i) = temp_pos_3D_init(0);
       pos_3D(3*i+1) = temp_pos_3D_init(1);
       pos_3D(3*i+2) = temp_pos_3D_init(2);
     }else{
       pos_3D(3*i) = temp_pos_3D(0);
       pos_3D(3*i+1) = temp_pos_3D(1);
       pos_3D(3*i+2) = temp_pos_3D(2);
     }
      
    //  std::cout << "Track Fitting: " << i << " " << temp_pos_3D(0) << " " << temp_pos_3D(1) << " " << temp_pos_3D(2) << std::endl;

  }
  
  fine_tracking_path.clear();
  pu.clear();
  pv.clear();
  pw.clear();
  pt.clear();

  WCP::PointVector temp_fine_tracking_path;

  int skip_count = 0;
  
  for (size_t i=0;i!=ps_vec.size();i++){
    Point p;
    p.x = pos_3D(3*i);
    p.y = pos_3D(3*i+1);
    p.z = pos_3D(3*i+2);

    bool flag_skip = skip_trajectory_point(p, i, i, ps_vec, map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge, fine_tracking_path, offset_t, slope_x,  offset_u,  slope_yu,  slope_zu,  offset_v,  slope_yv,  slope_zv,  offset_w, slope_yw,  slope_zw);

    // std::cout << "Skip: " << i << " " << flag_skip << std::endl;

    // protection ...
    if (flag_skip){
      skip_count ++;
      if (skip_count <=3)
	continue;
      else
	skip_count = 0;
    }
    
    temp_fine_tracking_path.push_back(ps_vec.at(i));
    
    fine_tracking_path.push_back(p);
    //fine_tracking_path.push_back(ps_vec.at(i));
    
    //	std::cout << ps_vec.at(i) << " " << p << " " << sqrt(pow(ps_vec.at(i).x-p.x,2)+pow(ps_vec.at(i).y-p.y,2)+pow(ps_vec.at(i).z-p.z,2))/units::mm <<std::endl;
  }

  //  std::cout << fine_tracking_path.size() << " " << ps_vec.size() << std::endl;
  
  for (size_t i=0; i!=fine_tracking_path.size(); i++){

    bool flag_replace = false;
    
    if (i != 0 && i+1 != fine_tracking_path.size()) {
      // -1 vs. +1
      double a = sqrt(pow(fine_tracking_path.at(i-1).x - fine_tracking_path.at(i).x,2)
		      +pow(fine_tracking_path.at(i-1).y - fine_tracking_path.at(i).y,2)
		      +pow(fine_tracking_path.at(i-1).z - fine_tracking_path.at(i).z,2));
      double b = sqrt(pow(fine_tracking_path.at(i+1).x - fine_tracking_path.at(i).x,2)
		      +pow(fine_tracking_path.at(i+1).y - fine_tracking_path.at(i).y,2)
		      +pow(fine_tracking_path.at(i+1).z - fine_tracking_path.at(i).z,2));
      double c = sqrt(pow(fine_tracking_path.at(i-1).x - fine_tracking_path.at(i+1).x,2)
		      +pow(fine_tracking_path.at(i-1).y - fine_tracking_path.at(i+1).y,2)
		      +pow(fine_tracking_path.at(i-1).z - fine_tracking_path.at(i+1).z,2));
      double s = (a+b+c)/2.;
      double area1 = sqrt(s*(s-a)*(s-b)*(s-c));

      a = sqrt(pow(fine_tracking_path.at(i-1).x - temp_fine_tracking_path.at(i).x,2)
	       +pow(fine_tracking_path.at(i-1).y - temp_fine_tracking_path.at(i).y,2)
	       +pow(fine_tracking_path.at(i-1).z - temp_fine_tracking_path.at(i).z,2));
      b = sqrt(pow(fine_tracking_path.at(i+1).x - temp_fine_tracking_path.at(i).x,2)
	       +pow(fine_tracking_path.at(i+1).y - temp_fine_tracking_path.at(i).y,2)
	       +pow(fine_tracking_path.at(i+1).z - temp_fine_tracking_path.at(i).z,2));
      s = (a+b+c)/2.;
      double area2 = sqrt(s*(s-a)*(s-b)*(s-c));

      //      std::cout << i << " A " << area1/c << " " << area2/c  << std::endl;

      if (area1 > 1.8*units::mm * c && area1 > 1.7 * area2)
	flag_replace = true;
    }
      
    // -2, +1
    if ((!flag_replace) && i>=2 && i+1 != fine_tracking_path.size()){
      double a = sqrt(pow(fine_tracking_path.at(i-2).x - fine_tracking_path.at(i).x,2)
		      +pow(fine_tracking_path.at(i-2).y - fine_tracking_path.at(i).y,2)
		      +pow(fine_tracking_path.at(i-2).z - fine_tracking_path.at(i).z,2));
      double b = sqrt(pow(fine_tracking_path.at(i+1).x - fine_tracking_path.at(i).x,2)
		      +pow(fine_tracking_path.at(i+1).y - fine_tracking_path.at(i).y,2)
		      +pow(fine_tracking_path.at(i+1).z - fine_tracking_path.at(i).z,2));
      double c = sqrt(pow(fine_tracking_path.at(i-2).x - fine_tracking_path.at(i+1).x,2)
		      +pow(fine_tracking_path.at(i-2).y - fine_tracking_path.at(i+1).y,2)
		      +pow(fine_tracking_path.at(i-2).z - fine_tracking_path.at(i+1).z,2));
      double s = (a+b+c)/2.;
      double area1 = sqrt(s*(s-a)*(s-b)*(s-c));

      a = sqrt(pow(fine_tracking_path.at(i-2).x - temp_fine_tracking_path.at(i).x,2)
	       +pow(fine_tracking_path.at(i-2).y - temp_fine_tracking_path.at(i).y,2)
	       +pow(fine_tracking_path.at(i-2).z - temp_fine_tracking_path.at(i).z,2));
      b = sqrt(pow(fine_tracking_path.at(i+1).x - temp_fine_tracking_path.at(i).x,2)
	       +pow(fine_tracking_path.at(i+1).y - temp_fine_tracking_path.at(i).y,2)
	       +pow(fine_tracking_path.at(i+1).z - temp_fine_tracking_path.at(i).z,2));
      s = (a+b+c)/2.;
      double area2 = sqrt(s*(s-a)*(s-b)*(s-c));
      //std::cout << i << " B " << area1/c << " " << area2/c  << std::endl;
      if (area1 > 1.8*units::mm * c && area1 > 1.7 * area2)
	flag_replace = true;	
     
      
    }
    
    
    // -1, +2
    if ((!flag_replace) && i>0 && i+2<fine_tracking_path.size()){
      double a = sqrt(pow(fine_tracking_path.at(i-1).x - fine_tracking_path.at(i).x,2)
		      +pow(fine_tracking_path.at(i-1).y - fine_tracking_path.at(i).y,2)
		      +pow(fine_tracking_path.at(i-1).z - fine_tracking_path.at(i).z,2));
      double b = sqrt(pow(fine_tracking_path.at(i+2).x - fine_tracking_path.at(i).x,2)
		      +pow(fine_tracking_path.at(i+2).y - fine_tracking_path.at(i).y,2)
		      +pow(fine_tracking_path.at(i+2).z - fine_tracking_path.at(i).z,2));
      double c = sqrt(pow(fine_tracking_path.at(i-1).x - fine_tracking_path.at(i+2).x,2)
		      +pow(fine_tracking_path.at(i-1).y - fine_tracking_path.at(i+2).y,2)
		      +pow(fine_tracking_path.at(i-1).z - fine_tracking_path.at(i+2).z,2));
      double s = (a+b+c)/2.;
      double area1 = sqrt(s*(s-a)*(s-b)*(s-c));

      a = sqrt(pow(fine_tracking_path.at(i-1).x - temp_fine_tracking_path.at(i).x,2)
	       +pow(fine_tracking_path.at(i-1).y - temp_fine_tracking_path.at(i).y,2)
	       +pow(fine_tracking_path.at(i-1).z - temp_fine_tracking_path.at(i).z,2));
      b = sqrt(pow(fine_tracking_path.at(i+2).x - temp_fine_tracking_path.at(i).x,2)
	       +pow(fine_tracking_path.at(i+2).y - temp_fine_tracking_path.at(i).y,2)
	       +pow(fine_tracking_path.at(i+2).z - temp_fine_tracking_path.at(i).z,2));
      s = (a+b+c)/2.;
      double area2 = sqrt(s*(s-a)*(s-b)*(s-c));

      //      std::cout << i << " C " << area1/c << " " << area2/c  << std::endl;
      
      if (area1 > 1.8*units::mm * c && area1 > 1.7 * area2)
	flag_replace = true;
    }

    
    if (flag_replace){
      fine_tracking_path.at(i) = temp_fine_tracking_path.at(i);
      //      std::cout << i << std::endl;
    }
    // if (area1 > area2 && area1 > 1.0*units::mm * c)
    // std::cout << i << " " << area1/c << " " << area2/c << std::endl;
  }
  

  for (size_t i=0; i!=fine_tracking_path.size(); i++){
    WCP::Point p = fine_tracking_path.at(i);
    pu.push_back(offset_u + 0.5 + (slope_yu * p.y + slope_zu * p.z));
    pv.push_back(offset_v + 0.5 + (slope_yv * p.y + slope_zv * p.z)+2400);
    pw.push_back(offset_w + 0.5 + (slope_yw * p.y + slope_zw * p.z)+4800);
    pt.push_back(offset_t + 0.5 + slope_x * p.x );
  }

  
  ps_vec = fine_tracking_path;
  
}

bool WCPPID::PR3DCluster::skip_trajectory_point(WCP::Point& p, int i, int index, WCP::PointVector& ps_vec, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, WCP::PointVector& fine_tracking_path,
						       double offset_t, double slope_x, double offset_u, double slope_yu, double slope_zu, double offset_v, double slope_yv, double slope_zv, double offset_w, double slope_yw, double slope_zw){
						       
  
  //examine the calculation
  int t1 = std::floor(offset_t + 0.5 + slope_x * p.x);
  int u1 = std::floor(offset_u + 0.5 + (slope_yu * p.y + slope_zu * p.z));
  int v1 = std::floor(offset_v + 0.5 + (slope_yv * p.y + slope_zv * p.z));
  int w1 = std::floor(offset_w + 0.5 + (slope_yw * p.y + slope_zw * p.z));
  
  double c1_u=0;
  {
    auto it = map_2D_ut_charge.find(std::make_pair(u1,t1));
    if (it != map_2D_ut_charge.end()){
      if (std::get<2>(it->second)!=0){
	c1_u += std::get<0>(it->second);
	//if (i==9) std::cout << std::get<0>(it->second) << " " ;
      }
    }
    auto it1 = map_2D_ut_charge.find(std::make_pair(u1,t1+1));
    if (it1 != map_2D_ut_charge.end()){
      if (std::get<2>(it1->second)!=0){
	c1_u += std::get<0>(it1->second);
	//if (i==9) std::cout << std::get<0>(it->second) << " " ;
      }
    }
    auto it2 = map_2D_ut_charge.find(std::make_pair(u1,t1-1));
    if (it2 != map_2D_ut_charge.end()){
      if (std::get<2>(it2->second)!=0){
	c1_u += std::get<0>(it2->second);
	//	if (i==9) std::cout << std::get<0>(it->second) << " " ;
      }
    }
    auto it3 = map_2D_ut_charge.find(std::make_pair(u1+1,t1));
    if (it3 != map_2D_ut_charge.end()){
      if (std::get<2>(it3->second)!=0)
	c1_u += std::get<0>(it3->second);
    }
    auto it4 = map_2D_ut_charge.find(std::make_pair(u1-1,t1));
    if (it4 != map_2D_ut_charge.end()){
      if (std::get<2>(it4->second)!=0)
	c1_u += std::get<0>(it4->second);
    }
  }
  
  double c1_v=0;
  {
    auto it = map_2D_vt_charge.find(std::make_pair(v1,t1));
    if (it != map_2D_vt_charge.end()){
      if (std::get<2>(it->second)!=0){
	c1_v += std::get<0>(it->second);
	//		if (i==9) std::cout << std::get<0>(it->second) << " " ;
      }
    }
    auto it1 = map_2D_vt_charge.find(std::make_pair(v1,t1+1));
    if (it1 != map_2D_vt_charge.end()){
      if (std::get<2>(it1->second)!=0){
	c1_v += std::get<0>(it1->second);
	//if (i==9) std::cout << std::get<0>(it1->second) << " " ;
      }
    }
    auto it2 = map_2D_vt_charge.find(std::make_pair(v1,t1-1));
    if (it2 != map_2D_vt_charge.end()){
      if (std::get<2>(it2->second)!=0){
	c1_v += std::get<0>(it2->second);
	//if (i==9) std::cout << std::get<0>(it2->second) << " " ;
      }
    }
    auto it3 = map_2D_vt_charge.find(std::make_pair(v1+1,t1));
    if (it3 != map_2D_vt_charge.end()){
      if (std::get<2>(it3->second)!=0){
	c1_v += std::get<0>(it3->second);
	//	if (i==9) std::cout << std::get<0>(it3->second) << " " ;
      }
    }
    auto it4 = map_2D_vt_charge.find(std::make_pair(v1-1,t1));
    if (it4 != map_2D_vt_charge.end()){
      if (std::get<2>(it4->second)!=0){
	c1_v += std::get<0>(it4->second);
	//if (i==9) std::cout << std::get<0>(it4->second) << " " ;
      }
    }
    //	    if (i==9) std::cout << std::endl;
  }
  
  
  double c1_w=0;
  {
    auto it = map_2D_wt_charge.find(std::make_pair(w1,t1));
    if (it != map_2D_wt_charge.end()){
      if (std::get<2>(it->second)!=0)
	c1_w += std::get<0>(it->second);
    }
    auto it1 = map_2D_wt_charge.find(std::make_pair(w1,t1+1));
    if (it1 != map_2D_wt_charge.end()){
      if (std::get<2>(it1->second)!=0)
	c1_w += std::get<0>(it1->second);
    }
    auto it2 = map_2D_wt_charge.find(std::make_pair(w1,t1-1));
    if (it2 != map_2D_wt_charge.end()){
      if (std::get<2>(it2->second)!=0)
	c1_w += std::get<0>(it2->second);
    }
    auto it3 = map_2D_wt_charge.find(std::make_pair(w1+1,t1));
    if (it3 != map_2D_wt_charge.end()){
      if (std::get<2>(it3->second)!=0)
	c1_w += std::get<0>(it3->second);
    }
    auto it4 = map_2D_wt_charge.find(std::make_pair(w1-1,t1));
    if (it4 != map_2D_wt_charge.end()){
      if (std::get<2>(it4->second)!=0)
	c1_w += std::get<0>(it4->second);
    }
  }
  
  
  int t2 = std::floor(offset_t + 0.5 + slope_x * ps_vec.at(i).x);
  int u2 = std::floor(offset_u + 0.5 + (slope_yu * ps_vec.at(i).y + slope_zu * ps_vec.at(i).z));
  int v2 = std::floor(offset_v + 0.5 + (slope_yv * ps_vec.at(i).y + slope_zv * ps_vec.at(i).z));
  int w2 = std::floor(offset_w + 0.5 + (slope_yw * ps_vec.at(i).y + slope_zw * ps_vec.at(i).z));
  
  
  double c2_u=0;
  {
    auto it = map_2D_ut_charge.find(std::make_pair(u2,t2));
    if (it != map_2D_ut_charge.end()){
      if (std::get<2>(it->second)!=0)
	c2_u += std::get<0>(it->second);
    }
    auto it1 = map_2D_ut_charge.find(std::make_pair(u2,t2+1));
    if (it1 != map_2D_ut_charge.end()){
      if (std::get<2>(it1->second)!=0)
	c2_u += std::get<0>(it1->second);
    }
    auto it2 = map_2D_ut_charge.find(std::make_pair(u2,t2-1));
    if (it2 != map_2D_ut_charge.end()){
      if (std::get<2>(it2->second)!=0)
	c2_u += std::get<0>(it2->second);
    }
    auto it3 = map_2D_ut_charge.find(std::make_pair(u2+1,t2));
    if (it3 != map_2D_ut_charge.end()){
      if (std::get<2>(it3->second)!=0)
	c2_u += std::get<0>(it3->second);
    }
    auto it4 = map_2D_ut_charge.find(std::make_pair(u2-1,t2));
    if (it4 != map_2D_ut_charge.end()){
      if (std::get<2>(it4->second)!=0)
	c2_u += std::get<0>(it4->second);
    }
  }
  
  double c2_v=0;
  {
    auto it = map_2D_vt_charge.find(std::make_pair(v2,t2));
    if (it != map_2D_vt_charge.end()){
      if (std::get<2>(it->second)!=0){
	c2_v += std::get<0>(it->second);
	//		if (i==9) std::cout << std::get<0>(it->second) << " ";
      }
    }
    auto it1 = map_2D_vt_charge.find(std::make_pair(v2,t2+1));
    if (it1 != map_2D_vt_charge.end()){
      if (std::get<2>(it1->second)!=0){
	c2_v += std::get<0>(it1->second);
	//if (i==9) std::cout << std::get<0>(it1->second) << " ";
      }
    }
    auto it2 = map_2D_vt_charge.find(std::make_pair(v2,t2-1));
    if (it2 != map_2D_vt_charge.end()){
      if (std::get<2>(it2->second)!=0){
	c2_v += std::get<0>(it2->second);
	//if (i==9) std::cout << std::get<0>(it2->second) << " ";
      }
    }
    auto it3 = map_2D_vt_charge.find(std::make_pair(v2+1,t2));
    if (it3 != map_2D_vt_charge.end()){
      if (std::get<2>(it3->second)!=0){
	c2_v += std::get<0>(it3->second);
	//if (i==9) std::cout << std::get<0>(it3->second) << " ";
      }
    }
    auto it4 = map_2D_vt_charge.find(std::make_pair(v2-1,t2));
    if (it4 != map_2D_vt_charge.end()){
      if (std::get<2>(it4->second)!=0){
	c2_v += std::get<0>(it4->second);
	//if (i==9) std::cout << std::get<0>(it4->second) << " ";
      }
    }
    //	    if (i==9) std::cout << std::endl;
  }
  
  double c2_w=0;
  {
    auto it = map_2D_wt_charge.find(std::make_pair(w2,t2));
    if (it != map_2D_wt_charge.end()){
      if (std::get<2>(it->second)!=0)
	c2_w += std::get<0>(it->second);
    }
    auto it1 = map_2D_wt_charge.find(std::make_pair(w2,t2+1));
    if (it1 != map_2D_wt_charge.end()){
      if (std::get<2>(it1->second)!=0)
	c2_w += std::get<0>(it1->second);
    }
    auto it2 = map_2D_wt_charge.find(std::make_pair(w2,t2-1));
    if (it2 != map_2D_wt_charge.end()){
      if (std::get<2>(it2->second)!=0)
	c2_w += std::get<0>(it2->second);
    }
    auto it3 = map_2D_wt_charge.find(std::make_pair(w2+1,t2));
    if (it3 != map_2D_wt_charge.end()){
      if (std::get<2>(it3->second)!=0)
	c2_w += std::get<0>(it3->second);
    }
    auto it4 = map_2D_wt_charge.find(std::make_pair(w2-1,t2));
    if (it4 != map_2D_wt_charge.end()){
      if (std::get<2>(it4->second)!=0)
	c2_w += std::get<0>(it4->second);
    }
  }
  /* if (i==10) */
  /*   std::cout << u1 << " " << v1 << " " << w1 << " " << t1 << " " */
  /* 	      << u2 << " " << v2 << " " << w2 << " " << t2 << " " */
  /* 	      <<  offset_v + (slope_yv * p.y + slope_zv * p.z) << " " << offset_v + (slope_yv * ps_vec.at(i).y + slope_zv * ps_vec.at(i).z) << std::endl; */
  
  // std::cout << "Skip inside: " << t1 * 4<< " " << u1 << " " << v1 << " " << w1 << " | " << t2*4 << " " << u2 << " " << v2 << " " << w2 << " | "  << c1_u << " " << c1_v << " " << c1_w << " " << c2_u << " " << c2_v << " " << c2_w << std::endl;


  double ratio=0;
  double ratio_1 = 1;
  if (c2_u!=0) {
    ratio += c1_u/c2_u;
    if (c1_u!=0)
      ratio_1 *= c1_u/c2_u;
    else
      ratio_1 *= 0.25;
    //    std::cout << i << " " << c1_u << " " << c2_u << std::endl;
  }else {
    ratio += 1; 
  }
  if (c2_v!=0) {
    ratio += c1_v/c2_v;
    if (c1_v!=0)
      ratio_1 *= c1_v/c2_v;
    else
      ratio_1 *= 0.25;
    //std::cout << i << " " << c1_v << " " << c2_v << std::endl;
  }else {
    ratio += 1;
  }
  if (c2_w!=0) {
    ratio += c1_w/c2_w;
    if (c1_w!=0)
      ratio_1 *= c1_w/c2_w;
    else
      ratio_1 *= 0.25;
    //    std::cout << i << " " << c1_w << " " << c2_w << std::endl;
  }else {
    ratio += 1;
  }

  // std::cout << "Inside: " << ratio << " " << ratio_1 << std::endl;

  /* if (ratio/3. >= 0.97 && ratio_1 < 1) */
  //  std::cout << i << " " << ratio/3. << " " << ratio_1 << std::endl; 
  //      if (i==88) std::cout << ratio/3. << std::endl;
  
  if (ratio/3. < 0.97 || ratio_1 < 0.75){
    // if (c1_u + c1_v + c1_w < (c2_u + c2_v + c2_w)) {
    /* std::cout << i << " " << ratio/3. << " " << (c1_u+c1_v+c1_w)*1./(c2_u + c2_v + c2_w) << " " << c1_u * 1.0/c2_u << " " << c1_v*1.0/c2_v << " " << c1_w*1.0/c2_w << " " << sqrt(pow(p.x-ps_vec.at(i).x,2) + pow(p.y-ps_vec.at(i).y,2) + pow(p.z-ps_vec.at(i).z,2)) << " " */
    /* 	      << u1 << " " << v1 << " " << w1 << " " << t1 << " " << u2 << " " << v2 << " " << w2 << " " << t2 << std::endl;// */
    p = ps_vec.at(i);
  }
  /* if (i==10) */
  /*   std::cout << c1_u + c1_v + c1_w << " " << c2_u + c2_v + c2_w << " " << ratio << " " << ratio_1 << std::endl; */
  
    
  // examine angle ...
  if (fine_tracking_path.size() >=2){
    TVector3 v1(fine_tracking_path.at(fine_tracking_path.size()-1).x - fine_tracking_path.at(fine_tracking_path.size()-2).x,
		fine_tracking_path.at(fine_tracking_path.size()-1).y - fine_tracking_path.at(fine_tracking_path.size()-2).y,
		fine_tracking_path.at(fine_tracking_path.size()-1).z - fine_tracking_path.at(fine_tracking_path.size()-2).z);
    TVector3 v2(p.x - fine_tracking_path.at(fine_tracking_path.size()-1).x,
		p.y - fine_tracking_path.at(fine_tracking_path.size()-1).y,
		p.z - fine_tracking_path.at(fine_tracking_path.size()-1).z);
    
    //  std::cout << ratio << " " << ratio_1 << " (" << p.x << " " << p.y << " " <<p.z <<") (" << fine_tracking_path.at(fine_tracking_path.size()-1).x << " " << fine_tracking_path.at(fine_tracking_path.size()-1).y << " " << fine_tracking_path.at(fine_tracking_path.size()-1).z << ") (" << fine_tracking_path.at(fine_tracking_path.size()-2).x << " " << fine_tracking_path.at(fine_tracking_path.size()-2).y << " " << fine_tracking_path.at(fine_tracking_path.size()-2).z << ")" << std::endl;
    //      if (i==56)

    TVector3 v3(ps_vec.at(i-1).x - ps_vec.at(i-2).x, ps_vec.at(i-1).y - ps_vec.at(i-2).y, ps_vec.at(i-1).z - ps_vec.at(i-2).z);
    TVector3 v4(ps_vec.at(i).x - ps_vec.at(i-1).x, ps_vec.at(i).y - ps_vec.at(i-1).y, ps_vec.at(i).z - ps_vec.at(i-1).z);

    double angle1 = v3.Angle(v4)/3.1415926*180.;

    
    double angle = v1.Angle(v2)/3.1415926*180.;

    /* if (i==10) */
    /*   std::cout << i << " " << angle << " " << angle1 << " " << ps_vec.size() << " " << v2.Mag() << " " << map_3D_2DU_set[index].second << " " << map_3D_2DV_set[index].second << " " << map_3D_2DW_set[index].second << std::endl; */

    // std::cout << "Inside: " << angle << " " << angle1 << " " << 0 << " " << v2.Mag() << std::endl;
    
    // related to the dead channels
    if (angle > 45 && ((map_3D_2DU_set[index].second==0 && map_3D_2DV_set[index].second==0) ||
		       (map_3D_2DU_set[index].second==0 && map_3D_2DW_set[index].second==0) ||
		       (map_3D_2DV_set[index].second==0 && map_3D_2DW_set[index].second==0)) )
      return true;
    
    if (angle > 160 || angle > angle1 + 90) {
      // completely fold back ...
      return true;
    }
    // protected against the last point ...
    if (i+1==ps_vec.size() && angle > 45 && v2.Mag() < 0.5*units::cm)
      return true;
    
    // short distance ...
    /* if (v2.Mag() < 0.45*units::cm && angle > 60) */
    /* 	continue; */
    // std::cout << i << " " << angle << " " << v2.Mag()/units::cm << " " << map_3D_2DU_set[i].second << " " << map_3D_2DV_set[i].second << " " << map_3D_2DW_set[i].second << " " << offset_t + 0.5 + slope_x * p.x << " " << offset_u + 0.5 + (slope_yu * p.y + slope_zu * p.z) << " " << offset_v + 0.5 + (slope_yv * p.y + slope_zv * p.z)+2400 << " " << offset_w + 0.5 + (slope_yw * p.y + slope_zw * p.z)+4800 << std::endl;
    
  }
  
  return false;
}

std::vector<float> WCPPID::PR3DCluster::examine_point_association(std::vector<int>& temp_results, std::set<std::pair<int,int> >& temp_2dut, std::set<std::pair<int,int> >& temp_2dvt, std::set<std::pair<int,int> >& temp_2dwt,
								       std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge, bool flag_end_point, double charge_cut){

  std::set<int> temp_types_u;
  std::set<int> temp_types_v;
  std::set<int> temp_types_w;
    
  std::set<std::pair<int,int> > saved_2dut;
  std::set<std::pair<int,int> > saved_2dvt;
  std::set<std::pair<int,int> > saved_2dwt;

  std::vector<float> results;
  results.resize(3,0);
  
  for (auto it = temp_2dut.begin(); it!=temp_2dut.end(); it++){
    auto it1 = map_2D_ut_charge.find(*it);
    if (it1!=map_2D_ut_charge.end() && std::get<0>(it1->second) > charge_cut ){
	temp_types_u.insert(std::get<2>(it1->second));
	if (std::get<2>(it1->second)==0) results.at(0)++;
	saved_2dut.insert(*it);
    }
  }

  for (auto it = temp_2dvt.begin(); it!=temp_2dvt.end(); it++){
    auto it1 = map_2D_vt_charge.find(*it);
    // std::cout << "V: " << it->second << " " << it->first + 2400 << std::endl;
    if (it1!=map_2D_vt_charge.end() && std::get<0>(it1->second) > charge_cut ){
      // std::cout << "V: " << it->second << " " << it->first +2400<< " " << std::get<0>(it1->second) << " " << charge_cut << std::endl;
      temp_types_v.insert(std::get<2>(it1->second));
      if (std::get<2>(it1->second)==0) results.at(1)++;
      saved_2dvt.insert(*it);
    }
  }

  //std::cout << temp_2dvt.size() << " " << saved_2dvt.size() << std::endl;
  
  for (auto it = temp_2dwt.begin(); it!=temp_2dwt.end(); it++){
    auto it1 = map_2D_wt_charge.find(*it);
    if (it1!=map_2D_wt_charge.end() && std::get<0>(it1->second) > charge_cut ){
      temp_types_w.insert(std::get<2>(it1->second));
      if (std::get<2>(it1->second)==0) results.at(2)++;
      saved_2dwt.insert(*it);
    }
  }
  if (temp_2dut.size()!=0)
    results.at(0) = (saved_2dut.size() - results.at(0)*1.0)/temp_2dut.size();
  else
    results.at(0) = 0;
  
  if (temp_2dvt.size()!=0)
    results.at(1) = (saved_2dvt.size() - results.at(1)*1.0)/temp_2dvt.size();
  else
    results.at(1) = 0;
  
  if (temp_2dwt.size()!=0)
    results.at(2) = (saved_2dwt.size() - results.at(2)*1.0)/temp_2dwt.size();
  else
    results.at(2) = 0;
  
  
  if (temp_types_u.find(0)!=temp_types_u.end() && temp_types_u.size()==1){
    saved_2dut.clear();
    results.at(0) = 0;
  }
  if (temp_types_v.find(0)!=temp_types_v.end() && temp_types_v.size()==1){
    saved_2dvt.clear();
    results.at(1) = 0;
  }
  if (temp_types_w.find(0)!=temp_types_w.end() && temp_types_w.size()==1){
    saved_2dwt.clear();
    results.at(2) = 0;
  }

  //  std::cout << saved_2dut.size() << " " << saved_2dvt.size() << " " << saved_2dwt.size() << " " << charge_cut << std::endl;

  // U and V planes are dead ...
  if (saved_2dut.size()==0 && saved_2dvt.size()==0 && saved_2dwt.size()!=0){
    //    temp_2dwt = saved_2dwt;
    saved_2dut.insert(std::make_pair(temp_results.at(1), temp_results.at(0)));
    saved_2dvt.insert(std::make_pair(temp_results.at(2), temp_results.at(0)));
    //std::cout << "haha1 " << temp_results.at(0) << " " << temp_results.at(1) << " " << temp_results.at(2) << " " << temp_results.at(3) << std::endl;
    // W plane check
    if (!flag_end_point)
    {
      std::pair<double, double> ave_pos = std::make_pair(0,0); // average positon
      double total_charge = 0;
      for (auto it1 = saved_2dwt.begin(); it1 != saved_2dwt.end(); it1++){
	auto it2 = map_2D_wt_charge.find(*it1);
	if (it2!=map_2D_wt_charge.end()){
	  ave_pos.first += it1->first * (std::get<0>(it2->second));
	  ave_pos.second += it1->second * (std::get<0>(it2->second));
	  total_charge += std::get<0>(it2->second);
	}
      }
      if (total_charge!=0){
	ave_pos.first /= total_charge;
	ave_pos.second /= total_charge;
      }
      double rms = 0;
      for (auto it1 = saved_2dwt.begin(); it1 != saved_2dwt.end(); it1++){
	rms += pow(it1->first-ave_pos.first,2) + pow(it1->second-ave_pos.second,2);
      }
      rms = sqrt(rms/saved_2dwt.size());

      if ( sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) > 0.75*rms && saved_2dwt.size() <=5 && saved_2dwt.size() < 0.2 * temp_2dwt.size()){
	saved_2dwt.clear();
	saved_2dwt.insert(std::make_pair(temp_results.at(3), temp_results.at(0)));
	results.at(2) = 0;
	/* std::cout << sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) << " " << rms << " " << saved_2dwt.size() << " " << temp_2dwt.size() << std::endl; */
      }
    }
  }else if (saved_2dut.size()==0 && saved_2dwt.size()==0 && saved_2dvt.size()!=0){
    // U and W planes are dead ...
    //temp_2dvt = saved_2dvt;
    saved_2dut.insert(std::make_pair(temp_results.at(1), temp_results.at(0)));
    saved_2dwt.insert(std::make_pair(temp_results.at(3), temp_results.at(0)));
    //std::cout << "haha2 " << temp_results.at(0) << " " << temp_results.at(1) << " " << temp_results.at(2) << " " << temp_results.at(3) << std::endl;
    // V plane check
    if (!flag_end_point)
    {
      std::pair<double, double> ave_pos = std::make_pair(0,0); // average positon
      double total_charge = 0;
      for (auto it1 = saved_2dvt.begin(); it1 != saved_2dvt.end(); it1++){
	auto it2 = map_2D_vt_charge.find(*it1);
	if (it2!=map_2D_vt_charge.end()){
	  ave_pos.first += it1->first * (std::get<0>(it2->second));
	  ave_pos.second += it1->second * (std::get<0>(it2->second));
	  total_charge += std::get<0>(it2->second);
	}
      }
      if (total_charge!=0){
	ave_pos.first /= total_charge;
	ave_pos.second /= total_charge;
      }
      double rms = 0;
      for (auto it1 = saved_2dvt.begin(); it1 != saved_2dvt.end(); it1++){
	rms += pow(it1->first-ave_pos.first,2) + pow(it1->second-ave_pos.second,2);
      }
      rms = sqrt(rms/saved_2dvt.size());

      if ( sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) > 0.75*rms && saved_2dvt.size() <=5 && saved_2dvt.size() < 0.2 * temp_2dvt.size()){
	saved_2dvt.clear();
	saved_2dvt.insert(std::make_pair(temp_results.at(2), temp_results.at(0)));
	results.at(1) = 0;
      }
    }
  }else if (saved_2dvt.size()==0 && saved_2dwt.size()==0 && saved_2dut.size()!=0){
    // V and W planes are dead ...
    //temp_2dut = saved_2dut;
    saved_2dvt.insert(std::make_pair(temp_results.at(2), temp_results.at(0)));
    saved_2dwt.insert(std::make_pair(temp_results.at(3), temp_results.at(0)));
    //std::cout << "haha3 " << temp_results.at(0) << " " << temp_results.at(1) << " " << temp_results.at(2) << " " << temp_results.at(3) << std::endl;
    // U plane check
    if (!flag_end_point)
    {
      std::pair<double, double> ave_pos = std::make_pair(0,0); // average positon
      double total_charge = 0;
      for (auto it1 = saved_2dut.begin(); it1 != saved_2dut.end(); it1++){
	auto it2 = map_2D_ut_charge.find(*it1);
	if (it2!=map_2D_ut_charge.end()){
	  ave_pos.first += it1->first * (std::get<0>(it2->second));
	  ave_pos.second += it1->second * (std::get<0>(it2->second));
	  total_charge += std::get<0>(it2->second);
	}
      }
      if (total_charge!=0){
	ave_pos.first /= total_charge;
	ave_pos.second /= total_charge;
      }
      double rms = 0;
      for (auto it1 = saved_2dut.begin(); it1 != saved_2dut.end(); it1++){
	rms += pow(it1->first-ave_pos.first,2) + pow(it1->second-ave_pos.second,2);
      }
      rms = sqrt(rms/saved_2dut.size());

      if ( sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) > 0.75*rms && saved_2dut.size() <=5 && saved_2dut.size() < 0.2 * temp_2dut.size()){
	saved_2dut.clear();
	saved_2dut.insert(std::make_pair(temp_results.at(1), temp_results.at(0)));
	results.at(0) = 0;
      }
    }
  }else if (saved_2dut.size()==0 && saved_2dwt.size()!=0 && saved_2dvt.size()!=0){
    // only U plane is dead ...  one of the other plane is problematic ...

    // W plane check
    if (!flag_end_point)
    {
      std::pair<double, double> ave_pos = std::make_pair(0,0); // average positon
      double total_charge = 0;
      for (auto it1 = saved_2dwt.begin(); it1 != saved_2dwt.end(); it1++){
	auto it2 = map_2D_wt_charge.find(*it1);
	if (it2!=map_2D_wt_charge.end()){
	  ave_pos.first += it1->first * (std::get<0>(it2->second));
	  ave_pos.second += it1->second * (std::get<0>(it2->second));
	  total_charge += std::get<0>(it2->second);
	}
      }
      if (total_charge!=0){
	ave_pos.first /= total_charge;
	ave_pos.second /= total_charge;
      }
      double rms = 0;
      for (auto it1 = saved_2dwt.begin(); it1 != saved_2dwt.end(); it1++){
	rms += pow(it1->first-ave_pos.first,2) + pow(it1->second-ave_pos.second,2);
      }
      rms = sqrt(rms/saved_2dwt.size());

      if ( sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) > 0.75*rms && saved_2dwt.size() <=5 && saved_2dwt.size() < 0.2 * temp_2dwt.size()){
	saved_2dwt.clear();
	saved_2dwt.insert(std::make_pair(temp_results.at(3), temp_results.at(0)));
	results.at(2) = 0;
	//	std::cout << "haha1 " << std::endl;
	/* std::cout << sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) << " " << rms << " " << saved_2dwt.size() << " " << temp_2dwt.size() << std::endl; */
      }
    }

    // V plane
    if (!flag_end_point)
    {
      std::pair<double, double> ave_pos = std::make_pair(0,0); // average positon
      double total_charge = 0;
      for (auto it1 = saved_2dvt.begin(); it1 != saved_2dvt.end(); it1++){
	auto it2 = map_2D_vt_charge.find(*it1);
	if (it2!=map_2D_vt_charge.end()){
	  ave_pos.first += it1->first * (std::get<0>(it2->second));
	  ave_pos.second += it1->second * (std::get<0>(it2->second));
	  total_charge += std::get<0>(it2->second);
	}
      }
      if (total_charge!=0){
	ave_pos.first /= total_charge;
	ave_pos.second /= total_charge;
      }
      double rms = 0;
      for (auto it1 = saved_2dvt.begin(); it1 != saved_2dvt.end(); it1++){
	rms += pow(it1->first-ave_pos.first,2) + pow(it1->second-ave_pos.second,2);
      }
      rms = sqrt(rms/saved_2dvt.size());

      if ( sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) > 0.75*rms && saved_2dvt.size() <=5 && saved_2dvt.size() < 0.2 * temp_2dvt.size()){
	saved_2dvt.clear();
	saved_2dvt.insert(std::make_pair(temp_results.at(2), temp_results.at(0)));
	results.at(1) = 0;
	//	std::cout << "haha2 " << std::endl;
      }
    }
    
  }else if (saved_2dvt.size()==0 && saved_2dut.size()!=0 && saved_2dwt.size()!=0){
    // only V plane is dead ... one of the other plane is problematic ...

    // U plane
    if (!flag_end_point)
    {
      std::pair<double, double> ave_pos = std::make_pair(0,0); // average positon
      double total_charge = 0;
      for (auto it1 = saved_2dut.begin(); it1 != saved_2dut.end(); it1++){
	auto it2 = map_2D_ut_charge.find(*it1);
	if (it2!=map_2D_ut_charge.end()){
	  ave_pos.first += it1->first * (std::get<0>(it2->second));
	  ave_pos.second += it1->second * (std::get<0>(it2->second));
	  total_charge += std::get<0>(it2->second);
	}
      }
      if (total_charge!=0){
	ave_pos.first /= total_charge;
	ave_pos.second /= total_charge;
      }
      double rms = 0;
      for (auto it1 = saved_2dut.begin(); it1 != saved_2dut.end(); it1++){
	rms += pow(it1->first-ave_pos.first,2) + pow(it1->second-ave_pos.second,2);
      }
      rms = sqrt(rms/saved_2dut.size());

      if ( sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) > 0.75*rms && saved_2dut.size() <=5 && saved_2dut.size() < 0.2 * temp_2dut.size()){
	saved_2dut.clear();
	saved_2dut.insert(std::make_pair(temp_results.at(1), temp_results.at(0)));
	results.at(0) = 0;
      }
    }

    
    // W plane
    if (!flag_end_point)
    {
      std::pair<double, double> ave_pos = std::make_pair(0,0); // average positon
      double total_charge = 0;
      for (auto it1 = saved_2dwt.begin(); it1 != saved_2dwt.end(); it1++){
	auto it2 = map_2D_wt_charge.find(*it1);
	if (it2!=map_2D_wt_charge.end()){
	  ave_pos.first += it1->first * (std::get<0>(it2->second));
	  ave_pos.second += it1->second * (std::get<0>(it2->second));
	  total_charge += std::get<0>(it2->second);
	}
      }
      if (total_charge!=0){
	ave_pos.first /= total_charge;
	ave_pos.second /= total_charge;
      }
      double rms = 0;
      for (auto it1 = saved_2dwt.begin(); it1 != saved_2dwt.end(); it1++){
	rms += pow(it1->first-ave_pos.first,2) + pow(it1->second-ave_pos.second,2);
      }
      rms = sqrt(rms/saved_2dwt.size());

      if ( sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) > 0.75*rms && saved_2dwt.size() <=5 && saved_2dwt.size() < 0.2 * temp_2dwt.size()){
	saved_2dwt.clear();
	saved_2dwt.insert(std::make_pair(temp_results.at(3), temp_results.at(0)));
	results.at(2) = 0;
	/* std::cout << sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) << " " << rms << " " << saved_2dwt.size() << " " << temp_2dwt.size() << std::endl; */
      }
    }
    
  }else if (saved_2dwt.size()==0 && saved_2dut.size()!=0 && saved_2dvt.size()!=0){
    // only W plane is dead ... one of the other plane is problematic ...

    // U plane
    if (!flag_end_point)
    {
      std::pair<double, double> ave_pos = std::make_pair(0,0); // average positon
      double total_charge = 0;
      for (auto it1 = saved_2dut.begin(); it1 != saved_2dut.end(); it1++){
	auto it2 = map_2D_ut_charge.find(*it1);
	if (it2!=map_2D_ut_charge.end()){
	  ave_pos.first += it1->first * (std::get<0>(it2->second));
	  ave_pos.second += it1->second * (std::get<0>(it2->second));
	  total_charge += std::get<0>(it2->second);
	}
      }
      if (total_charge!=0){
	ave_pos.first /= total_charge;
	ave_pos.second /= total_charge;
      }
      double rms = 0;
      for (auto it1 = saved_2dut.begin(); it1 != saved_2dut.end(); it1++){
	rms += pow(it1->first-ave_pos.first,2) + pow(it1->second-ave_pos.second,2);
      }
      rms = sqrt(rms/saved_2dut.size());

      if ( sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) > 0.75*rms && saved_2dut.size() <=5 && saved_2dut.size() < 0.2 * temp_2dut.size()){
	saved_2dut.clear();
	saved_2dut.insert(std::make_pair(temp_results.at(1), temp_results.at(0)));
	results.at(0) = 0;
      }
    }
    
    // V plane
    if (!flag_end_point)
    {
      std::pair<double, double> ave_pos = std::make_pair(0,0); // average positon
      double total_charge = 0;
      for (auto it1 = saved_2dvt.begin(); it1 != saved_2dvt.end(); it1++){
	auto it2 = map_2D_vt_charge.find(*it1);
	if (it2!=map_2D_vt_charge.end()){
	  ave_pos.first += it1->first * (std::get<0>(it2->second));
	  ave_pos.second += it1->second * (std::get<0>(it2->second));
	  total_charge += std::get<0>(it2->second);
	}
      }
      if (total_charge!=0){
	ave_pos.first /= total_charge;
	ave_pos.second /= total_charge;
      }
      double rms = 0;
      for (auto it1 = saved_2dvt.begin(); it1 != saved_2dvt.end(); it1++){
	rms += pow(it1->first-ave_pos.first,2) + pow(it1->second-ave_pos.second,2);
      }
      rms = sqrt(rms/saved_2dvt.size());

      if ( sqrt(pow(ave_pos.first - temp_results.at(3),2) + pow(ave_pos.second - temp_results.at(0),2)) > 0.75*rms && saved_2dvt.size() <=5 && saved_2dvt.size() < 0.2 * temp_2dvt.size()){
	saved_2dvt.clear();
	saved_2dvt.insert(std::make_pair(temp_results.at(2), temp_results.at(0)));
	results.at(1) = 0;
      }
    }
  }else if (saved_2dwt.size()==0 && saved_2dut.size()==0 && saved_2dvt.size()==0){
    saved_2dut.insert(std::make_pair(temp_results.at(1), temp_results.at(0)));
    saved_2dvt.insert(std::make_pair(temp_results.at(2), temp_results.at(0)));
    saved_2dwt.insert(std::make_pair(temp_results.at(3), temp_results.at(0)));
  }
  
  

  // std::cout << " " << saved_2dut.size() << " " << saved_2dvt.size() << " " << saved_2dwt.size() << " " << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << std::endl;
  
  temp_2dut = saved_2dut;
  temp_2dvt = saved_2dvt;
  temp_2dwt = saved_2dwt;
  
    
  
  return results;
}

void WCPPID::PR3DCluster::form_point_association(WCP::Point &p, std::set<std::pair<int,int> >& temp_2dut, std::set<std::pair<int,int> >& temp_2dvt, std::set<std::pair<int,int> >& temp_2dwt, WCP::ToyCTPointCloud& ct_point_cloud, double dis_cut, int nlevel, double time_cut ){
  // global information
  TPCParams& mp = Singleton<TPCParams>::Instance();
  double pitch_u = mp.get_pitch_u();
  double pitch_v = mp.get_pitch_v();
  double pitch_w = mp.get_pitch_w();
  double angle_u = mp.get_angle_u();
  double angle_v = mp.get_angle_v();
  double angle_w = mp.get_angle_w();
  double time_slice_width = mp.get_ts_width();
  double first_u_dis = mp.get_first_u_dis();
  double first_v_dis = mp.get_first_v_dis();
  double first_w_dis = mp.get_first_w_dis();
  
  float coef1 = 2 * pow(sin(angle_u),2);
  float coef2 = 2 * (pow(sin(angle_u),2) - pow(cos(angle_u),2));

  typedef boost::property_map<MCUGraph, boost::vertex_index_t>::type IndexMap;
  typedef boost::graph_traits<MCUGraph>::adjacency_iterator adjacency_iterator;
  
  // original point cloud ...
  if (point_cloud!=0 && graph!=0){
    WCP::WCPointCloud<double>& cloud = point_cloud->get_cloud();
    IndexMap index = get(boost::vertex_index,*graph);
    if (cloud.pts.size()>0){
      WCP::WCPointCloud<double>::WCPoint wcp = point_cloud->get_closest_wcpoint(p);
      double temp_dis = sqrt(pow(wcp.x-p.x,2)+pow(wcp.y-p.y,2)+pow(wcp.z-p.z,2));
      
      //std::cout << temp_dis/units::cm << " " << dis_cut/units::cm << std::endl;
      if (temp_dis < dis_cut){
	std::set<int> total_vertices_found;
	std::set<int> vertices_to_be_examined;
	std::set<int> vertices_saved_for_next;
	total_vertices_found.insert(wcp.index);
	vertices_to_be_examined.insert(wcp.index);

	//	std::cout << cloud.pts.size() << std::endl;
	
	for (int j=0;j!=nlevel;j++){
	  for (auto it = vertices_to_be_examined.begin(); it!=vertices_to_be_examined.end(); it++){
	    int temp_current_index = (*it);

	    //	    std::cout << temp_current_index << " " << cloud.pts.size() << std::endl;

	    std::pair<adjacency_iterator, adjacency_iterator> neighbors = boost::adjacent_vertices(vertex(temp_current_index,*graph),*graph);
	    for (; neighbors.first!=neighbors.second; ++neighbors.first){
	      //std::cout << *neighbors.first << " " << *neighbors.second << std::endl;
	      if (total_vertices_found.find(index(*neighbors.first))==total_vertices_found.end()){
		total_vertices_found.insert(index(*neighbors.first));
		vertices_saved_for_next.insert(index(*neighbors.first));
	      }
	    }
	  }
	  vertices_to_be_examined = vertices_saved_for_next;
	}
	
	SMGCSet nearby_mcells_set;
	for (auto it = total_vertices_found.begin(); it!=total_vertices_found.end(); it++){
	  SlimMergeGeomCell *mcell = cloud.pts[*it].mcell;
	  nearby_mcells_set.insert(mcell);
	}
	

	int cur_time_slice = cloud.pts[wcp.index].mcell->GetTimeSlice();
	int cur_wire_u = cloud.pts[wcp.index].index_u;
	int cur_wire_v = cloud.pts[wcp.index].index_v;
	int cur_wire_w = cloud.pts[wcp.index].index_w;

	  // std::cout << nearby_mcells_set.size() << " " << cur_time_slice << " " << cur_wire_u << " " << cur_wire_v << " " << cur_wire_w << " " << dis_cut << std::endl;

	
	double dis_cut_u = dis_cut;
	double dis_cut_v = dis_cut;
	double dis_cut_w = dis_cut;
	
	
	double max_time_slice_u = 0;
	double max_time_slice_v = 0;
	double max_time_slice_w = 0;
	for (auto it = nearby_mcells_set.begin(); it!=nearby_mcells_set.end(); it++){
	  SlimMergeGeomCell *mcell = *it;
	  int this_time_slice = mcell->GetTimeSlice();
	  GeomWireSelection uwires = mcell->get_uwires();
	  GeomWireSelection vwires = mcell->get_vwires();
	  GeomWireSelection wwires = mcell->get_wwires();
	  for (auto it1 = uwires.begin(); it1!=uwires.end(); it1++){
	    if (fabs((*it1)->index()-cur_wire_u)<=1)
	      if (fabs(this_time_slice-cur_time_slice)>max_time_slice_u)
		max_time_slice_u = fabs(this_time_slice-cur_time_slice);
	  }
	  for (auto it1 = vwires.begin(); it1!=vwires.end(); it1++){
	    if (fabs((*it1)->index()-cur_wire_v)<=1)
	      if (fabs(this_time_slice-cur_time_slice)>max_time_slice_v)
		max_time_slice_v = fabs(this_time_slice-cur_time_slice);
	  }
	  for (auto it1 = wwires.begin(); it1!=wwires.end(); it1++){
	    if (fabs((*it1)->index()-cur_wire_w)<=1)
	      if (fabs(this_time_slice-cur_time_slice)>max_time_slice_w)
		max_time_slice_w = fabs(this_time_slice-cur_time_slice);
	  }
	  
	}
	
	if (max_time_slice_u * time_slice_width*1.2 < dis_cut_u)
	  dis_cut_u = max_time_slice_u * time_slice_width*1.2;
	if (max_time_slice_v * time_slice_width*1.2 < dis_cut_v)
	  dis_cut_v = max_time_slice_v * time_slice_width*1.2;
	if (max_time_slice_w * time_slice_width*1.2 < dis_cut_w)
	  dis_cut_w = max_time_slice_w * time_slice_width*1.2;
	
    // std::cout << dis_cut_u << " " << dis_cut_v << " " << dis_cut_w << std::endl;
	
	for (auto it = nearby_mcells_set.begin(); it!=nearby_mcells_set.end(); it++){
	  SlimMergeGeomCell *mcell = *it;
	  int this_time_slice = mcell->GetTimeSlice();

    // std::cout << "Blob info: " << mcell->get_uwires().front()->index() << " " << mcell->get_uwires().back()->index()+1 << " " << mcell->get_vwires().front()->index() << " " << mcell->get_vwires().back()->index()+1 << " " << mcell->get_wwires().front()->index() << " " << mcell->get_wwires().back()->index()+1 << " " << this_time_slice << std::endl;

	  double rem_dis_cut_u = pow(dis_cut_u,2) - pow((cur_time_slice - this_time_slice)*time_slice_width,2);
	  double rem_dis_cut_v = pow(dis_cut_v,2) - pow((cur_time_slice - this_time_slice)*time_slice_width,2);
	  double rem_dis_cut_w = pow(dis_cut_w,2) - pow((cur_time_slice - this_time_slice)*time_slice_width,2);
    
    // std::cout << rem_dis_cut_u << " " << rem_dis_cut_v << " " << rem_dis_cut_w << " " << cur_time_slice << " " <<this_time_slice << " " << time_cut << std::endl;

	  if ((rem_dis_cut_u>0 || rem_dis_cut_v >0 || rem_dis_cut_w > 0 ) && fabs(cur_time_slice-this_time_slice)<=time_cut){
	    GeomWireSelection uwires = mcell->get_uwires();
	    GeomWireSelection vwires = mcell->get_vwires();
	    GeomWireSelection wwires = mcell->get_wwires();
	    float min_u_dis;
	    if (cur_wire_u < uwires.front()->index()){
	      min_u_dis = uwires.front()->index()-cur_wire_u;
	    }else if (cur_wire_u >= uwires.front()->index() &&
		      cur_wire_u <= uwires.back()->index()){
	      min_u_dis = 0;
	    }else{
	      min_u_dis = cur_wire_u-uwires.back()->index();
	    }
	    float min_v_dis;
	    if (cur_wire_v < vwires.front()->index()){
	      min_v_dis = vwires.front()->index()-cur_wire_v;
	    }else if (cur_wire_v >= vwires.front()->index() &&
		      cur_wire_v <= vwires.back()->index()){
	      min_v_dis = 0;
	    }else{
	      min_v_dis = cur_wire_v-vwires.back()->index();
	    }
	    float min_w_dis;
	    if (cur_wire_w < wwires.front()->index()){
	      min_w_dis = wwires.front()->index()-cur_wire_w;
	    }else if (cur_wire_w >= wwires.front()->index() &&
		      cur_wire_w <= wwires.back()->index()){
	      min_w_dis = 0;
	    }else{
	      min_w_dis = cur_wire_w-wwires.back()->index();
	    }
	    float range_u = rem_dis_cut_u*coef1 - pow(min_v_dis*pitch_v,2) - coef2*pow(min_w_dis*pitch_w,2);
	    float range_v = rem_dis_cut_v*coef1 - pow(min_u_dis*pitch_u,2) - coef2*pow(min_w_dis*pitch_w,2);
	    float range_w = (rem_dis_cut_w*coef1 - pow(min_u_dis*pitch_u,2) - pow(min_v_dis*pitch_v,2))/coef2;
	    
      // std::cout << "Cuts: " << range_u << " " << range_v << " " << range_w << " " << min_u_dis << " " << min_v_dis << " " << min_w_dis << " " << coef1 << " " << coef2 << std::endl;

	    if ( range_u > 0 && range_v >0 && range_w > 0){
	      float low_u_limit = cur_wire_u - sqrt(range_u)/pitch_u;
	      float high_u_limit = cur_wire_u + sqrt(range_u)/pitch_u;
	      float low_v_limit = cur_wire_v - sqrt(range_v)/pitch_v;
	      float high_v_limit = cur_wire_v + sqrt(range_v)/pitch_v;
	      float low_w_limit = cur_wire_w - sqrt(range_w)/pitch_w;
	      float high_w_limit = cur_wire_w + sqrt(range_w)/pitch_w;

        // std::cout << low_u_limit << " " << high_u_limit << " "
        //                       << low_v_limit << " " << high_v_limit << " "
        //                       << low_w_limit << " " << high_w_limit << " " << this_time_slice << std::endl;

	      for (int j = std::round(low_u_limit); j<= std::round(high_u_limit); j++){
		//auto it1 = map_2D_ut_charge.find(std::make_pair(j,this_time_slice));
		//if (it1!=map_2D_ut_charge.end() && std::get<0>(it1->second) > 0 )
		temp_2dut.insert(std::make_pair(j,this_time_slice));
	      }
	      for (int j=std::round(low_v_limit); j<=std::round(high_v_limit); j++){
		//auto it1 = map_2D_vt_charge.find(std::make_pair(j,this_time_slice));
		//if (it1!=map_2D_vt_charge.end() && std::get<0>(it1->second) > 0 )
		temp_2dvt.insert(std::make_pair(j,this_time_slice));
	      }
	      for (int j=std::round(low_w_limit); j<=std::round(high_w_limit); j++){
		//auto it1 = map_2D_wt_charge.find(std::make_pair(j,this_time_slice));
		//if (it1!=map_2D_wt_charge.end() && std::get<0>(it1->second) > 0 )
		temp_2dwt.insert(std::make_pair(j,this_time_slice));
	      }
	    } // cuts
	  } // cuts
	} // loop over all mcells
      } // find a good closest point?
    } // point cloud exist
  }
    
  // std::cout << "Pixels: " << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << std::endl;


  // steiner tree point cloud ...
  if (point_cloud_steiner!=0 && graph_steiner!=0 ){
    WCP::WCPointCloud<double>& cloud = point_cloud_steiner->get_cloud();
    IndexMap index = get(boost::vertex_index,*graph_steiner);



    if (cloud.pts.size()>0){
      WCP::WCPointCloud<double>::WCPoint wcp = point_cloud_steiner->get_closest_wcpoint(p);
      double temp_dis = sqrt(pow(wcp.x-p.x,2)+pow(wcp.y-p.y,2)+pow(wcp.z-p.z,2));
      
      // std::cout << "Steiner graph: vertices = " << boost::num_vertices(*graph_steiner)
      //       << ", edges = " << boost::num_edges(*graph_steiner) << std::endl;
      // std::cout << "Steiner: " << temp_dis << " " << dis_cut << " " << p.x << " " << p.y << " " << p.z << " " << wcp.x << " " << wcp.y << " " << wcp.z << std::endl;
      if (temp_dis < dis_cut){
        std::set<int> total_vertices_found;
        std::set<int> vertices_to_be_examined;
        std::set<int> vertices_saved_for_next;
        total_vertices_found.insert(wcp.index);
        vertices_to_be_examined.insert(wcp.index);
        
        for (int j=0;j!=nlevel;j++){
          for (auto it = vertices_to_be_examined.begin(); it!=vertices_to_be_examined.end(); it++){
            int temp_current_index = (*it);
            std::pair<adjacency_iterator, adjacency_iterator> neighbors = boost::adjacent_vertices(vertex(temp_current_index,*graph_steiner),*graph_steiner);

            // std::cout << "Level " << j << " " << temp_current_index << " " << std::endl;

            for (; neighbors.first!=neighbors.second; ++neighbors.first){
              //std::cout << *neighbors.first << " " << *neighbors.second << std::endl;
              if (total_vertices_found.find(index(*neighbors.first))==total_vertices_found.end()){
                total_vertices_found.insert(index(*neighbors.first));
                vertices_saved_for_next.insert(index(*neighbors.first));
              }
            }
          }
          vertices_to_be_examined = vertices_saved_for_next;
        }
	
        WCP::Point temp_p(wcp.x, wcp.y, wcp.z);
        std::vector<int> temp_results = ct_point_cloud.convert_3Dpoint_time_ch(temp_p);
        // std::cout << cloud.pts[wcp.index].index_u << " "  << temp_results.at(0) << " " << temp_results.at(1) << std::endl;
        //	if (wcp.mcell!=0)
        // std::cout << temp_results.at(0) << " " << wcp.mcell->GetTimeSlice() << std::endl;
        int cur_time_slice = temp_results.at(0);
        int cur_wire_u = cloud.pts[wcp.index].index_u;
        int cur_wire_v = cloud.pts[wcp.index].index_v;
        int cur_wire_w = cloud.pts[wcp.index].index_w;

        // std::cout << "B: " << cur_time_slice << " " << cur_wire_u << " " << cur_wire_v << " " << cur_wire_w << " " << total_vertices_found.size() << " " << nlevel << std::endl;
	
        SMGCSet nearby_mcells_set;
        std::vector<int> point_indices;
        std::vector<int> point_timeslices;
        
        for (auto it = total_vertices_found.begin(); it!=total_vertices_found.end(); it++){
          SlimMergeGeomCell *mcell = cloud.pts[*it].mcell;
          if (mcell!=0){
            nearby_mcells_set.insert(mcell);
          }else{
            temp_p.x = cloud.pts[*it].x;
            temp_p.y = cloud.pts[*it].y;
            temp_p.z = cloud.pts[*it].z;
            temp_results = ct_point_cloud.convert_3Dpoint_time_ch(temp_p);
            
            point_indices.push_back(*it);
            point_timeslices.push_back(temp_results.at(0));
          }
        }
	
	
	
	double dis_cut_u = dis_cut;
	double dis_cut_v = dis_cut;
	double dis_cut_w = dis_cut;
	
	double max_time_slice_u = 0;
	double max_time_slice_v = 0;
	double max_time_slice_w = 0;
	
	for (auto it = nearby_mcells_set.begin(); it!=nearby_mcells_set.end(); it++){
	  SlimMergeGeomCell *mcell = *it;
	  int this_time_slice = mcell->GetTimeSlice();
	  GeomWireSelection uwires = mcell->get_uwires();
	  GeomWireSelection vwires = mcell->get_vwires();
	  GeomWireSelection wwires = mcell->get_wwires();
	  for (auto it1 = uwires.begin(); it1!=uwires.end(); it1++){
	    if (fabs((*it1)->index()-cur_wire_u)<1)
	      if (fabs(this_time_slice-cur_time_slice)>max_time_slice_u)
		max_time_slice_u = fabs(this_time_slice-cur_time_slice);
	  }
	  for (auto it1 = vwires.begin(); it1!=vwires.end(); it1++){
	    if (fabs((*it1)->index()-cur_wire_v)<1)
	      if (fabs(this_time_slice-cur_time_slice)>max_time_slice_v)
		max_time_slice_v = fabs(this_time_slice-cur_time_slice);
	  }
	  for (auto it1 = wwires.begin(); it1!=wwires.end(); it1++){
	    if (fabs((*it1)->index()-cur_wire_w)<1)
	      if (fabs(this_time_slice-cur_time_slice)>max_time_slice_w)
		max_time_slice_w = fabs(this_time_slice-cur_time_slice);
	  }
	}
	for (size_t i=0; i!=point_indices.size(); i++){
	  int this_time_slice = point_timeslices.at(i);
	  if (fabs(cloud.pts[point_indices.at(i)].index_u - cur_wire_u)<=1){
	    if (fabs(this_time_slice-cur_time_slice)>max_time_slice_u)
	      max_time_slice_u = fabs(this_time_slice-cur_time_slice);
	  }
	  if (fabs(cloud.pts[point_indices.at(i)].index_v - cur_wire_v)<=1){
	    if (fabs(this_time_slice-cur_time_slice)>max_time_slice_v)
	      max_time_slice_v = fabs(this_time_slice-cur_time_slice);
	  }
	  if (fabs(cloud.pts[point_indices.at(i)].index_w - cur_wire_w)<=1){
	    if (fabs(this_time_slice-cur_time_slice)>max_time_slice_w)
	      max_time_slice_w = fabs(this_time_slice-cur_time_slice);
	  }
	}
	
	if (max_time_slice_u * time_slice_width*1.2 < dis_cut_u)
	  dis_cut_u = max_time_slice_u * time_slice_width*1.2;
	if (max_time_slice_v * time_slice_width*1.2 < dis_cut_v)
	  dis_cut_v = max_time_slice_v * time_slice_width*1.2;
	if (max_time_slice_w * time_slice_width*1.2 < dis_cut_w)
	  dis_cut_w = max_time_slice_w * time_slice_width*1.2;
	
  // std::cout << "Steiner dis: " << dis_cut_u << " " << dis_cut_v << " " << dis_cut_w << " " << nearby_mcells_set.size() << std::endl;


	// actual cut ...
	for (auto it = nearby_mcells_set.begin(); it!=nearby_mcells_set.end(); it++){
	  SlimMergeGeomCell *mcell = *it;
	  int this_time_slice = mcell->GetTimeSlice();
	  
	  double rem_dis_cut_u = pow(dis_cut_u,2) - pow((cur_time_slice - this_time_slice)*time_slice_width,2);
	  double rem_dis_cut_v = pow(dis_cut_v,2) - pow((cur_time_slice - this_time_slice)*time_slice_width,2);
	  double rem_dis_cut_w = pow(dis_cut_w,2) - pow((cur_time_slice - this_time_slice)*time_slice_width,2);

    // std::cout << rem_dis_cut_u << " " << rem_dis_cut_v << " " << rem_dis_cut_w << " " << cur_time_slice << " " <<this_time_slice << " " << time_cut << std::endl;
	  if ((rem_dis_cut_u>0 || rem_dis_cut_v >0 || rem_dis_cut_w > 0 ) && fabs(cur_time_slice-this_time_slice)<=time_cut){
	    GeomWireSelection uwires = mcell->get_uwires();
	    GeomWireSelection vwires = mcell->get_vwires();
	    GeomWireSelection wwires = mcell->get_wwires();
	    float min_u_dis;
	    if (cur_wire_u < uwires.front()->index()){
	      min_u_dis = uwires.front()->index()-cur_wire_u;
	    }else if (cur_wire_u >= uwires.front()->index() &&
		      cur_wire_u <= uwires.back()->index()){
	      min_u_dis = 0;
	    }else{
	      min_u_dis = cur_wire_u-uwires.back()->index();
	    }
	    float min_v_dis;
	    if (cur_wire_v < vwires.front()->index()){
	      min_v_dis = vwires.front()->index()-cur_wire_v;
	    }else if (cur_wire_v >= vwires.front()->index() &&
		      cur_wire_v <= vwires.back()->index()){
	      min_v_dis = 0;
	    }else{
	      min_v_dis = cur_wire_v-vwires.back()->index();
	    }
	    float min_w_dis;
	    if (cur_wire_w < wwires.front()->index()){
	      min_w_dis = wwires.front()->index()-cur_wire_w;
	    }else if (cur_wire_w >= wwires.front()->index() &&
		      cur_wire_w <= wwires.back()->index()){
	      min_w_dis = 0;
	    }else{
	      min_w_dis = cur_wire_w-wwires.back()->index();
	    }
	    float range_u = rem_dis_cut_u*coef1 - pow(min_v_dis*pitch_v,2) - coef2*pow(min_w_dis*pitch_w,2);
	    float range_v = rem_dis_cut_v*coef1 - pow(min_u_dis*pitch_u,2) - coef2*pow(min_w_dis*pitch_w,2);
	    float range_w = (rem_dis_cut_w*coef1 - pow(min_u_dis*pitch_u,2) - pow(min_v_dis*pitch_v,2))/coef2;
	    
	    if ( range_u > 0 && range_v >0 && range_w > 0){
	      float low_u_limit = cur_wire_u - sqrt(range_u)/pitch_u;
	      float high_u_limit = cur_wire_u + sqrt(range_u)/pitch_u;
	      float low_v_limit = cur_wire_v - sqrt(range_v)/pitch_v;
	      float high_v_limit = cur_wire_v + sqrt(range_v)/pitch_v;
	      float low_w_limit = cur_wire_w - sqrt(range_w)/pitch_w;
	      float high_w_limit = cur_wire_w + sqrt(range_w)/pitch_w;
	      for (int j = std::round(low_u_limit); j<= std::round(high_u_limit); j++){
		temp_2dut.insert(std::make_pair(j,this_time_slice));
	      }
	      for (int j=std::round(low_v_limit); j<=std::round(high_v_limit); j++){
		temp_2dvt.insert(std::make_pair(j,this_time_slice));
	      }
	      for (int j=std::round(low_w_limit); j<=std::round(high_w_limit); j++){
		temp_2dwt.insert(std::make_pair(j,this_time_slice));
	      }
	    } // cuts
	  } // cuts
	} // loop over all mcells
	
    // std::cout << "Pixels 1: " << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << std::endl;



	for (size_t i=0; i!=point_indices.size(); i++){

	  int this_time_slice = point_timeslices.at(i);
	  int this_index_u = cloud.pts[point_indices.at(i)].index_u;
	  int this_index_v = cloud.pts[point_indices.at(i)].index_v;
	  int this_index_w = cloud.pts[point_indices.at(i)].index_w;
	  
	  double rem_dis_cut_u = pow(dis_cut_u,2) - pow((cur_time_slice - this_time_slice)*time_slice_width,2);
	  double rem_dis_cut_v = pow(dis_cut_v,2) - pow((cur_time_slice - this_time_slice)*time_slice_width,2);
	  double rem_dis_cut_w = pow(dis_cut_w,2) - pow((cur_time_slice - this_time_slice)*time_slice_width,2);

    // std::cout << i << " " << rem_dis_cut_u << " " << rem_dis_cut_v << " " << rem_dis_cut_w << std::endl;

	  if ((rem_dis_cut_u>0 || rem_dis_cut_v >0 || rem_dis_cut_w > 0 ) && fabs(cur_time_slice-this_time_slice)<=time_cut){
	    float min_u_dis;
	    if (cur_wire_u < this_index_u-1){
	      min_u_dis = this_index_u -1 - cur_wire_u;
	    }else if (cur_wire_u > this_index_u + 1){
	      min_u_dis = cur_wire_u - this_index_u - 1;
	    }else{
	      min_u_dis = 0;
	    }
	    float min_v_dis;
	    if (cur_wire_v < this_index_v-1){
	      min_v_dis = this_index_v -1 - cur_wire_v;
	    }else if (cur_wire_v > this_index_v + 1){
	      min_v_dis = cur_wire_v - this_index_v -1 ;
	    }else{
	      min_v_dis = 0;
	    }
	    float min_w_dis;
	    if (cur_wire_w < this_index_w-1){
	      min_w_dis = this_index_w  -1 - cur_wire_w;
	    }else if (cur_wire_w > this_index_w+1){
	      min_w_dis = cur_wire_w - this_index_w -1;
	    }else{
	      min_w_dis = 0;
	    }
	    float range_u = rem_dis_cut_u*coef1 - pow(min_v_dis*pitch_v,2) - coef2*pow(min_w_dis*pitch_w,2);
	    float range_v = rem_dis_cut_v*coef1 - pow(min_u_dis*pitch_u,2) - coef2*pow(min_w_dis*pitch_w,2);
	    float range_w = (rem_dis_cut_w*coef1 - pow(min_u_dis*pitch_u,2) - pow(min_v_dis*pitch_v,2))/coef2;
	    
      // std::cout << min_u_dis << " " << min_v_dis << " " << min_w_dis << " "
                // << range_u << " " << range_v << " " << range_w << std::endl;

	    if ( range_u > 0 && range_v >0 && range_w > 0){
	      float low_u_limit = cur_wire_u - sqrt(range_u)/pitch_u;
	      float high_u_limit = cur_wire_u + sqrt(range_u)/pitch_u;
	      float low_v_limit = cur_wire_v - sqrt(range_v)/pitch_v;
	      float high_v_limit = cur_wire_v + sqrt(range_v)/pitch_v;
	      float low_w_limit = cur_wire_w - sqrt(range_w)/pitch_w;
	      float high_w_limit = cur_wire_w + sqrt(range_w)/pitch_w;
	      for (int j = std::round(low_u_limit); j<= std::round(high_u_limit); j++){
		temp_2dut.insert(std::make_pair(j,this_time_slice));
	      }
	      for (int j=std::round(low_v_limit); j<=std::round(high_v_limit); j++){
		temp_2dvt.insert(std::make_pair(j,this_time_slice));
	      }
	      for (int j=std::round(low_w_limit); j<=std::round(high_w_limit); j++){
		temp_2dwt.insert(std::make_pair(j,this_time_slice));
	      }
	    } // cuts
	  } // cuts
	  
	} // loop through points
	
      } // distance
    } // no steiner tree cloud
  }
    // std::cout << "Pixels 2: " << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << std::endl;


  // just projection ...
  if (temp_2dut.size()==0 && temp_2dvt.size()==0 && temp_2dwt.size()==0){
    //std::cout << "haha " << std::endl;
    std::vector<int> temp_results = ct_point_cloud.convert_3Dpoint_time_ch(p);
    int cur_time_slice = temp_results.at(0);
    int cur_index_u = temp_results.at(1);
    int cur_index_v = temp_results.at(2)-2400;
    int cur_index_w = temp_results.at(3)-4800;

    // std::cout << cur_time_slice << " " << cur_index_u << " " << cur_index_v << " " << cur_index_w << std::endl;
    
    for (int i=-time_cut; i!=time_cut+1;i++){
      for (int j=-time_cut; j!=time_cut+1; j++){
	if (abs(i)+abs(j) <= time_cut){
	  temp_2dut.insert(std::make_pair(cur_index_u+i, cur_time_slice+j));
	  temp_2dvt.insert(std::make_pair(cur_index_v+i, cur_time_slice+j));
	  temp_2dwt.insert(std::make_pair(cur_index_w+i, cur_time_slice+j));
	}
      }
    }
  }
  
  
}


void WCPPID::PR3DCluster::form_map(WCP::ToyCTPointCloud& ct_point_cloud, WCP::PointVector& pts,
		  std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge,
		  std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set,
		  std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set,
	       double end_point_factor, double mid_point_factor, int nlevel, double time_cut, double charge_cut){
  
  map_3D_2DU_set.clear();
  map_3D_2DV_set.clear();
  map_3D_2DW_set.clear();

  map_2DU_3D_set.clear();
  map_2DV_3D_set.clear();
  map_2DW_3D_set.clear();

 
  WCP::PointVector saved_pts;
  int count = 0;
  
  // distance ...
  std::vector<double> distances;
  for (size_t i=0;i+1!=pts.size();i++){
    distances.push_back(sqrt(pow(pts.at(i+1).x-pts.at(i).x,2) +
			     pow(pts.at(i+1).y-pts.at(i).y,2) +
			     pow(pts.at(i+1).z-pts.at(i).z,2)));
  }

  

  // start to loop over the path ...
  for (size_t i=0;i!=pts.size();i++){
    double dis_cut;
    if (i==0){
      dis_cut = std::min(distances.at(i) * end_point_factor,4/3.*end_point_factor*units::cm);
    }else if (i+1==pts.size()){
      dis_cut = std::min(distances.back() * end_point_factor,4/3.*end_point_factor*units::cm);
    }else{
      dis_cut = std::min(std::max(distances.at(i-1)*mid_point_factor,distances.at(i)*mid_point_factor),4/3.*mid_point_factor*units::cm);
    }

    // std::cout << i << " " << distances.at(i) << " " << end_point_factor << " " << dis_cut << std::endl;


    std::set<std::pair<int,int> > temp_2dut, temp_2dvt, temp_2dwt;
    form_point_association(pts.at(i), temp_2dut, temp_2dvt, temp_2dwt, ct_point_cloud, dis_cut, nlevel, time_cut);
    // examine ...
    // std::cout << i << " (" << pts.at(i).x << ", " << pts.at(i).y << ", " << pts.at(i).z << ") " << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << std::endl;

    // std::cout << (*temp_2dvt.begin()).first << " " << (*temp_2dvt.end()).second << std::endl;

    std::vector<int> temp_results = ct_point_cloud.convert_3Dpoint_time_ch(pts.at(i));
    temp_results.at(2)-=2400;
    temp_results.at(3)-=4800;
    
    //    std::cout << i << " ";// << pts.at(i) << " " << temp_results.at(0) << " " << temp_results.at(1) << " " << temp_results.at(2) << " " << temp_results.at(3) << std::endl;
    std::vector<float> temp_flag;
    if (i==0 || i==1 || i+1 ==pts.size() || i+2 == pts.size()){
      temp_flag = examine_point_association(temp_results, temp_2dut, temp_2dvt, temp_2dwt, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,true,charge_cut);
    }else{
      temp_flag = examine_point_association(temp_results, temp_2dut, temp_2dvt, temp_2dwt, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,false,charge_cut);
    }
    // std::cout << i << " E (" << pts.at(i).x << ", " << pts.at(i).y << ", " << pts.at(i).z << ") " << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << std::endl;


    //    std::cout << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << " " << temp_flag.at(0) << " " << temp_flag.at(1) << " " << temp_flag.at(2) << std::endl;
    // just projection ...
    // fill the data ...
    //    if (temp_2dut.size() + temp_2dvt.size() + temp_2dwt.size() > 0){
    if (temp_flag.at(0) + temp_flag.at(1) + temp_flag.at(2) > 0){
      map_3D_2DU_set[count] = std::make_pair(temp_2dut,temp_flag.at(0));
      map_3D_2DV_set[count] = std::make_pair(temp_2dvt,temp_flag.at(1));
      map_3D_2DW_set[count] = std::make_pair(temp_2dwt,temp_flag.at(2));
      for (auto it = temp_2dut.begin(); it!=temp_2dut.end();it++){
	if (map_2DU_3D_set.find(*it)==map_2DU_3D_set.end()){
	  std::set<int>  temp_set;
	  temp_set.insert(count);
	  map_2DU_3D_set[*it] = temp_set;
	}else{
	  map_2DU_3D_set[*it].insert(count);
	}
      }
      for (auto it = temp_2dvt.begin(); it!=temp_2dvt.end();it++){
	if (map_2DV_3D_set.find(*it)==map_2DV_3D_set.end()){
	  std::set<int>  temp_set;
	  temp_set.insert(count);
	  map_2DV_3D_set[*it] = temp_set;
	}else{
	  map_2DV_3D_set[*it].insert(count);
	}
      }
      for (auto it = temp_2dwt.begin(); it!=temp_2dwt.end();it++){
	if (map_2DW_3D_set.find(*it)==map_2DW_3D_set.end()){
	  std::set<int>  temp_set;
	  temp_set.insert(count);
	  map_2DW_3D_set[*it] = temp_set;
	}else{
	  map_2DW_3D_set[*it].insert(count);
	}
      }

      
      saved_pts.push_back(pts.at(i));
      count ++;
    }
  }

  // std::cout << "Form Map: " << pts.size() << " " << saved_pts.size() << " " << map_2DU_3D_set.size() + map_2DV_3D_set.size() + map_2DW_3D_set.size() << " " << map_3D_2DU_set.size() << std::endl;

  pts = saved_pts;
}

void WCPPID::PR3DCluster::prepare_data(WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge){
  
  std::vector<int> proj_channel;
  std::vector<int> proj_timeslice;
  std::vector<int> proj_charge;
  std::vector<int> proj_charge_err;
  std::vector<int> proj_flag;
  get_projection(proj_channel,proj_timeslice,proj_charge, proj_charge_err, proj_flag, global_wc_map);

  // enlarge uncertainty for channels next to dead channels ...
  std::map<int, std::pair<int, int> > map_dead_chs = ct_point_cloud.get_all_dead_chs();
  float factor_enlarge_ind = 5;
  float factor_enlarge_col = 2.5;
  
  for (size_t i=0;i!=proj_channel.size(); i++){
    int ch = proj_channel.at(i);
    int time = proj_timeslice.at(i);

    if (proj_charge.at(i)<=0) continue;
    
    bool flag_cur_dead = false;
    {
      auto it1 = map_dead_chs.find(ch);
      if (it1!=map_dead_chs.end())
	if (time >= it1->second.first && time <= it1->second.second)
	  flag_cur_dead = true;
    }
    
    bool flag_p_dead = false;
    if (ch!=2400-1 && ch!=4800-1){
      auto it1 = map_dead_chs.find(ch+1);
      if (it1!=map_dead_chs.end())
	if (time >= it1->second.first && time <= it1->second.second)
	  flag_p_dead = true;
    }

    bool flag_n_dead = false;
    if (ch!=2400 && ch !=4800){
      auto it1 = map_dead_chs.find(ch-1);
      if (it1!=map_dead_chs.end())
	if (time >= it1->second.first && time <= it1->second.second)
	  flag_n_dead = true;
    }

    if ( (!flag_cur_dead) && (flag_p_dead || flag_n_dead) ){
      // std::cout << ch << " " << proj_charge.at(i) << " " << proj_charge_err.at(i) << std::endl;
      if (ch < 4800){
	proj_charge_err.at(i) *= factor_enlarge_ind ;
      }else{
	proj_charge_err.at(i) *= factor_enlarge_col ;
      }
    }
    
  }
  //

  
  int min_time = 1e9;
  int max_time = -1e9;
  int min_uch = 1e9;
  int max_uch = -1e9;
  int min_vch = 1e9;
  int max_vch = -1e9;
  int min_wch = 1e9;
  int max_wch = -1e9;
  
  // std::cout << proj_charge.size() << " " << proj_flag.size() << std::endl;
  for (size_t i=0;i!=proj_channel.size();i++){
    if (min_time > proj_timeslice.at(i)) min_time = proj_timeslice.at(i);
    if (max_time < proj_timeslice.at(i)) max_time = proj_timeslice.at(i);
	
    if (proj_channel.at(i)<2400){
      map_2D_ut_charge[std::make_pair(proj_channel.at(i),proj_timeslice.at(i))] = std::make_tuple(proj_charge.at(i),proj_charge_err.at(i), proj_flag.at(i));

      if (min_uch > proj_channel.at(i)) min_uch = proj_channel.at(i);
      if (max_uch < proj_channel.at(i)) max_uch = proj_channel.at(i);
    }else if (proj_channel.at(i)<4800){
      map_2D_vt_charge[std::make_pair(proj_channel.at(i)-2400,proj_timeslice.at(i))] = std::make_tuple(proj_charge.at(i),proj_charge_err.at(i), proj_flag.at(i));

      if (min_vch > proj_channel.at(i)) min_vch = proj_channel.at(i);
      if (max_vch < proj_channel.at(i)) max_vch = proj_channel.at(i);
    }else{
      map_2D_wt_charge[std::make_pair(proj_channel.at(i)-4800,proj_timeslice.at(i))] = std::make_tuple(proj_charge.at(i),proj_charge_err.at(i), proj_flag.at(i));
      if (min_wch > proj_channel.at(i)) min_wch = proj_channel.at(i);
      if (max_wch < proj_channel.at(i)) max_wch = proj_channel.at(i);
    }
  }

  // flag 0 for dead 
  // flag 1 for good channels or overlapping channels 
  // flag 2 for isolated channels
  // flag 3 for additional channels ...
  
  
  // add the rest of live channels within range???
  std::map<std::pair<int,int>, std::pair<double,double> > map_u_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_uch, max_uch, 0);
  std::map<std::pair<int,int>, std::pair<double,double> > map_v_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_vch, max_vch, 1);
  std::map<std::pair<int,int>, std::pair<double,double> > map_w_tcc = ct_point_cloud.get_overlap_good_ch_charge(min_time, max_time, min_wch, max_wch, 2);

  for (auto it = map_u_tcc.begin(); it!=map_u_tcc.end(); it++){
    if (map_2D_ut_charge.find(std::make_pair(it->first.second, it->first.first))==map_2D_ut_charge.end()){
      map_2D_ut_charge[std::make_pair(it->first.second, it->first.first)] = std::make_tuple(it->second.first, it->second.second, 3);
      //std::cout << it->first.first << " " << it->first.second << std::endl;
    }
  }

  for (auto it = map_v_tcc.begin(); it!=map_v_tcc.end(); it++){
    if (map_2D_vt_charge.find(std::make_pair(it->first.second-2400, it->first.first))==map_2D_vt_charge.end()){
      map_2D_vt_charge[std::make_pair(it->first.second-2400, it->first.first)] = std::make_tuple(it->second.first, it->second.second, 3);
      //std::cout << it->first.first << " " << it->first.second << std::endl;
    }
  }

  for (auto it = map_w_tcc.begin(); it!=map_w_tcc.end(); it++){
    if (map_2D_wt_charge.find(std::make_pair(it->first.second-4800, it->first.first))==map_2D_wt_charge.end()){
      map_2D_wt_charge[std::make_pair(it->first.second-4800, it->first.first)] = std::make_tuple(it->second.first, it->second.second, 3);
      //      std::cout << it->first.first << " " << it->first.second << std::endl;
    }else{
      //      std::cout << it->first.first << " " << it->first.second << std::endl;
    }
  }
  
  
  //  for (auto it = map_w_tcc.begin(); it!=map_w_tcc.end(); it++){
  // std::cout << it->first.first << " " << it->first.second << std::endl;
  // }
  
}

WCP::PointVector WCPPID::PR3DCluster::examine_end_ps_vec(WCP::ToyCTPointCloud& ct_point_cloud, WCP::PointVector& pts, bool flag_start, bool flag_end){
  std::list<Point> ps_list(pts.begin(), pts.end());

  if (flag_start){
    // test start
    Point temp_start = ps_list.front(); 
    while (ps_list.size()>0){
      if (ct_point_cloud.is_good_point(ps_list.front(), 0.2*units::cm,0,0)) break;
      temp_start = ps_list.front();
      ps_list.pop_front();
    }
    
    if (ps_list.size()>0){
      double dis_step = 0.2*units::cm;
      double temp_dis = sqrt(pow(temp_start.x - ps_list.front().x,2) + pow(temp_start.y - ps_list.front().y,2) + pow(temp_start.z - ps_list.front().z,2));
      int ntest = std::round(temp_dis/dis_step);
      for (size_t i=1;i<ntest;i++){
	Point test_p(temp_start.x + (ps_list.front().x - temp_start.x)/ntest * i,
		     temp_start.y + (ps_list.front().y - temp_start.y)/ntest * i,
		     temp_start.z + (ps_list.front().z - temp_start.z)/ntest * i);
	if (ct_point_cloud.is_good_point(test_p, 0.2*units::cm,0,0)){
	  ps_list.push_front(test_p);
	  break;
	}
      }
    }else{
      ps_list.push_front(temp_start);
    }
  }
  
  
  if (flag_end){
    Point temp_end = ps_list.back();
    while (ps_list.size()>0){
      if (ct_point_cloud.is_good_point(ps_list.back(), 0.2*units::cm,0,0)) break;
      temp_end = ps_list.back();
      ps_list.pop_back();
    }
    if (ps_list.size()>0){
      double dis_step = 0.2*units::cm;
      double temp_dis = sqrt(pow(temp_end.x - ps_list.back().x,2) + pow(temp_end.y - ps_list.back().y,2) + pow(temp_end.z - ps_list.back().z,2));
      int ntest = std::round(temp_dis/dis_step);
      for (size_t i=1;i<ntest;i++){
	Point test_p(temp_end.x + (ps_list.back().x - temp_end.x)/ntest * i,
		     temp_end.y + (ps_list.back().y - temp_end.y)/ntest * i,
		     temp_end.z + (ps_list.back().z - temp_end.z)/ntest * i);
	if (ct_point_cloud.is_good_point(test_p, 0.2*units::cm,0,0)){
	  ps_list.push_back(test_p);
	  break;
	}
	
      }
    }else{
      ps_list.push_back(temp_end);
    }
  }

  PointVector tmp_pts(ps_list.begin(), ps_list.end());
  return tmp_pts;
}

void WCPPID::PR3DCluster::organize_ps_path(WCP::ToyCTPointCloud& ct_point_cloud, WCP::PointVector& pts, double low_dis_limit, double end_point_limit){

  
  WCP::PointVector ps_vec = examine_end_ps_vec(ct_point_cloud, pts, true, true);
  if (ps_vec.size()<=1) ps_vec = pts;
 
  pts.clear();
  // fill in the beginning part
  {
    Point p1 = ps_vec.front();
    Point p2 = ps_vec.front();
    double dis1 = 0;
    for (auto it = ps_vec.begin(); it!=ps_vec.end(); it++){
      p2 = *it;
      dis1 = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
      if (dis1 > low_dis_limit) break;
    }
    //    std::cout << ct_point_cloud.is_good_point(p1, 0.2*units::cm,0,0) << " " << p1 << std::endl;
    if (dis1 > low_dis_limit){
      p1.x += (p1.x - p2.x)/dis1 * end_point_limit;
      p1.y += (p1.y - p2.y)/dis1 * end_point_limit;
      p1.z += (p1.z - p2.z)/dis1 * end_point_limit;
      pts.push_back(p1);
      //std::cout << ct_point_cloud.is_good_point(p1, 0.2*units::cm,0,0) << " " << p1 << std::endl;
    }
  }
  
  // fill in the middle part
  for (size_t i=0;i!=ps_vec.size(); i++){
    Point p1 = ps_vec.at(i);
    double dis;
    if (pts.size()!=0){
      dis = sqrt(pow(p1.x-pts.back().x,2)+pow(p1.y-pts.back().y,2)+pow(p1.z-pts.back().z,2));
    }else{
      dis = sqrt(pow(p1.x-ps_vec.back().x,2)+pow(p1.y-ps_vec.back().y,2)+pow(p1.z-ps_vec.back().z,2));
    }

    // std::cout << i << " " << dis << " " << low_dis_limit * 0.8 << " " << low_dis_limit * 1.6 << std::endl;


    // std::cout << i << " " << p1 << " " << pts.back() << " " << dis/units::cm << " " << low_dis_limit/units::cm << std::endl;
    if (dis < low_dis_limit * 0.8 ){
      continue;
    }else if (dis < low_dis_limit * 1.6){
      pts.push_back(p1);
    }else{
      int npoints = std::round(dis/low_dis_limit);
      Point p_save = pts.back();
      for (int j=0;j!=npoints;j++){
	Point p(p_save.x + (p1.x-p_save.x) / npoints * (j+1),
		p_save.y + (p1.y-p_save.y) / npoints * (j+1),
		p_save.z + (p1.z-p_save.z) / npoints * (j+1));
	pts.push_back(p);
      }
    }
  }
  

  // fill in the end part
  if (end_point_limit!=0){
    Point p1 = ps_vec.back();
    Point p2 = ps_vec.back();
    double dis1 = 0;
    for (auto it = ps_vec.rbegin(); it!=ps_vec.rend(); it++){
      p2 = *it;
      dis1 = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
      if (dis1 > low_dis_limit) break;
    }
    if (dis1!=0){
      p1.x += (p1.x - p2.x)/dis1 * end_point_limit;
      p1.y += (p1.y - p2.y)/dis1 * end_point_limit;
      p1.z += (p1.z - p2.z)/dis1 * end_point_limit;
      pts.push_back(p1);
    }
  }else{
    Point p1 = ps_vec.back();
    double dis1 = sqrt(pow(p1.x-pts.back().x,2)+pow(p1.y-pts.back().y,2)+pow(p1.z-pts.back().z,2));
    if (dis1 >= 0.45*units::cm)
      pts.push_back(p1);
  }

  if (pts.size()<=1)
    pts = ps_vec;
}

WCP::PointVector WCPPID::PR3DCluster::organize_wcps_path(WCP::ToyCTPointCloud& ct_point_cloud, std::list<WCPointCloud<double>::WCPoint>& path_wcps_list,  double low_dis_limit, double end_point_limit){

  PointVector pts;
  
  std::vector<WCPointCloud<double>::WCPoint> temp_wcps_vec(path_wcps_list.begin(), path_wcps_list.end());

  // fill in the beginning point ...
  {
    Point p1(temp_wcps_vec.front().x, temp_wcps_vec.front().y, temp_wcps_vec.front().z);
    Point p2(temp_wcps_vec.front().x, temp_wcps_vec.front().y, temp_wcps_vec.front().z);
    double dis1 = 0;
    for (auto it = temp_wcps_vec.begin(); it!=temp_wcps_vec.end(); it++){
      p2.x = (*it).x;
      p2.y = (*it).y;
      p2.z = (*it).z;
      dis1 = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
      if (dis1 > low_dis_limit) break;
    }
    if (dis1!=0){
      p1.x += (p1.x - p2.x)/dis1 * end_point_limit;
      p1.y += (p1.y - p2.y)/dis1 * end_point_limit;
      p1.z += (p1.z - p2.z)/dis1 * end_point_limit;
      pts.push_back(p1);
    }
  }
  // std::cout << "Test b: " <<  pts.size() << " (" << pts.back().x << ", " << pts.back().y << ", " << pts.back().z << ") (" << temp_wcps_vec.front().x << ", " << temp_wcps_vec.front().y << ", " << temp_wcps_vec.front().z << ")" << std::endl;


  // fill in the middle part
  for (size_t i=0;i!=temp_wcps_vec.size(); i++){
    Point p1(temp_wcps_vec.at(i).x, temp_wcps_vec.at(i).y, temp_wcps_vec.at(i).z);

    
    double dis = low_dis_limit;
    if (pts.size()>0)
      dis = sqrt(pow(p1.x-pts.back().x,2)+pow(p1.y-pts.back().y,2)+pow(p1.z-pts.back().z,2));


    
    if (dis < low_dis_limit * 0.8 ){
      continue;
    }else if (dis < low_dis_limit * 1.6){
      pts.push_back(p1);
      //std::cout << p1 << " " << sqrt(pow(p1.x-pts.back().x,2)+pow(p1.y-pts.back().y,2)+pow(p1.z-pts.back().z,2))/units::cm << std::endl;
    }else{
      int npoints = std::round(dis/low_dis_limit);

      Point p_save = pts.back();
      for (int j=0;j!=npoints;j++){
	Point p(p_save.x + (p1.x-p_save.x) / npoints * (j+1),
		p_save.y + (p1.y-p_save.y) / npoints * (j+1),
		p_save.z + (p1.z-p_save.z) / npoints * (j+1));
	pts.push_back(p);
	//	std::cout << p << " " << sqrt(pow(p.x-pts.back().x,2)+pow(p.y-pts.back().y,2)+pow(p.z-pts.back().z,2))/units::cm << std::endl;
      }
    }
  }
    // std::cout << "Test m: " <<  pts.size() << " (" << pts.back().x << ", " << pts.back().y << ", " << pts.back().z << ") (" << temp_wcps_vec.front().x << ", " << temp_wcps_vec.front().y << ", " << temp_wcps_vec.front().z << ")" << std::endl;

  

  // fill in the end part
  {
    Point p1(temp_wcps_vec.back().x, temp_wcps_vec.back().y, temp_wcps_vec.back().z);
    Point p2(temp_wcps_vec.back().x, temp_wcps_vec.back().y, temp_wcps_vec.back().z);
    double dis1 = 0;
    for (auto it = temp_wcps_vec.rbegin(); it!=temp_wcps_vec.rend(); it++){
      p2.x = (*it).x;
      p2.y = (*it).y;
      p2.z = (*it).z;
      dis1 = sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
      if (dis1 > low_dis_limit) break;
    }
    if (dis1!=0){
      p1.x += (p1.x - p2.x)/dis1 * end_point_limit;
      p1.y += (p1.y - p2.y)/dis1 * end_point_limit;
      p1.z += (p1.z - p2.z)/dis1 * end_point_limit;
      pts.push_back(p1);
    }
  }
    // std::cout << "Test e: " <<  pts.size() << " (" << pts.back().x << ", " << pts.back().y << ", " << pts.back().z << ") (" << temp_wcps_vec.front().x << ", " << temp_wcps_vec.front().y << ", " << temp_wcps_vec.front().z << ")" << std::endl;


    
  
  
  return pts;
}
