void WCPPID::PR3DCluster::dQ_dx_multi_fit(WCPPID::Map_Proto_Vertex_Segments& map_vertex_segments, WCPPID::Map_Proto_Segment_Vertices& map_segment_vertices, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,  std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, double flash_time , double dis_end_point_ext , bool flag_dQ_dx_fit_reg ){
  
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
  update_data_dQ_dx_fit(global_wc_map, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge);

  int n_3D_pos = 0;
  int n_2D_u = map_2D_ut_charge.size();
  int n_2D_v = map_2D_vt_charge.size();
  int n_2D_w = map_2D_wt_charge.size();
  
  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoSegment *sg = it->first;
    WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
    if ( (*it->second.begin())->get_wcpt().index == sg->get_wcpt_vec().front().index){
       start_v = (*it->second.begin());
       end_v = (*it->second.rbegin());
     }else{
       end_v = (*it->second.begin());
       start_v = (*it->second.rbegin());
     }
    //    for (auto it1=it->second.begin(); it1!=it->second.end(); it1++){
    // WCPPID::ProtoVertex *vt = *it1;
    // if ( vt->get_wcpt().index == sg->get_wcpt_vec().front().index){
    //	start_v = vt;
    // }else if ( vt->get_wcpt().index == sg->get_wcpt_vec().back().index){
    //	end_v = vt;
    // }
    //}
    std::vector<WCP::Point >& pts = sg->get_point_vec();
    std::vector<int>& indices = sg->get_fit_index_vec();
    //std::cout << pts.size() << " " << indices.size() << std::endl;
    for (size_t i = 0;i!=indices.size(); i++){
      if (i==0){
	if (start_v->get_fit_index()==-1){
	  start_v->set_fit_index(n_3D_pos);
	  indices.at(i) = n_3D_pos;
	  n_3D_pos++;
	}else{
	  indices.at(i) = start_v->get_fit_index();
	}
      }else if (i+1 == pts.size()){
	if (end_v->get_fit_index()==-1){
	  end_v->set_fit_index(n_3D_pos);
	  indices.at(i) = n_3D_pos;
	  n_3D_pos++;
	}else{
	  indices.at(i) = end_v->get_fit_index();
	}
      }else{
	indices.at(i) = n_3D_pos;
	n_3D_pos++;
      }
    }
  }
  //  std::cout << n_3D_pos << std::endl;

  Eigen::VectorXd pos_3D(n_3D_pos), data_u_2D(n_2D_u), data_v_2D(n_2D_v), data_w_2D(n_2D_w), pred_data_u_2D(n_2D_u), pred_data_v_2D(n_2D_v), pred_data_w_2D(n_2D_w);
  Eigen::SparseMatrix<double> RU(n_2D_u, n_3D_pos) ;
  Eigen::SparseMatrix<double> RV(n_2D_v, n_3D_pos) ;
  Eigen::SparseMatrix<double> RW(n_2D_w, n_3D_pos) ;
  PointVector traj_pts(n_3D_pos);
  std::vector<double> local_dx(n_3D_pos,0);
  std::vector<double> traj_reduced_chi2(n_3D_pos,0);
  
  // initial values ...
  Eigen::VectorXd pos_3D_init(n_3D_pos);
  for (int i = 0;i!=n_3D_pos;i++){
    pos_3D_init(i) = 5000 * 6; // single MIP ...
  }

  // regular flag ...
  std::vector<int> reg_flag_u(n_3D_pos,0), reg_flag_v(n_3D_pos,0), reg_flag_w(n_3D_pos,0);

  // start to fill in the data
  {
    int n_u = 0;
    for (auto it = map_2D_ut_charge.begin(); it!= map_2D_ut_charge.end(); it++){
      if (std::get<0>(it->second) > 0)
	data_u_2D(n_u) = std::get<0>(it->second)/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2));
      else
	data_u_2D(n_u) = 0;
      if (std::isnan(data_u_2D(n_u)))
	std::cout << "U: " << data_u_2D(n_u) << " " << std::get<1>(it->second) << " " << std::get<0>(it->second)*rel_uncer_ind << " " << std::endl;
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

  
  // loop over segments ...
  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoSegment *sg = it->first;

    PointVector& pts = sg->get_point_vec();
    std::vector<int>& indices = sg->get_fit_index_vec();
    std::vector<double>& dx_vec = sg->get_dx_vec();

    for (size_t i=0;i!=indices.size();i++){
    //std::cout << i << " " << indices.at(i) << std::endl;
      traj_pts.at(indices.at(i)) = pts.at(i);
    }
    
    //    std::cout<< pts.size() << " " << indices.size() << " " << dx_vec.size() << std::endl;

    for (size_t i1=1; i1+1<pts.size();i1++){
      Point prev_rec_pos, next_rec_pos;
      Point curr_rec_pos = pts.at(i1);
      prev_rec_pos.x = (pts.at(i1-1).x + pts.at(i1).x)/2.;
      prev_rec_pos.y = (pts.at(i1-1).y + pts.at(i1).y)/2.;
      prev_rec_pos.z = (pts.at(i1-1).z + pts.at(i1).z)/2.;
      next_rec_pos.x = (pts.at(i1+1).x + pts.at(i1).x)/2.;
      next_rec_pos.y = (pts.at(i1+1).y + pts.at(i1).y)/2.;
      next_rec_pos.z = (pts.at(i1+1).z + pts.at(i1).z)/2.;

      // std::cout << i1 << " " << indices.at(i1) << std::endl;
      
      dx_vec.at(i1) = sqrt(pow(curr_rec_pos.x-prev_rec_pos.x,2)+pow(curr_rec_pos.y-prev_rec_pos.y,2)+pow(curr_rec_pos.z-prev_rec_pos.z,2))
	+ sqrt(pow(curr_rec_pos.x-next_rec_pos.x,2)+pow(curr_rec_pos.y-next_rec_pos.y,2)+pow(curr_rec_pos.z-next_rec_pos.z,2));

      //dx_vec.at(i1) *= 2;
      //std::cout<<" ---> dx "<<dx_vec.at(i1)/units::cm<<", x/y/z "<<pts.at(i1).x/units::cm<<"\t"<<pts.at(i1).y/units::cm<<std::endl;
      
      if( mp.get_flag_PosEfield_corr() ) {

	double dx_after_corr = mp.func_dx_after_Pos_Efield_SCE_correction( pts.at(i1-1).x/units::cm, pts.at(i1-1).y/units::cm, pts.at(i1-1).z/units::cm, 
									   pts.at(i1+0).x/units::cm, pts.at(i1+0).y/units::cm, pts.at(i1+0).z/units::cm, 
									   pts.at(i1+1).x/units::cm, pts.at(i1+1).y/units::cm, pts.at(i1+1).z/units::cm );	

	//std::cout<<" ---> dx "
	//	 <<dx_vec.at(i1)/units::cm<<"\t"
	//	 <<dx_after_corr<<"\t"
	//	 <<" after/before "<<dx_after_corr/(dx_vec.at(i1)/units::cm)<<std::endl;

	dx_vec.at(i1) = dx_after_corr * units::cm;
      }

      

      std::vector<double> centers_U ;
      std::vector<double> centers_V ;
      std::vector<double> centers_W ;
      std::vector<double> centers_T ;
      std::vector<double> sigmas_T;
      std::vector<double> sigmas_U;
      std::vector<double> sigmas_V;
      std::vector<double> sigmas_W;
      std::vector<double> weights;

      // std::cout << curr_rec_pos.x/units::cm << " " << curr_rec_pos.y/units::cm << " " << curr_rec_pos.z/units::cm << " " << prev_rec_pos.x/units::cm << " " << prev_rec_pos.y/units::cm << " " << prev_rec_pos.z/units::cm << " " << next_rec_pos.x/units::cm << " " << next_rec_pos.y/units::cm << " " << next_rec_pos.z/units::cm << std::endl;

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

    // std::cout << i1 << " U ";
    //  for (size_t idx = 0; idx < centers_U.size(); ++idx) {
    //         std::cout << centers_U[idx] << " ";
    //   }
    //   std::cout << std::endl;

    //   std::cout << i1 << " V ";
    //     for (size_t idx = 0; idx < centers_V.size(); ++idx) {
    //         std::cout << centers_V[idx]-2400 << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << i1 << " W ";
    //     for (size_t idx = 0; idx < centers_W.size(); ++idx) {
    //         std::cout << centers_W[idx]-4800 << " ";
    //     }
    //     std::cout << std::endl;

    //     std::cout << i1 << " T ";
    //     for (size_t idx = 0; idx < centers_T.size(); ++idx) {
    //         std::cout << centers_T[idx]*4 << " ";
    //     }
    //     std::cout << std::endl;

        // std::cout << i1 << " Weights ";
        // for (size_t idx = 0; idx < weights.size(); ++idx) {
        //     std::cout << weights[idx] << " ";
        // }
        // std::cout << std::endl;

        // std::cout <<i1 << " SU ";
        // for (size_t idx = 0; idx < sigmas_U.size(); ++idx) {
        //     std::cout << sigmas_U[idx] << " ";
        // }
        // std::cout << std::endl;

        // std::cout << i1 << " SV ";
        // for (size_t idx = 0; idx < sigmas_V.size(); ++idx) {
        //     std::cout << sigmas_V[idx] << " ";
        // }
        // std::cout << std::endl;

        // std::cout << i1 << " SW ";
        // for (size_t idx = 0; idx < sigmas_W.size(); ++idx) {
        //     std::cout << sigmas_W[idx] << " ";
        // }
        // std::cout << std::endl;

        // std::cout << i1 << " ST ";
        // for (size_t idx = 0; idx < sigmas_T.size(); ++idx) {
        //     std::cout << sigmas_T[idx] << " ";
        // }
        // std::cout << std::endl;

      int n_u = 0;
      double sum_u = 0;
      for (auto it = map_2D_ut_charge.begin(); it!= map_2D_ut_charge.end(); it++){
      	if (fabs(it->first.first - centers_U.front()) <= 10 && fabs(it->first.second - centers_T.front()) <= 10 ){
      	  double value = cal_gaus_integral_seg(it->first.second, it->first.first,centers_T, sigmas_T, centers_U, sigmas_U, weights , 0 , 4);
      	  sum_u += value;
      	  // near dead channels ...
      	  if (std::get<2>(it->second)==0 && value > 0) reg_flag_u.at(indices.at(i1)) = 1;
      	  if (value > 0 && std::get<0>(it->second) >0 && std::get<2>(it->second)!=0){
      	    // if (i!=143)
      	    RU.insert(n_u,indices.at(i1)) = value/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2));
      	    // if (i==143) std::cout << "U: " << it->first.first << " " << it->first.second << " " << value << std::endl;

            // std::cout << n_u << " " << i1 << " " << it->first.second*4 << " " << it->first.first << " " << i1 << " " << value / sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2)) << std::endl;
      	  }
      	}
      	n_u ++;
      }
      
      int n_v = 0;
      double sum_v = 0;
      for (auto it = map_2D_vt_charge.begin(); it!= map_2D_vt_charge.end(); it++){
      	if (fabs(it->first.first +  2400 - centers_V.front()) <= 10 && fabs(it->first.second - centers_T.front()) <= 10 ){
      	  double value = cal_gaus_integral_seg(it->first.second, it->first.first + 2400, centers_T, sigmas_T, centers_V, sigmas_V, weights , 0 , 4);
      	  sum_v += value;
      	  // near dead channels
      	  if (std::get<2>(it->second)==0 && value > 0) {
      	    // if (i==136) std::cout << it->first.first << " " << it->first.second << std::endl;
      	    reg_flag_v.at(indices.at(i1)) = 1;
      	  }
	  
      	  if (value > 0 && std::get<0>(it->second) >0 && std::get<2>(it->second)!=0){
      	    //	  if (i==136) std::cout << "V: " << it->first.first << " " << it->first.second << " " << value << std::endl;
      	    // if (i!=143)
      	    RV.insert(n_v,indices.at(i1)) = value/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2));
      	  }
      	}
      	n_v ++;
      }

      int n_w = 0;
      double sum_w = 0;
      for (auto it = map_2D_wt_charge.begin(); it!= map_2D_wt_charge.end(); it++){
      	if (fabs(it->first.first + 4800 - centers_W.front()) <= 10 && fabs(it->first.second - centers_T.front()) <= 10 ){
      	  double value = cal_gaus_integral_seg(it->first.second, it->first.first + 4800,centers_T, sigmas_T, centers_W, sigmas_W, weights , 0 , 4);
      	  sum_w += value;
      	  // near dead channels ...
      	  if (std::get<2>(it->second)==0 && value > 0) reg_flag_w.at(indices.at(i1)) = 1;
	  
      	  // relevant && charge > 0 && not dead channel ...
      	  if (value > 0 && std::get<0>(it->second) >0 && std::get<2>(it->second)!=0){
      	    // if (i!=143)
      	    RW.insert(n_w,indices.at(i1)) = value/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_col,2) + pow(add_uncer_col,2));
      	    //  if (i==147) std::cout << "W: " << it->first.first << " " << it->first.second << " " << value << std::endl;
      	  }
      	}
      	n_w ++;
      }

      // additional checks on the dead channels ...
      if (reg_flag_u.at(indices.at(i1))==0){
      	for (size_t kk = 0; kk!=centers_U.size(); kk++){
      	  if (map_2D_ut_charge.find(std::make_pair(int(centers_U.at(kk)), int(centers_T.at(kk)) ))==map_2D_ut_charge.end()) {
      	    reg_flag_u.at(indices.at(i1)) =1;
      	    break;
      	  }
      	}
      }
      if (reg_flag_v.at(indices.at(i1))==0){
      	for (size_t kk = 0; kk!=centers_V.size(); kk++){
      	  if (map_2D_vt_charge.find(std::make_pair(int(centers_V.at(kk)-2400), int(centers_T.at(kk)) ))==map_2D_vt_charge.end()) {
      	    //if (i==136)  std::cout << centers_V.at(kk) << " " << centers_T.at(kk) << std::endl;
      	    reg_flag_v.at(indices.at(i1)) =1;
      	    break;
      	  }
      	}
      }
      if (reg_flag_w.at(indices.at(i1))==0){
      	for (size_t kk = 0; kk!=centers_W.size(); kk++){
      	  if (map_2D_wt_charge.find(std::make_pair(int(centers_W.at(kk)-4800), int(centers_T.at(kk)) ))==map_2D_wt_charge.end()) {
      	    reg_flag_w.at(indices.at(i1)) =1;
      	    break;
      	  }
      	}
      }

      // std::cout << indices.at(i1) << " " << reg_flag_u.at(indices.at(i1)) << " " << reg_flag_v.at(indices.at(i1)) << " " << reg_flag_w.at(indices.at(i1)) << std::endl;


    } // loop over points ...
  } // loop over segments ...
  
  
  // loop over vertices ...
  for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    if (it->second.size() == 0) continue;
    WCPPID::ProtoVertex *vtx = it->first;
    Point curr_rec_pos = vtx->get_fit_pt();
    int index = vtx->get_fit_index();
    PointVector connected_pts;
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      WCPPID::ProtoSegment *sg = *it1;
      if ( vtx->get_wcpt().index == sg->get_wcpt_vec().front().index){
        Point p;
        p.x = (curr_rec_pos.x + sg->get_point_vec().at(1).x)/2.;
        p.y = (curr_rec_pos.y + sg->get_point_vec().at(1).y)/2.;
        p.z = (curr_rec_pos.z + sg->get_point_vec().at(1).z)/2.;
        connected_pts.push_back(p);
      }else if (vtx->get_wcpt().index == sg->get_wcpt_vec().back().index){
        Point p;
        p.x = (curr_rec_pos.x + sg->get_point_vec().at(sg->get_point_vec().size()-2).x)/2.;
        p.y = (curr_rec_pos.y + sg->get_point_vec().at(sg->get_point_vec().size()-2).y)/2.;
        p.z = (curr_rec_pos.z + sg->get_point_vec().at(sg->get_point_vec().size()-2).z)/2.;
        connected_pts.push_back(p);
      }
    }
    if (connected_pts.size()==1){
      double length = sqrt(pow(connected_pts.at(0).x - curr_rec_pos.x,2) + pow(connected_pts.at(0).y - curr_rec_pos.y,2) + pow(connected_pts.at(0).z - curr_rec_pos.z,2));
      Point p = curr_rec_pos;
      if (length>0){
        p.x = curr_rec_pos.x - (connected_pts.at(0).x - curr_rec_pos.x)/length * dis_end_point_ext;
        p.y = curr_rec_pos.y - (connected_pts.at(0).y - curr_rec_pos.y)/length * dis_end_point_ext;
        p.z = curr_rec_pos.z - (connected_pts.at(0).z - curr_rec_pos.z)/length * dis_end_point_ext;
      }
      connected_pts.push_back(p);
      // std::cout << length << std::endl;
      // std::cout << connected_pts.size() << std::endl;
    }
    //    std::cout << connected_pts.size() << std::endl;

    double tmp_dx = 0;
    for (size_t i=0;i!=connected_pts.size();i++){
      tmp_dx += sqrt(pow(connected_pts.at(i).x - curr_rec_pos.x,2) + pow(connected_pts.at(i).y - curr_rec_pos.y,2) + pow(connected_pts.at(i).z - curr_rec_pos.z,2));
    }
    
    // ttt
    if( mp.get_flag_PosEfield_corr() ) {
      double user_dx = 0;

      for (size_t i=0;i!=connected_pts.size();i++){
        double dx_after_corr = mp.func_dx_after_Pos_Efield_SCE_correction( connected_pts.at(i).x/units::cm, connected_pts.at(i).y/units::cm, connected_pts.at(i).z/units::cm, 
                          curr_rec_pos.x/units::cm, curr_rec_pos.y/units::cm, curr_rec_pos.z/units::cm);
        user_dx += dx_after_corr*units::cm;
      }
      
      tmp_dx = user_dx;

      //std::cout<<" ---> check "<<user_dx/tmp_dx<<std::endl;
    }


    vtx->set_dx(tmp_dx);
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      WCPPID::ProtoSegment *sg = *it1;
      std::vector<double>& dx_vec = sg->get_dx_vec();
      if ( vtx->get_wcpt().index == sg->get_wcpt_vec().front().index){
	dx_vec.front() = tmp_dx;
      }else if (vtx->get_wcpt().index == sg->get_wcpt_vec().back().index){
	dx_vec.back() = tmp_dx;
      }
    }

    /* for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){ */
    /*   WCPPID::ProtoSegment *sg = *it1; */
    /*   std::cout << vtx->get_dx() << " " << sg->get_dx_vec().front() << " " << sg->get_dx_vec().back() << " " << std::endl; */
    /* } */

    std::vector<double> centers_U ;
    std::vector<double> centers_V ;
    std::vector<double> centers_W ;
    std::vector<double> centers_T ;
    std::vector<double> sigmas_T;
    std::vector<double> sigmas_U;
    std::vector<double> sigmas_V;
    std::vector<double> sigmas_W;
    std::vector<double> weights;

    //    std::cout << vtx->get_id() << " " << connected_pts.size() << " " << map_vertex_segments[vtx].size() << std::endl;
    
    for (int k=0;k!=connected_pts.size();k++){

      // std::cout << k << " " << curr_rec_pos.x/units::cm << " " << curr_rec_pos.y/units::cm << " " << curr_rec_pos.z/units::cm << " " << connected_pts.at(k).x/units::cm << " " << connected_pts.at(k).y/units::cm << " " << connected_pts.at(k).z/units::cm << std::endl;

      for (int j=0;j!=5;j++){
	Point reco_pos;
	reco_pos.x = connected_pts.at(k).x + (curr_rec_pos.x - connected_pts.at(k).x)/5.*(j+0.5);
	reco_pos.y = connected_pts.at(k).y + (curr_rec_pos.y - connected_pts.at(k).y)/5.*(j+0.5);
	reco_pos.z = connected_pts.at(k).z + (curr_rec_pos.z - connected_pts.at(k).z)/5.*(j+0.5);
	double central_T = offset_t + slope_xt * reco_pos.x ;
	double central_U = offset_u + (slope_yu * reco_pos.y + slope_zu * reco_pos.z);
	double central_V = offset_v + (slope_yv * reco_pos.y + slope_zv * reco_pos.z)+2400;
	double central_W = offset_w + (slope_yw * reco_pos.y + slope_zw * reco_pos.z)+4800;
	double weight = sqrt(pow(connected_pts.at(k).x-curr_rec_pos.x,2)+
			     pow(connected_pts.at(k).y-curr_rec_pos.y,2)+
			     pow(connected_pts.at(k).z-curr_rec_pos.z,2));
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
      }
    }

    //    std::cout << " U ";
    //  for (size_t idx = 0; idx < centers_U.size(); ++idx) {
    //         std::cout << centers_U[idx] << " ";
    //   }
    //   std::cout << std::endl;
    
    int n_u = 0;
    double sum_u = 0;
    for (auto it = map_2D_ut_charge.begin(); it!= map_2D_ut_charge.end(); it++){
      if (fabs(it->first.first - centers_U.front()) <= 10 && fabs(it->first.second - centers_T.front()) <= 10 ){
	double value = cal_gaus_integral_seg(it->first.second, it->first.first,centers_T, sigmas_T, centers_U, sigmas_U, weights , 0 , 4);
	sum_u += value;
	// near dead channels ...
	if (std::get<2>(it->second)==0 && value > 0) reg_flag_u.at(index) = 1;
	
	if (value > 0 && std::get<0>(it->second) >0 && std::get<2>(it->second)!=0){
	  // if (i!=143)
	  RU.insert(n_u,index) = value/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2));
	  // if (i==143) std::cout << "U: " << it->first.first << " " << it->first.second << " " << value << std::endl;
    // std::cout << "U: " << n_u << " " << index << " " << it->first.second*4 << " " << it->first.first << " " << index << " " << value / sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2)) << std::endl;
  }
      }
      n_u ++;
    }

    int n_v = 0;
    double sum_v = 0;
    for (auto it = map_2D_vt_charge.begin(); it!= map_2D_vt_charge.end(); it++){
      if (fabs(it->first.first +  2400 - centers_V.front()) <= 10 && fabs(it->first.second - centers_T.front()) <= 10 ){
	double value = cal_gaus_integral_seg(it->first.second, it->first.first + 2400, centers_T, sigmas_T, centers_V, sigmas_V, weights , 0 , 4);
	sum_v += value;
	// near dead channels
	if (std::get<2>(it->second)==0 && value > 0) {
	  // if (i==136) std::cout << it->first.first << " " << it->first.second << std::endl;
	  reg_flag_v.at(index) = 1;
	}

	
	if (value > 0 && std::get<0>(it->second) >0 && std::get<2>(it->second)!=0){
	  //	  if (i==136) std::cout << "V: " << it->first.first << " " << it->first.second << " " << value << std::endl;
	  // if (i!=143)
	  RV.insert(n_v,index) = value/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_ind,2) + pow(add_uncer_ind,2));
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
	if (std::get<2>(it->second)==0 && value > 0) reg_flag_w.at(index) = 1;
	
	// relevant && charge > 0 && not dead channel ...
	if (value > 0 && std::get<0>(it->second) >0 && std::get<2>(it->second)!=0){
	  // if (i!=143)
	  RW.insert(n_w,index) = value/sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_col,2) + pow(add_uncer_col,2));
	  //  if (i==147) std::cout << "W: " << it->first.first << " " << it->first.second << " " << value << std::endl;
	}
      }
      n_w ++;
    }

    // additional checks on the dead channels ...
    if (reg_flag_u.at(index)==0){
      for (size_t kk = 0; kk!=centers_U.size(); kk++){
    	if (map_2D_ut_charge.find(std::make_pair(int(centers_U.at(kk)), int(centers_T.at(kk)) ))==map_2D_ut_charge.end()) {
    	  reg_flag_u.at(index) =1;
    	  break;
    	}
      }
    }
    if (reg_flag_v.at(index)==0){
      for (size_t kk = 0; kk!=centers_V.size(); kk++){
    	if (map_2D_vt_charge.find(std::make_pair(int(centers_V.at(kk)-2400), int(centers_T.at(kk)) ))==map_2D_vt_charge.end()) {
	  //if (i==136)  std::cout << centers_V.at(kk) << " " << centers_T.at(kk) << std::endl;
    	  reg_flag_v.at(index) =1;
    	  break;
    	}
      }
    }
    if (reg_flag_w.at(index)==0){
      for (size_t kk = 0; kk!=centers_W.size(); kk++){
    	if (map_2D_wt_charge.find(std::make_pair(int(centers_W.at(kk)-4800), int(centers_T.at(kk)) ))==map_2D_wt_charge.end()) {
    	  reg_flag_w.at(index) =1;
    	  break;
    	}
      }
    }
    
    // std::cout << index << " " << reg_flag_u.at(index) << " " << reg_flag_v.at(index) << " " << reg_flag_w.at(index) << std::endl;
  }

  Eigen::SparseMatrix<double> RUT = Eigen::SparseMatrix<double>(RU.transpose());
  Eigen::SparseMatrix<double> RVT = Eigen::SparseMatrix<double>(RV.transpose());
  Eigen::SparseMatrix<double> RWT = Eigen::SparseMatrix<double>(RW.transpose());
  
  // check the compact view, relax the uncertainties ... regularization ...
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
  std::vector<std::vector<int> > connected_vec(n_3D_pos);
  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoSegment *sg = it->first;
    std::vector<int>& indices = sg->get_fit_index_vec();
    for (size_t i=1;i+1<indices.size();i++){
      connected_vec.at(indices.at(i)).push_back(indices.at(i-1));
      connected_vec.at(indices.at(i)).push_back(indices.at(i+1));
    }
  }
  for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoVertex *vtx = it->first;
    int index = vtx->get_fit_index();
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      WCPPID::ProtoSegment *sg = *it1;
      std::vector<int>& indices = sg->get_fit_index_vec();
      if (index == indices.front()){
	connected_vec.at(index).push_back(indices.at(1));
      }else if (index == indices.back()){
	connected_vec.at(index).push_back(indices.size()-2);
      }else{
	std::cout << "Something Wrong! " << std::endl;
      }
    }
  }

 
  
  
  
  std::vector<std::vector<double> > overlap_u = cal_compact_matrix_multi(connected_vec, MU, RUT, n_2D_u, n_3D_pos,3); 
  std::vector<std::vector<double> > overlap_v = cal_compact_matrix_multi(connected_vec, MV, RVT, n_2D_v, n_3D_pos,3); // three wire sharing ... 
  std::vector<std::vector<double> > overlap_w = cal_compact_matrix_multi(connected_vec, MW, RWT, n_2D_w, n_3D_pos,2); // two wire sharing  ...


  // for(size_t i=0;i!=connected_vec.size();i++){ 
  //   std::cout << i << " " << connected_vec.at(i).size() << " " << overlap_u.at(i).size() << " " << overlap_v.at(i).size() << " " << overlap_w.at(i).size() << std::endl;
  // }

  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoSegment *sg = it->first;
    std::vector<int>& indices = sg->get_fit_index_vec();
    std::vector<double>& dx_vec = sg->get_dx_vec();

    for (size_t i=0;i!=indices.size();i++){
      local_dx.at(indices.at(i)) = dx_vec.at(i);
      //      std::cout << (local_dx.at(indices.at(i))/(0.6*units::cm)) << std::endl;
    }
  }
  
  
  // add regularization ...
  Eigen::SparseMatrix<double> FMatrix(n_3D_pos, n_3D_pos);
  
  double dead_ind_weight = 0.3;
  double dead_col_weight = 0.9;
  
  double close_ind_weight = 0.25;
  double close_col_weight = 0.75;
  
  for (size_t i=0; i!=n_3D_pos; i++){
    bool flag_u = reg_flag_u.at(i);
    bool flag_v = reg_flag_v.at(i);
    bool flag_w = reg_flag_w.at(i);

    double weight = 0;
    if (flag_u) weight += dead_ind_weight;
    if (flag_v) weight += dead_ind_weight;
    if (flag_w) weight += dead_col_weight;

    double scaling = 1.;
    if (connected_vec.at(i).size()>2) scaling = 2./connected_vec.at(i).size();
    
    for (size_t j=0; j!= connected_vec.at(i).size();j++){
      double weight1 = weight;
      int row = i;
      int col = connected_vec.at(i).at(j);

      if (overlap_u.at(i).at(j) > 0.5) weight1 += close_ind_weight * pow(overlap_u.at(i).at(j)-0.5,2);
      if (overlap_v.at(i).at(j) > 0.5) weight1 += close_ind_weight * pow(overlap_v.at(i).at(j)-0.5,2);
      if (overlap_w.at(i).at(j) > 0.5) weight1 += close_col_weight * pow(overlap_w.at(i).at(j)-0.5,2);

      FMatrix.coeffRef(row,row) += -weight1 * scaling/( (local_dx.at(row)+0.001*units::cm)/(0.6*units::cm) );
      FMatrix.coeffRef(row,col) += weight1 * scaling/( (local_dx.at(col)+0.001*units::cm)/(0.6*units::cm) );
    }    
  }
  
  // adjust regularization strength
  double lambda = 0.0008;
  FMatrix *= lambda;
  
  if (!flag_dQ_dx_fit_reg)
    FMatrix *=0.01;
  
  Eigen::SparseMatrix<double> FMatrixT = Eigen::SparseMatrix<double>(FMatrix.transpose());
  
  //  std::cout << overlap_u.size() << " " << overlap_v.size() << " " << overlap_w.size() << std::endl;
  
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
  Eigen::VectorXd b = RUT * MU * data_u_2D + RVT * MV * data_v_2D + RWT * MW * data_w_2D;
  Eigen::SparseMatrix<double> A =  RUT * MU * RU + RVT * MV * RV + RWT * MW * RW + FMatrixT * FMatrix;//
  solver.compute(A);
  
  pos_3D = solver.solveWithGuess(b,pos_3D_init);

  if (std::isnan(solver.error())){
    pos_3D = solver.solve(b);
  }

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
    // if (it->first.first+4800 > 8256) std::cout << "problem " << it->first.first+4800 << std::endl;
    
    proj_data_w_map[std::make_pair(it->first.first+4800, it->first.second)] = std::make_tuple(std::get<0>(it->second), std::get<1>(it->second), pred_data_w_2D(n_w) * sqrt(pow(std::get<1>(it->second),2)+pow(std::get<0>(it->second)*rel_uncer_col,2)+pow(add_uncer_col,2)));
    n_w++;
  }

  // label the data comparison ...
  traj_reduced_chi2.clear();
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
    traj_reduced_chi2.push_back(sqrt((sum[0] + sum[1] + sum[2]/4.)/(sum1[0]+sum1[1]+sum1[2])));
    //    std::cout << "Xin: " << cluster_id << " " << k << " " << sum[0] << " " << sum[1] << " " << sum[2] << " " << sum1[0] << " " << sum1[1] << " " << sum1[2] << " " <<  (fine_tracking_path.at(k).x - time_slice_width /( nrebin * 0.5*units::microsecond) * flash_time)/units::cm << " " <<  fine_tracking_path.at(k).y/units::cm << " " <<  fine_tracking_path.at(k).z/units::cm << std::endl;
  }

  //  std::cout << traj_pts.size() << " " << traj_reduced_chi2.size() << std::endl;
  
  
  // fill the final results ...
  double sum = 0;
  for (int i=0;i!=n_3D_pos;i++){
    //    std::cout << i << " " << pos_3D(i) << " " << traj_pts.at(i) << std::endl;
    double central_U = offset_u + (slope_yu * traj_pts.at(i).y + slope_zu * traj_pts.at(i).z);
    //    std::cout << offset_u << " " << slope_yu << " " << slope_zu << std::endl;
    //    std::cout << offset_w << " " << slope_yw << " " << slope_zw << std::endl;

    // does not seems to be a uniform correction ... 
    if (central_U >=296 && central_U <=327 ||
	central_U >=336 && central_U <=337 ||
	central_U >=343 && central_U <=351 ||
	central_U >=376 && central_U <=400 ||
	central_U >=410 && central_U <=484 ||
	central_U >=501 && central_U <=524 ||
	central_U >=536 && central_U <=671)
      pos_3D(i) = pos_3D(i)/0.7;
    
    sum += pos_3D(i);
  }
  std::cout << "Cluster: " << cluster_id << " multi-fit total: " << sum << std::endl;

  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoSegment *sg = it->first;

    // std::cout << sg->get_id() << " " << sg->get_fit_index_vec().size() << " " << sg->get_dQ_vec().size() << std::endl;
    
    //std::cout << mp.get_flag_corr() << std::endl;
    
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      WCPPID::ProtoVertex *vtx = *it1;
      Point p = vtx->get_fit_pt();

      double corr = 1.;
      if (mp.get_flag_corr()){
	corr = mp.get_corr_factor(p, offset_u,  slope_yu,  slope_zu,  offset_v,  slope_yv,  slope_zv,  offset_w,  slope_yw,  slope_zw);
      }
      
      // correction electron lifetime attenuation
      double elifetime_ratio =  mp.get_attenuation_ratio((p.x/time_slice_width * nrebin * 0.5*units::microsecond  - flash_time)/units::millisecond);
      //    std::cout << p.at(i).x << " " << (p.x/time_slice_width * nrebin * 0.5*units::microsecond  - flash_time)/units::millisecond << " " << elifetime_ratio << std::endl;
      corr /= elifetime_ratio ; 
      
      int index = vtx->get_fit_index();
      double tmp_dQ = pos_3D(index) * corr;
      double tmp_dx = vtx->get_dx();
      double tmp_reduced_chi2 = traj_reduced_chi2.at(index);
      double tmp_pu = vtx->get_pu();
      double tmp_pv = vtx->get_pv();
      double tmp_pw = vtx->get_pw();
      double tmp_pt = vtx->get_pt();      
      
      vtx->set_fit(p, tmp_dQ, tmp_dx, tmp_pu, tmp_pv, tmp_pw, tmp_pt, tmp_reduced_chi2);
    }

    PointVector& pts = sg->get_point_vec();
    std::vector<int>& indices = sg->get_fit_index_vec();
    std::vector<double>& dQ_vec = sg->get_dQ_vec();
    std::vector<double>& dx_vec = sg->get_dx_vec();//   
    std::vector<double>& reduced_chi2_vec = sg->get_reduced_chi2_vec();
    for (size_t i=0;i!=dQ_vec.size();i++){
      double corr = 1;
      if (mp.get_flag_corr()){
	corr = mp.get_corr_factor(pts.at(i), offset_u,  slope_yu,  slope_zu,  offset_v,  slope_yv,  slope_zv,  offset_w,  slope_yw,  slope_zw);
      }
      // correction electron lifetime attenuation
      double elifetime_ratio =  mp.get_attenuation_ratio((pts.at(i).x/time_slice_width * nrebin * 0.5*units::microsecond  - flash_time)/units::millisecond);
      //    std::cout << pts.at(i).x << " " << (pts.at(i).x/time_slice_width * nrebin * 0.5*units::microsecond  - flash_time)/units::millisecond << " " << elifetime_ratio << std::endl;
      corr /= elifetime_ratio ; 

      
      dQ_vec.at(i) = pos_3D(indices.at(i)) * corr;
      reduced_chi2_vec.at(i) = traj_reduced_chi2.at(indices.at(i));

      ///
      if( mp.get_flag_PosEfield_corr() ) {

	double dQdx_after_corr = mp.func_dQdx_after_Pos_Efield_SCE_correction( pts.at(i).x/units::cm, pts.at(i).y/units::cm, pts.at(i).z/units::cm, 
									       dQ_vec.at(i), dx_vec.at(i)/units::cm );

	double dQ_after_corr = dQdx_after_corr * dx_vec.at(i)/units::cm;

	dQ_vec.at(i) = dQ_after_corr;
	
	//std::cout<<" dQ_dx check ---> before and after "<<i<<"\t"<<dQ_vec.at(i)<<"\t"<<dQ_after_corr<<"\t"<<dQ_after_corr/dQ_vec.at(i)<<std::endl;
      }
           
      //std::cout<<" -dQ_dx --> check seg "<<i<<" "<<dx_vec.at(i)<<", pos "<<pts.at(i).x/units::cm<<std::endl;
    }
    
    
  }
  

  
  
  
}


std::vector<std::vector<double> > WCPPID::PR3DCluster::cal_compact_matrix_multi(std::vector<std::vector<int> >& connected_vec, Eigen::SparseMatrix<double>& MW, Eigen::SparseMatrix<double>& RWT, int n_2D_w, int n_3D_pos, double cut_pos){
  std::vector<std::vector<double> > results(connected_vec.size());

  // initial data ...
  std::vector<int> count_2D(n_2D_w,1);
  std::map<int, std::set<int> > map_2D_3D;
  std::map<int, std::set<int> > map_3D_2D;
  std::map<std::pair<int, int>, double> map_pair_val;    
  for (int k=0;k<RWT.outerSize(); ++k){
    int count = 0;
    
    for (Eigen::SparseMatrix<double>::InnerIterator it(RWT,k); it; ++it){
      
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

  // figure out the sharing among nearby points ...
  for (auto it = map_3D_2D.begin(); it!=map_3D_2D.end(); it++){
    int row = it->first;
    //std::cout << row << " " << connected_vec.at(row).size() << std::endl;
    for (size_t i=0;i!=connected_vec.at(row).size();i++){
      double sum[2]={0, 0};
      auto it1 = map_3D_2D.find(connected_vec.at(row).at(i) );
      for (auto it3 = it->second.begin(); it3!=it->second.end(); it3++){
	int col = *it3;
	double val = map_pair_val[std::make_pair(row, col)];
	sum[0] += 1;//val;
      }
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
      }
      results.at(row).push_back(sum[1]/(sum[0]+1e-9));
    }
  }

  for (size_t i=0;i!=results.size(); i++){
    if (results.at(i).size() != connected_vec.at(i).size()){
      results.at(i).resize(connected_vec.at(i).size(),0);
    }
  }
  
  
  
  return results;
}
