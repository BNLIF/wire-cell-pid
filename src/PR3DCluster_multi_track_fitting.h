
// WCT doc sbnd_xin/docs/pr/108 stage dump (debug only): WCP_TRAJ_DUMP=<path>.
// Same record layout as the toolkit's WCT_TRAJ_DUMP so the two can be diffed.
static FILE* g_wcp_traj_dump = (getenv("WCP_TRAJ_DUMP") ? fopen(getenv("WCP_TRAJ_DUMP"), "a") : (FILE*)0);
static int g_wcp_traj_call = 0;
static int g_wcp_traj_stage = 0;
static void wcp_traj_dump_fits(const char* tag, WCPPID::Map_Proto_Segment_Vertices& map_segment_vertices){
  if (!g_wcp_traj_dump) return;
  int si = 0;
  for (auto it = map_segment_vertices.begin(); it != map_segment_vertices.end(); it++, si++){
    WCPPID::ProtoSegment* sg = it->first;
    std::vector<WCP::Point>& pts = sg->get_point_vec();
    std::vector<double>& dQ = sg->get_dQ_vec();
    std::vector<double>& dx = sg->get_dx_vec();
    for (size_t i = 0; i != pts.size(); i++){
      fprintf(g_wcp_traj_dump, "%d %s %d %zu %.4f %.4f %.4f %.1f %.4f\n", g_wcp_traj_call, tag, si, i,
              pts.at(i).x/units::cm, pts.at(i).y/units::cm, pts.at(i).z/units::cm,
              i < dQ.size() ? dQ.at(i) : 0.0, i < dx.size() ? dx.at(i)/units::cm : 0.0);
    }
  }
  fflush(g_wcp_traj_dump);
}

void WCPPID::PR3DCluster::do_multi_tracking(WCPPID::Map_Proto_Vertex_Segments& map_vertex_segments, WCPPID::Map_Proto_Segment_Vertices& map_segment_vertices, WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, double time, bool flag_dQ_dx_fit_reg, bool flag_dQ_dx_fit, bool flag_exclusion){

  // WCT doc sbnd_xin/docs/pr/108 Test B (parity study, 2026-08-21): env
  // override WCP_FIT_EXCLUSION=0 forces the exclusion fit OFF at every
  // do_multi_tracking call site (the two already-false break_segments sites
  // are unaffected).  Unset => behaviour unchanged.
  {
    static const char* wcp_excl_env = getenv("WCP_FIT_EXCLUSION");
    if (wcp_excl_env && std::string(wcp_excl_env) == "0") flag_exclusion = false;
  }
  if (g_wcp_traj_dump){
    g_wcp_traj_call++; g_wcp_traj_stage = 0;
    fprintf(g_wcp_traj_dump, "%d call excl=%d cluster=%d\n", g_wcp_traj_call, (int)flag_exclusion, get_cluster_id());
  }

  bool flag_special = false;

  for (auto it = map_vertex_segments.begin(); it != map_vertex_segments.end(); it++){
    WCPPID::ProtoVertex *vtx = it->first;
    if (vtx->get_cluster_id() !=cluster_id) continue;
    if (!vtx->get_flag_fit_fix()){
      Point p(vtx->get_wcpt().x, vtx->get_wcpt().y, vtx->get_wcpt().z);
      vtx->set_fit_pt(p);
    }
  }
  

  // collect charge
  collect_charge_multi_trajectory(map_segment_vertices, ct_point_cloud);

  // clear things
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
  prepare_data(ct_point_cloud, global_wc_map, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge);

  // for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
  //   WCPPID::ProtoSegment *seg = it->first;
  //   std::cout << seg->get_point_vec().size() << " " << std::endl;
  //   for (auto p : seg->get_point_vec()){
  //     std::cout << p << " " << std::endl;
  //   }
  // }
  // std::cout << std::endl;

  // first round of organizing the path from the path_wcps (shortest path)
  double low_dis_limit = 1.2*units::cm;
  double end_point_limit = 0.6*units::cm;
  organize_segments_path(ct_point_cloud, map_vertex_segments, map_segment_vertices, low_dis_limit, end_point_limit);

  // std::cout << "After first organization " << std::endl;
  // for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
  //   WCPPID::ProtoSegment *seg = it->first;
  //   std::cout << seg->get_point_vec().size() << " " << std::endl;
  //   for (auto p : seg->get_point_vec()){
  //     std::cout << p << " " << std::endl;
  //   }
  // }
  // std::cout << std::endl;
  
  // form association ...
  // map 3D index to set of 2D points
  std::map<int,std::pair<std::set<std::pair<int,int>>, float> > map_3D_2DU_set;
  std::map<int,std::pair<std::set<std::pair<int,int>>, float> > map_3D_2DV_set;
  std::map<int,std::pair<std::set<std::pair<int,int>>, float> > map_3D_2DW_set;
  std::map<int,std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, int> > map_3D_tuple;
  // map 2D points to 3D indices
  std::map<std::pair<int,int>,std::set<int>> map_2DU_3D_set;
  std::map<std::pair<int,int>,std::set<int>> map_2DV_3D_set;
  std::map<std::pair<int,int>,std::set<int>> map_2DW_3D_set;

 
  if (flag_1st_tracking){
    g_wcp_traj_stage = 1;
    form_map_multi_segments(map_vertex_segments, map_segment_vertices, ct_point_cloud,
			    map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,
			    map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set, map_3D_tuple,
			    map_2DU_3D_set, map_2DV_3D_set, map_2DW_3D_set, flag_exclusion);

   
    multi_trajectory_fit(map_vertex_segments, map_segment_vertices,
			 map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set, map_3D_tuple,
			 map_2DU_3D_set, map_2DV_3D_set, map_2DW_3D_set,
			 map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge);
    wcp_traj_dump_fits("fit1", map_segment_vertices);
    

    // std::cout << "After first fit" << std::endl;
    // for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    //   WCPPID::ProtoSegment *seg = it->first;
    //   std::cout << seg->get_point_vec().size() << " " << std::endl;
    //   for (auto p : seg->get_point_vec()){
    //     std::cout << p << " " << std::endl;
    //   }
    // }
    // std::cout << std::endl;
    

  }

  
  if (flag_2nd_tracking){
    // second round trajectory fit ...
    low_dis_limit = 0.6*units::cm;
    end_point_limit = 0.3*units::cm;
    
    //  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    //   WCPPID::ProtoSegment *seg = it->first;
    //   std::cout << seg->get_point_vec().size() << " " ;
    // }
    // std::cout << std::endl;
    // organize path
    organize_segments_path_2nd(ct_point_cloud, map_vertex_segments, map_segment_vertices, low_dis_limit, end_point_limit);    
   
    // std::cout << "After second organization " << std::endl;
    //  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    //   WCPPID::ProtoSegment *seg = it->first;
    //   std::cout << seg->get_point_vec().size() << " " << std::endl;
    //   for (auto p : seg->get_point_vec()){
    //     std::cout << p << " " << std::endl;
    //   }
    // }
    // std::cout << std::endl;
    
    //    std::cout << "haha3_1 " << std::endl;
    
    map_3D_2DU_set.clear();
    map_3D_2DV_set.clear();
    map_3D_2DW_set.clear();
    map_3D_tuple.clear();
    // map 2D points to 3D indices
    map_2DU_3D_set.clear();
    map_2DV_3D_set.clear();
    map_2DW_3D_set.clear();

    g_wcp_traj_stage = 2;
    form_map_multi_segments(map_vertex_segments, map_segment_vertices, ct_point_cloud,
			    map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,
			    map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set, map_3D_tuple,
			    map_2DU_3D_set, map_2DV_3D_set, map_2DW_3D_set, flag_exclusion);

    multi_trajectory_fit(map_vertex_segments, map_segment_vertices,
			 map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set, map_3D_tuple,
			 map_2DU_3D_set, map_2DV_3D_set, map_2DW_3D_set,
			 map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge);
    wcp_traj_dump_fits("fit2", map_segment_vertices);
    
    // std::cout << "After second fit" << std::endl;
    //  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    //   WCPPID::ProtoSegment *seg = it->first;
    //   std::cout << seg->get_point_vec().size() << " " << std::endl;
    //   for (auto p : seg->get_point_vec()){
    //     std::cout << p << " " << std::endl;
    //   }
    // }
    // std::cout << std::endl;


    // organize path
    low_dis_limit = 0.6*units::cm;
    organize_segments_path_3rd(ct_point_cloud, map_vertex_segments, map_segment_vertices, low_dis_limit);

    //  std::cout << "After third organization " << std::endl;
    //   for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end(); it++){
    //   WCPPID::ProtoSegment *seg = it->first;
    //   std::cout << seg->get_point_vec().size() << " " << std::endl;
    //   for (auto p : seg->get_point_vec()){
    //     std::cout << p << " " << std::endl;
    //   }
    // }
    // std::cout << std::endl;
  }
  
  
  if (flag_dQ_dx){
    for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end(); it++){
      if (it->first->get_cluster_id() != cluster_id) continue;
      WCPPID::ProtoVertex *vtx = it->first;
      bool flag_fix = vtx->get_flag_fit_fix();
      vtx->reset_fit_prop();
      vtx->set_flag_fit_fix(flag_fix);
    }
    for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end();it++){
      if (it->first->get_cluster_id() != cluster_id) continue;
      WCPPID::ProtoSegment *seg = it->first;
      seg->reset_fit_prop();
    }
    
    dQ_dx_multi_fit(map_vertex_segments, map_segment_vertices, global_wc_map, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge, time, end_point_limit, flag_dQ_dx_fit_reg);
    wcp_traj_dump_fits("fit3", map_segment_vertices);
  }

  
}

WCP::Point WCPPID::PR3DCluster::get_pos_multi(std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, int>& input){
  WCPPID::ProtoVertex* vtx = std::get<0>(input);
  WCPPID::ProtoSegment* seg = std::get<1>(input);
  int num = std::get<2>(input);
  Point p;
  if (vtx!=0){
    p = vtx->get_fit_pt();
  }else{
    p = seg->get_point_vec().at(num);
  }
  return p;
}

void WCPPID::PR3DCluster::multi_trajectory_fit(WCPPID::Map_Proto_Vertex_Segments& map_vertex_segments, WCPPID::Map_Proto_Segment_Vertices& map_segment_vertices, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<int, std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, int> >& map_3D_tuple, std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set,std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, int charge_div_method, double div_sigma){
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

  // 2D pixel and then the 3D index ...
  std::map<std::tuple<int,int, int>, double> map_Udiv_fac;
  std::map<std::tuple<int,int, int>, double> map_Vdiv_fac;
  std::map<std::tuple<int,int, int>, double> map_Wdiv_fac;

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
	Point tmp_p = get_pos_multi(map_3D_tuple[*it1]);
  	double central_t = slope_x * tmp_p.x + offset_t;
  	double central_ch = slope_yu * tmp_p.y + slope_zu * tmp_p.z + offset_u;
  	double factor = exp(-0.5 * (pow((central_t-it->first.second)*time_slice_width,2) + pow((central_ch-it->first.first)*pitch_u,2))/pow(div_sigma,2));
  	map_Udiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] = factor;
  	sum += factor;
  	//std::cout << it->first.first << " " << it->first.second << " " << *it1 << std::endl;
      }
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
  	map_Udiv_fac[std::make_tuple(it->first.first, it->first.second, *it1)] /= sum;
      }
    }

    for (auto it = map_2DV_3D_set.begin(); it!=map_2DV_3D_set.end(); it++){
      double sum = 0;
      for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
	Point tmp_p = get_pos_multi(map_3D_tuple[*it1]);
  	double central_t = slope_x * tmp_p.x + offset_t;
  	double central_ch = slope_yv * tmp_p.y + slope_zv * tmp_p.z + offset_v;
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
	Point tmp_p = get_pos_multi(map_3D_tuple[*it1]);
  	double central_t = slope_x * tmp_p.x + offset_t;
  	double central_ch = slope_yw * tmp_p.y + slope_zw * tmp_p.z + offset_w;
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

  // deal vertex
  for (auto it = map_vertex_segments.begin(); it!= map_vertex_segments.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoVertex *vtx = it->first;
    int i = vtx->get_fit_index();
    bool flag_fit_fix = vtx->get_flag_fit_fix();
    Point init_p = vtx->get_fit_pt();

    auto init_p_before = init_p;
    
    if (!flag_fit_fix){ // not fix the fit ...
      init_p = fit_point(init_p, i, map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set,
			 map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,
			 map_Udiv_fac, map_Vdiv_fac, map_Wdiv_fac, offset_t, slope_x,
			 offset_u, slope_yu, slope_zu, offset_v, slope_yv, slope_zv,
			 offset_w, slope_yw, slope_zw);
    }
      //  std::cout << "Vertex: " << i << " " << init_p << " " << init_p_before << std::endl;

    vtx->set_fit(init_p, 0, -1, offset_u + 0.5 + (slope_yu * init_p.y + slope_zu * init_p.z), offset_v + 0.5 + (slope_yv * init_p.y + slope_zv * init_p.z)+2400, offset_w + 0.5 + (slope_yw * init_p.y + slope_zw * init_p.z)+4800, offset_t + 0.5 + slope_x * init_p.x, -1);
  }

  // deal tracks 
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
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
    /* for (auto it1=it->second.begin(); it1!=it->second.end(); it1++){ */
    /*   WCPPID::ProtoVertex *vt = *it1; */
    /*   if ( vt->get_wcpt().index == sg->get_wcpt_vec().front().index){ */
    /* 	start_v = vt; */
    /*   }else if ( vt->get_wcpt().index == sg->get_wcpt_vec().back().index){ */
    /* 	end_v = vt; */
    /*   } */
    /* } */
    PointVector init_ps = sg->get_point_vec();
    std::vector<int> init_indices = sg->get_fit_index_vec();
    std::vector<bool> init_fit_skip = sg->get_fit_flag_skip();
    //    std::cout << init_ps.size() << " " << init_indices.size() << " " << init_fit_skip.size() << " " << init_fit_skip.front() << " " << init_fit_skip.back() << std::endl;
    init_ps.front() = start_v->get_fit_pt();
    init_ps.back() = end_v->get_fit_pt();
    
    PointVector final_ps;
    
    for (size_t i=0;i!=init_ps.size();i++){
      if (i==0){
	final_ps.push_back(start_v->get_fit_pt());
      }else if (i+1==init_ps.size()){
	final_ps.push_back(end_v->get_fit_pt());
      }else{
	Point temp_p = init_ps.at(i);
	if (!init_fit_skip.at(i)){
	  temp_p = fit_point(init_ps.at(i), init_indices.at(i), map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set,
			     map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,
			     map_Udiv_fac, map_Vdiv_fac, map_Wdiv_fac, offset_t, slope_x,
			     offset_u, slope_yu, slope_zu, offset_v, slope_yv, slope_zv,
			     offset_w, slope_yw, slope_zw);
	      
	  
	}
		// std::cout << i << " Track: " << init_ps.at(i) << " " << temp_p << std::endl;
	final_ps.push_back(temp_p);
      }
    }

    //    std::cout << final_ps.size() << " ";
    // sort out the results ...
    final_ps = examine_trajectory(final_ps, init_ps, init_indices, map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge, offset_t, slope_x,
			     offset_u, slope_yu, slope_zu, offset_v, slope_yv, slope_zv,
			     offset_w, slope_yw, slope_zw);
    // std::cout << final_ps.size() << " " << init_ps.size() << std::endl;
    
    // set results
    std::vector<double> tmp_dQ_vec(final_ps.size(),0);
    std::vector<double> tmp_dx_vec(final_ps.size(),-1);
    std::vector<double> tmp_pu_vec(final_ps.size(),0);
    std::vector<double> tmp_pv_vec(final_ps.size(),0);
    std::vector<double> tmp_pw_vec(final_ps.size(),0);
    std::vector<double> tmp_pt_vec(final_ps.size(),0);
    std::vector<double> tmp_reduced_chi2_vec(final_ps.size(),0);
    for (size_t i=0;i!=final_ps.size();i++){
      tmp_pu_vec.at(i) = offset_u + 0.5 + (slope_yu * final_ps.at(i).y + slope_zu * final_ps.at(i).z);
      tmp_pv_vec.at(i) = offset_v + 0.5 + (slope_yv * final_ps.at(i).y + slope_zv * final_ps.at(i).z)+2400;
      tmp_pw_vec.at(i) = offset_w + 0.5 + (slope_yw * final_ps.at(i).y + slope_zw * final_ps.at(i).z)+4800;
      tmp_pt_vec.at(i) = offset_t + 0.5 + slope_x * final_ps.at(i).x ;
    }
    sg->set_fit_vec(final_ps, tmp_dQ_vec, tmp_dx_vec, tmp_pu_vec, tmp_pv_vec, tmp_pw_vec, tmp_pt_vec, tmp_reduced_chi2_vec);
    
  }
  
}

WCP::PointVector WCPPID::PR3DCluster::examine_trajectory(WCP::PointVector& final_ps_vec, WCP::PointVector& init_ps_vec, std::vector<int>& init_indices, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, double offset_t, double slope_x, double offset_u, double slope_yu, double slope_zu, double offset_v, double slope_yv, double slope_zv, double offset_w, double slope_yw, double slope_zw){
  
  PointVector result_ps;
  PointVector orig_ps;
  int skip_count = 0;
  for (size_t i=0; i!=final_ps_vec.size(); i++){
    Point p = final_ps_vec.at(i);
    bool flag_skip = skip_trajectory_point(p, i, init_indices.at(i), init_ps_vec, map_3D_2DU_set, map_3D_2DV_set, map_3D_2DW_set, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge, result_ps, offset_t, slope_x,  offset_u,  slope_yu,  slope_zu,  offset_v,  slope_yv,  slope_zv,  offset_w, slope_yw,  slope_zw);
    //  std::cout << i << " Skip " << p << " " << init_ps_vec.size() << " " << init_ps_vec.at(i) << " " << flag_skip << " " << result_ps.size() << std::endl;
    // vertex not moving ...
    if (i == 0 || i+1 == final_ps_vec.size()) flag_skip = false;
    // protection ...
    if (flag_skip){
      skip_count ++;
      if (skip_count <=3)
	continue;
      else
	skip_count = 0;
    }
    orig_ps.push_back(init_ps_vec.at(i));
    result_ps.push_back(p);
  }

  for (size_t i=0;i!=result_ps.size(); i++){
    bool flag_replace = false;
    
    if (i != 0 && i+1 != result_ps.size()) {
      // -1 vs. +1
      double a = sqrt(pow(result_ps.at(i-1).x - result_ps.at(i).x,2)
		      +pow(result_ps.at(i-1).y - result_ps.at(i).y,2)
		      +pow(result_ps.at(i-1).z - result_ps.at(i).z,2));
      double b = sqrt(pow(result_ps.at(i+1).x - result_ps.at(i).x,2)
		      +pow(result_ps.at(i+1).y - result_ps.at(i).y,2)
		      +pow(result_ps.at(i+1).z - result_ps.at(i).z,2));
      double c = sqrt(pow(result_ps.at(i-1).x - result_ps.at(i+1).x,2)
		      +pow(result_ps.at(i-1).y - result_ps.at(i+1).y,2)
		      +pow(result_ps.at(i-1).z - result_ps.at(i+1).z,2));
      double s = (a+b+c)/2.;
      double area1 = sqrt(s*(s-a)*(s-b)*(s-c));

      a = sqrt(pow(result_ps.at(i-1).x - orig_ps.at(i).x,2)
	       +pow(result_ps.at(i-1).y - orig_ps.at(i).y,2)
	       +pow(result_ps.at(i-1).z - orig_ps.at(i).z,2));
      b = sqrt(pow(result_ps.at(i+1).x - orig_ps.at(i).x,2)
	       +pow(result_ps.at(i+1).y - orig_ps.at(i).y,2)
	       +pow(result_ps.at(i+1).z - orig_ps.at(i).z,2));
      s = (a+b+c)/2.;
      double area2 = sqrt(s*(s-a)*(s-b)*(s-c));

      //      std::cout << i << " A " << area1/c << " " << area2/c  << std::endl;

      if (area1 > 1.8*units::mm * c && area1 > 1.7 * area2)
	flag_replace = true;
    }
      
    // -2, +1
    if ((!flag_replace) && i>=2 && i+1 != result_ps.size()){
      double a = sqrt(pow(result_ps.at(i-2).x - result_ps.at(i).x,2)
		      +pow(result_ps.at(i-2).y - result_ps.at(i).y,2)
		      +pow(result_ps.at(i-2).z - result_ps.at(i).z,2));
      double b = sqrt(pow(result_ps.at(i+1).x - result_ps.at(i).x,2)
		      +pow(result_ps.at(i+1).y - result_ps.at(i).y,2)
		      +pow(result_ps.at(i+1).z - result_ps.at(i).z,2));
      double c = sqrt(pow(result_ps.at(i-2).x - result_ps.at(i+1).x,2)
		      +pow(result_ps.at(i-2).y - result_ps.at(i+1).y,2)
		      +pow(result_ps.at(i-2).z - result_ps.at(i+1).z,2));
      double s = (a+b+c)/2.;
      double area1 = sqrt(s*(s-a)*(s-b)*(s-c));

      a = sqrt(pow(result_ps.at(i-2).x - orig_ps.at(i).x,2)
	       +pow(result_ps.at(i-2).y - orig_ps.at(i).y,2)
	       +pow(result_ps.at(i-2).z - orig_ps.at(i).z,2));
      b = sqrt(pow(result_ps.at(i+1).x - orig_ps.at(i).x,2)
	       +pow(result_ps.at(i+1).y - orig_ps.at(i).y,2)
	       +pow(result_ps.at(i+1).z - orig_ps.at(i).z,2));
      s = (a+b+c)/2.;
      double area2 = sqrt(s*(s-a)*(s-b)*(s-c));
      //std::cout << i << " B " << area1/c << " " << area2/c  << std::endl;
      if (area1 > 1.8*units::mm * c && area1 > 1.7 * area2)
	flag_replace = true;	
     
      
    }
    
    
    // -1, +2
    if ((!flag_replace) && i>0 && i+2<result_ps.size()){
      double a = sqrt(pow(result_ps.at(i-1).x - result_ps.at(i).x,2)
		      +pow(result_ps.at(i-1).y - result_ps.at(i).y,2)
		      +pow(result_ps.at(i-1).z - result_ps.at(i).z,2));
      double b = sqrt(pow(result_ps.at(i+2).x - result_ps.at(i).x,2)
		      +pow(result_ps.at(i+2).y - result_ps.at(i).y,2)
		      +pow(result_ps.at(i+2).z - result_ps.at(i).z,2));
      double c = sqrt(pow(result_ps.at(i-1).x - result_ps.at(i+2).x,2)
		      +pow(result_ps.at(i-1).y - result_ps.at(i+2).y,2)
		      +pow(result_ps.at(i-1).z - result_ps.at(i+2).z,2));
      double s = (a+b+c)/2.;
      double area1 = sqrt(s*(s-a)*(s-b)*(s-c));

      a = sqrt(pow(result_ps.at(i-1).x - orig_ps.at(i).x,2)
	       +pow(result_ps.at(i-1).y - orig_ps.at(i).y,2)
	       +pow(result_ps.at(i-1).z - orig_ps.at(i).z,2));
      b = sqrt(pow(result_ps.at(i+2).x - orig_ps.at(i).x,2)
	       +pow(result_ps.at(i+2).y - orig_ps.at(i).y,2)
	       +pow(result_ps.at(i+2).z - orig_ps.at(i).z,2));
      s = (a+b+c)/2.;
      double area2 = sqrt(s*(s-a)*(s-b)*(s-c));

      //      std::cout << i << " C " << area1/c << " " << area2/c  << std::endl;
      
      if (area1 > 1.8*units::mm * c && area1 > 1.7 * area2)
	flag_replace = true;
    }

    
    if (flag_replace){
      result_ps.at(i) = orig_ps.at(i);
      //      std::cout << i << std::endl;
    }
  }
  
  
  return result_ps;
}


Point WCPPID::PR3DCluster::fit_point(Point& init_p, int i, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, std::map<std::tuple<int,int, int>, double>& map_Udiv_fac, std::map<std::tuple<int,int, int>, double>& map_Vdiv_fac, std::map<std::tuple<int,int, int>, double>& map_Wdiv_fac, double offset_t, double slope_x, double offset_u, double slope_yu, double slope_zu, double offset_v, double slope_yv, double slope_zv, double offset_w, double slope_yw, double slope_zw){
  int n_2D_u = 2* map_3D_2DU_set[i].first.size();
  int n_2D_v = 2* map_3D_2DV_set[i].first.size();
  int n_2D_w = 2* map_3D_2DW_set[i].first.size();
  Eigen::VectorXd temp_pos_3D(3),data_u_2D(n_2D_u), data_v_2D(n_2D_v), data_w_2D(n_2D_w);
  Eigen::VectorXd temp_pos_3D_init(3);
  Eigen::SparseMatrix<double> RU(n_2D_u, 3) ;
  Eigen::SparseMatrix<double> RV(n_2D_v, 3) ;
  Eigen::SparseMatrix<double> RW(n_2D_w, 3) ;
  // initialization
  temp_pos_3D_init(0) = init_p.x;
  temp_pos_3D_init(1) = init_p.y;
  temp_pos_3D_init(2) = init_p.z;

  for (size_t i=0;i!=n_2D_u;i++){
    data_u_2D(i) = 0;
  }
  for (size_t i=0;i!=n_2D_v;i++){
    data_v_2D(i) = 0;
  }
  for (size_t i=0;i!=n_2D_w;i++){
    data_w_2D(i) = 0;
  }

  //  std::cout << map_3D_2DU_set[i].first.size() << " " << map_3D_2DV_set[i].first.size() << " " << map_3D_2DW_set[i].first.size() << std::endl;
  
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

    //    std::cout << "U: " << scaling << std::endl;
    
    if (scaling!=0){
      data_u_2D(2*index) =  scaling * (it->first - offset_u);
      data_u_2D(2*index+1) = scaling * (it->second - offset_t);
      
      RU.insert(2*index, 1) = scaling * slope_yu; // Y--> U
      RU.insert(2*index, 2) = scaling * slope_zu; // Z--> U
      RU.insert(2*index+1,0) = scaling * slope_x; // X --> T
    }
    index ++;
  }
  // std::cout << "U plane " << index << " " << std::endl;

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

    //    std::cout << "V: " << scaling << std::endl;
    if (scaling!=0){
      data_v_2D(2*index) =  scaling * (it->first - offset_v);
      data_v_2D(2*index+1) = scaling * (it->second - offset_t);
      
      RV.insert(2*index, 1) = scaling * slope_yv; // Y--> V
      RV.insert(2*index, 2) = scaling * slope_zv; // Z--> V
      RV.insert(2*index+1,0) = scaling * slope_x; // X --> T
    }
    index ++;
  }
  // std::cout << "V plane " << index << " " << std::endl;
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
    //std::cout << "W: " << scaling << std::endl;
    if (scaling!=0){
      data_w_2D(2*index) =  scaling * (it->first - offset_w);
      data_w_2D(2*index+1) = scaling * (it->second - offset_t);
      
      RW.insert(2*index, 2) = scaling * slope_zw; // Z--> W
      RW.insert(2*index+1,0) = scaling * slope_x; // X --> T
    }
    index ++;
  }
  // std::cout << "W plane " << index << " " << std::endl;


  Point final_p;
  
  Eigen::SparseMatrix<double> RUT = Eigen::SparseMatrix<double>(RU.transpose());
  Eigen::SparseMatrix<double> RVT = Eigen::SparseMatrix<double>(RV.transpose());
  Eigen::SparseMatrix<double> RWT = Eigen::SparseMatrix<double>(RW.transpose());
  
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
  Eigen::VectorXd b = RUT * data_u_2D + RVT * data_v_2D + RWT * data_w_2D;// + PMatrixT * pos_3D_init ;
  
  Eigen::SparseMatrix<double> A =   RUT * RU + RVT * RV + RWT * RW ;// + PMatrixT*PMatrix;
  solver.compute(A);
  
  temp_pos_3D = solver.solveWithGuess(b,temp_pos_3D_init);
  
  
  if (std::isnan(solver.error())){
    final_p.x = temp_pos_3D_init(0);
    final_p.y = temp_pos_3D_init(1);
    final_p.z = temp_pos_3D_init(2);
  }else{
    final_p.x = temp_pos_3D(0);
    final_p.y = temp_pos_3D(1);
    final_p.z = temp_pos_3D(2);
    }
  
    
  //  std::cout << init_p << " " << final_p << " " << std::isnan(solver.error()) << std::endl;
  
  return final_p;
}


void WCPPID::PR3DCluster::form_map_multi_segments(WCPPID::Map_Proto_Vertex_Segments& map_vertex_segments, WCPPID::Map_Proto_Segment_Vertices& map_segment_vertices, WCP::ToyCTPointCloud& ct_point_cloud,
						  std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge,
						  std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<int, std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, int> >& map_3D_tuple,
						  std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set, bool flag_exclusion,
						  double end_point_factor, double mid_point_factor, int nlevel, double time_cut, double charge_cut){
  map_3D_2DU_set.clear();
  map_3D_2DV_set.clear();
  map_3D_2DW_set.clear();
  map_3D_tuple.clear();

  map_2DU_3D_set.clear();
  map_2DV_3D_set.clear();
  map_2DW_3D_set.clear();

  for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoVertex *vtx = it->first;
    bool flag_fix = vtx->get_flag_fit_fix();
    vtx->reset_fit_prop();
    vtx->set_flag_fit_fix(flag_fix);
  }
  //  std::vector<ToyPointCloud* > associated_pclouds_steiner;
  ProtoSegmentSelection segments;
  
  for (auto it = map_segment_vertices.begin(); it!=map_segment_vertices.end();it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoSegment *seg = it->first;
    seg->reset_fit_prop();
    //    WCP::ToyPointCloud *pcloud_associated_steiner =  seg->get_associated_pcloud_steiner();
    //if (pcloud_associated_steiner != 0) associated_pclouds_steiner.push_back(pcloud_associated_steiner);
    segments.push_back(seg);
  }


  
  int count = 0;
  int dump_seg_index = -1;   // WCT pr/108 stage dump
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    dump_seg_index++;
    if (sg->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
    if ( (*it->second.begin())->get_wcpt().index == sg->get_wcpt_vec().front().index){
      start_v = (*it->second.begin());
      end_v = (*it->second.rbegin());
    }else{
      end_v = (*it->second.begin());
      start_v = (*it->second.rbegin());
    }
    
    /* for (auto it1=it->second.begin(); it1!=it->second.end(); it1++){ */
    /*   WCPPID::ProtoVertex *vt = *it1; */
    /*   if ( vt->get_wcpt().index == sg->get_wcpt_vec().front().index){ */
    /* 	start_v = vt; */
    /*   }else if ( vt->get_wcpt().index == sg->get_wcpt_vec().back().index){ */
    /* 	end_v = vt; */
    /*   } */
    /* } */
    std::vector<WCP::Point >& pts = sg->get_point_vec();
    PointVector saved_pts;
    std::vector<int> saved_index;
    std::vector<bool> saved_skip;
    
    
    std::vector<double> distances;
    for (size_t i=0;i+1!=pts.size();i++){
      distances.push_back(sqrt(pow(pts.at(i+1).x-pts.at(i).x,2) +
			       pow(pts.at(i+1).y-pts.at(i).y,2) +
			       pow(pts.at(i+1).z-pts.at(i).z,2)));
    }
    
    for (size_t i=0;i!=pts.size();i++){
      double dis_cut;
      if (i==0){
	dis_cut = std::min(distances.at(i) * end_point_factor,4/3.*end_point_factor*units::cm);
	if (start_v->get_fit_range()<0){
	  start_v->set_fit_range(dis_cut);
	}else{
	  if (dis_cut < start_v->get_fit_range()) start_v->set_fit_range(dis_cut);
	}
      }else if (i+1==pts.size()){
	dis_cut = std::min(distances.back() * end_point_factor,4/3.*end_point_factor*units::cm);
	if (end_v->get_fit_range()<0){
	  end_v->set_fit_range(dis_cut);
	}else{
	  if (dis_cut < end_v->get_fit_range()) end_v->set_fit_range(dis_cut);
	}
      }else{
	dis_cut = std::min(std::max(distances.at(i-1)*mid_point_factor,distances.at(i)*mid_point_factor),4/3.*mid_point_factor*units::cm);
      }

      // std::cout << i << " " << pts.at(i) << " " << end_point_factor << " " << mid_point_factor << " " << dis_cut/units::cm << std::endl;
      
      // not the first and last point
      if (i!=0 && i+1!=pts.size()){
        	std::vector<float> temp_flag(3,0);

	std::set<std::pair<int,int> > temp_2dut, temp_2dvt, temp_2dwt;
	form_point_association(pts.at(i), temp_2dut, temp_2dvt, temp_2dwt, ct_point_cloud, dis_cut, nlevel, time_cut);
	const size_t dump_n0[3] = {temp_2dut.size(), temp_2dvt.size(), temp_2dwt.size()};  // WCT pr/108 dump

  // std::cout << i << " " << pts.at(i) << " " << temp_flag.at(0) << " " << temp_flag.at(1) << " " << temp_flag.at(2) << " " << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << " " << dis_cut/units::cm << " " << nlevel << " " << time_cut << std::endl;


	// examine things ...
	//if (sg->get_associated_pcloud_steiner()!=0 )
	//std::cout << pts.at(i) << " " << sqrt(pow(pts.front().x-pts.back().x,2) + pow(pts.front().y-pts.back().y,2) + pow(pts.front().z-pts.back().z,2))/units::cm << std::endl;

	if (flag_exclusion)
	  update_association(temp_2dut, temp_2dvt, temp_2dwt, sg, segments);
	const size_t dump_n1[3] = {temp_2dut.size(), temp_2dvt.size(), temp_2dwt.size()};  // WCT pr/108: after exclusion, before examine
	
      

	// examine ...
	std::vector<int> temp_results = ct_point_cloud.convert_3Dpoint_time_ch(pts.at(i));
	temp_results.at(2)-=2400;
	temp_results.at(3)-=4800;
	if (i==0 || i==1 || i+1 ==pts.size() || i+2 == pts.size()){
	  temp_flag = examine_point_association(temp_results, temp_2dut, temp_2dvt, temp_2dwt, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,true,charge_cut);
	}else{
	  temp_flag = examine_point_association(temp_results, temp_2dut, temp_2dvt, temp_2dwt, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,false,charge_cut);
	}



	if (g_wcp_traj_dump && g_wcp_traj_stage > 0){   // WCT pr/108 stage dump
	  const bool kept = (temp_flag.at(0) + temp_flag.at(1) + temp_flag.at(2) > 0);
	  fprintf(g_wcp_traj_dump, "%d map%d %d %zu %.4f %.4f %.4f %zu %zu %zu %zu %zu %zu %.3f %.3f %.3f %d %.3f %zu %zu %zu\n",
		  g_wcp_traj_call, g_wcp_traj_stage, dump_seg_index, i,
		  pts.at(i).x/units::cm, pts.at(i).y/units::cm, pts.at(i).z/units::cm,
		  dump_n0[0], dump_n0[1], dump_n0[2], dump_n1[0], dump_n1[1], dump_n1[2],
		  temp_flag.at(0), temp_flag.at(1), temp_flag.at(2), (int)kept, dis_cut/units::cm,
		  temp_2dut.size(), temp_2dvt.size(), temp_2dwt.size());
	}
	if (temp_flag.at(0) + temp_flag.at(1) + temp_flag.at(2) > 0){
	  map_3D_2DU_set[count] = std::make_pair(temp_2dut,temp_flag.at(0));
	  map_3D_2DV_set[count] = std::make_pair(temp_2dvt,temp_flag.at(1));
	  map_3D_2DW_set[count] = std::make_pair(temp_2dwt,temp_flag.at(2));
	  map_3D_tuple[count] = std::make_tuple((WCPPID::ProtoVertex*)0,sg,i);
	  
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
	  saved_index.push_back(count);
	  saved_skip.push_back(false);
	  count ++;
	}
      }else if (i==0){ // first point
	saved_pts.push_back(pts.at(i));
	saved_index.push_back(count);
	saved_skip.push_back(true);
	if (start_v->get_fit_index()==-1){
	  start_v->set_fit_index(count);
	  count ++;
	}
      }else if (i+1==pts.size()){ // last point
	saved_pts.push_back(pts.at(i));
	saved_index.push_back(count);
	saved_skip.push_back(true);
	if (end_v->get_fit_index()==-1){
	  end_v->set_fit_index(count);
	  count ++;
	}
      } 
    } // loop over points ...
    // fill the segments ...
    sg->set_fit_associate_vec(saved_pts, saved_index, saved_skip);
    // std::cout << "Form Map: " << saved_pts.size() << " " << map_2DU_3D_set.size() + map_2DV_3D_set.size() + map_2DW_3D_set.size() << " " << map_3D_2DU_set.size() << " " << map_3D_2DV_set.size() << " " << map_3D_2DW_set.size() << std::endl;
    //    std::cout << saved_pts.size() << " " << saved_index.size() << " " << saved_skip.size() << std::endl;
  } // loop over segment

  // deal with all the vertex again ...
  for (auto it = map_vertex_segments.begin(); it!=map_vertex_segments.end(); it++){
    if (it->first->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoVertex *vtx = it->first;
    double dis_cut = vtx->get_fit_range();
    count = vtx->get_fit_index();
    Point pt = vtx->get_fit_pt();

    //std::cout << dis_cut/units::cm << std::endl;
    std::vector<float> temp_flag(3,0);

    std::set<std::pair<int,int> > temp_2dut, temp_2dvt, temp_2dwt; 
    form_point_association(pt, temp_2dut, temp_2dvt, temp_2dwt, ct_point_cloud, dis_cut, nlevel, time_cut); 

    // std::cout << "V " << pt << " " << temp_flag.at(0) << " " << temp_flag.at(1) << " " << temp_flag.at(2) << " " << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << " " << dis_cut/units::cm << " " << nlevel << " " << time_cut<< std::endl;


    // examine ... 
    std::vector<int> temp_results = ct_point_cloud.convert_3Dpoint_time_ch(pt); 
    temp_results.at(2)-=2400; 
    temp_results.at(3)-=4800; 

    //    std::cout << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << std::endl;
    
    temp_flag = examine_point_association(temp_results, temp_2dut, temp_2dvt, temp_2dwt, map_2D_ut_charge, map_2D_vt_charge, map_2D_wt_charge,true,charge_cut); 

    //    std::cout << temp_2dut.size() << " " << temp_2dvt.size() << " " << temp_2dwt.size() << std::endl;
    


    map_3D_2DU_set[count] = std::make_pair(temp_2dut,temp_flag.at(0)); 
    map_3D_2DV_set[count] = std::make_pair(temp_2dvt,temp_flag.at(1)); 
    map_3D_2DW_set[count] = std::make_pair(temp_2dwt,temp_flag.at(2)); 
    map_3D_tuple[count] = std::make_tuple(vtx,(WCPPID::ProtoSegment*)0,0); 
	  
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
   
  }

  // std::cout << "Form Map: "  << " I " <<  " " << map_2DU_3D_set.size() << " " << map_2DV_3D_set.size() << " " << map_2DW_3D_set.size() << " " << map_3D_2DU_set.size() << " " << map_3D_2DV_set.size() << " " << map_3D_2DW_set.size() << std::endl;


}

void WCPPID::PR3DCluster::update_association(std::set<std::pair<int,int> >& temp_2dut, std::set<std::pair<int,int> >& temp_2dvt, std::set<std::pair<int,int> >& temp_2dwt, WCPPID::ProtoSegment* sg, WCPPID::ProtoSegmentSelection& segments){
  
  // get global information
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

  std::set<std::pair<int,int> > save_2dut, save_2dvt, save_2dwt;
  
  for (auto it = temp_2dut.begin(); it!=temp_2dut.end(); it++){
    double x = (it->second - offset_t)/slope_x;
    double y = (it->first - offset_u)*pitch_u;
    
    //    double min_dis = sg->get_associated_pcloud_steiner()->get_closest_2d_dis(x,y,0);
    double min_dis_track = sg->get_closest_2d_dis(x,y,0);

    //    double min_dis1 = 1e9;
    double min_dis1_track = 1e9;
    
    for (size_t i=0;i!=segments.size();i++){ 
      if (segments.at(i)==sg) continue;
      // if (segments.at(i)->get_associated_pcloud_steiner()!=0){
      //	double temp_dis = segments.at(i)->get_associated_pcloud_steiner()->get_closest_2d_dis(x,y,0);
      //	if (temp_dis < min_dis1) min_dis1 = temp_dis;
      //}
      double temp_dis = segments.at(i)->get_closest_2d_dis(x,y,0);
      if (temp_dis < min_dis1_track)
	min_dis1_track = temp_dis;
    } 
    bool flag_save = false;
    
    if (min_dis_track < min_dis1_track || // closer to the main track ...
	min_dis_track < 0.3*units::cm 
	//	|| 	min_dis_track < 0.6*units::cm && min_dis1_track > 0.45*units::cm
	){
      flag_save = true;
      save_2dut.insert(*it);
      
      //  if (sg->get_associated_pcloud_steiner()!=0)
      //std::cout << it->first << " " << it->second << " " << flag_save << " " << min_dis_track/units::cm <<  " " << min_dis1_track/units::cm << std::endl;
      
    }
  }

  for (auto it = temp_2dvt.begin(); it!=temp_2dvt.end(); it++){
    double x = (it->second - offset_t)/slope_x;
    double y = (it->first - offset_v)*pitch_v;
    
    double min_dis_track = sg->get_closest_2d_dis(x,y,1);
    double min_dis1_track = 1e9;
    
    for (size_t i=0;i!=segments.size();i++){ 
      if (segments.at(i)==sg) continue;
      double temp_dis = segments.at(i)->get_closest_2d_dis(x,y,1);
      if (temp_dis < min_dis1_track)
	min_dis1_track = temp_dis;
    } 
    
    if (min_dis_track < min_dis1_track || // closer to the main track ...
	min_dis_track < 0.3*units::cm 
	//|| 	min_dis_track < 0.6*units::cm && min_dis1_track > 0.45*units::cm
	){
      save_2dvt.insert(*it);
    }
  }
  
  for (auto it = temp_2dwt.begin(); it!=temp_2dwt.end(); it++){
    double x = (it->second - offset_t)/slope_x;
    double y = (it->first - offset_w)*pitch_w;
    
    double min_dis_track = sg->get_closest_2d_dis(x,y,2);
    double min_dis1_track = 1e9;
    
    for (size_t i=0;i!=segments.size();i++){ 
      if (segments.at(i)==sg) continue;
      double temp_dis = segments.at(i)->get_closest_2d_dis(x,y,2);
      if (temp_dis < min_dis1_track)
	min_dis1_track = temp_dis;
    } 
    
    if (min_dis_track < min_dis1_track  // closer to the main track ...
	|| min_dis_track < 0.3*units::cm 
	// || min_dis_track < 0.6*units::cm && min_dis1_track > 0.45*units::cm
	){
      save_2dwt.insert(*it);
    }
  }

  //  std::cout << temp_2dut.size() << " " << save_2dut.size() << " " << temp_2dvt.size() << " " << save_2dvt.size() << " " << temp_2dwt.size() << " " << save_2dwt.size() << std::endl;
  
  temp_2dut = save_2dut; 
  temp_2dvt = save_2dvt; 
  temp_2dwt = save_2dwt; 
  
}



void WCPPID::PR3DCluster::organize_segments_path_3rd(WCP::ToyCTPointCloud& ct_point_cloud, WCPPID::Map_Proto_Vertex_Segments& map_vertex_segments, WCPPID::Map_Proto_Segment_Vertices& map_segment_vertices, double step_size){
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

   // examine things
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
    if ( (*it->second.begin())->get_wcpt().index == sg->get_wcpt_vec().front().index){
       start_v = (*it->second.begin());
       end_v = (*it->second.rbegin());
     }else{
       end_v = (*it->second.begin());
       start_v = (*it->second.rbegin());
     }
    //for (auto it1=it->second.begin(); it1!=it->second.end(); it1++){
    // WCPPID::ProtoVertex *vt = *it1;
    //  if ( vt->get_wcpt().index == sg->get_wcpt_vec().front().index){
    //	start_v = vt;
    // }else if ( vt->get_wcpt().index == sg->get_wcpt_vec().back().index){
    //	end_v = vt;
    // }
    //}
    // check the distance between the two vertices ...
    if (sqrt(pow(start_v->get_fit_pt().x - end_v->get_fit_pt().x,2) + pow(start_v->get_fit_pt().y - end_v->get_fit_pt().y,2) + pow(start_v->get_fit_pt().z - end_v->get_fit_pt().z,2)) < 0.01*units::cm){
      //      std::cout << "too close" << std::endl;
      if (map_vertex_segments[start_v].size()==1){
	Point tmp_p(start_v->get_wcpt().x, start_v->get_wcpt().y, start_v->get_wcpt().z);
	start_v->set_fit_pt(tmp_p);
      }
      if (map_vertex_segments[end_v].size()==1){
	Point tmp_p(end_v->get_wcpt().x, end_v->get_wcpt().y, end_v->get_wcpt().z);
	end_v->set_fit_pt(tmp_p);
      }
    }
  }


  
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
    if ( (*it->second.begin())->get_wcpt().index == sg->get_wcpt_vec().front().index){
       start_v = (*it->second.begin());
       end_v = (*it->second.rbegin());
     }else{
       end_v = (*it->second.begin());
       start_v = (*it->second.rbegin());
     }
    //for (auto it1=it->second.begin(); it1!=it->second.end(); it1++){
    // WCPPID::ProtoVertex *vt = *it1;
    // if ( vt->get_wcpt().index == sg->get_wcpt_vec().front().index){
    //	start_v = vt;
    // }else if ( vt->get_wcpt().index == sg->get_wcpt_vec().back().index){
    //	end_v = vt;
    // }
    //}
    bool flag_startv_end = true;
    bool flag_endv_end = true;
    if (map_vertex_segments[start_v].size()>1) flag_startv_end = false;
    if (map_vertex_segments[end_v].size()>1) flag_endv_end = false;
    
    PointVector pts;
    PointVector curr_pts = examine_end_ps_vec(ct_point_cloud, sg->get_point_vec(), flag_startv_end, flag_endv_end); //= sg->get_point_vec();
    Point start_p, end_p;
    
    start_p = curr_pts.front();
    end_p = curr_pts.back();

    //    std::cout << start_p << " " << end_p << std::endl;
    
    // middle points
    pts.push_back(start_p);
    double dis_end;
    double dis_prev;

    // extra distance ...
    double extra_dis = 0;
    for (size_t i=0;i!=curr_pts.size(); i++){
      Point p1 = curr_pts.at(i);
      //      std::cout << i << " " << p1 << std::endl;

      dis_end = sqrt(pow(p1.x-end_p.x,2) + pow(p1.y-end_p.y,2) + pow(p1.z-end_p.z,2)); 
      if (dis_end < step_size) continue;

      dis_prev = sqrt(pow(p1.x-pts.back().x,2)+pow(p1.y-pts.back().y,2)+pow(p1.z-pts.back().z,2));
      if (dis_prev + extra_dis > step_size){
	extra_dis += dis_prev;
	while (extra_dis > step_size){
	  Point tmp_p;
	  tmp_p.x = pts.back().x + (p1.x - pts.back().x)/dis_prev * step_size;
	  tmp_p.y = pts.back().y + (p1.y - pts.back().y)/dis_prev * step_size;
	  tmp_p.z = pts.back().z + (p1.z - pts.back().z)/dis_prev * step_size;
	  pts.push_back(tmp_p);
	  dis_prev = sqrt(pow(p1.x-pts.back().x,2)+pow(p1.y-pts.back().y,2)+pow(p1.z-pts.back().z,2));	  
	  extra_dis -= step_size;
	}
	
      }else if (dis_prev + extra_dis < step_size){
	extra_dis += dis_prev;
	continue;
      }else{
	pts.push_back(p1);
      }
    }

    // proper deal with the end_point ...
    {
      double dis1 = sqrt(pow(pts.back().x-end_p.x,2) + pow(pts.back().y-end_p.y,2) + pow(pts.back().z-end_p.z,2));

      /* if (sg->get_id()==3){ */
      /* 	std::cout << "kkk: " << dis1/units::cm << " " << std::endl; */
      /* 	for (size_t i=0;i+1!=curr_pts.size();i++){ */
      /* 	  std::cout << "kk: " << i << " " << sqrt(pow(curr_pts.at(i).x - curr_pts.at(i+1).x,2) + pow(curr_pts.at(i).y - curr_pts.at(i+1).y,2) + pow(curr_pts.at(i).z - curr_pts.at(i+1).z,2))/units::cm << std::endl; */
      /* 	} */
      /* } */
      
      if (dis1 < step_size * 0.6) {
	if (pts.size()<=1){
	}else{
	  double dis2 = sqrt(pow(pts.back().x - pts.at(pts.size()-2).x,2) + pow(pts.back().y - pts.at(pts.size()-2).y,2) + pow(pts.back().z - pts.at(pts.size()-2).z,2));
	  double dis3 = (dis1+dis2)/2.;

	  Point tmp_p;
	  tmp_p.x =  pts.at(pts.size()-2).x + (pts.back().x - pts.at(pts.size()-2).x)/dis2*dis3;
	  tmp_p.y =  pts.at(pts.size()-2).y + (pts.back().y - pts.at(pts.size()-2).y)/dis2*dis3;
	  tmp_p.z =  pts.at(pts.size()-2).z + (pts.back().z - pts.at(pts.size()-2).z)/dis2*dis3;
	  pts.pop_back();
	  pts.push_back(tmp_p);
	}
      }else if (dis1 > step_size * 1.6){

	int npoints = std::round(dis1/step_size);
	Point p_save = pts.back();
	for (int j=0;j+1<npoints;j++){
	  Point p(p_save.x + (end_p.x-p_save.x) / npoints * (j+1),
		  p_save.y + (end_p.y-p_save.y) / npoints * (j+1),
		  p_save.z + (end_p.z-p_save.z) / npoints * (j+1));
	  pts.push_back(p);
	}
      }
      
      pts.push_back(end_p);
    }


    // ensure there is no single point ...
    if (pts.size()==1){
      //if (sqrt(pow(end_p.x - pts.back().x,2) + pow(end_p.y - pts.back().y,2) + pow(end_p.z - pts.back().z,2) ) >0)
      pts.push_back(end_p);
      //else{
      //	pts.clear();
      //	Point tmp1(start_v->get_wcpt().x, start_v->get_wcpt().y, start_v->get_wcpt().z);
      //	pts.push_back(tmp1);
      //Point tmp2(end_v->get_wcpt().x, end_v->get_wcpt().y, end_v->get_wcpt().z);
      //pts.push_back(tmp2);
      //}
    }
    
    //    for (size_t i=1;i<pts.size();i++){
    //  std::cout << i << " " << sqrt(pow(pts.at(i).x-pts.at(i-1).x,2) + pow(pts.at(i).y-pts.at(i-1).y,2) + pow(pts.at(i).z-pts.at(i-1).z,2))/units::cm << std::endl;
    //}
    
    
    std::vector<double> tmp_dQ_vec(pts.size(),0);
    std::vector<double> tmp_dx_vec(pts.size(),-1);
    std::vector<double> tmp_pu_vec(pts.size(),0);
    std::vector<double> tmp_pv_vec(pts.size(),0);
    std::vector<double> tmp_pw_vec(pts.size(),0);
    std::vector<double> tmp_pt_vec(pts.size(),0);
    std::vector<double> tmp_reduced_chi2_vec(pts.size(),0);
    for (size_t i=0;i!=pts.size();i++){
      tmp_pu_vec.at(i) = offset_u + 0.5 + (slope_yu * pts.at(i).y + slope_zu * pts.at(i).z);
      tmp_pv_vec.at(i) = offset_v + 0.5 + (slope_yv * pts.at(i).y + slope_zv * pts.at(i).z)+2400;
      tmp_pw_vec.at(i) = offset_w + 0.5 + (slope_yw * pts.at(i).y + slope_zw * pts.at(i).z)+4800;
      tmp_pt_vec.at(i) = offset_t + 0.5 + slope_x * pts.at(i).x ;
    }
    sg->set_fit_vec(pts, tmp_dQ_vec, tmp_dx_vec, tmp_pu_vec, tmp_pv_vec, tmp_pw_vec, tmp_pt_vec, tmp_reduced_chi2_vec);

    
    //    std::cout << sg << " " << pts.front() << " " << pts.back() << " " << start_v << " " << end_v << std::endl;
    //    std::cout << sg << flag_startv_end << " " << flag_endv_end << std::endl;
    //    std::cout << sg << " " << start_v << " " << end_v << std::endl;
    
  }

  
}

void WCPPID::PR3DCluster::organize_segments_path_2nd(WCP::ToyCTPointCloud& ct_point_cloud, WCPPID::Map_Proto_Vertex_Segments& map_vertex_segments, WCPPID::Map_Proto_Segment_Vertices& map_segment_vertices,double low_dis_limit, double end_point_limit){
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


  // examine things
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
    if ( (*it->second.begin())->get_wcpt().index == sg->get_wcpt_vec().front().index){
       start_v = (*it->second.begin());
       end_v = (*it->second.rbegin());
     }else{
       end_v = (*it->second.begin());
       start_v = (*it->second.rbegin());
     }
    //for (auto it1=it->second.begin(); it1!=it->second.end(); it1++){
    // WCPPID::ProtoVertex *vt = *it1;
    // if ( vt->get_wcpt().index == sg->get_wcpt_vec().front().index){
    //	start_v = vt;
    // }else if ( vt->get_wcpt().index == sg->get_wcpt_vec().back().index){
    //	end_v = vt;
    // }
    //}
    // check the distance between the two vertices ...
    if (sqrt(pow(start_v->get_fit_pt().x - end_v->get_fit_pt().x,2) + pow(start_v->get_fit_pt().y - end_v->get_fit_pt().y,2) + pow(start_v->get_fit_pt().z - end_v->get_fit_pt().z,2)) < 0.01*units::cm){
      //      std::cout << "too close" << std::endl;
      if (map_vertex_segments[start_v].size()==1){
	Point tmp_p(start_v->get_wcpt().x, start_v->get_wcpt().y, start_v->get_wcpt().z);
	start_v->set_fit_pt(tmp_p);
      }
      if (map_vertex_segments[end_v].size()==1){
	Point tmp_p(end_v->get_wcpt().x, end_v->get_wcpt().y, end_v->get_wcpt().z);
	end_v->set_fit_pt(tmp_p);
      }
    }
  }
  
  // start organization 
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != cluster_id) continue;
    WCPPID::ProtoVertex *start_v = 0, *end_v = 0;
    if ( (*it->second.begin())->get_wcpt().index == sg->get_wcpt_vec().front().index){
       start_v = (*it->second.begin());
       end_v = (*it->second.rbegin());
     }else{
       end_v = (*it->second.begin());
       start_v = (*it->second.rbegin());
     }
    //for (auto it1=it->second.begin(); it1!=it->second.end(); it1++){
    // WCPPID::ProtoVertex *vt = *it1;
    // if ( vt->get_wcpt().index == sg->get_wcpt_vec().front().index){
    //	start_v = vt;
    //}else if ( vt->get_wcpt().index == sg->get_wcpt_vec().back().index){
    //	end_v = vt;
    //}
    //}
    bool flag_startv_end = true;
    bool flag_endv_end = true;
    if (map_vertex_segments[start_v].size()>1) flag_startv_end = false;
    if (map_vertex_segments[end_v].size()>1) flag_endv_end = false;

    PointVector pts;
    PointVector curr_pts = examine_end_ps_vec(ct_point_cloud, sg->get_point_vec(), flag_startv_end, flag_endv_end);
    Point start_p, end_p;

    if (!start_v->get_flag_fit_fix()){
      // start_ point
      start_p = curr_pts.front();
      
      if (flag_startv_end){
	Point p2 = curr_pts.front();
	double dis1 = 0;
	for (auto it = curr_pts.begin(); it!= curr_pts.end(); it++){
	  p2 = (*it);
	  dis1 = sqrt(pow(start_p.x-p2.x,2)+pow(start_p.y-p2.y,2)+pow(start_p.z-p2.z,2));
	  if (dis1 > low_dis_limit) break;
	}
	if (dis1!=0){
	  start_p.x += (start_p.x-p2.x)/dis1*end_point_limit;
	  start_p.y += (start_p.y-p2.y)/dis1*end_point_limit;
	  start_p.z += (start_p.z-p2.z)/dis1*end_point_limit;
	}
      }
      start_v->set_fit_pt(start_p);
    }else{
      start_p = start_v->get_fit_pt();
    }

    if (!end_v->get_flag_fit_fix()){
      // end_point
      end_p = curr_pts.back();
      
      if (flag_endv_end){
	Point p2 = curr_pts.back();
	double dis1 = 0;
	for (auto it = curr_pts.rbegin(); it!=curr_pts.rend(); it++){
	  p2 = (*it);
	  dis1 = sqrt(pow(end_p.x-p2.x,2)+pow(end_p.y-p2.y,2)+pow(end_p.z-p2.z,2));
	  if (dis1 > low_dis_limit) break;
	}
	if (dis1!=0){
	  end_p.x += (end_p.x - p2.x)/dis1 * end_point_limit;
	  end_p.y += (end_p.y - p2.y)/dis1 * end_point_limit;
	  end_p.z += (end_p.z - p2.z)/dis1 * end_point_limit;
	}
      }
      
      end_v->set_fit_pt(end_p);
    }else{
      end_p = end_v->get_fit_pt();
    }

    
    //    std::cout << start_p << " " << end_p << std::endl;
    

    
    // middle points
    pts.push_back(start_p);
    for (size_t i=0;i!=curr_pts.size(); i++){
      Point p1 = curr_pts.at(i);
      double dis = low_dis_limit;
      double dis1 = sqrt(pow(p1.x-end_p.x,2) + pow(p1.y-end_p.y,2) + pow(p1.z-end_p.z,2));
      if (pts.size()>0)
	dis = sqrt(pow(p1.x-pts.back().x,2)+pow(p1.y-pts.back().y,2)+pow(p1.z-pts.back().z,2));

      if (dis1 < low_dis_limit * 0.8){
	continue;
      }else if (dis < low_dis_limit * 0.8 ){
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

    {
      double dis1 = sqrt(pow(pts.back().x-end_p.x,2) + pow(pts.back().y-end_p.y,2) + pow(pts.back().z-end_p.z,2));
      if (dis1 < low_dis_limit * 0.2) {
	if (pts.size()>1) pts.pop_back();
      }else if (dis1 > low_dis_limit * 1.6){

	int npoints = std::round(dis1/low_dis_limit);
	Point p_save = pts.back();
	for (int j=0;j+1<npoints;j++){
	  Point p(p_save.x + (end_p.x-p_save.x) / npoints * (j+1),
		  p_save.y + (end_p.y-p_save.y) / npoints * (j+1),
		  p_save.z + (end_p.z-p_save.z) / npoints * (j+1));
	  pts.push_back(p);
	}
	
	//Point tmp_p;
	//tmp_p.x =  pts.back().x + (end_p.x - pts.back().x )/2.;
	//tmp_p.y =  pts.back().y + (end_p.y - pts.back().y )/2.;
	//tmp_p.z =  pts.back().z + (end_p.z - pts.back().z )/2.;
	//pts.push_back(tmp_p);
      }
      pts.push_back(end_p);
    }
    
    if (pts.size()==1){
      //      if (sqrt(pow(end_p.x - pts.back().x,2) + pow(end_p.y - pts.back().y,2) + pow(end_p.z - pts.back().z,2) ) >0)
      pts.push_back(end_p);
      /* else{ */
      /* 	pts.clear(); */
      /* 	Point tmp1(start_v->get_wcpt().x, start_v->get_wcpt().y, start_v->get_wcpt().z); */
      /* 	pts.push_back(tmp1); */
      /* 	Point tmp2(end_v->get_wcpt().x, end_v->get_wcpt().y, end_v->get_wcpt().z); */
      /* 	pts.push_back(tmp2); */
      /* } */
    }
    
    
    // sg->set_point_vec(pts);
    std::vector<double> tmp_dQ_vec(pts.size(),0);
    std::vector<double> tmp_dx_vec(pts.size(),-1);
    std::vector<double> tmp_pu_vec(pts.size(),0);
    std::vector<double> tmp_pv_vec(pts.size(),0);
    std::vector<double> tmp_pw_vec(pts.size(),0);
    std::vector<double> tmp_pt_vec(pts.size(),0);
    std::vector<double> tmp_reduced_chi2_vec(pts.size(),0);
    for (size_t i=0;i!=pts.size();i++){
      tmp_pu_vec.at(i) = offset_u + 0.5 + (slope_yu * pts.at(i).y + slope_zu * pts.at(i).z);
      tmp_pv_vec.at(i) = offset_v + 0.5 + (slope_yv * pts.at(i).y + slope_zv * pts.at(i).z)+2400;
      tmp_pw_vec.at(i) = offset_w + 0.5 + (slope_yw * pts.at(i).y + slope_zw * pts.at(i).z)+4800;
      tmp_pt_vec.at(i) = offset_t + 0.5 + slope_x * pts.at(i).x ;
    }
    sg->set_fit_vec(pts, tmp_dQ_vec, tmp_dx_vec, tmp_pu_vec, tmp_pv_vec, tmp_pw_vec, tmp_pt_vec, tmp_reduced_chi2_vec);
    
    
    //    std::cout << sg << " " << pts.front() << " " << pts.back() << " " << start_v << " " << end_v << std::endl;
    //    std::cout << sg << flag_startv_end << " " << flag_endv_end << std::endl;
    //    std::cout << sg << " " << start_v << " " << end_v << std::endl;
  }
}

void WCPPID::PR3DCluster::organize_segments_path(WCP::ToyCTPointCloud& ct_point_cloud, WCPPID::Map_Proto_Vertex_Segments& map_vertex_segments, WCPPID::Map_Proto_Segment_Vertices& map_segment_vertices, double low_dis_limit, double end_point_limit){
  
  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if (sg->get_cluster_id() != cluster_id) continue;
    
     WCPPID::ProtoVertex *start_v = 0, *end_v = 0;     
    /* for (auto it1=it->second.begin(); it1!=it->second.end(); it1++){ */
    /*   WCPPID::ProtoVertex *vt = *it1; */
    /*   if ( vt->get_wcpt().index == sg->get_wcpt_vec().front().index){ */
    /* 	start_v = vt; */
    /*   }else if ( vt->get_wcpt().index == sg->get_wcpt_vec().back().index){ */
    /* 	end_v = vt; */
    /*   } */
    /* } */
     if ( (*it->second.begin())->get_wcpt().index == sg->get_wcpt_vec().front().index){
       start_v = (*it->second.begin());
       end_v = (*it->second.rbegin());
     }else{
       end_v = (*it->second.begin());
       start_v = (*it->second.rbegin());
     }
     
    
    bool flag_startv_end = true;
    bool flag_endv_end = true;
    if (map_vertex_segments[start_v].size()>1) flag_startv_end = false;
    if (map_vertex_segments[end_v].size()>1) flag_endv_end = false;

    PointVector pts;
    std::vector<WCPointCloud<double>::WCPoint> temp_wcps_vec = sg->get_wcpt_vec();
    Point start_p, end_p;

    if (!start_v->get_flag_fit_fix()){
      // start_point
      start_p.x = temp_wcps_vec.front().x;
      start_p.y = temp_wcps_vec.front().y;
      start_p.z = temp_wcps_vec.front().z;
      
      if (flag_startv_end){
	Point p2(temp_wcps_vec.front().x, temp_wcps_vec.front().y, temp_wcps_vec.front().z);
	double dis1 = 0;
	for (auto it = temp_wcps_vec.begin(); it!=temp_wcps_vec.end(); it++){
	  p2.x = (*it).x;
	  p2.y = (*it).y;
	  p2.z = (*it).z;
	  dis1 = sqrt(pow(start_p.x-p2.x,2)+pow(start_p.y-p2.y,2)+pow(start_p.z-p2.z,2));
	  if (dis1 > low_dis_limit) break;
	}
	if (dis1!=0){
	  start_p.x += (start_p.x-p2.x)/dis1*end_point_limit;
	  start_p.y += (start_p.y-p2.y)/dis1*end_point_limit;
	  start_p.z += (start_p.z-p2.z)/dis1*end_point_limit;
	}
      }
      start_v->set_fit_pt(start_p);
    }else{
      start_p = start_v->get_fit_pt();
    }

    if (!end_v->get_flag_fit_fix()){
      // end_point
      end_p.x = temp_wcps_vec.back().x;
      end_p.y = temp_wcps_vec.back().y;
      end_p.z = temp_wcps_vec.back().z;
      if (flag_endv_end){
	Point p2(temp_wcps_vec.back().x, temp_wcps_vec.back().y, temp_wcps_vec.back().z);
	double dis1 = 0;
	for (auto it = temp_wcps_vec.rbegin(); it!=temp_wcps_vec.rend(); it++){
	  p2.x = (*it).x;
	  p2.y = (*it).y;
	  p2.z = (*it).z;
	  dis1 = sqrt(pow(end_p.x-p2.x,2)+pow(end_p.y-p2.y,2)+pow(end_p.z-p2.z,2));
	  if (dis1 > low_dis_limit) break;
	}
	if (dis1!=0){
	  end_p.x += (end_p.x - p2.x)/dis1 * end_point_limit;
	  end_p.y += (end_p.y - p2.y)/dis1 * end_point_limit;
	  end_p.z += (end_p.z - p2.z)/dis1 * end_point_limit;
	}
      }
      end_v->set_fit_pt(end_p);
    }else{
      end_p = end_v->get_fit_pt();
    }

    //    std::cout << start_p << " " << end_p << std::endl;
    
    // middle points
    pts.push_back(start_p);
    for (size_t i=0;i!=temp_wcps_vec.size(); i++){
      Point p1(temp_wcps_vec.at(i).x, temp_wcps_vec.at(i).y, temp_wcps_vec.at(i).z);
      double dis = low_dis_limit;
      double dis1 = sqrt(pow(p1.x-end_p.x,2) + pow(p1.y-end_p.y,2) + pow(p1.z-end_p.z,2));
      if (pts.size()>0)
	dis = sqrt(pow(p1.x-pts.back().x,2)+pow(p1.y-pts.back().y,2)+pow(p1.z-pts.back().z,2));

      if (dis1 < low_dis_limit * 0.8){
	continue;
      }else if (dis < low_dis_limit * 0.8 ){
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

    {
      double dis1 = sqrt(pow(pts.back().x-end_p.x,2) + pow(pts.back().y-end_p.y,2) + pow(pts.back().z-end_p.z,2));
      if (dis1 < low_dis_limit * 0.2) {
	if (pts.size()>1) pts.pop_back();
      }else if (dis1 > low_dis_limit * 1.6){
	
	int npoints = std::round(dis1/low_dis_limit);
	Point p_save = pts.back();
	for (int j=0;j+1<npoints;j++){
	  Point p(p_save.x + (end_p.x-p_save.x) / npoints * (j+1),
		  p_save.y + (end_p.y-p_save.y) / npoints * (j+1),
		  p_save.z + (end_p.z-p_save.z) / npoints * (j+1));
	  pts.push_back(p);
	}
	
	//Point tmp_p;
	//tmp_p.x =  pts.back().x + (end_p.x - pts.back().x )/2.;
	//tmp_p.y =  pts.back().y + (end_p.y - pts.back().y )/2.;
	//tmp_p.z =  pts.back().z + (end_p.z - pts.back().z )/2.;
	//pts.push_back(tmp_p);
      }
      pts.push_back(end_p);
    }

    sg->set_point_vec(pts);

    //    std::cout << sg->get_id() << " " << sg->get_point_vec().size() << " " << pts.front() << " " << pts.back() << std::endl;
    //    std::cout << sg << flag_startv_end << " " << flag_endv_end << std::endl;
    //    std::cout << sg << " " << start_v << " " << end_v << std::endl;
  }
}
    


void WCPPID::PR3DCluster::collect_charge_multi_trajectory(WCPPID::Map_Proto_Segment_Vertices& map_segment_vertices, WCP::ToyCTPointCloud& ct_point_cloud, double dis_cut, double range_cut){
  //clear up ...
  collected_charge_map.clear();
  
  std::set<std::pair<int,int>> existing_tcs;

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


  for (auto it2 = map_segment_vertices.begin(); it2 != map_segment_vertices.end(); it2++){
    WCPPID::ProtoSegment* sg1 = it2->first;
    if (sg1->get_cluster_id() != cluster_id) continue;
    // form a trajectory according to dis and fine tracking?
    PointVector traj_pts;
    //  PointVector& pts = get_fine_tracking_path();
    
    // not using fine tracking results ... 
    PointVector pts;
    std::vector<WCPointCloud<double>::WCPoint>& path_wcps = sg1->get_wcpt_vec();
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
    
    //    std::cout << collected_charge_map.size() << std::endl;
    
  }

  
}



