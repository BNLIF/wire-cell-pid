bool WCPPID::NeutrinoID::ssm_tagger(){

  tagger_info.ssmsp_Ntrack = 0;
  tagger_info.ssmsp_Nsp_tot = 0;

  TVector3 dir_beam(0,0,1);
  TVector3 dir_drift(1,0,0);
  TVector3 dir_vertical(0,1,0);
  TVector3 target_dir(0.46, 0.05, 0.885);
  TVector3 absorber_dir(0.33, 0.75, -0.59);  

  std::cout<<std::endl;
  std::cout<<"Starting SSM tagger"<<std::endl;
  int  Nsm = 0; //Number of Short Muons
  int  Nsm_wivtx = 0; //Number of Short Muons wi vertex activity
  bool backwards_muon = false;
  TVector3 init_dir(0,0,0);
  TVector3 init_dir_10(0,0,0);
  TVector3 init_dir_15(0,0,0);
  TVector3 init_dir_20(0,0,0);
  TVector3 init_dir_10_bp(0,0,0);
  TVector3 init_dir_20_bp(0,0,0);
  TVector3 mom(0,0,0);

  //only filled if there is one ssm
    //properties of the ssm
      //dq/dx info
  double dq_dx_fwd_1 = -999;//1st dq dx point wrt vertex
  double dq_dx_fwd_2 = -999;//2nd dq dx point wrt vertex
  double dq_dx_fwd_3 = -999;//3rd dq dx point wrt vertex
  double dq_dx_fwd_4 = -999;//4th dq dx point wrt vertex
  double dq_dx_fwd_5 = -999;//5th dq dx point wrt vertex
  double dq_dx_bck_1 = -999;//1st dq dx point wrt end
  double dq_dx_bck_2 = -999;//2nd dq dx point wrt end
  double dq_dx_bck_3 = -999;//3rd dq dx point wrt end
  double dq_dx_bck_4 = -999;//4th dq dx point wrt end
  double dq_dx_bck_5 = -999;//5th dq dx point wrt end
  double d_dq_dx_fwd_12 = -999;//1st d dq dx point wrt vertex
  double d_dq_dx_fwd_23 = -999;//2nd d dq dx point wrt vertex
  double d_dq_dx_fwd_34 = -999;//3rd d dq dx point wrt vertex
  double d_dq_dx_fwd_45 = -999;//4th d dq dx point wrt vertex
  double d_dq_dx_bck_12 = -999;//1st d dq dx point wrt end
  double d_dq_dx_bck_23 = -999;//2nd d dq dx point wrt end
  double d_dq_dx_bck_34 = -999;//3rd d dq dx point wrt end
  double d_dq_dx_bck_45 = -999;//4th d dq dx point wrt end
  double max_dq_dx_fwd_3 = -999;//max dq/dx in first 3 points wrt vertex
  double max_dq_dx_fwd_5 = -999;//max dq/dx in first 5 points wrt vertex
  double max_dq_dx_bck_3 = -999;//max dq/dx in first 3 points wrt end
  double max_dq_dx_bck_5 = -999;//max dq/dx in first 5 points wrt end
  double max_d_dq_dx_fwd_3 = -999;//max d dq/dx in first 3 points wrt vertex
  double max_d_dq_dx_fwd_5 = -999;//max d dq/dx in first 5 points wrt vertex
  double max_d_dq_dx_bck_3 = -999;//max d dq/dx in first 3 points wrt end
  double max_d_dq_dx_bck_5 = -999;//max d dq/dx in first 5 points wrt end
  double medium_dq_dx = -999;//medium dq/dx over the entire track
  double medium_dq_dx_bp = -999;//medium dq/dx over the entire track without the vertex acivity
      //angluar info
  double angle_to_z = -999;//angle to the z det cord
  double angle_to_target = -999;//angle to the target
  double angle_to_absorber = -999;//angle to the absorber
  double angle_to_vertical = -999;//angle to the vertical

  double angle_to_z_10 = -999;//angle to the z det cord
  double angle_to_target_10 = -999;//angle to the target
  double angle_to_absorber_10 = -999;//angle to the absorber
  double angle_to_vertical_10 = -999;//angle to the vertical

  double angle_to_z_15 = -999;//angle to the z det cord
  double angle_to_target_15 = -999;//angle to the target
  double angle_to_absorber_15 = -999;//angle to the absorber
  double angle_to_vertical_15 = -999;//angle to the vertical

  double angle_to_z_20 = -999;//angle to the z det cord
  double angle_to_target_20 = -999;//angle to the target
  double angle_to_absorber_20 = -999;//angle to the absorber
  double angle_to_vertical_20 = -999;//angle to the vertical
    //directional info
  double x_dir = -999;//x dir det cord
  double y_dir = -999;//y dir det cord
  double z_dir = -999;//z dir det cord
    //energy info
  double kine_energy = -999;//KE of the muon with range
  double kine_energy_reduced = -999;//KE of the muon with range, exluding high dqdx at vertex
      //general properties
  double vtx_activity = 0;//flag for identified vertex activity
  double pdg = -999;//initial pdg assigned to the track
  double dQ_dx_cut = -999;//cut normally used to choose muons
  double score_mu_fwd = -999;//ID score normally normally used to choose muons
  double score_p_fwd = -999;//ID score normally normally used to choose protons
  double score_e_fwd = -999;//ID score normally normally used to choose e
  double score_mu_bck = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck = -999;//ID score normally normally used to choose e but calculated backwards
  double score_mu_fwd_bp = -999;//ID score normally normally used to choose muons but calculated without the vertex activity
  double score_p_fwd_bp = -999;//ID score normally normally used to choose protons but calculated without the vertex activity
  double score_e_fwd_bp = -999;//ID score normally normally used to choose e but calculated without the vertex activity
      //track "straighness"
  double length = -999;//length of track direct caluclated from point to point
  double direct_length = -999;//length of track direct from end to end
  //double direct_length/length = -999;
  double max_dev = -999;//furthest distance from track's primary axis along the track
    //number of other particles
  double n_prim_tracks_1 = 0;//number of other primary tracks greater than 1 cm
  double n_prim_tracks_3 = 0;//number of other primary tracks greater than 3 cm
  double n_prim_tracks_5 = 0;//number of other primary tracks greater than 5 cm
  double n_prim_tracks_8 = 0;//number of other primary tracks greater than 8 cm
  double n_prim_tracks_11 = 0;//number of other primary tracks greater than 11 cm
  double n_prim_all_1 = 0;//number of other primary tracks and showers greater than 1 cm
  double n_prim_all_3 = 0;//number of other primary tracks and showers greater than 3 cm
  double n_prim_all_5 = 0;//number of other primary tracks and showers greater than 5 cm
  double n_prim_all_8 = 0;//number of other primary tracks and showers greater than 8 cm
  double n_prim_all_11 = 0;//number of other primary tracks and showers greater than 11 cm
  double n_daughter_tracks_1 = 0;//number of other daughter tracks greater than 1 cm
  double n_daughter_tracks_3 = 0;//number of other daughter tracks greater than 3 cm
  double n_daughter_tracks_5 = 0;//number of other daughter tracks greater than 5 cm
  double n_daughter_tracks_8 = 0;//number of other daughter tracks greater than 8 cm
  double n_daughter_tracks_11 = 0;//number of other daughter tracks greater than 11 cm
  double n_daughter_all_1 = 0;//number of other daughter tracks and showers greater than 1 cm
  double n_daughter_all_3 = 0;//number of other daughter tracks and showers greater than 3 cm
  double n_daughter_all_5 = 0;//number of other daughter tracks and showers greater than 5 cm
  double n_daughter_all_8 = 0;//number of other daughter tracks and showers greater than 8 cm
  double n_daughter_all_11 = 0;//number of other daughter tracks and showers greater than 11 cm
    //properties of leading other primary track
  double pdg_prim_track1 = -999;//initial pdg assigned to the track
  double score_mu_fwd_prim_track1 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_prim_track1 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_prim_track1 = -999;//ID score normally normally used to choose e
  double score_mu_bck_prim_track1 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_prim_track1 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_prim_track1 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_prim_track1 = -999;//length of track direct caluclated from point to point
  double direct_length_prim_track1 = -999;//length of track direct from end to end
  double max_dev_prim_track1 = -999;//furthest distance from track's primary axis along the track
  double kine_energy_range_prim_track1 = -999;//KE of the muon with range
  double kine_energy_range_mu_prim_track1 = -999;
  double kine_energy_range_p_prim_track1 = -999;
  double kine_energy_range_e_prim_track1 = -999;
  double kine_energy_cal_prim_track1 = -999;//KE of the track with dq/dx
  double medium_dq_dx_prim_track1 = -999;//medium dq/dx over the entire track
  double x_dir_prim_track1 = -999;//x dir det cord
  double y_dir_prim_track1 = -999;//y dir det cord
  double z_dir_prim_track1 = -999;//z dir det cord
  double add_daught_track_counts_1_prim_track1 = -999;//additional daughter tracks longer than 1 cm
  double add_daught_all_counts_1_prim_track1 = -999;//additional daughter tracks+showers longer than 1 cm
  double add_daught_track_counts_5_prim_track1 = -999;//additional daughter tracks longer than 5 cm
  double add_daught_all_counts_5_prim_track1 = -999;//additional daughter tracks+showers longer than 5 cm
  double add_daught_track_counts_11_prim_track1 = -999;//additional daughter tracks longer than 11 cm
  double add_daught_all_counts_11_prim_track1 = -999;//additional daughter tracks+showers longer than 11 cm
  //properties of sub-leading other primary track
  double pdg_prim_track2 = -999;//initial pdg assigned to the track
  double score_mu_fwd_prim_track2 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_prim_track2 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_prim_track2 = -999;//ID score normally normally used to choose e
  double score_mu_bck_prim_track2 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_prim_track2 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_prim_track2 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_prim_track2 = -999;//length of track direct caluclated from point to point
  double direct_length_prim_track2 = -999;//length of track direct from end to end
  double max_dev_prim_track2 = -999;//furthest distance from track's primary axis along the track
  double kine_energy_range_prim_track2 = -999;//KE of the track with range
  double kine_energy_range_mu_prim_track2 = -999;
  double kine_energy_range_p_prim_track2 = -999;
  double kine_energy_range_e_prim_track2 = -999;
  double kine_energy_cal_prim_track2 = -999;//KE of the track with dq/dx
  double medium_dq_dx_prim_track2 = -999;//medium dq/dx over the entire track
  double x_dir_prim_track2 = -999;//x dir det cord
  double y_dir_prim_track2 = -999;//y dir det cord
  double z_dir_prim_track2 = -999;//z dir det cord
  double add_daught_track_counts_1_prim_track2 = -999;//additional daughter tracks longer than 1 cm
  double add_daught_all_counts_1_prim_track2 = -999;//additional daughter tracks+showers longer than 1 cm
  double add_daught_track_counts_5_prim_track2 = -999;//additional daughter tracks longer than 5 cm
  double add_daught_all_counts_5_prim_track2 = -999;//additional daughter tracks+showers longer than 5 cm
  double add_daught_track_counts_11_prim_track2 = -999;//additional daughter tracks longer than 11 cm
  double add_daught_all_counts_11_prim_track2 = -999;//additional daughter tracks+showers longer than 11 cm
    //properties of leading daughter track
  double pdg_daught_track1 = -999;//initial pdg assigned to the track
  double score_mu_fwd_daught_track1 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_daught_track1 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_daught_track1 = -999;//ID score normally normally used to choose e
  double score_mu_bck_daught_track1 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_daught_track1 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_daught_track1 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_daught_track1 = -999;//length of track direct caluclated from point to point
  double direct_length_daught_track1 = -999;//length of track direct from end to end
  double max_dev_daught_track1 = -999;//furthest distance from track's daughtary axis along the track
  double kine_energy_range_daught_track1 = -999;//KE of the track with range
  double kine_energy_range_mu_daught_track1 = -999;
  double kine_energy_range_p_daught_track1 = -999;
  double kine_energy_range_e_daught_track1 = -999;
  double kine_energy_cal_daught_track1 = -999;//KE of the track with dq/dx
  double medium_dq_dx_daught_track1 = -999;//medium dq/dx over the entire track 
  double x_dir_daught_track1 = -999;//x dir det cord
  double y_dir_daught_track1 = -999;//y dir det cord
  double z_dir_daught_track1 = -999;//z dir det cord
  double add_daught_track_counts_1_daught_track1 = -999;//additional daughter tracks longer than 1 cm
  double add_daught_all_counts_1_daught_track1 = -999;//additional daughter tracks+showers longer than 1 cm
  double add_daught_track_counts_5_daught_track1 = -999;//additional daughter tracks longer than 5 cm
  double add_daught_all_counts_5_daught_track1 = -999;//additional daughter tracks+showers longer than 5 cm
  double add_daught_track_counts_11_daught_track1 = -999;//additional daughter tracks longer than 11 cm
  double add_daught_all_counts_11_daught_track1 = -999;//additional daughter tracks+showers longer than 11 cm
  //properties of sub-leading daughter track
  double pdg_daught_track2 = -999;//initial pdg assigned to the track
  double score_mu_fwd_daught_track2 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_daught_track2 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_daught_track2 = -999;//ID score normally normally used to choose e
  double score_mu_bck_daught_track2 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_daught_track2 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_daught_track2 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_daught_track2 = -999;//length of track direct caluclated from point to point
  double direct_length_daught_track2 = -999;//length of track direct from end to end
  double max_dev_daught_track2 = -999;//furthest distance from track's daughtary axis along the track
  double kine_energy_range_daught_track2 = -999;//KE of the muon with range
  double kine_energy_range_mu_daught_track2 = -999;
  double kine_energy_range_p_daught_track2 = -999;
  double kine_energy_range_e_daught_track2 = -999;
  double kine_energy_cal_daught_track2 = -999;//KE of the muon with dq/dx
  double medium_dq_dx_daught_track2 = -999;//medium dq/dx over the entire track
  double x_dir_daught_track2 = -999;//x dir det cord
  double y_dir_daught_track2 = -999;//y dir det cord
  double z_dir_daught_track2 = -999;//z dir det cord
  double add_daught_track_counts_1_daught_track2 = -999;//additional daughter tracks longer than 1 cm
  double add_daught_all_counts_1_daught_track2 = -999;//additional daughter tracks+showers longer than 1 cm
  double add_daught_track_counts_5_daught_track2 = -999;//additional daughter tracks longer than 5 cm
  double add_daught_all_counts_5_daught_track2 = -999;//additional daughter tracks+showers longer than 5 cm
  double add_daught_track_counts_11_daught_track2 = -999;//additional daughter tracks longer than 11 cm
  double add_daught_all_counts_11_daught_track2 = -999;//additional daughter tracks+showers longer than 11 cm
    //properties of leading other primary shower
  double pdg_prim_shw1 = -999;//initial pdg assigned to the shw
  double score_mu_fwd_prim_shw1 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_prim_shw1 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_prim_shw1 = -999;//ID score normally normally used to choose e
  double score_mu_bck_prim_shw1 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_prim_shw1 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_prim_shw1 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_prim_shw1 = -999;//length of shw direct caluclated from point to point
  double direct_length_prim_shw1 = -999;//length of shw direct from end to end
  double max_dev_prim_shw1 = -999;//furthest distance from shw's primary axis along the shw
  double kine_energy_best_prim_shw1 = -999;//best esitmate of the shower KE
  double kine_energy_range_prim_shw1 = -999;//KE of the muon with range
  double kine_energy_range_mu_prim_shw1 = -999;
  double kine_energy_range_p_prim_shw1 = -999;
  double kine_energy_range_e_prim_shw1 = -999;
  double kine_energy_cal_prim_shw1 = -999;//KE of the muon with dq/dx
  double medium_dq_dx_prim_shw1 = -999;//medium dq/dx over the entire shw
  double x_dir_prim_shw1 = -999;//x dir det cord
  double y_dir_prim_shw1 = -999;//y dir det cord
  double z_dir_prim_shw1 = -999;//z dir det cord
  double add_daught_track_counts_1_prim_shw1 = -999;//additional daughter tracks longer than 1 cm
  double add_daught_all_counts_1_prim_shw1 = -999;//additional daughter tracks+showers longer than 1 cm
  double add_daught_track_counts_5_prim_shw1 = -999;//additional daughter tracks longer than 5 cm
  double add_daught_all_counts_5_prim_shw1 = -999;//additional daughter tracks+showers longer than 5 cm
  double add_daught_track_counts_11_prim_shw1 = -999;//additional daughter tracks longer than 11 cm
  double add_daught_all_counts_11_prim_shw1 = -999;//additional daughter tracks+showers longer than 11 cm
    //properties of sub-leading other primary shower
  double pdg_prim_shw2 = -999;//initial pdg assigned to the shw
  double score_mu_fwd_prim_shw2 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_prim_shw2 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_prim_shw2 = -999;//ID score normally normally used to choose e
  double score_mu_bck_prim_shw2 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_prim_shw2 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_prim_shw2 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_prim_shw2 = -999;//length of shw direct caluclated from point to point
  double direct_length_prim_shw2 = -999;//length of shw direct from end to end
  double max_dev_prim_shw2 = -999;//furthest distance from shw's primary axis along the shw
  double kine_energy_best_prim_shw2 = -999;//best esitmate of the shower KE
  double kine_energy_range_prim_shw2 = -999;//KE of the shower with range
  double kine_energy_range_mu_prim_shw2 = -999;
  double kine_energy_range_p_prim_shw2 = -999;
  double kine_energy_range_e_prim_shw2 = -999;
  double kine_energy_cal_prim_shw2 = -999;//KE of the shower with dq/dx
  double medium_dq_dx_prim_shw2 = -999;//medium dq/dx over the entire shw
  double x_dir_prim_shw2 = -999;//x dir det cord
  double y_dir_prim_shw2 = -999;//y dir det cord
  double z_dir_prim_shw2 = -999;//z dir det cord
  double add_daught_track_counts_1_prim_shw2 = -999;//additional daughter tracks longer than 1 cm
  double add_daught_all_counts_1_prim_shw2 = -999;//additional daughter tracks+showers longer than 1 cm
  double add_daught_track_counts_5_prim_shw2 = -999;//additional daughter tracks longer than 5 cm
  double add_daught_all_counts_5_prim_shw2 = -999;//additional daughter tracks+showers longer than 5 cm 
  double add_daught_track_counts_11_prim_shw2 = -999;//additional daughter tracks longer than 11 cm
  double add_daught_all_counts_11_prim_shw2 = -999;//additional daughter tracks+showers longer than 11 cm
   //properties of leading daughter shower
  double pdg_daught_shw1 = -999;//initial pdg assigned to the shw
  double score_mu_fwd_daught_shw1 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_daught_shw1 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_daught_shw1 = -999;//ID score normally normally used to choose e
  double score_mu_bck_daught_shw1 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_daught_shw1 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_daught_shw1 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_daught_shw1 = -999;//length of shw direct caluclated from point to point
  double direct_length_daught_shw1 = -999;//length of shw direct from end to end
  double max_dev_daught_shw1 = -999;//furthest distance from shw's daughtary axis along the shw
  double kine_energy_best_daught_shw1 = -999;//best esitmate of the shower KE
  double kine_energy_range_daught_shw1 = -999;//KE of the shower with range
  double kine_energy_range_mu_daught_shw1 = -999;
  double kine_energy_range_p_daught_shw1 = -999;
  double kine_energy_range_e_daught_shw1 = -999;
  double kine_energy_cal_daught_shw1 = -999;//KE of the shower with dq/dx
  double medium_dq_dx_daught_shw1 = -999;//medium dq/dx over the entire shw 
  double x_dir_daught_shw1 = -999;//x dir det cord
  double y_dir_daught_shw1 = -999;//y dir det cord
  double z_dir_daught_shw1 = -999;//z dir det cord
  double add_daught_track_counts_1_daught_shw1 = -999;//additional daughter tracks longer than 1 cm
  double add_daught_all_counts_1_daught_shw1 = -999;//additional daughter tracks+showers longer than 1 cm
  double add_daught_track_counts_5_daught_shw1 = -999;//additional daughter tracks longer than 5 cm
  double add_daught_all_counts_5_daught_shw1 = -999;//additional daughter tracks+showers longer than 5 cm
  double add_daught_track_counts_11_daught_shw1 = -999;//additional daughter tracks longer than 11 cm
  double add_daught_all_counts_11_daught_shw1 = -999;//additional daughter tracks+showers longer than 11 cm
    //properties of sub-leading daughter shower
  double pdg_daught_shw2 = -999;//initial pdg assigned to the shw
  double score_mu_fwd_daught_shw2 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_daught_shw2 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_daught_shw2 = -999;//ID score normally normally used to choose e
  double score_mu_bck_daught_shw2 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_daught_shw2 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_daught_shw2 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_daught_shw2 = -999;//length of shw direct caluclated from point to point
  double direct_length_daught_shw2 = -999;//length of shw direct from end to end
  double max_dev_daught_shw2 = -999;//furthest distance from shw's daughtary axis along the shw
  double kine_energy_best_daught_shw2 = -999;//best esitmate of the shower KE
  double kine_energy_range_daught_shw2 = -999;//KE of the shower with range
  double kine_energy_range_mu_daught_shw2 = -999;
  double kine_energy_range_p_daught_shw2 = -999;
  double kine_energy_range_e_daught_shw2 = -999;
  double kine_energy_cal_daught_shw2 = -999;//KE of the shower with dq/dx
  double medium_dq_dx_daught_shw2 = -999;//medium dq/dx over the entire shw
  double x_dir_daught_shw2 = -999;//x dir det cord
  double y_dir_daught_shw2 = -999;//y dir det cord
  double z_dir_daught_shw2 = -999;//z dir det cord
  double add_daught_track_counts_1_daught_shw2 = -999;//additional daughter tracks longer than 1 cm
  double add_daught_all_counts_1_daught_shw2 = -999;//additional daughter tracks+showers longer than 1 cm
  double add_daught_track_counts_5_daught_shw2 = -999;//additional daughter tracks longer than 5 cm
  double add_daught_all_counts_5_daught_shw2 = -999;//additional daughter tracks+showers longer than 5 cm
  double add_daught_track_counts_11_daught_shw2 = -999;//additional daughter tracks longer than 11 cm
  double add_daught_all_counts_11_daught_shw2 = -999;//additional daughter tracks+showers longer than 11 cm
    //event level properties
  double nu_angle_to_z = -999;//angle to the z det cord
  double nu_angle_to_target = -999;
  double nu_angle_to_absorber = -999;
  double nu_angle_to_vertical = -999;
  double con_nu_angle_to_z = -999;//angle to the z det cord
  double con_nu_angle_to_target = -999;
  double con_nu_angle_to_absorber = -999;
  double con_nu_angle_to_vertical = -999;
  double prim_nu_angle_to_z = -999;//angle to the z det cord
  double prim_nu_angle_to_target = -999;
  double prim_nu_angle_to_absorber = -999;
  double prim_nu_angle_to_vertical = -999;
  double track_angle_to_z = -999;//angle to the z det cord
  double track_angle_to_target = -999;
  double track_angle_to_absorber = -999;
  double track_angle_to_vertical = -999;

  //off vertex stuff
  double off_vtx_length = 0;//total length of off vertex segments
  double off_vtx_energy = 0;//total energy of off vertex activity
  double n_offvtx_tracks_1 = 0;//number of other daughter tracks greater than 1 cm
  double n_offvtx_tracks_3 = 0;//number of other daughter tracks greater than 3 cm
  double n_offvtx_tracks_5 = 0;//number of other daughter tracks greater than 5 cm
  double n_offvtx_tracks_8 = 0;//number of other daughter tracks greater than 8 cm
  double n_offvtx_tracks_11 = 0;//number of other daughter tracks greater than 11 cm
  double n_offvtx_showers_1 = 0;//number of other daughter tracks and showers greater than 1 cm
  double n_offvtx_showers_3 = 0;//number of other daughter tracks and showers greater than 3 cm
  double n_offvtx_showers_5 = 0;//number of other daughter tracks and showers greater than 5 cm
  double n_offvtx_showers_8 = 0;//number of other daughter tracks and showers greater than 8 cm
  double n_offvtx_showers_11 = 0;//number of other daughter tracks and showers greater than 11 cm
   //properties of leading off vertex track
  double pdg_offvtx_track1 = -999;//initial pdg assigned to the track
  double score_mu_fwd_offvtx_track1 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_offvtx_track1 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_offvtx_track1 = -999;//ID score normally normally used to choose e
  double score_mu_bck_offvtx_track1 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_offvtx_track1 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_offvtx_track1 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_offvtx_track1 = -999;//length of track direct calculated from point to point
  double direct_length_offvtx_track1 = -999;//length of track direct from end to end
  double max_dev_offvtx_track1 = -999;//furthest distance from track's primary axis along the track
  double kine_energy_range_offvtx_track1 = -999;//KE of the track with range
  double kine_energy_range_mu_offvtx_track1 = -999;
  double kine_energy_range_p_offvtx_track1 = -999;
  double kine_energy_range_e_offvtx_track1 = -999;
  double kine_energy_cal_offvtx_track1 = -999;//KE of the shower with dq/dx
  double medium_dq_dx_offvtx_track1 = -999;//medium dq/dx over the entire track 
  double x_dir_offvtx_track1 = -999;//x dir det cord
  double y_dir_offvtx_track1 = -999;//y dir det cord
  double z_dir_offvtx_track1 = -999;//z dir det cord
  double dist_mainvtx_offvtx_track1 = -999; //distance from the main vertex
   //properties of leading off vertex shower
  double pdg_offvtx_shw1 = -999;//initial pdg assigned to the shw
  double score_mu_fwd_offvtx_shw1 = -999;//ID score normally normally used to choose muons
  double score_p_fwd_offvtx_shw1 = -999;//ID score normally normally used to choose protons
  double score_e_fwd_offvtx_shw1 = -999;//ID score normally normally used to choose e
  double score_mu_bck_offvtx_shw1 = -999;//ID score normally normally used to choose muons but calculated backwards
  double score_p_bck_offvtx_shw1 = -999;//ID score normally normally used to choose protons but calculated backwards
  double score_e_bck_offvtx_shw1 = -999;//ID score normally normally used to choose e but calculated backwards
  double length_offvtx_shw1 = -999;//length of shw direct calculated from point to point
  double direct_length_offvtx_shw1 = -999;//length of shw direct from end to end
  double max_dev_offvtx_shw1 = -999;//furthest distance from shw's primary axis along the shw
  double kine_energy_best_offvtx_shw1 = -999;//best esitmate of the shower KE
  double kine_energy_range_offvtx_shw1 = -999;//KE of the shower with range
  double kine_energy_range_mu_offvtx_shw1 = -999;
  double kine_energy_range_p_offvtx_shw1 = -999;
  double kine_energy_range_e_offvtx_shw1 = -999;
  double kine_energy_cal_offvtx_shw1 = -999;//KE of the shower with dq/dx
  double medium_dq_dx_offvtx_shw1 = -999;//medium dq/dx over the entire shw 
  double x_dir_offvtx_shw1 = -999;//x dir det cord
  double y_dir_offvtx_shw1 = -999;//y dir det cord
  double z_dir_offvtx_shw1 = -999;//z dir det cord
  double dist_mainvtx_offvtx_shw1 = -999; //distance from the main vertex

  //saves all ssm, if they have vtx activity, if they are backwards and their length
  std::map<WCPPID::ProtoSegment*, std::tuple<bool,bool,double> > all_ssm_sg;
  //saves the main ssm
  WCPPID::ProtoSegment* ssm_sg;
  //KDAR tagger, check everything connected to the vertex, make sure there is a ssm
  for (auto it = map_vertex_segments[main_vertex].begin(); it!= map_vertex_segments[main_vertex].end(); it++){
    WCPPID::ProtoSegment *sg = *it;
    double sg_length = sg->get_length()/units::cm;
    double sg_direct_length = sg->get_direct_length()/units::cm;
    bool sg_flag_backwards_muon = false;
    bool sg_flag_vtx_activity = false;
    // Aren't you a little short for a stormtrooper?
    //if( (sg_length <= 34 && sg_length >= 1 && abs(sg->get_particle_type())==13) || (sg_direct_length > 0.9 * sg_length && sg_length <= 32 && sg_length >= 1 && abs(sg->get_particle_type())==11)){
    if( (sg_length <= 46 && sg_length >= 1 && abs(sg->get_particle_type())==13) || (sg_direct_length > 0.9 * sg_length && sg_length <= 44 && sg_length >= 1 && abs(sg->get_particle_type())==11)){
    //veto stuff we still dont want
      if(sg_length<1) {std::cout<<"\n too short"<<std::endl; continue;}//previously was 5
      if(sg->get_medium_dQ_dx()/(43e3/units::cm)<0.95){ std::cout<<"\n bad dq/dx"<<std::endl; continue;}//previously was 1
      Nsm+=1;

      //make the d dq/dx vector
      std::vector<double>& vec_dQ = sg->get_dQ_vec();
      std::vector<double>& vec_dx = sg->get_dx_vec();
      std::vector<double> vec_d_dq_dx;
      bool vtx_activity_fwd = false;
      bool vtx_activity_bck = false;
      double sg_max_d_dq_dx_fwd_5 = 0;
      double sg_max_d_dq_dx_bck_5 = 0;
      double sg_max_d_dq_dx_fwd_3 = 0;//used as the "tiebreaker"
      double sg_max_d_dq_dx_bck_3 = 0;//used as the "tiebreaker"
      double last_dq_dx = vec_dQ.at(0)/vec_dx.at(0)/(43e3/units::cm);
      for (int point = 1; point<vec_dQ.size(); point++){
        double dq_dx = vec_dQ.at(point)/vec_dx.at(point)/(43e3/units::cm);
        vec_d_dq_dx.push_back( dq_dx - last_dq_dx  );
        last_dq_dx = dq_dx;
      }

      //check the first 5 points for a large change in dq/dx
      int end = 4;
      int end_3 = 2;
      if(end>vec_d_dq_dx.size()){end=vec_d_dq_dx.size();}
      if(end_3>vec_d_dq_dx.size()){end_3=vec_d_dq_dx.size();}
      for (int point = 0; point<end; point++){
        if(abs(vec_d_dq_dx.at(point))>0.7 && abs(vec_d_dq_dx.at(point))>sg_max_d_dq_dx_fwd_5){
          vtx_activity_fwd = true;
          sg_max_d_dq_dx_fwd_5 = abs(vec_d_dq_dx.at(point));
	}
	if(abs(vec_d_dq_dx.at(point))>0.7 && abs(vec_d_dq_dx.at(point))>sg_max_d_dq_dx_fwd_3 && point<end_3){
	  sg_max_d_dq_dx_fwd_3 = abs(vec_d_dq_dx.at(point));
        }
      }

      //check the last 5 points for a large change in dq/dx
      int start=vec_d_dq_dx.size()-4;
      int start_3=vec_d_dq_dx.size()-2;
      if(start<0) {start=0;}
      if(start_3<0) {start_3=0;}
      for (int point = start; point<vec_d_dq_dx.size(); point++){
        if(abs(vec_d_dq_dx.at(point))>0.7 && abs(vec_d_dq_dx.at(point))>sg_max_d_dq_dx_bck_5){
          vtx_activity_bck = true;
          sg_max_d_dq_dx_bck_5 = abs(vec_d_dq_dx.at(point));
        }
        if(abs(vec_d_dq_dx.at(point))>0.7 && abs(vec_d_dq_dx.at(point))>sg_max_d_dq_dx_bck_3 && point>=start_3){
          sg_max_d_dq_dx_bck_3 = abs(vec_d_dq_dx.at(point));
        }	
      }

      //decide which direction has vertex activity 
      if(vtx_activity_bck && vtx_activity_fwd){
        if(sg_max_d_dq_dx_fwd_5<1.0) vtx_activity_fwd = false;
        if(sg_max_d_dq_dx_bck_5<1.0) vtx_activity_bck = false;
        if(sg_max_d_dq_dx_bck_5==sg_max_d_dq_dx_fwd_5){
            if(sg_max_d_dq_dx_fwd_3==sg_max_d_dq_dx_bck_3){
              vtx_activity_fwd = false;
              vtx_activity_bck = false;
            }else if(sg_max_d_dq_dx_fwd_3>sg_max_d_dq_dx_bck_3){
              vtx_activity_fwd = true;
              vtx_activity_bck = false;
            }else{
              vtx_activity_fwd = false;
              vtx_activity_bck = true;
            }
        }else if(!vtx_activity_bck && !vtx_activity_fwd){
          if(sg_max_d_dq_dx_fwd_5>=sg_max_d_dq_dx_bck_5){
            vtx_activity_fwd = true;
            vtx_activity_bck = false;
          }else{
            vtx_activity_fwd = false;
            vtx_activity_bck = true;
          }
        }else if(vtx_activity_bck && vtx_activity_fwd){
          if(sg_max_d_dq_dx_fwd_5<1.3) vtx_activity_fwd = false;
          if(sg_max_d_dq_dx_bck_5<1.3) vtx_activity_bck = false;
          if(!vtx_activity_bck && !vtx_activity_fwd){
            if(sg_max_d_dq_dx_fwd_5>=sg_max_d_dq_dx_bck_5){
              vtx_activity_fwd = true;
              vtx_activity_bck = false;
            }else{
              vtx_activity_fwd = false;
              vtx_activity_bck = true;
            }
          }else if(vtx_activity_bck && vtx_activity_fwd){
            if(sg_max_d_dq_dx_fwd_5>=sg_max_d_dq_dx_bck_5){
              vtx_activity_fwd = true;
              vtx_activity_bck = false;
            }else{
              vtx_activity_fwd = false;
              vtx_activity_bck = true;
            }
          }
        }
      }
	    
      //flag the muon as backwards if the direction with vertex activity does not match where the start of the muon is
      if(vtx_activity_bck){
        Nsm_wivtx+=1;
        if(sg->get_flag_dir()==1) {sg_flag_backwards_muon=true;}
        sg_flag_vtx_activity = true;
      }else if(vtx_activity_fwd){
        Nsm_wivtx+=1;
        if(sg->get_flag_dir()==-1) {sg_flag_backwards_muon=true;}
        sg_flag_vtx_activity = true;
      }
      std::cout<<"sg_flag_backwards_muon "<<sg_flag_backwards_muon<<std::endl;
      std::tuple<bool,bool,double> temp_ssm_info = {sg_flag_vtx_activity,sg_flag_backwards_muon,sg_length};
      all_ssm_sg.insert(std::pair<WCPPID::ProtoSegment*, std::tuple<bool,bool,double>>(sg,temp_ssm_info));
    }

  }//end loop over all segments connected to the vertex to determine if we have an ssm


  tagger_info.ssm_Nsm = Nsm;//number of short straight muons
  tagger_info.ssm_Nsm_wivtx = Nsm_wivtx;//number of short straight muons with vertex activity

  std::cout<<"tagger_info.ssm_Nsm "<<tagger_info.ssm_Nsm<<"  tagger_info.ssm_Nsm_wivtx "<<tagger_info.ssm_Nsm_wivtx<<std::endl;
  if( Nsm==0 ){ std::cout<<"Exit Tagger"<<std::endl; return exit_ssm_tagger(); }

  //decision on which muon is the ssm
  //Pick the longest with vtx activity, if none of these just pick the longest
  for (auto it = all_ssm_sg.begin(); it!=all_ssm_sg.end(); it++){
    std::tuple<bool,bool,double> ssm_info = it->second;
    if( get<0>(ssm_info)==true && ( get<2>(ssm_info) > length || vtx_activity==false ) ){      
      ssm_sg = it->first;
      length = get<2>(ssm_info);
      backwards_muon = get<1>(ssm_info);
      vtx_activity = get<0>(ssm_info);
    }
    else if( get<2>(ssm_info) > length && vtx_activity==false){
      ssm_sg = it->first;
      length = get<2>(ssm_info);
      backwards_muon = get<1>(ssm_info);
      vtx_activity = get<0>(ssm_info);
    } 
  }

  pdg = abs(ssm_sg->get_particle_type());
  direct_length = ssm_sg->get_direct_length()/units::cm;

  //get the direction and angle of the muon
  int dir = ssm_sg->get_flag_dir();
  if(backwards_muon) {dir=dir*-1;}
  init_dir = ssm_sg->cal_dir_3vector(dir,5);
  init_dir_10 = ssm_sg->cal_dir_3vector(dir,10);
  init_dir_15 = ssm_sg->cal_dir_3vector(dir,15);
  init_dir_20 = ssm_sg->cal_dir_3vector(dir,20);
  init_dir = init_dir.Unit();
  init_dir_10 = init_dir_10.Unit();
  init_dir_15 = init_dir_15.Unit();
  init_dir_20 = init_dir_20.Unit();
  x_dir = init_dir_10[0];
  y_dir = init_dir_10[1];
  z_dir = init_dir_10[2];
  
  max_dev = ssm_sg->get_max_deviation(int(0),int(ssm_sg->get_point_vec().size()))/units::cm;
    
  angle_to_z = acos( init_dir[0]*dir_beam[0] + init_dir[1]*dir_beam[1] + init_dir[2]*dir_beam[2] );
  angle_to_z_10 = acos( init_dir_10[0]*dir_beam[0] + init_dir_10[1]*dir_beam[1] + init_dir_10[2]*dir_beam[2] );
  angle_to_z_15 = acos( init_dir_15[0]*dir_beam[0] + init_dir_15[1]*dir_beam[1] + init_dir_15[2]*dir_beam[2] );
  angle_to_z_20 = acos( init_dir_20[0]*dir_beam[0] + init_dir_20[1]*dir_beam[1] + init_dir_20[2]*dir_beam[2] );

  angle_to_target = acos( init_dir[0]*target_dir[0] + init_dir[1]*target_dir[1] + init_dir[2]*target_dir[2] );
  angle_to_target_10 = acos( init_dir_10[0]*target_dir[0] + init_dir_10[1]*target_dir[1] + init_dir_10[2]*target_dir[2] );
  angle_to_target_15 = acos( init_dir_15[0]*target_dir[0] + init_dir_15[1]*target_dir[1] + init_dir_15[2]*target_dir[2] );
  angle_to_target_20 = acos( init_dir_20[0]*target_dir[0] + init_dir_20[1]*target_dir[1] + init_dir_20[2]*target_dir[2] );

  angle_to_absorber = acos( init_dir[0]*absorber_dir[0] + init_dir[1]*absorber_dir[1] + init_dir[2]*absorber_dir[2] );
  angle_to_absorber_10 = acos( init_dir_10[0]*absorber_dir[0] + init_dir_10[1]*absorber_dir[1] + init_dir_10[2]*absorber_dir[2] );
  angle_to_absorber_15 = acos( init_dir_15[0]*absorber_dir[0] + init_dir_15[1]*absorber_dir[1] + init_dir_15[2]*absorber_dir[2] );
  angle_to_absorber_20 = acos( init_dir_20[0]*absorber_dir[0] + init_dir_20[1]*absorber_dir[1] + init_dir_20[2]*absorber_dir[2] );

  angle_to_vertical = acos( init_dir[0]*dir_vertical[0] + init_dir[1]*dir_vertical[1] + init_dir[2]*dir_vertical[2] );
  angle_to_vertical_10 = acos( init_dir_10[0]*dir_vertical[0] + init_dir_10[1]*dir_vertical[1] + init_dir_10[2]*dir_vertical[2] );  
  angle_to_vertical_15 = acos( init_dir_15[0]*dir_vertical[0] + init_dir_15[1]*dir_vertical[1] + init_dir_15[2]*dir_vertical[2] );    
  angle_to_vertical_20 = acos( init_dir_20[0]*dir_vertical[0] + init_dir_20[1]*dir_vertical[1] + init_dir_20[2]*dir_vertical[2] );

  //add dq/dx info
  int break_point_fwd = -1;
  int break_point_bck = -1;
  int break_point = 0;
  double reduced_muon_length = length;
  std::vector<double>& vec_dQ = ssm_sg->get_dQ_vec();
  std::vector<double>& vec_dx = ssm_sg->get_dx_vec();
  std::vector<double> vec_dq_dx;
  std::vector<double> vec_d_dq_dx;
  std::vector<double> vec_abs_d_dq_dx;
  double last_dq_dx = vec_dQ.at(0)/vec_dx.at(0)/(43e3/units::cm);
  for (int point = 0; point<vec_dQ.size(); point++){
    double dq_dx = vec_dQ.at(point)/vec_dx.at(point)/(43e3/units::cm);
    vec_dq_dx.push_back( dq_dx  );
    if (point>0) {vec_d_dq_dx.push_back( dq_dx - last_dq_dx  ); vec_abs_d_dq_dx.push_back( abs(dq_dx - last_dq_dx)  );}
    last_dq_dx = dq_dx;
  }

  //check the first 5 points for a large change in dq/dx
  int end = 4;
  bool fwd_vtx_activity = false;
  if(vec_d_dq_dx.size()<end){end = vec_d_dq_dx.size();}
  for (int point = 0; point<end; point++){
    if(fabs(vec_d_dq_dx.at(point))>0.7){
      fwd_vtx_activity = true;
      break_point_fwd = point+1;
    }
  }
  if(dir==1 && fwd_vtx_activity == true){
    for (int point = 0; point<break_point_fwd; point++){
      reduced_muon_length-=abs(vec_dx.at(point)/units::cm);
    }
    break_point = break_point_fwd;
  }
  max_dq_dx_fwd_3 = *std::max_element(vec_dq_dx.begin(), vec_dq_dx.begin()+std::min(int(vec_dq_dx.size()),3));
  max_dq_dx_fwd_5 = *std::max_element(vec_dq_dx.begin(), vec_dq_dx.begin()+std::min(int(vec_dq_dx.size()),5));
  max_d_dq_dx_fwd_3 = *std::max_element(vec_abs_d_dq_dx.begin(), vec_abs_d_dq_dx.begin()+std::min(int(vec_abs_d_dq_dx.size()),2));
  max_d_dq_dx_fwd_5 = *std::max_element(vec_abs_d_dq_dx.begin(), vec_abs_d_dq_dx.begin()+std::min(int(vec_abs_d_dq_dx.size()),4));

  //check the last 5 points for a large change in dq/dx
  int start = vec_d_dq_dx.size()-4;
  bool bck_vtx_activity = false;
  if(start<0){start=0;}
  for (int point = start; point<vec_d_dq_dx.size(); point++){
    if(fabs(vec_d_dq_dx.at(point))>0.7){
      if(bck_vtx_activity==false){break_point_bck = point+1; bck_vtx_activity = true;}
    }
  }

  max_dq_dx_bck_3 = *std::max_element(vec_dq_dx.begin()+ std::max(int(vec_dq_dx.size()-3),0), vec_dq_dx.end());
  max_dq_dx_bck_5 = *std::max_element(vec_dq_dx.begin()+ std::max(int(vec_dq_dx.size()-5),0), vec_dq_dx.end());
  max_d_dq_dx_bck_3 = *std::max_element(vec_abs_d_dq_dx.begin()+ std::max(int(vec_abs_d_dq_dx.size()-2),0), vec_abs_d_dq_dx.end());
  max_d_dq_dx_bck_5 = *std::max_element(vec_abs_d_dq_dx.begin()+ std::max(int(vec_abs_d_dq_dx.size()-4),0), vec_abs_d_dq_dx.end());

  if(dir==-1 && bck_vtx_activity == true){
    for (int point = break_point_bck; point<vec_dx.size(); point++){
      reduced_muon_length-=abs(vec_dx.at(point)/units::cm);
    }
    break_point = break_point_bck;
  }
  
  if(vec_dq_dx.size()<1){dq_dx_fwd_1=0;}
  else {dq_dx_fwd_1 = vec_dq_dx.at(0);}
  if(vec_dq_dx.size()<2){dq_dx_fwd_2=0;}
  else {dq_dx_fwd_2 = vec_dq_dx.at(1);}
  if(vec_dq_dx.size()<3){dq_dx_fwd_3=0;}
  else {dq_dx_fwd_3 = vec_dq_dx.at(2);}
  if(vec_dq_dx.size()<4){dq_dx_fwd_4=0;}
  else {dq_dx_fwd_4 = vec_dq_dx.at(3);}
  if(vec_dq_dx.size()<5){dq_dx_fwd_5=0;}
  else {dq_dx_fwd_5 = vec_dq_dx.at(4);}

  if(vec_dq_dx.size()<1){dq_dx_bck_1=0;}
  else {dq_dx_bck_1 = vec_dq_dx.at( vec_dq_dx.size()-1 );}
  if(vec_dq_dx.size()<2){dq_dx_bck_2=0;}
  else {dq_dx_bck_2 = vec_dq_dx.at( vec_dq_dx.size()-2 );}
  if(vec_dq_dx.size()<3){dq_dx_bck_3=0;}
  else {dq_dx_bck_3 = vec_dq_dx.at( vec_dq_dx.size()-3 );}
  if(vec_dq_dx.size()<4){dq_dx_bck_4=0;}
  else {dq_dx_bck_4 = vec_dq_dx.at( vec_dq_dx.size()-4 );}
  if(vec_dq_dx.size()<5){dq_dx_bck_5=0;}
  else {dq_dx_bck_5 = vec_dq_dx.at( vec_dq_dx.size()-5 );}

  if(vec_d_dq_dx.size()<1){d_dq_dx_fwd_12=0;}
  else {d_dq_dx_fwd_12 = vec_d_dq_dx.at(0);}
  if(vec_d_dq_dx.size()<2){d_dq_dx_fwd_23=0;}
  else {d_dq_dx_fwd_23 = vec_d_dq_dx.at(1);}
  if(vec_d_dq_dx.size()<3){d_dq_dx_fwd_34=0;}
  else {d_dq_dx_fwd_34 = vec_d_dq_dx.at(2);}
  if(vec_d_dq_dx.size()<4){d_dq_dx_fwd_45=0;}
  else {d_dq_dx_fwd_45 = vec_d_dq_dx.at(3);}

  if(vec_d_dq_dx.size()<1){d_dq_dx_bck_12=0;}
  else {d_dq_dx_bck_12 = vec_d_dq_dx.at( vec_d_dq_dx.size()-1 );}
  if(vec_d_dq_dx.size()<2){d_dq_dx_bck_23=0;}
  else {d_dq_dx_bck_23 = vec_d_dq_dx.at( vec_d_dq_dx.size()-2 );}
  if(vec_d_dq_dx.size()<3){d_dq_dx_bck_34=0;}
  else {d_dq_dx_bck_34 = vec_d_dq_dx.at( vec_d_dq_dx.size()-3 );}
  if(vec_d_dq_dx.size()<4){d_dq_dx_bck_45=0;}
  else {d_dq_dx_bck_45 = vec_d_dq_dx.at( vec_d_dq_dx.size()-4 );}

  if(dir==-1){
    std::swap(dq_dx_fwd_1,dq_dx_bck_1);
    std::swap(dq_dx_fwd_2,dq_dx_bck_2);
    std::swap(dq_dx_fwd_3,dq_dx_bck_3);
    std::swap(dq_dx_fwd_4,dq_dx_bck_4);
    std::swap(dq_dx_fwd_5,dq_dx_bck_5);
    std::swap(d_dq_dx_fwd_12,d_dq_dx_bck_12);
    std::swap(d_dq_dx_fwd_23,d_dq_dx_bck_23);
    std::swap(d_dq_dx_fwd_34,d_dq_dx_bck_34);
    std::swap(d_dq_dx_fwd_45,d_dq_dx_bck_45);
    std::swap(max_dq_dx_fwd_3,max_dq_dx_bck_3);
    std::swap(max_dq_dx_fwd_5,max_dq_dx_bck_5);  
    std::swap(max_d_dq_dx_fwd_3,max_d_dq_dx_bck_5);
    std::swap(max_d_dq_dx_fwd_5,max_d_dq_dx_bck_5);
  }

  std::vector<double> scores = get_scores(ssm_sg);
  score_mu_fwd = scores.at(0);
  score_p_fwd = scores.at(1);
  score_e_fwd= scores.at(2);
  score_mu_bck = scores.at(3);
  score_p_bck = scores.at(4);
  score_e_bck = scores.at(5);
  if(dir==-1){
    std::swap(score_mu_fwd,score_mu_bck);
    std::swap(score_p_fwd,score_p_bck);
    std::swap(score_e_fwd,score_e_bck);
  }

  score_mu_fwd_bp = score_mu_fwd;
  score_p_fwd_bp = score_p_fwd;
  score_e_fwd_bp = score_e_fwd;
  if(vtx_activity == true){
    std::vector<double> bp_scores = get_scores(ssm_sg,break_point,dir);
    score_mu_fwd_bp = bp_scores.at(1);
    score_p_fwd_bp = bp_scores.at(2);
    score_e_fwd_bp = bp_scores.at(3);
  }

  dQ_dx_cut = 0.8866+0.9533 *pow(18/length/units::cm, 0.4234);
  medium_dq_dx = ssm_sg->get_medium_dQ_dx()/(43e3/units::cm);
  medium_dq_dx_bp = medium_dq_dx;
  if(dir==1 && vtx_activity == true){medium_dq_dx_bp = ssm_sg->get_medium_dQ_dx(break_point,ssm_sg->get_dQ_vec().size())/(43e3/units::cm);}
  else if(vtx_activity == true){medium_dq_dx_bp = ssm_sg->get_medium_dQ_dx(0,break_point)/(43e3/units::cm);}

  //catch and fix cases where we have a large d(dq/dx) for all points
  if( (break_point==vec_d_dq_dx.size() && dir==1 && vtx_activity) || (break_point==0 && dir==-1 && vtx_activity) ){
    reduced_muon_length = length;
    score_mu_fwd_bp = score_mu_fwd;
    score_p_fwd_bp = score_p_fwd;
    score_e_fwd_bp = score_e_fwd;
    medium_dq_dx_bp = medium_dq_dx;
  }

  //get the energy of the ssm
  TPCParams& mp = Singleton<TPCParams>::Instance();
  TGraph *g_range = 0;
  g_range = mp.get_muon_r2ke();
  kine_energy = g_range->Eval(length) * units::MeV;
  kine_energy_reduced = g_range->Eval(reduced_muon_length) * units::MeV;


  WCPPID::ProtoSegment *prim_track1_sg;
  WCPPID::ProtoSegment *prim_track2_sg;
  WCPPID::ProtoSegment *daught_track1_sg;
  WCPPID::ProtoSegment *daught_track2_sg;
  WCPPID::ProtoSegment *prim_shw1_sg;
  WCPPID::ProtoSegment *prim_shw2_sg;
  WCPPID::ProtoSegment *daught_shw1_sg;
  WCPPID::ProtoSegment *daught_shw2_sg;

  TVector3 dir_prim_track1(0,0,0);
  TVector3 dir_prim_track2(0,0,0);
  TVector3 dir_daught_track1(0,0,0);
  TVector3 dir_daught_track2(0,0,0);
  TVector3 dir_prim_shw1(0,0,0);
  TVector3 dir_prim_shw2(0,0,0);
  TVector3 dir_daught_shw1(0,0,0);
  TVector3 dir_daught_shw2(0,0,0);

  TVector3 mom_prim_track1(0,0,0);
  TVector3 mom_prim_track2(0,0,0);
  TVector3 mom_daught_track1(0,0,0);
  TVector3 mom_daught_track2(0,0,0);
  TVector3 mom_prim_shw1(0,0,0);
  TVector3 mom_prim_shw2(0,0,0);
  TVector3 mom_daught_shw1(0,0,0);
  TVector3 mom_daught_shw2(0,0,0);

  //loop to fill the prim and daughter stuff
  for (auto it = map_vertex_segments[main_vertex].begin(); it!= map_vertex_segments[main_vertex].end(); it++){
    WCPPID::ProtoSegment *sg = *it;
    if(sg==ssm_sg) continue;
    double sg_length = sg->get_length()/units::cm;
    if(sg_length>1){
      n_prim_all_1 +=1;
      if (!sg->get_flag_shower()){
        n_prim_tracks_1 +=1;
      }
    }
    if(sg_length>3){
      n_prim_all_3 +=1;
      if (!sg->get_flag_shower()){
        n_prim_tracks_3 +=1;
      }
    }
    if(sg_length>5){
      n_prim_all_5 +=1;
      if (!sg->get_flag_shower()){
        n_prim_tracks_5 +=1;
      }
    }
    if(sg_length>8){
      n_prim_all_8 +=1;
      if (!sg->get_flag_shower()){
        n_prim_tracks_8 +=1;
      }
    }
    if(sg_length>11){
      n_prim_all_11 +=1;
      if (!sg->get_flag_shower()){
        n_prim_tracks_11 +=1;
      }
    }
    double sg_direct_length = sg->get_direct_length()/units::cm;
    double sg_pdg = abs(sg->get_particle_type()); 
    double sg_medium_dq_dx =  sg->get_medium_dQ_dx()/(43e3/units::cm);
    double sg_max_dev = sg->get_max_deviation(int(0),int(sg->get_point_vec().size()))/units::cm;
    double sg_kine_energy_cal = sg->cal_kine_dQdx();
    double sg_kine_energy_range = sg->cal_kine_range();
    std::vector<double> sg_kine_energy_range_pdg = calc_kine_range_multi_pdg(sg_length);
    TVector3 sg_dir = sg->cal_dir_3vector();
    //track at the origional vertex
    if (!sg->get_flag_shower()){
      if(sg_length<length_prim_track2) continue;
      int sg_add_daught_track_counts_1 = calculate_num_daughter_tracks(main_vertex, sg, false, 1*units::cm).first-1;
      if(sg_length<1) sg_add_daught_track_counts_1+=1;//didn't subtract, was too short
      if(sg_add_daught_track_counts_1<0) sg_add_daught_track_counts_1=0;
      int sg_add_daught_all_counts_1 = calculate_num_daughter_tracks(main_vertex, sg, true, 1*units::cm).first-1;
      if(sg_length<1) sg_add_daught_all_counts_1+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_1<0) sg_add_daught_all_counts_1=0;
      int sg_add_daught_track_counts_5 = calculate_num_daughter_tracks(main_vertex, sg, false, 5*units::cm).first-1;
      if(sg_length<5) sg_add_daught_track_counts_5+=1;//didn't subtract, was too short
      if(sg_add_daught_track_counts_5<0) sg_add_daught_track_counts_5=0;
      int sg_add_daught_all_counts_5 = calculate_num_daughter_tracks(main_vertex, sg, true, 5*units::cm).first-1;
      if(sg_length<5) sg_add_daught_all_counts_5+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_5<0) sg_add_daught_all_counts_5=0;
      int sg_add_daught_track_counts_11 = calculate_num_daughter_tracks(main_vertex, sg, false, 11*units::cm).first-1;
      if(sg_length<11) sg_add_daught_track_counts_11+=1;//didn't subtract, was too short
      if(sg_add_daught_track_counts_11<0) sg_add_daught_track_counts_11=0;
      int sg_add_daught_all_counts_11 = calculate_num_daughter_tracks(main_vertex, sg, true, 11*units::cm).first-1;
      if(sg_length<11) sg_add_daught_all_counts_11+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_11<0) sg_add_daught_all_counts_11=0;
      if(sg_length<length_prim_track1){//this is the second longest track
        length_prim_track2 = sg_length;
        direct_length_prim_track2 = sg_direct_length;
        pdg_prim_track2 = sg_pdg;
        medium_dq_dx_prim_track2 = sg_medium_dq_dx;
        max_dev_prim_track2 = sg_max_dev;
        prim_track2_sg = sg;
        kine_energy_cal_prim_track2 = sg_kine_energy_cal;
        kine_energy_range_prim_track2 = sg_kine_energy_range;
	kine_energy_range_mu_prim_track2 = sg_kine_energy_range_pdg.at(0);
        kine_energy_range_p_prim_track2 = sg_kine_energy_range_pdg.at(1);
        kine_energy_range_e_prim_track2 = sg_kine_energy_range_pdg.at(2);
        dir_prim_track2 = sg_dir;
	add_daught_track_counts_1_prim_track2 = sg_add_daught_track_counts_1;
	add_daught_all_counts_1_prim_track2 = sg_add_daught_all_counts_1;
        add_daught_track_counts_5_prim_track2 = sg_add_daught_track_counts_5;
        add_daught_all_counts_5_prim_track2 = sg_add_daught_all_counts_5;
        add_daught_track_counts_11_prim_track2 = sg_add_daught_track_counts_11;
        add_daught_all_counts_11_prim_track2 = sg_add_daught_all_counts_11;
      }
      else{//longest track
        length_prim_track2 = length_prim_track1;
        direct_length_prim_track2 = direct_length_prim_track1;
        pdg_prim_track2 = pdg_prim_track1;
        medium_dq_dx_prim_track2 = medium_dq_dx_prim_track1;
        max_dev_prim_track2 = max_dev_prim_track1;
        prim_track2_sg = prim_track1_sg;
        kine_energy_cal_prim_track2 = kine_energy_cal_prim_track1;
        kine_energy_range_prim_track2 = kine_energy_range_prim_track1;
	kine_energy_range_mu_prim_track2 = kine_energy_range_mu_prim_track1;
	kine_energy_range_p_prim_track2 = kine_energy_range_p_prim_track1;
	kine_energy_range_e_prim_track2 = kine_energy_range_e_prim_track1;
        dir_prim_track2 = dir_prim_track1;
        add_daught_track_counts_1_prim_track2 = add_daught_track_counts_1_prim_track1;
        add_daught_all_counts_1_prim_track2 = add_daught_all_counts_1_prim_track1;
        add_daught_track_counts_5_prim_track2 = add_daught_track_counts_5_prim_track1;
        add_daught_all_counts_5_prim_track2 = add_daught_all_counts_5_prim_track1;
        add_daught_track_counts_11_prim_track2 = add_daught_track_counts_11_prim_track1;
        add_daught_all_counts_11_prim_track2 = add_daught_all_counts_11_prim_track1;
	length_prim_track1 = sg_length;
        direct_length_prim_track1 = sg_direct_length;
        pdg_prim_track1 = sg_pdg;
        medium_dq_dx_prim_track1 = sg_medium_dq_dx;
        max_dev_prim_track1 = sg_max_dev;
        prim_track1_sg = sg;
        kine_energy_cal_prim_track1 = sg_kine_energy_cal;
        kine_energy_range_prim_track1 = sg_kine_energy_range;
        kine_energy_range_mu_prim_track1 = sg_kine_energy_range_pdg.at(0);
        kine_energy_range_p_prim_track1 = sg_kine_energy_range_pdg.at(1);
        kine_energy_range_e_prim_track1 = sg_kine_energy_range_pdg.at(2);
	dir_prim_track1 = sg_dir;
	add_daught_track_counts_1_prim_track1 = sg_add_daught_track_counts_1;
        add_daught_all_counts_1_prim_track1 = sg_add_daught_all_counts_1;
        add_daught_track_counts_5_prim_track1 = sg_add_daught_track_counts_5;
        add_daught_all_counts_5_prim_track1 = sg_add_daught_all_counts_5;
        add_daught_track_counts_11_prim_track1 = sg_add_daught_track_counts_11;
        add_daught_all_counts_11_prim_track1 = sg_add_daught_all_counts_11;
      }
    }
    else{//shower at the origional vertex
      if(sg_length<length_prim_shw2) continue;
      int sg_add_daught_track_counts_1 = calculate_num_daughter_tracks(main_vertex, sg, false, 1*units::cm).first;//not a track, don't subtract
      if(sg_add_daught_track_counts_1<0) sg_add_daught_track_counts_1=0;
      int sg_add_daught_all_counts_1 = calculate_num_daughter_tracks(main_vertex, sg, true, 1*units::cm).first-1;
      if(sg_length<1) sg_add_daught_all_counts_1+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_1<0) sg_add_daught_all_counts_1=0;
      int sg_add_daught_track_counts_5 = calculate_num_daughter_tracks(main_vertex, sg, false, 5*units::cm).first;//not a track, don't subtract
      if(sg_add_daught_track_counts_5<0) sg_add_daught_track_counts_5=0;
      int sg_add_daught_all_counts_5 = calculate_num_daughter_tracks(main_vertex, sg, true, 5*units::cm).first-1;
      if(sg_length<5) sg_add_daught_all_counts_5+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_5<0) sg_add_daught_all_counts_5=0;
      int sg_add_daught_track_counts_11 = calculate_num_daughter_tracks(main_vertex, sg, false, 11*units::cm).first;//not a track, don't subtract
      if(sg_add_daught_track_counts_11<0) sg_add_daught_track_counts_11=0;
      int sg_add_daught_all_counts_11 = calculate_num_daughter_tracks(main_vertex, sg, true, 11*units::cm).first-1;
      if(sg_length<11) sg_add_daught_all_counts_11+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_11<0) sg_add_daught_all_counts_11=0;
      if(sg_length<length_prim_shw1){//this is the second longest shw
        length_prim_shw2 = sg_length;
        direct_length_prim_shw2 = sg_direct_length;
        pdg_prim_shw2 = sg_pdg; 
	medium_dq_dx_prim_shw2 = sg_medium_dq_dx;
        max_dev_prim_shw2 = sg_max_dev;
        prim_shw2_sg = sg;
        kine_energy_cal_prim_shw2 = sg_kine_energy_cal;
        kine_energy_range_prim_shw2 = sg_kine_energy_range;
        kine_energy_range_mu_prim_shw2 = sg_kine_energy_range_pdg.at(0);
        kine_energy_range_p_prim_shw2 = sg_kine_energy_range_pdg.at(1);
        kine_energy_range_e_prim_shw2 = sg_kine_energy_range_pdg.at(2);
        kine_energy_best_prim_shw2 = kine_energy_range_prim_shw2;
        auto it_shower = map_segment_in_shower.find(sg);
        if (it_shower!=map_segment_in_shower.end()){
          WCPPID::WCShower *shower = it_shower->second;
          kine_energy_best_prim_shw2 = shower->get_kine_best();
          if (kine_energy_best_prim_shw2==0 ) {kine_energy_best_prim_shw2 = shower->get_kine_charge();}
        }
	dir_prim_shw2 = sg_dir;
	add_daught_track_counts_1_prim_shw2 = sg_add_daught_track_counts_1;
        add_daught_all_counts_1_prim_shw2 = sg_add_daught_all_counts_1;
        add_daught_track_counts_5_prim_shw2 = sg_add_daught_track_counts_5;
        add_daught_all_counts_5_prim_shw2 = sg_add_daught_all_counts_5;
        add_daught_track_counts_11_prim_shw2 = sg_add_daught_track_counts_11;
        add_daught_all_counts_11_prim_shw2 = sg_add_daught_all_counts_11;
      }
      else{//longest shw
        length_prim_shw2 = length_prim_shw1;
        direct_length_prim_shw2 = direct_length_prim_shw1;
        pdg_prim_shw2 = pdg_prim_shw1;
        medium_dq_dx_prim_shw2 = medium_dq_dx_prim_shw1;
        max_dev_prim_shw2 = max_dev_prim_shw1;
        prim_shw2_sg = prim_shw1_sg;
        kine_energy_cal_prim_shw2 = kine_energy_cal_prim_shw1;
        kine_energy_range_prim_shw2 = kine_energy_range_prim_shw1;
        kine_energy_range_mu_prim_shw2 = kine_energy_range_mu_prim_shw1;
        kine_energy_range_p_prim_shw2 = kine_energy_range_p_prim_shw1;
        kine_energy_range_e_prim_shw2 = kine_energy_range_e_prim_shw1;
        kine_energy_best_prim_shw2 = kine_energy_best_prim_shw1;
	dir_prim_shw2 = dir_prim_shw1;
	add_daught_track_counts_1_prim_shw2 = add_daught_track_counts_1_prim_shw1;
        add_daught_all_counts_1_prim_shw2 = add_daught_all_counts_1_prim_shw1;
        add_daught_track_counts_5_prim_shw2 = add_daught_track_counts_5_prim_shw1;
        add_daught_all_counts_5_prim_shw2 = add_daught_all_counts_5_prim_shw1;
        add_daught_track_counts_11_prim_shw2 = add_daught_track_counts_11_prim_shw1;
        add_daught_all_counts_11_prim_shw2 = add_daught_all_counts_11_prim_shw1;
        length_prim_shw1 = sg_length;
        direct_length_prim_shw1 = sg_direct_length;
        pdg_prim_shw1 = sg_pdg;
        medium_dq_dx_prim_shw1 = sg_medium_dq_dx;
        max_dev_prim_shw1 = sg_max_dev;
        prim_shw1_sg = sg;
        kine_energy_cal_prim_shw1 = sg_kine_energy_cal;
        kine_energy_range_prim_shw1 = sg_kine_energy_range;
        kine_energy_range_mu_prim_shw1 = sg_kine_energy_range_pdg.at(0);
        kine_energy_range_p_prim_shw1 = sg_kine_energy_range_pdg.at(1);
        kine_energy_range_e_prim_shw1 = sg_kine_energy_range_pdg.at(2);
        kine_energy_best_prim_shw1 = sg_kine_energy_range;
        auto it_shower = map_segment_in_shower.find(sg);
        if (it_shower!=map_segment_in_shower.end()){
          WCPPID::WCShower *shower = it_shower->second;
          kine_energy_best_prim_shw1 = shower->get_kine_best();
          if (kine_energy_best_prim_shw1==0 ){ kine_energy_best_prim_shw1 = shower->get_kine_charge();}
        }
	dir_prim_shw1 = sg_dir;
	add_daught_track_counts_1_prim_shw1 = sg_add_daught_track_counts_1;
        add_daught_all_counts_1_prim_shw1 = sg_add_daught_all_counts_1;
        add_daught_track_counts_5_prim_shw1 = sg_add_daught_track_counts_5;
        add_daught_all_counts_5_prim_shw1 = sg_add_daught_all_counts_5;
        add_daught_track_counts_11_prim_shw1 = sg_add_daught_track_counts_11;
        add_daught_all_counts_11_prim_shw1 = sg_add_daught_all_counts_11;
      }
    }
  }

  //loop the vertex corresponding to what was the origional muon butt vertex
  WCPPID::ProtoVertex* second_vtx = find_other_vertex(ssm_sg,main_vertex);
  for (auto it = map_vertex_segments[second_vtx].begin(); it!= map_vertex_segments[second_vtx].end(); it++){
    WCPPID::ProtoSegment *sg = *it;
    if(sg==ssm_sg) continue;
    double sg_length = sg->get_length()/units::cm;
    if(sg_length>1){
      n_daughter_all_1 +=1;
      if (!sg->get_flag_shower()){
        n_daughter_tracks_1 +=1;
      }
    }
    if(sg_length>3){
      n_daughter_all_3 +=1;
      if (!sg->get_flag_shower()){
        n_daughter_tracks_3 +=1;
      }
    }
    if(sg_length>5){
      n_daughter_all_5 +=1;
      if (!sg->get_flag_shower()){
        n_daughter_tracks_5 +=1;
      }
    }
    if(sg_length>8){
      n_daughter_all_8 +=1;
      if (!sg->get_flag_shower()){
        n_daughter_tracks_8 +=1;
      }
    }
    if(sg_length>11){
      n_daughter_all_11 +=1;
      if (!sg->get_flag_shower()){
        n_daughter_tracks_11 +=1;
      }
    }
    double sg_direct_length = sg->get_direct_length()/units::cm;
    double sg_pdg = abs(sg->get_particle_type());
    double sg_medium_dq_dx =  sg->get_medium_dQ_dx()/(43e3/units::cm);
    double sg_max_dev = sg->get_max_deviation(int(0),int(sg->get_point_vec().size()))/units::cm;
    double sg_kine_energy_cal = sg->cal_kine_dQdx();
    double sg_kine_energy_range = sg->cal_kine_range();
    std::vector<double> sg_kine_energy_range_pdg = calc_kine_range_multi_pdg(sg_length);
    TVector3 sg_dir = sg->cal_dir_3vector();
    //track at the muon butt vertex
    if (!sg->get_flag_shower()){
      if(sg_length<length_daught_track2) continue;
      int sg_add_daught_track_counts_1 = calculate_num_daughter_tracks(second_vtx, sg, false, 1*units::cm).first-1;
      if(sg_length<1) sg_add_daught_track_counts_1+=1;//didn't subtract, was too short
      if(sg_add_daught_track_counts_1<0) sg_add_daught_track_counts_1=0;
      int sg_add_daught_all_counts_1 = calculate_num_daughter_tracks(second_vtx, sg, true, 1*units::cm).first-1;
      if(sg_length<1) sg_add_daught_all_counts_1+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_1<0) sg_add_daught_all_counts_1=0;
      int sg_add_daught_track_counts_5 = calculate_num_daughter_tracks(second_vtx, sg, false, 5*units::cm).first-1;
      if(sg_length<5) sg_add_daught_track_counts_5+=1;//didn't subtract, was too short
      if(sg_add_daught_track_counts_5<0) sg_add_daught_track_counts_5=0;
      int sg_add_daught_all_counts_5 = calculate_num_daughter_tracks(second_vtx, sg, true, 5*units::cm).first-1;
      if(sg_length<5) sg_add_daught_all_counts_5+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_5<0) sg_add_daught_all_counts_5=0;
      int sg_add_daught_track_counts_11 = calculate_num_daughter_tracks(second_vtx, sg, false, 11*units::cm).first-1;
      if(sg_length<11) sg_add_daught_track_counts_11+=1;//didn't subtract, was too short
      if(sg_add_daught_track_counts_11<0) sg_add_daught_track_counts_11=0;
      int sg_add_daught_all_counts_11 = calculate_num_daughter_tracks(second_vtx, sg, true, 11*units::cm).first-1;
      if(sg_length<11) sg_add_daught_all_counts_11+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_11<0) sg_add_daught_all_counts_11=0;
      if(sg_length<length_daught_track1){//this is the second longest track
        length_daught_track2 = sg_length;
        direct_length_daught_track2 = sg_direct_length;
        pdg_daught_track2 = sg_pdg;
        medium_dq_dx_daught_track2 = sg_medium_dq_dx;
        max_dev_daught_track2 = sg_max_dev;
        daught_track2_sg = sg;
        kine_energy_cal_daught_track2 = sg_kine_energy_cal;
	kine_energy_range_daught_track2 = sg_kine_energy_range;
	kine_energy_range_mu_daught_track2 = sg_kine_energy_range_pdg.at(0);
	kine_energy_range_p_daught_track2 = sg_kine_energy_range_pdg.at(1);
	kine_energy_range_e_daught_track2 = sg_kine_energy_range_pdg.at(2);
	dir_daught_track2 = sg_dir;
	add_daught_track_counts_1_daught_track2 = sg_add_daught_track_counts_1;
        add_daught_all_counts_1_daught_track2 = sg_add_daught_all_counts_1;
        add_daught_track_counts_5_daught_track2 = sg_add_daught_track_counts_5;
        add_daught_all_counts_5_daught_track2 = sg_add_daught_all_counts_5;
        add_daught_track_counts_11_daught_track2 = sg_add_daught_track_counts_11;
        add_daught_all_counts_11_daught_track2 = sg_add_daught_all_counts_11;
      }
      else{//longest track
        length_daught_track2 = length_daught_track1;
        direct_length_daught_track2 = direct_length_daught_track1;
        pdg_daught_track2 = pdg_daught_track1;
        medium_dq_dx_daught_track2 = medium_dq_dx_daught_track1;
        max_dev_daught_track2 = max_dev_daught_track1;
        daught_track2_sg = daught_track1_sg;
	kine_energy_cal_daught_track2 = kine_energy_cal_daught_track1;
	kine_energy_range_daught_track2 = kine_energy_range_daught_track1;
        kine_energy_range_mu_daught_track2 = kine_energy_range_mu_daught_track1;
        kine_energy_range_p_daught_track2 = kine_energy_range_p_daught_track1;
        kine_energy_range_e_daught_track2 = kine_energy_range_e_daught_track1;
	dir_daught_track2 = dir_daught_track1;
	add_daught_track_counts_1_daught_track2 = add_daught_track_counts_1_daught_track1;
        add_daught_all_counts_1_daught_track2 = add_daught_all_counts_1_daught_track1;
        add_daught_track_counts_5_daught_track2 = add_daught_track_counts_5_daught_track1;
        add_daught_all_counts_5_daught_track2 = add_daught_all_counts_5_daught_track1;
        add_daught_track_counts_11_daught_track2 = add_daught_track_counts_11_daught_track1;
        add_daught_all_counts_11_daught_track2 = add_daught_all_counts_11_daught_track1;
        length_daught_track1 = sg_length;
        direct_length_daught_track1 = sg_direct_length;
        pdg_daught_track1 = sg_pdg;
        medium_dq_dx_daught_track1 = sg_medium_dq_dx;
        max_dev_daught_track1 = sg_max_dev;  
        daught_track1_sg = sg;
	kine_energy_cal_daught_track1 = sg_kine_energy_cal;
	kine_energy_range_daught_track1 = sg_kine_energy_range;
        kine_energy_range_mu_daught_track1 = sg_kine_energy_range_pdg.at(0);
        kine_energy_range_p_daught_track1 = sg_kine_energy_range_pdg.at(1);
        kine_energy_range_e_daught_track1 = sg_kine_energy_range_pdg.at(2);
	dir_daught_track1 = sg_dir;
	add_daught_track_counts_1_daught_track1 = sg_add_daught_track_counts_1;
        add_daught_all_counts_1_daught_track1 = sg_add_daught_all_counts_1;
        add_daught_track_counts_5_daught_track1 = sg_add_daught_track_counts_5;
        add_daught_all_counts_5_daught_track1 = sg_add_daught_all_counts_5;
        add_daught_track_counts_11_daught_track1 = sg_add_daught_track_counts_11;
        add_daught_all_counts_11_daught_track1 = sg_add_daught_all_counts_11;
      }
    }
    else{//shower at the muon butt vertex
      if(sg_length<length_daught_shw2) continue;
      int sg_add_daught_track_counts_1 = calculate_num_daughter_tracks(second_vtx, sg, false, 1*units::cm).first;//not a track, don't subtract
      if(sg_add_daught_track_counts_1<0) sg_add_daught_track_counts_1=0;
      int sg_add_daught_all_counts_1 = calculate_num_daughter_tracks(second_vtx, sg, true, 1*units::cm).first-1;
      if(sg_length<1) sg_add_daught_all_counts_1+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_1<0) sg_add_daught_all_counts_1=0;
      int sg_add_daught_track_counts_5 = calculate_num_daughter_tracks(second_vtx, sg, false, 5*units::cm).first;//not a track, don't subtract
      if(sg_add_daught_track_counts_5<0) sg_add_daught_track_counts_5=0;
      int sg_add_daught_all_counts_5 = calculate_num_daughter_tracks(second_vtx, sg, true, 5*units::cm).first-1;
      if(sg_length<5) sg_add_daught_all_counts_5+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_5<0) sg_add_daught_all_counts_5=0;
      int sg_add_daught_track_counts_11 = calculate_num_daughter_tracks(second_vtx, sg, false, 11*units::cm).first;//not a track, don't subtract
      if(sg_add_daught_track_counts_11<0) sg_add_daught_track_counts_11=0;
      int sg_add_daught_all_counts_11 = calculate_num_daughter_tracks(second_vtx, sg, true, 11*units::cm).first-1;
      if(sg_length<11) sg_add_daught_all_counts_11+=1;//didn't subtract, was too short
      if(sg_add_daught_all_counts_11<0) sg_add_daught_all_counts_11=0;
      if(sg_length<length_daught_shw1){//this is the second longest shw
        length_daught_shw2 = sg_length;
        direct_length_daught_shw2 = sg_direct_length;
        pdg_daught_shw2 = sg_pdg;
        medium_dq_dx_daught_shw2 = sg_medium_dq_dx;
        max_dev_daught_shw2 = sg_max_dev;
        daught_shw2_sg = sg;
        kine_energy_cal_daught_shw2 = sg_kine_energy_cal;
        kine_energy_range_daught_shw2 = sg_kine_energy_range;
	kine_energy_range_mu_daught_shw2 = sg_kine_energy_range_pdg.at(0);
        kine_energy_range_p_daught_shw2 = sg_kine_energy_range_pdg.at(1);
        kine_energy_range_e_daught_shw2 = sg_kine_energy_range_pdg.at(2);
        kine_energy_best_daught_shw2 = sg_kine_energy_range;
        auto it_shower = map_segment_in_shower.find(sg);
        if (it_shower!=map_segment_in_shower.end()){
          WCPPID::WCShower *shower = it_shower->second;
          kine_energy_best_daught_shw2 = shower->get_kine_best();
          if (kine_energy_best_daught_shw2==0 ) kine_energy_best_daught_shw2 = shower->get_kine_charge();
        }
        dir_daught_shw2 = sg_dir;
	add_daught_track_counts_1_daught_shw2 = sg_add_daught_track_counts_1;
        add_daught_all_counts_1_daught_shw2 = sg_add_daught_all_counts_1;
        add_daught_track_counts_5_daught_shw2 = sg_add_daught_track_counts_5;
        add_daught_all_counts_5_daught_shw2 = sg_add_daught_all_counts_5;
        add_daught_track_counts_11_daught_shw2 = sg_add_daught_track_counts_11;
        add_daught_all_counts_11_daught_shw2 = sg_add_daught_all_counts_11;
      }
      else{//longest shw
        length_daught_shw2 = length_daught_shw1;
        direct_length_daught_shw2 = direct_length_daught_shw1;
        pdg_daught_shw2 = pdg_daught_shw1;
        medium_dq_dx_daught_shw2 = medium_dq_dx_daught_shw1;
        max_dev_daught_shw2 = max_dev_daught_shw1;
        daught_shw2_sg = daught_shw1_sg;
        kine_energy_cal_daught_shw2 = kine_energy_cal_daught_shw1;
        kine_energy_range_daught_shw2 = kine_energy_range_daught_shw1;
	kine_energy_range_mu_daught_shw2 = kine_energy_range_mu_daught_shw1;
        kine_energy_range_p_daught_shw2 = kine_energy_range_p_daught_shw1;
        kine_energy_range_e_daught_shw2 = kine_energy_range_e_daught_shw1;
        kine_energy_best_daught_shw2 = kine_energy_best_daught_shw1;
        dir_daught_shw2 = dir_daught_shw1;
	add_daught_track_counts_1_daught_shw2 = add_daught_track_counts_1_daught_shw1;
        add_daught_all_counts_1_daught_shw2 = add_daught_all_counts_1_daught_shw1;
        add_daught_track_counts_5_daught_shw2 = add_daught_track_counts_5_daught_shw1;
        add_daught_all_counts_5_daught_shw2 = add_daught_all_counts_5_daught_shw1;
        add_daught_track_counts_11_daught_shw2 = add_daught_track_counts_11_daught_shw1;
        add_daught_all_counts_11_daught_shw2 = add_daught_all_counts_11_daught_shw1;
        length_daught_shw1 = sg_length;
        direct_length_daught_shw1 = sg_direct_length;
        pdg_daught_shw1 = sg_pdg;
        medium_dq_dx_daught_shw1 = sg_medium_dq_dx;
        max_dev_daught_shw1 = sg_max_dev;        
        daught_shw1_sg = sg;
        kine_energy_cal_daught_shw1 = sg_kine_energy_cal;
        kine_energy_range_daught_shw1 = sg_kine_energy_range;
        kine_energy_range_mu_daught_shw1 = sg_kine_energy_range_pdg.at(0);
        kine_energy_range_p_daught_shw1 = sg_kine_energy_range_pdg.at(1);
        kine_energy_range_e_daught_shw1 = sg_kine_energy_range_pdg.at(2);
        kine_energy_best_daught_shw1 = sg_kine_energy_range;
        auto it_shower = map_segment_in_shower.find(sg);
        if (it_shower!=map_segment_in_shower.end()){
          WCPPID::WCShower *shower = it_shower->second;
          kine_energy_best_daught_shw1 = shower->get_kine_best();
          if (kine_energy_best_daught_shw1==0 ) kine_energy_best_daught_shw1 = shower->get_kine_charge();
        }
	dir_daught_shw1 = sg_dir;
	add_daught_track_counts_1_daught_shw1 = sg_add_daught_track_counts_1;
        add_daught_all_counts_1_daught_shw1 = sg_add_daught_all_counts_1;
        add_daught_track_counts_5_daught_shw1 = sg_add_daught_track_counts_5;
        add_daught_all_counts_5_daught_shw1 = sg_add_daught_all_counts_5;
        add_daught_track_counts_11_daught_shw1 = sg_add_daught_track_counts_11;
        add_daught_all_counts_11_daught_shw1 = sg_add_daught_all_counts_11;
      }
    }
  }

  //need a check on direction of the other particles?
  if(length_prim_track1>0){
    std::vector<double> scores_prim_track1 = get_scores(prim_track1_sg);
    score_mu_fwd_prim_track1 = scores_prim_track1.at(0);
    score_p_fwd_prim_track1 = scores_prim_track1.at(1);
    score_e_fwd_prim_track1 = scores_prim_track1.at(2);
    score_mu_bck_prim_track1 = scores_prim_track1.at(3);
    score_p_bck_prim_track1 = scores_prim_track1.at(4);
    score_e_bck_prim_track1 = scores_prim_track1.at(5);
    if(prim_track1_sg->get_flag_dir()==-1){
      std::swap(score_mu_fwd_prim_track1,score_mu_bck_prim_track1);
      std::swap(score_p_fwd_prim_track1,score_p_bck_prim_track1);
      std::swap(score_e_fwd_prim_track1,score_e_bck_prim_track1);
    }
  }
  if(length_prim_track2>0){
    std::vector<double> scores_prim_track2 = get_scores(prim_track2_sg);
    score_mu_fwd_prim_track2 = scores_prim_track2.at(0);
    score_p_fwd_prim_track2 = scores_prim_track2.at(1);
    score_e_fwd_prim_track2 = scores_prim_track2.at(2);
    score_mu_bck_prim_track2 = scores_prim_track2.at(3);
    score_p_bck_prim_track2 = scores_prim_track2.at(4);
    score_e_bck_prim_track2 = scores_prim_track2.at(5);
    if(prim_track2_sg->get_flag_dir()==-1){
      std::swap(score_mu_fwd_prim_track2,score_mu_bck_prim_track2);
      std::swap(score_p_fwd_prim_track2,score_p_bck_prim_track2);
      std::swap(score_e_fwd_prim_track2,score_e_bck_prim_track2);
    }
  }
  if(length_daught_track1>0){
    std::vector<double> scores_daught_track1 = get_scores(daught_track1_sg);
    score_mu_fwd_daught_track1 = scores_daught_track1.at(0);
    score_p_fwd_daught_track1 = scores_daught_track1.at(1);
    score_e_fwd_daught_track1 = scores_daught_track1.at(2);
    score_mu_bck_daught_track1 = scores_daught_track1.at(3);
    score_p_bck_daught_track1 = scores_daught_track1.at(4);
    score_e_bck_daught_track1 = scores_daught_track1.at(5);
    if(daught_track1_sg->get_flag_dir()==-1){
      std::swap(score_mu_fwd_daught_track1,score_mu_bck_daught_track1);
      std::swap(score_p_fwd_daught_track1,score_p_bck_daught_track1);
      std::swap(score_e_fwd_daught_track1,score_e_bck_daught_track1);
    }
  }
  if(length_daught_track2>0){
    std::vector<double> scores_daught_track2 = get_scores(daught_track2_sg);
    score_mu_fwd_daught_track2 = scores_daught_track2.at(0);
    score_p_fwd_daught_track2 = scores_daught_track2.at(1);
    score_e_fwd_daught_track2 = scores_daught_track2.at(2);
    score_mu_bck_daught_track2 = scores_daught_track2.at(3);
    score_p_bck_daught_track2 = scores_daught_track2.at(4);
    score_e_bck_daught_track2 = scores_daught_track2.at(5);
    if(daught_track2_sg->get_flag_dir()==-1){
      std::swap(score_mu_fwd_daught_track2,score_mu_bck_daught_track2);
      std::swap(score_p_fwd_daught_track2,score_p_bck_daught_track2);
      std::swap(score_e_fwd_daught_track2,score_e_bck_daught_track2);
    }
  }
  if(length_prim_shw1>0){
    std::vector<double> scores_prim_shw1 = get_scores(prim_shw1_sg);
    score_mu_fwd_prim_shw1 = scores_prim_shw1.at(0);
    score_p_fwd_prim_shw1 = scores_prim_shw1.at(1);
    score_e_fwd_prim_shw1 = scores_prim_shw1.at(2);
    score_mu_bck_prim_shw1 = scores_prim_shw1.at(3);
    score_p_bck_prim_shw1 = scores_prim_shw1.at(4);
    score_e_bck_prim_shw1 = scores_prim_shw1.at(5);
    if(prim_shw1_sg->get_flag_dir()==-1){
      std::swap(score_mu_fwd_prim_shw1,score_mu_bck_prim_shw1);
      std::swap(score_p_fwd_prim_shw1,score_p_bck_prim_shw1);
      std::swap(score_e_fwd_prim_shw1,score_e_bck_prim_shw1);
    }
  }
  if(length_prim_shw2>0){
    std::vector<double> scores_prim_shw2 = get_scores(prim_shw2_sg);
    score_mu_fwd_prim_shw2 = scores_prim_shw2.at(0);
    score_p_fwd_prim_shw2 = scores_prim_shw2.at(1);
    score_e_fwd_prim_shw2 = scores_prim_shw2.at(2);
    score_mu_bck_prim_shw2 = scores_prim_shw2.at(3);
    score_p_bck_prim_shw2 = scores_prim_shw2.at(4);
    score_e_bck_prim_shw2 = scores_prim_shw2.at(5);
    if(prim_shw2_sg->get_flag_dir()==-1){
      std::swap(score_mu_fwd_prim_shw2,score_mu_bck_prim_shw2);
      std::swap(score_p_fwd_prim_shw2,score_p_bck_prim_shw2);
      std::swap(score_e_fwd_prim_shw2,score_e_bck_prim_shw2);
    }
  }
  if(length_daught_shw1>0){
    std::vector<double> scores_daught_shw1 = get_scores(daught_shw1_sg);
    score_mu_fwd_daught_shw1 = scores_daught_shw1.at(0);
    score_p_fwd_daught_shw1 = scores_daught_shw1.at(1);
    score_e_fwd_daught_shw1 = scores_daught_shw1.at(2);
    score_mu_bck_daught_shw1 = scores_daught_shw1.at(3);
    score_p_bck_daught_shw1 = scores_daught_shw1.at(4);
    score_e_bck_daught_shw1 = scores_daught_shw1.at(5);
    if(daught_shw1_sg->get_flag_dir()==-1){
      std::swap(score_mu_fwd_daught_shw1,score_mu_bck_daught_shw1);
      std::swap(score_p_fwd_daught_shw1,score_p_bck_daught_shw1);
      std::swap(score_e_fwd_daught_shw1,score_e_bck_daught_shw1);
    }
  }
  if(length_daught_shw2>0){
    std::vector<double> scores_daught_shw2 = get_scores(daught_shw2_sg);
    score_mu_fwd_daught_shw2 = scores_daught_shw2.at(0);
    score_p_fwd_daught_shw2 = scores_daught_shw2.at(1);
    score_e_fwd_daught_shw2 = scores_daught_shw2.at(2);
    score_mu_bck_daught_shw2 = scores_daught_shw2.at(3);
    score_p_bck_daught_shw2 = scores_daught_shw2.at(4);
    score_e_bck_daught_shw2 = scores_daught_shw2.at(5);
    if(daught_shw2_sg->get_flag_dir()==-1){
      std::swap(score_mu_fwd_daught_shw2,score_mu_bck_daught_shw2);
      std::swap(score_p_fwd_daught_shw2,score_p_bck_daught_shw2);
      std::swap(score_e_fwd_daught_shw2,score_e_bck_daught_shw2);
    }
  }

    //figure our condition needed to swap, for now just daught and prim if muon is backwards
    //will be more complex if we look for off vertex stuff, here fwd/bckg may also switch if the muon in non-prim
  if(backwards_muon){

    std::swap(n_prim_all_1, n_daughter_all_1);
    std::swap(n_prim_tracks_1, n_daughter_tracks_1);
    std::swap(n_prim_all_3, n_daughter_all_3);
    std::swap(n_prim_tracks_3, n_daughter_tracks_3);
    std::swap(n_prim_all_5, n_daughter_all_5);
    std::swap(n_prim_tracks_5, n_daughter_tracks_5);
    std::swap(n_prim_all_8, n_daughter_all_8);
    std::swap(n_prim_tracks_8, n_daughter_tracks_8);
    std::swap(n_prim_all_11, n_daughter_all_11);
    std::swap(n_prim_tracks_11, n_daughter_tracks_11);

    std::swap(add_daught_track_counts_1_prim_track1, add_daught_track_counts_1_daught_track1);
    std::swap(add_daught_all_counts_1_prim_track1, add_daught_all_counts_1_daught_track1);
    std::swap(add_daught_track_counts_1_prim_track2, add_daught_track_counts_1_daught_track2);
    std::swap(add_daught_all_counts_1_prim_track2, add_daught_all_counts_1_daught_track2);
    std::swap(add_daught_track_counts_1_prim_shw1, add_daught_track_counts_1_daught_shw1);
    std::swap(add_daught_all_counts_1_prim_shw1, add_daught_all_counts_1_daught_shw1);
    std::swap(add_daught_track_counts_1_prim_shw2, add_daught_track_counts_1_daught_shw2);
    std::swap(add_daught_all_counts_1_prim_shw2, add_daught_all_counts_1_daught_shw2);

    std::swap(add_daught_track_counts_5_prim_track1, add_daught_track_counts_5_daught_track1);
    std::swap(add_daught_all_counts_5_prim_track1, add_daught_all_counts_5_daught_track1);
    std::swap(add_daught_track_counts_5_prim_track2, add_daught_track_counts_5_daught_track2);
    std::swap(add_daught_all_counts_5_prim_track2, add_daught_all_counts_5_daught_track2);
    std::swap(add_daught_track_counts_5_prim_shw1, add_daught_track_counts_5_daught_shw1);
    std::swap(add_daught_all_counts_5_prim_shw1, add_daught_all_counts_5_daught_shw1);
    std::swap(add_daught_track_counts_5_prim_shw2, add_daught_track_counts_5_daught_shw2);
    std::swap(add_daught_all_counts_5_prim_shw2, add_daught_all_counts_5_daught_shw2);

    std::swap(add_daught_track_counts_11_prim_track1, add_daught_track_counts_11_daught_track1);
    std::swap(add_daught_all_counts_11_prim_track1, add_daught_all_counts_11_daught_track1);
    std::swap(add_daught_track_counts_11_prim_track2, add_daught_track_counts_11_daught_track2);
    std::swap(add_daught_all_counts_11_prim_track2, add_daught_all_counts_11_daught_track2);
    std::swap(add_daught_track_counts_11_prim_shw1, add_daught_track_counts_11_daught_shw1);
    std::swap(add_daught_all_counts_11_prim_shw1, add_daught_all_counts_11_daught_shw1);
    std::swap(add_daught_track_counts_11_prim_shw2, add_daught_track_counts_11_daught_shw2);
    std::swap(add_daught_all_counts_11_prim_shw2, add_daught_all_counts_11_daught_shw2);

    std::swap(pdg_prim_track1, pdg_daught_track1);
    std::swap(length_prim_track1, length_daught_track1);
    std::swap(direct_length_prim_track1, direct_length_daught_track1);
    std::swap(max_dev_prim_track1, max_dev_daught_track1);
    std::swap(kine_energy_range_prim_track1, kine_energy_range_daught_track1);
    std::swap(kine_energy_range_mu_prim_track1, kine_energy_range_mu_daught_track1);
    std::swap(kine_energy_range_p_prim_track1, kine_energy_range_p_daught_track1);
    std::swap(kine_energy_range_e_prim_track1, kine_energy_range_e_daught_track1);
    std::swap(kine_energy_cal_prim_track1, kine_energy_cal_daught_track1);
    std::swap(medium_dq_dx_prim_track1, medium_dq_dx_daught_track1);
    std::swap(dir_prim_track1, dir_daught_track1);

    std::swap(pdg_prim_track2, pdg_daught_track2);
    std::swap(length_prim_track2, length_daught_track2);
    std::swap(direct_length_prim_track2, direct_length_daught_track2);
    std::swap(max_dev_prim_track2, max_dev_daught_track2);
    std::swap(kine_energy_range_prim_track2, kine_energy_range_daught_track2);
    std::swap(kine_energy_range_mu_prim_track2, kine_energy_range_mu_daught_track2);
    std::swap(kine_energy_range_p_prim_track2, kine_energy_range_p_daught_track2);
    std::swap(kine_energy_range_e_prim_track2, kine_energy_range_e_daught_track2);
    std::swap(kine_energy_cal_prim_track2, kine_energy_cal_daught_track2);
    std::swap(medium_dq_dx_prim_track2, medium_dq_dx_daught_track2);
    std::swap(dir_prim_track2, dir_daught_track2);

    std::swap(pdg_prim_shw1, pdg_daught_shw1);
    std::swap(length_prim_shw1, length_daught_shw1);
    std::swap(direct_length_prim_shw1, direct_length_daught_shw1);
    std::swap(max_dev_prim_shw1, max_dev_daught_shw1);
    std::swap(kine_energy_range_prim_shw1, kine_energy_range_daught_shw1);
    std::swap(kine_energy_range_mu_prim_shw1, kine_energy_range_mu_daught_shw1);
    std::swap(kine_energy_range_p_prim_shw1, kine_energy_range_p_daught_shw1);
    std::swap(kine_energy_range_e_prim_shw1, kine_energy_range_e_daught_shw1);
    std::swap(kine_energy_cal_prim_shw1, kine_energy_cal_daught_shw1);
    std::swap(kine_energy_best_prim_shw1, kine_energy_best_daught_shw1);
    std::swap(medium_dq_dx_prim_shw1, medium_dq_dx_daught_shw1);
    std::swap(dir_prim_shw1, dir_daught_shw1);

    std::swap(pdg_prim_shw2, pdg_daught_shw2);
    std::swap(length_prim_shw2, length_daught_shw2);
    std::swap(direct_length_prim_shw2, direct_length_daught_shw2);
    std::swap(max_dev_prim_shw2, max_dev_daught_shw2);
    std::swap(kine_energy_range_prim_shw2, kine_energy_range_daught_shw2);
    std::swap(kine_energy_range_mu_prim_shw2, kine_energy_range_mu_daught_shw2);
    std::swap(kine_energy_range_p_prim_shw2, kine_energy_range_p_daught_shw2);
    std::swap(kine_energy_range_e_prim_shw2, kine_energy_range_e_daught_shw2);
    std::swap(kine_energy_cal_prim_shw2, kine_energy_cal_daught_shw2);
    std::swap(kine_energy_best_prim_shw2, kine_energy_best_daught_shw2);
    std::swap(medium_dq_dx_prim_shw2, medium_dq_dx_daught_shw2);
    std::swap(dir_prim_shw2, dir_daught_shw2);

    std::swap(score_mu_fwd_prim_track1, score_mu_fwd_daught_track1);  
    std::swap(score_p_fwd_prim_track1, score_p_fwd_daught_track1); 
    std::swap(score_e_fwd_prim_track1, score_e_fwd_daught_track1);
    std::swap(score_mu_bck_prim_track1, score_mu_bck_daught_track1); 
    std::swap(score_p_bck_prim_track1, score_p_bck_daught_track1);
    std::swap(score_e_bck_prim_track1, score_e_bck_daught_track1);

    std::swap(score_mu_fwd_prim_track2, score_mu_fwd_daught_track2); 
    std::swap(score_p_fwd_prim_track2, score_p_fwd_daught_track2); 
    std::swap(score_e_fwd_prim_track2, score_e_fwd_daught_track2);
    std::swap(score_mu_bck_prim_track2, score_mu_bck_daught_track2); 
    std::swap(score_p_bck_prim_track2, score_p_bck_daught_track2);
    std::swap(score_e_bck_prim_track2, score_e_bck_daught_track2);

    std::swap(score_mu_fwd_prim_shw1, score_mu_fwd_daught_shw1);
    std::swap(score_p_fwd_prim_shw1, score_p_fwd_daught_shw1);
    std::swap(score_e_fwd_prim_shw1, score_e_fwd_daught_shw1);
    std::swap(score_mu_bck_prim_shw1, score_mu_bck_daught_shw1);
    std::swap(score_p_bck_prim_shw1, score_p_bck_daught_shw1); 
    std::swap(score_e_bck_prim_shw1, score_e_bck_daught_shw1);

    std::swap(score_mu_fwd_prim_shw2, score_mu_fwd_daught_shw2);
    std::swap(score_p_fwd_prim_shw2, score_p_fwd_daught_shw2);
    std::swap(score_e_fwd_prim_shw2, score_e_fwd_daught_shw2);
    std::swap(score_mu_bck_prim_shw2, score_mu_bck_daught_shw2);
    std::swap(score_p_bck_prim_shw2, score_p_bck_daught_shw2);
    std::swap(score_e_bck_prim_shw2, score_e_bck_daught_shw2);

    std::swap(prim_track1_sg,daught_track1_sg);
    std::swap(prim_track2_sg,daught_track2_sg);
    std::swap(prim_shw1_sg,daught_shw1_sg);
    std::swap(prim_shw2_sg,daught_shw2_sg);
  }

  if(length_prim_track1>0){
    double mass_prim_track1 = prim_track1_sg->get_particle_mass();
    if(mass_prim_track1<0){mass_prim_track1=0;}
    double mom_mag_prim_track1 = sqrt(pow(kine_energy_range_prim_track1,2) + 2*kine_energy_range_prim_track1*mass_prim_track1);
    x_dir_prim_track1 = dir_prim_track1[0];
    y_dir_prim_track1 = dir_prim_track1[1];
    z_dir_prim_track1 = dir_prim_track1[2];
    mom_prim_track1[0] = dir_prim_track1[0]*mom_mag_prim_track1;
    mom_prim_track1[1] = dir_prim_track1[1]*mom_mag_prim_track1;
    mom_prim_track1[2] = dir_prim_track1[2]*mom_mag_prim_track1;
  }
  if(length_prim_track2>0){
    double mass_prim_track2 = prim_track2_sg->get_particle_mass();
    if(mass_prim_track2<0){mass_prim_track2=0;}
    double mom_mag_prim_track2 = sqrt(pow(kine_energy_range_prim_track2,2) + 2*kine_energy_range_prim_track2*mass_prim_track2);
    x_dir_prim_track2 = dir_prim_track2[0];
    y_dir_prim_track2 = dir_prim_track2[1];
    z_dir_prim_track2 = dir_prim_track2[2];
    mom_prim_track2[0] = dir_prim_track2[0]*mom_mag_prim_track2;
    mom_prim_track2[1] = dir_prim_track2[1]*mom_mag_prim_track2;
    mom_prim_track2[2] = dir_prim_track2[2]*mom_mag_prim_track2;
  }
  if(length_daught_track1>0){
    double mass_daught_track1 = daught_track1_sg->get_particle_mass();
    if(mass_daught_track1<0){mass_daught_track1=0;}
    double mom_mag_daught_track1 = sqrt(pow(kine_energy_range_daught_track1,2) + 2*kine_energy_range_daught_track1*mass_daught_track1);
    x_dir_daught_track1 = dir_prim_track1[0];
    y_dir_daught_track1 = dir_prim_track1[1];
    z_dir_daught_track1 = dir_prim_track1[2];
    mom_daught_track1[0] = dir_daught_track1[0]*mom_mag_daught_track1;
    mom_daught_track1[1] = dir_daught_track1[1]*mom_mag_daught_track1;
    mom_daught_track1[2] = dir_daught_track1[2]*mom_mag_daught_track1;
  }
  if(length_daught_track2>0){
    double mass_daught_track2 = daught_track2_sg->get_particle_mass();
    if(mass_daught_track2<0){mass_daught_track2=0;}
    double mom_mag_daught_track2 = sqrt(pow(kine_energy_range_daught_track2,2) + 2*kine_energy_range_daught_track2*mass_daught_track2);
    x_dir_daught_track2 = dir_daught_track2[0];
    y_dir_daught_track2 = dir_daught_track2[1];
    z_dir_daught_track2 = dir_daught_track2[2];
    mom_daught_track2[0] = dir_daught_track2[0]*mom_mag_daught_track2;
    mom_daught_track2[1] = dir_daught_track2[1]*mom_mag_daught_track2;
    mom_daught_track2[2] = dir_daught_track2[2]*mom_mag_daught_track2;
  }
  if(length_prim_shw1>0){
    double mass_prim_shw1 = prim_shw1_sg->get_particle_mass();
    if(mass_prim_shw1<0){mass_prim_shw1=0;}
    double mom_mag_prim_shw1 = sqrt(pow(kine_energy_range_prim_shw1,2) + 2*kine_energy_range_prim_shw1*mass_prim_shw1);
    x_dir_prim_shw1 = dir_prim_shw1[0];
    y_dir_prim_shw1 = dir_prim_shw1[1];
    z_dir_prim_shw1 = dir_prim_shw1[2];
    mom_prim_shw1[0] = dir_prim_shw1[0]*mom_mag_prim_shw1;
    mom_prim_shw1[1] = dir_prim_shw1[1]*mom_mag_prim_shw1;
    mom_prim_shw1[2] = dir_prim_shw1[2]*mom_mag_prim_shw1;
  }
  if(length_prim_shw2>0){
    double mass_prim_shw2 = prim_shw2_sg->get_particle_mass();
    if(mass_prim_shw2<0){mass_prim_shw2=0;}
    double mom_mag_prim_shw2 = sqrt(pow(kine_energy_range_prim_shw2,2) + 2*kine_energy_range_prim_shw2*mass_prim_shw2);
    x_dir_prim_shw2 = dir_prim_shw2[0];
    y_dir_prim_shw2 = dir_prim_shw2[1];
    z_dir_prim_shw2 = dir_prim_shw2[2];
    mom_prim_shw2[0] = dir_prim_shw2[0]*mom_mag_prim_shw2;
    mom_prim_shw2[1] = dir_prim_shw2[1]*mom_mag_prim_shw2;
    mom_prim_shw2[2] = dir_prim_shw2[2]*mom_mag_prim_shw2;
  }
  //daugher showers we will use best instead of range
  if(length_daught_shw1>0){
    double mass_daught_shw1 = daught_shw1_sg->get_particle_mass();
    if(mass_daught_shw1<0){mass_daught_shw1=0;}
    double mom_mag_daught_shw1 = sqrt(pow(kine_energy_best_daught_shw1,2) + 2*kine_energy_best_daught_shw1*mass_daught_shw1);
    x_dir_daught_shw1 = dir_daught_shw1[0];
    y_dir_daught_shw1 = dir_daught_shw1[1];
    z_dir_daught_shw1 = dir_daught_shw1[2];
    mom_daught_shw1[0] = dir_daught_shw1[0]*mom_mag_daught_shw1;
    mom_daught_shw1[1] = dir_daught_shw1[1]*mom_mag_daught_shw1;
    mom_daught_shw1[2] = dir_daught_shw1[2]*mom_mag_daught_shw1;
  }
  if(length_daught_shw2>0){
    double mass_daught_shw2 = daught_shw2_sg->get_particle_mass();
    if(mass_daught_shw2<0){mass_daught_shw2=0;}
    double mom_mag_daught_shw2 = sqrt(pow(kine_energy_best_daught_shw2,2) + 2*kine_energy_best_daught_shw2*mass_daught_shw2);
    x_dir_daught_shw2 = dir_daught_shw2[0];
    y_dir_daught_shw2 = dir_daught_shw2[1];
    z_dir_daught_shw2 = dir_daught_shw2[2];
    mom_daught_shw2[0] = dir_daught_shw2[0]*mom_mag_daught_shw2;
    mom_daught_shw2[1] = dir_daught_shw2[1]*mom_mag_daught_shw2;
    mom_daught_shw2[2] = dir_daught_shw2[2]*mom_mag_daught_shw2;
  }

  // Set the main vertex
  WCPPID::ProtoVertex* ssm_main_vtx = main_vertex;
  WCPPID::ProtoVertex* ssm_second_vtx = find_other_vertex(ssm_sg,main_vertex);
  if(backwards_muon){
    ssm_main_vtx = ssm_second_vtx;
    ssm_second_vtx = main_vertex;
  }

  //now take a look at the off vertex stuff via the shower object
  WCPPID::ProtoSegment *offvtx_track1_sg;
  WCPPID::ProtoSegment *offvtx_shw1_sg;
  TVector3 dir_offvtx_track1(0,0,0);
  TVector3 dir_offvtx_shw1(0,0,0);
  TVector3 mom_offvtx_track1(0,0,0);
  TVector3 mom_offvtx_shw1(0,0,0);

  for (auto it = map_segment_vertices.begin(); it!= map_segment_vertices.end(); it++){
    WCPPID::ProtoSegment *sg = it->first;
    if( (map_vertex_segments[ssm_main_vtx].find(sg) != map_vertex_segments[ssm_main_vtx].end()) || (map_vertex_segments[ssm_second_vtx].find(sg) != map_vertex_segments[ssm_second_vtx].end()) ){
     continue; 
    }

    double sg_length = sg->get_length()/units::cm;
    double sg_pdg = abs(sg->get_particle_type());
    double dx = sg->get_point_vec().front().x/units::cm - ssm_main_vtx->get_fit_pt().x/units::cm;
    double dy = sg->get_point_vec().front().y/units::cm - ssm_main_vtx->get_fit_pt().y/units::cm;
    double dz = sg->get_point_vec().front().z/units::cm - ssm_main_vtx->get_fit_pt().z/units::cm;
    double sep_dist = sqrt(dx*dx+dy*dy+dz*dz);
    double sg_kine_energy_cal = sg->cal_kine_dQdx();
    if(sep_dist>80){
      continue;
    }
    off_vtx_length += sg_length;
    off_vtx_energy += sg_kine_energy_cal;
    if(sg_length>1){
      if (sg_pdg!=22 && sg_pdg!=11){
        n_offvtx_tracks_1 +=1;
      } else{ n_offvtx_showers_1 +=1;}
    }
    if(sg_length>3){
      if (sg_pdg!=22 && sg_pdg!=11){
        n_offvtx_tracks_3 +=1;
      } else{ n_offvtx_showers_3 +=1;}
    }
    if(sg_length>5){
      if (sg_pdg!=22 && sg_pdg!=11){
        n_offvtx_tracks_5 +=1;
      } else{ n_offvtx_showers_5 +=1;}
    }
    if(sg_length>8){
      if (sg_pdg!=22 && sg_pdg!=11){
        n_offvtx_tracks_8 +=1;
      } else{ n_offvtx_showers_8 +=1;}
    }
    if(sg_length>11){
      if (sg_pdg!=22 && sg_pdg!=11){
        n_offvtx_tracks_11 +=1;
      } else{ n_offvtx_showers_11 +=1;}
    }
    double sg_direct_length = sg->get_direct_length()/units::cm;
    double sg_medium_dq_dx =  sg->get_medium_dQ_dx()/(43e3/units::cm);
    double sg_max_dev = sg->get_max_deviation(int(0),int(sg->get_point_vec().size()))/units::cm;
    double sg_kine_energy_range = sg->cal_kine_range();
    std::vector<double> sg_kine_energy_range_pdg = calc_kine_range_multi_pdg(sg_length);
    TVector3 sg_dir = sg->cal_dir_3vector();
    if(sg_length>length_offvtx_track1 && sg_pdg!=22 && sg_pdg!=11){//this is the longest off vertex track
        length_offvtx_track1 = sg_length;
        direct_length_offvtx_track1 = sg_direct_length;
        pdg_offvtx_track1 = sg_pdg;
        medium_dq_dx_offvtx_track1 = sg_medium_dq_dx;
        max_dev_offvtx_track1 = sg_max_dev;
        kine_energy_cal_offvtx_track1 = sg_kine_energy_cal;
        kine_energy_range_offvtx_track1 = sg_kine_energy_range;
        kine_energy_range_mu_offvtx_track1 = sg_kine_energy_range_pdg.at(0);
        kine_energy_range_p_offvtx_track1 = sg_kine_energy_range_pdg.at(1);
        kine_energy_range_e_offvtx_track1 = sg_kine_energy_range_pdg.at(2);
        dir_offvtx_track1 = sg_dir;
        std::vector<double> scores_offvtx_track1 = get_scores(sg);
        score_mu_fwd_offvtx_track1 = scores_offvtx_track1.at(0);
        score_p_fwd_offvtx_track1 = scores_offvtx_track1.at(1);
        score_e_fwd_offvtx_track1 = scores_offvtx_track1.at(2);
        score_mu_bck_offvtx_track1 = scores_offvtx_track1.at(3);
        score_p_bck_offvtx_track1 = scores_offvtx_track1.at(4);
        score_e_bck_offvtx_track1 = scores_offvtx_track1.at(5);
	dist_mainvtx_offvtx_track1 = sep_dist;
        offvtx_track1_sg = sg;
        if(offvtx_track1_sg->get_flag_dir()==-1){
          std::swap(score_mu_fwd_offvtx_track1,score_mu_bck_offvtx_track1);
          std::swap(score_p_fwd_offvtx_track1,score_p_bck_offvtx_track1);
          std::swap(score_e_fwd_offvtx_track1,score_e_bck_offvtx_track1);
        }
    }
    else if(sg_length>length_offvtx_shw1 && (sg_pdg==22 || sg_pdg==11) ){//this is the longest off vertex shower
        length_offvtx_shw1 = sg_length;
        direct_length_offvtx_shw1 = sg_direct_length;
        pdg_offvtx_shw1 = sg_pdg;
        medium_dq_dx_offvtx_shw1 = sg_medium_dq_dx;
        max_dev_offvtx_shw1 = sg_max_dev;
        kine_energy_cal_offvtx_shw1 = sg_kine_energy_cal;
        kine_energy_range_offvtx_shw1 = sg_kine_energy_range;
        kine_energy_best_offvtx_shw1 = sg_kine_energy_range;
        auto it_shower = map_segment_in_shower.find(sg);
        if (it_shower!=map_segment_in_shower.end()){
          WCPPID::WCShower *shower = it_shower->second;
          kine_energy_best_offvtx_shw1 = shower->get_kine_best();
          if (kine_energy_best_offvtx_shw1==0 ) kine_energy_best_offvtx_shw1 = shower->get_kine_charge();
        }
        kine_energy_range_mu_offvtx_shw1 = sg_kine_energy_range_pdg.at(0);
        kine_energy_range_p_offvtx_shw1 = sg_kine_energy_range_pdg.at(1);
        kine_energy_range_e_offvtx_shw1 = sg_kine_energy_range_pdg.at(2);
        dir_offvtx_shw1 = sg_dir;
        std::vector<double> scores_offvtx_shw1 = get_scores(sg);
        score_mu_fwd_offvtx_shw1 = scores_offvtx_shw1.at(0);
        score_p_fwd_offvtx_shw1 = scores_offvtx_shw1.at(1);
        score_e_fwd_offvtx_shw1 = scores_offvtx_shw1.at(2);
        score_mu_bck_offvtx_shw1 = scores_offvtx_shw1.at(3);
        score_p_bck_offvtx_shw1 = scores_offvtx_shw1.at(4);
        score_e_bck_offvtx_shw1 = scores_offvtx_shw1.at(5);
        dist_mainvtx_offvtx_shw1 = sep_dist;
	offvtx_shw1_sg = sg;
        if(offvtx_shw1_sg->get_flag_dir()==-1){
          std::swap(score_mu_fwd_offvtx_shw1,score_mu_bck_offvtx_shw1);
          std::swap(score_p_fwd_offvtx_shw1,score_p_bck_offvtx_shw1);
          std::swap(score_e_fwd_offvtx_shw1,score_e_bck_offvtx_shw1);
        }
    }

  }
  if(length_offvtx_track1>0){
    double mass_offvtx_track1 = offvtx_track1_sg->get_particle_mass();
    if(mass_offvtx_track1<0){mass_offvtx_track1=0;}
    double mom_mag_offvtx_track1 = sqrt(pow(kine_energy_range_offvtx_track1,2) + 2*kine_energy_range_offvtx_track1*mass_offvtx_track1);
    x_dir_offvtx_track1 = dir_offvtx_track1[0];
    y_dir_offvtx_track1 = dir_offvtx_track1[1];
    z_dir_offvtx_track1 = dir_offvtx_track1[2];
    mom_offvtx_track1[0] = dir_offvtx_track1[0]*mom_mag_offvtx_track1;
    mom_offvtx_track1[1] = dir_offvtx_track1[1]*mom_mag_offvtx_track1;
    mom_offvtx_track1[2] = dir_offvtx_track1[2]*mom_mag_offvtx_track1;
  }
  if(length_offvtx_shw1>0){
    double mass_offvtx_shw1 = offvtx_shw1_sg->get_particle_mass();
    if(mass_offvtx_shw1<0){mass_offvtx_shw1=0;}
    double mom_mag_offvtx_shw1 = sqrt(pow(kine_energy_best_offvtx_shw1,2) + 2*kine_energy_best_offvtx_shw1*mass_offvtx_shw1);
    x_dir_offvtx_shw1 = dir_offvtx_shw1[0];
    y_dir_offvtx_shw1 = dir_offvtx_shw1[1];
    z_dir_offvtx_shw1 = dir_offvtx_shw1[2];
    mom_offvtx_shw1[0] = dir_offvtx_shw1[0]*mom_mag_offvtx_shw1;
    mom_offvtx_shw1[1] = dir_offvtx_shw1[1]*mom_mag_offvtx_shw1;
    mom_offvtx_shw1[2] = dir_offvtx_shw1[2]*mom_mag_offvtx_shw1;
  }

  //if there is a pi0 take a look at that too, this may be somewhat incorrect if the vertex is redefined
  TVector3 dir_pi0(0,0,0);
  TVector3 mom_pi0(0,0,0);
  if(kine_pio_mass>0){
    TVector3 p1(kine_pio_energy_1*TMath::Sin(kine_pio_theta_1/180.*3.1415926)*TMath::Cos(kine_pio_phi_1/180.*3.1415926), kine_pio_energy_1*TMath::Sin(kine_pio_theta_1/180.*3.1415926)*TMath::Sin(kine_pio_phi_1/180.*3.1415926), kine_pio_energy_1*TMath::Cos(kine_pio_theta_1/180.*3.1415926));
    TVector3 p2(kine_pio_energy_2*TMath::Sin(kine_pio_theta_2/180.*3.1415926)*TMath::Cos(kine_pio_phi_2/180.*3.1415926), kine_pio_energy_2*TMath::Sin(kine_pio_theta_2/180.*3.1415926)*TMath::Sin(kine_pio_phi_2/180.*3.1415926), kine_pio_energy_2*TMath::Cos(kine_pio_theta_2/180.*3.1415926));
    mom_pi0 = p1 + p2;
    dir_pi0 = mom_pi0.Unit();
  }

  double mom_mag = sqrt(pow(kine_energy,2) + 2*kine_energy*105.66);
  mom[0] = init_dir_10[0]*mom_mag;
  mom[1] = init_dir_10[1]*mom_mag;
  mom[2] = init_dir_10[2]*mom_mag;

  TVector3 nu_dir = mom+mom_prim_track1+mom_prim_track2+mom_prim_shw1+mom_prim_shw2 + mom_daught_track1+mom_daught_track2+mom_daught_shw1+mom_daught_shw2 + mom_offvtx_track1+mom_offvtx_shw1;
  nu_dir = nu_dir.Unit();
  nu_angle_to_z = acos( nu_dir[0]*dir_beam[0] + nu_dir[1]*dir_beam[1] + nu_dir[2]*dir_beam[2] );
  nu_angle_to_target = acos( nu_dir[0]*target_dir[0] + nu_dir[1]*target_dir[1] + nu_dir[2]*target_dir[2] );
  nu_angle_to_absorber = acos( nu_dir[0]*absorber_dir[0] + nu_dir[1]*absorber_dir[1] + nu_dir[2]*absorber_dir[2] );
  nu_angle_to_vertical = acos( nu_dir[0]*dir_vertical[0] + nu_dir[1]*dir_vertical[1] + nu_dir[2]*dir_vertical[2] );

  TVector3 con_nu_dir = mom+mom_prim_track1+mom_prim_track2+mom_prim_shw1+mom_prim_shw2 + mom_daught_track1+mom_daught_track2+mom_daught_shw1+mom_daught_shw2;
  con_nu_dir = con_nu_dir.Unit();
  con_nu_angle_to_z = acos( con_nu_dir[0]*dir_beam[0] + con_nu_dir[1]*dir_beam[1] + con_nu_dir[2]*dir_beam[2] );
  con_nu_angle_to_target = acos( con_nu_dir[0]*target_dir[0] + con_nu_dir[1]*target_dir[1] + con_nu_dir[2]*target_dir[2] );
  con_nu_angle_to_absorber = acos( con_nu_dir[0]*absorber_dir[0] + con_nu_dir[1]*absorber_dir[1] + con_nu_dir[2]*absorber_dir[2] );
  con_nu_angle_to_vertical = acos( con_nu_dir[0]*dir_vertical[0] + con_nu_dir[1]*dir_vertical[1] + con_nu_dir[2]*dir_vertical[2] );

  TVector3 prim_nu_dir = mom+mom_prim_track1+mom_prim_track2+mom_prim_shw1+mom_prim_shw2;
  prim_nu_dir = prim_nu_dir.Unit();
  prim_nu_angle_to_z = acos( prim_nu_dir[0]*dir_beam[0] + prim_nu_dir[1]*dir_beam[1] + prim_nu_dir[2]*dir_beam[2] );
  prim_nu_angle_to_target = acos( prim_nu_dir[0]*target_dir[0] + prim_nu_dir[1]*target_dir[1] + prim_nu_dir[2]*target_dir[2] );
  prim_nu_angle_to_absorber = acos( prim_nu_dir[0]*absorber_dir[0] + prim_nu_dir[1]*absorber_dir[1] + prim_nu_dir[2]*absorber_dir[2] );
  prim_nu_angle_to_vertical = acos( prim_nu_dir[0]*dir_vertical[0] + prim_nu_dir[1]*dir_vertical[1] + prim_nu_dir[2]*dir_vertical[2] );

  TVector3 track_dir = mom+mom_prim_track1+mom_prim_track2;
  track_dir = track_dir.Unit();
  track_angle_to_z = acos( track_dir[0]*dir_beam[0] + track_dir[1]*dir_beam[1] + track_dir[2]*dir_beam[2] );
  track_angle_to_target = acos( track_dir[0]*target_dir[0] + track_dir[1]*target_dir[1] + track_dir[2]*target_dir[2] );
  track_angle_to_absorber = acos( track_dir[0]*absorber_dir[0] + track_dir[1]*absorber_dir[1] + track_dir[2]*absorber_dir[2] );
  track_angle_to_vertical = acos( track_dir[0]*dir_vertical[0] + track_dir[1]*dir_vertical[1] + track_dir[2]*dir_vertical[2] );

  if(flag_ssmsp>=0){
    // Stuff for bookeeping
    std::set<WCPPID::ProtoVertex* > used_vertices;
    std::set<WCPPID::ProtoSegment* > used_segments;
    std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > segments_to_be_examined;
    int temp_acc_segment_id = acc_segment_id;

    //first add the ssm
    fill_ssmsp(ssm_sg, 13, 0, dir);
    segments_to_be_examined.push_back(std::make_pair(ssm_second_vtx,ssm_sg ));
    used_segments.insert(ssm_sg);

    //now all other tracks connected to the ssm main vertex
    for (auto it = map_vertex_segments[ssm_main_vtx].begin(); it != map_vertex_segments[ssm_main_vtx].end(); it++){
      if(*it==ssm_sg) continue; // We already did the ssm, so skip it
      fill_ssmsp(*it, 2212, 0, (*it)->get_flag_dir()); // Assume everything connected to the ssm is a proton
      used_segments.insert(*it);
      WCPPID::ProtoVertex *other_vertex = find_other_vertex(*it, ssm_main_vtx);
      segments_to_be_examined.push_back(std::make_pair(other_vertex, *it));
    }
    used_vertices.insert(ssm_main_vtx);
  
    // now all the daughters
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

          // fill the tree, we will trust the nominal PID for daughters
          fill_ssmsp(curr_sg, curr_sg->get_particle_type(), prev_sg->get_id()+prev_sg->get_cluster_id()*1000, curr_sg->get_flag_dir());

          used_segments.insert(curr_sg);
	  WCPPID::ProtoVertex *other_vertex = find_other_vertex(curr_sg, curr_vtx);
	  if (used_vertices.find(other_vertex) == used_vertices.end())
	    temp_segments.push_back(std::make_pair(other_vertex, curr_sg));
        }
        used_vertices.insert(curr_vtx);
      }
      segments_to_be_examined = temp_segments;
    }

    //check for other showers which may not be in the same cluster
    for (auto it = showers.begin(); it!= showers.end(); it++){
      WCPPID::WCShower *shower = *it;
      WCPPID::ProtoSegment* curr_sg = shower->get_start_segment();
      std::pair<ProtoVertex*, int> pair_vertex = shower->get_start_vertex();
      int temp_mother_id = 0;
      double kine_best = shower->get_kine_best();
      if (kine_best ==0 ) kine_best = shower->get_kine_charge();
      if (pair_vertex.second == 1){ // direct connection
        WCPPID::ProtoSegment* curr_sg = shower->get_start_segment();
        if (used_segments.find(curr_sg)!=used_segments.end()) {std::cout<<"Shower "<<curr_sg->get_id()+curr_sg->get_cluster_id()*1000 <<" already added"<<std::endl; temp_mother_id = 0;}
        else if (pair_vertex.first == ssm_main_vtx){
          std::cout<<"pair_vertex.first == main_vertex but sg not in used_segments"<<std::endl;
          used_segments.insert(curr_sg);
          temp_mother_id = 0;

        }else{
          WCPPID::ProtoSegment* prev_sg = 0;
          if (map_vertex_in_shower.find(pair_vertex.first) != map_vertex_in_shower.end()){
            prev_sg = map_vertex_in_shower[pair_vertex.first]->get_start_segment();
          }else{
            prev_sg = find_incoming_segment(pair_vertex.first);
          }
          temp_mother_id = prev_sg->get_id()+prev_sg->get_cluster_id()*1000;
          // fill the tree, we will trust the nominal PID for daughters
          fill_ssmsp(curr_sg, shower->get_particle_type(), temp_mother_id, curr_sg->get_flag_dir());
        }
      }else if (pair_vertex.second == 2 || pair_vertex.second == 3){
        if (pair_vertex.first == ssm_main_vtx){
          temp_mother_id = fill_ssmsp_psuedo(shower,0,temp_acc_segment_id);//daughter, mother, id
          temp_acc_segment_id++;
          fill_ssmsp(curr_sg, curr_sg->get_particle_type(), temp_mother_id, curr_sg->get_flag_dir());
        }else{
          WCPPID::ProtoSegment* prev_sg = 0;
          if (map_vertex_in_shower.find(pair_vertex.first) != map_vertex_in_shower.end()){
            prev_sg = map_vertex_in_shower[pair_vertex.first]->get_start_segment();
          }else{
            prev_sg = find_incoming_segment(pair_vertex.first);
          }
          if(prev_sg==0){
            std::cout<<"Missed Prev Seg, Setting to ssm "<<std::endl;
            temp_mother_id = fill_ssmsp_psuedo(shower,ssm_sg->get_cluster_id()*1000+ssm_sg->get_id(),temp_acc_segment_id);//daughter, mother, id
          }
          else{
            temp_mother_id= fill_ssmsp_psuedo(shower, prev_sg, temp_acc_segment_id);
          }
          // fill the tree, we will trust the nominal PID for daughters
          temp_acc_segment_id++;
          fill_ssmsp(curr_sg, curr_sg->get_particle_type(), temp_mother_id, curr_sg->get_flag_dir());
        }
      }else{std::cout<<curr_sg->get_id()+curr_sg->get_cluster_id()*1000<<" has pair vertex "<<pair_vertex.second<<std::endl; continue;}//missing case is 4, really far away so do not save
      used_segments.insert(curr_sg);
      // Filling in the other space points?
      std::cout<<std::endl;
      std::cout<<"Checking for other Shower spacepoints in "<<curr_sg->get_id()+curr_sg->get_cluster_id()*1000<<std::endl;
      Map_Proto_Segment_Vertices& tmp_map_seg_vtxs = shower->get_map_seg_vtxs();
      for (auto it = tmp_map_seg_vtxs.begin(); it!= tmp_map_seg_vtxs.end(); it++){
        WCPPID::ProtoSegment *shower_sg = it->first;
        if (used_segments.find(shower_sg)!=used_segments.end()) {std::cout<<"Shower segment "<<shower_sg->get_id()+shower_sg->get_cluster_id()*1000<<" already added"<<std::endl; continue;}
          //put a gamma between here and the start of the shower. Appreoximation for now, could maybe make this better
          std::cout<<"Adding segment and sudo segment"<<std::endl;
          if(temp_mother_id==0 && ssm_main_vtx!=main_vertex){std::cout<<"Flipping mother"<<std::endl; temp_mother_id=ssm_sg->get_cluster_id()*1000+ssm_sg->get_id();}
          int psuedo_particle_id = fill_ssmsp_psuedo(shower,shower_sg,temp_mother_id,temp_acc_segment_id);
          temp_acc_segment_id++;
          fill_ssmsp(shower_sg, shower_sg->get_particle_type(), psuedo_particle_id, shower_sg->get_flag_dir());
          used_segments.insert(shower_sg);
      }
      std::cout<<"Check completed"<<std::endl;
      std::cout<<std::endl;
    }
  }

    bool flag_st_kdar = false;
    if(n_prim_tracks_1==0 && n_prim_all_3==0 && n_daughter_tracks_5==0 && n_daughter_all_5<2 && Nsm_wivtx==1 && !(kine_pio_mass>70 && kine_pio_mass<200)){
      std::cout<<"Pass KDAR Cutbased"<<std::endl;
      flag_st_kdar = true;
    }
    else if(Nsm_wivtx!=1){std::cout<<"Fail: no short muons with vertex activity"<<std::endl;}
    else if(kine_pio_mass>70 && kine_pio_mass<20){std::cout<<"Fail: pi0 spotted"<<std::endl;}
    else if(n_daughter_all_5>=2){std::cout<<"Fail: too many daughter showers"<<std::endl;}
    else if(n_daughter_tracks_5!=0){std::cout<<"Fail: long daughter track"<<std::endl;}
    else if(n_prim_tracks_1!=0){std::cout<<"Fail: other primary track"<<std::endl;}
    else if(n_prim_all_3!=0){std::cout<<"Fail: other primary shower"<<std::endl;}
    std::cout<<std::endl;


  tagger_info.ssm_flag_st_kdar = flag_st_kdar; //pass cutbased

  //only filled if there is one ssm
    //properties of the ssm
      //dq/dx info
  tagger_info.ssm_dq_dx_fwd_1 = dq_dx_fwd_1;//1st dq dx point wrt vertex
  tagger_info.ssm_dq_dx_fwd_2 = dq_dx_fwd_2;//2nd dq dx point wrt vertex
  tagger_info.ssm_dq_dx_fwd_3 = dq_dx_fwd_3;//3rd dq dx point wrt vertex
  tagger_info.ssm_dq_dx_fwd_4 = dq_dx_fwd_4;//4th dq dx point wrt vertex
  tagger_info.ssm_dq_dx_fwd_5 = dq_dx_fwd_5;//5th dq dx point wrt vertex
  tagger_info.ssm_dq_dx_bck_1 = dq_dx_bck_1;//1st dq dx point wrt end
  tagger_info.ssm_dq_dx_bck_2 = dq_dx_bck_2;//2nd dq dx point wrt end
  tagger_info.ssm_dq_dx_bck_3 = dq_dx_bck_3;//3rd dq dx point wrt end
  tagger_info.ssm_dq_dx_bck_4 = dq_dx_bck_4;//4th dq dx point wrt end
  tagger_info.ssm_dq_dx_bck_5 = dq_dx_bck_5;//5th dq dx point wrt end
  tagger_info.ssm_d_dq_dx_fwd_12 = d_dq_dx_fwd_12;//1st d dq dx point wrt vertex
  tagger_info.ssm_d_dq_dx_fwd_23 = d_dq_dx_fwd_23;//2nd d dq dx point wrt vertex
  tagger_info.ssm_d_dq_dx_fwd_34 = d_dq_dx_fwd_34;//3rd d dq dx point wrt vertex
  tagger_info.ssm_d_dq_dx_fwd_45 = d_dq_dx_fwd_45;//4th d dq dx point wrt vertex
  tagger_info.ssm_d_dq_dx_bck_12 = d_dq_dx_bck_12;//1st d dq dx point wrt end
  tagger_info.ssm_d_dq_dx_bck_23 = d_dq_dx_bck_23;//2nd d dq dx point wrt end
  tagger_info.ssm_d_dq_dx_bck_34 = d_dq_dx_bck_34;//3rd d dq dx point wrt end
  tagger_info.ssm_d_dq_dx_bck_45 = d_dq_dx_bck_45;//4th d dq dx point wrt end
  tagger_info.ssm_max_dq_dx_fwd_3 = max_dq_dx_fwd_3;//max dq/dx in first 3 points wrt vertex
  tagger_info.ssm_max_dq_dx_fwd_5 = max_dq_dx_fwd_5;//max dq/dx in first 5 points wrt vertex
  tagger_info.ssm_max_dq_dx_bck_3 = max_dq_dx_bck_3;//max dq/dx in first 3 points wrt end
  tagger_info.ssm_max_dq_dx_bck_5 = max_dq_dx_bck_5;//max dq/dx in first 5 points wrt end
  tagger_info.ssm_max_d_dq_dx_fwd_3 = max_d_dq_dx_fwd_3;//max d dq/dx in first 3 points wrt vertex
  tagger_info.ssm_max_d_dq_dx_fwd_5 = max_d_dq_dx_fwd_5;//max d dq/dx in first 5 points wrt vertex
  tagger_info.ssm_max_d_dq_dx_bck_3 = max_d_dq_dx_bck_3;//max d dq/dx in first 3 points wrt end
  tagger_info.ssm_max_d_dq_dx_bck_5 = max_d_dq_dx_bck_5;//max d dq/dx in first 5 points wrt end
  tagger_info.ssm_medium_dq_dx = medium_dq_dx;//medium dq/dx over the entire track
  tagger_info.ssm_medium_dq_dx_bp = medium_dq_dx_bp;//medium dq/dx over the entire track without the vertex activity
      //angluar info
  tagger_info.ssm_angle_to_z = angle_to_z_10;//angle to the z det cord
  tagger_info.ssm_angle_to_target = angle_to_target_10;//angle to the target
  tagger_info.ssm_angle_to_absorber = angle_to_absorber_10;//angle to the absorber
  tagger_info.ssm_angle_to_vertical = angle_to_vertical_10;//angle to the vertical
      //direction info
  tagger_info.ssm_x_dir = x_dir;//x det cord direction
  tagger_info.ssm_y_dir = y_dir;//y det cord direction
  tagger_info.ssm_z_dir = z_dir;//z det cord direction
      //energy info
  tagger_info.ssm_kine_energy = kine_energy;//KE of the muon with range
  tagger_info.ssm_kine_energy_reduced = kine_energy_reduced;//KE of the muon with range, exluding high dqdx at vertex
      //general properties
  tagger_info.ssm_vtx_activity = vtx_activity;//flag for identified vertex activity
  tagger_info.ssm_pdg = pdg;//initial pdg assigned to the track
  tagger_info.ssm_dQ_dx_cut = dQ_dx_cut;//cut normally used to choose muons
  tagger_info.ssm_score_mu_fwd = score_mu_fwd;//ID score normally normally used to choose muons
  tagger_info.ssm_score_p_fwd = score_p_fwd;//ID score normally normally used to choose protons
  tagger_info.ssm_score_e_fwd = score_e_fwd;//ID score normally normally used to choose e
  tagger_info.ssm_score_mu_bck = score_mu_bck;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_score_p_bck = score_p_bck;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_score_e_bck = score_e_bck;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_score_mu_fwd_bp = score_mu_fwd_bp;//ID score normally normally used to choose muons but calculated without the vertex activity
  tagger_info.ssm_score_p_fwd_bp = score_p_fwd_bp;//ID score normally normally used to choose protons but calculated without the vertex activity
  tagger_info.ssm_score_e_fwd_bp = score_e_fwd_bp;//ID score normally normally used to choose e but calculated without the vertex activity
      //track "straighness"
  tagger_info.ssm_length = length;//length of track direct caluclated from point to point
  tagger_info.ssm_direct_length = direct_length;//length of track direct from end to end
  double length_ratio = -999;
  if(direct_length>0 && length>0) length_ratio = direct_length/length;
  tagger_info.ssm_length_ratio = length_ratio;
  tagger_info.ssm_max_dev = max_dev;//furthest distance from track's primary axis along the track
    //number of other particles
  tagger_info.ssm_n_prim_tracks_1 = n_prim_tracks_1;//number of other primary tracks greater than 1 cm
  tagger_info.ssm_n_prim_tracks_3 = n_prim_tracks_3;//number of other primary tracks greater than 3 cm
  tagger_info.ssm_n_prim_tracks_5 = n_prim_tracks_5;//number of other primary tracks greater than 5 cm
  tagger_info.ssm_n_prim_tracks_8 = n_prim_tracks_8;//number of other primary tracks greater than 8 cm
  tagger_info.ssm_n_prim_tracks_11 = n_prim_tracks_11;//number of other primary tracks greater than 11 cm
  tagger_info.ssm_n_all_tracks_1 = n_prim_all_1;//number of other primary tracks and showers greater than 1 cm
  tagger_info.ssm_n_all_tracks_3 = n_prim_all_3;//number of other primary tracks and showers greater than 3 cm
  tagger_info.ssm_n_all_tracks_5 = n_prim_all_5;//number of other primary tracks and showers greater than 5 cm
  tagger_info.ssm_n_all_tracks_8 = n_prim_all_8;//number of other primary tracks and showers greater than 8 cm
  tagger_info.ssm_n_all_tracks_11 = n_prim_all_11;//number of other primary tracks and showers greater than 11 cm
  tagger_info.ssm_n_daughter_tracks_1 = n_daughter_tracks_1;//number of other daughter tracks greater than 1 cm
  tagger_info.ssm_n_daughter_tracks_3 = n_daughter_tracks_3;//number of other daughter tracks greater than 3 cm
  tagger_info.ssm_n_daughter_tracks_5 = n_daughter_tracks_5;//number of other daughter tracks greater than 5 cm
  tagger_info.ssm_n_daughter_tracks_8 = n_daughter_tracks_8;//number of other daughter tracks greater than 8 cm
  tagger_info.ssm_n_daughter_tracks_11 = n_daughter_tracks_11;//number of other daughter tracks greater than 11 cm
  tagger_info.ssm_n_daughter_all_1 = n_daughter_all_1;//number of other daughter tracks and showers greater than 1 cm
  tagger_info.ssm_n_daughter_all_3 = n_daughter_all_3;//number of other daughter tracks and showers greater than 3 cm
  tagger_info.ssm_n_daughter_all_5 = n_daughter_all_5;//number of other daughter tracks and showers greater than 5 cm
  tagger_info.ssm_n_daughter_all_8 = n_daughter_all_8;//number of other daughter tracks and showers greater than 8 cm
  tagger_info.ssm_n_daughter_all_11 = n_daughter_all_11;//number of other daughter tracks and showers greater than 11 cm
    //properties of leading other primary track
  tagger_info.ssm_prim_track1_pdg = pdg_prim_track1;//initial pdg assigned to the track
  tagger_info.ssm_prim_track1_score_mu_fwd = score_mu_fwd_prim_track1;//ID score normally normally used to choose muons
  tagger_info.ssm_prim_track1_score_p_fwd = score_p_fwd_prim_track1;//ID score normally normally used to choose protons
  tagger_info.ssm_prim_track1_score_e_fwd = score_e_fwd_prim_track1;//ID score normally normally used to choose e
  tagger_info.ssm_prim_track1_score_mu_bck = score_mu_bck_prim_track1;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_prim_track1_score_p_bck = score_p_bck_prim_track1;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_prim_track1_score_e_bck = score_e_bck_prim_track1;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_prim_track1_length = length_prim_track1;//length of track direct caluclated from point to point
  tagger_info.ssm_prim_track1_direct_length = direct_length_prim_track1;//length of track direct from end to end
  double length_ratio_prim_track1 = -999;
  if(direct_length_prim_track1>0 && length_prim_track1>0) length_ratio_prim_track1 = direct_length_prim_track1/length_prim_track1;
  tagger_info.ssm_prim_track1_length_ratio = length_ratio_prim_track1;
  tagger_info.ssm_prim_track1_max_dev = max_dev_prim_track1;//furthest distance from track's primary axis along the track
  tagger_info.ssm_prim_track1_kine_energy_range = kine_energy_range_prim_track1;//KE of the muon with range
  tagger_info.ssm_prim_track1_kine_energy_range_mu = kine_energy_range_mu_prim_track1;
  tagger_info.ssm_prim_track1_kine_energy_range_p = kine_energy_range_p_prim_track1;
  tagger_info.ssm_prim_track1_kine_energy_range_e = kine_energy_range_e_prim_track1;
  tagger_info.ssm_prim_track1_kine_energy_cal = kine_energy_cal_prim_track1;//KE of the track with dq/dx
  tagger_info.ssm_prim_track1_medium_dq_dx = medium_dq_dx_prim_track1;//medium dq/dx over the entire track
  tagger_info.ssm_prim_track1_x_dir = x_dir_prim_track1;//x det cord direction
  tagger_info.ssm_prim_track1_y_dir = y_dir_prim_track1;//y det cord direction
  tagger_info.ssm_prim_track1_z_dir = z_dir_prim_track1;//z det cord direction
  tagger_info.ssm_prim_track1_add_daught_track_counts_1 = add_daught_track_counts_1_prim_track1;//additional daughter tracks longer than 1 cm
  tagger_info.ssm_prim_track1_add_daught_all_counts_1 = add_daught_all_counts_1_prim_track1;//additional daughter tracks+showers longer than 1 cm
  tagger_info.ssm_prim_track1_add_daught_track_counts_5 = add_daught_track_counts_5_prim_track1;//additional daughter tracks longer than 5 cm
  tagger_info.ssm_prim_track1_add_daught_all_counts_5 = add_daught_all_counts_5_prim_track1;//additional daughter tracks+showers longer than 5 cm
  tagger_info.ssm_prim_track1_add_daught_track_counts_11 = add_daught_track_counts_11_prim_track1;//additional daughter tracks longer than 11 cm
  tagger_info.ssm_prim_track1_add_daught_all_counts_11 = add_daught_all_counts_11_prim_track1;//additional daughter tracks+showers longer than 11 cm
  //properties of sub-leading other primary track
  tagger_info.ssm_prim_track2_pdg = pdg_prim_track2;//initial pdg assigned to the track
  tagger_info.ssm_prim_track2_score_mu_fwd = score_mu_fwd_prim_track2;//ID score normally normally used to choose muons
  tagger_info.ssm_prim_track2_score_p_fwd = score_p_fwd_prim_track2;//ID score normally normally used to choose protons
  tagger_info.ssm_prim_track2_score_e_fwd = score_e_fwd_prim_track2;//ID score normally normally used to choose e
  tagger_info.ssm_prim_track2_score_mu_bck = score_mu_bck_prim_track2;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_prim_track2_score_p_bck = score_p_bck_prim_track2;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_prim_track2_score_e_bck = score_e_bck_prim_track2;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_prim_track2_length = length_prim_track2;//length of track direct caluclated from point to point
  tagger_info.ssm_prim_track2_direct_length = direct_length_prim_track2;//length of track direct from end to end
  double length_ratio_prim_track2 = -999;
  if(direct_length_prim_track2>0 && length_prim_track2>0) length_ratio_prim_track2 = direct_length_prim_track2/length_prim_track2;
  tagger_info.ssm_prim_track2_length_ratio = length_ratio_prim_track2;
  tagger_info.ssm_prim_track2_max_dev = max_dev_prim_track2;//furthest distance from track's primary axis along the track
  tagger_info.ssm_prim_track2_kine_energy_range = kine_energy_range_prim_track2;//KE of the track with range
  tagger_info.ssm_prim_track2_kine_energy_range_mu = kine_energy_range_mu_prim_track2;
  tagger_info.ssm_prim_track2_kine_energy_range_p = kine_energy_range_p_prim_track2;
  tagger_info.ssm_prim_track2_kine_energy_range_e = kine_energy_range_e_prim_track2;
  tagger_info.ssm_prim_track2_kine_energy_cal = kine_energy_cal_prim_track2;//KE of the track with dq/dx
  tagger_info.ssm_prim_track2_medium_dq_dx = medium_dq_dx_prim_track2;//medium dq/dx over the entire track
  tagger_info.ssm_prim_track2_x_dir = x_dir_prim_track2;//x det cord direction
  tagger_info.ssm_prim_track2_y_dir = y_dir_prim_track2;//y det cord direction
  tagger_info.ssm_prim_track2_z_dir = z_dir_prim_track2;//z det cord direction
  tagger_info.ssm_prim_track2_add_daught_track_counts_1 = add_daught_track_counts_1_prim_track2;//additional daughter tracks longer than 1 cm
  tagger_info.ssm_prim_track2_add_daught_all_counts_1 = add_daught_all_counts_1_prim_track2;//additional daughter tracks+showers longer than 1 cm
  tagger_info.ssm_prim_track2_add_daught_track_counts_5 = add_daught_track_counts_5_prim_track2;//additional daughter tracks longer than 5 cm
  tagger_info.ssm_prim_track2_add_daught_all_counts_5 = add_daught_all_counts_5_prim_track2;//additional daughter tracks+showers longer than 5 cm
  tagger_info.ssm_prim_track2_add_daught_track_counts_11 = add_daught_track_counts_11_prim_track2;//additional daughter tracks longer than 11 cm
  tagger_info.ssm_prim_track2_add_daught_all_counts_11 = add_daught_all_counts_11_prim_track2;//additional daughter tracks+showers longer than 11 cm
  //properties of leading daughter track
  tagger_info.ssm_daught_track1_pdg = pdg_daught_track1;//initial pdg assigned to the track
  tagger_info.ssm_daught_track1_score_mu_fwd = score_mu_fwd_daught_track1;//ID score normally normally used to choose muons
  tagger_info.ssm_daught_track1_score_p_fwd = score_p_fwd_daught_track1;//ID score normally normally used to choose protons
  tagger_info.ssm_daught_track1_score_e_fwd = score_e_fwd_daught_track1;//ID score normally normally used to choose e
  tagger_info.ssm_daught_track1_score_mu_bck = score_mu_bck_daught_track1;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_daught_track1_score_p_bck = score_p_bck_daught_track1;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_daught_track1_score_e_bck = score_e_bck_daught_track1;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_daught_track1_length = length_daught_track1;//length of track direct caluclated from point to point
  tagger_info.ssm_daught_track1_direct_length = direct_length_daught_track1;//length of track direct from end to end
  double length_ratio_daught_track1 = -999;
  if(direct_length_daught_track1>0 && length_daught_track1>0) length_ratio_daught_track1 = direct_length_daught_track1/length_daught_track1;
  tagger_info.ssm_daught_track1_length_ratio = length_ratio_daught_track1;
  tagger_info.ssm_daught_track1_max_dev = max_dev_daught_track1;//furthest distance from track's daughtary axis along the track
  tagger_info.ssm_daught_track1_kine_energy_range = kine_energy_range_daught_track1;//KE of the track with range
  tagger_info.ssm_daught_track1_kine_energy_range_mu = kine_energy_range_mu_daught_track1;
  tagger_info.ssm_daught_track1_kine_energy_range_p = kine_energy_range_p_daught_track1;
  tagger_info.ssm_daught_track1_kine_energy_range_e = kine_energy_range_e_daught_track1;
  tagger_info.ssm_daught_track1_kine_energy_cal = kine_energy_cal_daught_track1;//KE of the track with dq/dx
  tagger_info.ssm_daught_track1_medium_dq_dx = medium_dq_dx_daught_track1;//medium dq/dx over the entire track 
  tagger_info.ssm_daught_track1_x_dir = x_dir_daught_track1;//x det cord direction
  tagger_info.ssm_daught_track1_y_dir = y_dir_daught_track1;//y det cord direction
  tagger_info.ssm_daught_track1_z_dir = z_dir_daught_track1;//z det cord direction
  tagger_info.ssm_daught_track1_add_daught_track_counts_1 = add_daught_track_counts_1_daught_track1;//additional daughter tracks longer than 1 cm
  tagger_info.ssm_daught_track1_add_daught_all_counts_1 = add_daught_all_counts_1_daught_track1;//additional daughter tracks+showers longer than 1 cm
  tagger_info.ssm_daught_track1_add_daught_track_counts_5 = add_daught_track_counts_5_daught_track1;//additional daughter tracks longer than 5 cm
  tagger_info.ssm_daught_track1_add_daught_all_counts_5 = add_daught_all_counts_5_daught_track1;//additional daughter tracks+showers longer than 5 cm
  tagger_info.ssm_daught_track1_add_daught_track_counts_11 = add_daught_track_counts_11_daught_track1;//additional daughter tracks longer than 11 cm
  tagger_info.ssm_daught_track1_add_daught_all_counts_11 = add_daught_all_counts_11_daught_track1;//additional daughter tracks+showers longer than 11 cm
  //properties of sub-leading daughter track
  tagger_info.ssm_daught_track2_pdg = pdg_daught_track2;//initial pdg assigned to the track
  tagger_info.ssm_daught_track2_score_mu_fwd = score_mu_fwd_daught_track2;//ID score normally normally used to choose muons
  tagger_info.ssm_daught_track2_score_p_fwd = score_p_fwd_daught_track2;//ID score normally normally used to choose protons
  tagger_info.ssm_daught_track2_score_e_fwd = score_e_fwd_daught_track2;//ID score normally normally used to choose e
  tagger_info.ssm_daught_track2_score_mu_bck = score_mu_bck_daught_track2;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_daught_track2_score_p_bck = score_p_bck_daught_track2;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_daught_track2_score_e_bck = score_e_bck_daught_track2;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_daught_track2_length = length_daught_track2;//length of track direct caluclated from point to point
  tagger_info.ssm_daught_track2_direct_length = direct_length_daught_track2;//length of track direct from end to end
  double length_ratio_daught_track2 = -999;
  if(direct_length_daught_track2>0 && length_daught_track2>0) length_ratio_daught_track2 = direct_length_daught_track2/length_daught_track2;
  tagger_info.ssm_daught_track2_length_ratio = length_ratio_daught_track2;
  tagger_info.ssm_daught_track2_max_dev = max_dev_daught_track2;//furthest distance from track's daughtary axis along the track
  tagger_info.ssm_daught_track2_kine_energy_range = kine_energy_range_daught_track2;//KE of the muon with range
  tagger_info.ssm_daught_track2_kine_energy_range_mu = kine_energy_range_mu_daught_track2;
  tagger_info.ssm_daught_track2_kine_energy_range_p = kine_energy_range_p_daught_track2;
  tagger_info.ssm_daught_track2_kine_energy_range_e = kine_energy_range_e_daught_track2;
  tagger_info.ssm_daught_track2_kine_energy_cal = kine_energy_cal_daught_track2;//KE of the muon with dq/dx
  tagger_info.ssm_daught_track2_medium_dq_dx = medium_dq_dx_daught_track2;//medium dq/dx over the entire track
  tagger_info.ssm_daught_track2_x_dir = x_dir_daught_track2;//x det cord direction
  tagger_info.ssm_daught_track2_y_dir = y_dir_daught_track2;//y det cord direction
  tagger_info.ssm_daught_track2_z_dir = z_dir_daught_track2;//z det cord direction
  tagger_info.ssm_daught_track2_add_daught_track_counts_1 = add_daught_track_counts_1_daught_track2;//additional daughter tracks longer than 1 cm
  tagger_info.ssm_daught_track2_add_daught_all_counts_1 = add_daught_all_counts_1_daught_track2;//additional daughter tracks+showers longer than 1 cm
  tagger_info.ssm_daught_track2_add_daught_track_counts_5 = add_daught_track_counts_5_daught_track2;//additional daughter tracks longer than 5 cm
  tagger_info.ssm_daught_track2_add_daught_all_counts_5 = add_daught_all_counts_5_daught_track2;//additional daughter tracks+showers longer than 5 cm
  tagger_info.ssm_daught_track2_add_daught_track_counts_11 = add_daught_track_counts_11_daught_track2;//additional daughter tracks longer than 11 cm
  tagger_info.ssm_daught_track2_add_daught_all_counts_11 = add_daught_all_counts_11_daught_track2;//additional daughter tracks+showers longer than 11 cm
  //properties of leading other primary shower
  tagger_info.ssm_prim_shw1_pdg = pdg_prim_shw1;//initial pdg assigned to the shw
  tagger_info.ssm_prim_shw1_score_mu_fwd = score_mu_fwd_prim_shw1;//ID score normally normally used to choose muons
  tagger_info.ssm_prim_shw1_score_p_fwd = score_p_fwd_prim_shw1;//ID score normally normally used to choose protons
  tagger_info.ssm_prim_shw1_score_e_fwd = score_e_fwd_prim_shw1;//ID score normally normally used to choose e
  tagger_info.ssm_prim_shw1_score_mu_bck = score_mu_bck_prim_shw1;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_prim_shw1_score_p_bck = score_p_bck_prim_shw1;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_prim_shw1_score_e_bck = score_e_bck_prim_shw1;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_prim_shw1_length = length_prim_shw1;//length of shw direct caluclated from point to point
  tagger_info.ssm_prim_shw1_direct_length = direct_length_prim_shw1;//length of shw direct from end to end
  double length_ratio_prim_shw1 = -999;
  if(direct_length_prim_shw1>0 && length_prim_shw1>0) length_ratio_prim_shw1 = direct_length_prim_shw1/length_prim_shw1;
  tagger_info.ssm_prim_shw1_length_ratio = length_ratio_prim_shw1;
  tagger_info.ssm_prim_shw1_max_dev = max_dev_prim_shw1;//furthest distance from shw's primary axis along the shw
  tagger_info.ssm_prim_shw1_kine_energy_best  =  kine_energy_best_prim_shw1;//best estimate of the KE of the shower
  tagger_info.ssm_prim_shw1_kine_energy_range = kine_energy_range_prim_shw1;//KE of the muon with range
  tagger_info.ssm_prim_shw1_kine_energy_range_mu = kine_energy_range_mu_prim_shw1;
  tagger_info.ssm_prim_shw1_kine_energy_range_p = kine_energy_range_p_prim_shw1;
  tagger_info.ssm_prim_shw1_kine_energy_range_e = kine_energy_range_e_prim_shw1;
  tagger_info.ssm_prim_shw1_kine_energy_cal = kine_energy_cal_prim_shw1;//KE of the muon with dq/dx
  tagger_info.ssm_prim_shw1_medium_dq_dx = medium_dq_dx_prim_shw1;//medium dq/dx over the entire shw
  tagger_info.ssm_prim_shw1_x_dir = x_dir_prim_shw1;//x det cord direction
  tagger_info.ssm_prim_shw1_y_dir = y_dir_prim_shw1;//y det cord direction
  tagger_info.ssm_prim_shw1_z_dir = z_dir_prim_shw1;//z det cord direction
  tagger_info.ssm_prim_shw1_add_daught_track_counts_1 = add_daught_track_counts_1_prim_shw1;//additional daughter tracks longer than 1 cm
  tagger_info.ssm_prim_shw1_add_daught_all_counts_1 = add_daught_all_counts_1_prim_shw1;//additional daughter tracks+showers longer than 1 cm
  tagger_info.ssm_prim_shw1_add_daught_track_counts_5 = add_daught_track_counts_5_prim_shw1;//additional daughter tracks longer than 5 cm
  tagger_info.ssm_prim_shw1_add_daught_all_counts_5 = add_daught_all_counts_5_prim_shw1;//additional daughter tracks+showers longer than 5 cm
  tagger_info.ssm_prim_shw1_add_daught_track_counts_11 = add_daught_track_counts_11_prim_shw1;//additional daughter tracks longer than 11 cm
  tagger_info.ssm_prim_shw1_add_daught_all_counts_11 = add_daught_all_counts_11_prim_shw1;//additional daughter tracks+showers longer than 11 cm
  //properties of sub-leading other primary shower
  tagger_info.ssm_prim_shw2_pdg = pdg_prim_shw2;//initial pdg assigned to the shw
  tagger_info.ssm_prim_shw2_score_mu_fwd = score_mu_fwd_prim_shw2;//ID score normally normally used to choose muons
  tagger_info.ssm_prim_shw2_score_p_fwd = score_p_fwd_prim_shw2;//ID score normally normally used to choose protons
  tagger_info.ssm_prim_shw2_score_e_fwd = score_e_fwd_prim_shw2;//ID score normally normally used to choose e
  tagger_info.ssm_prim_shw2_score_mu_bck = score_mu_bck_prim_shw2;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_prim_shw2_score_p_bck = score_p_bck_prim_shw2;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_prim_shw2_score_e_bck = score_e_bck_prim_shw2;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_prim_shw2_length = length_prim_shw2;//length of shw direct caluclated from point to point
  tagger_info.ssm_prim_shw2_direct_length = direct_length_prim_shw2;//length of shw direct from end to end
  double length_ratio_prim_shw2 = -999;
  if(direct_length_prim_shw2>0 && length_prim_shw2>0) length_ratio_prim_shw2 = direct_length_prim_shw2/length_prim_shw2;
  tagger_info.ssm_prim_shw2_length_ratio = length_ratio_prim_shw2;
  tagger_info.ssm_prim_shw2_max_dev = max_dev_prim_shw2;//furthest distance from shw's primary axis along the shw
  tagger_info.ssm_prim_shw2_kine_energy_best  =  kine_energy_best_prim_shw2;//best estimate of the KE of the shower
  tagger_info.ssm_prim_shw2_kine_energy_range = kine_energy_range_prim_shw2;//KE of the shower with range
  tagger_info.ssm_prim_shw2_kine_energy_range_mu = kine_energy_range_mu_prim_shw2;
  tagger_info.ssm_prim_shw2_kine_energy_range_p = kine_energy_range_p_prim_shw2;
  tagger_info.ssm_prim_shw2_kine_energy_range_e = kine_energy_range_e_prim_shw2;
  tagger_info.ssm_prim_shw2_kine_energy_cal = kine_energy_cal_prim_shw2;//KE of the shower with dq/dx
  tagger_info.ssm_prim_shw2_medium_dq_dx = medium_dq_dx_prim_shw2;//medium dq/dx over the entire shw
  tagger_info.ssm_prim_shw2_x_dir = x_dir_prim_shw2;//x det cord direction
  tagger_info.ssm_prim_shw2_y_dir = y_dir_prim_shw2;//y det cord direction
  tagger_info.ssm_prim_shw2_z_dir = z_dir_prim_shw2;//z det cord direction
  tagger_info.ssm_prim_shw2_add_daught_track_counts_1 = add_daught_track_counts_1_prim_shw2;//additional daughter tracks longer than 1 cm
  tagger_info.ssm_prim_shw2_add_daught_all_counts_1 = add_daught_all_counts_1_prim_shw2;//additional daughter tracks+showers longer than 1 cm
  tagger_info.ssm_prim_shw2_add_daught_track_counts_5 = add_daught_track_counts_5_prim_shw2;//additional daughter tracks longer than 5 cm
  tagger_info.ssm_prim_shw2_add_daught_all_counts_5 = add_daught_all_counts_5_prim_shw2;//additional daughter tracks+showers longer than 5 cm
  tagger_info.ssm_prim_shw2_add_daught_track_counts_11 = add_daught_track_counts_11_prim_shw2;//additional daughter tracks longer than 11 cm
  tagger_info.ssm_prim_shw2_add_daught_all_counts_11 = add_daught_all_counts_11_prim_shw2;//additional daughter tracks+showers longer than 11 cm
  //properties of leading daughter shower
  tagger_info.ssm_daught_shw1_pdg = pdg_daught_shw1;//initial pdg assigned to the shw
  tagger_info.ssm_daught_shw1_score_mu_fwd = score_mu_fwd_daught_shw1;//ID score normally normally used to choose muons
  tagger_info.ssm_daught_shw1_score_p_fwd = score_p_fwd_daught_shw1;//ID score normally normally used to choose protons
  tagger_info.ssm_daught_shw1_score_e_fwd = score_e_fwd_daught_shw1;//ID score normally normally used to choose e
  tagger_info.ssm_daught_shw1_score_mu_bck = score_mu_bck_daught_shw1;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_daught_shw1_score_p_bck = score_p_bck_daught_shw1;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_daught_shw1_score_e_bck = score_e_bck_daught_shw1;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_daught_shw1_length = length_daught_shw1;//length of shw direct caluclated from point to point
  tagger_info.ssm_daught_shw1_direct_length = direct_length_daught_shw1;//length of shw direct from end to end
  double length_ratio_daught_shw1 = -999;
  if(direct_length_daught_shw1>0 && length_daught_shw1>0) length_ratio_daught_shw1 = direct_length_daught_shw1/length_daught_shw1;
  tagger_info.ssm_daught_shw1_length_ratio = length_ratio_daught_shw1;
  tagger_info.ssm_daught_shw1_max_dev = max_dev_daught_shw1;//furthest distance from shw's daughtary axis along the shw
  tagger_info.ssm_daught_shw1_kine_energy_best  =  kine_energy_best_daught_shw1;//best estimate of the KE of the shower
  tagger_info.ssm_daught_shw1_kine_energy_range = kine_energy_range_daught_shw1;//KE of the shower with range
  tagger_info.ssm_daught_shw1_kine_energy_range_mu = kine_energy_range_mu_daught_shw1;
  tagger_info.ssm_daught_shw1_kine_energy_range_p = kine_energy_range_p_daught_shw1;
  tagger_info.ssm_daught_shw1_kine_energy_range_e = kine_energy_range_e_daught_shw1;
  tagger_info.ssm_daught_shw1_kine_energy_cal = kine_energy_cal_daught_shw1;//KE of the shower with dq/dx
  tagger_info.ssm_daught_shw1_medium_dq_dx = medium_dq_dx_daught_shw1;//medium dq/dx over the entire shw 
  tagger_info.ssm_daught_shw1_x_dir = x_dir_daught_shw1;//x det cord direction
  tagger_info.ssm_daught_shw1_y_dir = y_dir_daught_shw1;//y det cord direction
  tagger_info.ssm_daught_shw1_z_dir = z_dir_daught_shw1;//z det cord direction
  tagger_info.ssm_daught_shw1_add_daught_track_counts_1 = add_daught_track_counts_1_daught_shw1;//additional daughter tracks longer than 1 cm
  tagger_info.ssm_daught_shw1_add_daught_all_counts_1 = add_daught_all_counts_1_daught_shw1;//additional daughter tracks+showers longer than 1 cm
  tagger_info.ssm_daught_shw1_add_daught_track_counts_5 = add_daught_track_counts_5_daught_shw1;//additional daughter tracks longer than 5 cm
  tagger_info.ssm_daught_shw1_add_daught_all_counts_5 = add_daught_all_counts_5_daught_shw1;//additional daughter tracks+showers longer than 5 cm
  tagger_info.ssm_daught_shw1_add_daught_track_counts_11 = add_daught_track_counts_11_daught_shw1;//additional daughter tracks longer than 11 cm
  tagger_info.ssm_daught_shw1_add_daught_all_counts_11 = add_daught_all_counts_11_daught_shw1;//additional daughter tracks+showers longer than 11 cm
  //properties of sub-leading daughter shower
  tagger_info.ssm_daught_shw2_pdg = pdg_daught_shw2;//initial pdg assigned to the shw
  tagger_info.ssm_daught_shw2_score_mu_fwd = score_mu_fwd_daught_shw2;//ID score normally normally used to choose muons
  tagger_info.ssm_daught_shw2_score_p_fwd = score_p_fwd_daught_shw2;//ID score normally normally used to choose protons
  tagger_info.ssm_daught_shw2_score_e_fwd = score_e_fwd_daught_shw2;//ID score normally normally used to choose e
  tagger_info.ssm_daught_shw2_score_mu_bck = score_mu_bck_daught_shw2;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_daught_shw2_score_p_bck = score_p_bck_daught_shw2;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_daught_shw2_score_e_bck = score_e_bck_daught_shw2;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_daught_shw2_length = length_daught_shw2;//length of shw direct caluclated from point to point
  tagger_info.ssm_daught_shw2_direct_length = direct_length_daught_shw2;//length of shw direct from end to end
  double length_ratio_daught_shw2 = -999;
  if(direct_length_daught_shw2>0 && length_daught_shw2>0) length_ratio_daught_shw2 = direct_length_daught_shw2/length_daught_shw2;
  tagger_info.ssm_daught_shw2_length_ratio = length_ratio_daught_shw2;
  tagger_info.ssm_daught_shw2_max_dev = max_dev_daught_shw2;//furthest distance from shw's daughtary axis along the shw
  tagger_info.ssm_daught_shw2_kine_energy_best  =  kine_energy_best_daught_shw2;//best estimate of the KE of the shower
  tagger_info.ssm_daught_shw2_kine_energy_range = kine_energy_range_daught_shw2;//KE of the shower with range
  tagger_info.ssm_daught_shw2_kine_energy_range_mu = kine_energy_range_mu_daught_shw2;
  tagger_info.ssm_daught_shw2_kine_energy_range_p = kine_energy_range_p_daught_shw2;
  tagger_info.ssm_daught_shw2_kine_energy_range_e = kine_energy_range_e_daught_shw2;
  tagger_info.ssm_daught_shw2_kine_energy_cal = kine_energy_cal_daught_shw2;//KE of the shower with dq/dx
  tagger_info.ssm_daught_shw2_medium_dq_dx = medium_dq_dx_daught_shw2;//medium dq/dx over the entire shw
  tagger_info.ssm_daught_shw2_x_dir = x_dir_daught_shw2;//x det cord direction
  tagger_info.ssm_daught_shw2_y_dir = y_dir_daught_shw2;//y det cord direction
  tagger_info.ssm_daught_shw2_z_dir = z_dir_daught_shw2;//z det cord direction
  tagger_info.ssm_daught_shw2_add_daught_track_counts_1 = add_daught_track_counts_1_daught_shw2;//additional daughter tracks longer than 1 cm
  tagger_info.ssm_daught_shw2_add_daught_all_counts_1 = add_daught_all_counts_1_daught_shw2;//additional daughter tracks+showers longer than 1 cm
  tagger_info.ssm_daught_shw2_add_daught_track_counts_5 = add_daught_track_counts_5_daught_shw2;//additional daughter tracks longer than 5 cm
  tagger_info.ssm_daught_shw2_add_daught_all_counts_5 = add_daught_all_counts_5_daught_shw2;//additional daughter tracks+showers longer than 5 cm
  tagger_info.ssm_daught_shw2_add_daught_track_counts_11 = add_daught_track_counts_11_daught_shw2;//additional daughter tracks longer than 11 cm
  tagger_info.ssm_daught_shw2_add_daught_all_counts_11 = add_daught_all_counts_11_daught_shw2;//additional daughter tracks+showers longer than 11 cm
  //event level properties
  tagger_info.ssm_nu_angle_z = nu_angle_to_z;
  tagger_info.ssm_nu_angle_target = nu_angle_to_target;
  tagger_info.ssm_nu_angle_absorber = nu_angle_to_absorber;
  tagger_info.ssm_nu_angle_vertical = nu_angle_to_vertical;
  tagger_info.ssm_con_nu_angle_z = con_nu_angle_to_z;
  tagger_info.ssm_con_nu_angle_target = con_nu_angle_to_target;
  tagger_info.ssm_con_nu_angle_absorber = con_nu_angle_to_absorber;
  tagger_info.ssm_con_nu_angle_vertical = con_nu_angle_to_vertical;
  tagger_info.ssm_prim_nu_angle_z = prim_nu_angle_to_z;
  tagger_info.ssm_prim_nu_angle_target = prim_nu_angle_to_target;
  tagger_info.ssm_prim_nu_angle_absorber = prim_nu_angle_to_absorber;
  tagger_info.ssm_prim_nu_angle_vertical = prim_nu_angle_to_vertical;
  tagger_info.ssm_track_angle_z = track_angle_to_z;
  tagger_info.ssm_track_angle_target = track_angle_to_target;
  tagger_info.ssm_track_angle_absorber = track_angle_to_absorber;
  tagger_info.ssm_track_angle_vertical = track_angle_to_vertical;
  tagger_info.ssm_vtxX = ssm_main_vtx->get_fit_pt().x/units::cm;
  tagger_info.ssm_vtxY = ssm_main_vtx->get_fit_pt().y/units::cm;
  tagger_info.ssm_vtxZ = ssm_main_vtx->get_fit_pt().z/units::cm;

  //off vertex stuff
  tagger_info.ssm_offvtx_length  =  off_vtx_length;//total length of off vertex segments
  tagger_info.ssm_offvtx_energy  =  off_vtx_energy;//total energy of off vertex activity
  tagger_info.ssm_n_offvtx_tracks_1  =  n_offvtx_tracks_1;//number of other daughter tracks greater than 1 cm
  tagger_info.ssm_n_offvtx_tracks_3  =  n_offvtx_tracks_3;//number of other daughter tracks greater than 3 cm
  tagger_info.ssm_n_offvtx_tracks_5  =  n_offvtx_tracks_5;//number of other daughter tracks greater than 5 cm
  tagger_info.ssm_n_offvtx_tracks_8  =  n_offvtx_tracks_8;//number of other daughter tracks greater than 8 cm
  tagger_info.ssm_n_offvtx_tracks_11  =  n_offvtx_tracks_11;//number of other daughter tracks greater than 11 cm
  tagger_info.ssm_n_offvtx_showers_1  =  n_offvtx_showers_1;//number of other daughter tracks and showers greater than 1 cm
  tagger_info.ssm_n_offvtx_showers_3  =  n_offvtx_showers_3;//number of other daughter tracks and showers greater than 3 cm
  tagger_info.ssm_n_offvtx_showers_5  =  n_offvtx_showers_5;//number of other daughter tracks and showers greater than 5 cm
  tagger_info.ssm_n_offvtx_showers_8  =  n_offvtx_showers_8;//number of other daughter tracks and showers greater than 8 cm
  tagger_info.ssm_n_offvtx_showers_11  =  n_offvtx_showers_11;//number of other daughter tracks and showers greater than 11 cm
   //properties of leading off vertex track
  tagger_info.ssm_offvtx_track1_pdg  =  pdg_offvtx_track1;//initial pdg assigned to the track
  tagger_info.ssm_offvtx_track1_score_mu_fwd  =  score_mu_fwd_offvtx_track1;//ID score normally normally used to choose muons
  tagger_info.ssm_offvtx_track1_score_p_fwd  =  score_p_fwd_offvtx_track1;//ID score normally normally used to choose protons
  tagger_info.ssm_offvtx_track1_score_e_fwd  =  score_e_fwd_offvtx_track1;//ID score normally normally used to choose e
  tagger_info.ssm_offvtx_track1_score_mu_bck  =  score_mu_bck_offvtx_track1;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_offvtx_track1_score_p_bck  =  score_p_bck_offvtx_track1;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_offvtx_track1_score_e_bck  =  score_e_bck_offvtx_track1;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_offvtx_track1_length  =  length_offvtx_track1;//length of track direct calculated from point to point
  tagger_info.ssm_offvtx_track1_direct_length  =  direct_length_offvtx_track1;//length of track direct from end to end
  tagger_info.ssm_offvtx_track1_max_dev  =  max_dev_offvtx_track1;//furthest distance from track's primary axis along the track
  tagger_info.ssm_offvtx_track1_kine_energy_range  =  kine_energy_range_offvtx_track1;//KE of the track with range
  tagger_info.ssm_offvtx_track1_kine_energy_range_mu  =  kine_energy_range_mu_offvtx_track1;
  tagger_info.ssm_offvtx_track1_kine_energy_range_p  =  kine_energy_range_p_offvtx_track1;
  tagger_info.ssm_offvtx_track1_kine_energy_range_e  =  kine_energy_range_e_offvtx_track1;
  tagger_info.ssm_offvtx_track1_kine_energy_cal  =  kine_energy_cal_offvtx_track1;//KE of the shower with dq/dx
  tagger_info.ssm_offvtx_track1_medium_dq_dx  =  medium_dq_dx_offvtx_track1;//medium dq/dx over the entire track 
  tagger_info.ssm_offvtx_track1_x_dir  =  x_dir_offvtx_track1;//x dir det cord
  tagger_info.ssm_offvtx_track1_y_dir  =  y_dir_offvtx_track1;//y dir det cord
  tagger_info.ssm_offvtx_track1_z_dir  =  z_dir_offvtx_track1;//z dir det cord
  tagger_info.ssm_offvtx_track1_dist_mainvtx  =  dist_mainvtx_offvtx_track1; //distance from the main vertex
   //properties of leading off vertex shower
  tagger_info.ssm_offvtx_shw1_pdg_offvtx  =  pdg_offvtx_shw1;//initial pdg assigned to the shw
  tagger_info.ssm_offvtx_shw1_score_mu_fwd  =  score_mu_fwd_offvtx_shw1;//ID score normally normally used to choose muons
  tagger_info.ssm_offvtx_shw1_score_p_fwd  =  score_p_fwd_offvtx_shw1;//ID score normally normally used to choose protons
  tagger_info.ssm_offvtx_shw1_score_e_fwd  =  score_e_fwd_offvtx_shw1;//ID score normally normally used to choose e
  tagger_info.ssm_offvtx_shw1_score_mu_bck  =  score_mu_bck_offvtx_shw1;//ID score normally normally used to choose muons but calculated backwards
  tagger_info.ssm_offvtx_shw1_score_p_bck  =  score_p_bck_offvtx_shw1;//ID score normally normally used to choose protons but calculated backwards
  tagger_info.ssm_offvtx_shw1_score_e_bck  =  score_e_bck_offvtx_shw1;//ID score normally normally used to choose e but calculated backwards
  tagger_info.ssm_offvtx_shw1_length  =  length_offvtx_shw1;//length of shw direct calculated from point to point
  tagger_info.ssm_offvtx_shw1_direct_length  =  direct_length_offvtx_shw1;//length of shw direct from end to end
  tagger_info.ssm_offvtx_shw1_max_dev  =  max_dev_offvtx_shw1;//furthest distance from shw's primary axis along the shw
  tagger_info.ssm_offvtx_shw1_kine_energy_best  =  kine_energy_best_offvtx_shw1;//best estimate of the KE of the shower
  tagger_info.ssm_offvtx_shw1_kine_energy_range  =  kine_energy_range_offvtx_shw1;//KE of the shower with range
  tagger_info.ssm_offvtx_shw1_kine_energy_range_mu  =  kine_energy_range_mu_offvtx_shw1;
  tagger_info.ssm_offvtx_shw1_kine_energy_range_p  =  kine_energy_range_p_offvtx_shw1;
  tagger_info.ssm_offvtx_shw1_kine_energy_range_e  =  kine_energy_range_e_offvtx_shw1;
  tagger_info.ssm_offvtx_shw1_kine_energy_cal  =  kine_energy_cal_offvtx_shw1;//KE of the shower with dq/dx
  tagger_info.ssm_offvtx_shw1_medium_dq_dx  =  medium_dq_dx_offvtx_shw1;//medium dq/dx over the entire shw 
  tagger_info.ssm_offvtx_shw1_x_dir  =  x_dir_offvtx_shw1;//x dir det cord
  tagger_info.ssm_offvtx_shw1_y_dir  =  y_dir_offvtx_shw1;//y dir det cord
  tagger_info.ssm_offvtx_shw1_z_dir  =  z_dir_offvtx_shw1;//z dir det cord
  tagger_info.ssm_offvtx_shw1_dist_mainvtx  =  dist_mainvtx_offvtx_shw1; //distance from the main vertex


  print_ssm_tagger();

  return true;
};


std::vector<double> WCPPID::NeutrinoID::get_scores(WCPPID::ProtoSegment *sg){

  std::vector<WCP::Point >& fit_pt_vec =  sg->get_point_vec();
  std::vector<double>& vec_dQ = sg->get_dQ_vec();
  std::vector<double>& vec_dx = sg->get_dx_vec();

  int size = fit_pt_vec.size();

  std::vector<double> L(size,0);
  std::vector<double> dQ_dx(size,0);
  std::vector<double> rL(size,0);
  std::vector<double> rdQ_dx(size,0);

  double dis = 0;
  for (int i = 0; i<size; i++){
    double dq_dx = vec_dQ.at(i)/(vec_dx.at(i)/units::cm+1e-9);
    dQ_dx.at(i) = dq_dx;
    rdQ_dx.at(size-1-i) = dq_dx;
    L.at(i) = dis;
    if (i+1 < size)
      dis += sqrt(pow(fit_pt_vec.at(i+1).x-fit_pt_vec.at(i).x,2) + pow(fit_pt_vec.at(i+1).y-fit_pt_vec.at(i).y,2) + pow(fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z,2));
  }
  // get reverse vector for L
  for (size_t i=0;i!=L.size();i++){
    rL.at(i) = L.back() - L.at(L.size()-1-i);
  }

  std::vector<double> result_forward = sg->do_track_comp(L, dQ_dx, 15*units::cm);
  std::vector<double> result_backward = sg->do_track_comp(rL, rdQ_dx, 15*units::cm);

  std::vector<double> result;

  result.push_back( result_forward.at(1) );
  result.push_back( result_forward.at(2) );
  result.push_back( result_forward.at(3) );
  result.push_back( result_backward.at(1) );
  result.push_back( result_backward.at(2) );
  result.push_back( result_backward.at(3) );

  return result;

}


std::vector<double> WCPPID::NeutrinoID::get_scores(WCPPID::ProtoSegment *sg, int break_point, int dir){

  std::vector<WCP::Point >& fit_pt_vec =  sg->get_point_vec();
  std::vector<double>& vec_dQ = sg->get_dQ_vec();
  std::vector<double>& vec_dx = sg->get_dx_vec();

  int size = fit_pt_vec.size()-break_point;
  int start = break_point;
  if(dir==-1){size=break_point; start=0;}

  std::vector<double> L(size,0);
  std::vector<double> dQ_dx(size,0);
  std::vector<double> rL(size,0);
  std::vector<double> rdQ_dx(size,0);

  std::cout<<std::endl;
  double dis = 0;
    for (int i = start; i<start+size; i++){
      double dq_dx = vec_dQ.at(i)/(vec_dx.at(i)/units::cm+1e-9);
      dQ_dx.at(i-start) = dq_dx;
      rdQ_dx.at(size-1-i+start) = dq_dx;
      L.at(i-start) = dis;
      if (i+1 < size+start)
        dis += sqrt(pow(fit_pt_vec.at(i+1).x-fit_pt_vec.at(i).x,2) + pow(fit_pt_vec.at(i+1).y-fit_pt_vec.at(i).y,2) + pow(fit_pt_vec.at(i+1).z - fit_pt_vec.at(i).z,2));
    }
    for (size_t i=0;i<L.size();i++){
        rL.at(i) = L.back() - L.at(L.size()-1-i);
    }

  std::vector<double> result = sg->do_track_comp(L, dQ_dx, 15*units::cm);
  if(dir==-1){result = sg->do_track_comp(rL, rdQ_dx, 15*units::cm);}

  return result;

}


void WCPPID::NeutrinoID::fill_ssmsp(WCPPID::ProtoSegment* sg, int pdg, int mother, int dir){
     std::vector<WCP::Point >& vec_pts = sg->get_point_vec();
     int temp_num_points = vec_pts.size();

     double length = sg->get_length()/units::cm;
     double residual_range = length;

     std::vector<double>& vec_dQ = sg->get_dQ_vec();
     std::vector<double>& vec_dx = sg->get_dx_vec();

     std::cout<<"vec_dQ.size() "<<vec_dQ.size()<<" vec_dx.size() "<<vec_dQ.size()<<"  vec_pts.size() "<<vec_pts.size()<<std::endl;
     std::cout<<"pdg "<<pdg<<"  mother "<<mother<<"  id "<<sg->get_id()+sg->get_cluster_id()*1000<<std::endl;

     tagger_info.ssmsp_Nsp.push_back(0);

     int containing_shower_id = get_containing_shower_id(sg);
     double containing_shower_ke = -1;
     int containing_shower_flag = -1;
     if(containing_shower_id>0){
       WCPPID::WCShower* containing_shower = get_containing_shower(sg);
       containing_shower_ke = containing_shower->get_kine_best();
       if (containing_shower_ke == 0 ) containing_shower_ke = containing_shower->get_kine_charge();
       containing_shower_flag = containing_shower->get_flag_shower();
     }
     for (int point = 0; point<temp_num_points; point++){

        int temp_point = point;
	if(dir==-1){ temp_point = temp_num_points-point-1; }

      	tagger_info.ssmsp_id.push_back(sg->get_id()+sg->get_cluster_id()*1000);
	tagger_info.ssmsp_pdg.push_back(pdg);
	tagger_info.ssmsp_mother.push_back(mother);

	tagger_info.ssmsp_x.push_back(vec_pts.at(temp_point).x/units::cm);
        tagger_info.ssmsp_y.push_back(vec_pts.at(temp_point).y/units::cm);
	tagger_info.ssmsp_z.push_back(vec_pts.at(temp_point).z/units::cm);

	tagger_info.ssmsp_dQ.push_back(vec_dQ.at(temp_point));
        tagger_info.ssmsp_dx.push_back(vec_dx.at(temp_point)/units::cm);

        tagger_info.ssmsp_KE.push_back( calc_kine_range_pdg(residual_range, pdg)  );

        tagger_info.ssmsp_containing_shower_id.push_back(containing_shower_id);
        tagger_info.ssmsp_containing_shower_ke.push_back(containing_shower_ke);
        tagger_info.ssmsp_containing_shower_flag.push_back(containing_shower_flag);

        std::cout<<"x "<<tagger_info.ssmsp_x.at(tagger_info.ssmsp_Nsp_tot)<<"  y "<<tagger_info.ssmsp_y.at(tagger_info.ssmsp_Nsp_tot)<<"  z "<<tagger_info.ssmsp_z.at(tagger_info.ssmsp_Nsp_tot)<<"  dx "<<tagger_info.ssmsp_dx.at(tagger_info.ssmsp_Nsp_tot)<<"  dq "<<tagger_info.ssmsp_dQ.at(tagger_info.ssmsp_Nsp_tot)<<"  KE "<<tagger_info.ssmsp_KE.at(tagger_info.ssmsp_Nsp_tot)<<"  rr "<<residual_range<<"  in shower "<<tagger_info.ssmsp_containing_shower_id.at(tagger_info.ssmsp_Nsp_tot)<<"  containing_shower_ke "<<containing_shower_ke<<"  containing_shower_flag "<<containing_shower_flag<<std::endl;

        if(point<temp_num_points-1){
	  if(dir==1) residual_range -= sqrt( pow(vec_pts.at(temp_point).x-vec_pts.at(temp_point+1).x,2) + pow(vec_pts.at(temp_point).y-vec_pts.at(temp_point+1).y,2) + pow(vec_pts.at(temp_point).z-vec_pts.at(temp_point+1).z,2))/units::cm;
          else residual_range -= sqrt( pow(vec_pts.at(temp_point).x-vec_pts.at(temp_point-1).x,2) + pow(vec_pts.at(temp_point).y-vec_pts.at(temp_point-1).y,2) + pow(vec_pts.at(temp_point).z-vec_pts.at(temp_point-1).z,2))/units::cm;
          if(residual_range<0) residual_range=0;
	}
        tagger_info.ssmsp_Nsp_tot+=1;
	tagger_info.ssmsp_Nsp.at(tagger_info.ssmsp_Ntrack)+=1;
        std::cout<<"ssmsp_Nsp_tot "<<tagger_info.ssmsp_Nsp_tot<<"  ssmsp_Nsp "<<tagger_info.ssmsp_Nsp.at(tagger_info.ssmsp_Ntrack)<<std::endl;
      }

  tagger_info.ssmsp_Ntrack+=1;
}


int WCPPID::NeutrinoID::fill_ssmsp_psuedo(WCPPID::WCShower* daught_shower, int mother, int acc_id){

     tagger_info.ssmsp_Nsp.push_back(2);

     int pdg = 2112;
     if (fabs(daught_shower->get_particle_type())==11 || fabs(daught_shower->get_particle_type())==22){pdg = 22;}

     int id = daught_shower->get_start_segment()->get_cluster_id()*1000 + acc_id;

     std::cout<<"pdg "<<pdg<<"  mother "<<mother<<"  id "<<id<<std::endl;

     //first point, the vertex we wish to connect to
     tagger_info.ssmsp_id.push_back(id);
     tagger_info.ssmsp_pdg.push_back(pdg);
     tagger_info.ssmsp_mother.push_back(mother);
 
     tagger_info.ssmsp_containing_shower_id.push_back(-1); 
     tagger_info.ssmsp_containing_shower_ke.push_back(-1);
     tagger_info.ssmsp_containing_shower_flag.push_back(-1);

     tagger_info.ssmsp_x.push_back(daught_shower->get_start_vertex().first->get_fit_pt().x/units::cm);
     tagger_info.ssmsp_y.push_back(daught_shower->get_start_vertex().first->get_fit_pt().y/units::cm);
     tagger_info.ssmsp_z.push_back(daught_shower->get_start_vertex().first->get_fit_pt().z/units::cm);
     tagger_info.ssmsp_dQ.push_back(0);
     tagger_info.ssmsp_dx.push_back(0);
     tagger_info.ssmsp_KE.push_back(0);

     std::cout<<"x "<<tagger_info.ssmsp_x.at(tagger_info.ssmsp_Nsp_tot)<<"  y "<<tagger_info.ssmsp_y.at(tagger_info.ssmsp_Nsp_tot)<<"  z "<<tagger_info.ssmsp_z.at(tagger_info.ssmsp_Nsp_tot)<<"  in shower "<<tagger_info.ssmsp_containing_shower_id.at(tagger_info.ssmsp_Nsp_tot)<<std::endl;
     tagger_info.ssmsp_Nsp_tot+=1;

     //second point, the shower we wish to connect to
     tagger_info.ssmsp_id.push_back(id);
     tagger_info.ssmsp_pdg.push_back(pdg);
     tagger_info.ssmsp_mother.push_back(mother);

     tagger_info.ssmsp_containing_shower_id.push_back(-1);
     tagger_info.ssmsp_containing_shower_ke.push_back(-1);
     tagger_info.ssmsp_containing_shower_flag.push_back(-1);
     if(daught_shower->get_start_segment()->get_flag_dir()==1){
       tagger_info.ssmsp_x.push_back(daught_shower->get_start_segment()->get_point_vec().front().x/units::cm);
       tagger_info.ssmsp_y.push_back(daught_shower->get_start_segment()->get_point_vec().front().y/units::cm);
       tagger_info.ssmsp_z.push_back(daught_shower->get_start_segment()->get_point_vec().front().z/units::cm);
     }
     else{
       tagger_info.ssmsp_x.push_back(daught_shower->get_start_segment()->get_point_vec().back().x/units::cm);
       tagger_info.ssmsp_y.push_back(daught_shower->get_start_segment()->get_point_vec().back().y/units::cm);
       tagger_info.ssmsp_z.push_back(daught_shower->get_start_segment()->get_point_vec().back().z/units::cm);
     }
     tagger_info.ssmsp_dQ.push_back(0);
     tagger_info.ssmsp_dx.push_back(0);
     tagger_info.ssmsp_KE.push_back(0);

        std::cout<<"x "<<tagger_info.ssmsp_x.at(tagger_info.ssmsp_Nsp_tot)<<"  y "<<tagger_info.ssmsp_y.at(tagger_info.ssmsp_Nsp_tot)<<"  z "<<tagger_info.ssmsp_z.at(tagger_info.ssmsp_Nsp_tot)<<"  in shower "<<tagger_info.ssmsp_containing_shower_id.at(tagger_info.ssmsp_Nsp_tot)<<std::endl;
     tagger_info.ssmsp_Nsp_tot+=1;

     tagger_info.ssmsp_Ntrack+=1;

     return id;
}


int WCPPID::NeutrinoID::fill_ssmsp_psuedo(WCPPID::WCShower* shower, WCPPID::ProtoSegment* sg, int mother, int acc_id){

     tagger_info.ssmsp_Nsp.push_back(2);
    
     int pdg = 22;
     int id = shower->get_start_segment()->get_cluster_id()*1000 + acc_id;

     std::cout<<"pdg "<<pdg<<"  mother "<<mother<<"  id "<<id<<std::endl;

     //first point, the vertex we wish to connect to
     tagger_info.ssmsp_id.push_back(id);
     tagger_info.ssmsp_pdg.push_back(pdg);
     tagger_info.ssmsp_mother.push_back(mother);

     double containing_shower_ke = shower->get_kine_best();
     if (containing_shower_ke == 0 ) containing_shower_ke = shower->get_kine_charge(); 
     int containing_shower_flag = shower->get_flag_shower();
     tagger_info.ssmsp_containing_shower_id.push_back(shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id());
     tagger_info.ssmsp_containing_shower_ke.push_back(containing_shower_ke);
     tagger_info.ssmsp_containing_shower_flag.push_back(containing_shower_flag);

     if(shower->get_start_segment()->get_flag_dir()==1){
       tagger_info.ssmsp_x.push_back(shower->get_start_segment()->get_point_vec().front().x/units::cm);
       tagger_info.ssmsp_y.push_back(shower->get_start_segment()->get_point_vec().front().y/units::cm);
       tagger_info.ssmsp_z.push_back(shower->get_start_segment()->get_point_vec().front().z/units::cm);
     }
     else{
       tagger_info.ssmsp_x.push_back(shower->get_start_segment()->get_point_vec().back().x/units::cm);
       tagger_info.ssmsp_y.push_back(shower->get_start_segment()->get_point_vec().back().y/units::cm);
       tagger_info.ssmsp_z.push_back(shower->get_start_segment()->get_point_vec().back().z/units::cm);
     }

     tagger_info.ssmsp_dQ.push_back(0);
     tagger_info.ssmsp_dx.push_back(0);
     tagger_info.ssmsp_KE.push_back(0);

     std::cout<<"x "<<tagger_info.ssmsp_x.at(tagger_info.ssmsp_Nsp_tot)<<"  y "<<tagger_info.ssmsp_y.at(tagger_info.ssmsp_Nsp_tot)<<"  z "<<tagger_info.ssmsp_z.at(tagger_info.ssmsp_Nsp_tot)<<"  in shower "<<tagger_info.ssmsp_containing_shower_id.at(tagger_info.ssmsp_Nsp_tot)<<"  containing_shower_ke "<<containing_shower_ke<<"  containing_shower_flag "<<containing_shower_flag<<std::endl;
     tagger_info.ssmsp_Nsp_tot+=1;

     //second point, the shower we wish to connect to
     tagger_info.ssmsp_id.push_back(id);
     tagger_info.ssmsp_pdg.push_back(pdg);
     tagger_info.ssmsp_mother.push_back(mother);

     tagger_info.ssmsp_containing_shower_id.push_back(shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id());
     tagger_info.ssmsp_containing_shower_ke.push_back(containing_shower_ke);
     tagger_info.ssmsp_containing_shower_flag.push_back(containing_shower_flag);

     if(sg->get_flag_dir()==1){
       tagger_info.ssmsp_x.push_back(sg->get_point_vec().front().x/units::cm);
       tagger_info.ssmsp_y.push_back(sg->get_point_vec().front().y/units::cm);
       tagger_info.ssmsp_z.push_back(sg->get_point_vec().front().z/units::cm);
     }
     else{
       tagger_info.ssmsp_x.push_back(sg->get_point_vec().back().x/units::cm);
       tagger_info.ssmsp_y.push_back(sg->get_point_vec().back().y/units::cm);
       tagger_info.ssmsp_z.push_back(sg->get_point_vec().back().z/units::cm);
     }
     tagger_info.ssmsp_dQ.push_back(0);
     tagger_info.ssmsp_dx.push_back(0);
     tagger_info.ssmsp_KE.push_back(0);

     std::cout<<"x "<<tagger_info.ssmsp_x.at(tagger_info.ssmsp_Nsp_tot)<<"  y "<<tagger_info.ssmsp_y.at(tagger_info.ssmsp_Nsp_tot)<<"  z "<<tagger_info.ssmsp_z.at(tagger_info.ssmsp_Nsp_tot)<<"  in shower "<<tagger_info.ssmsp_containing_shower_id.at(tagger_info.ssmsp_Nsp_tot)<<"  containing_shower_ke "<<containing_shower_ke<<"  containing_shower_flag "<<containing_shower_flag<<std::endl;
     tagger_info.ssmsp_Nsp_tot+=1;

     tagger_info.ssmsp_Ntrack+=1;

     return id;
}


//potnetially more correct to look at start and end points of segments?
int WCPPID::NeutrinoID::fill_ssmsp_psuedo(WCPPID::WCShower* daught_shower, WCPPID::ProtoSegment* mother_sg, int acc_id){

     tagger_info.ssmsp_Nsp.push_back(2);

     int pdg = 2112;
     if (fabs(daught_shower->get_particle_type())==11 || fabs(daught_shower->get_particle_type())==22){pdg = 22;}

     int id = daught_shower->get_start_segment()->get_cluster_id()*1000 + acc_id;

     int mother = 0;
     if(mother_sg!=0) mother = mother_sg->get_cluster_id()*1000 + mother_sg->get_id();
     std::cout<<"pdg "<<pdg<<"  mother "<<mother<<"  id "<<id<<std::endl;

     //first point, the vertex we wish to connect to
     tagger_info.ssmsp_id.push_back(id);
     tagger_info.ssmsp_pdg.push_back(pdg);
     tagger_info.ssmsp_mother.push_back(mother);
 
     tagger_info.ssmsp_containing_shower_id.push_back(-1);
     tagger_info.ssmsp_containing_shower_ke.push_back(-1);
     tagger_info.ssmsp_containing_shower_flag.push_back(-1);

     if(mother_sg==0){
       tagger_info.ssmsp_x.push_back(daught_shower->get_start_vertex().first->get_fit_pt().x/units::cm);
       tagger_info.ssmsp_y.push_back(daught_shower->get_start_vertex().first->get_fit_pt().y/units::cm);
       tagger_info.ssmsp_z.push_back(daught_shower->get_start_vertex().first->get_fit_pt().z/units::cm);
     }
     else{
       if(mother_sg->get_flag_dir()==1){
         tagger_info.ssmsp_x.push_back(mother_sg->get_point_vec().back().x/units::cm);
         tagger_info.ssmsp_y.push_back(mother_sg->get_point_vec().back().y/units::cm);
         tagger_info.ssmsp_z.push_back(mother_sg->get_point_vec().back().z/units::cm);
       }
       else{
         tagger_info.ssmsp_x.push_back(mother_sg->get_point_vec().front().x/units::cm);
         tagger_info.ssmsp_y.push_back(mother_sg->get_point_vec().front().y/units::cm);
         tagger_info.ssmsp_z.push_back(mother_sg->get_point_vec().front().z/units::cm);
       }
     }
     tagger_info.ssmsp_dQ.push_back(0);
     tagger_info.ssmsp_dx.push_back(0);
     tagger_info.ssmsp_KE.push_back(0);

     std::cout<<"x "<<tagger_info.ssmsp_x.at(tagger_info.ssmsp_Nsp_tot)<<"  y "<<tagger_info.ssmsp_y.at(tagger_info.ssmsp_Nsp_tot)<<"  z "<<tagger_info.ssmsp_z.at(tagger_info.ssmsp_Nsp_tot)<<"  in shower "<<tagger_info.ssmsp_containing_shower_id.at(tagger_info.ssmsp_Nsp_tot)<<std::endl;
     tagger_info.ssmsp_Nsp_tot+=1;

     //second point, the shower we wish to connect to
     tagger_info.ssmsp_id.push_back(id);
     tagger_info.ssmsp_pdg.push_back(pdg);
     tagger_info.ssmsp_mother.push_back(mother);

     tagger_info.ssmsp_containing_shower_id.push_back(-1);
     tagger_info.ssmsp_containing_shower_ke.push_back(-1);
     tagger_info.ssmsp_containing_shower_flag.push_back(-1);
     if(daught_shower->get_start_segment()->get_flag_dir()==1){
       tagger_info.ssmsp_x.push_back(daught_shower->get_start_segment()->get_point_vec().front().x/units::cm);
       tagger_info.ssmsp_y.push_back(daught_shower->get_start_segment()->get_point_vec().front().y/units::cm);
       tagger_info.ssmsp_z.push_back(daught_shower->get_start_segment()->get_point_vec().front().z/units::cm);
     }
     else{
       tagger_info.ssmsp_x.push_back(daught_shower->get_start_segment()->get_point_vec().back().x/units::cm);
       tagger_info.ssmsp_y.push_back(daught_shower->get_start_segment()->get_point_vec().back().y/units::cm);
       tagger_info.ssmsp_z.push_back(daught_shower->get_start_segment()->get_point_vec().back().z/units::cm);
     }

     tagger_info.ssmsp_dQ.push_back(0);
     tagger_info.ssmsp_dx.push_back(0);
     tagger_info.ssmsp_KE.push_back(0);

     std::cout<<"x "<<tagger_info.ssmsp_x.at(tagger_info.ssmsp_Nsp_tot)<<"  y "<<tagger_info.ssmsp_y.at(tagger_info.ssmsp_Nsp_tot)<<"  z "<<tagger_info.ssmsp_z.at(tagger_info.ssmsp_Nsp_tot)<<"  in shower "<<tagger_info.ssmsp_containing_shower_id.at(tagger_info.ssmsp_Nsp_tot)<<std::endl;
     tagger_info.ssmsp_Nsp_tot+=1;

     tagger_info.ssmsp_Ntrack+=1;

     return id;
}


std::vector<double> WCPPID::NeutrinoID::calc_kine_range_multi_pdg(double length){

  std::vector<double> result;

  double kine_energy_mu = calc_kine_range_pdg(length, 13);
  result.push_back(kine_energy_mu);

  double kine_energy_p = calc_kine_range_pdg(length, 2212);
  result.push_back(kine_energy_p);

  double kine_energy_e = calc_kine_range_pdg(length, 11);
  result.push_back(kine_energy_e);

  return result;
}


double WCPPID::NeutrinoID::calc_kine_range_pdg(double length, int pdg){

  TPCParams& mp = Singleton<TPCParams>::Instance();
  TGraph *g_range = 0;

  if(abs(pdg)==13 || abs(pdg)==211){
    g_range = mp.get_muon_r2ke();
  }
  else if(abs(pdg)==2212){
    g_range = mp.get_proton_r2ke();
  }
  else if(abs(pdg)==11){
    g_range = mp.get_electron_r2ke();
  }
  else{ return 0;}

  return g_range->Eval(length) * units::MeV;

}


int WCPPID::NeutrinoID::get_containing_shower_id(WCPPID::ProtoSegment* seg){
      //check which shower we are in, if any
      int in_shower_id = -1;
      auto this_seg_in_shower = map_segment_in_shower.find(seg);
      if (this_seg_in_shower!=map_segment_in_shower.end()){
        WCPPID::WCShower *shower = this_seg_in_shower->second;
        in_shower_id = shower->get_start_segment()->get_cluster_id()*1000 + shower->get_start_segment()->get_id();
      }
      return in_shower_id;
}


WCPPID::WCShower* WCPPID::NeutrinoID::get_containing_shower(WCPPID::ProtoSegment* seg){
      auto this_seg_in_shower = map_segment_in_shower.find(seg);
      if (this_seg_in_shower!=map_segment_in_shower.end()){
        WCPPID::WCShower *shower = this_seg_in_shower->second;
        return shower;
      }
      return 0;
}


bool WCPPID::NeutrinoID::exit_ssm_tagger(){
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
  tagger_info.ssm_prim_shw1_kine_energy_best = -999;
  tagger_info.ssm_prim_shw1_kine_energy_cal = -999;
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
  tagger_info.ssm_prim_shw2_kine_energy_best = -999;
  tagger_info.ssm_prim_shw2_kine_energy_cal = -999;
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
  tagger_info.ssm_daught_shw1_kine_energy_best = -999;
  tagger_info.ssm_daught_shw1_kine_energy_cal = -999;
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
  tagger_info.ssm_daught_shw2_kine_energy_best = -999;
  tagger_info.ssm_daught_shw2_kine_energy_cal = -999;
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
  tagger_info.ssm_nu_angle_z = -999;
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

  // Fill the spacepoints despite not spotting an ssm
  // This is a bit of a mess still. To do this really well we may need some additional revision for showers
  if(flag_ssmsp>0){
    std::cout<<std::endl;
    std::cout<<"Filling ssm spacepoints"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"Main Vertex "<<main_vertex->get_fit_pt().x/units::cm<<" "<<main_vertex->get_fit_pt().y/units::cm<<" "<<main_vertex->get_fit_pt().z/units::cm<<std::endl;
    // Stuff for bookeeping
    std::set<WCPPID::ProtoVertex* > used_vertices;
    std::set<WCPPID::ProtoSegment* > used_segments;
    std::vector<std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*> > segments_to_be_examined;
    int temp_acc_segment_id = acc_segment_id;

    //Everything connected to the ssm main vertex
    for (auto it = map_vertex_segments[main_vertex].begin(); it != map_vertex_segments[main_vertex].end(); it++){
      fill_ssmsp(*it, (*it)->get_particle_type(), 0, (*it)->get_flag_dir());//main vertex, mother is zero
      used_segments.insert(*it);
      WCPPID::ProtoVertex *other_vertex = find_other_vertex(*it, main_vertex);
      segments_to_be_examined.push_back(std::make_pair(other_vertex, *it));
    }
    used_vertices.insert(main_vertex);

    // now all the daughters
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

          // fill the tree, we will trust the nominal PID for daughters

          fill_ssmsp(curr_sg, curr_sg->get_particle_type(), prev_sg->get_id()+prev_sg->get_cluster_id()*1000, curr_sg->get_flag_dir());

          used_segments.insert(curr_sg);
          WCPPID::ProtoVertex *other_vertex = find_other_vertex(curr_sg, curr_vtx);
          if (used_vertices.find(other_vertex) == used_vertices.end())
            temp_segments.push_back(std::make_pair(other_vertex, curr_sg));
        }
        used_vertices.insert(curr_vtx);
      }
      segments_to_be_examined = temp_segments;
    }

    //check for other showers which may not be in the same cluster
    std::cout<<showers.size()<<" total showers."<<std::endl;
    for (auto it = showers.begin(); it!= showers.end(); it++){
      WCPPID::WCShower *shower = *it;
      WCPPID::ProtoSegment* curr_sg = shower->get_start_segment();
      std::pair<ProtoVertex*, int> pair_vertex = shower->get_start_vertex();
      int temp_mother_id = 0;
      double kine_best = shower->get_kine_best();
      if (kine_best ==0 ) kine_best = shower->get_kine_charge();
      if (pair_vertex.second == 1){ // direct connection
        WCPPID::ProtoSegment* curr_sg = shower->get_start_segment();
        if (used_segments.find(curr_sg)!=used_segments.end()) {std::cout<<"Shower "<<curr_sg->get_id()+curr_sg->get_cluster_id()*1000 <<" already added"<<std::endl; temp_mother_id = 0;}
        else if (pair_vertex.first == main_vertex){
          std::cout<<"pair_vertex.first == main_vertex but sg not in used_segments"<<std::endl;
          used_segments.insert(curr_sg);
          temp_mother_id = 0;
        
        }else{
          WCPPID::ProtoSegment* prev_sg = 0;
          if (map_vertex_in_shower.find(pair_vertex.first) != map_vertex_in_shower.end()){
            prev_sg = map_vertex_in_shower[pair_vertex.first]->get_start_segment();
          }else{
            prev_sg = find_incoming_segment(pair_vertex.first);
          }
          temp_mother_id = prev_sg->get_id()+prev_sg->get_cluster_id()*1000;
          // fill the tree, we will trust the nominal PID for daughters
          fill_ssmsp(curr_sg, shower->get_particle_type(), temp_mother_id, curr_sg->get_flag_dir());
        }
      }else if (pair_vertex.second == 2 || pair_vertex.second == 3){
        if (pair_vertex.first == main_vertex){
          temp_mother_id = fill_ssmsp_psuedo(shower,0,temp_acc_segment_id);//daughter, mother, id
	  temp_acc_segment_id++;
          fill_ssmsp(curr_sg, curr_sg->get_particle_type(), temp_mother_id, curr_sg->get_flag_dir());
        }else{
          WCPPID::ProtoSegment* prev_sg = 0;
          if (map_vertex_in_shower.find(pair_vertex.first) != map_vertex_in_shower.end()){
            prev_sg = map_vertex_in_shower[pair_vertex.first]->get_start_segment();
          }else{
            prev_sg = find_incoming_segment(pair_vertex.first);
          }
          if(prev_sg==0){
            std::cout<<"Missed Prev Seg, Setting to 0 "<<std::endl;
            temp_mother_id = fill_ssmsp_psuedo(shower,0,temp_acc_segment_id);//daughter, mother, id
          }
          else{
            temp_mother_id= fill_ssmsp_psuedo(shower, prev_sg, temp_acc_segment_id);
          }
          // fill the tree, we will trust the nominal PID for daughters
	  temp_acc_segment_id++;
          fill_ssmsp(curr_sg, curr_sg->get_particle_type(), temp_mother_id, curr_sg->get_flag_dir());
        }
      }else{std::cout<<curr_sg->get_id()+curr_sg->get_cluster_id()*1000<<" has pair vertex "<<pair_vertex.second<<std::endl; continue;}//missing case is 4, really far away so do not save
      used_segments.insert(curr_sg);
      // Filling in the other space points?
      std::cout<<std::endl;
      std::cout<<"Checking for other Shower spacepoints in "<<curr_sg->get_id()+curr_sg->get_cluster_id()*1000<<std::endl;
      Map_Proto_Segment_Vertices& tmp_map_seg_vtxs = shower->get_map_seg_vtxs();
      for (auto it = tmp_map_seg_vtxs.begin(); it!= tmp_map_seg_vtxs.end(); it++){
        WCPPID::ProtoSegment *shower_sg = it->first;
        if (used_segments.find(shower_sg)!=used_segments.end()) {std::cout<<"Shower segment "<<shower_sg->get_id()+shower_sg->get_cluster_id()*1000<<" already added"<<std::endl; continue;}
          //put a gamma between here and the start of the shower. Appreoximation for now, could maybe make this better
          std::cout<<"Adding segment and sudo segment"<<std::endl;
          int psuedo_particle_id = fill_ssmsp_psuedo(shower,shower_sg,temp_mother_id,temp_acc_segment_id);
          temp_acc_segment_id++;
          fill_ssmsp(shower_sg, shower_sg->get_particle_type(), psuedo_particle_id, shower_sg->get_flag_dir());
          used_segments.insert(shower_sg);
      }
      std::cout<<"Check completed"<<std::endl;
      std::cout<<std::endl;
    }

  }

	return false;
}


void WCPPID::NeutrinoID::print_ssm_tagger(){
  std::cout<<std::endl;
  std::cout<<"tagger_info.ssm_flag_st_kdar: "<<tagger_info.ssm_flag_st_kdar<<std::endl; //pass cutbased

  std::cout<<"tagger_info.ssm_Nsm: "<<tagger_info.ssm_Nsm<<std::endl;//number of short straight muons
  std::cout<<"tagger_info.ssm_Nsm_wivtx: "<<tagger_info.ssm_Nsm_wivtx<<std::endl;//number of short straight muons with vertex activity

  std::cout<<"tagger_info.ssm_dq_dx_fwd_1: "<<tagger_info.ssm_dq_dx_fwd_1<<std::endl;
  std::cout<<"tagger_info.ssm_dq_dx_fwd_2: "<<tagger_info.ssm_dq_dx_fwd_2<<std::endl;
  std::cout<<"tagger_info.ssm_dq_dx_fwd_3: "<<tagger_info.ssm_dq_dx_fwd_3<<std::endl;
  std::cout<<"tagger_info.ssm_dq_dx_fwd_4: "<<tagger_info.ssm_dq_dx_fwd_4<<std::endl;
  std::cout<<"tagger_info.ssm_dq_dx_fwd_5: "<<tagger_info.ssm_dq_dx_fwd_5<<std::endl;
  std::cout<<"tagger_info.ssm_dq_dx_bck_1: "<<tagger_info.ssm_dq_dx_bck_1<<std::endl;
  std::cout<<"tagger_info.ssm_dq_dx_bck_2: "<<tagger_info.ssm_dq_dx_bck_2<<std::endl;
  std::cout<<"tagger_info.ssm_dq_dx_bck_3: "<<tagger_info.ssm_dq_dx_bck_3<<std::endl;
  std::cout<<"tagger_info.ssm_dq_dx_bck_4: "<<tagger_info.ssm_dq_dx_bck_4<<std::endl;
  std::cout<<"tagger_info.ssm_dq_dx_bck_5: "<<tagger_info.ssm_dq_dx_bck_5<<std::endl;
  std::cout<<"tagger_info.ssm_d_dq_dx_fwd_12: "<<tagger_info.ssm_d_dq_dx_fwd_12<<std::endl;
  std::cout<<"tagger_info.ssm_d_dq_dx_fwd_23: "<<tagger_info.ssm_d_dq_dx_fwd_23<<std::endl;
  std::cout<<"tagger_info.ssm_d_dq_dx_fwd_34: "<<tagger_info.ssm_d_dq_dx_fwd_34<<std::endl;
  std::cout<<"tagger_info.ssm_d_dq_dx_fwd_45: "<<tagger_info.ssm_d_dq_dx_fwd_45<<std::endl;
  std::cout<<"tagger_info.ssm_d_dq_dx_bck_12: "<<tagger_info.ssm_d_dq_dx_bck_12<<std::endl;
  std::cout<<"tagger_info.ssm_d_dq_dx_bck_23: "<<tagger_info.ssm_d_dq_dx_bck_23<<std::endl;
  std::cout<<"tagger_info.ssm_d_dq_dx_bck_34: "<<tagger_info.ssm_d_dq_dx_bck_34<<std::endl;
  std::cout<<"tagger_info.ssm_d_dq_dx_bck_45: "<<tagger_info.ssm_d_dq_dx_bck_45<<std::endl;
  std::cout<<"tagger_info.ssm_max_dq_dx_fwd_3: "<<tagger_info.ssm_max_dq_dx_fwd_3<<std::endl;
  std::cout<<"tagger_info.ssm_max_dq_dx_fwd_5: "<<tagger_info.ssm_max_dq_dx_fwd_5<<std::endl;
  std::cout<<"tagger_info.ssm_max_dq_dx_bck_3: "<<tagger_info.ssm_max_dq_dx_bck_3<<std::endl;
  std::cout<<"tagger_info.ssm_max_dq_dx_bck_5: "<<tagger_info.ssm_max_dq_dx_bck_5<<std::endl;
  std::cout<<"tagger_info.ssm_max_d_dq_dx_fwd_3: "<<tagger_info.ssm_max_d_dq_dx_fwd_3<<std::endl;
  std::cout<<"tagger_info.ssm_max_d_dq_dx_fwd_5: "<<tagger_info.ssm_max_d_dq_dx_fwd_5<<std::endl;
  std::cout<<"tagger_info.ssm_max_d_dq_dx_bck_3: "<<tagger_info.ssm_max_d_dq_dx_bck_3<<std::endl;
  std::cout<<"tagger_info.ssm_max_d_dq_dx_bck_5: "<<tagger_info.ssm_max_d_dq_dx_bck_5<<std::endl;
  std::cout<<"tagger_info.ssm_medium_dq_dx: "<<tagger_info.ssm_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_medium_dq_dx_bp: "<<tagger_info.ssm_medium_dq_dx_bp<<std::endl;
      //angluar info
  std::cout<<"tagger_info.ssm_angle_to_z: "<<tagger_info.ssm_angle_to_z<<std::endl;
  std::cout<<"tagger_info.ssm_angle_to_target: "<<tagger_info.ssm_angle_to_target<<std::endl;
  std::cout<<"tagger_info.ssm_angle_to_absorber: "<<tagger_info.ssm_angle_to_absorber<<std::endl;
  std::cout<<"tagger_info.ssm_angle_to_vertical: "<<tagger_info.ssm_angle_to_vertical<<std::endl;
     //directional info
  std::cout<<"tagger_info.ssm_x_dir: "<<tagger_info.ssm_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_y_dir: "<<tagger_info.ssm_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_z_dir: "<<tagger_info.ssm_z_dir<<std::endl; 
     //energy info
  std::cout<<"tagger_info.ssm_kine_energy: "<<tagger_info.ssm_kine_energy<<std::endl;
  std::cout<<"tagger_info.ssm_kine_energy_reduced: "<<tagger_info.ssm_kine_energy_reduced<<std::endl;
      //general properties
  std::cout<<"tagger_info.ssm_vtx_activity: "<<tagger_info.ssm_vtx_activity<<std::endl;
  std::cout<<"tagger_info.ssm_pdg: "<<tagger_info.ssm_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_dQ_dx_cut: "<<tagger_info.ssm_dQ_dx_cut<<std::endl;
  std::cout<<"tagger_info.ssm_score_mu_fwd: "<<tagger_info.ssm_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_score_p_fwd: "<<tagger_info.ssm_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_score_e_fwd: "<<tagger_info.ssm_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_score_mu_bck: "<<tagger_info.ssm_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_score_p_bck: "<<tagger_info.ssm_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_score_e_bck: "<<tagger_info.ssm_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_score_mu_fwd_bp: "<<tagger_info.ssm_score_mu_fwd_bp<<std::endl;
  std::cout<<"tagger_info.ssm_score_p_fwd_bp: "<<tagger_info.ssm_score_p_fwd_bp<<std::endl;
  std::cout<<"tagger_info.ssm_score_e_fwd_bp: "<<tagger_info.ssm_score_e_fwd_bp<<std::endl;
      //track "straighness"
  std::cout<<"tagger_info.ssm_length: "<<tagger_info.ssm_length<<std::endl;
  std::cout<<"tagger_info.ssm_direct_length: "<<tagger_info.ssm_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_length_ratio: "<<tagger_info.ssm_length_ratio<<std::endl;
  std::cout<<"tagger_info.ssm_max_dev: "<<tagger_info.ssm_max_dev<<std::endl;
    //number of other particles
  std::cout<<"tagger_info.ssm_n_prim_tracks_1: "<<tagger_info.ssm_n_prim_tracks_1<<std::endl;
  std::cout<<"tagger_info.ssm_n_prim_tracks_3: "<<tagger_info.ssm_n_prim_tracks_3<<std::endl;
  std::cout<<"tagger_info.ssm_n_prim_tracks_5: "<<tagger_info.ssm_n_prim_tracks_5<<std::endl;
  std::cout<<"tagger_info.ssm_n_prim_tracks_8: "<<tagger_info.ssm_n_prim_tracks_8<<std::endl;
  std::cout<<"tagger_info.ssm_n_prim_tracks_11: "<<tagger_info.ssm_n_prim_tracks_11<<std::endl;
  std::cout<<"tagger_info.ssm_n_all_tracks_1: "<<tagger_info.ssm_n_all_tracks_1<<std::endl;
  std::cout<<"tagger_info.ssm_n_all_tracks_3: "<<tagger_info.ssm_n_all_tracks_3<<std::endl;
  std::cout<<"tagger_info.ssm_n_all_tracks_5: "<<tagger_info.ssm_n_all_tracks_5<<std::endl;
  std::cout<<"tagger_info.ssm_n_all_tracks_8: "<<tagger_info.ssm_n_all_tracks_8<<std::endl;
  std::cout<<"tagger_info.ssm_n_all_tracks_11: "<<tagger_info.ssm_n_all_tracks_11<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_tracks_1: "<<tagger_info.ssm_n_daughter_tracks_1<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_tracks_3: "<<tagger_info.ssm_n_daughter_tracks_3<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_tracks_5: "<<tagger_info.ssm_n_daughter_tracks_5<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_tracks_8: "<<tagger_info.ssm_n_daughter_tracks_8<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_tracks_11: "<<tagger_info.ssm_n_daughter_tracks_11<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_all_1: "<<tagger_info.ssm_n_daughter_all_1<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_all_3: "<<tagger_info.ssm_n_daughter_all_3<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_all_5: "<<tagger_info.ssm_n_daughter_all_5<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_all_8: "<<tagger_info.ssm_n_daughter_all_8<<std::endl;
  std::cout<<"tagger_info.ssm_n_daughter_all_11: "<<tagger_info.ssm_n_daughter_all_11<<std::endl;
    //properties of leading other primary track
  std::cout<<"tagger_info.ssm_prim_track1_pdg: "<<tagger_info.ssm_prim_track1_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_score_mu_fwd: "<<tagger_info.ssm_prim_track1_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_score_p_fwd: "<<tagger_info.ssm_prim_track1_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_score_e_fwd: "<<tagger_info.ssm_prim_track1_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_score_mu_bck: "<<tagger_info.ssm_prim_track1_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_score_p_bck: "<<tagger_info.ssm_prim_track1_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_score_e_bck: "<<tagger_info.ssm_prim_track1_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_length: "<<tagger_info.ssm_prim_track1_length<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_direct_length: "<<tagger_info.ssm_prim_track1_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_length_ratio: "<<tagger_info.ssm_prim_track1_length_ratio<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_max_dev: "<<tagger_info.ssm_prim_track1_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_kine_energy_range: "<<tagger_info.ssm_prim_track1_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_kine_energy_range_mu: "<<tagger_info.ssm_prim_track1_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_kine_energy_range_p: "<<tagger_info.ssm_prim_track1_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_kine_energy_range_e: "<<tagger_info.ssm_prim_track1_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_kine_energy_cal: "<<tagger_info.ssm_prim_track1_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_medium_dq_dx: "<<tagger_info.ssm_prim_track1_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_x_dir: "<<tagger_info.ssm_prim_track1_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_y_dir: "<<tagger_info.ssm_prim_track1_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_z_dir: "<<tagger_info.ssm_prim_track1_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_add_daught_track_counts_1: "<<tagger_info.ssm_prim_track1_add_daught_track_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_add_daught_all_counts_1: "<<tagger_info.ssm_prim_track1_add_daught_all_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_add_daught_track_counts_5: "<<tagger_info.ssm_prim_track1_add_daught_track_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_add_daught_all_counts_5: "<<tagger_info.ssm_prim_track1_add_daught_all_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_add_daught_track_counts_11: "<<tagger_info.ssm_prim_track1_add_daught_track_counts_11<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track1_add_daught_all_counts_11: "<<tagger_info.ssm_prim_track1_add_daught_all_counts_11<<std::endl;
  //properties of sub-leading other primary track
  std::cout<<"tagger_info.ssm_prim_track2_pdg: "<<tagger_info.ssm_prim_track2_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_score_mu_fwd: "<<tagger_info.ssm_prim_track2_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_score_p_fwd: "<<tagger_info.ssm_prim_track2_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_score_e_fwd: "<<tagger_info.ssm_prim_track2_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_score_mu_bck: "<<tagger_info.ssm_prim_track2_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_score_p_bck: "<<tagger_info.ssm_prim_track2_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_score_e_bck: "<<tagger_info.ssm_prim_track2_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_length: "<<tagger_info.ssm_prim_track2_length<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_direct_length: "<<tagger_info.ssm_prim_track2_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_length_ratio: "<<tagger_info.ssm_prim_track2_length_ratio<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_max_dev: "<<tagger_info.ssm_prim_track2_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_kine_energy_range: "<<tagger_info.ssm_prim_track2_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_kine_energy_range_mu: "<<tagger_info.ssm_prim_track2_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_kine_energy_range_p: "<<tagger_info.ssm_prim_track2_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_kine_energy_range_e: "<<tagger_info.ssm_prim_track2_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_kine_energy_cal: "<<tagger_info.ssm_prim_track2_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_medium_dq_dx: "<<tagger_info.ssm_prim_track2_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_x_dir: "<<tagger_info.ssm_prim_track2_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_y_dir: "<<tagger_info.ssm_prim_track2_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_z_dir: "<<tagger_info.ssm_prim_track2_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_add_daught_track_counts_1: "<<tagger_info.ssm_prim_track2_add_daught_track_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_add_daught_all_counts_1: "<<tagger_info.ssm_prim_track2_add_daught_all_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_add_daught_track_counts_5: "<<tagger_info.ssm_prim_track2_add_daught_track_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_add_daught_all_counts_5: "<<tagger_info.ssm_prim_track2_add_daught_all_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_add_daught_track_counts_11: "<<tagger_info.ssm_prim_track2_add_daught_track_counts_11<<std::endl;
  std::cout<<"tagger_info.ssm_prim_track2_add_daught_all_counts_11: "<<tagger_info.ssm_prim_track2_add_daught_all_counts_11<<std::endl;
  //properties of leading daughter track
  std::cout<<"tagger_info.ssm_daught_track1_pdg: "<<tagger_info.ssm_daught_track1_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_score_mu_fwd: "<<tagger_info.ssm_daught_track1_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_score_p_fwd: "<<tagger_info.ssm_daught_track1_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_score_e_fwd: "<<tagger_info.ssm_daught_track1_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_score_mu_bck: "<<tagger_info.ssm_daught_track1_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_score_p_bck: "<<tagger_info.ssm_daught_track1_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_score_e_bck: "<<tagger_info.ssm_daught_track1_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_length: "<<tagger_info.ssm_daught_track1_length<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_direct_length: "<<tagger_info.ssm_daught_track1_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_length_ratio: "<<tagger_info.ssm_daught_track1_length_ratio<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_max_dev: "<<tagger_info.ssm_daught_track1_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_kine_energy_range: "<<tagger_info.ssm_daught_track1_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_kine_energy_range_mu: "<<tagger_info.ssm_daught_track1_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_kine_energy_range_p: "<<tagger_info.ssm_daught_track1_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_kine_energy_range_e: "<<tagger_info.ssm_daught_track1_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_kine_energy_cal: "<<tagger_info.ssm_daught_track1_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_medium_dq_dx: "<<tagger_info.ssm_daught_track1_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_x_dir: "<<tagger_info.ssm_daught_track1_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_y_dir: "<<tagger_info.ssm_daught_track1_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_z_dir: "<<tagger_info.ssm_daught_track1_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_add_daught_track_counts_1: "<<tagger_info.ssm_daught_track1_add_daught_track_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_add_daught_all_counts_1: "<<tagger_info.ssm_daught_track1_add_daught_all_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_add_daught_track_counts_5: "<<tagger_info.ssm_daught_track1_add_daught_track_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_add_daught_all_counts_5: "<<tagger_info.ssm_daught_track1_add_daught_all_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_add_daught_track_counts_11: "<<tagger_info.ssm_daught_track1_add_daught_track_counts_11<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track1_add_daught_all_counts_11: "<<tagger_info.ssm_daught_track1_add_daught_all_counts_11<<std::endl;
  //properties of sub-leading daughter track
  std::cout<<"tagger_info.ssm_daught_track2_pdg: "<<tagger_info.ssm_daught_track2_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_score_mu_fwd: "<<tagger_info.ssm_daught_track2_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_score_p_fwd: "<<tagger_info.ssm_daught_track2_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_score_e_fwd: "<<tagger_info.ssm_daught_track2_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_score_mu_bck: "<<tagger_info.ssm_daught_track2_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_score_p_bck: "<<tagger_info.ssm_daught_track2_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_score_e_bck: "<<tagger_info.ssm_daught_track2_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_length: "<<tagger_info.ssm_daught_track2_length<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_direct_length: "<<tagger_info.ssm_daught_track2_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_length_ratio: "<<tagger_info.ssm_daught_track2_length_ratio<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_max_dev: "<<tagger_info.ssm_daught_track2_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_kine_energy_range: "<<tagger_info.ssm_daught_track2_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_kine_energy_range_mu: "<<tagger_info.ssm_daught_track2_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_kine_energy_range_p: "<<tagger_info.ssm_daught_track2_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_kine_energy_range_e: "<<tagger_info.ssm_daught_track2_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_kine_energy_cal: "<<tagger_info.ssm_daught_track2_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_medium_dq_dx: "<<tagger_info.ssm_daught_track2_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_x_dir: "<<tagger_info.ssm_daught_track2_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_y_dir: "<<tagger_info.ssm_daught_track2_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_z_dir: "<<tagger_info.ssm_daught_track2_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_add_daught_track_counts_1: "<<tagger_info.ssm_daught_track2_add_daught_track_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_add_daught_all_counts_1: "<<tagger_info.ssm_daught_track2_add_daught_all_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_add_daught_track_counts_5: "<<tagger_info.ssm_daught_track2_add_daught_track_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_add_daught_all_counts_5: "<<tagger_info.ssm_daught_track2_add_daught_all_counts_5<<std::endl; 
  std::cout<<"tagger_info.ssm_daught_track2_add_daught_track_counts_11: "<<tagger_info.ssm_daught_track2_add_daught_track_counts_11<<std::endl;
  std::cout<<"tagger_info.ssm_daught_track2_add_daught_all_counts_11: "<<tagger_info.ssm_daught_track2_add_daught_all_counts_11<<std::endl;
  //properties of leading other primary shower
  std::cout<<"tagger_info.ssm_prim_shw1_pdg: "<<tagger_info.ssm_prim_shw1_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_score_mu_fwd: "<<tagger_info.ssm_prim_shw1_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_score_p_fwd: "<<tagger_info.ssm_prim_shw1_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_score_e_fwd: "<<tagger_info.ssm_prim_shw1_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_score_mu_bck: "<<tagger_info.ssm_prim_shw1_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_score_p_bck: "<<tagger_info.ssm_prim_shw1_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_score_e_bck: "<<tagger_info.ssm_prim_shw1_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_length: "<<tagger_info.ssm_prim_shw1_length<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_direct_length: "<<tagger_info.ssm_prim_shw1_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_length_ratio: "<<tagger_info.ssm_prim_shw1_length_ratio<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_max_dev: "<<tagger_info.ssm_prim_shw1_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_kine_energy_range: "<<tagger_info.ssm_prim_shw1_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_kine_energy_range_mu: "<<tagger_info.ssm_prim_shw1_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_kine_energy_range_p: "<<tagger_info.ssm_prim_shw1_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_kine_energy_range_e: "<<tagger_info.ssm_prim_shw1_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_kine_energy_best: "<<tagger_info.ssm_prim_shw1_kine_energy_best<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_kine_energy_cal: "<<tagger_info.ssm_prim_shw1_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_medium_dq_dx: "<<tagger_info.ssm_prim_shw1_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_x_dir: "<<tagger_info.ssm_prim_shw1_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_y_dir: "<<tagger_info.ssm_prim_shw1_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_z_dir: "<<tagger_info.ssm_prim_shw1_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_add_daught_track_counts_1: "<<tagger_info.ssm_prim_shw1_add_daught_track_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_add_daught_all_counts_1: "<<tagger_info.ssm_prim_shw1_add_daught_all_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_add_daught_track_counts_5: "<<tagger_info.ssm_prim_shw1_add_daught_track_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_add_daught_all_counts_5: "<<tagger_info.ssm_prim_shw1_add_daught_all_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_add_daught_track_counts_11: "<<tagger_info.ssm_prim_shw1_add_daught_track_counts_11<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw1_add_daught_all_counts_11: "<<tagger_info.ssm_prim_shw1_add_daught_all_counts_11<<std::endl;
  //properties of sub-leading other primary shower
  std::cout<<"tagger_info.ssm_prim_shw2_pdg: "<<tagger_info.ssm_prim_shw2_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_score_mu_fwd: "<<tagger_info.ssm_prim_shw2_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_score_p_fwd: "<<tagger_info.ssm_prim_shw2_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_score_e_fwd: "<<tagger_info.ssm_prim_shw2_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_score_mu_bck: "<<tagger_info.ssm_prim_shw2_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_score_p_bck: "<<tagger_info.ssm_prim_shw2_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_score_e_bck: "<<tagger_info.ssm_prim_shw2_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_length: "<<tagger_info.ssm_prim_shw2_length<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_direct_length: "<<tagger_info.ssm_prim_shw2_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_length_ratio: "<<tagger_info.ssm_prim_shw2_length_ratio<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_max_dev: "<<tagger_info.ssm_prim_shw2_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_kine_energy_range: "<<tagger_info.ssm_prim_shw2_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_kine_energy_range_mu: "<<tagger_info.ssm_prim_shw2_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_kine_energy_range_p: "<<tagger_info.ssm_prim_shw2_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_kine_energy_range_e: "<<tagger_info.ssm_prim_shw2_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_kine_energy_best: "<<tagger_info.ssm_prim_shw2_kine_energy_best<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_kine_energy_cal: "<<tagger_info.ssm_prim_shw2_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_medium_dq_dx: "<<tagger_info.ssm_prim_shw2_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_x_dir: "<<tagger_info.ssm_prim_shw2_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_y_dir: "<<tagger_info.ssm_prim_shw2_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_z_dir: "<<tagger_info.ssm_prim_shw2_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_add_daught_track_counts_1: "<<tagger_info.ssm_prim_shw2_add_daught_track_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_add_daught_all_counts_1: "<<tagger_info.ssm_prim_shw2_add_daught_all_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_add_daught_track_counts_5: "<<tagger_info.ssm_prim_shw2_add_daught_track_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_add_daught_all_counts_5: "<<tagger_info.ssm_prim_shw2_add_daught_all_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_add_daught_track_counts_11: "<<tagger_info.ssm_prim_shw2_add_daught_track_counts_11<<std::endl;
  std::cout<<"tagger_info.ssm_prim_shw2_add_daught_all_counts_11: "<<tagger_info.ssm_prim_shw2_add_daught_all_counts_11<<std::endl;
  //properties of leading daughter shower
  std::cout<<"tagger_info.ssm_daught_shw1_pdg: "<<tagger_info.ssm_daught_shw1_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_score_mu_fwd: "<<tagger_info.ssm_daught_shw1_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_score_p_fwd: "<<tagger_info.ssm_daught_shw1_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_score_e_fwd: "<<tagger_info.ssm_daught_shw1_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_score_mu_bck: "<<tagger_info.ssm_daught_shw1_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_score_p_bck: "<<tagger_info.ssm_daught_shw1_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_score_e_bck: "<<tagger_info.ssm_daught_shw1_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_length: "<<tagger_info.ssm_daught_shw1_length<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_direct_length: "<<tagger_info.ssm_daught_shw1_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_length_ratio: "<<tagger_info.ssm_daught_shw1_length_ratio<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_max_dev: "<<tagger_info.ssm_daught_shw1_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_kine_energy_range: "<<tagger_info.ssm_daught_shw1_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_kine_energy_range_mu: "<<tagger_info.ssm_daught_shw1_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_kine_energy_range_p: "<<tagger_info.ssm_daught_shw1_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_kine_energy_range_e: "<<tagger_info.ssm_daught_shw1_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_kine_energy_best: "<<tagger_info.ssm_daught_shw1_kine_energy_best<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_kine_energy_cal: "<<tagger_info.ssm_daught_shw1_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_medium_dq_dx: "<<tagger_info.ssm_daught_shw1_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_x_dir: "<<tagger_info.ssm_daught_shw1_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_y_dir: "<<tagger_info.ssm_daught_shw1_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_z_dir: "<<tagger_info.ssm_daught_shw1_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_add_daught_track_counts_1: "<<tagger_info.ssm_daught_shw1_add_daught_track_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_add_daught_all_counts_1: "<<tagger_info.ssm_daught_shw1_add_daught_all_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_add_daught_track_counts_5: "<<tagger_info.ssm_daught_shw1_add_daught_track_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_add_daught_all_counts_5: "<<tagger_info.ssm_daught_shw1_add_daught_all_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_add_daught_track_counts_11: "<<tagger_info.ssm_daught_shw1_add_daught_track_counts_11<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw1_add_daught_all_counts_11: "<<tagger_info.ssm_daught_shw1_add_daught_all_counts_11<<std::endl;
  //properties of sub-leading daughter shower
  std::cout<<"tagger_info.ssm_daught_shw2_pdg: "<<tagger_info.ssm_daught_shw2_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_score_mu_fwd: "<<tagger_info.ssm_daught_shw2_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_score_p_fwd: "<<tagger_info.ssm_daught_shw2_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_score_e_fwd: "<<tagger_info.ssm_daught_shw2_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_score_mu_bck: "<<tagger_info.ssm_daught_shw2_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_score_p_bck: "<<tagger_info.ssm_daught_shw2_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_score_e_bck: "<<tagger_info.ssm_daught_shw2_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_length: "<<tagger_info.ssm_daught_shw2_length<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_direct_length: "<<tagger_info.ssm_daught_shw2_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_length_ratio: "<<tagger_info.ssm_daught_shw2_length_ratio<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_max_dev: "<<tagger_info.ssm_daught_shw2_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_kine_energy_range: "<<tagger_info.ssm_daught_shw2_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_kine_energy_range_mu: "<<tagger_info.ssm_daught_shw2_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_kine_energy_range_p: "<<tagger_info.ssm_daught_shw2_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_kine_energy_range_e: "<<tagger_info.ssm_daught_shw2_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_kine_energy_cal: "<<tagger_info.ssm_daught_shw2_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_kine_energy_best: "<<tagger_info.ssm_daught_shw2_kine_energy_best<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_medium_dq_dx: "<<tagger_info.ssm_daught_shw2_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_x_dir: "<<tagger_info.ssm_daught_shw2_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_y_dir: "<<tagger_info.ssm_daught_shw2_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_z_dir: "<<tagger_info.ssm_daught_shw2_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_add_daught_track_counts_1: "<<tagger_info.ssm_daught_shw2_add_daught_track_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_add_daught_all_counts_1: "<<tagger_info.ssm_daught_shw2_add_daught_all_counts_1<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_add_daught_track_counts_5: "<<tagger_info.ssm_daught_shw2_add_daught_track_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_add_daught_all_counts_5: "<<tagger_info.ssm_daught_shw2_add_daught_all_counts_5<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_add_daught_track_counts_11: "<<tagger_info.ssm_daught_shw2_add_daught_track_counts_11<<std::endl;
  std::cout<<"tagger_info.ssm_daught_shw2_add_daught_all_counts_11: "<<tagger_info.ssm_daught_shw2_add_daught_all_counts_11<<std::endl;
  //event level properties
  std::cout<<"tagger_info.ssm_nu_angle_z: "<<tagger_info.ssm_nu_angle_z<<std::endl;
  std::cout<<"tagger_info.ssm_nu_angle_target: "<<tagger_info.ssm_nu_angle_target<<std::endl;
  std::cout<<"tagger_info.ssm_nu_angle_absorber: "<<tagger_info.ssm_nu_angle_absorber<<std::endl;
  std::cout<<"tagger_info.ssm_nu_angle_vertical: "<<tagger_info.ssm_nu_angle_vertical<<std::endl;
  std::cout<<"tagger_info.ssm_con_nu_angle_z: "<<tagger_info.ssm_con_nu_angle_z<<std::endl;
  std::cout<<"tagger_info.ssm_con_nu_angle_target: "<<tagger_info.ssm_con_nu_angle_target<<std::endl;
  std::cout<<"tagger_info.ssm_con_nu_angle_absorber: "<<tagger_info.ssm_con_nu_angle_absorber<<std::endl;
  std::cout<<"tagger_info.ssm_con_nu_angle_vertical: "<<tagger_info.ssm_con_nu_angle_vertical<<std::endl;
  std::cout<<"tagger_info.ssm_prim_nu_angle_z: "<<tagger_info.ssm_prim_nu_angle_z<<std::endl;
  std::cout<<"tagger_info.ssm_prim_nu_angle_target: "<<tagger_info.ssm_prim_nu_angle_target<<std::endl;
  std::cout<<"tagger_info.ssm_prim_nu_angle_absorber: "<<tagger_info.ssm_prim_nu_angle_absorber<<std::endl;
  std::cout<<"tagger_info.ssm_prim_nu_angle_vertical: "<<tagger_info.ssm_prim_nu_angle_vertical<<std::endl;
  std::cout<<"tagger_info.ssm_track_angle_z: "<<tagger_info.ssm_track_angle_z<<std::endl;
  std::cout<<"tagger_info.ssm_track_angle_target: "<<tagger_info.ssm_track_angle_target<<std::endl;
  std::cout<<"tagger_info.ssm_track_angle_absorber: "<<tagger_info.ssm_track_angle_absorber<<std::endl;
  std::cout<<"tagger_info.ssm_track_angle_vertical: "<<tagger_info.ssm_track_angle_vertical<<std::endl;
  std::cout<<"tagger_info.ssm_vtxX: "<<tagger_info.ssm_vtxX<<std::endl;
  std::cout<<"tagger_info.ssm_vtxY: "<<tagger_info.ssm_vtxY<<std::endl;
  std::cout<<"tagger_info.ssm_vtxZ: "<<tagger_info.ssm_vtxZ<<std::endl;
  //off vertex stuff
  std::cout<<"tagger_info.ssm_offvtx_length: " <<tagger_info.ssm_offvtx_length<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_energy: " <<tagger_info.ssm_offvtx_energy<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_tracks_1: " <<tagger_info.ssm_n_offvtx_tracks_1<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_tracks_3: " <<tagger_info.ssm_n_offvtx_tracks_3<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_tracks_5: " <<tagger_info.ssm_n_offvtx_tracks_5<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_tracks_8: " <<tagger_info.ssm_n_offvtx_tracks_8<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_tracks_11: " <<tagger_info.ssm_n_offvtx_tracks_11<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_showers_1: " <<tagger_info.ssm_n_offvtx_showers_1<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_showers_3: " <<tagger_info.ssm_n_offvtx_showers_3<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_showers_5: " <<tagger_info.ssm_n_offvtx_showers_5<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_showers_8: " <<tagger_info.ssm_n_offvtx_showers_8<<std::endl;
  std::cout<<"tagger_info.ssm_n_offvtx_showers_11: " <<tagger_info.ssm_n_offvtx_showers_11<<std::endl;
  //properties of leading off vertex track
  std::cout<<"tagger_info.ssm_offvtx_track1_pdg: " <<tagger_info.ssm_offvtx_track1_pdg<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_score_mu_fwd: " <<tagger_info.ssm_offvtx_track1_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_score_p_fwd: " <<tagger_info.ssm_offvtx_track1_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_score_e_fwd: " <<tagger_info.ssm_offvtx_track1_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_score_mu_bck: " <<tagger_info.ssm_offvtx_track1_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_score_p_bck: " <<tagger_info.ssm_offvtx_track1_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_score_e_bck: " <<tagger_info.ssm_offvtx_track1_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_length: " <<tagger_info.ssm_offvtx_track1_length<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_direct_length: " <<tagger_info.ssm_offvtx_track1_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_max_dev: " <<tagger_info.ssm_offvtx_track1_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_kine_energy_range: " <<tagger_info.ssm_offvtx_track1_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_kine_energy_range_mu: " <<tagger_info.ssm_offvtx_track1_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_kine_energy_range_p: " <<tagger_info.ssm_offvtx_track1_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_kine_energy_range_e: " <<tagger_info.ssm_offvtx_track1_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_kine_energy_cal: " <<tagger_info.ssm_offvtx_track1_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_medium_dq_dx: " <<tagger_info.ssm_offvtx_track1_medium_dq_dx<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_x_dir: " <<tagger_info.ssm_offvtx_track1_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_y_dir: " <<tagger_info.ssm_offvtx_track1_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_z_dir: " <<tagger_info.ssm_offvtx_track1_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_track1_dist_mainvtx: " <<tagger_info.ssm_offvtx_track1_dist_mainvtx<<std::endl;
  //properties of leading off vertex shower
  std::cout<<"tagger_info.ssm_offvtx_shw1_pdg_offvtx: " <<tagger_info.ssm_offvtx_shw1_pdg_offvtx<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_score_mu_fwd: " <<tagger_info.ssm_offvtx_shw1_score_mu_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_score_p_fwd: " <<tagger_info.ssm_offvtx_shw1_score_p_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_score_e_fwd: " <<tagger_info.ssm_offvtx_shw1_score_e_fwd<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_score_mu_bck: " <<tagger_info.ssm_offvtx_shw1_score_mu_bck<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_score_p_bck: " <<tagger_info.ssm_offvtx_shw1_score_p_bck<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_score_e_bck: " <<tagger_info.ssm_offvtx_shw1_score_e_bck<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_length: " <<tagger_info.ssm_offvtx_shw1_length<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_direct_length: " <<tagger_info.ssm_offvtx_shw1_direct_length<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_max_dev: " <<tagger_info.ssm_offvtx_shw1_max_dev<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_kine_energy_best: " <<tagger_info.ssm_offvtx_shw1_kine_energy_best<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_kine_energy_range: " <<tagger_info.ssm_offvtx_shw1_kine_energy_range<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_kine_energy_range_mu: " <<tagger_info.ssm_offvtx_shw1_kine_energy_range_mu<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_kine_energy_range_p: " <<tagger_info.ssm_offvtx_shw1_kine_energy_range_p<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_kine_energy_range_e: " <<tagger_info.ssm_offvtx_shw1_kine_energy_range_e<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_kine_energy_cal: " <<tagger_info.ssm_offvtx_shw1_kine_energy_cal<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_medium_dq_dx: " <<tagger_info.ssm_offvtx_shw1_medium_dq_dx<<std::endl; 
  std::cout<<"tagger_info.ssm_offvtx_shw1_x_dir: " <<tagger_info.ssm_offvtx_shw1_x_dir<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_y_dir: " <<tagger_info.ssm_offvtx_shw1_y_dir<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_z_dir: " <<tagger_info.ssm_offvtx_shw1_z_dir<<std::endl;
  std::cout<<"tagger_info.ssm_offvtx_shw1_dist_mainvtx: " <<tagger_info.ssm_offvtx_shw1_dist_mainvtx<<std::endl;


  std::cout<<std::endl;
  std::cout<<std::endl;

        
}
