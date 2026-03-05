#ifndef WIRECELLPID_NEUTRINOID_H
#define WIRECELLPID_NEUTRINOID_H

#include "WCPSst/GeomDataSource.h"

#include "WCPPID/PR3DCluster.h"
#include "WCPData/ToyCTPointCloud.h"

#include "WCPPID/WCShower.h"

#include "WCPPID/ToyFiducial.h"

#include "WCPPID/Map_Proto_Vertex_Segment.h"

#include <set>
#include <string>

//#include "WCPPID/ProtoVertex.h"
//#include "Minuit2/FCNBase.h"

namespace WCPPID{
  struct WCPointTree
  {
    Double_t reco_x;
    Double_t reco_y;
    Double_t reco_z;
    Double_t reco_dQ;
    Double_t reco_dx;
    Double_t reco_chi2;
    Double_t reco_ndf;
    Double_t reco_pu;
    Double_t reco_pv;
    Double_t reco_pw;
    Double_t reco_pt;
    Double_t reco_rr;
    Double_t reco_reduced_chi2;
    Int_t reco_flag_vertex; // vertex or not ...
    Int_t reco_mother_cluster_id; // parent cluster id
    Int_t reco_cluster_id; // current cluster id ...
    Int_t reco_proto_cluster_id; // proto segments ...
    Int_t reco_particle_id; // particle level ...
    Int_t reco_flag_track_shower; // track or shower
    Double_t reco_flag_track_shower_charge; //label charge ...
  };

  struct WCRecoTree
  {
    int mc_Ntrack;  // number of tracks in MC
    int mc_id[1000];  // track id; size == mc_Ntrack
    int mc_pdg[1000];  // track particle pdg; size == mc_Ntrack
    int mc_process[1000];  // track generation process code; size == mc_Ntrack
    int mc_mother[1000];  // mother id of this track; size == mc_Ntrack
    int mc_included[1000]; // whether this particle should be included in the energy calculation ...
    
    int mc_dir_weak[1000]; // weak direction ...
    int mc_stopped[1000]; // if things are stopped
    float mc_length[1000]; // length
    
    float mc_kine_range[1000];
    float mc_kine_dQdx[1000];
    float mc_kine_charge[1000];
    
    
    float mc_startXYZT[1000][4];  // start position of this track; size == mc_Ntrack
    float mc_endXYZT[1000][4];  // end position of this track; size == mc_Ntrack
    float mc_startMomentum[1000][4];  // start momentum of this track; size == mc_Ntrack
    float mc_endMomentum[1000][4];  // end momentum of this track; size == mc_Ntrack
    std::vector<std::vector<int> > *mc_daughters;
  };


  struct KineInfo
  {
    // neutrino vertex ...
    float kine_nu_x_corr;
    float kine_nu_y_corr;
    float kine_nu_z_corr;
    
    // energy ...
    float kine_reco_Enu; // kinetic energy  + additional energy ...
    float kine_reco_add_energy;  // mass, binding energy ...
    std::vector<float> kine_energy_particle;  // energy of each particle
    std::vector<int> kine_energy_info; // what kind of energy reconstruction?
    std::vector<int> kine_particle_type;
    std::vector<int> kine_energy_included; // included in the neutrino energy calculation?

    // pi0 information ...
    float kine_pio_mass; // mass
    int kine_pio_flag; // 0 not filled, 1, with vertex, 2 without vertex
    float kine_pio_vtx_dis;
    
    float kine_pio_energy_1;
    float kine_pio_theta_1;
    float kine_pio_phi_1;
    float kine_pio_dis_1;
    
    float kine_pio_energy_2;
    float kine_pio_theta_2;
    float kine_pio_phi_2;
    float kine_pio_dis_2;
    
    float kine_pio_angle;
    
    
  };
  
  
  //
  struct TaggerInfo
  {
    // cosmic tagger ones, one case of cosmics ...
    float cosmic_flag; // default is true ...
    float cosmic_filled;   // if things are filled
    // variable ...
    float cosmic_n_solid_tracks;    // used
    float cosmic_energy_main_showers; // used
    float cosmic_energy_indirect_showers; // used
    // not used ...
    float cosmic_energy_direct_showers;  
    float cosmic_n_direct_showers;
    float cosmic_n_indirect_showers;
    float cosmic_n_main_showers;


    // shower gap identification
    float gap_flag;
    float gap_filled;
    // main variables:
    float gap_n_bad;
    float gap_n_points;
    float gap_energy;
    float gap_flag_single_shower;
    float gap_flag_parallel;
    // not directly used
    float gap_flag_prolong_u;
    float gap_flag_prolong_v;
    float gap_flag_prolong_w;
    float gap_num_valid_tracks;

    
    // mip_quality
    float mip_quality_flag;
    float mip_quality_filled;
    // variables
    float mip_quality_energy;
    float mip_quality_overlap;
    float mip_quality_n_showers;
    float mip_quality_n_tracks;
    float mip_quality_flag_inside_pi0;
    float mip_quality_n_pi0_showers;
    float mip_quality_shortest_length;
    float mip_quality_acc_length;
    float mip_quality_shortest_angle;
    float mip_quality_flag_proton;
    

    // mip identification
    float mip_flag;
    float mip_filled;
    // variables
    float mip_energy; // checked
    float mip_n_end_reduction;     // checked
    float mip_n_first_mip; // checked
    float mip_n_first_non_mip; // checked
    float mip_n_first_non_mip_1; // checked
    float mip_n_first_non_mip_2; // checked
    float mip_vec_dQ_dx_0; // checked
    float mip_vec_dQ_dx_1; // checked
    float mip_max_dQ_dx_sample; // checked
    float mip_n_below_threshold; // checked
    float mip_n_below_zero;  // checked
    float mip_n_lowest;  // checked
    float mip_n_highest;  // checked
    float mip_lowest_dQ_dx;  // checked
    float mip_highest_dQ_dx;  // checked
    float mip_medium_dQ_dx;  // checked
    float mip_stem_length;  // checked
    float mip_length_main;   // checked
    float mip_length_total; // checked
    float mip_angle_beam;  // checked
    float mip_iso_angle;  // checked
    float mip_n_vertex;   // checked
    float mip_n_good_tracks;  // checked
    float mip_E_indirect_max_energy;  // checked
    float mip_flag_all_above;     // checked
    float mip_min_dQ_dx_5;  // checked
    float mip_n_other_vertex;   // checked
    float mip_n_stem_size;  // checked
    float mip_flag_stem_trajectory;  // checked
    float mip_min_dis;   // checked


    // extra
    float mip_vec_dQ_dx_2;
    float mip_vec_dQ_dx_3;
    float mip_vec_dQ_dx_4;
    float mip_vec_dQ_dx_5;
    float mip_vec_dQ_dx_6;
    float mip_vec_dQ_dx_7;
    float mip_vec_dQ_dx_8;
    float mip_vec_dQ_dx_9;
    float mip_vec_dQ_dx_10;
    float mip_vec_dQ_dx_11;
    float mip_vec_dQ_dx_12;
    float mip_vec_dQ_dx_13;
    float mip_vec_dQ_dx_14;
    float mip_vec_dQ_dx_15;
    float mip_vec_dQ_dx_16;
    float mip_vec_dQ_dx_17;
    float mip_vec_dQ_dx_18;
    float mip_vec_dQ_dx_19;


  float ssm_flag_st_kdar;

  float ssm_Nsm;
  float ssm_Nsm_wivtx;

  float ssm_dq_dx_fwd_1;
  float ssm_dq_dx_fwd_2;
  float ssm_dq_dx_fwd_3;
  float ssm_dq_dx_fwd_4;
  float ssm_dq_dx_fwd_5;
  float ssm_dq_dx_bck_1;
  float ssm_dq_dx_bck_2;
  float ssm_dq_dx_bck_3;
  float ssm_dq_dx_bck_4;
  float ssm_dq_dx_bck_5;
  float ssm_d_dq_dx_fwd_12;
  float ssm_d_dq_dx_fwd_23;
  float ssm_d_dq_dx_fwd_34;
  float ssm_d_dq_dx_fwd_45;
  float ssm_d_dq_dx_bck_12;
  float ssm_d_dq_dx_bck_23;
  float ssm_d_dq_dx_bck_34;
  float ssm_d_dq_dx_bck_45;
  float ssm_max_dq_dx_fwd_3;
  float ssm_max_dq_dx_fwd_5;
  float ssm_max_dq_dx_bck_3;
  float ssm_max_dq_dx_bck_5;
  float ssm_max_d_dq_dx_fwd_3;
  float ssm_max_d_dq_dx_fwd_5;
  float ssm_max_d_dq_dx_bck_3;
  float ssm_max_d_dq_dx_bck_5;
  float ssm_medium_dq_dx;
  float ssm_medium_dq_dx_bp;
      //angluar info
  float ssm_angle_to_z;
  float ssm_angle_to_target;
  float ssm_angle_to_absorber;
  float ssm_angle_to_vertical;
      //directional info
  float ssm_x_dir;
  float ssm_y_dir;
  float ssm_z_dir;
      //energy info
  float ssm_kine_energy;
  float ssm_kine_energy_reduced;
      //general properties
  float ssm_vtx_activity;
  float ssm_pdg;
  float ssm_dQ_dx_cut;
  float ssm_score_mu_fwd;
  float ssm_score_p_fwd;
  float ssm_score_e_fwd;
  float ssm_score_mu_bck;
  float ssm_score_p_bck;
  float ssm_score_e_bck;
  float ssm_score_mu_fwd_bp;
  float ssm_score_p_fwd_bp;
  float ssm_score_e_fwd_bp;
      //track "straighness"
  float ssm_length;
  float ssm_direct_length;
  float ssm_length_ratio;
  float ssm_max_dev;
    //number of other particles
  float ssm_n_prim_tracks_1;
  float ssm_n_prim_tracks_3;
  float ssm_n_prim_tracks_5;
  float ssm_n_prim_tracks_8;
  float ssm_n_prim_tracks_11;
  float ssm_n_all_tracks_1;
  float ssm_n_all_tracks_3;
  float ssm_n_all_tracks_5;
  float ssm_n_all_tracks_8;
  float ssm_n_all_tracks_11;
  float ssm_n_daughter_tracks_1;
  float ssm_n_daughter_tracks_3;
  float ssm_n_daughter_tracks_5;
  float ssm_n_daughter_tracks_8;
  float ssm_n_daughter_tracks_11;
  float ssm_n_daughter_all_1;
  float ssm_n_daughter_all_3;
  float ssm_n_daughter_all_5;
  float ssm_n_daughter_all_8;
  float ssm_n_daughter_all_11;
    //properties of leading other primary track
  float ssm_prim_track1_pdg;
  float ssm_prim_track1_score_mu_fwd;
  float ssm_prim_track1_score_p_fwd;
  float ssm_prim_track1_score_e_fwd;
  float ssm_prim_track1_score_mu_bck;
  float ssm_prim_track1_score_p_bck;
  float ssm_prim_track1_score_e_bck;
  float ssm_prim_track1_length;
  float ssm_prim_track1_direct_length;
  float ssm_prim_track1_length_ratio;
  float ssm_prim_track1_max_dev;
  float ssm_prim_track1_kine_energy_range;
  float ssm_prim_track1_kine_energy_range_mu;
  float ssm_prim_track1_kine_energy_range_p;
  float ssm_prim_track1_kine_energy_range_e;
  float ssm_prim_track1_kine_energy_cal;
  float ssm_prim_track1_medium_dq_dx;
  float ssm_prim_track1_x_dir;
  float ssm_prim_track1_y_dir;
  float ssm_prim_track1_z_dir;
  float ssm_prim_track1_add_daught_track_counts_1;
  float ssm_prim_track1_add_daught_all_counts_1;
  float ssm_prim_track1_add_daught_track_counts_5;
  float ssm_prim_track1_add_daught_all_counts_5;
  float ssm_prim_track1_add_daught_track_counts_11;
  float ssm_prim_track1_add_daught_all_counts_11;
  //properties of sub-leading other primary track
  float ssm_prim_track2_pdg;
  float ssm_prim_track2_score_mu_fwd;
  float ssm_prim_track2_score_p_fwd;
  float ssm_prim_track2_score_e_fwd;
  float ssm_prim_track2_score_mu_bck;
  float ssm_prim_track2_score_p_bck;
  float ssm_prim_track2_score_e_bck;
  float ssm_prim_track2_length;
  float ssm_prim_track2_direct_length;
  float ssm_prim_track2_length_ratio;
  float ssm_prim_track2_max_dev;
  float ssm_prim_track2_kine_energy_range;
  float ssm_prim_track2_kine_energy_range_mu;
  float ssm_prim_track2_kine_energy_range_p;
  float ssm_prim_track2_kine_energy_range_e;
  float ssm_prim_track2_kine_energy_cal;
  float ssm_prim_track2_medium_dq_dx;
  float ssm_prim_track2_x_dir;
  float ssm_prim_track2_y_dir;
  float ssm_prim_track2_z_dir;
  float ssm_prim_track2_add_daught_track_counts_1;
  float ssm_prim_track2_add_daught_all_counts_1;
  float ssm_prim_track2_add_daught_track_counts_5;
  float ssm_prim_track2_add_daught_all_counts_5;
  float ssm_prim_track2_add_daught_track_counts_11;
  float ssm_prim_track2_add_daught_all_counts_11;
  //properties of leading daughter track
  float ssm_daught_track1_pdg;
  float ssm_daught_track1_score_mu_fwd;
  float ssm_daught_track1_score_p_fwd;
  float ssm_daught_track1_score_e_fwd;
  float ssm_daught_track1_score_mu_bck;
  float ssm_daught_track1_score_p_bck;
  float ssm_daught_track1_score_e_bck;
  float ssm_daught_track1_length;
  float ssm_daught_track1_direct_length;
  float ssm_daught_track1_length_ratio;
  float ssm_daught_track1_max_dev;
  float ssm_daught_track1_kine_energy_range;
  float ssm_daught_track1_kine_energy_range_mu;
  float ssm_daught_track1_kine_energy_range_p;
  float ssm_daught_track1_kine_energy_range_e;
  float ssm_daught_track1_kine_energy_cal;
  float ssm_daught_track1_medium_dq_dx;
  float ssm_daught_track1_x_dir;
  float ssm_daught_track1_y_dir;
  float ssm_daught_track1_z_dir;
  float ssm_daught_track1_add_daught_track_counts_1;
  float ssm_daught_track1_add_daught_all_counts_1;
  float ssm_daught_track1_add_daught_track_counts_5;
  float ssm_daught_track1_add_daught_all_counts_5;
  float ssm_daught_track1_add_daught_track_counts_11;
  float ssm_daught_track1_add_daught_all_counts_11;
    //properties of sub-leading daughter track
  float ssm_daught_track2_pdg;
  float ssm_daught_track2_score_mu_fwd;
  float ssm_daught_track2_score_p_fwd;
  float ssm_daught_track2_score_e_fwd;
  float ssm_daught_track2_score_mu_bck;
  float ssm_daught_track2_score_p_bck;
  float ssm_daught_track2_score_e_bck;
  float ssm_daught_track2_length;
  float ssm_daught_track2_direct_length;
  float ssm_daught_track2_length_ratio;
  float ssm_daught_track2_max_dev;
  float ssm_daught_track2_kine_energy_range;
  float ssm_daught_track2_kine_energy_range_mu;
  float ssm_daught_track2_kine_energy_range_p;
  float ssm_daught_track2_kine_energy_range_e;
  float ssm_daught_track2_kine_energy_cal;
  float ssm_daught_track2_medium_dq_dx;
  float ssm_daught_track2_x_dir;
  float ssm_daught_track2_y_dir;
  float ssm_daught_track2_z_dir;
  float ssm_daught_track2_add_daught_track_counts_1;
  float ssm_daught_track2_add_daught_all_counts_1;
  float ssm_daught_track2_add_daught_track_counts_5;
  float ssm_daught_track2_add_daught_all_counts_5;
  float ssm_daught_track2_add_daught_track_counts_11;
  float ssm_daught_track2_add_daught_all_counts_11;
  //properties of leading other primary shower
  float ssm_prim_shw1_pdg;
  float ssm_prim_shw1_score_mu_fwd;
  float ssm_prim_shw1_score_p_fwd;
  float ssm_prim_shw1_score_e_fwd;
  float ssm_prim_shw1_score_mu_bck;
  float ssm_prim_shw1_score_p_bck;
  float ssm_prim_shw1_score_e_bck;
  float ssm_prim_shw1_length;
  float ssm_prim_shw1_direct_length;
  float ssm_prim_shw1_length_ratio;
  float ssm_prim_shw1_max_dev;
  float ssm_prim_shw1_kine_energy_range;
  float ssm_prim_shw1_kine_energy_range_mu;
  float ssm_prim_shw1_kine_energy_range_p;
  float ssm_prim_shw1_kine_energy_range_e;
  float ssm_prim_shw1_kine_energy_cal;
  float ssm_prim_shw1_kine_energy_best;
  float ssm_prim_shw1_medium_dq_dx;
  float ssm_prim_shw1_x_dir;
  float ssm_prim_shw1_y_dir;
  float ssm_prim_shw1_z_dir;
  float ssm_prim_shw1_add_daught_track_counts_1;
  float ssm_prim_shw1_add_daught_all_counts_1;
  float ssm_prim_shw1_add_daught_track_counts_5;
  float ssm_prim_shw1_add_daught_all_counts_5;
  float ssm_prim_shw1_add_daught_track_counts_11;
  float ssm_prim_shw1_add_daught_all_counts_11;
    //properties of sub-leading other primary shower
  float ssm_prim_shw2_pdg;
  float ssm_prim_shw2_score_mu_fwd;
  float ssm_prim_shw2_score_p_fwd;
  float ssm_prim_shw2_score_e_fwd;
  float ssm_prim_shw2_score_mu_bck;
  float ssm_prim_shw2_score_p_bck;
  float ssm_prim_shw2_score_e_bck; 
  float ssm_prim_shw2_length;
  float ssm_prim_shw2_direct_length;
  float ssm_prim_shw2_length_ratio;
  float ssm_prim_shw2_max_dev;
  float ssm_prim_shw2_kine_energy_range;
  float ssm_prim_shw2_kine_energy_range_mu;
  float ssm_prim_shw2_kine_energy_range_p;
  float ssm_prim_shw2_kine_energy_range_e;
  float ssm_prim_shw2_kine_energy_cal;
  float ssm_prim_shw2_kine_energy_best;
  float ssm_prim_shw2_medium_dq_dx;
  float ssm_prim_shw2_x_dir;
  float ssm_prim_shw2_y_dir;
  float ssm_prim_shw2_z_dir;
  float ssm_prim_shw2_add_daught_track_counts_1;
  float ssm_prim_shw2_add_daught_all_counts_1;
  float ssm_prim_shw2_add_daught_track_counts_5;
  float ssm_prim_shw2_add_daught_all_counts_5;
  float ssm_prim_shw2_add_daught_track_counts_11;
  float ssm_prim_shw2_add_daught_all_counts_11;
  //properties of leading daughter shower
  float ssm_daught_shw1_pdg;
  float ssm_daught_shw1_score_mu_fwd;
  float ssm_daught_shw1_score_p_fwd;
  float ssm_daught_shw1_score_e_fwd;
  float ssm_daught_shw1_score_mu_bck;
  float ssm_daught_shw1_score_p_bck;
  float ssm_daught_shw1_score_e_bck;
  float ssm_daught_shw1_length;
  float ssm_daught_shw1_direct_length;
  float ssm_daught_shw1_length_ratio;
  float ssm_daught_shw1_max_dev;
  float ssm_daught_shw1_kine_energy_range;
  float ssm_daught_shw1_kine_energy_range_mu;
  float ssm_daught_shw1_kine_energy_range_p;
  float ssm_daught_shw1_kine_energy_range_e;
  float ssm_daught_shw1_kine_energy_cal;
  float ssm_daught_shw1_kine_energy_best;
  float ssm_daught_shw1_medium_dq_dx;
  float ssm_daught_shw1_x_dir;
  float ssm_daught_shw1_y_dir;
  float ssm_daught_shw1_z_dir;
  float ssm_daught_shw1_add_daught_track_counts_1;
  float ssm_daught_shw1_add_daught_all_counts_1;
  float ssm_daught_shw1_add_daught_track_counts_5;
  float ssm_daught_shw1_add_daught_all_counts_5;
  float ssm_daught_shw1_add_daught_track_counts_11;
  float ssm_daught_shw1_add_daught_all_counts_11;
  //properties of sub-leading daughter shower
  float ssm_daught_shw2_pdg;
  float ssm_daught_shw2_score_mu_fwd;
  float ssm_daught_shw2_score_p_fwd;
  float ssm_daught_shw2_score_e_fwd;
  float ssm_daught_shw2_score_mu_bck;
  float ssm_daught_shw2_score_p_bck;
  float ssm_daught_shw2_score_e_bck;
  float ssm_daught_shw2_length;
  float ssm_daught_shw2_direct_length;
  float ssm_daught_shw2_length_ratio;
  float ssm_daught_shw2_max_dev;
  float ssm_daught_shw2_kine_energy_range;
  float ssm_daught_shw2_kine_energy_range_mu;
  float ssm_daught_shw2_kine_energy_range_p;
  float ssm_daught_shw2_kine_energy_range_e;
  float ssm_daught_shw2_kine_energy_cal;
  float ssm_daught_shw2_kine_energy_best;
  float ssm_daught_shw2_medium_dq_dx;
  float ssm_daught_shw2_x_dir;
  float ssm_daught_shw2_y_dir;
  float ssm_daught_shw2_z_dir;
  float ssm_daught_shw2_add_daught_track_counts_1;
  float ssm_daught_shw2_add_daught_all_counts_1;
  float ssm_daught_shw2_add_daught_track_counts_5;
  float ssm_daught_shw2_add_daught_all_counts_5;
  float ssm_daught_shw2_add_daught_track_counts_11;
  float ssm_daught_shw2_add_daught_all_counts_11;
    //event level properties
  float ssm_nu_angle_z;
  float ssm_nu_angle_target;
  float ssm_nu_angle_absorber;
  float ssm_nu_angle_vertical;
  float ssm_con_nu_angle_z;
  float ssm_con_nu_angle_target;
  float ssm_con_nu_angle_absorber;
  float ssm_con_nu_angle_vertical;
  float ssm_prim_nu_angle_z;
  float ssm_prim_nu_angle_target;
  float ssm_prim_nu_angle_absorber;
  float ssm_prim_nu_angle_vertical;
  float ssm_track_angle_z;
  float ssm_track_angle_target;
  float ssm_track_angle_absorber;
  float ssm_track_angle_vertical;
  float ssm_vtxX;
  float ssm_vtxY;
  float ssm_vtxZ;
    //off vertex stuff
  float ssm_offvtx_length;
  float ssm_offvtx_energy;
  float ssm_n_offvtx_tracks_1;
  float ssm_n_offvtx_tracks_3;
  float ssm_n_offvtx_tracks_5;
  float ssm_n_offvtx_tracks_8;
  float ssm_n_offvtx_tracks_11;
  float ssm_n_offvtx_showers_1;
  float ssm_n_offvtx_showers_3;
  float ssm_n_offvtx_showers_5;
  float ssm_n_offvtx_showers_8;
  float ssm_n_offvtx_showers_11;
   //properties of leading off vertex track
  float ssm_offvtx_track1_pdg;
  float ssm_offvtx_track1_score_mu_fwd;
  float ssm_offvtx_track1_score_p_fwd;
  float ssm_offvtx_track1_score_e_fwd;
  float ssm_offvtx_track1_score_mu_bck;
  float ssm_offvtx_track1_score_p_bck;
  float ssm_offvtx_track1_score_e_bck;
  float ssm_offvtx_track1_length;
  float ssm_offvtx_track1_direct_length;
  float ssm_offvtx_track1_max_dev;
  float ssm_offvtx_track1_kine_energy_range;
  float ssm_offvtx_track1_kine_energy_range_mu;
  float ssm_offvtx_track1_kine_energy_range_p;
  float ssm_offvtx_track1_kine_energy_range_e;
  float ssm_offvtx_track1_kine_energy_cal;
  float ssm_offvtx_track1_medium_dq_dx;
  float ssm_offvtx_track1_x_dir;
  float ssm_offvtx_track1_y_dir;
  float ssm_offvtx_track1_z_dir;
  float ssm_offvtx_track1_dist_mainvtx;
   //properties of leading off vertex shower
  float ssm_offvtx_shw1_pdg_offvtx;
  float ssm_offvtx_shw1_score_mu_fwd;
  float ssm_offvtx_shw1_score_p_fwd;
  float ssm_offvtx_shw1_score_e_fwd;
  float ssm_offvtx_shw1_score_mu_bck;
  float ssm_offvtx_shw1_score_p_bck;
  float ssm_offvtx_shw1_score_e_bck;
  float ssm_offvtx_shw1_length;
  float ssm_offvtx_shw1_direct_length;
  float ssm_offvtx_shw1_max_dev;
  float ssm_offvtx_shw1_kine_energy_best;
  float ssm_offvtx_shw1_kine_energy_range;
  float ssm_offvtx_shw1_kine_energy_range_mu;
  float ssm_offvtx_shw1_kine_energy_range_p;
  float ssm_offvtx_shw1_kine_energy_range_e;
  float ssm_offvtx_shw1_kine_energy_cal;
  float ssm_offvtx_shw1_medium_dq_dx;
  float ssm_offvtx_shw1_x_dir;
  float ssm_offvtx_shw1_y_dir;
  float ssm_offvtx_shw1_z_dir;
  float ssm_offvtx_shw1_dist_mainvtx;

  // Spacepoints
  int ssmsp_Ntrack;
  std::vector<int> ssmsp_Nsp;
  int ssmsp_Nsp_tot;
  std::vector<int> ssmsp_pdg;
  std::vector<int> ssmsp_id;
  std::vector<int> ssmsp_mother;
  std::vector<float> ssmsp_x;
  std::vector<float> ssmsp_y;
  std::vector<float> ssmsp_z;
  std::vector<float> ssmsp_dx;
  std::vector<float> ssmsp_dQ;
  std::vector<float> ssmsp_KE;
  std::vector<float> ssmsp_containing_shower_id;
  std::vector<float> ssmsp_containing_shower_ke;
  std::vector<float> ssmsp_containing_shower_flag;


  //Kine vars
  float ssm_kine_reco_Enu; // kinetic energy  + additional energy ...
  float ssm_kine_reco_add_energy;  // mass, binding energy ...
  std::vector<float> ssm_kine_energy_particle;  // energy of each particle
  std::vector<int> ssm_kine_energy_info; // what kind of energy reconstruction?
  std::vector<int> ssm_kine_particle_type;
  std::vector<int> ssm_kine_energy_included; // included in the neutrino energy calculation?
  float ssm_kine_pio_mass; // mass
  int ssm_kine_pio_flag; // 0 not filled, 1, with vertex, 2 without vertex
  float ssm_kine_pio_vtx_dis;
  float ssm_kine_pio_energy_1;
  float ssm_kine_pio_theta_1;
  float ssm_kine_pio_phi_1;
  float ssm_kine_pio_dis_1;
  float ssm_kine_pio_energy_2;
  float ssm_kine_pio_theta_2;
  float ssm_kine_pio_phi_2;
  float ssm_kine_pio_dis_2;
  float ssm_kine_pio_angle;

    //single photon shower id
    float shw_sp_flag;
    float shw_sp_filled;
    // variables
    float shw_sp_num_mip_tracks;
    float shw_sp_num_muons;
    float shw_sp_num_pions;
    float shw_sp_num_protons;
    float shw_sp_proton_length_1;
    float shw_sp_proton_dqdx_1;
    float shw_sp_proton_energy_1;
    float shw_sp_proton_length_2;
    float shw_sp_proton_dqdx_2;
    float shw_sp_proton_energy_2;
    float shw_sp_n_good_showers;
    float shw_sp_n_20mev_showers;
    float shw_sp_n_br1_showers;
    float shw_sp_n_br2_showers;
    float shw_sp_n_br3_showers;
    float shw_sp_n_br4_showers;
    float shw_sp_n_20br1_showers;
    std::vector<int> shw_sp_20mev_showers;
    std::vector<int> shw_sp_br1_showers;
    std::vector<int> shw_sp_br2_showers;
    std::vector<int> shw_sp_br3_showers;
    std::vector<int> shw_sp_br4_showers;
    float shw_sp_shw_vtx_dis;
    float shw_sp_max_shw_dis;
    float shw_sp_energy;
    //float shw_sp_n_end_reduction;
    //float shw_sp_n_first_mip;
    //float shw_sp_n_first_non_mip;
    //float shw_sp_n_first_non_mip_1;
    //float shw_sp_n_first_non_mip_2;
    float shw_sp_vec_dQ_dx_0;
    float shw_sp_vec_dQ_dx_1;
    float shw_sp_max_dQ_dx_sample;
    float shw_sp_n_below_threshold;
    float shw_sp_n_below_zero;
    float shw_sp_n_lowest;
    float shw_sp_n_highest;
    float shw_sp_lowest_dQ_dx;
    float shw_sp_highest_dQ_dx;
    float shw_sp_medium_dQ_dx;
    float shw_sp_stem_length;
    float shw_sp_length_main;
    float shw_sp_length_total;
    float shw_sp_angle_beam;
    float shw_sp_iso_angle;
    float shw_sp_n_vertex;
    float shw_sp_n_good_tracks;
    float shw_sp_E_indirect_max_energy;
    float shw_sp_flag_all_above;
    float shw_sp_min_dQ_dx_5;
    float shw_sp_n_other_vertex;
    float shw_sp_n_stem_size;
    float shw_sp_flag_stem_trajectory;
    float shw_sp_min_dis;


    // extra
    float shw_sp_vec_dQ_dx_2;
    float shw_sp_vec_dQ_dx_3;
    float shw_sp_vec_dQ_dx_4;
    float shw_sp_vec_dQ_dx_5;
    float shw_sp_vec_dQ_dx_6;
    float shw_sp_vec_dQ_dx_7;
    float shw_sp_vec_dQ_dx_8;
    float shw_sp_vec_dQ_dx_9;
    float shw_sp_vec_dQ_dx_10;
    float shw_sp_vec_dQ_dx_11;
    float shw_sp_vec_dQ_dx_12;
    float shw_sp_vec_dQ_dx_13;
    float shw_sp_vec_dQ_dx_14;
    float shw_sp_vec_dQ_dx_15;
    float shw_sp_vec_dQ_dx_16;
    float shw_sp_vec_dQ_dx_17;
    float shw_sp_vec_dQ_dx_18;
    float shw_sp_vec_dQ_dx_19;
    float shw_sp_vec_median_dedx;
    float shw_sp_vec_mean_dedx;

    // photon shower pi0 identification
    float shw_sp_pio_flag;
    float shw_sp_pio_mip_id;    // pre-condition to run this tagger
    float shw_sp_pio_filled;
    float shw_sp_pio_flag_pio;  // only when this is true, we run the first part of tagger ...
    // first part of tagger ...
    float shw_sp_pio_1_flag;
    float shw_sp_pio_1_mass;
    float shw_sp_pio_1_pio_type;
    float shw_sp_pio_1_energy_1;
    float shw_sp_pio_1_energy_2;
    float shw_sp_pio_1_dis_1;
    float shw_sp_pio_1_dis_2;
    // second part of tagger
    std::vector<float> shw_sp_pio_2_v_dis2;
    std::vector<float> shw_sp_pio_2_v_angle2;
    std::vector<float> shw_sp_pio_2_v_acc_length;
    std::vector<float> shw_sp_pio_2_v_flag;

    // single photon low-energy michel
    float shw_sp_lem_shower_total_length;
    float shw_sp_lem_shower_main_length;
    float shw_sp_lem_n_3seg;
    float shw_sp_lem_e_charge;
    float shw_sp_lem_e_dQdx;
    float shw_sp_lem_shower_num_segs;
    float shw_sp_lem_shower_num_main_segs;
    float shw_sp_lem_flag;

    // bad reconstruction_1
    float shw_sp_br_filled;
    float shw_sp_br1_flag;

    //bad reconstruction 1_1
    float shw_sp_br1_1_flag;
    float shw_sp_br1_1_shower_type;
    float shw_sp_br1_1_vtx_n_segs;
    float shw_sp_br1_1_energy;
    float shw_sp_br1_1_n_segs;
    float shw_sp_br1_1_flag_sg_topology;
    float shw_sp_br1_1_flag_sg_trajectory;
    float shw_sp_br1_1_sg_length;

    // bad reconstruction 1_2
    float shw_sp_br1_2_flag;
    float shw_sp_br1_2_energy;
    float shw_sp_br1_2_n_connected;
    float shw_sp_br1_2_max_length;
    float shw_sp_br1_2_n_connected_1;
    float shw_sp_br1_2_vtx_n_segs;
    float shw_sp_br1_2_n_shower_segs;
    float shw_sp_br1_2_max_length_ratio;
    float shw_sp_br1_2_shower_length;

    // bad_reconstruction 1_3
    float shw_sp_br1_3_flag;
    float shw_sp_br1_3_energy;
    float shw_sp_br1_3_n_connected_p;
    float shw_sp_br1_3_max_length_p;
    float shw_sp_br1_3_n_shower_segs;
    float shw_sp_br1_3_flag_sg_topology;
    float shw_sp_br1_3_flag_sg_trajectory;
    float shw_sp_br1_3_n_shower_main_segs;
    float shw_sp_br1_3_sg_length;


    // bad reconstruction 2
    float shw_sp_br2_flag;
    float shw_sp_br2_flag_single_shower;
    float shw_sp_br2_num_valid_tracks;
    float shw_sp_br2_energy;
    float shw_sp_br2_angle1;
    float shw_sp_br2_angle2;
    float shw_sp_br2_angle;
    float shw_sp_br2_angle3;
    float shw_sp_br2_n_shower_main_segs;
    float shw_sp_br2_max_angle;
    float shw_sp_br2_sg_length;
    float shw_sp_br2_flag_sg_trajectory;


     //bad reconstruction 3
    float shw_sp_br3_1_energy;
    float shw_sp_br3_1_n_shower_segments;
    float shw_sp_br3_1_sg_flag_trajectory;
    float shw_sp_br3_1_sg_direct_length;
    float shw_sp_br3_1_sg_length;
    float shw_sp_br3_1_total_main_length;
    float shw_sp_br3_1_total_length;
    float shw_sp_br3_1_iso_angle;
    float shw_sp_br3_1_sg_flag_topology;
    float shw_sp_br3_1_flag;

    float shw_sp_br3_2_n_ele;
    float shw_sp_br3_2_n_other;
    float shw_sp_br3_2_energy;
    float shw_sp_br3_2_total_main_length;
    float shw_sp_br3_2_total_length;
    float shw_sp_br3_2_other_fid;
    float shw_sp_br3_2_flag;

    std::vector<float> shw_sp_br3_3_v_energy;
    std::vector<float> shw_sp_br3_3_v_angle;
    std::vector<float> shw_sp_br3_3_v_dir_length;
    std::vector<float> shw_sp_br3_3_v_length;
    std::vector<float> shw_sp_br3_3_v_flag;

    float shw_sp_br3_4_acc_length;
    float shw_sp_br3_4_total_length;
    float shw_sp_br3_4_energy;
    float shw_sp_br3_4_flag;

    std::vector<float> shw_sp_br3_5_v_dir_length;
    std::vector<float> shw_sp_br3_5_v_total_length;
    std::vector<float> shw_sp_br3_5_v_flag_avoid_muon_check;
    std::vector<float> shw_sp_br3_5_v_n_seg;
    std::vector<float> shw_sp_br3_5_v_angle;
    std::vector<float> shw_sp_br3_5_v_sg_length;
    std::vector<float> shw_sp_br3_5_v_energy;
    std::vector<float> shw_sp_br3_5_v_n_main_segs;
    std::vector<float> shw_sp_br3_5_v_n_segs;
    std::vector<float> shw_sp_br3_5_v_shower_main_length;
    std::vector<float> shw_sp_br3_5_v_shower_total_length;
    std::vector<float> shw_sp_br3_5_v_flag;

    std::vector<float> shw_sp_br3_6_v_angle;
    std::vector<float> shw_sp_br3_6_v_angle1;
    std::vector<float> shw_sp_br3_6_v_flag_shower_trajectory;
    std::vector<float> shw_sp_br3_6_v_direct_length;
    std::vector<float> shw_sp_br3_6_v_length;
    std::vector<float> shw_sp_br3_6_v_n_other_vtx_segs;
    std::vector<float> shw_sp_br3_6_v_energy;
    std::vector<float> shw_sp_br3_6_v_flag;

    float shw_sp_br3_7_energy;
    float shw_sp_br3_7_min_angle;
    float shw_sp_br3_7_sg_length;
    float shw_sp_br3_7_shower_main_length;
    float shw_sp_br3_7_flag;

    float shw_sp_br3_8_max_dQ_dx;
    float shw_sp_br3_8_energy;
    float shw_sp_br3_8_n_main_segs;
    float shw_sp_br3_8_shower_main_length;
    float shw_sp_br3_8_shower_length;
    float shw_sp_br3_8_flag;

    float shw_sp_br3_flag;

    // BR 4
    float shw_sp_br4_1_shower_main_length;
    float shw_sp_br4_1_shower_total_length;
    float shw_sp_br4_1_min_dis;
    float shw_sp_br4_1_energy;
    float shw_sp_br4_1_flag_avoid_muon_check;
    float shw_sp_br4_1_n_vtx_segs;
    float shw_sp_br4_1_n_main_segs;
    float shw_sp_br4_1_flag;

    float shw_sp_br4_2_ratio_45;
    float shw_sp_br4_2_ratio_35;
    float shw_sp_br4_2_ratio_25;
    float shw_sp_br4_2_ratio_15;
    float shw_sp_br4_2_energy;
    float shw_sp_br4_2_ratio1_45;
    float shw_sp_br4_2_ratio1_35;
    float shw_sp_br4_2_ratio1_25;
    float shw_sp_br4_2_ratio1_15;
    float shw_sp_br4_2_iso_angle;
    float shw_sp_br4_2_iso_angle1;
    float shw_sp_br4_2_angle;
    float shw_sp_br4_2_flag;

    float shw_sp_br4_flag;

    // high energy overlap
    float shw_sp_hol_1_n_valid_tracks;
    float shw_sp_hol_1_min_angle;
    float shw_sp_hol_1_energy;
    float shw_sp_hol_1_flag_all_shower;
    float shw_sp_hol_1_min_length;
    float shw_sp_hol_1_flag;

    float shw_sp_hol_2_min_angle;
    float shw_sp_hol_2_medium_dQ_dx;
    float shw_sp_hol_2_ncount;
    float shw_sp_hol_2_energy;
    float shw_sp_hol_2_flag;

    float shw_sp_hol_flag;

    // low-energy overlap ...
    float shw_sp_lol_flag;

    std::vector<float> shw_sp_lol_1_v_energy;
    std::vector<float> shw_sp_lol_1_v_vtx_n_segs;
    std::vector<float> shw_sp_lol_1_v_nseg;
    std::vector<float> shw_sp_lol_1_v_angle;
    std::vector<float> shw_sp_lol_1_v_flag;

    std::vector<float> shw_sp_lol_2_v_length;
    std::vector<float> shw_sp_lol_2_v_angle;
    std::vector<float> shw_sp_lol_2_v_type;
    std::vector<float> shw_sp_lol_2_v_vtx_n_segs;
    std::vector<float> shw_sp_lol_2_v_energy;
    std::vector<float> shw_sp_lol_2_v_shower_main_length;
    std::vector<float> shw_sp_lol_2_v_flag_dir_weak;
    std::vector<float> shw_sp_lol_2_v_flag;

    float shw_sp_lol_3_angle_beam;
    float shw_sp_lol_3_n_valid_tracks;
    float shw_sp_lol_3_min_angle;
    float shw_sp_lol_3_vtx_n_segs;
    float shw_sp_lol_3_energy;
    float shw_sp_lol_3_shower_main_length;
    float shw_sp_lol_3_n_out;
    float shw_sp_lol_3_n_sum;
    float shw_sp_lol_3_flag;
    
    // shower pi0 identification
    float pio_flag;
    float pio_mip_id;    // pre-condition to run this tagger
    float pio_filled;
    float pio_flag_pio;  // only when this is true, we run the first part of tagger ...
    // first part of tagger ...
    float pio_1_flag;
    float pio_1_mass;
    float pio_1_pio_type;
    float pio_1_energy_1;
    float pio_1_energy_2;
    float pio_1_dis_1;
    float pio_1_dis_2;
    // second part of tagger
    std::vector<float> pio_2_v_dis2;
    std::vector<float> pio_2_v_angle2;
    std::vector<float> pio_2_v_acc_length;
    std::vector<float> pio_2_v_flag;

    
    //single shower pi0 case ...
    std::vector<float> sig_1_v_angle;
    std::vector<float> sig_1_v_flag_single_shower;
    std::vector<float> sig_1_v_energy;
    std::vector<float> sig_1_v_energy_1;
    std::vector<float> sig_1_v_flag;

    std::vector<float> sig_2_v_energy;
    std::vector<float> sig_2_v_shower_angle;
    std::vector<float> sig_2_v_flag_single_shower;
    std::vector<float> sig_2_v_medium_dQ_dx;
    std::vector<float> sig_2_v_start_dQ_dx;
    std::vector<float> sig_2_v_flag;

    float sig_flag;

     // multiple gamma I
    float mgo_energy;
    float mgo_max_energy;
    float mgo_total_energy;
    float mgo_n_showers;
    float mgo_max_energy_1;
    float mgo_max_energy_2;
    float mgo_total_other_energy;
    float mgo_n_total_showers;
    float mgo_total_other_energy_1;
    float mgo_flag;
    
    // multiple gamma II
    float mgt_flag_single_shower;
    float mgt_max_energy;
    float mgt_energy;
    float mgt_total_other_energy;
    float mgt_max_energy_1;
    float mgt_e_indirect_max_energy;
    float mgt_e_direct_max_energy;
    float mgt_n_direct_showers;
    float mgt_e_direct_total_energy;
    float mgt_flag_indirect_max_pio;
    float mgt_e_indirect_total_energy;
    float mgt_flag;

    // shower to wall
    float stw_1_energy;
    float stw_1_dis;
    float stw_1_dQ_dx;
    float stw_1_flag_single_shower;
    float stw_1_n_pi0;
    float stw_1_num_valid_tracks;
    float stw_1_flag;

    std::vector<float> stw_2_v_medium_dQ_dx;
    std::vector<float> stw_2_v_energy;
    std::vector<float> stw_2_v_angle;
    std::vector<float> stw_2_v_dir_length;
    std::vector<float> stw_2_v_max_dQ_dx;
    std::vector<float> stw_2_v_flag;

    std::vector<float> stw_3_v_angle;
    std::vector<float> stw_3_v_dir_length;
    std::vector<float> stw_3_v_energy;
    std::vector<float> stw_3_v_medium_dQ_dx;
    std::vector<float> stw_3_v_flag;

    std::vector<float> stw_4_v_angle;
    std::vector<float> stw_4_v_dis;
    std::vector<float> stw_4_v_energy;
    std::vector<float> stw_4_v_flag;

    float stw_flag;

    // single photon  cases ...
    float spt_flag_single_shower;
    float spt_energy;
    float spt_shower_main_length;
    float spt_shower_total_length;
    float spt_angle_beam;
    float spt_angle_vertical;
    float spt_max_dQ_dx;
    float spt_angle_beam_1;
    float spt_angle_drift;
    float spt_angle_drift_1;
    float spt_num_valid_tracks;
    float spt_n_vtx_segs;
    float spt_max_length;
    float spt_flag;

    // stem length ...
    float stem_len_energy;
    float stem_len_length;
    float stem_len_flag_avoid_muon_check;
    float stem_len_num_daughters;
    float stem_len_daughter_length;
    float stem_len_flag;

    // low-energy michel
    float lem_shower_total_length;
    float lem_shower_main_length;
    float lem_n_3seg;
    float lem_e_charge;
    float lem_e_dQdx;
    float lem_shower_num_segs;
    float lem_shower_num_main_segs;
    float lem_flag;

     // broken muon ...
    float brm_n_mu_segs;
    float brm_Ep;
    float brm_energy;
    float brm_acc_length;
    float brm_shower_total_length;
    float brm_connected_length;
    float brm_n_size;
    float brm_acc_direct_length;
    float brm_n_shower_main_segs;
    float brm_n_mu_main;
    float brm_flag;


    // compare with muon
    float cme_mu_energy;
    float cme_energy;
    float cme_mu_length;
    float cme_length;
    float cme_angle_beam;
    float cme_flag;


    // angular cut ...
    float anc_energy;
    float anc_angle;
    float anc_max_angle;
    float anc_max_length;
    float anc_acc_forward_length;
    float anc_acc_backward_length;
    float anc_acc_forward_length1;
    float anc_shower_main_length;
    float anc_shower_total_length;
    float anc_flag_main_outside;
    float anc_flag;

        
    // stem direction
    float stem_dir_flag;
    float stem_dir_flag_single_shower;
    float stem_dir_filled;
    float stem_dir_angle;
    float stem_dir_energy;
    float stem_dir_angle1;
    float stem_dir_angle2;
    float stem_dir_angle3;
    float stem_dir_ratio;

     // vertex inside shower
    float vis_1_filled;
    float vis_1_n_vtx_segs;
    float vis_1_energy;
    float vis_1_num_good_tracks;
    float vis_1_max_angle;
    float vis_1_max_shower_angle;
    float vis_1_tmp_length1;
    float vis_1_tmp_length2;
    float vis_1_particle_type;
    float vis_1_flag;

    float vis_2_filled;
    float vis_2_n_vtx_segs;
    float vis_2_min_angle;
    float vis_2_min_weak_track;
    float vis_2_angle_beam;
    float vis_2_min_angle1;
    float vis_2_iso_angle1;
    float vis_2_min_medium_dQ_dx;
    float vis_2_min_length;
    float vis_2_sg_length;
    float vis_2_max_angle;
    float vis_2_max_weak_track;
    float vis_2_flag;

    float vis_flag;


    // bad reconstruction_1
    float br_filled;
    float br1_flag;
    
    //bad reconstruction 1_1
    float br1_1_flag;
    float br1_1_shower_type;
    float br1_1_vtx_n_segs;
    float br1_1_energy;
    float br1_1_n_segs;
    float br1_1_flag_sg_topology;
    float br1_1_flag_sg_trajectory;
    float br1_1_sg_length;

    // bad reconstruction 1_2
    float br1_2_flag;
    float br1_2_energy;
    float br1_2_n_connected;
    float br1_2_max_length;
    float br1_2_n_connected_1;
    float br1_2_vtx_n_segs;
    float br1_2_n_shower_segs;
    float br1_2_max_length_ratio;
    float br1_2_shower_length;
    
    // bad_reconstruction 1_3
    float br1_3_flag;
    float br1_3_energy;
    float br1_3_n_connected_p;
    float br1_3_max_length_p;
    float br1_3_n_shower_segs;
    float br1_3_flag_sg_topology;
    float br1_3_flag_sg_trajectory;
    float br1_3_n_shower_main_segs;
    float br1_3_sg_length;


    // bad reconstruction 2
    float br2_flag;
    float br2_flag_single_shower;
    float br2_num_valid_tracks;
    float br2_energy;
    float br2_angle1;
    float br2_angle2;
    float br2_angle;
    float br2_angle3;
    float br2_n_shower_main_segs;
    float br2_max_angle;
    float br2_sg_length;
    float br2_flag_sg_trajectory;


     //bad reconstruction 3
    float br3_1_energy;
    float br3_1_n_shower_segments;
    float br3_1_sg_flag_trajectory;
    float br3_1_sg_direct_length;
    float br3_1_sg_length;
    float br3_1_total_main_length;
    float br3_1_total_length;
    float br3_1_iso_angle;
    float br3_1_sg_flag_topology;
    float br3_1_flag;

    float br3_2_n_ele;
    float br3_2_n_other;
    float br3_2_energy;
    float br3_2_total_main_length;
    float br3_2_total_length;
    float br3_2_other_fid;
    float br3_2_flag;

    std::vector<float> br3_3_v_energy;
    std::vector<float> br3_3_v_angle;
    std::vector<float> br3_3_v_dir_length;
    std::vector<float> br3_3_v_length;
    std::vector<float> br3_3_v_flag;

    float br3_4_acc_length;
    float br3_4_total_length;
    float br3_4_energy;
    float br3_4_flag;

    std::vector<float> br3_5_v_dir_length;
    std::vector<float> br3_5_v_total_length;
    std::vector<float> br3_5_v_flag_avoid_muon_check;
    std::vector<float> br3_5_v_n_seg;
    std::vector<float> br3_5_v_angle;
    std::vector<float> br3_5_v_sg_length;
    std::vector<float> br3_5_v_energy;
    std::vector<float> br3_5_v_n_main_segs;
    std::vector<float> br3_5_v_n_segs;
    std::vector<float> br3_5_v_shower_main_length;
    std::vector<float> br3_5_v_shower_total_length;
    std::vector<float> br3_5_v_flag;

    std::vector<float> br3_6_v_angle;
    std::vector<float> br3_6_v_angle1;
    std::vector<float> br3_6_v_flag_shower_trajectory;
    std::vector<float> br3_6_v_direct_length;
    std::vector<float> br3_6_v_length;
    std::vector<float> br3_6_v_n_other_vtx_segs;
    std::vector<float> br3_6_v_energy;
    std::vector<float> br3_6_v_flag;

    float br3_7_energy;
    float br3_7_min_angle;
    float br3_7_sg_length;
    float br3_7_shower_main_length;
    float br3_7_flag;

    float br3_8_max_dQ_dx;
    float br3_8_energy;
    float br3_8_n_main_segs;
    float br3_8_shower_main_length;
    float br3_8_shower_length;
    float br3_8_flag;
    
    float br3_flag;
    
    // BR 4
    float br4_1_shower_main_length;
    float br4_1_shower_total_length;
    float br4_1_min_dis;
    float br4_1_energy;
    float br4_1_flag_avoid_muon_check;
    float br4_1_n_vtx_segs;
    float br4_1_n_main_segs;
    float br4_1_flag;

    float br4_2_ratio_45;
    float br4_2_ratio_35;
    float br4_2_ratio_25;
    float br4_2_ratio_15;
    float br4_2_energy;
    float br4_2_ratio1_45;
    float br4_2_ratio1_35;
    float br4_2_ratio1_25;
    float br4_2_ratio1_15;
    float br4_2_iso_angle;
    float br4_2_iso_angle1;
    float br4_2_angle;
    float br4_2_flag;

    float br4_flag;

     // track overclustering ..
    std::vector<float> tro_1_v_particle_type;
    std::vector<float> tro_1_v_flag_dir_weak;
    std::vector<float> tro_1_v_min_dis;
    std::vector<float> tro_1_v_sg1_length;
    std::vector<float> tro_1_v_shower_main_length;
    std::vector<float> tro_1_v_max_n_vtx_segs;
    std::vector<float> tro_1_v_tmp_length;
    std::vector<float> tro_1_v_medium_dQ_dx;
    std::vector<float> tro_1_v_dQ_dx_cut;
    std::vector<float> tro_1_v_flag_shower_topology;
    std::vector<float> tro_1_v_flag;

    std::vector<float> tro_2_v_energy;
    std::vector<float> tro_2_v_stem_length;
    std::vector<float> tro_2_v_iso_angle;
    std::vector<float> tro_2_v_max_length;
    std::vector<float> tro_2_v_angle;
    std::vector<float> tro_2_v_flag;

    float tro_3_stem_length;
    float tro_3_n_muon_segs;
    float tro_3_energy;
    float tro_3_flag;

    std::vector<float> tro_4_v_dir2_mag;
    std::vector<float> tro_4_v_angle;
    std::vector<float> tro_4_v_angle1;
    std::vector<float> tro_4_v_angle2;
    std::vector<float> tro_4_v_length;
    std::vector<float> tro_4_v_length1;
    std::vector<float> tro_4_v_medium_dQ_dx;
    std::vector<float> tro_4_v_end_dQ_dx;
    std::vector<float> tro_4_v_energy;
    std::vector<float> tro_4_v_shower_main_length;
    std::vector<float> tro_4_v_flag_shower_trajectory;
    std::vector<float> tro_4_v_flag;

    std::vector<float> tro_5_v_max_angle;
    std::vector<float> tro_5_v_min_angle;
    std::vector<float> tro_5_v_max_length;
    std::vector<float> tro_5_v_iso_angle;
    std::vector<float> tro_5_v_n_vtx_segs;
    std::vector<float> tro_5_v_min_count;
    std::vector<float> tro_5_v_max_count;
    std::vector<float> tro_5_v_energy;
    std::vector<float> tro_5_v_flag;

    float tro_flag;

    // high energy overlap 
    float hol_1_n_valid_tracks;
    float hol_1_min_angle;
    float hol_1_energy;
    float hol_1_flag_all_shower;
    float hol_1_min_length;
    float hol_1_flag;

    float hol_2_min_angle;
    float hol_2_medium_dQ_dx;
    float hol_2_ncount;
    float hol_2_energy;
    float hol_2_flag;

    float hol_flag;
    
    // low-energy overlap ...
    float lol_flag;

    std::vector<float> lol_1_v_energy;
    std::vector<float> lol_1_v_vtx_n_segs;
    std::vector<float> lol_1_v_nseg;
    std::vector<float> lol_1_v_angle;
    std::vector<float> lol_1_v_flag;

    std::vector<float> lol_2_v_length;
    std::vector<float> lol_2_v_angle;
    std::vector<float> lol_2_v_type;
    std::vector<float> lol_2_v_vtx_n_segs;
    std::vector<float> lol_2_v_energy;
    std::vector<float> lol_2_v_shower_main_length;
    std::vector<float> lol_2_v_flag_dir_weak;
    std::vector<float> lol_2_v_flag;

    float lol_3_angle_beam;
    float lol_3_n_valid_tracks;
    float lol_3_min_angle;
    float lol_3_vtx_n_segs;
    float lol_3_energy;
    float lol_3_shower_main_length;
    float lol_3_n_out;
    float lol_3_n_sum;    
    float lol_3_flag;


    // cosmic tagger ...
    float cosmict_flag_1; // fiducial volume vertex
    float cosmict_flag_2;  // single muon
    float cosmict_flag_3;  // single muon (long)
    float cosmict_flag_4;  // kinematics muon
    float cosmict_flag_5; // kinematics muon (long)
    float cosmict_flag_6; // special ...
    float cosmict_flag_7;  // muon+ michel
    float cosmict_flag_8;  // muon + michel + special
    float cosmict_flag_9;  // this tagger is relevant for nueCC, see "cosmic tagger ones, one case of cosmics ..." (frist one ...)
    std::vector<float> cosmict_flag_10;  // front upstream (dirt)
    float cosmict_flag;

    // single muon
    float cosmict_2_filled;
    float cosmict_2_particle_type;
    float cosmict_2_n_muon_tracks;
    float cosmict_2_total_shower_length;
    float cosmict_2_flag_inside;
    float cosmict_2_angle_beam;
    float cosmict_2_flag_dir_weak;
    float cosmict_2_dQ_dx_end;
    float cosmict_2_dQ_dx_front;
    float cosmict_2_theta;
    float cosmict_2_phi;
    float cosmict_2_valid_tracks;

    // signel muon (long)
    float cosmict_3_filled;
    float cosmict_3_flag_inside;
    float cosmict_3_angle_beam;
    float cosmict_3_flag_dir_weak;
    float cosmict_3_dQ_dx_end;
    float cosmict_3_dQ_dx_front;
    float cosmict_3_theta;
    float cosmict_3_phi;
    float cosmict_3_valid_tracks;

    // kinematics muon
    float cosmict_4_filled;
    float cosmict_4_flag_inside;
    float cosmict_4_angle_beam;
    float cosmict_4_connected_showers;  // need to be careful about the nueCC ...
    
    // kinematics muon (long)
    float cosmict_5_filled;
    float cosmict_5_flag_inside;
    float cosmict_5_angle_beam;
    float cosmict_5_connected_showers;

    // special
    float cosmict_6_filled;
    float cosmict_6_flag_dir_weak;
    float cosmict_6_flag_inside;
    float cosmict_6_angle;

    // muon + michel
    float cosmict_7_filled;
    float cosmict_7_flag_sec;
    float cosmict_7_n_muon_tracks;
    float cosmict_7_total_shower_length;
    float cosmict_7_flag_inside;
    float cosmict_7_angle_beam;
    float cosmict_7_flag_dir_weak;
    float cosmict_7_dQ_dx_end;
    float cosmict_7_dQ_dx_front;
    float cosmict_7_theta;
    float cosmict_7_phi;
    
    // muon + michel + special
    float cosmict_8_filled;
    float cosmict_8_flag_out;
    float cosmict_8_muon_length;
    float cosmict_8_acc_length;
    
    // front upstream (dirt)
    std::vector<float> cosmict_10_flag_inside;
    std::vector<float> cosmict_10_vtx_z;
    std::vector<float> cosmict_10_flag_shower;
    std::vector<float> cosmict_10_flag_dir_weak;
    std::vector<float> cosmict_10_angle_beam;
    std::vector<float> cosmict_10_length;
    
    // numu vs. nc tagger
    float numu_cc_flag;

    // single muon connected to neutrino vertex
    std::vector<float> numu_cc_flag_1;
    std::vector<float> numu_cc_1_particle_type;
    std::vector<float> numu_cc_1_length;
    std::vector<float> numu_cc_1_medium_dQ_dx;
    std::vector<float> numu_cc_1_dQ_dx_cut;
    std::vector<float> numu_cc_1_direct_length;
    std::vector<float> numu_cc_1_n_daughter_tracks;
    std::vector<float> numu_cc_1_n_daughter_all;
    
    // long muon connected to neutrino vertex
    std::vector<float> numu_cc_flag_2;
    std::vector<float> numu_cc_2_length;
    std::vector<float> numu_cc_2_total_length;
    std::vector<float> numu_cc_2_n_daughter_tracks;
    std::vector<float> numu_cc_2_n_daughter_all;
    
    // any muon ...
    float numu_cc_flag_3;
    float numu_cc_3_particle_type;
    float numu_cc_3_max_length;
    float numu_cc_3_acc_track_length;
    float numu_cc_3_max_length_all;
    float numu_cc_3_max_muon_length;
    float numu_cc_3_n_daughter_tracks;
    float numu_cc_3_n_daughter_all;

    // numu_BDTs
    float cosmict_2_4_score;
    float cosmict_3_5_score;
    float cosmict_6_score;
    float cosmict_7_score;
    float cosmict_8_score;
    // vector ...
    float cosmict_10_score;
    
    // vector
    float numu_1_score;
    float numu_2_score;
    // scalar
    float numu_3_score;

    // total one
    float cosmict_score;
    float numu_score;
    
    
    
    // nue BDTs
    float mipid_score;
    float gap_score;
    float hol_lol_score;
    float cme_anc_score;
    float mgo_mgt_score;
    float br1_score;
    float br3_score;
    float br3_3_score;
    float br3_5_score;
    float br3_6_score;
    float stemdir_br2_score;
    float trimuon_score;
    float br4_tro_score;
    float mipquality_score;
    float pio_1_score;
    float pio_2_score;
    float stw_spt_score;
    float vis_1_score;
    float vis_2_score;
    float stw_2_score;
    float stw_3_score;
    float stw_4_score;
    float sig_1_score;
    float sig_2_score;
    float lol_1_score;
    float lol_2_score;
    float tro_1_score;
    float tro_2_score;
    float tro_4_score;
    float tro_5_score;
    float nue_score;

    float photon_flag;

    
  };

  
  
  //  class MyFCN : public ROOT::Minuit2::FCNBase {
  class MyFCN {
  
  public: 
    //    double Up() const { return 1.;}

    MyFCN(ProtoVertex* vtx, bool flag_vtx_constraint = false, double vtx_constraint_range = 1*units::cm, double vertex_protect_dis = 1.5*units::cm, double vertex_protect_dis_short_track = 0.9*units::cm, double fit_dis = 6 * units::cm);    
    ~MyFCN();

    void update_fit_range(double tmp_vertex_protect_dis = 1.5*units::cm, double  tmp_vertex_protect_dis_short_track = 0.9*units::cm, double tmp_fit_dis = 6 * units::cm);
    void AddSegment(ProtoSegment *sg);
    std::pair<bool, WCP::Point> FitVertex();
    void UpdateInfo(WCP::Point fit_pos, WCPPID::PR3DCluster* temp_cluster, double default_dis_cut = 4.0*units::cm);
    
    std::pair<ProtoSegment*, int> get_seg_info(int i);
    int get_fittable_tracks();
    bool get_flag_vtx_constraint(){return flag_vtx_constraint;};
    void set_flag_vtx_constraint(bool val){flag_vtx_constraint = val;};

    void set_vtx_constraint_range(double val){vtx_constraint_range = val;};

    std::vector<ProtoSegment*>& get_segments(){return segments;};
    std::vector<WCP::PointVector>& get_vec_points(){return vec_points;};

    void print_points();
    void set_enforce_two_track_fit(bool val){enforce_two_track_fit = val;};
    bool get_enforce_two_track_fit(){return enforce_two_track_fit;};
   
    //  double operator() (const std::vector<double> & xx) const;
    //  double get_chi2(const std::vector<double> & xx) const;
    
  private:
    ProtoVertex *vtx;
    bool enforce_two_track_fit;
    bool flag_vtx_constraint;
    double vtx_constraint_range;
    
    double vertex_protect_dis;
    double vertex_protect_dis_short_track;
    double fit_dis;
    
    std::vector<ProtoSegment* > segments;
    std::vector<WCP::PointVector> vec_points;

    std::vector<std::tuple<WCP::Point, WCP::Point, WCP::Point> > vec_PCA_dirs;
    std::vector<std::tuple<double, double, double> > vec_PCA_vals;
    std::vector<WCP::Point> vec_centers;
  };
  
  
  class NeutrinoID{
  public:
    // Enable/check named debug prints. Use the function name as the key.
    // From a run script: call neutrino->enable_debug("init_first_segment").
    // Multiple functions can be enabled independently.
    void enable_debug(const std::string& func_name){ debug_funcs.insert(func_name); }
    bool is_debug(const std::string& func_name) const { return debug_funcs.count(func_name) > 0; }

    NeutrinoID(WCPPID::PR3DCluster *main_cluster, std::vector<WCPPID::PR3DCluster*>& other_clusters, std::vector<WCPPID::PR3DCluster*>& all_clusters, WCPPID::ToyFiducial* fid, WCPSst::GeomDataSource& gds, int nrebin, int frame_length, float unit_dis,	WCP::ToyCTPointCloud* ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, double flash_time, double offset_x, int flag_neutrino_id_process=1, int flag_bdt = 0, bool flag_dl_vtx = false, double dl_vtx_cut = 2.0*units::cm, float match_isFC = 0, bool is_neutrino_candidate = true, const std::set<std::string>& debug_funcs_init = {});
    ~NeutrinoID();

    // deal with the map ...
    bool del_proto_vertex(ProtoVertex *pv);
    bool del_proto_segment(ProtoSegment *ps);
    bool add_proto_connection(ProtoVertex *pv, ProtoSegment *ps, WCPPID::PR3DCluster* cluster);
    bool del_proto_connection(ProtoVertex *pv, ProtoSegment *ps);
    
    void organize_vertices_segments();
    
    std::tuple<ProtoVertex*, ProtoSegment*, WCP::Point> check_end_point(WCPPID::PR3DCluster* temp_cluster, WCP::PointVector& tracking_path, bool flag_front = true, double vtx_cut1 = 0.9*units::cm, double vtx_cut2 = 2.0*units::cm, double sg_cut1 = 2.0 * units::cm, double sg_cut2 = 1.2*units::cm);

    bool modify_vertex_isochronous(WCPPID::ProtoVertex* vtx, WCPPID::ProtoVertex *v1, WCPPID::ProtoSegment* sg, WCPPID::ProtoVertex *v2, WCPPID::PR3DCluster* temp_cluster);
    bool modify_segment_isochronous(WCPPID::ProtoSegment* sg1, WCPPID::ProtoVertex *v1, WCPPID::ProtoSegment* sg, WCPPID::ProtoVertex *v2, WCPPID::PR3DCluster* temp_cluster, double dis_cut = 6*units::cm, double angle_cut = 15, double extend_cut = 15*units::cm);
    // get segments
    int get_num_segments(ProtoVertex *pv);

    WCPPID::PR3DCluster* get_main_cluster(){return main_cluster;};

    // swap main cluster
    void swap_main_cluster(WCPPID::PR3DCluster *new_main_cluster);
    
    // actual functions ...
    void process_main_cluster();
    void process_other_clusters();

    // fill reco information
    void fill_reco_tree(WCPPID::ProtoSegment* seg, WCRecoTree& rtree);
    void fill_reco_tree(WCPPID::WCShower* shower, WCRecoTree& rtree);
    std::pair<int, int> fill_pi0_reco_tree(WCPPID::WCShower* shower, WCRecoTree& rtree);
    int fill_psuedo_reco_tree(WCPPID::WCShower* shower, WCRecoTree& rtree);

    void fill_reco_simple_tree(WCRecoTree& rtree);
    void fill_proto_main_tree(WCRecoTree& rtree);
    void fill_particle_tree(WCRecoTree& rtree);

    void fill_kine_tree(KineInfo& ktree);
    
    void fill_skeleton_info_magnify(int mother_cluster_id, WCPointTree& ptree, TTree *T, double dQdx_scale, double dQdx_offset, bool flag_skip_vertex = false);
    void fill_skeleton_info(int mother_cluster_id, WCPointTree& ptree, TTree *T, double dQdx_scale, double dQdx_offset, bool flag_skip_vertex = false);
    void fill_point_info(int mother_cluster_id, WCPointTree& ptree, TTree *T);
    
    void check_end_segments(WCPPID::PR3DCluster* temp_cluster);
    
    // proto-vertex finder
    bool find_proto_vertex(WCPPID::PR3DCluster *cluster, bool flag_break_trak = true, int nrounds_find_other_tracks = 2, bool flag_back_search = true);
    WCPPID::ProtoSegment* init_first_segment(WCPPID::PR3DCluster *cluster, bool flag_back_search = true);
    void init_point_segment(WCPPID::PR3DCluster *cluster);
    void break_segments(std::vector<WCPPID::ProtoSegment*>& remaining_segments, WCPPID::PR3DCluster* temp_cluster, float dis_cut = 0);
    void examine_vertices(WCPPID::PR3DCluster* temp_cluster);
    bool examine_vertices_1(WCPPID::PR3DCluster* temp_cluster); // one segment can be skipped ...
    bool examine_vertices_1(WCPPID::ProtoVertex* v1, WCPPID::ProtoVertex *v2, double offset_t, double slope_xt, double offset_u, double slope_yu, double slope_zu, double offset_v, double slope_yv, double slope_zv, double offset_w, double slope_yw, double slope_zw);
    
    bool examine_vertices_2(WCPPID::PR3DCluster* temp_cluster); // two close vertices  ...
    void examine_segment(WCPPID::PR3DCluster* temp_cluster);
    bool crawl_segment(WCPPID::ProtoSegment *sg, WCPPID::ProtoVertex *vtx, WCPPID::PR3DCluster* temp_cluster);

    void examine_partial_identical_segments(WCPPID::PR3DCluster* temp_cluster);
    
    
    bool examine_vertices_4(WCPPID::PR3DCluster* temp_cluster); // two close vertices with multiple segments
    bool examine_vertices_4(WCPPID::ProtoVertex* v1, WCPPID::ProtoVertex *v2); // check v1's segment against v2 ...
    
    void examine_vertices_3(); // main cluster only examine the two initial points

    // after the main vertex determination ... examine structure ... fix mistakes ...
    bool examine_structure_final(WCPPID::PR3DCluster* temp_cluster); 
    bool examine_structure_final_1(WCPPID::PR3DCluster* temp_cluster); // merge two segments if a direct connection is better ...
    bool examine_structure_final_1p(WCPPID::PR3DCluster* temp_cluster); // merge two segments if a direct connection is better for main vertex ...
    
    bool examine_structure_final_2(WCPPID::PR3DCluster* temp_cluster); // merge vertices close to the main vertex
    bool examine_structure_final_3(WCPPID::PR3DCluster* temp_cluster); // move main vertex to a nearby vertex, if it is close ...

    
    //  bool examine_vertices_5(WCPPID::ProtoSegment *sg, WCPPID::ProtoVertex* vtx, WCPPID::ProtoVertex *vtx2, WCPPID::ProtoSegment *sg1, WCPPID::ProtoVertex *vtx1, WCPPID::ProtoSegment *sg2);

    
    

    void examine_structure(WCPPID::PR3DCluster *temp_cluster);
    // straighten one short wiggled track with high dQ/dx 
    bool examine_structure_1(WCPPID::PR3DCluster *temp_cluster);
    // merge two short tracks into a straight one
    bool examine_structure_2(WCPPID::PR3DCluster *temp_cluster);
    // merge two tracks into one if their angle are consistent ...
    bool examine_structure_3(WCPPID::PR3DCluster *temp_cluster);
    // search for missing tracks  ...
    bool examine_structure_4(WCPPID::ProtoVertex *vertex, WCPPID::PR3DCluster *temp_cluster, bool flag_final_vertex = false);

    
    void find_other_segments(WCPPID::PR3DCluster* temp_cluster, bool flag_break_track = true, double search_range = 1.5*units::cm, double scaling_2d = 0.8);
    WCPPID::ProtoSegment* find_incoming_segment(WCPPID::ProtoVertex *vtx);
    ProtoVertex* find_vertex_other_segment(WCPPID::PR3DCluster *temp_cluster, bool flag_forward, WCP::WCPointCloud<double>::WCPoint& wcp);
    std::pair<WCPPID::ProtoSegment*, WCPPID::ProtoVertex* > find_cont_muon_segment(WCPPID::ProtoSegment* sg, WCPPID::ProtoVertex* vtx, bool flag_ignore_dQ_dx = false);
    std::pair<WCPPID::ProtoSegment*, WCPPID::ProtoVertex* > find_cont_muon_segment_nue(WCPPID::ProtoSegment* sg, WCPPID::ProtoVertex* vtx, bool flag_ignore_dQ_dx = false);
    
    // calculate charge
    void collect_2D_charges();
    double cal_kine_charge(WCPPID::ProtoSegment *sg);
    double cal_kine_charge(WCPPID::WCShower *shower);
    double cal_corr_factor(WCP::Point& p, double offset_u, double slope_yu, double slope_zu, double offset_v, double slope_yv, double slope_zv, double offset_w, double slope_yw, double slope_zw);

    
    // improve vertex ...
    void improve_vertex(WCPPID::PR3DCluster* temp_cluster, bool flag_search_vertex_activity = true, bool flag_final_vertex = false);
    bool fit_vertex(WCPPID::ProtoVertex *vtx, WCPPID::ProtoSegmentSet& sg_set, WCPPID::PR3DCluster* temp_cluster);
    bool search_for_vertex_activities(WCPPID::ProtoVertex *vtx, WCPPID::ProtoSegmentSet& sg_set, WCPPID::PR3DCluster* temp_cluster, double search_range = 1.5*units::cm);
    bool eliminate_short_vertex_activities(WCPPID::PR3DCluster *temp_cluster, std::set<WCPPID::ProtoSegment*>& existing_segments);

    std::map<WCPPID::PR3DCluster*, WCPPID::ProtoVertex* >& get_map_cluster_vertex(){return map_cluster_main_vertices;};
    std::map<WCPPID::PR3DCluster*, WCPPID::ProtoVertexSelection>& get_map_cluster_candidate_vertices(){return map_cluster_main_candidate_vertices;};
    
    // get direction 
    TVector3 get_dir(WCPPID::ProtoVertex *vtx, WCPPID::ProtoSegment *sg, double dis = 2*units::cm);
    TVector3 get_dir(WCPPID::ProtoVertex *vtx, double dis_cut = 5*units::cm);
    
    void determine_direction(WCPPID::PR3DCluster* temp_cluster);
    void determine_main_vertex(WCPPID::PR3DCluster* temp_cluster, bool flag_print = true);
    std::tuple<bool, int, int> examine_main_vertex_candidate(WCPPID::ProtoVertex *vertex);

    void determine_overall_main_vertex();
    bool determine_overall_main_vertex_DL();

    
    void examine_main_vertices();
    void examine_main_vertices(WCPPID::ProtoVertexSelection& vertices);
    void check_switch_main_cluster(WCPPID::ProtoVertex *temp_main_vertex, WCPPID::PR3DCluster *max_length_cluster);
    void check_switch_main_cluster();
    
    // if there is one in, fix the others ...
    void improve_maps_one_in(WCPPID::PR3DCluster* temp_cluster, bool flag_strong_check = true);
    void fix_maps_shower_in_track_out(int temp_cluster_id);
    void fix_maps_multiple_tracks_in(int temp_cluster_id);
    void improve_maps_shower_in_track_out(int temp_cluster_id, bool flag_strong_check = true);
    void improve_maps_multiple_tracks_in(int temp_cluster_id);
    void improve_maps_no_dir_tracks(int temp_cluster_id);
    void judge_no_dir_tracks_close_to_showers(int temp_cluster_id);
    void examine_good_tracks(int temp_cluster_id);
    
    void examine_all_showers(WCPPID::PR3DCluster* temp_cluster);
    TVector3 calc_dir_cluster(int tmp_cluster_id, WCP::Point& test_p, double dis_cut);
    
    std::pair<int, double> calculate_num_daughter_showers(WCPPID::ProtoVertex *vtx, WCPPID::ProtoSegment *sg, bool flag_count_shower = true);

    std::pair<int, double> calculate_num_daughter_tracks(WCPPID::ProtoVertex *vtx, WCPPID::ProtoSegment *sg, bool flag_count_shower = false, double length_cut = 0);
    
    void change_daughter_type(WCPPID::ProtoVertex *vtx, WCPPID::ProtoSegment *sg, int particle_type, double mass);
    
    bool examine_maps(WCPPID::PR3DCluster* temp_cluster);
    bool examine_maps(WCPPID::ProtoVertex *temp_vertex);
    bool examine_maps(int temp_cluster_id);

    float calc_conflict_maps(WCPPID::ProtoVertex *temp_vertex);
    
    void print_segs_info(WCPPID::PR3DCluster* temp_cluster);
    void print_segs_info(WCPPID::ProtoVertex *temp_vertex);
    void print_segs_info(int temp_cluster_id, WCPPID::ProtoVertex *spec_vertex=0);
    
    
    WCPPID::ProtoVertex* compare_main_vertices_all_showers(WCPPID::ProtoVertexSelection& vertex_candidates, WCPPID::PR3DCluster *temp_cluster);
    WCPPID::ProtoVertex* compare_main_vertices(WCPPID::ProtoVertexSelection& vertex_candidates);
    WCPPID::ProtoVertex* compare_main_vertices_global(WCPPID::ProtoVertexSelection& vertex_candidates);
    std::pair<WCP::Point, TVector3> calc_PCA_main_axis(WCP::PointVector& points);
    
    bool examine_direction(WCPPID::ProtoVertex* vertex, bool flag_final = false);
    
    // clustering points
    void clustering_points(WCPPID::PR3DCluster* temp_cluster);

    WCPPID::ProtoSegment* find_segment(WCPPID::ProtoVertex *v1, WCPPID::ProtoVertex *v2);
    std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> find_vertices(WCPPID::ProtoSegment* sg);
    WCPPID::ProtoVertex* find_other_vertex(WCPPID::ProtoSegment *sg, WCPPID::ProtoVertex* v1); 
    WCPPID::ProtoVertexSelection find_vertices(WCPPID::PR3DCluster* temp_cluster);
    WCPPID::ProtoSegmentSelection find_segments(WCPPID::PR3DCluster* temp_cluster);
    std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*>get_start_end_vertices(WCPPID::ProtoSegment* seg);
    
    Map_Proto_Vertex_Segments& get_map_vertex_segments(){return map_vertex_segments;};
    Map_Proto_Segment_Vertices& get_map_segment_verteices(){return map_segment_vertices;};

    // deghost
    void deghosting();
    void deghost_clusters();
    void deghost_segments();
    
    void order_clusters(WCPPID::PR3DClusterSelection& ordered_clusters, std::map<int, WCPPID::ProtoSegmentSelection>& map_cluster_id_segments, std::map<WCPPID::PR3DCluster*, double>& map_cluster_total_length);
    void order_segments(WCPPID::ProtoSegmentSelection& ordered_segments, WCPPID::ProtoSegmentSelection& segments);
    
    // track shower separation
    void separate_track_shower();
    void separate_track_shower(WCPPID::PR3DCluster* temp_cluster);
    std::pair<int,int> count_num_tracks_showers(WCPPID::PR3DCluster* temp_cluster);
    
    // particle_clustering
    void shower_determing_in_main_cluster(WCPPID::PR3DCluster *temp_cluster);
    void shower_clustering_with_nv();
    void shower_clustering_with_nv_in_main_cluster();
    void shower_clustering_connecting_to_main_vertex();
    void shower_clustering_with_nv_from_main_cluster();
    void shower_clustering_with_nv_from_vertices();
    void examine_merge_showers();
    // holder for now ...
    void shower_clustering_in_other_clusters(bool flag_save = true);
    void id_pi0_with_vertex();
    void id_pi0_without_vertex();
    
    // establish map
    //    void establish_cluster_segment_maps();
    
    void calculate_shower_kinematics();
    void update_shower_maps();
    void clean_up_maps_vertices_segments(WCPPID::PR3DCluster *temp_cluster);
    
    // fill_fit_parameters();
    void fill_fit_parameters();
    WCPPID::ProtoVertex* get_main_vertex(){return main_vertex;};
    int get_neutrino_type(){return neutrino_type;};

    bool cosmic_tagger();
    std::pair<bool, double> numu_tagger();
    std::pair<int, int> count_daughters(WCPPID::ProtoSegment *max_muon);
    std::pair<int, int> count_daughters(WCPPID::WCShower *max_long_muon);
 
    bool nue_tagger(double muon_kine_energy = 0);
    bool singlephoton_tagger(double muon_kine_energy = 0);
     
    bool ssm_tagger();
    void set_save_ssmsp(int save_ssmsp){flag_ssmsp = save_ssmsp;};
    std::vector<double> get_scores(WCPPID::ProtoSegment *sg);
    std::vector<double> get_scores(WCPPID::ProtoSegment *sg, int break_point, int dir);
    std::vector<double> calc_kine_range_multi_pdg(double length);
    double calc_kine_range_pdg(double length, int pdg);
    std::pair< std::pair<int, int>,std::pair<int, int> >count_daughters(WCPPID::ProtoSegment *max_muon, double length_cut);
    void fill_ssmsp(WCPPID::ProtoSegment* sg, int pdg, int mother, int dir);
    int  fill_ssmsp_psuedo(WCPPID::WCShower* daught_shower, int mother, int acc_id);
    int fill_ssmsp_psuedo(WCPPID::WCShower* shower, WCPPID::ProtoSegment* sg, int mother, int acc_id);
    int fill_ssmsp_psuedo(WCPPID::WCShower* daught_shower, WCPPID::ProtoSegment* mother_sg, int acc_id);
    int get_containing_shower_id(WCPPID::ProtoSegment* seg);
    WCPPID::WCShower* get_containing_shower(WCPPID::ProtoSegment* seg);
    bool exit_ssm_tagger();
    void print_ssm_tagger();
    
    float cal_numu_bdts();
    float cal_numu_bdts_xgboost();
    float cal_cosmict_2_4_bdt(float default_val);
    float cal_cosmict_3_5_bdt(float default_val);
    float cal_cosmict_6_bdt(float default_val);
    float cal_cosmict_7_bdt(float default_val);
    float cal_cosmict_8_bdt(float default_val);
    float cal_cosmict_10_bdt(float default_val);
    float cal_numu_1_bdt(float default_val);
    float cal_numu_2_bdt(float default_val);
    float cal_numu_3_bdt(float default_val);
    
      
    
    float cal_bdts();
    float cal_bdts_xgboost();
    
    float cal_mipid_bdt(float default_val = 0.5);
    float cal_gap_bdt(float default_val = 0.5);
    float cal_hol_lol_bdt(float default_val = 0.5);
    float cal_cme_anc_bdt(float default_val = 0.5);
    float cal_mgo_mgt_bdt(float default_val = 0.5);
    float cal_br1_bdt(float default_val = 0.5);
    float cal_br3_bdt(float default_val = 0.5);
    float cal_br3_3_bdt(float default_val = 0.5);
    float cal_br3_5_bdt(float default_val = 0.5);
    float cal_br3_6_bdt(float default_val = 0.5);
    float cal_stemdir_br2_bdt(float default_val = 0.5);
    float cal_trimuon_bdt(float default_val = 0.5);
    float cal_br4_tro_bdt(float default_val = 0.5);
    float cal_mipquality_bdt(float default_val = 0.5);
    float cal_pio_1_bdt(float default_val = 0.5);
    float cal_pio_2_bdt(float default_val = 0.5);
    float cal_stw_spt_bdt(float default_val = 0.5);
    float cal_vis_1_bdt(float default_val = 0.5);
    float cal_vis_2_bdt(float default_val = 0.5);
    float cal_stw_2_bdt(float default_val = 0.5);
    float cal_stw_3_bdt(float default_val = 0.5);
    float cal_stw_4_bdt(float default_val = 0.5);
    float cal_sig_1_bdt(float default_val = 0.5);
    float cal_sig_2_bdt(float default_val = 0.5);
    float cal_lol_1_bdt(float default_val = 0.5);
    float cal_lol_2_bdt(float default_val = 0.5);
    float cal_tro_1_bdt(float default_val = 0.5);
    float cal_tro_2_bdt(float default_val = 0.5);
    float cal_tro_4_bdt(float default_val = 0.5);
    float cal_tro_5_bdt(float default_val = 0.5);
    
    void examine_showers();
    void examine_shower_1();
    std::pair<bool, int> gap_identification(WCPPID::ProtoVertex* vertex, WCPPID::ProtoSegment* sg, bool flag_single_shower = false, int valid_tracks = 0, double Eshower = 0, bool flag_fill = false);
    bool mip_quality(WCPPID::ProtoVertex *vertex, WCPPID::ProtoSegment *sg, WCPPID::WCShower *shower, bool flag_print = false, bool flag_fill = false);
    int mip_identification(WCPPID::ProtoVertex* vertex, WCPPID::ProtoSegment *sg, WCPPID::WCShower *shower, bool flag_single_shower, bool flag_strong_check = false, bool flag_print = false, bool flag_fill = false);
    bool pi0_identification(WCPPID::ProtoVertex* vertex, WCPPID::ProtoSegment *sg, WCPPID::WCShower *shower, double threshild  = 0, bool flag_fill = false);
    bool stem_direction(WCPPID::WCShower *shower, double energy, bool flag_print = false, bool flag_fill = false);
    bool bad_reconstruction(WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);
    bool bad_reconstruction_1(WCPPID::WCShower* shower, bool flag_single_shower, int num_valid_tracks, bool flag_fill = false);
    bool low_energy_overlapping(WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill =false);
    
    bool bad_reconstruction_2(WCPPID::ProtoVertex *vertex, WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);
    // main shower stem separated from the other parts
    bool bad_reconstruction_3(WCPPID::ProtoVertex *vertex, WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);

    bool stem_length(WCPPID::WCShower *shower, double energy, bool flag_print = false, bool flag_fill = false);
    bool compare_muon_energy(WCPPID::WCShower *shower, double energy, double muon_length, bool flag_print = false, bool flag_fill = false);
    
    bool high_energy_overlapping(WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);

    bool single_shower_pio_tagger(WCPPID::WCShower* shower, bool flag_single_shower, bool flag_print = false, bool flag_fill = false);
    bool shower_to_wall(WCPPID::WCShower* shower, double shower_energy, bool flag_single_shower, bool flag_print = false, bool flag_fill = false);
   
    bool broken_muon_id(WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);
    bool track_overclustering(WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);

    bool multiple_showers(WCPPID::WCShower *shower, double max_energy, bool flag_print = false, bool flag_fill = false);
      
    
    bool other_showers(WCPPID::WCShower* shower, bool flag_single_shower, bool flag_print = false, bool flag_fill = false);
    bool vertex_inside_shower(WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);
    bool angular_cut(WCPPID::WCShower* shower, double energy, double angle, bool flag_print = false, bool flag_fill =false);
    bool single_shower(WCPPID::WCShower* shower, bool flag_single_shower, bool flag_print = false, bool flag_fill = false);

    bool low_energy_michel(WCPPID::WCShower*shower, bool flag_print = false, bool flag_fill = false);

      //single photon

    int mip_identification_sp(WCPPID::ProtoVertex* vertex, WCPPID::ProtoSegment *sg, WCPPID::WCShower *shower, bool flag_single_shower, bool flag_strong_check = false, bool flag_print = false, bool flag_fill = false);
    bool pi0_identification_sp(WCPPID::ProtoVertex* vertex, WCPPID::ProtoSegment *sg, WCPPID::WCShower *shower, double threshild  = 0, bool flag_fill = false);
    bool bad_reconstruction_sp(WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);
    bool bad_reconstruction_1_sp(WCPPID::WCShower* shower, bool flag_single_shower, int num_valid_tracks, bool flag_fill = false);
    bool low_energy_overlapping_sp(WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill =false);
    bool bad_reconstruction_2_sp(WCPPID::ProtoVertex *vertex, WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);
    // main shower stem separated from the other parts
    bool bad_reconstruction_3_sp(WCPPID::ProtoVertex *vertex, WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);
    bool high_energy_overlapping_sp(WCPPID::WCShower* shower, bool flag_print = false, bool flag_fill = false);
    bool low_energy_michel_sp(WCPPID::WCShower*shower, bool flag_print = false, bool flag_fill = false);
    
    void init_tagger_info();
    TaggerInfo tagger_info;
    KineInfo get_kine_info(){return kine_info;};
    
    KineInfo kine_info;
    
    float kine_pio_mass; // mass
    int kine_pio_flag; // 0 not filled, 1, with vertex, 2 without vertex
    float kine_pio_vtx_dis;
    
    float kine_pio_energy_1;
    float kine_pio_theta_1;
    float kine_pio_phi_1;
    float kine_pio_dis_1;
    
    float kine_pio_energy_2;
    float kine_pio_theta_2;
    float kine_pio_phi_2;
    float kine_pio_dis_2;

    float kine_pio_angle;
    
    
  protected:
    std::set<std::string> debug_funcs;  // names of functions with debug printing on; empty = all off

    float match_isFC;
      
    int neutrino_type;
    int flag_bdt;
    bool flag_dl_vtx;
    double dl_vtx_cut;
    
    int acc_vertex_id;
    int acc_segment_id;

    int flag_neutrino_id_process;

    // global variable ...
    std::map<WCPPID::PR3DCluster*, double> map_cluster_length;

    std::map<WCPPID::PR3DCluster*, WCPPID::ProtoVertex* > map_cluster_main_vertices;
    std::map<WCPPID::PR3DCluster*, WCPPID::ProtoVertexSelection> map_cluster_main_candidate_vertices;
    
    // input ...
    WCPPID::PR3DCluster *main_cluster;
    std::pair<WCPPID::ProtoVertex*, WCPPID::ProtoVertex*> main_cluster_initial_pair_vertices;
    
    std::vector<WCPPID::PR3DCluster*> other_clusters;
    std::vector<WCPPID::PR3DCluster*> all_clusters;
    WCPPID::ToyFiducial* fid;
      
    WCPPID::ProtoVertex *main_vertex;
    
    std::map<int, WCPPID::PR3DCluster*> map_id_cluster;
    
    WCP::ToyCTPointCloud* ct_point_cloud;
    std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > > global_wc_map;
    double flash_time;
    double offset_x;
    // output ...
    int type; // nue, numu, NC for 1,2,3,    0 for no ID

    // graph ...
    ProtoVertexSelection proto_vertices;
    ProtoSegmentSelection proto_segments;
    Map_Proto_Vertex_Segments map_vertex_segments;
    Map_Proto_Segment_Vertices map_segment_vertices;
    // map the cluster to the vertices/segments
    std::map<PR3DCluster*, ProtoVertexSet> map_cluster_vertices;
    std::map<ProtoVertex*, PR3DCluster*> map_vertex_cluster;
    std::map<PR3DCluster*, ProtoSegmentSet> map_cluster_segments;
    std::map<ProtoSegment*, PR3DCluster*> map_segment_cluster;

    std::vector<std::tuple<PR3DCluster*, int, int> > residual_segment_candidates;

    std::map<std::pair<int,int>, std::pair<double,double> > charge_2d_u;
    std::map<std::pair<int,int>, std::pair<double,double> > charge_2d_v;
    std::map<std::pair<int,int>, std::pair<double,double> > charge_2d_w;
    
    // after fit, for alter direction, further clustering particles ...
    WCShowerSelection showers;
    
    // find the particle, given something inside ...
    std::map<WCPPID::ProtoVertex*, WCShower* > map_vertex_in_shower; 
    std::map<WCPPID::ProtoSegment*, WCShower*> map_segment_in_shower;
    // find the connection ...
    std::map<WCPPID::ProtoVertex*, std::set<WCShower*> > map_vertex_to_shower;
    std::set<int> used_shower_clusters;

    // pi0 information
    std::set<WCShower*> pi0_showers;
    std::map<WCShower*, int> map_shower_pio_id;
    std::map<int, std::vector<WCShower* > > map_pio_id_showers;
    std::map<int, std::pair<double, int> > map_pio_id_mass; // 1 for with vertex, 2 for displayced vertex
    std::map<int, std::pair<int, int> > map_pio_id_saved_pair;

    std::set<WCPPID::ProtoSegment*> segments_in_long_muon;
    std::set<WCPPID::ProtoVertex*> vertices_in_long_muon;
    
    //ssm sapcepoints flag, 0 save spacepoints when tagged, 1 always save spacepoints, -1 never save space points
    int flag_ssmsp=-1;
  };

  struct Res_proto_segment
  {
    int group_num;
    int number_points;
    int special_A, special_B;
    double length;
    int number_not_faked;
    double max_dis_u;
    double max_dis_v;
    double max_dis_w;
  };
  
}

#endif
