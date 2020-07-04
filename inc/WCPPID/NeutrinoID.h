#ifndef WIRECELLPID_NEUTRINOID_H
#define WIRECELLPID_NEUTRINOID_H

#include "WCPSst/GeomDataSource.h"

#include "WCPPID/PR3DCluster.h"
#include "WCPData/ToyCTPointCloud.h"

#include "WCPPID/WCShower.h"

#include "WCPPID/ToyFiducial.h"

#include "WCPPID/Map_Proto_Vertex_Segment.h"

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

  //
  struct TaggerInfo
  {
    // cosmic tagger ones, one case of cosmics ...
    int cosmic_flag; // default is true ...
    int cosmic_filled;   // if things are filled
    // variable ...
    int cosmic_n_solid_tracks;    // used
    double cosmic_energy_main_showers; // used
    double cosmic_energy_indirect_showers; // used
    // not used ...
    double cosmic_energy_direct_showers;  
    int cosmic_n_direct_showers;
    int cosmic_n_indirect_showers;
    int cosmic_n_main_showers;


    // shower gap identification
    int gap_flag;
    int gap_filled;
    // main variables:
    int gap_n_bad;
    int gap_n_points;
    double gap_energy;
    int gap_flag_single_shower;
    int gap_flag_parallel;
    // not directly used
    int gap_flag_prolong_u;
    int gap_flag_prolong_v;
    int gap_flag_prolong_w;
    int gap_num_valid_tracks;

    
    // mip_quality
    int mip_quality_flag;
    int mip_quality_filled;
    // variables
    double mip_quality_energy;
    int mip_quality_overlap;
    int mip_quality_n_showers;
    int mip_quality_n_tracks;
    int mip_quality_flag_inside_pi0;
    int mip_quality_n_pi0_showers;
    double mip_quality_shortest_length;
    double mip_quality_acc_length;
    double mip_quality_shortest_angle;
    int mip_quality_flag_proton;
    

    // mip identification
    int mip_flag;
    int mip_filled;
    // variables
    double mip_energy; // checked
    int mip_n_end_reduction;     // checked
    int mip_n_first_mip; // checked
    int mip_n_first_non_mip; // checked
    int mip_n_first_non_mip_1; // checked
    int mip_n_first_non_mip_2; // checked
    double mip_vec_dQ_dx_0; // checked
    double mip_vec_dQ_dx_1; // checked
    double mip_max_dQ_dx_sample; // checked
    int mip_n_below_threshold; // checked
    int mip_n_below_zero;  // checked
    int mip_n_lowest;  // checked
    int mip_n_highest;  // checked
    double mip_lowest_dQ_dx;  // checked
    double mip_highest_dQ_dx;  // checked
    double mip_medium_dQ_dx;  // checked
    double mip_stem_length;  // checked
    double mip_length_main;   // checked
    double mip_length_total; // checked
    double mip_angle_beam;  // checked
    double mip_iso_angle;  // checked
    int mip_n_vertex;   // checked
    int mip_n_good_tracks;  // checked
    double mip_E_indirect_max_energy;  // checked
    int mip_flag_all_above;     // checked
    double mip_min_dQ_dx_5;  // checked
    int mip_n_other_vertex;   // checked
    int mip_n_stem_size;  // checked
    int mip_flag_stem_trajectory;  // checked
    double mip_min_dis;   // checked


    // extra
    double mip_vec_dQ_dx_2;
    double mip_vec_dQ_dx_3;
    double mip_vec_dQ_dx_4;
    double mip_vec_dQ_dx_5;
    double mip_vec_dQ_dx_6;
    double mip_vec_dQ_dx_7;
    double mip_vec_dQ_dx_8;
    double mip_vec_dQ_dx_9;
    double mip_vec_dQ_dx_10;
    double mip_vec_dQ_dx_11;
    double mip_vec_dQ_dx_12;
    double mip_vec_dQ_dx_13;
    double mip_vec_dQ_dx_14;
    double mip_vec_dQ_dx_15;
    double mip_vec_dQ_dx_16;
    double mip_vec_dQ_dx_17;
    double mip_vec_dQ_dx_18;
    double mip_vec_dQ_dx_19;
    
    // shower pi0 identification
    int pio_flag;
    int pio_mip_id;    // pre-condition to run this tagger
    int pio_filled;
    int pio_flag_pio;  // only when this is true, we run the first part of tagger ...
    // first part of tagger ...
    int pio_1_flag;
    double pio_1_mass;
    int pio_1_pio_type;
    double pio_1_energy_1;
    double pio_1_energy_2;
    double pio_1_dis_1;
    double pio_1_dis_2;
    // second part of tagger
    std::vector<double> pio_2_v_dis2;
    std::vector<double> pio_2_v_angle2;
    std::vector<double> pio_2_v_acc_length;
    std::vector<int> pio_2_v_flag;

    
    //single shower pi0 case ...
    std::vector<double> sig_1_v_angle;
    std::vector<int> sig_1_v_flag_single_shower;
    std::vector<double> sig_1_v_energy;
    std::vector<double> sig_1_v_energy_1;
    std::vector<int> sig_1_v_flag;

    std::vector<double> sig_2_v_energy;
    std::vector<double> sig_2_v_shower_angle;
    std::vector<int> sig_2_v_flag_single_shower;
    std::vector<double> sig_2_v_medium_dQ_dx;
    std::vector<double> sig_2_v_start_dQ_dx;
    std::vector<int> sig_2_v_flag;

    int sig_flag;

     // multiple gamma I
    double mgo_energy;
    double mgo_max_energy;
    double mgo_total_energy;
    int mgo_n_showers;
    double mgo_max_energy_1;
    double mgo_max_energy_2;
    double mgo_total_other_energy;
    int mgo_n_total_showers;
    double mgo_total_other_energy_1;
    int mgo_flag;
    
    // multiple gamma II
    int mgt_flag_single_shower;
    double mgt_max_energy;
    double mgt_energy;
    double mgt_total_other_energy;
    double mgt_max_energy_1;
    double mgt_e_indirect_max_energy;
    double mgt_e_direct_max_energy;
    int mgt_n_direct_showers;
    double mgt_e_direct_total_energy;
    int mgt_flag_indirect_max_pio;
    double mgt_e_indirect_total_energy;
    int mgt_flag;

    // shower to wall
    double stw_1_energy;
    double stw_1_dis;
    double stw_1_dQ_dx;
    int stw_1_flag_single_shower;
    int stw_1_n_pi0;
    int stw_1_num_valid_tracks;
    int stw_1_flag;

    std::vector<double> stw_2_v_medium_dQ_dx;
    std::vector<double> stw_2_v_energy;
    std::vector<double> stw_2_v_angle;
    std::vector<double> stw_2_v_dir_length;
    std::vector<double> stw_2_v_max_dQ_dx;
    std::vector<int> stw_2_v_flag;

    std::vector<double> stw_3_v_angle;
    std::vector<double> stw_3_v_dir_length;
    std::vector<double> stw_3_v_energy;
    std::vector<double> stw_3_v_medium_dQ_dx;
    std::vector<int> stw_3_v_flag;

    std::vector<double> stw_4_v_angle;
    std::vector<double> stw_4_v_dis;
    std::vector<double> stw_4_v_energy;
    std::vector<int> stw_4_v_flag;

    int stw_flag;

    // single photon  cases ...
    int spt_flag_single_shower;
    double spt_energy;
    double spt_shower_main_length;
    double spt_shower_total_length;
    double spt_angle_beam;
    double spt_angle_vertical;
    double spt_max_dQ_dx;
    double spt_angle_beam_1;
    double spt_angle_drift;
    double spt_angle_drift_1;
    int spt_num_valid_tracks;
    double spt_n_vtx_segs;
    double spt_max_length;
    int spt_flag;

    // stem length ...
    double stem_len_energy;
    double stem_len_length;
    int stem_len_flag_avoid_muon_check;
    int stem_len_num_daughters;
    double stem_len_daughter_length;
    int stem_len_flag;

    // low-energy michel
    double lem_shower_total_length;
    double lem_shower_main_length;
    int lem_n_3seg;
    double lem_e_charge;
    double lem_e_dQdx;
    int lem_shower_num_segs;
    int lem_shower_num_main_segs;
    int lem_flag;

     // broken muon ...
    int brm_n_mu_segs;
    double brm_Ep;
    double brm_energy;
    double brm_acc_length;
    double brm_shower_total_length;
    double brm_connected_length;
    int brm_n_size;
    double brm_acc_direct_length;
    int brm_n_shower_main_segs;
    int brm_n_mu_main;
    int brm_flag;


    // compare with muon
    double cme_mu_energy;
    double cme_energy;
    double cme_mu_length;
    double cme_length;
    double cme_angle_beam;
    int cme_flag;


    // angular cut ...
    double anc_energy;
    double anc_angle;
    double anc_max_angle;
    double anc_max_length;
    double anc_acc_forward_length;
    double anc_acc_backward_length;
    double anc_acc_forward_length1;
    double anc_shower_main_length;
    double anc_shower_total_length;
    int anc_flag_main_outside;
    int anc_flag;

        
    // stem direction
    int stem_dir_flag;
    int stem_dir_flag_single_shower;
    int stem_dir_filled;
    double stem_dir_angle;
    double stem_dir_energy;
    double stem_dir_angle1;
    double stem_dir_angle2;
    double stem_dir_angle3;
    double stem_dir_ratio;

     // vertex inside shower
    int vis_1_filled;
    int vis_1_n_vtx_segs;
    double vis_1_energy;
    int vis_1_num_good_tracks;
    double vis_1_max_angle;
    double vis_1_tmp_length1;
    double vis_1_tmp_length2;
    double vis_1_particle_type;
    int vis_1_flag;

    int vis_2_filled;
    int vis_2_n_vtx_segs;
    double vis_2_min_angle;
    int vis_2_min_weak_track;
    double vis_2_angle_beam;
    double vis_2_min_angle1;
    double vis_2_iso_angle1;
    double vis_2_min_medium_dQ_dx;
    double vis_2_min_length;
    double vis_2_sg_length;
    double vis_2_max_angle;
    int vis_2_max_weak_track;
    int vis_2_flag;

    int vis_flag;


    // bad reconstruction_1
    
    int br_filled;
    //bad reconstruction 1_1
    int br1_1_flag;
    int br1_1_shower_type;
    int br1_1_vtx_n_segs;
    double br1_1_energy;
    int br1_1_n_segs;
    int br1_1_flag_sg_topology;
    int br1_1_flag_sg_trajectory;
    double br1_1_sg_length;

    // bad reconstruction 1_2
    int br1_2_flag;
    double br1_2_energy;
    int br1_2_n_connected;
    double br1_2_max_length;
    int br1_2_n_connected_1;
    int br1_2_vtx_n_segs;
    int br1_2_n_shower_segs;
    double br1_2_max_length_ratio;
    double br1_2_shower_length;
    
    // bad_reconstruction 1_3
    int br1_3_flag;
    double br1_3_energy;
    int br1_3_n_connected_p;
    double br1_3_max_length_p;
    int br1_3_n_shower_segs;
    int br1_3_flag_sg_topology;
    int br1_3_flag_sg_trajectory;
    int br1_3_n_shower_main_segs;
    double br1_3_sg_length;


    // bad reconstruction 2
    int br2_flag;
    int br2_flag_single_shower;
    int br2_num_valid_tracks;
    double br2_energy;
    double br2_angle1;
    double br2_angle2;
    double br2_angle;
    double br2_angle3;
    int br2_n_shower_main_segs;
    double br2_max_angle;
    double br2_sg_length;
    int br2_flag_sg_trajectory;


     //bad reconstruction 3
    double br3_1_energy;
    int br3_1_n_shower_segments;
    int br3_1_sg_flag_trajectory;
    double br3_1_sg_direct_length;
    double br3_1_sg_length;
    double br3_1_total_main_length;
    double br3_1_total_length;
    double br3_1_iso_angle;
    int br3_1_sg_flag_topology;
    int br3_1_flag;

    int br3_2_n_ele;
    int br3_2_n_other;
    double br3_2_energy;
    double br3_2_total_main_length;
    double br3_2_total_length;
    int br3_2_other_fid;
    int br3_2_flag;

    std::vector<double> br3_3_v_energy;
    std::vector<double> br3_3_v_angle;
    std::vector<double> br3_3_v_dir_length;
    std::vector<double> br3_3_v_length;
    std::vector<int> br3_3_v_flag;

    double br3_4_acc_length;
    double br3_4_total_length;
    double br3_4_energy;
    int br3_4_flag;

    std::vector<double> br3_5_v_dir_length;
    std::vector<double> br3_5_v_total_length;
    std::vector<int> br3_5_v_flag_avoid_muon_check;
    std::vector<int> br3_5_v_n_seg;
    std::vector<double> br3_5_v_angle;
    std::vector<double> br3_5_v_sg_length;
    std::vector<double> br3_5_v_energy;
    std::vector<int> br3_5_v_n_main_segs;
    std::vector<int> br3_5_v_n_segs;
    std::vector<double> br3_5_v_shower_main_length;
    std::vector<double> br3_5_v_shower_total_length;
    std::vector<int> br3_5_v_flag;

    std::vector<double> br3_6_v_angle;
    std::vector<double> br3_6_v_angle1;
    std::vector<int> br3_6_v_flag_shower_trajectory;
    std::vector<double> br3_6_v_direct_length;
    std::vector<double> br3_6_v_length;
    std::vector<int> br3_6_v_n_other_vtx_segs;
    std::vector<double> br3_6_v_energy;
    std::vector<int> br3_6_v_flag;

    double br3_7_energy;
    double br3_7_min_angle;
    double br3_7_sg_length;
    double br3_7_shower_main_length;
    int br3_7_flag;

    double br3_8_max_dQ_dx;
    double br3_8_energy;
    int br3_8_n_main_segs;
    double br3_8_shower_main_length;
    double br3_8_shower_length;
    int br3_8_flag;
    
    int br3_flag;
    
    // BR 4
    double br4_1_shower_main_length;
    double br4_1_shower_total_length;
    double br4_1_min_dis;
    double br4_1_energy;
    int br4_1_flag_avoid_muon_check;
    int br4_1_n_vtx_segs;
    int br4_1_n_main_segs;
    int br4_1_flag;

    double br4_2_ratio_45;
    double br4_2_ratio_35;
    double br4_2_ratio_25;
    double br4_2_ratio_15;
    double br4_2_energy;
    double br4_2_ratio1_45;
    double br4_2_ratio1_35;
    double br4_2_ratio1_25;
    double br4_2_ratio1_15;
    double br4_2_iso_angle;
    double br4_2_iso_angle1;
    double br4_2_angle;
    int br4_2_flag;

    int br4_flag;

     // track overclustering ..
    std::vector<int> tro_1_v_particle_type;
    std::vector<int> tro_1_v_flag_dir_weak;
    std::vector<double> tro_1_v_min_dis;
    std::vector<double> tro_1_v_sg1_length;
    std::vector<double> tro_1_v_shower_main_length;
    std::vector<int> tro_1_v_max_n_vtx_segs;
    std::vector<double> tro_1_v_tmp_length;
    std::vector<double> tro_1_v_medium_dQ_dx;
    std::vector<double> tro_1_v_dQ_dx_cut;
    std::vector<int> tro_1_v_flag_shower_topology;
    std::vector<int> tro_1_v_flag;

    std::vector<double> tro_2_v_energy;
    std::vector<double> tro_2_v_stem_length;
    std::vector<double> tro_2_v_iso_angle;
    std::vector<double> tro_2_v_max_length;
    std::vector<double> tro_2_v_angle;
    std::vector<int> tro_2_v_flag;

    double tro_3_stem_length;
    int tro_3_n_muon_segs;
    double tro_3_energy;
    int tro_3_flag;

    std::vector<double> tro_4_v_dir2_mag;
    std::vector<double> tro_4_v_angle;
    std::vector<double> tro_4_v_angle1;
    std::vector<double> tro_4_v_angle2;
    std::vector<double> tro_4_v_length;
    std::vector<double> tro_4_v_length1;
    std::vector<double> tro_4_v_medium_dQ_dx;
    std::vector<double> tro_4_v_end_dQ_dx;
    std::vector<double> tro_4_v_energy;
    std::vector<double> tro_4_v_shower_main_length;
    std::vector<int> tro_4_v_flag_shower_trajectory;
    std::vector<int> tro_4_v_flag;

    std::vector<double> tro_5_v_max_angle;
    std::vector<double> tro_5_v_min_angle;
    std::vector<double> tro_5_v_max_length;
    std::vector<double> tro_5_v_iso_angle;
    std::vector<int> tro_5_v_n_vtx_segs;
    std::vector<int> tro_5_v_min_count;
    std::vector<int> tro_5_v_max_count;
    std::vector<double> tro_5_v_energy;
    std::vector<int> tro_5_v_flag;

    int tro_flag;

    // high energy overlap 
    int hol_1_n_valid_tracks;
    double hol_1_min_angle;
    double hol_1_energy;
    int hol_1_flag_all_shower;
    double hol_1_min_length;
    int hol_1_flag;

    double hol_2_min_angle;
    double hol_2_medium_dQ_dx;
    int hol_2_ncount;
    double hol_2_energy;
    int hol_2_flag;

    int hol_flag;
    
    // low-energy overlap ...
    int lol_flag;

    std::vector<double> lol_1_v_energy;
    std::vector<int> lol_1_v_vtx_n_segs;
    std::vector<int> lol_1_v_nseg;
    std::vector<double> lol_1_v_angle;
    std::vector<int> lol_1_v_flag;

    std::vector<double> lol_2_v_length;
    std::vector<double> lol_2_v_angle;
    std::vector<int> lol_2_v_type;
    std::vector<int> lol_2_v_vtx_n_segs;
    std::vector<double> lol_2_v_energy;
    std::vector<double> lol_2_v_shower_main_length;
    std::vector<int> lol_2_v_flag_dir_weak;
    std::vector<int> lol_2_v_flag;

    double lol_3_angle_beam;
    int lol_3_n_valid_tracks;
    double lol_3_min_angle;
    int lol_3_vtx_n_segs;
    double lol_3_energy;
    double lol_3_shower_main_length;
    int lol_3_n_out;
    int lol_3_n_sum;    
    int lol_3_flag;
    
    
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
    NeutrinoID(WCPPID::PR3DCluster *main_cluster, std::vector<WCPPID::PR3DCluster*>& other_clusters, std::vector<WCPPID::PR3DCluster*>& all_clusters, WCPPID::ToyFiducial* fid, WCPSst::GeomDataSource& gds, int nrebin, int frame_length, float unit_dis,	WCP::ToyCTPointCloud* ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, double flash_time, double offset_x, int flag_neutrino_id_process=1);
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
    bool nue_tagger(double muon_kine_energy = 0);
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

    void init_tagger_info();
    TaggerInfo tagger_info; 

    
  protected:
    int neutrino_type;
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
