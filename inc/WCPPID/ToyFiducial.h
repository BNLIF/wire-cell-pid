#ifndef WIRECELLPID_UBOONE_TOYFIDUCIAL_H
#define WIRECELLPID_UBOONE_TOYFIDUCIAL_H

#include "WCPData/Units.h"
#include "WCPData/Point.h"
#include "WCPData/SlimMergeGeomCell.h"
//#include "WCPData/FlashTPCBundle.h"
#include "WCPData/ToyCTPointCloud.h"
#include "WCPData/LMBDT.h"
#include "WCPData/Opflash.h"
#include "WCPData/PhotonLibrary.h"

#include "WCPPID/PR3DCluster.h"

#include "TVector3.h"
#include "TGraph.h"
#include "TFile.h"
 
#include <vector>
#include <map>

namespace WCPPID{
  class ToyFiducial{
  public:
    ToyFiducial(int dead_region_ch_ext = 3, double offset_t=800, double offset_u=0, double offset_v=0, double offset_w=0, double slope_t=1./2*units::mm, double slope_u=1./(3*units::mm), double slope_v=1./(3*units::mm), double slope_w=1./(3*units::mm), double angle_u=-1.047198, double angle=1.047198, double angle_w=0,
		double boundary_dis_cut=2*units::cm, double top=117*units::cm, double bottom=-116*units::cm, double upstream=0*units::cm, double downstream=1037*units::cm, double anode = 0*units::cm, double cathode=256*units::cm, int flag_data=1);
    ~ToyFiducial();

  
    
    
    void set_offset_t(double value){offset_t=value;};
    
    // helper functions ...
    bool inside_fiducial_volume(WCP::Point& p, double offset_x=0, std::vector<double>* tolerance_vec=NULL);
    bool inside_dead_region(WCP::Point& p);
    bool check_dead_volume(WCP::Point& p, TVector3& dir, double step = 1.0*units::cm, double offset_x=0);
    bool check_signal_processing(WCP::Point& p, TVector3& dir, WCP::ToyCTPointCloud& ct_point_cloud, double step = 1.0*units::cm, double offset_x=0);

    // TGM tagger ...
    bool check_neutrino_candidate(WCPPID::PR3DCluster *main_cluster, WCP::WCPointCloud<double>::WCPoint& wcp1, WCP::WCPointCloud<double>::WCPoint& wcp2, double offset_x, WCP::ToyCTPointCloud& ct_point_cloud, bool flag_2view_check = true);
    bool check_tgm(WCPPID::PR3DCluster* main_cluster,WCP::Opflash* main_flash, double offset_x, WCP::ToyCTPointCloud& ct_point_cloud);
    
    // fully contained tagger ...
    bool check_fully_contained(WCPPID::PR3DCluster* main_cluster, double offset_x, WCP::ToyCTPointCloud& ct_point_cloud);
    
    // London's new tagger
    std::vector<double> get_boundary_SCB_xy_x(WCP::Point& p){
      int index_z = floor(p.z/units::m);
      if(index_z<0){index_z=0;} else if(index_z>9){index_z=9;}
      return boundary_SCB_xy_x_array[index_z];
    }
    std::vector<double> get_boundary_SCB_xy_y(WCP::Point& p){
      int index_z = floor(p.z/units::m);
      if(index_z<0){index_z=0;} else if(index_z>9){index_z=9;}
      return boundary_SCB_xy_y_array[index_z];
    }
    std::vector<double> get_boundary_SCB_xz_x(WCP::Point& p){
      int index_y = floor((p.y/units::cm+116)/24);
      if(index_y<0){index_y=0;} else if(index_y>9){index_y=9;}
      return boundary_SCB_xz_x_array[index_y];
    }
    std::vector<double> get_boundary_SCB_xz_z(WCP::Point& p){
      int index_y = floor((p.y/units::cm+116)/24);
      if(index_y<0){index_y=0;} else if(index_y>9){index_y=9;}
      return boundary_SCB_xz_z_array[index_y];
    }
    bool inside_x_region(std::vector<std::vector<WCP::WCPointCloud<double>::WCPoint>> extreme_points, double offset_x, double x_low, double x_high);

    int convert_xyz_voxel_id(WCP::Point &p);

    int check_boundary(std::vector<std::vector<WCP::WCPointCloud<double>::WCPoint>> extreme_points, double offset_x, std::vector<double>* tol_vec=NULL);

    std::vector<double> calculate_pred_pe(double eventTime, int run_no, double offset_x, WCP::Photon_Library *pl, WCPPID::PR3DCluster* main_cluster, std::vector<WCPPID::PR3DCluster*> additional_clusters, WCP::Opflash* flash, bool flag_match_data, bool flag_timestamp = false);

    void write_debug(int run_no, int subrun_no, int event_no, int tag_type = 0);

    std::tuple<int, WCPPID::PR3DCluster*, WCP::Opflash*> glm_tagger(double eventTime, WCP::OpflashSelection& flashes, WCPPID::PR3DCluster* main_cluster, std::vector<WCPPID::PR3DCluster*> additional_clusters, WCP::Opflash* main_flash, std::tuple<int, double, double, int>& bundle_info, WCP::Photon_Library *pl, int time_offset, int nrebin, float unit_dis, WCP::ToyCTPointCloud& ct_point_cloud, int run_no, int subrun_no, int event_no, bool fully_contained, bool flag_match_data, bool flag_timestamp = false, bool debug_tagger=false);

    // check STM code ...
    bool check_stm(WCPPID::PR3DCluster* cluster, std::vector<WCPPID::PR3DCluster*>& additional_clusters, double offset_x, double flash_time, WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, int& event_type);
    bool check_stm_only(WCPPID::PR3DCluster* cluster, std::vector<WCPPID::PR3DCluster*>& additional_clusters, double offset_x, double flash_time, WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, int& event_type);

    bool eval_stm(WCPPID::PR3DCluster* main_cluster, int kink_num, double peak_range = 40*units::cm, double offset_x = 0*units::cm, double com_range = 35*units::cm, bool flag_strong_check = false);
    bool check_other_tracks(WCPPID::PR3DCluster* main_cluster, double offset_x);
    bool check_other_clusters(WCPPID::PR3DCluster* main_cluster, std::vector<WCPPID::PR3DCluster*>& additional_clusters);

    bool detect_proton(WCPPID::PR3DCluster* main_cluster, int kink_num);

    int find_first_kink(WCPPID::PR3DCluster* main_cluster);

    bool check_full_detector_dead();
    
    void AddDeadRegion(WCP::SlimMergeGeomCell* mcell, std::vector<int>& time_slices);

    WCP::SMGCSelection& get_Dead_mcells(){return mcells;};

    ///////////////////////////////////////////////////// M2

    bool inside1_outside0_SCB(WCP::Point& p, double offset_x, double tolerence_x, double tolerence_y, double tolerence_z);

    std::tuple<int, WCPPID::PR3DCluster*, WCP::Opflash*> M2_cosmic_tagger(double eventTime, WCP::OpflashSelection& flashes, WCPPID::PR3DCluster* main_cluster, std::vector<WCPPID::PR3DCluster*> additional_clusters, WCP::Opflash* main_flash, std::tuple<int, double, double, int>& bundle_info, WCP::Photon_Library *pl, int time_offset, int nrebin, float unit_dis, WCP::ToyCTPointCloud& ct_point_cloud, int run_no, int subrun_no, int event_no, bool flag_data, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, bool flag_timestamp = false, bool debug_tagger=false);
    
    double M2_offset_YX_x(WCP::Point& p);

    bool M2_distance_cut(WCPPID::PR3DCluster* main_cluster, double user_offset_dist, double user_offset_frac);

    bool M2_check_tgm(WCPPID::PR3DCluster* main_cluster, double offset_x, WCP::ToyCTPointCloud& ct_point_cloud);

    bool M2_check_stm(WCPPID::PR3DCluster* main_cluster, std::vector<WCPPID::PR3DCluster*>& additional_clusters, double offset_x, double flash_time, WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map);


    
    
  protected:

    //////////////////////////////////////// M2
    
    double SCB_YX_TOP_x1_array[11], SCB_YX_TOP_y1_array[11];
    double SCB_YX_TOP_x2_array[11], SCB_YX_TOP_y2_array[11];
    double SCB_YX_BOT_x2_array[11], SCB_YX_BOT_y2_array[11];
    double SCB_YX_BOT_x1_array[11], SCB_YX_BOT_y1_array[11];

    double SCB_ZX_TOP_x1_array[11], SCB_ZX_TOP_z1_array[11];
    double SCB_ZX_TOP_x2_array[11], SCB_ZX_TOP_z2_array[11];
    double SCB_ZX_BOT_x2_array[11], SCB_ZX_BOT_z2_array[11];
    double SCB_ZX_BOT_x1_array[11], SCB_ZX_BOT_z1_array[11];

    // boundary
    double m_top; // top distance
    double m_bottom; // bottom distance
    double m_upstream;
    double m_downstream;
    double m_anode;
    double m_cathode;
    
    // space charge boundary
    double m_sc_bottom_1_x, m_sc_bottom_1_y;
    double m_sc_bottom_2_x, m_sc_bottom_2_y;

    double m_sc_top_1_x, m_sc_top_1_y;
    double m_sc_top_2_x, m_sc_top_2_y;

    double m_sc_upstream_1_x, m_sc_upstream_1_z;
    double m_sc_upstream_2_x, m_sc_upstream_2_z;

    double m_sc_downstream_1_x, m_sc_downstream_1_z;
    double m_sc_downstream_2_x, m_sc_downstream_2_z;

    std::vector<double> boundary_xy_x, boundary_xy_y;
    std::vector<double> boundary_xz_x, boundary_xz_z;
    std::vector<double> boundary_SCB_xy_x, boundary_SCB_xy_y;
    std::vector<double> boundary_SCB_xz_x, boundary_SCB_xz_z;

    std::vector<std::vector<double>> boundary_xy_x_array, boundary_xy_y_array;
    std::vector<std::vector<double>> boundary_xz_x_array, boundary_xz_z_array;
    std::vector<std::vector<double>> boundary_SCB_xy_x_array, boundary_SCB_xy_y_array;
    std::vector<std::vector<double>> boundary_SCB_xz_x_array, boundary_SCB_xz_z_array;
    
    // dead regions ... 
    WCP::SMGCSelection mcells;
    std::map<WCP::SlimMergeGeomCell*, std::pair<int,int>> mcell_time_map;
    std::map<int, std::set<WCP::SlimMergeGeomCell*>> ch_mcell_set_map;
    
    // conversion between positions to the channel and time ???

    // convert time into a position
    // (time_slice - offset_t) / slope_t = position_x 
    double offset_t, slope_t;
    // convert u wire number into a position
    // (u_index -offset_u) / slope_u = position_u
    double offset_u, slope_u;
    // convert v wire number into a position
    double offset_v, slope_v;
    // convert w wire number into a position 
    double offset_w, slope_w;
    double angle_u, angle_v, angle_w;

    int dead_region_ch_ext;

    TFile *file;
    TGraph *g_muon;
    TGraph *g_pion;
    TGraph *g_kaon;
    TGraph *g_proton;
    TGraph *g_electron;
    
  };
}

#endif
