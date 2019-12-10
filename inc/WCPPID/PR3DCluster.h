#ifndef WIRECELLPID_PR3DCLUSTER_H
#define WIRECELLPID_PR3DCLUSTER_H

#include "WCPData/SlimMergeGeomCell.h"
#include "WCPData/ToyPointCloud.h"
#include "WCPSst/GeomDataSource.h"
#include "WCPData/ToyCTPointCloud.h"
#include "WCPData/TrackInfo.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <Eigen/Sparse>

using namespace boost;

namespace WCPPID{

  struct VertexProp {
    int index;
    // WCPointCloud<double>::WCPoint wcpoint;
    // add pointer to merged cell
  };
  /* struct EdgeProp { */
  /*   float dist; // edge distance */
  /* }; */

  using EdgeProp = boost::property<boost::edge_weight_t, float>; 
  // using VertexProp = boost::property(boost::vertex_color_t, int>; 
  
  typedef adjacency_list<setS, vecS, undirectedS, VertexProp, EdgeProp> MCUGraph;
  typedef graph_traits<MCUGraph>::vertex_descriptor vertex_descriptor;
  typedef graph_traits<MCUGraph>::edge_descriptor edge_descriptor; 
  

  
  class PR3DCluster{
  public:
    PR3DCluster(int cluster_id);
    ~PR3DCluster();
    void AddCell(WCP::SlimMergeGeomCell* mcell, int time_slice);
    
    int get_cluster_id(){return cluster_id;};
    void set_cluster_id(int value){cluster_id=value;};
    int get_num_mcells(){return mcells.size();};
    int get_num_points(){return point_cloud->get_num_points(); };
    int get_num_time_slices(){return time_cells_set_map.size();};
    std::map<int,WCP::SMGCSet>& get_time_cells_set_map(){return time_cells_set_map;};
    WCP::SMGCSelection& get_mcells(){return mcells;};

    // Point cloud related ...
    void Create_point_cloud(WCP::ToyPointCloud *global_point_cloud = 0);
    void Del_point_cloud();
    WCP::ToyPointCloud* get_point_cloud(){return point_cloud;};
    WCP::ToyPointCloud* get_point_cloud_steiner(){return point_cloud_steiner;};
    WCP::ToyPointCloud* get_point_cloud_steiner_terminal(){return point_cloud_steiner_terminal;};
    
    WCP::Point calc_ave_pos(WCP::Point& p, double dis);
    WCP::Point calc_ave_pos(WCP::Point& p, int N);
    int get_num_points(WCP::Point& p_test, double dis);


    //PCA related ...
    void Calc_PCA();
    bool flag_PCA;
    void Calc_PCA(WCP::PointVector& points);
    WCP::Vector get_center(){return center;};
    WCP::Vector get_PCA_axis(int axis){return PCA_axis[axis];};
    double get_PCA_value(int axis){return PCA_values[axis];};
    
    
    //Hough Transformation ...
    std::pair<double,double> HoughTrans(WCP::Point& p, double dis);
    TVector3 VHoughTrans(WCP::Point& p, double dis);
    std::pair<double,double> HoughTrans(WCP::Point& p, double dis, WCP::ToyPointCloud *point_cloud1);
    TVector3 VHoughTrans(WCP::Point& p, double dis, WCP::ToyPointCloud *point_cloud1);

    TVector3 calc_PCA_dir(WCP::Point& p, double dis);
    TVector3 calc_PCA_dir(WCP::Point& p, WCP::PointVector& pts);
    
    // graph related ...
    void Create_graph(WCP::ToyPointCloud* ref_point_cloud = 0);
    void Create_graph(WCP::ToyCTPointCloud& ct_point_cloud, WCP::ToyPointCloud* ref_point_cloud = 0);
    void Establish_close_connected_graph();
    void Connect_graph(WCP::ToyPointCloud* ref_point_cloud = 0);
    void Connect_graph(WCP::ToyCTPointCloud& ct_point_cloud, WCP::ToyPointCloud* ref_point_cloud = 0);
    void Del_graph();

    // Steiner tree
    void create_steiner_graph(WCP::ToyCTPointCloud& ct_point_cloud, WCPSst::GeomDataSource& gds, int nrebin, int frame_length, double unit_dis);
    MCUGraph* Create_steiner_tree(WCP::ToyPointCloud *point_cloud_steiner, std::vector<bool>& flag_steiner_terminal, WCP::GeomDataSource& gds, WCP::SMGCSelection& mcells, bool flag_path = false, bool disable_dead_mix_cell = true);
    // example for more sophisticated algorithm later ...
    // use the full steiner terminal graph ...
    void recover_steiner_graph();

    void establish_same_mcell_steiner_edges(WCP::GeomDataSource& gds, bool disable_dead_mix_cell = true, int flag=1);
    void remove_same_mcell_steiner_edges(int flag=1);
    
    
    void find_steiner_terminals(WCP::GeomDataSource& gds, bool disable_dead_mix_cell = true);
    void form_cell_points_map();
    std::set<int> get_steiner_terminals(){return steiner_terminal_indices;};
    std::set<int> get_selected_terminals(){return selected_terminal_indices;};

    std::set<int> get_steiner_graph_terminals(){return steiner_graph_terminal_indices;};
    std::set<int> get_steiner_graph_selected_terminals(){return steiner_graph_selected_terminal_indices;};
    
    // find peak points within the mcells ...
    std::set<int> find_peak_point_indices(WCP::SMGCSelection mcells,WCP::GeomDataSource& gds, bool disable_dead_mix_cell = true, int nlevel = 1);

    std::pair<bool,double> calc_charge_wcp(WCP::WCPointCloud<double>::WCPoint& wcp, WCP::GeomDataSource& gds, bool disable_dead_mix_cell = true, double charge_cut = 4000);


    // path related
     // create things for Dijkstra
    std::vector<vertex_descriptor> parents;
    std::vector<int> distances;
    int source_wcp_index;
    int dest_wcp_index;
    
    std::list<WCP::WCPointCloud<double>::WCPoint>& get_path_wcps(){return path_wcps;};
    std::list<WCP::SlimMergeGeomCell*>& get_path_mcells(){return path_mcells;};

    // get exterme points ...
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_highest_lowest_wcps(int flag = 1);
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_front_back_wcps(int flag = 1);
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_earliest_latest_wcps(int flag = 1);
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_main_axis_wcps(int flag = 1);

    std::vector<std::vector<WCP::WCPointCloud<double>::WCPoint>> get_extreme_wcps(int flag = 1, std::map<int,WCP::SMGCSelection>* old_time_mcells_map=0);
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_two_boundary_wcps(int flag = 1, bool flag_cosmic = false);
    
    std::pair<WCP::Point,WCP::Point> get_two_extreme_points(int flag = 1);


    
    void dijkstra_shortest_paths(WCP::WCPointCloud<double>::WCPoint& wcp_source, int flag = 1);
    void cal_shortest_path(WCP::WCPointCloud<double>::WCPoint& wcp_target, int flag = 1);

    // projection related
    void get_projection(std::vector<int>& proj_channel, std::vector<int>& proj_timeslice, std::vector<int>& proj_charge, std::vector<int>& proj_charge_err , std::vector<int>& proj_flag, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map);
    void collect_charge_trajectory(WCP::ToyCTPointCloud& ct_point_cloud, double dis_cut = 0.6*units::cm, double range_cut = 1.0*units::cm);


    //fine tracking related ...
    WCP::PointVector& get_fine_tracking_path(){return fine_tracking_path;};
    std::vector<double>& get_dQ(){return dQ;};
    std::vector<double>& get_dx(){return dx;};
    std::vector<double>& get_pu(){return pu;};
    std::vector<double>& get_pv(){return pv;};
    std::vector<double>& get_pw(){return pw;};
    std::vector<double>& get_pt(){return pt;};
    std::vector<double>& get_reduced_chi2(){return reduced_chi2;};
    
    // main function to do the overall tracking, 
    void do_tracking(WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, double time = 4*units::microsecond, bool flag_dQ_dx_fit_reg = true);
    // organize path from the shortest path 
    WCP::PointVector organize_wcps_path(std::list<WCP::WCPointCloud<double>::WCPoint>& path_wcps_list,  double low_dis_limit, double end_point_limit);
    void organize_ps_path(WCP::PointVector& ps_vec, double low_dis_limit, double end_point_limit);
    // prepare data
    void prepare_data(WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge);
    // form map ...
    void form_map(WCP::ToyCTPointCloud& ct_point_cloud, WCP::PointVector& pts,
		  std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge,
		  std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set,
		  std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set,
		  double end_point_factor=0.6, double mid_point_factor=0.9, int nlevel = 3, double time_cut = 5, double charge_cut = 2000);

    void form_point_association(WCP::Point &p, std::set<std::pair<int,int> >& temp_2dut, std::set<std::pair<int,int> >& temp_2dvt, std::set<std::pair<int,int> >& temp_2dwt, WCP::ToyCTPointCloud& ct_point_cloud, double dis_cut, int nlevel = 3, double time_cut = 5);
    
    std::vector<float> examine_point_association(std::vector<int>& temp_results, std::set<std::pair<int,int> >& temp_2dut, std::set<std::pair<int,int> >& temp_2dvt, std::set<std::pair<int,int> >& temp_2dwt,
						 std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge, bool flag_end_point = false, double charge_cut = 2000);
    
    void trajectory_fit(WCP::PointVector& ps_vec, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set,std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, int charge_div_method = 1, double div_sigma = 0.6*units::cm);

    bool skip_trajectory_point(WCP::Point& p_fit, int i, WCP::PointVector& ps_vec, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, WCP::PointVector& fine_tracking_path,
				 double offset_t, double slope_x, double offset_u, double slope_yu, double slope_zu, double offset_v, double slope_yv, double slope_zv, double offset_w, double slope_yw, double slope_zw);
    
    void fill_data_map_trajectory(std::vector<int> indices, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>,  std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge);

    // dQ/dx fit part
    double cal_gaus_integral(int tbin, int wbin, double t_center, double t_sigma, double w_center, double w_sigma, int flag=0, double nsigma = 3);
    double cal_gaus_integral_seg(int tbin, int wbin, std::vector<double>& t_centers, std::vector<double>& t_sigmas, std::vector<double>& w_centers, std::vector<double>& w_sigmas, std::vector<double>& weights, int flag=0, double nsigma = 3);

    void dQ_dx_fit(std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,  std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, double flash_time = 4*units::microsecond, double dis_end_point_ext = 4.5*units::mm, bool flag_dQ_dx_fit_reg = true);
    void update_data_dQ_dx_fit(std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_wt_charge);

    std::vector<std::pair<double,double> > cal_compact_matrix(Eigen::SparseMatrix<double>& MW, Eigen::SparseMatrix<double>& RWT, int n_2D_w, int n_3D_pos, double cut_pos = 2);
    
    
    std::map<std::pair<int,int>, std::tuple<double,double,double> > & get_proj_data_u_map(){return proj_data_u_map;};
    std::map<std::pair<int,int>, std::tuple<double,double,double> > & get_proj_data_v_map(){return proj_data_v_map;};
    std::map<std::pair<int,int>, std::tuple<double,double,double> > & get_proj_data_w_map(){return proj_data_w_map;};
    // crawl alg for stm
    void do_rough_path(WCP::WCPointCloud<double>::WCPoint& first_wcp, WCP::WCPointCloud<double>::WCPoint& last_wcp);
    WCP::Point adjust_rough_path();
    
    WCP::Point do_stm_crawl(WCP::WCPointCloud<double>::WCPoint& first_wcp, WCP::WCPointCloud<double>::WCPoint& last_wcp, int flag_end = 1);

    WCP::TrackInfoSelection& get_fit_tracks(){return fit_tracks;};
    void clear_fit_tracks();
    void search_other_tracks(WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, double time = 4*units::microsecond, double search_range = 1.5*units::cm, double scaling_2d = 0.8);
    
  protected:
    
    int cluster_id;
    
    WCP::SMGCSelection mcells;
    std::map<int,WCP::SMGCSet> time_cells_set_map;
    std::map<WCP::SlimMergeGeomCell*, std::set<int>> cell_times_set_map;

    WCP::ToyPointCloud *point_cloud;
    WCP::Vector center;
    
    WCP::Vector PCA_axis[3];
    double PCA_values[3];

    // graph
    MCUGraph *graph;


    std::vector<edge_descriptor> same_mcell_steiner_edges;
    std::set<int> excluded_points;
    // saved steiner tree related products
    
    
    // prepare for steiner tree
    std::map<WCP::SlimMergeGeomCell*, std::set<int>> cell_point_indices_map;
    std::set<int> steiner_terminal_indices;
    std::set<int> selected_terminal_indices;

    // data product for Steinter Tree Results
    MCUGraph *graph_steiner;
    WCP::ToyPointCloud *point_cloud_steiner;
    WCP::ToyPointCloud *point_cloud_steiner_terminal;
    std::vector<bool> flag_steiner_terminal;
    // more derived quantities to come ...
    std::set<int> steiner_graph_terminal_indices;
    std::set<int> steiner_graph_selected_terminal_indices;
    
    //path related
    std::list<WCP::WCPointCloud<double>::WCPoint> path_wcps;
    std::list<WCP::SlimMergeGeomCell*> path_mcells;

    // projection related
    std::map<std::pair<int,int>,std::pair<int,int>> collected_charge_map;


     // fine tracking related ...
    WCP::PointVector fine_tracking_path;
    std::vector<double> dQ;
    std::vector<double> dx;
    std::vector<double> pu;
    std::vector<double> pv;
    std::vector<double> pw;
    std::vector<double> pt;
    std::vector<double> reduced_chi2;

    WCP::TrackInfoSelection fit_tracks;
    
    std::map<std::pair<int,int>, std::tuple<double,double,double> > proj_data_u_map;
    std::map<std::pair<int,int>, std::tuple<double,double,double> > proj_data_v_map;
    std::map<std::pair<int,int>, std::tuple<double,double,double> > proj_data_w_map;
    
  };
  typedef std::vector<PR3DCluster*> PR3DClusterSelection;
}

#endif
