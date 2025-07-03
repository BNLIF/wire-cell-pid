#ifndef WIRECELLPID_PR3DCLUSTER_H
#define WIRECELLPID_PR3DCLUSTER_H

#include "WCPData/SlimMergeGeomCell.h"
#include "WCPData/ToyPointCloud.h"
#include "WCPSst/GeomDataSource.h"
#include "WCPData/ToyCTPointCloud.h"
#include "WCPData/TrackInfo.h"

#include "WCPPID/Map_Proto_Vertex_Segment.h"

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
    std::vector<bool>& get_flag_tagged_steiner_graph(){return flag_tagged_steiner_graph;};
    
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
    std::pair<double,double> HoughTrans(WCP::Point& p, double dis, WCP::ToyPointCloud *point_cloud1, bool flag_print = false);
    TVector3 VHoughTrans(WCP::Point& p, double dis, WCP::ToyPointCloud *point_cloud1, bool flag_print = false);

    TVector3 calc_PCA_dir(WCP::Point& p, double dis);
    TVector3 calc_PCA_dir(WCP::Point& p, WCP::PointVector& pts);
    
    // graph related ...
    void Create_graph(WCP::ToyPointCloud* ref_point_cloud = 0);
    void Create_graph(WCP::ToyCTPointCloud& ct_point_cloud, WCP::ToyPointCloud* ref_point_cloud = 0);
    void Establish_close_connected_graph();
    void Connect_graph(WCP::ToyPointCloud* ref_point_cloud = 0);
    void Connect_graph(WCP::ToyCTPointCloud& ct_point_cloud, WCP::ToyPointCloud* ref_point_cloud = 0);
    void Del_graph();
    void search_for_connection_isochronous(std::pair<int,double>& result1, WCP::Point& p1, TVector3& dir1, WCP::ToyPointCloud* pcloud1, WCP::ToyPointCloud* pcloud2, double search_distance, double angle_cut, double tran_dis_cut);

    // protect against over clustering
    std::vector<WCP::SMGCSelection> Examine_graph(WCP::ToyCTPointCloud& ct_point_cloud);
    void Connect_graph_overclustering_protection(WCP::ToyCTPointCloud& ct_point_cloud);
    bool check_connectivity(std::tuple<int, int, double>& index_index_dis, WCP::WCPointCloud<double>& cloud, WCP::ToyCTPointCloud& ct_point_cloud, WCP::ToyPointCloud* pc1, WCP::ToyPointCloud* pc2, double step_size = 0.6*units::cm, bool flag_strong_check = false);
    std::vector<bool> check_direction(TVector3& v1);
    
    // Steiner tree
    void create_steiner_graph(WCP::ToyCTPointCloud& ct_point_cloud, WCPSst::GeomDataSource& gds, int nrebin, int frame_length, double unit_dis);
    MCUGraph* Create_steiner_tree(WCP::ToyPointCloud *point_cloud_steiner, std::vector<bool>& flag_steiner_terminal, WCP::GeomDataSource& gds, WCP::SMGCSelection& mcells, bool flag_path = false, bool disable_dead_mix_cell = true);
    std::vector<bool>& get_flag_steiner_terminal(){return flag_steiner_terminal;};
    MCUGraph* get_graph_steiner(){return graph_steiner;};
    MCUGraph* get_graph(){return graph;};

    void calc_num_components();
    
    // example for moresophisticated algorithm later ...
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

    void set_path_wcps( std::list<WCP::WCPointCloud<double>::WCPoint>& list){path_wcps = list;};
    std::list<WCP::WCPointCloud<double>::WCPoint>& get_path_wcps(){return path_wcps;};
    std::list<WCP::SlimMergeGeomCell*>& get_path_mcells(){return path_mcells;};

    // get exterme points ...
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_highest_lowest_wcps(int flag = 1);
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_front_back_wcps(int flag = 1);
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_earliest_latest_wcps(int flag = 1);
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_main_axis_wcps(int flag = 1);

    std::vector<std::vector<WCP::WCPointCloud<double>::WCPoint>> get_extreme_wcps(int flag = 1, std::map<int,WCP::SMGCSelection>* old_time_mcells_map=0);
    std::pair<WCP::WCPointCloud<double>::WCPoint,WCP::WCPointCloud<double>::WCPoint> get_two_boundary_wcps(int flag = 1, bool flag_cosmic = false);
    WCP::WCPointCloud<double>::WCPoint get_local_extension(WCP::WCPointCloud<double>::WCPoint wcp, int flag = 1);
    int get_num_outside_range_points(int flag = 1);
    
    std::pair<WCP::Point,WCP::Point> get_two_extreme_points(int flag = 1);


    
    void dijkstra_shortest_paths(WCP::WCPointCloud<double>::WCPoint& wcp_source, int flag = 1);
    void cal_shortest_path(WCP::WCPointCloud<double>::WCPoint& wcp_target, int flag = 1);

    // projection related
    void get_projection(std::vector<int>& proj_channel, std::vector<int>& proj_timeslice, std::vector<int>& proj_charge, std::vector<int>& proj_charge_err , std::vector<int>& proj_flag, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map);
    void collect_charge_trajectory(WCP::ToyCTPointCloud& ct_point_cloud, double dis_cut = 0.6*units::cm, double range_cut = 1.0*units::cm);
    void collect_charge_multi_trajectory(Map_Proto_Segment_Vertices& map_segment_vertices, WCP::ToyCTPointCloud& ct_point_cloud, double dis_cut = 0.6*units::cm, double range_cut = 1.0*units::cm);

    //fine tracking related ...
    WCP::PointVector& get_fine_tracking_path(){return fine_tracking_path;};
    std::vector<double>& get_dQ(){return dQ;};
    std::vector<double>& get_dx(){return dx;};
    std::vector<double>& get_pu(){return pu;};
    std::vector<double>& get_pv(){return pv;};
    std::vector<double>& get_pw(){return pw;};
    std::vector<double>& get_pt(){return pt;};
    std::vector<double>& get_reduced_chi2(){return reduced_chi2;};
    std::vector<bool>& get_flag_vertex(){return flag_vertex;};
    std::vector<int>& get_sub_cluster_id(){return sub_cluster_id;};
    std::vector<bool>& get_flag_shower(){return flag_shower;};
    std::vector<double>& get_track_rr(){return sub_cluster_rr;};
    
    // main function to do the overall tracking, 
    void do_tracking(WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, double time = 4*units::microsecond, bool flag_dQ_dx_fit_reg = true, bool flag_dQ_dx_fit = true);
    void do_multi_tracking(Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices, WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, double time = 4*units::microsecond, bool flag_dQ_dx_fit_reg = true, bool flag_dQ_dx_fit = true, bool flag_exclusion = false);
    
    // organize path from the shortest path 
    WCP::PointVector organize_wcps_path(WCP::ToyCTPointCloud& ct_point_cloud, std::list<WCP::WCPointCloud<double>::WCPoint>& path_wcps_list,  double low_dis_limit, double end_point_limit);
    void organize_ps_path(WCP::ToyCTPointCloud& ct_point_cloud, WCP::PointVector& ps_vec, double low_dis_limit, double end_point_limit);

    WCP::PointVector examine_end_ps_vec(WCP::ToyCTPointCloud& ct_point_cloud, WCP::PointVector& pts, bool flag_start, bool flag_end);
    
    // prepare data
    void prepare_data(WCP::ToyCTPointCloud& ct_point_cloud, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge);
    void organize_segments_path(WCP::ToyCTPointCloud& ct_point_cloud, Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices,double low_dis_limit, double end_point_limit);
    void organize_segments_path_2nd(WCP::ToyCTPointCloud& ct_point_cloud, Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices,double low_dis_limit, double end_point_limit);
    void organize_segments_path_3rd(WCP::ToyCTPointCloud& ct_point_cloud, Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices,double step_size);

    // form map ...
    void form_map(WCP::ToyCTPointCloud& ct_point_cloud, WCP::PointVector& pts,
		  std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge,
		  std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set,
		  std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set,
		  double end_point_factor=0.6, double mid_point_factor=0.9, int nlevel = 3, double time_cut = 5, double charge_cut = 2000);
    void form_map_multi_segments(Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices, WCP::ToyCTPointCloud& ct_point_cloud,
		  std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge,
				 std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<int, std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, int> >& map_3D_tuple,
				 std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set, bool flag_exclusion,
		  double end_point_factor=0.6, double mid_point_factor=0.9, int nlevel = 3, double time_cut = 5, double charge_cut = 2000);

    void form_point_association(WCP::Point &p, std::set<std::pair<int,int> >& temp_2dut, std::set<std::pair<int,int> >& temp_2dvt, std::set<std::pair<int,int> >& temp_2dwt, WCP::ToyCTPointCloud& ct_point_cloud, double dis_cut, int nlevel = 3, double time_cut = 5);
    void update_association(std::set<std::pair<int,int> >& temp_2dut, std::set<std::pair<int,int> >& temp_2dvt, std::set<std::pair<int,int> >& temp_2dwt, WCPPID::ProtoSegment* sg, WCPPID::ProtoSegmentSelection& segments);
    
    std::vector<float> examine_point_association(std::vector<int>& temp_results, std::set<std::pair<int,int> >& temp_2dut, std::set<std::pair<int,int> >& temp_2dvt, std::set<std::pair<int,int> >& temp_2dwt,
						 std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double,double, int> >& map_2D_wt_charge, bool flag_end_point = false, double charge_cut = 2000);
    
    void trajectory_fit(WCP::PointVector& ps_vec, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set,std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, int charge_div_method = 1, double div_sigma = 0.6*units::cm);
    void multi_trajectory_fit(Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<int, std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, int> >& map_3D_tuple, std::map<std::pair<int,int>,std::set<int>>& map_2DU_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DV_3D_set, std::map<std::pair<int,int>,std::set<int>>& map_2DW_3D_set,std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, int charge_div_method = 1, double div_sigma = 0.6*units::cm);
    WCP::Point get_pos_multi(std::tuple<WCPPID::ProtoVertex*, WCPPID::ProtoSegment*, int>& input);
    WCP::Point fit_point(WCP::Point& init_p, int i,
			 std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set,
			 std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge,
			 std::map<std::tuple<int,int, int>, double>& map_Udiv_fac, std::map<std::tuple<int,int, int>, double>& map_Vdiv_fac, std::map<std::tuple<int,int, int>, double>& map_Wdiv_fac,
			 double offset_t, double slope_x, double offset_u, double slope_yu, double slope_zu, double offset_v, double slope_yv, double slope_zv, double offset_w, double slope_yw, double slope_zw);
    WCP::PointVector examine_trajectory(WCP::PointVector& final_ps_vec, WCP::PointVector& init_ps_vec, std::vector<int>& init_indices, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, double offset_t, double slope_x, double offset_u, double slope_yu, double slope_zu, double offset_v, double slope_yv, double slope_zv, double offset_w, double slope_yw, double slope_zw);
    
    bool skip_trajectory_point(WCP::Point& p_fit, int i, int index, WCP::PointVector& ps_vec, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>, std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, WCP::PointVector& fine_tracking_path,
				 double offset_t, double slope_x, double offset_u, double slope_yu, double slope_zu, double offset_v, double slope_yv, double slope_zv, double offset_w, double slope_yw, double slope_zw);

    
    
    void fill_data_map_trajectory(std::vector<int> indices, std::map<int, std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DU_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DV_set, std::map<int,std::pair<std::set<std::pair<int,int>>, float> >& map_3D_2DW_set, std::map<std::pair<int,int>,  std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge);

    // dQ/dx fit part
    double cal_gaus_integral(int tbin, int wbin, double t_center, double t_sigma, double w_center, double w_sigma, int flag=0, double nsigma = 3);
    double cal_gaus_integral_seg(int tbin, int wbin, std::vector<double>& t_centers, std::vector<double>& t_sigmas, std::vector<double>& w_centers, std::vector<double>& w_sigmas, std::vector<double>& weights, int flag=0, double nsigma = 3);

    void dQ_dx_fit(std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,  std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, double flash_time = 4*units::microsecond, double dis_end_point_ext = 4.5*units::mm, bool flag_dQ_dx_fit_reg = true);
    void dQ_dx_fill(double dis_end_point_ext = 4.5*units::mm);
    void dQ_dx_multi_fit(Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices, std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,  std::tuple<double, double, int > >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_wt_charge, double flash_time = 4*units::microsecond, double dis_end_point_ext = 4.5*units::mm, bool flag_dQ_dx_fit_reg = true);
    
    void update_data_dQ_dx_fit(std::map<int,std::map<const WCP::GeomWire*, WCP::SMGCSelection > >& global_wc_map, std::map<std::pair<int,int>,std::tuple<double, double, int> >& map_2D_ut_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_vt_charge, std::map<std::pair<int,int>, std::tuple<double, double, int> >& map_2D_wt_charge);

    std::vector<std::pair<double,double> > cal_compact_matrix(Eigen::SparseMatrix<double>& MW, Eigen::SparseMatrix<double>& RWT, int n_2D_w, int n_3D_pos, double cut_pos = 2);
    std::vector<std::vector<double> > cal_compact_matrix_multi(std::vector<std::vector<int> >& connected_vec, Eigen::SparseMatrix<double>& MW, Eigen::SparseMatrix<double>& RWT, int n_2D_w, int n_3D_pos, double cut_pos = 2);
    
    
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


    // prepare for the multiple track fitting ...
    WCP::WCPointCloud<double>::WCPoint proto_extend_point(WCP::Point& p, TVector3& dir, TVector3& dir1, bool flag_extend=true);
    bool proto_break_tracks(WCP::WCPointCloud<double>::WCPoint& start_wcp, WCP::WCPointCloud<double>::WCPoint& break_wcp, WCP::WCPointCloud<double>::WCPoint& end_wcp, std::list<WCP::WCPointCloud<double>::WCPoint>& wcp_list1, std::list<WCP::WCPointCloud<double>::WCPoint>& wcp_list2, bool flag_pass_check = false);

    void set_fit_parameters(Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices);
    void set_fit_parameters(ProtoVertexSelection& temp_vertices, ProtoSegmentSelection& temp_segments, Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices);
    void set_fit_parameters(WCPPID::ProtoVertex* vtx);
    void set_fit_parameters(WCPPID::ProtoSegment* seg, int start_n, int end_n);

    void clustering_points_master(Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices, WCP::ToyCTPointCloud& ct_point_cloud, double search_range = 1.2*units::cm, double scaling_2d = 0.7);
    void clustering_points(Map_Proto_Vertex_Segments& map_vertex_segments, Map_Proto_Segment_Vertices& map_segment_vertices, WCP::ToyCTPointCloud& ct_point_cloud, int choice = 1, WCP::ToyPointCloud* pcloud = 0, double search_range = 1.2*units::cm, double scaling_2d = 0.7);

    std::vector<int>& get_point_sub_cluster_ids(){return point_sub_cluster_ids;};
    std::vector<bool>& get_point_flag_showers(){return point_flag_showers;};
    std::vector<int>& get_point_steiner_sub_cluster_ids(){return point_steiner_sub_cluster_ids;};

    std::vector<float> get_time_ch_range(); 
    void fill_2d_charge_dead_chs(std::map<std::pair<int,int>, std::pair<double,double> >& charge_2d_u, std::map<std::pair<int,int>, std::pair<double,double> >& charge_2d_v, std::map<std::pair<int,int>, std::pair<double,double> >& charge_2d_w);
    
    // hack for debug purpose ...
    std::map<WCP::SlimMergeGeomCell*, std::set<int>> temp_cell_point_indices_map(){return cell_point_indices_map;};

  protected:
    
    int cluster_id;
    
    WCP::SMGCSelection mcells;
    std::map<int,WCP::SMGCSet> time_cells_set_map;
    std::map<WCP::SlimMergeGeomCell*, std::set<int>> cell_times_set_map;

    // graph
    MCUGraph *graph;
    WCP::ToyPointCloud *point_cloud;
    std::vector<int> point_sub_cluster_ids;
    std::vector<bool> point_flag_showers;
    
    WCP::Vector center;
    WCP::Vector PCA_axis[3];
    double PCA_values[3];

    


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
    std::vector<int> point_steiner_sub_cluster_ids;
    WCP::ToyPointCloud *point_cloud_steiner_terminal;
    std::vector<bool> flag_steiner_terminal;
    // more derived quantities to come ...
    std::set<int> steiner_graph_terminal_indices;
    std::set<int> steiner_graph_selected_terminal_indices;
    std::vector<bool> flag_tagged_steiner_graph;
    
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
    std::vector<bool> flag_vertex;
    std::vector<int> sub_cluster_id;
    std::vector<bool> flag_shower;
    std::vector<double> sub_cluster_rr;

    WCP::TrackInfoSelection fit_tracks;
    
    std::map<std::pair<int,int>, std::tuple<double,double,double> > proj_data_u_map;
    std::map<std::pair<int,int>, std::tuple<double,double,double> > proj_data_v_map;
    std::map<std::pair<int,int>, std::tuple<double,double,double> > proj_data_w_map;
    
  };
  typedef std::vector<PR3DCluster*> PR3DClusterSelection;
}

#endif
