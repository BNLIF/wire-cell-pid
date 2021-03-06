#ifndef WCPPID_PROTOVERTEX_H
#define WCPPID_PROTOVERTEX_H

#include "WCPData/WCPointCloud.h"

namespace WCPPID{
  class ProtoVertex{
  public:
    ProtoVertex(int id, WCP::WCPointCloud<double>::WCPoint& wcpt, int cluster_id);
    ~ProtoVertex();

    void set_wcpt(WCP::WCPointCloud<double>::WCPoint& tmp_pt);
    WCP::WCPointCloud<double>::WCPoint& get_wcpt() {return wcpt;};

    //get fit flag ...
    bool get_fit_flag(){return flag_fit;};
    void set_fit_flag(bool flag){flag_fit = flag;};

    // after trajectory fit ...
    void set_fit(WCP::Point& tmp_fit_pt, double tmp_dQ, double tmp_dx, double tmp_pu, double tmp_pv, double tmp_pw, double tmp_pt, double tmp_reduced_chi2);
    
    WCP::Point& get_fit_pt() {return fit_pt;};
    // prepare for the fit ...
    void set_fit_pt(WCP::Point& p){fit_pt = p;};
    int get_fit_index(){return fit_index;};
    void set_fit_index(int index){fit_index = index;};
    bool get_flag_fit_fix(){return flag_fit_fix;};
    void set_flag_fit_fix(bool flag){flag_fit_fix = flag;};
    double get_fit_range(){return fit_range;};
    void set_fit_range(double val){fit_range = val;};

    void reset_fit_prop();
    
    // after dQ/dx fit ...
    void set_dQ(double val){dQ = val;};
    double get_dQ(){return dQ;};
    void set_dx(double val){dx = val;};
    double get_dx(){return dx;};
    double get_dQ_dx(){return dQ/(dx+1e-9);};
    void set_pu(double val){pu = val;};
    double get_pu(){return pu;};
    void set_pv(double val){pv = val;};
    double get_pv(){return pv;};
    void set_pw(double val){pw = val;};
    double get_pw(){return pw;};
    void set_pt(double val){pt = val;};
    double get_pt(){return pt;};
    void set_reduced_chi2(double val){reduced_chi2 = val;};
    double get_reduced_chi2(){return reduced_chi2;};
    
    int get_id(){return id;};
    int get_cluster_id(){return cluster_id;};
    
    // get_distance ...  
    double get_dis(WCP::Point& p);
    double get_fit_init_dis();

    void set_neutrino_vertex(bool value){flag_neutrino_vertex=value;};
    bool get_flag_neutrino_vertex(){return flag_neutrino_vertex;};
    
  protected:
    int id;
    int cluster_id;
    bool flag_neutrino_vertex;
    
    WCP::WCPointCloud<double>::WCPoint wcpt; // initial WCP point from the graph ...
    WCP::Point fit_pt; // best fit points ...

    double dQ;
    double dx;
    double pu, pv, pw, pt;
    double reduced_chi2;

    int fit_index;
    bool flag_fit_fix;
    double fit_range;
        
    bool flag_fit;
  };
  typedef std::vector<ProtoVertex*> ProtoVertexSelection;

  struct ProtoVertexCompare{
    bool operator() (ProtoVertex *a, ProtoVertex *b) const {
      
      if (a->get_id() < b->get_id()){
	return true;
      }else if (a->get_id() > b->get_id()){
	return false;
      }else if (a->get_id() == b->get_id()){
	return a < b;
      }
      return a < b;
    }
  };
  
  typedef std::set<ProtoVertex*, ProtoVertexCompare> ProtoVertexSet;
}

#endif
