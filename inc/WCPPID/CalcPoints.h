#ifndef WIRECELLPID_CALC_POINTS_H
#define WIRECELLPID_CALC_POINTS_H

#include "WCPSst/GeomDataSource.h"
#include "WCPData/SlimMergeGeomCell.h"
#include "WCPPID/PR3DCluster.h"

namespace WCPPID{
  void calc_sampling_points(WCP::GeomDataSource& gds, WCPPID::PR3DCluster* cluster, int nrebin, int frame_length, double unit_dis, bool disable_mix_dead_cell = true);
  void calc_sampling_points(WCP::GeomDataSource& gds, WCP::SlimMergeGeomCell* mcell, int nrebin, int frame_length, double unit_dis, bool disable_mix_dead_cell = true);

  void calc_sampling_points_2dtoy(WCP::GeomDataSource& gds, WCP::SlimMergeGeomCell* mcell, int nrebin, int frame_length, double unit_dis);
  
}

#endif 
