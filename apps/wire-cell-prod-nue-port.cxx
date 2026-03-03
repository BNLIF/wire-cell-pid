#include "WCPSst/GeomDataSource.h"

#include "WCPData/SlimMergeGeomCell.h"
#include "WCPData/TPCParams.h"
#include "WCPData/Singleton.h"
#include "WCPData/ToyCTPointCloud.h"

#include "WCPPID/ToyFiducial.h"
#include "WCPPID/CalcPoints.h"
#include "WCPPID/ProtectOverClustering.h"

#include "WCPPID/ExecMon.h"
#include "WCPPID/NeutrinoID.h"

#include "TH1.h"
#include "TFile.h"
#include "TTree.h"

using namespace WCP;
using namespace std;

int main(int argc, char* argv[])
{
  if (argc < 4) {
    cerr << "usage: wire-cell-prod-nue /path/to/ChannelWireGeometry.txt /path/to/matching.root #entry -d[data=0, overlay=1, full mc=2]" << endl;
    return 1;
  }
  TH1::AddDirectory(kFALSE);

  bool flag_debug_output = true; // output
  int datatier = 0; // data=0, overlay=1, full mc=2
  int flag_calib_corr = 1;
  int flag_neutrino_id_process = 1; // 1 for development chain, 2 for current frozen chain
  int flag_bdt = 1; // default 1 xgboost, 2 for TMVA
  bool flag_dl_vtx = true;
  bool flag_PosEfield_corr = false;// Position and E-field correction for SCE
  bool flag_timestamp = false;

  float dl_vtx_cut = 2.0;

  bool flag_lifetime_corr = true;

  int flag_ssmsp = 0; //Save spacepoints for ssm tagger, 0 only when tagged, 1 always, -1 never  

  for (Int_t i=1;i!=argc;i++){
    switch(argv[i][1]){
    case 'a':
      flag_lifetime_corr = atoi(&argv[i][2]);
      break;
    case 'd':
      datatier = atoi(&argv[i][2]);
      break;
    case 'q':
      flag_calib_corr = atoi(&argv[i][2]);
      break;
    case 'n':
      flag_neutrino_id_process = atoi(&argv[i][2]);
      break;
    case 'b':
      flag_bdt = atoi(&argv[i][2]);
      break;
    case 'l':
      flag_dl_vtx = atoi(&argv[i][2]);
      break;
    case 'c':
      dl_vtx_cut = atof(&argv[i][2]);
      break;
    case 'p':
      flag_PosEfield_corr = atoi(&argv[i][2]);
      break;
    case 'z':
      flag_timestamp = atoi(&argv[i][2]);
      break;
    case 's':
      flag_ssmsp = atoi(&argv[i][2]);
      break;
    }
  }
  dl_vtx_cut = dl_vtx_cut * units::cm;

  int flag_data = 1; // data
  if(datatier==1 || datatier==2) flag_data=0; // overlay, full mc
  bool flag_match_data = true;
  if (datatier == 2) flag_match_data = false; // if MC we do not take into account the dead PMT

  WCPPID::ExecMon em("starting");
  cout << em("load geometry") << endl;

  WCPSst::GeomDataSource gds(argv[1]);
  std::vector<double> ex = gds.extent();
  cout << "Extent: "
       << " x:" << ex[0]/units::mm << " mm"
       << " y:" << ex[1]/units::m << " m"
       << " z:" << ex[2]/units::m << " m"
       << endl;
  cout << "Pitch: " << gds.pitch(WirePlaneType_t(0))
       << " " << gds.pitch(WirePlaneType_t(1))
       << " " << gds.pitch(WirePlaneType_t(2))
       << endl;
  cout << "Angle: " << gds.angle(WirePlaneType_t(0))
       << " " << gds.angle(WirePlaneType_t(1))
       << " " << gds.angle(WirePlaneType_t(2))
       << endl;
  std::cout << argv[2] << " " << argv[3] << std::endl;
  cout<<endl<<" --> flag_PosEfield_corr "<<flag_PosEfield_corr<<endl<<endl;

  TString filename = argv[2];
  int entry_no = atoi(argv[3]);

  TFile *file = new TFile(filename);
  TTree *Trun = (TTree*)file->Get("Trun");

  if (entry_no >=Trun->GetEntries()) return 0;
  double eventTime = 0;
  int run_no, subrun_no, event_no;
  int time_offset;
  int nrebin;
  int frame_length;
  int eve_num;
  float unit_dis;

  // get electron lifetime
  
  Float_t elifetime = 1000; // large number 
  if (Trun->GetBranch("elifetime") && flag_lifetime_corr){
    Trun->SetBranchAddress("elifetime",&elifetime);
  }

  
  std::vector<int> *timesliceId = new std::vector<int>;
  std::vector<std::vector<int>> *timesliceChannel = new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *raw_charge = new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *raw_charge_err = new std::vector<std::vector<int>>;

  Trun->SetBranchAddress("eventNo",&event_no);
  Trun->SetBranchAddress("runNo",&run_no);
  Trun->SetBranchAddress("subRunNo",&subrun_no);

  if (Trun->GetBranch("eventTime")){
    Trun->SetBranchAddress("eventTime",&eventTime);
  }else{
    eventTime = 0;
    flag_timestamp = false;
  }



  unsigned int triggerbits;
  Trun->SetBranchAddress("triggerBits",&triggerbits);
  Trun->SetBranchAddress("unit_dis",&unit_dis);
  Trun->SetBranchAddress("frame_length",&frame_length);
  Trun->SetBranchAddress("eve_num",&eve_num);
  Trun->SetBranchAddress("nrebin",&nrebin);
  Trun->SetBranchAddress("time_offset",&time_offset);

  Trun->SetBranchAddress("timesliceId",&timesliceId);
  Trun->SetBranchAddress("timesliceChannel",&timesliceChannel);
  Trun->SetBranchAddress("raw_charge",&raw_charge);
  Trun->SetBranchAddress("raw_charge_err",&raw_charge_err);

  Trun->GetEntry(entry_no);



  double lowerwindow = 0;
  double upperwindow = 0;
  if((triggerbits>>11) & 1U) { lowerwindow=3.1875; upperwindow=4.96876;} // bnb
  if(((triggerbits>>12) & 1U)){ lowerwindow=4.9295; upperwindow=16.6483; } // NUMI
  if(((triggerbits>>9) & 1U) && time_offset != 5) { lowerwindow=3.5625; upperwindow=5.34376; } // extbnb
  if (((triggerbits>>9) & 1U) && time_offset == 5){ lowerwindow=5.3045; upperwindow=17.0233;}


   // define singleton ...
  TPCParams& mp = Singleton<TPCParams>::Instance();

  double pitch_u = gds.pitch(WirePlaneType_t(0));
  double pitch_v = gds.pitch(WirePlaneType_t(1));
  double pitch_w = gds.pitch(WirePlaneType_t(2));
  double time_slice_width = nrebin * unit_dis * 0.5 * units::mm;

  double angle_u = gds.angle(WirePlaneType_t(0));
  double angle_v = gds.angle(WirePlaneType_t(1));
  double angle_w = gds.angle(WirePlaneType_t(2));

  mp.set_pitch_u(pitch_u);
  mp.set_pitch_v(pitch_v);
  mp.set_pitch_w(pitch_w);
  mp.set_angle_u(angle_u);
  mp.set_angle_v(angle_v);
  mp.set_angle_w(angle_w);
  mp.set_ts_width(time_slice_width);
  if (flag_calib_corr==1)
    mp.init_corr_files();
  mp.init_PID_dq_dx();   // default
  //mp.init_PID_dq_dx("input_data_files/stopping_ave_dQ_dx_v2.root");

  mp.init_Pos_Efield_SCE_correction();

  if( flag_PosEfield_corr ) {
    mp.set_flag_PosEfield_corr(true);
    std::cout<<std::endl<<" --> mp.get_flag_PosEfield_corr() "<<mp.get_flag_PosEfield_corr()<<std::endl<<std::endl;

    // search the following in ~/PID/src
    // get_flag_PosEfield_corr()

  }

   // test geometry ...
  const GeomWire *uwire = gds.by_planeindex(WirePlaneType_t(0),0);
  const GeomWire *vwire = gds.by_planeindex(WirePlaneType_t(1),0);
  const GeomWire *wwire = gds.by_planeindex(WirePlaneType_t(2),0);
  double first_u_dis = gds.wire_dist(*uwire) ; // first U wire center ...
  double first_v_dis = gds.wire_dist(*vwire) ; // first V wire center ...
  double first_w_dis = gds.wire_dist(*wwire) ; // first W wire center ...

  mp.set_first_u_dis(first_u_dis);
  mp.set_first_v_dis(first_v_dis);
  mp.set_first_w_dis(first_w_dis);

  if (elifetime < 1000){
    // read the variable from the Trun tree ...
    mp.set_electron_lifetime(elifetime);

    std::cout << "Electron Lifetime Read in: " << elifetime << " ms" << std::endl;
  }
  
  
  std::map<int,std::pair<double,double>> dead_u_index;
  std::map<int,std::pair<double,double>> dead_v_index;
  std::map<int,std::pair<double,double>> dead_w_index;

  bool match_isFC = false;
  TTree *T_eval = (TTree*)file->Get("T_eval");
  Int_t temp_run_no, temp_subrun_no, temp_event_no;
  float lm_cluster_length = -999;
  T_eval->SetBranchAddress("match_isFC",&match_isFC);
  T_eval->SetBranchAddress("run",&temp_run_no);
  T_eval->SetBranchAddress("subrun",&temp_subrun_no);
  T_eval->SetBranchAddress("event",&temp_event_no);
  if(T_eval->GetBranch("lm_cluster_length")){
    T_eval->SetBranchAddress("lm_cluster_length",&lm_cluster_length);
  }
  for (Int_t i = 0; i!= T_eval->GetEntries();i++){
    T_eval->GetEntry(i);
    if (temp_run_no!=run_no || temp_subrun_no!=subrun_no || temp_event_no != event_no) {
      match_isFC = false;
      continue;
    }
    break;
  }

  std::cout << "A: " << match_isFC << std::endl;


  TTree *T_flash = (TTree*)file->Get("T_flash");
  Double_t time;
  Double_t low_time, high_time;
  Int_t type;
  Int_t flash_id;
  //  Int_t temp_run_no, temp_subrun_no, temp_event_no;
  Double_t total_PE;
  Double_t PE[32],PE_err[32];
  // std::vector<int> matched_tpc_ids;

  T_flash->SetBranchAddress("runNo",&temp_run_no);
  T_flash->SetBranchAddress("subRunNo",&temp_subrun_no);
  T_flash->SetBranchAddress("eventNo",&temp_event_no);
  T_flash->SetBranchAddress("time",&time);
  T_flash->SetBranchAddress("type",&type);
  T_flash->SetBranchAddress("flash_id",&flash_id);
  T_flash->SetBranchAddress("low_time",&low_time);
  T_flash->SetBranchAddress("high_time",&high_time);
  T_flash->SetBranchAddress("total_PE",&total_PE);
  T_flash->SetBranchAddress("PE",PE);
  T_flash->SetBranchAddress("PE_err",PE_err);
  std::vector<int> *fired_channels = new std::vector<int>;
  std::vector<double> *l1_fired_time = new std::vector<double>;
  std::vector<double> *l1_fired_pe = new std::vector<double>;
  T_flash->SetBranchAddress("fired_channels",&fired_channels);
  T_flash->SetBranchAddress("l1_fired_time",&l1_fired_time);
  T_flash->SetBranchAddress("l1_fired_pe",&l1_fired_pe);

  TTree *T_match = (TTree*)file->Get("T_match");
  T_match->SetBranchAddress("runNo",&temp_run_no);
  T_match->SetBranchAddress("subRunNo",&temp_subrun_no);
  T_match->SetBranchAddress("eventNo",&temp_event_no);
  Int_t tpc_cluster_id;
  Int_t event_type;
  T_match->SetBranchAddress("tpc_cluster_id",&tpc_cluster_id); // parent cluster id
  T_match->SetBranchAddress("flash_id",&flash_id);  // flash id
  T_match->SetBranchAddress("event_type",&event_type);

  Double_t strength;
  Double_t pe_pred[32];
  Double_t pe_meas[32];
  Double_t pe_meas_err[32];
  T_match->SetBranchAddress("strength",&strength);
  T_match->SetBranchAddress("pe_pred",pe_pred);
  T_match->SetBranchAddress("pe_meas",pe_meas);
  T_match->SetBranchAddress("pe_meas_err",pe_meas_err);
  T_match->SetBranchAddress("event_type",&event_type);
  bool flag_close_to_PMT;
  bool flag_at_x_boundary;
  double ks_dis;
  double chi2;
  int ndf;
  double cluster_length;

  T_match->SetBranchAddress("ks_dis",&ks_dis);
  T_match->SetBranchAddress("chi2",&chi2);
  T_match->SetBranchAddress("ndf",&ndf);
  T_match->SetBranchAddress("cluster_length",&cluster_length);


  //  std::map<int, std::pair<int, double> > map_flash_info;
  std::map<int, Opflash* > map_flash_info;
  std::map<int, int> map_flash_tpc_ids;
  std::map<int, int> map_tpc_flash_ids;
  std::map<std::pair<int, int>, int> map_flash_tpc_pair_type;

  OpflashSelection flashes;
  for (int i=0;i!=T_flash->GetEntries();i++){
    T_flash->GetEntry(i);
    if (temp_run_no!=run_no || temp_subrun_no!=subrun_no || temp_event_no != event_no) continue;
    Opflash *flash = new Opflash(type, flash_id, low_time, high_time, time, total_PE, *fired_channels, PE, PE_err, *l1_fired_time, *l1_fired_pe);
    map_flash_info[flash_id] = flash;
    //    map_flash_info[flash_id] = std::make_pair(type, time);
  }
  for (int i=0;i!=T_match->GetEntries();i++){
    T_match->GetEntry(i);
    if (temp_run_no!=run_no || temp_subrun_no!=subrun_no || temp_event_no != event_no) continue;
    if (flash_id==-1) continue;
    map_flash_tpc_ids[flash_id] = tpc_cluster_id;
    map_tpc_flash_ids[tpc_cluster_id] = flash_id;
    map_flash_tpc_pair_type[std::make_pair(flash_id, tpc_cluster_id)] = event_type;
  }
  // no need to reconstruct flash ...


  // load mcell
  TTree *TC = (TTree*)file->Get("TC");
  std::vector<int> *cluster_id_vec = new std::vector<int>;
  std::vector<int> *parent_cluster_id = new std::vector<int>;
  std::vector<int> *time_slice_vec = new std::vector<int>;
  std::vector<double> *q_vec = new std::vector<double>;
  std::vector<double> *uq_vec = new std::vector<double>;
  std::vector<double> *vq_vec = new std::vector<double>;
  std::vector<double> *wq_vec = new std::vector<double>;
  std::vector<double> *udq_vec = new std::vector<double>;
  std::vector<double> *vdq_vec = new std::vector<double>;
  std::vector<double> *wdq_vec = new std::vector<double>;

  std::vector<int> *nwire_u_vec = new  std::vector<int>;
  std::vector<int> *nwire_v_vec = new  std::vector<int>;
  std::vector<int> *nwire_w_vec = new  std::vector<int>;
  std::vector<int> *flag_u_vec = new  std::vector<int>;
  std::vector<int> *flag_v_vec = new  std::vector<int>;
  std::vector<int> *flag_w_vec = new  std::vector<int>;

  std::vector<std::vector<int>> *wire_index_u_vec = new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *wire_index_v_vec = new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *wire_index_w_vec = new std::vector<std::vector<int>>;
  std::vector<std::vector<double>> *wire_charge_u_vec = new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> *wire_charge_v_vec = new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> *wire_charge_w_vec = new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> *wire_charge_err_u_vec = new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> *wire_charge_err_v_vec = new std::vector<std::vector<double>>;
  std::vector<std::vector<double>> *wire_charge_err_w_vec = new std::vector<std::vector<double>>;

  TC->SetBranchAddress("cluster_id",&cluster_id_vec);
  TC->SetBranchAddress("parent_cluster_id",&parent_cluster_id);
  TC->SetBranchAddress("time_slice",&time_slice_vec);
  TC->SetBranchAddress("q",&q_vec);
  TC->SetBranchAddress("uq",&uq_vec);
  TC->SetBranchAddress("vq",&vq_vec);
  TC->SetBranchAddress("wq",&wq_vec);
  TC->SetBranchAddress("udq",&udq_vec);
  TC->SetBranchAddress("vdq",&vdq_vec);
  TC->SetBranchAddress("wdq",&wdq_vec);
  TC->SetBranchAddress("nwire_u",&nwire_u_vec);
  TC->SetBranchAddress("nwire_v",&nwire_v_vec);
  TC->SetBranchAddress("nwire_w",&nwire_w_vec);
  TC->SetBranchAddress("flag_u",&flag_u_vec);
  TC->SetBranchAddress("flag_v",&flag_v_vec);
  TC->SetBranchAddress("flag_w",&flag_w_vec);
  TC->SetBranchAddress("wire_index_u",&wire_index_u_vec);
  TC->SetBranchAddress("wire_index_v",&wire_index_v_vec);
  TC->SetBranchAddress("wire_index_w",&wire_index_w_vec);
  TC->SetBranchAddress("wire_charge_u",&wire_charge_u_vec);
  TC->SetBranchAddress("wire_charge_v",&wire_charge_v_vec);
  TC->SetBranchAddress("wire_charge_w",&wire_charge_w_vec);
  TC->SetBranchAddress("wire_charge_err_u",&wire_charge_err_u_vec);
  TC->SetBranchAddress("wire_charge_err_v",&wire_charge_err_v_vec);
  TC->SetBranchAddress("wire_charge_err_w",&wire_charge_err_w_vec);

  //load mcell
  TTree *TDC = (TTree*)file->Get("TDC");
  std::vector<int> *ntime_slice_vec = new std::vector<int>;
  std::vector<std::vector<int>> *time_slices_vec = new std::vector<std::vector<int>>;
  TDC->SetBranchAddress("cluster_id",&cluster_id_vec);
  TDC->SetBranchAddress("ntime_slice",&ntime_slice_vec);
  TDC->SetBranchAddress("time_slice",&time_slices_vec);

  TDC->SetBranchAddress("nwire_u",&nwire_u_vec);
  TDC->SetBranchAddress("nwire_v",&nwire_v_vec);
  TDC->SetBranchAddress("nwire_w",&nwire_w_vec);
  TDC->SetBranchAddress("flag_u",&flag_u_vec);
  TDC->SetBranchAddress("flag_v",&flag_v_vec);
  TDC->SetBranchAddress("flag_w",&flag_w_vec);
  TDC->SetBranchAddress("wire_index_u",&wire_index_u_vec);
  TDC->SetBranchAddress("wire_index_v",&wire_index_v_vec);
  TDC->SetBranchAddress("wire_index_w",&wire_index_w_vec);

  WCPPID::ToyFiducial *fid = new WCPPID::ToyFiducial(3,800,-first_u_dis/pitch_u, -first_v_dis/pitch_v, -first_w_dis/pitch_w,
								   1./time_slice_width, 1./pitch_u, 1./pitch_v, 1./pitch_w, // slope
								   angle_u,angle_v,angle_w,// angle
								   3*units::cm, 117*units::cm, -116*units::cm, 0*units::cm, 1037*units::cm, 0*units::cm, 256*units::cm, flag_data);

  // load cells ...
  GeomCellSelection mcells;
  //  CellIndexMap map_mcell_cluster_id;
  WCPPID::PR3DClusterSelection live_clusters;
  WCPPID::PR3DCluster *cluster;
  std::map<WCPPID::PR3DCluster*, int> map_cluster_parent_id; // cluster to main cluster
  std::map<int, std::vector<WCPPID::PR3DCluster*> > map_parentid_clusters; // main cluster to clusters

  int prev_cluster_id=-1;
  int ident = 0;
  TC->GetEntry(entry_no);
  for (int i=0;i!=cluster_id_vec->size();i++){
    int cluster_id = cluster_id_vec->at(i);
    SlimMergeGeomCell *mcell = new SlimMergeGeomCell(ident);
    int time_slice = time_slice_vec->at(i);
    std::vector<int> wire_index_u = wire_index_u_vec->at(i);
    std::vector<int> wire_index_v = wire_index_v_vec->at(i);
    std::vector<int> wire_index_w = wire_index_w_vec->at(i);
    std::vector<double> wire_charge_u = wire_charge_u_vec->at(i);
    std::vector<double> wire_charge_v = wire_charge_v_vec->at(i);
    std::vector<double> wire_charge_w = wire_charge_w_vec->at(i);
    std::vector<double> wire_charge_err_u = wire_charge_err_u_vec->at(i);
    std::vector<double> wire_charge_err_v = wire_charge_err_v_vec->at(i);
    std::vector<double> wire_charge_err_w = wire_charge_err_w_vec->at(i);

    mcell->SetTimeSlice(time_slice);

    mcell->set_uq(uq_vec->at(i));
    mcell->set_vq(vq_vec->at(i));
    mcell->set_wq(wq_vec->at(i));

    mcell->set_udq(udq_vec->at(i));
    mcell->set_vdq(vdq_vec->at(i));
    mcell->set_wdq(wdq_vec->at(i));

    mcell->set_q(q_vec->at(i));

    double temp_x = (time_slice*nrebin/2.*unit_dis/10. - frame_length/2.*unit_dis/10.) * units::cm;

    if (flag_u_vec->at(i)==0){
      mcell->add_bad_planes(WirePlaneType_t(0));
      for (int j=0;j!=nwire_u_vec->at(i);j++){
	if (dead_u_index.find(wire_index_u.at(j))==dead_u_index.end()){
	  dead_u_index[wire_index_u.at(j)] = std::make_pair(temp_x-0.1*units::cm,temp_x+0.1*units::cm);
	}else{
	  if (temp_x-0.1*units::cm < dead_u_index[wire_index_u.at(j)].first){
	    dead_u_index[wire_index_u.at(j)].first = temp_x - 0.1*units::cm;
	  }else if (temp_x+0.1*units::cm > dead_u_index[wire_index_u.at(j)].second){
	    dead_u_index[wire_index_u.at(j)].second = temp_x + 0.1*units::cm;
	  }
	}

      }
    }
    if (flag_v_vec->at(i)==0){
      mcell->add_bad_planes(WirePlaneType_t(1));
      for (int j=0;j!=nwire_v_vec->at(i);j++){
	if (dead_v_index.find(wire_index_v.at(j))==dead_v_index.end()){
	  dead_v_index[wire_index_v.at(j)] = std::make_pair(temp_x-0.1*units::cm,temp_x+0.1*units::cm);
	}else{
	  if (temp_x-0.1*units::cm < dead_v_index[wire_index_v.at(j)].first){
	    dead_v_index[wire_index_v.at(j)].first = temp_x-0.1*units::cm;
	  }else if (temp_x+0.1*units::cm > dead_v_index[wire_index_v.at(j)].second){
	    dead_v_index[wire_index_v.at(j)].second = temp_x + 0.1*units::cm;
	  }
	}
      }
    }
    if (flag_w_vec->at(i)==0){
      mcell->add_bad_planes(WirePlaneType_t(2));
      for (int j=0;j!=nwire_w_vec->at(i);j++){
	if (dead_w_index.find(wire_index_w.at(j))==dead_w_index.end()){
	  dead_w_index[wire_index_w.at(j)] = std::make_pair(temp_x-0.1*units::cm,temp_x+0.1*units::cm);
	}else{
	  if (temp_x-0.1*units::cm < dead_w_index[wire_index_w.at(j)].first){
	    dead_w_index[wire_index_w.at(j)].first = temp_x-0.1*units::cm;
	  }else if (temp_x+0.1*units::cm > dead_w_index[wire_index_w.at(j)].second){
	    dead_w_index[wire_index_w.at(j)].second = temp_x + 0.1*units::cm;
	  }
	}
      }
    }
    for (int j=0;j!=nwire_u_vec->at(i);j++){
      const GeomWire *wire = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(j));
      mcell->AddWire(wire,WirePlaneType_t(0),wire_charge_u.at(j),wire_charge_err_u.at(j));
    }
    for (int j=0;j!=nwire_v_vec->at(i);j++){
      const GeomWire *wire = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(j));
      mcell->AddWire(wire,WirePlaneType_t(1),wire_charge_v.at(j),wire_charge_err_v.at(j));
    }
    for (int j=0;j!=nwire_w_vec->at(i);j++){
      const GeomWire *wire = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(j));
      mcell->AddWire(wire,WirePlaneType_t(2),wire_charge_w.at(j),wire_charge_err_w.at(j));
    }
    mcells.push_back(mcell);

    if (cluster_id != prev_cluster_id){
      cluster = new WCPPID::PR3DCluster(cluster_id);
      map_cluster_parent_id[cluster] = parent_cluster_id->at(i);
      live_clusters.push_back(cluster);
      if (map_parentid_clusters.find(parent_cluster_id->at(i)) == map_parentid_clusters.end()){
	std::vector<WCPPID::PR3DCluster*> temp_clusters;
	temp_clusters.push_back(cluster);
	map_parentid_clusters[parent_cluster_id->at(i)] = temp_clusters;
      }else{
	map_parentid_clusters[parent_cluster_id->at(i)].push_back(cluster);
      }
    }
    cluster->AddCell(mcell,time_slice);

    prev_cluster_id = cluster_id;
    ident++;
  }

  prev_cluster_id = -1;
  cluster_id_vec->clear();
  wire_index_u_vec->clear();
  wire_index_v_vec->clear();
  wire_index_w_vec->clear();
  nwire_u_vec->clear();
  nwire_v_vec->clear();
  nwire_w_vec->clear();
  flag_u_vec->clear();
  flag_v_vec->clear();
  flag_w_vec->clear();


  TDC->GetEntry(entry_no);
  for (int i=0;i!=cluster_id_vec->size();i++){
    int cluster_id = cluster_id_vec->at(i);
    SlimMergeGeomCell *mcell = new SlimMergeGeomCell(ident);
    std::vector<int> time_slices = time_slices_vec->at(i);
    std::vector<int> wire_index_u = wire_index_u_vec->at(i);
    std::vector<int> wire_index_v = wire_index_v_vec->at(i);
    std::vector<int> wire_index_w = wire_index_w_vec->at(i);

    mcell->SetTimeSlice(time_slices.at(0));

    double temp_x1 = (time_slices.front()*nrebin/2.*unit_dis/10. - frame_length/2.*unit_dis/10.) * units::cm;
    double temp_x2 = (time_slices.back()*nrebin/2.*unit_dis/10. - frame_length/2.*unit_dis/10.) * units::cm;

    if (flag_u_vec->at(i)==0){
      mcell->add_bad_planes(WirePlaneType_t(0));
      for (int j=0;j!=nwire_u_vec->at(i);j++){
	if (dead_u_index.find(wire_index_u.at(j))==dead_u_index.end()){
	  dead_u_index[wire_index_u.at(j)] = std::make_pair(temp_x1-0.1*units::cm,temp_x2+0.1*units::cm);
	}else{
	  if (temp_x1-0.1*units::cm < dead_u_index[wire_index_u.at(j)].first){
	    dead_u_index[wire_index_u.at(j)].first = temp_x1 - 0.1*units::cm;
	  }else if (temp_x2+0.1*units::cm > dead_u_index[wire_index_u.at(j)].second){
	    dead_u_index[wire_index_u.at(j)].second = temp_x2 + 0.1*units::cm;
	  }
	}
      }
    }
    if (flag_v_vec->at(i)==0){
      mcell->add_bad_planes(WirePlaneType_t(1));
      for (int j=0;j!=nwire_v_vec->at(i);j++){
	if (dead_v_index.find(wire_index_v.at(j))==dead_v_index.end()){
	  dead_v_index[wire_index_v.at(j)] = std::make_pair(temp_x1-0.1*units::cm,temp_x2+0.1*units::cm);
	}else{
	  if (temp_x1-0.1*units::cm < dead_v_index[wire_index_v.at(j)].first){
	    dead_v_index[wire_index_v.at(j)].first = temp_x1 - 0.1*units::cm;
	  }else if (temp_x2+0.1*units::cm > dead_v_index[wire_index_v.at(j)].second){
	    dead_v_index[wire_index_v.at(j)].second = temp_x2 + 0.1*units::cm;
	  }
	}
      }
    }
    if (flag_w_vec->at(i)==0){
      mcell->add_bad_planes(WirePlaneType_t(2));
      for (int j=0;j!=nwire_w_vec->at(i);j++){
	if (dead_w_index.find(wire_index_w.at(j))==dead_w_index.end()){
	  dead_w_index[wire_index_w.at(j)] = std::make_pair(temp_x1-0.1*units::cm,temp_x2+0.1*units::cm);
	}else{
	  if (temp_x1-0.1*units::cm < dead_w_index[wire_index_w.at(j)].first){
	    dead_w_index[wire_index_w.at(j)].first = temp_x1 - 0.1*units::cm;
	  }else if (temp_x2+0.1*units::cm > dead_w_index[wire_index_w.at(j)].second){
	    dead_w_index[wire_index_w.at(j)].second = temp_x2 + 0.1*units::cm;
	  }
	}
      }
    }
    for (int j=0;j!=nwire_u_vec->at(i);j++){
      const GeomWire *wire = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(j));
      mcell->AddWire(wire,WirePlaneType_t(0));
    }
    for (int j=0;j!=nwire_v_vec->at(i);j++){
      const GeomWire *wire = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(j));
      mcell->AddWire(wire,WirePlaneType_t(1));
    }
    for (int j=0;j!=nwire_w_vec->at(i);j++){
      const GeomWire *wire = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(j));
      mcell->AddWire(wire,WirePlaneType_t(2));
    }

    // figure out the boundary here ...
    PointVector pcell;
    if (flag_u_vec->at(i)==0 && flag_v_vec->at(i)==0){
      const GeomWire *uwire_1 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.front());
      const GeomWire *uwire_2 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.back());
      float dis_u[3];
      float u_pitch = gds.pitch(kUwire);
      dis_u[0] = gds.wire_dist(*uwire_1) - u_pitch/2.;
      dis_u[1] = gds.wire_dist(*uwire_2) + u_pitch/2.;

      const GeomWire *vwire_1 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.front());
      const GeomWire *vwire_2 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.back());
      float dis_v[3];
      float v_pitch = gds.pitch(kVwire);
      dis_v[0] = gds.wire_dist(*vwire_1) - v_pitch/2.;
      dis_v[1] = gds.wire_dist(*vwire_2) + v_pitch/2.;

      std::vector<Vector> pcross(4);
      bool flag1 = gds.crossing_point(dis_u[0],dis_v[0],kUwire,kVwire, pcross[0]); // check the inner point

      if (flag1){
	// fill the outer point
  	pcell.push_back(pcross[0]);
      }else{
  	// scan u-wire
	for (int k=0;k!=wire_index_u.size();k++){
	  const GeomWire *uwire_3 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(k));
	  dis_u[2] =  gds.wire_dist(*uwire_3) - u_pitch/2.;

	  if (gds.crossing_point(dis_u[2],dis_v[0],kUwire,kVwire, pcross[0])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[0].y,2) + pow(pcell.at(k1).z - pcross[0].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }

	    if (flag_qx == 1)
	      pcell.push_back(pcross[0]);
	    break;
	  }
	}
	// scan v-wire
	for (int k=0;k!=wire_index_v.size();k++){
	  const GeomWire *vwire_3 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(k));
	  dis_v[2] = gds.wire_dist(*vwire_3) - v_pitch/2.;
	  if (gds.crossing_point(dis_u[0],dis_v[2],kUwire,kVwire, pcross[0])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[0].y,2) + pow(pcell.at(k1).z - pcross[0].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[0]);
	    break;
	  }
	}
      }

      bool flag2 = gds.crossing_point(dis_u[0],dis_v[1],kUwire,kVwire, pcross[1]);
      if (flag2){
  	pcell.push_back(pcross[1]);
      }else{
  	// scan u-wire
	for (int k=0;k!=wire_index_u.size();k++){
	  const GeomWire *uwire_3 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(k));
	  dis_u[2] =  gds.wire_dist(*uwire_3) - u_pitch/2.;
	  if (gds.crossing_point(dis_u[2],dis_v[1],kUwire,kVwire, pcross[1])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[1].y,2) + pow(pcell.at(k1).z - pcross[1].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[1]);
	    break;
	  }
	}
	// scan v-wire
	for (int k=0;k!=wire_index_v.size();k++){
	  const GeomWire *vwire_3 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(wire_index_v.size()-1-k));
	  dis_v[2] = gds.wire_dist(*vwire_3) + v_pitch/2.;
	  if (gds.crossing_point(dis_u[0],dis_v[2],kUwire,kVwire, pcross[1])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[1].y,2) + pow(pcell.at(k1).z - pcross[1].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[1]);
	    break;
	  }
	}
      }

      bool flag3 = gds.crossing_point(dis_u[1],dis_v[0],kUwire,kVwire, pcross[2]);
      if (flag3){
  	pcell.push_back(pcross[2]);
      }else{
  	for (int k=0;k!=wire_index_u.size();k++){
	  const GeomWire *uwire_3 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(wire_index_u.size()-1-k));
	  dis_u[2] =  gds.wire_dist(*uwire_3) + u_pitch/2.;
	  if (gds.crossing_point(dis_u[2],dis_v[0],kUwire,kVwire, pcross[2])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[2].y,2) + pow(pcell.at(k1).z - pcross[2].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[2]);
	    break;
	  }
	}
	// scan v-wire
	for (int k=0;k!=wire_index_v.size();k++){
	  const GeomWire *vwire_3 =  gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(k));
	  dis_v[2] = gds.wire_dist(*vwire_3) - v_pitch/2.;
	  if (gds.crossing_point(dis_u[1],dis_v[2],kUwire,kVwire, pcross[2])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[2].y,2) + pow(pcell.at(k1).z - pcross[2].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[2]);
	    break;
	  }
	}
      }

      bool flag4 = gds.crossing_point(dis_u[1],dis_v[1],kUwire,kVwire, pcross[3]);
      if (flag4){
  	pcell.push_back(pcross[3]);
      }else{
	// scan u-wire
	for (int k=0;k!=wire_index_u.size();k++){
	  const GeomWire *uwire_3 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(wire_index_u.size()-1-k));
	  dis_u[2] =  gds.wire_dist(*uwire_3) + u_pitch/2.;
	  if (gds.crossing_point(dis_u[2],dis_v[1],kUwire,kVwire, pcross[3])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[3].y,2) + pow(pcell.at(k1).z - pcross[3].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[3]);
	    break;
	  }
	}
	// scan v-wire
	for (int k=0;k!=wire_index_v.size();k++){
	  const GeomWire *vwire_3 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(wire_index_v.size()-1-k));
	  dis_v[2] = gds.wire_dist(*vwire_3) + v_pitch/2.;
	  if (gds.crossing_point(dis_u[1],dis_v[2],kUwire,kVwire, pcross[3])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[3].y,2) + pow(pcell.at(k1).z - pcross[3].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	    pcell.push_back(pcross[3]);
	    break;
	  }
	}
      }

      mcell->AddBoundary(pcell);

    }else if (flag_u_vec->at(i)==0 && flag_w_vec->at(i)==0){
      const GeomWire *uwire_1 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.front());
      const GeomWire *uwire_2 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.back());
      float dis_u[2];
      float u_pitch = gds.pitch(kUwire);
      dis_u[0] = gds.wire_dist(*uwire_1) - u_pitch/2.;
      dis_u[1] = gds.wire_dist(*uwire_2) + u_pitch/2.;

      const GeomWire *wwire_1 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.front());
      const GeomWire *wwire_2 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.back());
      float dis_w[2];
      float w_pitch = gds.pitch(kYwire);
      dis_w[0] = gds.wire_dist(*wwire_1) - w_pitch/2.;
      dis_w[1] = gds.wire_dist(*wwire_2) + w_pitch/2.;

      std::vector<Vector> pcross(4);

      bool flag1 = gds.crossing_point(dis_u[0],dis_w[0],kUwire,kYwire, pcross[0]);
      if (flag1){
  	pcell.push_back(pcross[0]);
      }else{
  	// scan u-wire
	for (int k=0;k!=wire_index_u.size();k++){
	  const GeomWire *uwire_3 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(k));
	  dis_u[2] =  gds.wire_dist(*uwire_3) - u_pitch/2.;
	  if (gds.crossing_point(dis_u[2],dis_w[0],kUwire,kYwire, pcross[0])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[0].y,2) + pow(pcell.at(k1).z - pcross[0].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }

	    if (flag_qx == 1)
	      pcell.push_back(pcross[0]);
	    break;
	  }
	}
	// scan w-wire
	for (int k=0;k!=wire_index_w.size();k++){
	  const GeomWire *wwire_3 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(k));
	  dis_w[2] = gds.wire_dist(*wwire_3) - w_pitch/2.;
	  if (gds.crossing_point(dis_u[0],dis_w[2],kUwire,kYwire, pcross[0])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[0].y,2) + pow(pcell.at(k1).z - pcross[0].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[0]);
	    break;
	  }
	}
      }

      bool flag2 = gds.crossing_point(dis_u[0],dis_w[1],kUwire,kYwire, pcross[1]);
      if (flag2){
  	pcell.push_back(pcross[1]);
      }else{
  	// scan u-wire
	for (int k=0;k!=wire_index_u.size();k++){
	  const GeomWire *uwire_3 = gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(k));
	  dis_u[2] =  gds.wire_dist(*uwire_3) - u_pitch/2.;
	  if (gds.crossing_point(dis_u[2],dis_w[1],kUwire,kYwire, pcross[1])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[1].y,2) + pow(pcell.at(k1).z - pcross[1].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[1]);
	    break;
	  }
	}
	// scan w-wire
	for (int k=0;k!=wire_index_w.size();k++){
	  const GeomWire *wwire_3 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(wire_index_w.size()-1-k));
	  dis_w[2] = gds.wire_dist(*wwire_3) + w_pitch/2.;
	  if (gds.crossing_point(dis_u[0],dis_w[2],kUwire,kYwire, pcross[1])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[1].y,2) + pow(pcell.at(k1).z - pcross[1].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[1]);
	    break;
	  }
	}
      }

      bool flag3 = gds.crossing_point(dis_u[1],dis_w[0],kUwire,kYwire, pcross[2]);
      if (flag3){
  	pcell.push_back(pcross[2]);
      }else{
  	for (int k=0;k!=wire_index_u.size();k++){
	  const GeomWire *uwire_3 =gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(wire_index_u.size()-1-k));
	  dis_u[2] =  gds.wire_dist(*uwire_3) + u_pitch/2.;
	  if (gds.crossing_point(dis_u[2],dis_w[0],kUwire,kYwire, pcross[2])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[2].y,2) + pow(pcell.at(k1).z - pcross[2].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[2]);
	    break;
	  }
	}
	// scan w-wire
	for (int k=0;k!=wire_index_w.size();k++){
	  const GeomWire *wwire_3 =gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(k));
	  dis_w[2] = gds.wire_dist(*wwire_3) - w_pitch/2.;
	  if (gds.crossing_point(dis_u[1],dis_w[2],kUwire,kYwire, pcross[2])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[2].y,2) + pow(pcell.at(k1).z - pcross[2].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[2]);
	    break;
	  }
	}
      }

      bool flag4 = gds.crossing_point(dis_u[1],dis_w[1],kUwire,kYwire, pcross[3]);
      if (flag4){
  	pcell.push_back(pcross[3]);
      }else{
	// scan u-wire
	for (int k=0;k!=wire_index_u.size();k++){
	  const GeomWire *uwire_3 =gds.by_planeindex(WirePlaneType_t(0),wire_index_u.at(wire_index_u.size()-1-k));
	  dis_u[2] =  gds.wire_dist(*uwire_3) + u_pitch/2.;
	  if (gds.crossing_point(dis_u[2],dis_w[1],kUwire,kYwire, pcross[3])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[3].y,2) + pow(pcell.at(k1).z - pcross[3].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[3]);
	    break;
	  }
	}
	// scan w-wire
	for (int k=0;k!=wire_index_w.size();k++){
	  const GeomWire *wwire_3 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(wire_index_w.size()-1-k));
	  dis_w[2] = gds.wire_dist(*wwire_3) + w_pitch/2.;
	  if (gds.crossing_point(dis_u[1],dis_w[2],kUwire,kYwire, pcross[3])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[3].y,2) + pow(pcell.at(k1).z - pcross[3].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	    pcell.push_back(pcross[3]);
	    break;
	  }
	}
      }
      mcell->AddBoundary(pcell);

    }else if (flag_v_vec->at(i)==0 && flag_w_vec->at(i)==0){
      const GeomWire *wwire_1 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.front());
      const GeomWire *wwire_2 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.back());
      float dis_w[2];
      float w_pitch = gds.pitch(kYwire);
      dis_w[0] = gds.wire_dist(*wwire_1) - w_pitch/2.;
      dis_w[1] = gds.wire_dist(*wwire_2) + w_pitch/2.;

      const GeomWire *vwire_1 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.front());
      const GeomWire *vwire_2 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.back());
      float dis_v[2];
      float v_pitch = gds.pitch(kVwire);
      dis_v[0] = gds.wire_dist(*vwire_1) - v_pitch/2.;
      dis_v[1] = gds.wire_dist(*vwire_2) + v_pitch/2.;

      std::vector<Vector> pcross(4);

       bool flag1 = gds.crossing_point(dis_w[0],dis_v[0],kYwire,kVwire, pcross[0]);
      if (flag1){
  	pcell.push_back(pcross[0]);
      }else{
  	// scan u-wire
	for (int k=0;k!=wire_index_w.size();k++){
	  const GeomWire *wwire_3 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(k));
	  dis_w[2] =  gds.wire_dist(*wwire_3) - w_pitch/2.;
	  if (gds.crossing_point(dis_w[2],dis_v[0],kYwire,kVwire, pcross[0])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[0].y,2) + pow(pcell.at(k1).z - pcross[0].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }

	    if (flag_qx == 1)
	      pcell.push_back(pcross[0]);
	    break;
	  }
	}
	// scan v-wire
	for (int k=0;k!=wire_index_v.size();k++){
	  const GeomWire *vwire_3 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(k));
	  dis_v[2] = gds.wire_dist(*vwire_3) - v_pitch/2.;
	  if (gds.crossing_point(dis_w[0],dis_v[2],kYwire,kVwire, pcross[0])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[0].y,2) + pow(pcell.at(k1).z - pcross[0].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[0]);
	    break;
	  }
	}
      }

      bool flag2 = gds.crossing_point(dis_w[0],dis_v[1],kYwire,kVwire, pcross[1]);
      if (flag2){
  	pcell.push_back(pcross[1]);
      }else{
  	// scan u-wire
	for (int k=0;k!=wire_index_w.size();k++){
	  const GeomWire *wwire_3 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(k));
	  dis_w[2] =  gds.wire_dist(*wwire_3) - w_pitch/2.;
	  if (gds.crossing_point(dis_w[2],dis_v[1],kYwire,kVwire, pcross[1])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[1].y,2) + pow(pcell.at(k1).z - pcross[1].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[1]);
	    break;
	  }
	}
	// scan v-wire
	for (int k=0;k!=wire_index_v.size();k++){
	  const GeomWire *vwire_3 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(wire_index_v.size()-1-k));
	  dis_v[2] = gds.wire_dist(*vwire_3) + v_pitch/2.;
	  if (gds.crossing_point(dis_w[0],dis_v[2],kYwire,kVwire, pcross[1])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[1].y,2) + pow(pcell.at(k1).z - pcross[1].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[1]);
	    break;
	  }
	}
      }

      bool flag3 = gds.crossing_point(dis_w[1],dis_v[0],kYwire,kVwire, pcross[2]);
      if (flag3){
  	pcell.push_back(pcross[2]);
      }else{
  	for (int k=0;k!=wire_index_w.size();k++){
	  const GeomWire *wwire_3 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(wire_index_w.size()-1-k));
	  dis_w[2] =  gds.wire_dist(*wwire_3) + w_pitch/2.;
	  if (gds.crossing_point(dis_w[2],dis_v[0],kYwire,kVwire, pcross[2])){

	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[2].y,2) + pow(pcell.at(k1).z - pcross[2].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[2]);
	    break;
	  }
	}
	// scan v-wire
	for (int k=0;k!=wire_index_v.size();k++){
	  const GeomWire *vwire_3 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(k));
	  dis_v[2] = gds.wire_dist(*vwire_3) - v_pitch/2.;
	  if (gds.crossing_point(dis_w[1],dis_v[2],kYwire,kVwire, pcross[2])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[2].y,2) + pow(pcell.at(k1).z - pcross[2].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[2]);
	    break;
	  }
	}
      }

      bool flag4 = gds.crossing_point(dis_w[1],dis_v[1],kYwire,kVwire, pcross[3]);
      if (flag4){
  	pcell.push_back(pcross[3]);
      }else{
	// scan u-wire
	for (int k=0;k!=wire_index_w.size();k++){
	  const GeomWire *wwire_3 = gds.by_planeindex(WirePlaneType_t(2),wire_index_w.at(wire_index_w.size()-1-k));
	  dis_w[2] =  gds.wire_dist(*wwire_3) + w_pitch/2.;
	  if (gds.crossing_point(dis_w[2],dis_v[1],kYwire,kVwire, pcross[3])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[3].y,2) + pow(pcell.at(k1).z - pcross[3].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	      pcell.push_back(pcross[3]);
	    break;
	  }
	}
	// scan v-wire
	for (int k=0;k!=wire_index_v.size();k++){
	  const GeomWire *vwire_3 = gds.by_planeindex(WirePlaneType_t(1),wire_index_v.at(wire_index_v.size()-1-k));
	  dis_v[2] = gds.wire_dist(*vwire_3) + v_pitch/2.;
	  if (gds.crossing_point(dis_w[1],dis_v[2],kYwire,kVwire, pcross[3])){
	    int flag_qx = 1;
	    for (int k1 = 0;k1!=pcell.size();k1++){
	      if (sqrt(pow(pcell.at(k1).y - pcross[3].y,2) + pow(pcell.at(k1).z - pcross[3].z,2))<0.5*units::mm){
		flag_qx = 0;
		break;
	      }
	    }
	    if (flag_qx == 1)
	    pcell.push_back(pcross[3]);
	    break;
	  }
	}
      }
      mcell->AddBoundary(pcell);
    }
    // figure out the boundary ...

    fid->AddDeadRegion(mcell,time_slices);
    ident++;
  }


  // Load T_ch_bad tree ...
  TTree *T_bad_ch = (TTree*)file->Get("T_bad_ch");
  if (T_bad_ch!=0){
    Int_t chid, plane;
    Int_t start_time,end_time;
    T_bad_ch->SetBranchAddress("chid",&chid);
    T_bad_ch->SetBranchAddress("plane",&plane);
    T_bad_ch->SetBranchAddress("start_time",&start_time);
    T_bad_ch->SetBranchAddress("end_time",&end_time);
    Int_t temp_run_no, temp_subrun_no, temp_event_no;
    T_bad_ch->SetBranchAddress("runNo",&temp_run_no);
    T_bad_ch->SetBranchAddress("subRunNo",&temp_subrun_no);
    T_bad_ch->SetBranchAddress("eventNo",&temp_event_no);

    for (int i=0;i!=T_bad_ch->GetEntries();i++){
      T_bad_ch->GetEntry(i);
      if (temp_run_no != run_no || temp_subrun_no!=subrun_no || temp_event_no!=event_no) continue;

      double temp_x1 = (start_time/2.*unit_dis/10. - frame_length/2.*unit_dis/10.) * units::cm;
      double temp_x2 = (end_time/2.*unit_dis/10. - frame_length/2.*unit_dis/10.) * units::cm;
      if (plane==1){
	chid -= 2400;
      }else if (plane==2){
	chid -=4800;
      }
      if (plane==0){
	if (dead_u_index.find(chid)==dead_u_index.end()){
	  dead_u_index[chid] = std::make_pair(temp_x1-0.1*units::cm,temp_x2+0.1*units::cm);
	}else{
	  if (temp_x1-0.1*units::cm < dead_u_index[chid].first){
	    dead_u_index[chid].first = temp_x1 - 0.1*units::cm;
	  }else if (temp_x2+0.1*units::cm > dead_u_index[chid].second){
	    dead_u_index[chid].second = temp_x2 + 0.1*units::cm;
	  }
	}
      }else if (plane==1){
	if (dead_v_index.find(chid)==dead_v_index.end()){
	  dead_v_index[chid] = std::make_pair(temp_x1-0.1*units::cm,temp_x2+0.1*units::cm);
	}else{
	  if (temp_x1-0.1*units::cm < dead_v_index[chid].first){
	    dead_v_index[chid].first = temp_x1 - 0.1*units::cm;
	  }else if (temp_x2+0.1*units::cm > dead_v_index[chid].second){
	    dead_v_index[chid].second = temp_x2 + 0.1*units::cm;
	  }
	}
      }else if (plane==2){
	if (dead_w_index.find(chid)==dead_w_index.end()){
	  dead_w_index[chid] = std::make_pair(temp_x1-0.1*units::cm,temp_x2+0.1*units::cm);
	}else{
	  if (temp_x1-0.1*units::cm < dead_w_index[chid].first){
	    dead_w_index[chid].first = temp_x1 - 0.1*units::cm;
	  }else if (temp_x2+0.1*units::cm > dead_w_index[chid].second){
	    dead_w_index[chid].second = temp_x2 + 0.1*units::cm;
	  }
	}
      }
    }
  }
  cout << em("load clusters from file") << endl;

  // form a global map with the current map information
  std::map<int,std::map<const GeomWire*, SMGCSelection > > global_wc_map;
  for (size_t i=0; i!=live_clusters.size();i++){
    WCPPID::PR3DCluster *cluster = live_clusters.at(i);
    SMGCSelection& mcells = cluster->get_mcells();
    for (auto it = mcells.begin(); it!= mcells.end(); it++){
      SlimMergeGeomCell *mcell = *it;
      int time_slice = mcell->GetTimeSlice();
      if (global_wc_map.find(time_slice)==global_wc_map.end()){
	std::map<const GeomWire*, SMGCSelection> temp_wc_map;
	global_wc_map[time_slice] = temp_wc_map;
      }
      std::map<const GeomWire*, SMGCSelection>& timeslice_wc_map = global_wc_map[time_slice];

      GeomWireSelection& uwires = mcell->get_uwires();
      GeomWireSelection& vwires = mcell->get_vwires();
      GeomWireSelection& wwires = mcell->get_wwires();
      std::vector<WirePlaneType_t> bad_planes = mcell->get_bad_planes();
      if (find(bad_planes.begin(),bad_planes.end(),WirePlaneType_t(0))==bad_planes.end()){
	for (int j=0;j!=uwires.size();j++){
	  const GeomWire *wire = uwires.at(j);
	  if (timeslice_wc_map.find(wire)==timeslice_wc_map.end()){
	    SMGCSelection temp_mcells;
	    temp_mcells.push_back(mcell);
	    timeslice_wc_map[wire] = temp_mcells;
	  }else{
	    timeslice_wc_map[wire].push_back(mcell);
	  }
	}
      }
      if (find(bad_planes.begin(),bad_planes.end(),WirePlaneType_t(1))==bad_planes.end()){
	for (int j=0;j!=vwires.size();j++){
	  const GeomWire *wire = vwires.at(j);
	  if (timeslice_wc_map.find(wire)==timeslice_wc_map.end()){
	    SMGCSelection temp_mcells;
	    temp_mcells.push_back(mcell);
	     timeslice_wc_map[wire] = temp_mcells;
	  }else{
	    timeslice_wc_map[wire].push_back(mcell);
	  }
	}
      }
      if (find(bad_planes.begin(),bad_planes.end(),WirePlaneType_t(2))==bad_planes.end()){
	for (int j=0;j!=wwires.size();j++){
	  const GeomWire *wire = wwires.at(j);
	  if (timeslice_wc_map.find(wire)==timeslice_wc_map.end()){
	    SMGCSelection temp_mcells;
	    temp_mcells.push_back(mcell);
	    timeslice_wc_map[wire] = temp_mcells;
	  }else{
	    timeslice_wc_map[wire].push_back(mcell);
	  }
	}
      }
    }
  }

  // replace by the new sampling points ...
  for (size_t i=0; i!=live_clusters.size();i++){
    WCPPID::calc_sampling_points(gds,live_clusters.at(i),nrebin, frame_length, unit_dis);
    live_clusters.at(i)->Create_point_cloud();
  }
  cout << em("Add X, Y, Z points") << std::endl;

   double first_t_dis = live_clusters.at(0)->get_mcells().front()->GetTimeSlice()*time_slice_width - live_clusters.at(0)->get_mcells().front()->get_sampling_points().front().x;
  double offset_t = first_t_dis/time_slice_width;

  // test the fiducial volume cut
  fid->set_offset_t(offset_t);

  ToyCTPointCloud ct_point_cloud(0,2399,2400,4799,4800,8255, // channel range
				 offset_t, -first_u_dis/pitch_u, -first_v_dis/pitch_v, -first_w_dis/pitch_w, // offset
				 1./time_slice_width, 1./pitch_u, 1./pitch_v, 1./pitch_w, // slope
				 angle_u,angle_v,angle_w// angle
				 );
  ct_point_cloud.AddPoints(timesliceId,timesliceChannel,raw_charge,raw_charge_err);
  ct_point_cloud.AddDeadChs(dead_u_index, dead_v_index, dead_w_index);
  //  ct_point_cloud.UpdateDeadChs();
  ct_point_cloud.build_kdtree_index();

  // examine the clustering ...
  std::vector<std::pair<int, Opflash*> > to_be_checked;
  for (auto it = map_flash_tpc_ids.begin(); it!=map_flash_tpc_ids.end(); it++){
    // double flash_time = map_flash_info[it->first].second;
    double flash_time = map_flash_info[it->first]->get_time();
    if ( (flash_time <= lowerwindow || flash_time >= upperwindow)) continue;
    to_be_checked.push_back(std::make_pair(it->second, map_flash_info[it->first]) );
  }

  WCPPID::Protect_Over_Clustering(eventTime,to_be_checked, live_clusters, map_cluster_parent_id, map_parentid_clusters, ct_point_cloud, flag_match_data, run_no, time_offset, nrebin, time_slice_width, flag_timestamp);

  for (size_t i=0; i!=live_clusters.size();i++){
    if (live_clusters.at(i)->get_point_cloud()==0){
      WCPPID::calc_sampling_points(gds,live_clusters.at(i),nrebin, frame_length, unit_dis);
      live_clusters.at(i)->Create_point_cloud();
    }
  }


  std::vector<WCPPID::NeutrinoID *> neutrino_vec;
  std::map<std::pair<int, int>, WCPPID::NeutrinoID* > map_flash_tpc_pair_neutrino_id;
  // Code to select nue ...
  for (auto it = map_flash_tpc_ids.begin(); it!=map_flash_tpc_ids.end(); it++){
    //    double flash_time = map_flash_info[it->first].second;
    double flash_time = map_flash_info[it->first]->get_time();
    //std::cout << flash_time << " " << triggerbits << " " << lowerwindow << " " << upperwindow << std::endl;
    if ( (flash_time <= lowerwindow || flash_time >= upperwindow)) continue;
    if (map_parentid_clusters.find(it->second) == map_parentid_clusters.end()) continue;

    // hack for now ...
    //continue;

    std::vector<WCPPID::PR3DCluster*> temp_clusters = map_parentid_clusters[it->second];
    WCPPID::PR3DCluster* main_cluster = 0;
    for (auto it1 = temp_clusters.begin(); it1!=temp_clusters.end();it1++){
      if ((*it1)->get_cluster_id() == it->second){
  	main_cluster = *it1;
  	break;
      }
    }
    std::vector<WCPPID::PR3DCluster*> additional_clusters;
    for (auto it1 = temp_clusters.begin(); it1!=temp_clusters.end();it1++){
      if (*it1 != main_cluster)
  	additional_clusters.push_back(*it1);
    }

    double offset_x =     (flash_time - time_offset)*2./nrebin*time_slice_width;
    WCPPID::NeutrinoID *neutrino = new WCPPID::NeutrinoID(main_cluster, additional_clusters, live_clusters, fid, gds, nrebin, frame_length, unit_dis, &ct_point_cloud, global_wc_map, flash_time, offset_x, flag_neutrino_id_process, flag_bdt, flag_dl_vtx, dl_vtx_cut, match_isFC);
    neutrino->set_save_ssmsp(flag_ssmsp);
    neutrino_vec.push_back(neutrino);
    map_flash_tpc_pair_neutrino_id[std::make_pair(it->first, it->second)] = neutrino;
  }
  

  // start saving ...



  TFile *file1 = new TFile(Form("nue_%d_%d_%d.root",run_no,subrun_no,event_no),"RECREATE");
  TTree *Trun1 = new TTree("Trun","Trun");
  Trun1->SetDirectory(file1);
  Trun1->Branch("eventNo",&event_no,"eventNo/I");
  Trun1->Branch("runNo",&run_no,"runNo/I");
  Trun1->Branch("subRunNo",&subrun_no,"subRunNo/I");
  Trun1->Branch("eventTime",&eventTime,"eventTime/D");

  Trun1->Branch("triggerBits",&triggerbits,"triggerBits/i");
  Trun1->Branch("unit_dis",&unit_dis,"unit_dis/F");
  Trun1->Branch("frame_length",&frame_length,"frame_length/I");
  Trun1->Branch("eve_num",&eve_num,"eve_num/I");
  Trun1->Branch("nrebin",&nrebin,"nrebin/I");
  Trun1->Branch("time_offset",&time_offset,"time_offset/I");
  int detector = 0; // MicroBooNE
  Trun1->Branch("detector",&detector,"detector/I");
  // Trun1->Branch("timesliceId",&timesliceId);
  // Trun1->Branch("timesliceChannel",&timesliceChannel);
  // Trun1->Branch("raw_charge",&raw_charge);
  // Trun1->Branch("raw_charge_err",&raw_charge_err);

  
  
  Double_t dQdx_scale = 0.1;
  Trun1->Branch("dQdx_scale",&dQdx_scale,"dQdx_scale/D");
  Double_t dQdx_offset = -1000;
  Trun1->Branch("dQdx_offset",&dQdx_offset,"dQdx_offset/D");

  if (elifetime < 1000){
    Trun1->Branch("elifetime",&elifetime,"elifetime/F");
  }
  
  Trun1->Fill();

  if (T_bad_ch!=0){
    //  T_bad_ch->CloneTree(-1,"fast");
    Int_t chid, plane;
    Int_t start_time,end_time;
    T_bad_ch->SetBranchAddress("chid",&chid);
    T_bad_ch->SetBranchAddress("plane",&plane);
    T_bad_ch->SetBranchAddress("start_time",&start_time);
    T_bad_ch->SetBranchAddress("end_time",&end_time);
    Int_t temp_run_no, temp_subrun_no, temp_event_no;
    T_bad_ch->SetBranchAddress("runNo",&temp_run_no);
    T_bad_ch->SetBranchAddress("subRunNo",&temp_subrun_no);
    T_bad_ch->SetBranchAddress("eventNo",&temp_event_no);

    TTree *T_bad_ch1 = new TTree("T_bad_ch","T_bad_ch");
    T_bad_ch1->SetDirectory(file1);
    T_bad_ch1->Branch("chid",&chid,"chid/I");
    T_bad_ch1->Branch("plane",&plane,"plane/I");
    T_bad_ch1->Branch("start_time",&start_time,"start_time/I");
    T_bad_ch1->Branch("end_time",&end_time,"end_time/I");
    T_bad_ch1->Branch("runNo",&temp_run_no,"runNo/I");
    T_bad_ch1->Branch("subRunNo",&temp_subrun_no,"subRunNo/I");
    T_bad_ch1->Branch("eventNo",&temp_event_no,"eventNo/I");

    for (int i=0;i!=T_bad_ch->GetEntries();i++){
      T_bad_ch->GetEntry(i);
      if (temp_run_no != run_no || temp_subrun_no!=subrun_no || temp_event_no!=event_no) continue;
      T_bad_ch1->Fill();
    }

  }



  TTree *T_flash1 = new TTree("T_flash","T_flash");
  T_flash1->SetDirectory(file1);
  T_flash1->Branch("type",&type);
  T_flash1->Branch("flash_id",&flash_id);
  T_flash1->Branch("low_time",&low_time);
  T_flash1->Branch("high_time",&high_time);
  T_flash1->Branch("time",&time);
  T_flash1->Branch("total_PE",&total_PE);
  T_flash1->Branch("PE",PE,"PE[32]/D");
  T_flash1->Branch("PE_err",PE_err,"PE_err[32]/D");
  T_flash1->Branch("fired_channels",&fired_channels);
  T_flash1->Branch("l1_fired_time",&l1_fired_time);
  T_flash1->Branch("l1_fired_pe",&l1_fired_pe);
  for (int i=0;i!=T_flash->GetEntries();i++){
    T_flash->GetEntry(i);
    if (temp_run_no!=run_no || temp_subrun_no!=subrun_no || temp_event_no != event_no) continue;
    T_flash1->Fill();
  }



  TTree *T_match1 = new TTree("T_match","T_match");
  T_match1->SetDirectory(file1);
  T_match1->Branch("tpc_cluster_id",&tpc_cluster_id,"tpc_cluster_id/I"); // parent cluster id
  T_match1->Branch("flash_id",&flash_id,"flash_id/I");  // flash id

  T_match1->Branch("strength",&strength,"strength/D");
  T_match1->Branch("pe_pred",pe_pred,"pe_pred[32]/D");
  T_match1->Branch("pe_meas",pe_meas,"pe_meas[32]/D");
  T_match1->Branch("pe_meas_err",pe_meas_err,"pe_meas_err[32]/D");
  T_match1->Branch("event_type",&event_type,"event_type/I");
  T_match1->Branch("ks_dis",&ks_dis,"ks_dis/D");
  T_match1->Branch("chi2",&chi2,"chi2/D");
  T_match1->Branch("ndf",&ndf,"ndf/I");
  T_match1->Branch("cluster_length",&cluster_length,"cluster_length/D");
  int neutrino_type = 0;
  T_match1->Branch("neutrino_type",&neutrino_type,"neutrino_type/I");
  Double_t flash_time;
  T_match1->Branch("flash_time",&flash_time,"flash_time/D");

  for (int i=0;i!=T_match->GetEntries();i++){
    T_match->GetEntry(i);
    if (temp_run_no!=run_no || temp_subrun_no!=subrun_no || temp_event_no != event_no) continue;
    neutrino_type = 0;

    bool flag_flash = map_flash_info.find(flash_id) != map_flash_info.end() ;
    flash_time = -1000;

    if (flag_flash){
      flash_time =  map_flash_info[flash_id]->get_time();
    }
    //    std::cout << flash_id << " " << flag << std::endl;

    auto it = map_flash_tpc_pair_neutrino_id.find(std::make_pair(flash_id, tpc_cluster_id));
    if (it != map_flash_tpc_pair_neutrino_id.end()){
      neutrino_type = it->second->get_neutrino_type();
    }
    T_match1->Fill();
  }

  TTree *T_vtx = new TTree("T_vtx","T_vtx");
  T_vtx->SetDirectory(file1);
  Double_t x_vtx, y_vtx, z_vtx;
  Int_t type_vtx=0;
  // 1: Steiner-inspired graph
  // 2: Steiner terminals
  // 3: end point (extreme point, connecting to one object)
  // 4: kink (connecting to two objects
  // 5: vertex (connecting to three or more objects)
  Int_t flag_main_vtx=0; // main vertex or not
  Int_t cluster_id_vtx=-1; // -1 if not belonging to any cluster ...
  std::vector<int> *sub_cluster_ids_vtx = new std::vector<int>;

  T_vtx->Branch("x",&x_vtx,"x/D");
  T_vtx->Branch("y",&y_vtx,"y/D");
  T_vtx->Branch("z",&z_vtx,"z/D");
  T_vtx->Branch("type",&type_vtx,"type/I");
  T_vtx->Branch("flag_main",&flag_main_vtx,"flag_main/I");
  T_vtx->Branch("cluster_id",&cluster_id_vtx,"cluster_id/I");
  T_vtx->Branch("sub_cluster_ids",&sub_cluster_ids_vtx);

  for (size_t i=0;i!=neutrino_vec.size();i++){
    std::map<WCPPID::PR3DCluster*, WCPPID::ProtoVertex* >& map_cluster_vertex = neutrino_vec.at(i)->get_map_cluster_vertex();
    WCPPID::Map_Proto_Vertex_Segments& map_vertex_segments = neutrino_vec.at(i)->get_map_vertex_segments();
    std::map<WCPPID::PR3DCluster*, WCPPID::ProtoVertexSelection>& map_cluster_candidate_vertices = neutrino_vec.at(i)->get_map_cluster_candidate_vertices();

    std::map<WCPPID::ProtoVertex*, int> map_vertex_type;
    for (auto it = map_vertex_segments.begin(); it != map_vertex_segments.end(); it++){
      WCPPID::ProtoVertex *vtx = it->first;
      int type = 0;
      map_vertex_type[vtx] = type;
    }
    for (auto it = map_cluster_vertex.begin(); it!= map_cluster_vertex.end(); it++){
      WCPPID::ProtoVertex *vtx = it->second;
      if (map_vertex_type.find(vtx) == map_vertex_type.end()) map_vertex_type[vtx] = 0;
      map_vertex_type[vtx] |= 1UL << 2;
    }
    for (auto it = map_cluster_candidate_vertices.begin(); it != map_cluster_candidate_vertices.end(); it++){
      for (auto it1 = it->second.begin(); it1 != it->second.end(); it1++){
	WCPPID::ProtoVertex *vtx = *it1;
	if (map_vertex_type.find(vtx) == map_vertex_type.end()) map_vertex_type[vtx] = 0;
	map_vertex_type[vtx] |= 1UL << 1;
      }
    }

    for (auto it = map_vertex_type.begin();  it != map_vertex_type.end(); it++){
      WCPPID::ProtoVertex *vtx = it->first;
      auto it1 = map_vertex_segments.find(vtx);
      Point vertex_point;

      if (it1 != map_vertex_segments.end() && it1->second.size()>0){
	WCPPID::ProtoSegment *sg = *map_vertex_segments[vtx].begin();

	if (vtx->get_wcpt().index == sg->get_wcpt_vec().front().index){
	  vertex_point = sg->get_point_vec().front();
	}else{
	  vertex_point = sg->get_point_vec().back();
	}
      }else{
	continue;
      }
      x_vtx = vertex_point.x/units::cm;
      y_vtx = vertex_point.y/units::cm;
      z_vtx = vertex_point.z/units::cm;

      type_vtx = it->second;
      if (vtx ==  neutrino_vec.at(i)->get_main_vertex())
	flag_main_vtx = 1;
      else
	flag_main_vtx = 0;

      cluster_id_vtx = vtx->get_cluster_id();
      sub_cluster_ids_vtx->clear();
      //     for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      //     	WCPPID::ProtoSegment *sg = *it1;
      //     	sub_cluster_ids_vtx->push_back(cluster_id_vtx*1000 + sg->get_id());
      //     }
      T_vtx->Fill();
    }

    WCPPID::TaggerInfo tagger_info;
    TTree *T_tagger = new TTree("T_tagger","T_tagger");
    T_tagger->SetDirectory(file1);
    T_tagger->Branch("nu_x",&x_vtx,"nu_x/F");
    T_tagger->Branch("nu_y",&y_vtx,"nu_y/F");
    T_tagger->Branch("nu_z",&z_vtx,"nu_z/F");
    // cosmic tagger
    T_tagger->Branch("cosmic_flag", &tagger_info.cosmic_flag, "cosmic_flag/F");
    T_tagger->Branch("cosmic_n_solid_tracks",&tagger_info.cosmic_n_solid_tracks,"cosmic_n_solid_tracks/F");
    T_tagger->Branch("cosmic_energy_main_showers",&tagger_info.cosmic_energy_main_showers,"cosmic_energy_main_showers/F");
    T_tagger->Branch("cosmic_energy_direct_showers",&tagger_info.cosmic_energy_direct_showers,"cosmic_energy_direct_showers/F");
    T_tagger->Branch("cosmic_energy_indirect_showers",&tagger_info.cosmic_energy_indirect_showers,"cosmic_energy_indirect_showers/F");
    T_tagger->Branch("cosmic_n_direct_showers",&tagger_info.cosmic_n_direct_showers,"cosmic_n_direct_showers/F");
    T_tagger->Branch("cosmic_n_indirect_showers",&tagger_info.cosmic_n_indirect_showers,"cosmic_n_indirect_showers/F");
    T_tagger->Branch("cosmic_n_main_showers",&tagger_info.cosmic_n_main_showers,"cosmic_n_main_showers/F");
    T_tagger->Branch("cosmic_filled",&tagger_info.cosmic_filled,"cosmic_filled/F");

    // gap tagger
    T_tagger->Branch("gap_flag",&tagger_info.gap_flag,"gap_flag/F");
    T_tagger->Branch("gap_flag_prolong_u",&tagger_info.gap_flag_prolong_u,"gap_flag_prolong_u/F");
    T_tagger->Branch("gap_flag_prolong_v",&tagger_info.gap_flag_prolong_v,"gap_flag_prolong_v/F");
    T_tagger->Branch("gap_flag_prolong_w",&tagger_info.gap_flag_prolong_w,"gap_flag_prolong_w/F");
    T_tagger->Branch("gap_flag_parallel",&tagger_info.gap_flag_parallel,"gap_flag_parallel/F");
    T_tagger->Branch("gap_n_points",&tagger_info.gap_n_points,"gap_n_points/F");
    T_tagger->Branch("gap_n_bad",&tagger_info.gap_n_bad,"gap_n_bad/F");
    T_tagger->Branch("gap_energy",&tagger_info.gap_energy,"gap_energy/F");
    T_tagger->Branch("gap_num_valid_tracks",&tagger_info.gap_num_valid_tracks,"gap_num_valid_tracks/F");
    T_tagger->Branch("gap_flag_single_shower",&tagger_info.gap_flag_single_shower,"gap_flag_single_shower/F");
    T_tagger->Branch("gap_filled",&tagger_info.gap_filled,"gap_filled/F");

    // mip quality
    T_tagger->Branch("mip_quality_flag",&tagger_info.mip_quality_flag,"mip_quality_flag/F");
    T_tagger->Branch("mip_quality_energy",&tagger_info.mip_quality_energy,"mip_quality_energy/F");
    T_tagger->Branch("mip_quality_overlap",&tagger_info.mip_quality_overlap,"mip_quality_overlap/F");
    T_tagger->Branch("mip_quality_n_showers",&tagger_info.mip_quality_n_showers,"mip_quality_n_showers/F");
    T_tagger->Branch("mip_quality_n_tracks",&tagger_info.mip_quality_n_tracks,"mip_quality_n_tracks/F");
    T_tagger->Branch("mip_quality_flag_inside_pi0",&tagger_info.mip_quality_flag_inside_pi0,"mip_quality_flag_inside_pi0/F");
    T_tagger->Branch("mip_quality_n_pi0_showers",&tagger_info.mip_quality_n_pi0_showers,"mip_quality_n_pi0_showers/F");
    T_tagger->Branch("mip_quality_shortest_length",&tagger_info.mip_quality_shortest_length,"mip_quality_shortest_length/F");
    T_tagger->Branch("mip_quality_acc_length",&tagger_info.mip_quality_acc_length,"mip_quality_acc_length/F");
    T_tagger->Branch("mip_quality_shortest_angle",&tagger_info.mip_quality_shortest_angle,"mip_quality_shortest_angle/F");
    T_tagger->Branch("mip_quality_flag_proton",&tagger_info.mip_quality_flag_proton,"mip_quality_flag_proton/F");
    T_tagger->Branch("mip_quality_filled",&tagger_info.mip_quality_filled,"mip_quality_filled/F");

    // mip
    T_tagger->Branch("mip_flag",&tagger_info.mip_flag,"mip_flag/F");
    T_tagger->Branch("mip_energy",&tagger_info.mip_energy,"mip_energy/F");
    T_tagger->Branch("mip_n_end_reduction",&tagger_info.mip_n_end_reduction,"mip_n_end_reduction/F");
    T_tagger->Branch("mip_n_first_mip",&tagger_info.mip_n_first_mip,"mip_n_first_mip/F");
    T_tagger->Branch("mip_n_first_non_mip",&tagger_info.mip_n_first_non_mip,"mip_n_first_non_mip/F");
    T_tagger->Branch("mip_n_first_non_mip_1",&tagger_info.mip_n_first_non_mip_1,"mip_n_first_non_mip_1/F");
    T_tagger->Branch("mip_n_first_non_mip_2",&tagger_info.mip_n_first_non_mip_2,"mip_n_first_non_mip_2/F");

    T_tagger->Branch("mip_vec_dQ_dx_0",&tagger_info.mip_vec_dQ_dx_0,"mip_vec_dQ_dx_0/F");
    T_tagger->Branch("mip_vec_dQ_dx_1",&tagger_info.mip_vec_dQ_dx_1,"mip_vec_dQ_dx_1/F");
    T_tagger->Branch("mip_vec_dQ_dx_2",&tagger_info.mip_vec_dQ_dx_2,"mip_vec_dQ_dx_2/F");
    T_tagger->Branch("mip_vec_dQ_dx_3",&tagger_info.mip_vec_dQ_dx_3,"mip_vec_dQ_dx_3/F");
    T_tagger->Branch("mip_vec_dQ_dx_4",&tagger_info.mip_vec_dQ_dx_4,"mip_vec_dQ_dx_4/F");
    T_tagger->Branch("mip_vec_dQ_dx_5",&tagger_info.mip_vec_dQ_dx_5,"mip_vec_dQ_dx_5/F");
    T_tagger->Branch("mip_vec_dQ_dx_6",&tagger_info.mip_vec_dQ_dx_6,"mip_vec_dQ_dx_6/F");
    T_tagger->Branch("mip_vec_dQ_dx_7",&tagger_info.mip_vec_dQ_dx_7,"mip_vec_dQ_dx_7/F");
    T_tagger->Branch("mip_vec_dQ_dx_8",&tagger_info.mip_vec_dQ_dx_8,"mip_vec_dQ_dx_8/F");
    T_tagger->Branch("mip_vec_dQ_dx_9",&tagger_info.mip_vec_dQ_dx_9,"mip_vec_dQ_dx_9/F");
    T_tagger->Branch("mip_vec_dQ_dx_10",&tagger_info.mip_vec_dQ_dx_10,"mip_vec_dQ_dx_10/F");
    T_tagger->Branch("mip_vec_dQ_dx_11",&tagger_info.mip_vec_dQ_dx_11,"mip_vec_dQ_dx_11/F");
    T_tagger->Branch("mip_vec_dQ_dx_12",&tagger_info.mip_vec_dQ_dx_12,"mip_vec_dQ_dx_12/F");
    T_tagger->Branch("mip_vec_dQ_dx_13",&tagger_info.mip_vec_dQ_dx_13,"mip_vec_dQ_dx_13/F");
    T_tagger->Branch("mip_vec_dQ_dx_14",&tagger_info.mip_vec_dQ_dx_14,"mip_vec_dQ_dx_14/F");
    T_tagger->Branch("mip_vec_dQ_dx_15",&tagger_info.mip_vec_dQ_dx_15,"mip_vec_dQ_dx_15/F");
    T_tagger->Branch("mip_vec_dQ_dx_16",&tagger_info.mip_vec_dQ_dx_16,"mip_vec_dQ_dx_16/F");
    T_tagger->Branch("mip_vec_dQ_dx_17",&tagger_info.mip_vec_dQ_dx_17,"mip_vec_dQ_dx_17/F");
    T_tagger->Branch("mip_vec_dQ_dx_18",&tagger_info.mip_vec_dQ_dx_18,"mip_vec_dQ_dx_18/F");
    T_tagger->Branch("mip_vec_dQ_dx_19",&tagger_info.mip_vec_dQ_dx_19,"mip_vec_dQ_dx_19/F");

    T_tagger->Branch("mip_max_dQ_dx_sample",&tagger_info.mip_max_dQ_dx_sample,"mip_max_dQ_dx_sample/F");
    T_tagger->Branch("mip_n_below_threshold",&tagger_info.mip_n_below_threshold,"mip_n_below_threshold/F");
    T_tagger->Branch("mip_n_below_zero",&tagger_info.mip_n_below_zero,"mip_n_below_zero/F");
    T_tagger->Branch("mip_n_lowest",&tagger_info.mip_n_lowest,"mip_n_lowest/F");
    T_tagger->Branch("mip_n_highest",&tagger_info.mip_n_highest,"mip_n_highest/F");

    T_tagger->Branch("mip_lowest_dQ_dx",&tagger_info.mip_lowest_dQ_dx,"mip_lowest_dQ_dx/F");
    T_tagger->Branch("mip_highest_dQ_dx",&tagger_info.mip_highest_dQ_dx,"mip_highest_dQ_dx/F");
    T_tagger->Branch("mip_medium_dQ_dx",&tagger_info.mip_medium_dQ_dx,"mip_medium_dQ_dx/F");
    T_tagger->Branch("mip_stem_length",&tagger_info.mip_stem_length,"mip_stem_length/F");
    T_tagger->Branch("mip_length_main",&tagger_info.mip_length_main,"mip_length_main/F");
    T_tagger->Branch("mip_length_total",&tagger_info.mip_length_total,"mip_length_total/F");
    T_tagger->Branch("mip_angle_beam",&tagger_info.mip_angle_beam,"mip_angle_beam/F");
    T_tagger->Branch("mip_iso_angle",&tagger_info.mip_iso_angle,"mip_iso_angle/F");

    T_tagger->Branch("mip_n_vertex",&tagger_info.mip_n_vertex,"mip_n_vertex/F");
    T_tagger->Branch("mip_n_good_tracks",&tagger_info.mip_n_good_tracks,"mip_n_good_tracks/F");
    T_tagger->Branch("mip_E_indirect_max_energy",&tagger_info.mip_E_indirect_max_energy,"mip_E_indirect_max_energy/F");
    T_tagger->Branch("mip_flag_all_above",&tagger_info.mip_flag_all_above,"mip_flag_all_above/F");
    T_tagger->Branch("mip_min_dQ_dx_5",&tagger_info.mip_min_dQ_dx_5,"mip_min_dQ_dx_5/F");
    T_tagger->Branch("mip_n_other_vertex",&tagger_info.mip_n_other_vertex,"mip_n_other_vertex/F");
    T_tagger->Branch("mip_n_stem_size",&tagger_info.mip_n_stem_size,"mip_n_stem_size/F");
    T_tagger->Branch("mip_flag_stem_trajectory",&tagger_info.mip_flag_stem_trajectory,"mip_flag_stem_trajectory/F");
    T_tagger->Branch("mip_min_dis",&tagger_info.mip_min_dis,"mip_min_dis/F");
    T_tagger->Branch("mip_filled",&tagger_info.mip_filled,"mip_filled/F");

    //kdar tagger
    T_tagger->Branch("ssm_flag_st_kdar",&tagger_info.ssm_flag_st_kdar,"ssm_flag_st_kdar/F");
    //std::cout<<"nue:  tagger_info.ssm_Nsm"<<tagger_info.ssm_Nsm<<std::endl;
    T_tagger->Branch("ssm_Nsm",&tagger_info.ssm_Nsm,"ssm_Nsm/F");
    T_tagger->Branch("ssm_Nsm_wivtx",&tagger_info.ssm_Nsm_wivtx,"ssm_Nsm_wivtx/F");

    //only filled if there is one ssm
      //properties of the ssm
      //dq/dx info
      T_tagger->Branch("ssm_dq_dx_fwd_1", &tagger_info.ssm_dq_dx_fwd_1, "ssm_dq_dx_fwd_1/F");
      T_tagger->Branch("ssm_dq_dx_fwd_2", &tagger_info.ssm_dq_dx_fwd_2, "ssm_dq_dx_fwd_2/F");
      T_tagger->Branch("ssm_dq_dx_fwd_3", &tagger_info.ssm_dq_dx_fwd_3, "ssm_dq_dx_fwd_3/F");
      T_tagger->Branch("ssm_dq_dx_fwd_4", &tagger_info.ssm_dq_dx_fwd_4, "ssm_dq_dx_fwd_4/F");
      T_tagger->Branch("ssm_dq_dx_fwd_5", &tagger_info.ssm_dq_dx_fwd_5, "ssm_dq_dx_fwd_5/F");
      T_tagger->Branch("ssm_dq_dx_bck_1", &tagger_info.ssm_dq_dx_bck_1, "ssm_dq_dx_bck_1/F");
      T_tagger->Branch("ssm_dq_dx_bck_2", &tagger_info.ssm_dq_dx_bck_2, "ssm_dq_dx_bck_2/F");
      T_tagger->Branch("ssm_dq_dx_bck_3", &tagger_info.ssm_dq_dx_bck_3, "ssm_dq_dx_bck_3/F");
      T_tagger->Branch("ssm_dq_dx_bck_4", &tagger_info.ssm_dq_dx_bck_4, "ssm_dq_dx_bck_4/F");
      T_tagger->Branch("ssm_dq_dx_bck_5", &tagger_info.ssm_dq_dx_bck_5, "ssm_dq_dx_bck_5/F");
      T_tagger->Branch("ssm_d_dq_dx_fwd_12", &tagger_info.ssm_d_dq_dx_fwd_12, "ssm_d_dq_dx_fwd_12/F");
      T_tagger->Branch("ssm_d_dq_dx_fwd_23", &tagger_info.ssm_d_dq_dx_fwd_23, "ssm_d_dq_dx_fwd_23/F");
      T_tagger->Branch("ssm_d_dq_dx_fwd_34", &tagger_info.ssm_d_dq_dx_fwd_34, "ssm_d_dq_dx_fwd_34/F");
      T_tagger->Branch("ssm_d_dq_dx_fwd_45", &tagger_info.ssm_d_dq_dx_fwd_45, "ssm_d_dq_dx_fwd_45/F");
      T_tagger->Branch("ssm_d_dq_dx_bck_12", &tagger_info.ssm_d_dq_dx_bck_12, "ssm_d_dq_dx_bck_12/F");
      T_tagger->Branch("ssm_d_dq_dx_bck_23", &tagger_info.ssm_d_dq_dx_bck_23, "ssm_d_dq_dx_bck_23/F");
      T_tagger->Branch("ssm_d_dq_dx_bck_34", &tagger_info.ssm_d_dq_dx_bck_34, "ssm_d_dq_dx_bck_34/F");
      T_tagger->Branch("ssm_d_dq_dx_bck_45", &tagger_info.ssm_d_dq_dx_bck_45, "ssm_d_dq_dx_bck_45/F");
      T_tagger->Branch("ssm_max_dq_dx_fwd_3", &tagger_info.ssm_max_dq_dx_fwd_3, "ssm_max_dq_dx_fwd_3/F");
      T_tagger->Branch("ssm_max_dq_dx_fwd_5", &tagger_info.ssm_max_dq_dx_fwd_5, "ssm_max_dq_dx_fwd_5/F");
      T_tagger->Branch("ssm_max_dq_dx_bck_3", &tagger_info.ssm_max_dq_dx_bck_3, "ssm_max_dq_dx_bck_3/F");
      T_tagger->Branch("ssm_max_dq_dx_bck_5", &tagger_info.ssm_max_dq_dx_bck_5, "ssm_max_dq_dx_bck_5/F");
      T_tagger->Branch("ssm_max_d_dq_dx_fwd_3", &tagger_info.ssm_max_d_dq_dx_fwd_3, "ssm_max_d_dq_dx_fwd_3/F");
      T_tagger->Branch("ssm_max_d_dq_dx_fwd_5", &tagger_info.ssm_max_d_dq_dx_fwd_5, "ssm_max_d_dq_dx_fwd_5/F");
      T_tagger->Branch("ssm_max_d_dq_dx_bck_3", &tagger_info.ssm_max_d_dq_dx_bck_3, "ssm_max_d_dq_dx_bck_3/F");
      T_tagger->Branch("ssm_max_d_dq_dx_bck_5", &tagger_info.ssm_max_d_dq_dx_bck_5, "ssm_max_d_dq_dx_bck_5/F");
      T_tagger->Branch("ssm_medium_dq_dx", &tagger_info.ssm_medium_dq_dx, "ssm_medium_dq_dx/F");
      T_tagger->Branch("ssm_medium_dq_dx_bp", &tagger_info.ssm_medium_dq_dx_bp, "ssm_medium_dq_dx_bp/F");
      //angluar info
      T_tagger->Branch("ssm_angle_to_z", &tagger_info.ssm_angle_to_z, "ssm_angle_to_z/F");
      T_tagger->Branch("ssm_angle_to_target", &tagger_info.ssm_angle_to_target, "ssm_angle_to_target/F");
      T_tagger->Branch("ssm_angle_to_absorber", &tagger_info.ssm_angle_to_absorber, "ssm_angle_to_absorber/F");
      T_tagger->Branch("ssm_angle_to_vertical", &tagger_info.ssm_angle_to_vertical, "ssm_angle_to_vertical/F");
      //directional info
      T_tagger->Branch("ssm_x_dir", &tagger_info.ssm_x_dir, "ssm_x_dir/F");
      T_tagger->Branch("ssm_y_dir", &tagger_info.ssm_y_dir, "ssm_y_dir/F");
      T_tagger->Branch("ssm_z_dir", &tagger_info.ssm_z_dir, "ssm_z_dir/F");
      //energy info
      T_tagger->Branch("ssm_kine_energy", &tagger_info.ssm_kine_energy, "ssm_kine_energy/F");
      T_tagger->Branch("ssm_kine_energy_reduced", &tagger_info.ssm_kine_energy_reduced, "ssm_kine_energy_reduced/F");
      //general properties
      T_tagger->Branch("ssm_vtx_activity", &tagger_info.ssm_vtx_activity, "ssm_vtx_activity/F");
      T_tagger->Branch("ssm_pdg", &tagger_info.ssm_pdg, "ssm_pdg/F");
      T_tagger->Branch("ssm_dQ_dx_cut", &tagger_info.ssm_dQ_dx_cut, "ssm_dQ_dx_cut/F");
      T_tagger->Branch("ssm_score_mu_fwd", &tagger_info.ssm_score_mu_fwd, "ssm_score_mu_fwd/F");
      T_tagger->Branch("ssm_score_p_fwd", &tagger_info.ssm_score_p_fwd, "ssm_score_p_fwd/F");
      T_tagger->Branch("ssm_score_e_fwd", &tagger_info.ssm_score_e_fwd, "ssm_score_e_fwd/F");
      T_tagger->Branch("ssm_score_mu_bck", &tagger_info.ssm_score_mu_bck, "ssm_score_mu_bck/F");
      T_tagger->Branch("ssm_score_p_bck", &tagger_info.ssm_score_p_bck, "ssm_score_p_bck/F");
      T_tagger->Branch("ssm_score_e_bck", &tagger_info.ssm_score_e_bck, "ssm_score_e_bck/F");
      T_tagger->Branch("ssm_score_mu_fwd_bp", &tagger_info.ssm_score_mu_fwd_bp, "ssm_score_mu_fwd_bp/F");
      T_tagger->Branch("ssm_score_p_fwd_bp", &tagger_info.ssm_score_p_fwd_bp, "ssm_score_p_fwd_bp/F");
      T_tagger->Branch("ssm_score_e_fwd_bp", &tagger_info.ssm_score_e_fwd_bp, "ssm_score_e_fwd_bp/F");
      //track "straighness"
      T_tagger->Branch("ssm_length", &tagger_info.ssm_length, "ssm_length/F");
      T_tagger->Branch("ssm_direct_length", &tagger_info.ssm_direct_length, "ssm_direct_length/F");
      T_tagger->Branch("ssm_length_ratio", &tagger_info.ssm_length_ratio, "ssm_length_ratio/F");
      T_tagger->Branch("ssm_max_dev", &tagger_info.ssm_max_dev, "ssm_max_dev/F");
    //number of other particles
      T_tagger->Branch("ssm_n_prim_tracks_1", &tagger_info.ssm_n_prim_tracks_1, "ssm_n_prim_tracks_1/F");
      T_tagger->Branch("ssm_n_prim_tracks_3", &tagger_info.ssm_n_prim_tracks_3, "ssm_n_prim_tracks_3/F");
      T_tagger->Branch("ssm_n_prim_tracks_5", &tagger_info.ssm_n_prim_tracks_5, "ssm_n_prim_tracks_5/F");
      T_tagger->Branch("ssm_n_prim_tracks_8", &tagger_info.ssm_n_prim_tracks_8, "ssm_n_prim_tracks_8/F");
      T_tagger->Branch("ssm_n_prim_tracks_11", &tagger_info.ssm_n_prim_tracks_11, "ssm_n_prim_tracks_11/F");
      T_tagger->Branch("ssm_n_all_tracks_1", &tagger_info.ssm_n_all_tracks_1, "ssm_n_all_tracks_1/F");
      T_tagger->Branch("ssm_n_all_tracks_3", &tagger_info.ssm_n_all_tracks_3, "ssm_n_all_tracks_3/F");
      T_tagger->Branch("ssm_n_all_tracks_5", &tagger_info.ssm_n_all_tracks_5, "ssm_n_all_tracks_5/F");
      T_tagger->Branch("ssm_n_all_tracks_8", &tagger_info.ssm_n_all_tracks_8, "ssm_n_all_tracks_8/F");
      T_tagger->Branch("ssm_n_all_tracks_11", &tagger_info.ssm_n_all_tracks_11, "ssm_n_all_tracks_11/F");
      T_tagger->Branch("ssm_n_daughter_tracks_1", &tagger_info.ssm_n_daughter_tracks_1, "ssm_n_daughter_tracks_1/F");
      T_tagger->Branch("ssm_n_daughter_tracks_3", &tagger_info.ssm_n_daughter_tracks_3, "ssm_n_daughter_tracks_3/F");
      T_tagger->Branch("ssm_n_daughter_tracks_5", &tagger_info.ssm_n_daughter_tracks_5, "ssm_n_daughter_tracks_5/F");
      T_tagger->Branch("ssm_n_daughter_tracks_8", &tagger_info.ssm_n_daughter_tracks_8, "ssm_n_daughter_tracks_8/F");
      T_tagger->Branch("ssm_n_daughter_tracks_11", &tagger_info.ssm_n_daughter_tracks_11, "ssm_n_daughter_tracks_11/F");
      T_tagger->Branch("ssm_n_daughter_all_1", &tagger_info.ssm_n_daughter_all_1, "ssm_n_daughter_all_1/F");
      T_tagger->Branch("ssm_n_daughter_all_3", &tagger_info.ssm_n_daughter_all_3, "ssm_n_daughter_all_3/F");
      T_tagger->Branch("ssm_n_daughter_all_5", &tagger_info.ssm_n_daughter_all_5, "ssm_n_daughter_all_5/F");
      T_tagger->Branch("ssm_n_daughter_all_8", &tagger_info.ssm_n_daughter_all_8, "ssm_n_daughter_all_8/F");
      T_tagger->Branch("ssm_n_daughter_all_11", &tagger_info.ssm_n_daughter_all_11, "ssm_n_daughter_all_11/F");
    //properties of leading other primary track
      T_tagger->Branch("ssm_prim_track1_pdg", &tagger_info.ssm_prim_track1_pdg, "ssm_prim_track1_pdg/F");
      T_tagger->Branch("ssm_prim_track1_score_mu_fwd", &tagger_info.ssm_prim_track1_score_mu_fwd, "ssm_prim_track1_score_mu_fwd/F");
      T_tagger->Branch("ssm_prim_track1_score_p_fwd", &tagger_info.ssm_prim_track1_score_p_fwd, "ssm_prim_track1_score_p_fwd/F");
      T_tagger->Branch("ssm_prim_track1_score_e_fwd", &tagger_info.ssm_prim_track1_score_e_fwd, "ssm_prim_track1_score_e_fwd/F");
      T_tagger->Branch("ssm_prim_track1_score_mu_bck", &tagger_info.ssm_prim_track1_score_mu_bck, "ssm_prim_track1_score_mu_bck/F");
      T_tagger->Branch("ssm_prim_track1_score_p_bck", &tagger_info.ssm_prim_track1_score_p_bck, "ssm_prim_track1_score_p_bck/F");
      T_tagger->Branch("ssm_prim_track1_score_e_bck", &tagger_info.ssm_prim_track1_score_e_bck, "ssm_prim_track1_score_e_bck/F");
      T_tagger->Branch("ssm_prim_track1_length", &tagger_info.ssm_prim_track1_length, "ssm_prim_track1_length/F");
      T_tagger->Branch("ssm_prim_track1_direct_length", &tagger_info.ssm_prim_track1_direct_length, "ssm_prim_track1_direct_length/F");
      T_tagger->Branch("ssm_prim_track1_length_ratio", &tagger_info.ssm_prim_track1_length_ratio, "ssm_prim_track1_length_ratio/F");
      T_tagger->Branch("ssm_prim_track1_max_dev", &tagger_info.ssm_prim_track1_max_dev, "ssm_prim_track1_max_dev/F");
      T_tagger->Branch("ssm_prim_track1_kine_energy_range", &tagger_info.ssm_prim_track1_kine_energy_range, "ssm_prim_track1_kine_energy_range/F");
      T_tagger->Branch("ssm_prim_track1_kine_energy_range_mu", &tagger_info.ssm_prim_track1_kine_energy_range_mu, "ssm_prim_track1_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_prim_track1_kine_energy_range_p", &tagger_info.ssm_prim_track1_kine_energy_range_p, "ssm_prim_track1_kine_energy_range_p/F");
      T_tagger->Branch("ssm_prim_track1_kine_energy_range_e", &tagger_info.ssm_prim_track1_kine_energy_range_e, "ssm_prim_track1_kine_energy_range_e/F");
      T_tagger->Branch("ssm_prim_track1_kine_energy_cal", &tagger_info.ssm_prim_track1_kine_energy_cal, "ssm_prim_track1_kine_energy_cal/F");
      T_tagger->Branch("ssm_prim_track1_medium_dq_dx", &tagger_info.ssm_prim_track1_medium_dq_dx, "ssm_prim_track1_medium_dq_dx/F");
      T_tagger->Branch("ssm_prim_track1_x_dir", &tagger_info.ssm_prim_track1_x_dir, "ssm_prim_track1_x_dir/F");
      T_tagger->Branch("ssm_prim_track1_y_dir", &tagger_info.ssm_prim_track1_y_dir, "ssm_prim_track1_y_dir/F");
      T_tagger->Branch("ssm_prim_track1_z_dir", &tagger_info.ssm_prim_track1_z_dir, "ssm_prim_track1_z_dir/F");
      T_tagger->Branch("ssm_prim_track1_add_daught_track_counts_1", &tagger_info.ssm_prim_track1_add_daught_track_counts_1, "ssm_prim_track1_add_daught_track_counts_1/F");
      T_tagger->Branch("ssm_prim_track1_add_daught_all_counts_1", &tagger_info.ssm_prim_track1_add_daught_all_counts_1, "ssm_prim_track1_add_daught_all_counts_1/F");
      T_tagger->Branch("ssm_prim_track1_add_daught_track_counts_5", &tagger_info.ssm_prim_track1_add_daught_track_counts_5, "ssm_prim_track1_add_daught_track_counts_5/F");
      T_tagger->Branch("ssm_prim_track1_add_daught_all_counts_5", &tagger_info.ssm_prim_track1_add_daught_all_counts_5, "ssm_prim_track1_add_daught_all_counts_5/F");
      T_tagger->Branch("ssm_prim_track1_add_daught_track_counts_11", &tagger_info.ssm_prim_track1_add_daught_track_counts_11, "ssm_prim_track1_add_daught_track_counts_11/F");
      T_tagger->Branch("ssm_prim_track1_add_daught_all_counts_11", &tagger_info.ssm_prim_track1_add_daught_all_counts_11, "ssm_prim_track1_add_daught_all_counts_11/F");
      //properties of sub-leading other primary track
      T_tagger->Branch("ssm_prim_track2_pdg", &tagger_info.ssm_prim_track2_pdg, "ssm_prim_track2_pdg/F");
      T_tagger->Branch("ssm_prim_track2_score_mu_fwd", &tagger_info.ssm_prim_track2_score_mu_fwd, "ssm_prim_track2_score_mu_fwd/F");
      T_tagger->Branch("ssm_prim_track2_score_p_fwd", &tagger_info.ssm_prim_track2_score_p_fwd, "ssm_prim_track2_score_p_fwd/F");
      T_tagger->Branch("ssm_prim_track2_score_e_fwd", &tagger_info.ssm_prim_track2_score_e_fwd, "ssm_prim_track2_score_e_fwd/F");
      T_tagger->Branch("ssm_prim_track2_score_mu_bck", &tagger_info.ssm_prim_track2_score_mu_bck, "ssm_prim_track2_score_mu_bck/F");
      T_tagger->Branch("ssm_prim_track2_score_p_bck", &tagger_info.ssm_prim_track2_score_p_bck, "ssm_prim_track2_score_p_bck/F");
      T_tagger->Branch("ssm_prim_track2_score_e_bck", &tagger_info.ssm_prim_track2_score_e_bck, "ssm_prim_track2_score_e_bck/F");
      T_tagger->Branch("ssm_prim_track2_length", &tagger_info.ssm_prim_track2_length, "ssm_prim_track2_length/F");
      T_tagger->Branch("ssm_prim_track2_direct_length", &tagger_info.ssm_prim_track2_direct_length, "ssm_prim_track2_direct_length/F");
      T_tagger->Branch("ssm_prim_track2_length_ratio", &tagger_info.ssm_prim_track2_length_ratio, "ssm_prim_track2_length_ratio/F");
      T_tagger->Branch("ssm_prim_track2_max_dev", &tagger_info.ssm_prim_track2_max_dev, "ssm_prim_track2_max_dev/F");
      T_tagger->Branch("ssm_prim_track2_kine_energy_range", &tagger_info.ssm_prim_track2_kine_energy_range, "ssm_prim_track2_kine_energy_range/F");
      T_tagger->Branch("ssm_prim_track2_kine_energy_range_mu", &tagger_info.ssm_prim_track2_kine_energy_range_mu, "ssm_prim_track2_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_prim_track2_kine_energy_range_p", &tagger_info.ssm_prim_track2_kine_energy_range_p, "ssm_prim_track2_kine_energy_range_p/F");
      T_tagger->Branch("ssm_prim_track2_kine_energy_range_e", &tagger_info.ssm_prim_track2_kine_energy_range_e, "ssm_prim_track2_kine_energy_range_e/F");
      T_tagger->Branch("ssm_prim_track2_kine_energy_cal", &tagger_info.ssm_prim_track2_kine_energy_cal, "ssm_prim_track2_kine_energy_cal/F");
      T_tagger->Branch("ssm_prim_track2_medium_dq_dx", &tagger_info.ssm_prim_track2_medium_dq_dx, "ssm_prim_track2_medium_dq_dx/F");
      T_tagger->Branch("ssm_prim_track2_x_dir", &tagger_info.ssm_prim_track2_x_dir, "ssm_prim_track2_x_dir/F");
      T_tagger->Branch("ssm_prim_track2_y_dir", &tagger_info.ssm_prim_track2_y_dir, "ssm_prim_track2_y_dir/F");
      T_tagger->Branch("ssm_prim_track2_z_dir", &tagger_info.ssm_prim_track2_z_dir, "ssm_prim_track2_z_dir/F");
      T_tagger->Branch("ssm_prim_track2_add_daught_track_counts_1", &tagger_info.ssm_prim_track2_add_daught_track_counts_1, "ssm_prim_track2_add_daught_track_counts_1/F");
      T_tagger->Branch("ssm_prim_track2_add_daught_all_counts_1", &tagger_info.ssm_prim_track2_add_daught_all_counts_1, "ssm_prim_track2_add_daught_all_counts_1/F");
      T_tagger->Branch("ssm_prim_track2_add_daught_track_counts_5", &tagger_info.ssm_prim_track2_add_daught_track_counts_5, "ssm_prim_track2_add_daught_track_counts_5/F");
      T_tagger->Branch("ssm_prim_track2_add_daught_all_counts_5", &tagger_info.ssm_prim_track2_add_daught_all_counts_5, "ssm_prim_track2_add_daught_all_counts_5/F");
      T_tagger->Branch("ssm_prim_track2_add_daught_track_counts_11", &tagger_info.ssm_prim_track2_add_daught_track_counts_11, "ssm_prim_track2_add_daught_track_counts_11/F");
      T_tagger->Branch("ssm_prim_track2_add_daught_all_counts_11", &tagger_info.ssm_prim_track2_add_daught_all_counts_11, "ssm_prim_track2_add_daught_all_counts_11/F");
      //properties of leading daughter track
      T_tagger->Branch("ssm_daught_track1_pdg", &tagger_info.ssm_daught_track1_pdg, "ssm_daught_track1_pdg/F");
      T_tagger->Branch("ssm_daught_track1_score_mu_fwd", &tagger_info.ssm_daught_track1_score_mu_fwd, "ssm_daught_track1_score_mu_fwd/F");
      T_tagger->Branch("ssm_daught_track1_score_p_fwd", &tagger_info.ssm_daught_track1_score_p_fwd, "ssm_daught_track1_score_p_fwd/F");
      T_tagger->Branch("ssm_daught_track1_score_e_fwd", &tagger_info.ssm_daught_track1_score_e_fwd, "ssm_daught_track1_score_e_fwd/F");
      T_tagger->Branch("ssm_daught_track1_score_mu_bck", &tagger_info.ssm_daught_track1_score_mu_bck, "ssm_daught_track1_score_mu_bck/F");
      T_tagger->Branch("ssm_daught_track1_score_p_bck", &tagger_info.ssm_daught_track1_score_p_bck, "ssm_daught_track1_score_p_bck/F");
      T_tagger->Branch("ssm_daught_track1_score_e_bck", &tagger_info.ssm_daught_track1_score_e_bck, "ssm_daught_track1_score_e_bck/F");
      T_tagger->Branch("ssm_daught_track1_length", &tagger_info.ssm_daught_track1_length, "ssm_daught_track1_length/F");
      T_tagger->Branch("ssm_daught_track1_direct_length", &tagger_info.ssm_daught_track1_direct_length, "ssm_daught_track1_direct_length/F");
      T_tagger->Branch("ssm_daught_track1_length_ratio", &tagger_info.ssm_daught_track1_length_ratio, "ssm_daught_track1_length_ratio/F");
      T_tagger->Branch("ssm_daught_track1_max_dev", &tagger_info.ssm_daught_track1_max_dev, "ssm_daught_track1_max_dev/F");
      T_tagger->Branch("ssm_daught_track1_kine_energy_range", &tagger_info.ssm_daught_track1_kine_energy_range, "ssm_daught_track1_kine_energy_range/F");
      T_tagger->Branch("ssm_daught_track1_kine_energy_range_mu", &tagger_info.ssm_daught_track1_kine_energy_range_mu, "ssm_daught_track1_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_daught_track1_kine_energy_range_p", &tagger_info.ssm_daught_track1_kine_energy_range_p, "ssm_daught_track1_kine_energy_range_p/F");
      T_tagger->Branch("ssm_daught_track1_kine_energy_range_e", &tagger_info.ssm_daught_track1_kine_energy_range_e, "ssm_daught_track1_kine_energy_range_e/F");
      T_tagger->Branch("ssm_daught_track1_kine_energy_cal", &tagger_info.ssm_daught_track1_kine_energy_cal, "ssm_daught_track1_kine_energy_cal/F");
      T_tagger->Branch("ssm_daught_track1_medium_dq_dx", &tagger_info.ssm_daught_track1_medium_dq_dx, "ssm_daught_track1_medium_dq_dx/F");
      T_tagger->Branch("ssm_daught_track1_x_dir", &tagger_info.ssm_daught_track1_x_dir, "ssm_daught_track1_x_dir/F");
      T_tagger->Branch("ssm_daught_track1_y_dir", &tagger_info.ssm_daught_track1_y_dir, "ssm_daught_track1_y_dir/F");
      T_tagger->Branch("ssm_daught_track1_z_dir", &tagger_info.ssm_daught_track1_z_dir, "ssm_daught_track1_z_dir/F");
      T_tagger->Branch("ssm_daught_track1_add_daught_track_counts_1", &tagger_info.ssm_daught_track1_add_daught_track_counts_1, "ssm_daught_track1_add_daught_track_counts_1/F");
      T_tagger->Branch("ssm_daught_track1_add_daught_all_counts_1", &tagger_info.ssm_daught_track1_add_daught_all_counts_1, "ssm_daught_track1_add_daught_all_counts_1/F");
      T_tagger->Branch("ssm_daught_track1_add_daught_track_counts_5", &tagger_info.ssm_daught_track1_add_daught_track_counts_5, "ssm_daught_track1_add_daught_track_counts_5/F");
      T_tagger->Branch("ssm_daught_track1_add_daught_all_counts_5", &tagger_info.ssm_daught_track1_add_daught_all_counts_5, "ssm_daught_track1_add_daught_all_counts_5/F");
      T_tagger->Branch("ssm_daught_track1_add_daught_track_counts_11", &tagger_info.ssm_daught_track1_add_daught_track_counts_11, "ssm_daught_track1_add_daught_track_counts_11/F");
      T_tagger->Branch("ssm_daught_track1_add_daught_all_counts_11", &tagger_info.ssm_daught_track1_add_daught_all_counts_11, "ssm_daught_track1_add_daught_all_counts_11/F");
      //properties of sub-leading daughter track
      T_tagger->Branch("ssm_daught_track2_pdg", &tagger_info.ssm_daught_track2_pdg, "ssm_daught_track2_pdg/F");
      T_tagger->Branch("ssm_daught_track2_score_mu_fwd", &tagger_info.ssm_daught_track2_score_mu_fwd, "ssm_daught_track2_score_mu_fwd/F");
      T_tagger->Branch("ssm_daught_track2_score_p_fwd", &tagger_info.ssm_daught_track2_score_p_fwd, "ssm_daught_track2_score_p_fwd/F");
      T_tagger->Branch("ssm_daught_track2_score_e_fwd", &tagger_info.ssm_daught_track2_score_e_fwd, "ssm_daught_track2_score_e_fwd/F");
      T_tagger->Branch("ssm_daught_track2_score_mu_bck", &tagger_info.ssm_daught_track2_score_mu_bck, "ssm_daught_track2_score_mu_bck/F");
      T_tagger->Branch("ssm_daught_track2_score_p_bck", &tagger_info.ssm_daught_track2_score_p_bck, "ssm_daught_track2_score_p_bck/F");
      T_tagger->Branch("ssm_daught_track2_score_e_bck", &tagger_info.ssm_daught_track2_score_e_bck, "ssm_daught_track2_score_e_bck/F");
      T_tagger->Branch("ssm_daught_track2_length", &tagger_info.ssm_daught_track2_length, "ssm_daught_track2_length/F");
      T_tagger->Branch("ssm_daught_track2_direct_length", &tagger_info.ssm_daught_track2_direct_length, "ssm_daught_track2_direct_length/F");
      T_tagger->Branch("ssm_daught_track2_length_ratio", &tagger_info.ssm_daught_track2_length_ratio, "ssm_daught_track2_length_ratio/F");
      T_tagger->Branch("ssm_daught_track2_max_dev", &tagger_info.ssm_daught_track2_max_dev, "ssm_daught_track2_max_dev/F");
      T_tagger->Branch("ssm_daught_track2_kine_energy_range", &tagger_info.ssm_daught_track2_kine_energy_range, "ssm_daught_track2_kine_energy_range/F");
      T_tagger->Branch("ssm_daught_track2_kine_energy_range_mu", &tagger_info.ssm_daught_track2_kine_energy_range_mu, "ssm_daught_track2_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_daught_track2_kine_energy_range_p", &tagger_info.ssm_daught_track2_kine_energy_range_p, "ssm_daught_track2_kine_energy_range_p/F");
      T_tagger->Branch("ssm_daught_track2_kine_energy_range_e", &tagger_info.ssm_daught_track2_kine_energy_range_e, "ssm_daught_track2_kine_energy_range_e/F");
      T_tagger->Branch("ssm_daught_track2_kine_energy_cal", &tagger_info.ssm_daught_track2_kine_energy_cal, "ssm_daught_track2_kine_energy_cal/F");
      T_tagger->Branch("ssm_daught_track2_medium_dq_dx", &tagger_info.ssm_daught_track2_medium_dq_dx, "ssm_daught_track2_medium_dq_dx/F");
      T_tagger->Branch("ssm_daught_track2_x_dir", &tagger_info.ssm_daught_track2_x_dir, "ssm_daught_track2_x_dir/F");
      T_tagger->Branch("ssm_daught_track2_y_dir", &tagger_info.ssm_daught_track2_y_dir, "ssm_daught_track2_y_dir/F");
      T_tagger->Branch("ssm_daught_track2_z_dir", &tagger_info.ssm_daught_track2_z_dir, "ssm_daught_track2_z_dir/F");
      T_tagger->Branch("ssm_daught_track2_add_daught_track_counts_1", &tagger_info.ssm_daught_track2_add_daught_track_counts_1, "ssm_daught_track2_add_daught_track_counts_1/F");
      T_tagger->Branch("ssm_daught_track2_add_daught_all_counts_1", &tagger_info.ssm_daught_track2_add_daught_all_counts_1, "ssm_pdaught_track2_add_daught_all_counts_1/F");
      T_tagger->Branch("ssm_daught_track2_add_daught_track_counts_5", &tagger_info.ssm_daught_track2_add_daught_track_counts_5, "ssm_daught_track2_add_daught_track_counts_5/F");
      T_tagger->Branch("ssm_daught_track2_add_daught_all_counts_5", &tagger_info.ssm_daught_track2_add_daught_all_counts_5, "ssm_pdaught_track2_add_daught_all_counts_5/F");
      T_tagger->Branch("ssm_daught_track2_add_daught_track_counts_11", &tagger_info.ssm_daught_track2_add_daught_track_counts_11, "ssm_daught_track2_add_daught_track_counts_11/F");
      T_tagger->Branch("ssm_daught_track2_add_daught_all_counts_11", &tagger_info.ssm_daught_track2_add_daught_all_counts_11, "ssm_pdaught_track2_add_daught_all_counts_11/F");
      //properties of leading other primary shower
      T_tagger->Branch("ssm_prim_shw1_pdg", &tagger_info.ssm_prim_shw1_pdg, "ssm_prim_shw1_pdg/F");
      T_tagger->Branch("ssm_prim_shw1_score_mu_fwd", &tagger_info.ssm_prim_shw1_score_mu_fwd, "ssm_prim_shw1_score_mu_fwd/F");
      T_tagger->Branch("ssm_prim_shw1_score_p_fwd", &tagger_info.ssm_prim_shw1_score_p_fwd, "ssm_prim_shw1_score_p_fwd/F");
      T_tagger->Branch("ssm_prim_shw1_score_e_fwd", &tagger_info.ssm_prim_shw1_score_e_fwd, "ssm_prim_shw1_score_e_fwd/F");
      T_tagger->Branch("ssm_prim_shw1_score_mu_bck", &tagger_info.ssm_prim_shw1_score_mu_bck, "ssm_prim_shw1_score_mu_bck/F");
      T_tagger->Branch("ssm_prim_shw1_score_p_bck", &tagger_info.ssm_prim_shw1_score_p_bck, "ssm_prim_shw1_score_p_bck/F");
      T_tagger->Branch("ssm_prim_shw1_score_e_bck", &tagger_info.ssm_prim_shw1_score_e_bck, "ssm_prim_shw1_score_e_bck/F");
      T_tagger->Branch("ssm_prim_shw1_length", &tagger_info.ssm_prim_shw1_length, "ssm_prim_shw1_length/F");
      T_tagger->Branch("ssm_prim_shw1_direct_length", &tagger_info.ssm_prim_shw1_direct_length, "ssm_prim_shw1_direct_length/F");
      T_tagger->Branch("ssm_prim_shw1_length_ratio", &tagger_info.ssm_prim_shw1_length_ratio, "ssm_prim_shw1_length_ratio/F");
      T_tagger->Branch("ssm_prim_shw1_max_dev", &tagger_info.ssm_prim_shw1_max_dev, "ssm_prim_shw1_max_dev/F");
      T_tagger->Branch("ssm_prim_shw1_kine_energy_range", &tagger_info.ssm_prim_shw1_kine_energy_range, "ssm_prim_shw1_kine_energy_range/F");
      T_tagger->Branch("ssm_prim_shw1_kine_energy_range_mu", &tagger_info.ssm_prim_shw1_kine_energy_range_mu, "ssm_prim_shw1_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_prim_shw1_kine_energy_range_p", &tagger_info.ssm_prim_shw1_kine_energy_range_p, "ssm_prim_shw1_kine_energy_range_p/F");
      T_tagger->Branch("ssm_prim_shw1_kine_energy_range_e", &tagger_info.ssm_prim_shw1_kine_energy_range_e, "ssm_prim_shw1_kine_energy_range_e/F");
      T_tagger->Branch("ssm_prim_shw1_kine_energy_cal", &tagger_info.ssm_prim_shw1_kine_energy_cal, "ssm_prim_shw1_kine_energy_cal/F");
      T_tagger->Branch("ssm_prim_shw1_kine_energy_best", &tagger_info.ssm_prim_shw1_kine_energy_best, "ssm_prim_shw1_kine_energy_best/F");
      T_tagger->Branch("ssm_prim_shw1_medium_dq_dx", &tagger_info.ssm_prim_shw1_medium_dq_dx, "ssm_prim_shw1_medium_dq_dx/F");
      T_tagger->Branch("ssm_prim_shw1_x_dir", &tagger_info.ssm_prim_shw1_x_dir, "ssm_prim_shw1_x_dir/F");
      T_tagger->Branch("ssm_prim_shw1_y_dir", &tagger_info.ssm_prim_shw1_y_dir, "ssm_prim_shw1_y_dir/F");
      T_tagger->Branch("ssm_prim_shw1_z_dir", &tagger_info.ssm_prim_shw1_z_dir, "ssm_prim_shw1_z_dir/F");
      T_tagger->Branch("ssm_prim_shw1_add_daught_track_counts_1", &tagger_info.ssm_prim_shw1_add_daught_track_counts_1, "ssm_prim_shw1_add_daught_track_counts_1/F");
      T_tagger->Branch("ssm_prim_shw1_add_daught_all_counts_1", &tagger_info.ssm_prim_shw1_add_daught_all_counts_1, "ssm_prim_shw1_add_daught_all_counts_1/F");
      T_tagger->Branch("ssm_prim_shw1_add_daught_track_counts_5", &tagger_info.ssm_prim_shw1_add_daught_track_counts_5, "ssm_prim_shw1_add_daught_track_counts_5/F");
      T_tagger->Branch("ssm_prim_shw1_add_daught_all_counts_5", &tagger_info.ssm_prim_shw1_add_daught_all_counts_5, "ssm_prim_shw1_add_daught_all_counts_5/F");
      T_tagger->Branch("ssm_prim_shw1_add_daught_track_counts_11", &tagger_info.ssm_prim_shw1_add_daught_track_counts_11, "ssm_prim_shw1_add_daught_track_counts_11/F");
      T_tagger->Branch("ssm_prim_shw1_add_daught_all_counts_11", &tagger_info.ssm_prim_shw1_add_daught_all_counts_11, "ssm_prim_shw1_add_daught_all_counts_11/F");
      //properties of sub-leading other primary shower
      T_tagger->Branch("ssm_prim_shw2_pdg", &tagger_info.ssm_prim_shw2_pdg, "ssm_prim_shw2_pdg/F");
      T_tagger->Branch("ssm_prim_shw2_score_mu_fwd", &tagger_info.ssm_prim_shw2_score_mu_fwd, "ssm_prim_shw2_score_mu_fwd/F");
      T_tagger->Branch("ssm_prim_shw2_score_p_fwd", &tagger_info.ssm_prim_shw2_score_p_fwd, "ssm_prim_shw2_score_p_fwd/F");
      T_tagger->Branch("ssm_prim_shw2_score_e_fwd", &tagger_info.ssm_prim_shw2_score_e_fwd, "ssm_prim_shw2_score_e_fwd/F");
      T_tagger->Branch("ssm_prim_shw2_score_mu_bck", &tagger_info.ssm_prim_shw2_score_mu_bck, "ssm_prim_shw2_score_mu_bck/F");
      T_tagger->Branch("ssm_prim_shw2_score_p_bck", &tagger_info.ssm_prim_shw2_score_p_bck, "ssm_prim_shw2_score_p_bck/F");
      T_tagger->Branch("ssm_prim_shw2_score_e_bck", &tagger_info.ssm_prim_shw2_score_e_bck, "ssm_prim_shw2_score_e_bck/F");
      T_tagger->Branch("ssm_prim_shw2_length", &tagger_info.ssm_prim_shw2_length, "ssm_prim_shw2_length/F");
      T_tagger->Branch("ssm_prim_shw2_direct_length", &tagger_info.ssm_prim_shw2_direct_length, "ssm_prim_shw2_direct_length/F");
      T_tagger->Branch("ssm_prim_shw2_length_ratio", &tagger_info.ssm_prim_shw2_length_ratio, "ssm_prim_shw2_length_ratio/F");
      T_tagger->Branch("ssm_prim_shw2_max_dev", &tagger_info.ssm_prim_shw2_max_dev, "ssm_prim_shw2_max_dev/F");
      T_tagger->Branch("ssm_prim_shw2_kine_energy_range", &tagger_info.ssm_prim_shw2_kine_energy_range, "ssm_prim_shw2_kine_energy_range/F");
      T_tagger->Branch("ssm_prim_shw2_kine_energy_range_mu", &tagger_info.ssm_prim_shw2_kine_energy_range_mu, "ssm_prim_shw2_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_prim_shw2_kine_energy_range_p", &tagger_info.ssm_prim_shw2_kine_energy_range_p, "ssm_prim_shw2_kine_energy_range_p/F");
      T_tagger->Branch("ssm_prim_shw2_kine_energy_range_e", &tagger_info.ssm_prim_shw2_kine_energy_range_e, "ssm_prim_shw2_kine_energy_range_e/F");
      T_tagger->Branch("ssm_prim_shw2_kine_energy_cal", &tagger_info.ssm_prim_shw2_kine_energy_cal, "ssm_prim_shw2_kine_energy_cal/F");
      T_tagger->Branch("ssm_prim_shw2_kine_energy_best", &tagger_info.ssm_prim_shw2_kine_energy_best, "ssm_prim_shw2_kine_energy_best/F");
      T_tagger->Branch("ssm_prim_shw2_medium_dq_dx", &tagger_info.ssm_prim_shw2_medium_dq_dx, "ssm_prim_shw2_medium_dq_dx/F");
      T_tagger->Branch("ssm_prim_shw2_x_dir", &tagger_info.ssm_prim_shw2_x_dir, "ssm_prim_shw2_x_dir/F");
      T_tagger->Branch("ssm_prim_shw2_y_dir", &tagger_info.ssm_prim_shw2_y_dir, "ssm_prim_shw2_y_dir/F");
      T_tagger->Branch("ssm_prim_shw2_z_dir", &tagger_info.ssm_prim_shw2_z_dir, "ssm_prim_shw2_z_dir/F");
      T_tagger->Branch("ssm_prim_shw2_add_daught_track_counts_1", &tagger_info.ssm_prim_shw2_add_daught_track_counts_1, "ssm_prim_shw2_add_daught_track_counts_1/F");
      T_tagger->Branch("ssm_prim_shw2_add_daught_all_counts_1", &tagger_info.ssm_prim_shw2_add_daught_all_counts_1, "ssm_prim_shw2_add_daught_all_counts_1/F");
      T_tagger->Branch("ssm_prim_shw2_add_daught_track_counts_5", &tagger_info.ssm_prim_shw2_add_daught_track_counts_5, "ssm_prim_shw2_add_daught_track_counts_5/F");
      T_tagger->Branch("ssm_prim_shw2_add_daught_all_counts_5", &tagger_info.ssm_prim_shw2_add_daught_all_counts_5, "ssm_prim_shw2_add_daught_all_counts_5/F");
      T_tagger->Branch("ssm_prim_shw2_add_daught_track_counts_11", &tagger_info.ssm_prim_shw2_add_daught_track_counts_11, "ssm_prim_shw2_add_daught_track_counts_11/F");
      T_tagger->Branch("ssm_prim_shw2_add_daught_all_counts_11", &tagger_info.ssm_prim_shw2_add_daught_all_counts_11, "ssm_prim_shw2_add_daught_all_counts_11/F");
      //properties of leading daughter shower
      T_tagger->Branch("ssm_daught_shw1_pdg", &tagger_info.ssm_daught_shw1_pdg, "ssm_daught_shw1_pdg/F");
      T_tagger->Branch("ssm_daught_shw1_score_mu_fwd", &tagger_info.ssm_daught_shw1_score_mu_fwd, "ssm_daught_shw1_score_mu_fwd/F");
      T_tagger->Branch("ssm_daught_shw1_score_p_fwd", &tagger_info.ssm_daught_shw1_score_p_fwd, "ssm_daught_shw1_score_p_fwd/F");
      T_tagger->Branch("ssm_daught_shw1_score_e_fwd", &tagger_info.ssm_daught_shw1_score_e_fwd, "ssm_daught_shw1_score_e_fwd/F");
      T_tagger->Branch("ssm_daught_shw1_score_mu_bck", &tagger_info.ssm_daught_shw1_score_mu_bck, "ssm_daught_shw1_score_mu_bck/F");
      T_tagger->Branch("ssm_daught_shw1_score_p_bck", &tagger_info.ssm_daught_shw1_score_p_bck, "ssm_daught_shw1_score_p_bck/F");
      T_tagger->Branch("ssm_daught_shw1_score_e_bck", &tagger_info.ssm_daught_shw1_score_e_bck, "ssm_daught_shw1_score_e_bck/F");
      T_tagger->Branch("ssm_daught_shw1_length", &tagger_info.ssm_daught_shw1_length, "ssm_daught_shw1_length/F");
      T_tagger->Branch("ssm_daught_shw1_direct_length", &tagger_info.ssm_daught_shw1_direct_length, "ssm_daught_shw1_direct_length/F");
      T_tagger->Branch("ssm_daught_shw1_length_ratio", &tagger_info.ssm_daught_shw1_length_ratio, "ssm_daught_shw1_length_ratio/F");
      T_tagger->Branch("ssm_daught_shw1_max_dev", &tagger_info.ssm_daught_shw1_max_dev, "ssm_daught_shw1_max_dev/F");
      T_tagger->Branch("ssm_daught_shw1_kine_energy_range", &tagger_info.ssm_daught_shw1_kine_energy_range, "ssm_daught_shw1_kine_energy_range/F");
      T_tagger->Branch("ssm_daught_shw1_kine_energy_range_mu", &tagger_info.ssm_daught_shw1_kine_energy_range_mu, "ssm_daught_shw1_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_daught_shw1_kine_energy_range_p", &tagger_info.ssm_daught_shw1_kine_energy_range_p, "ssm_daught_shw1_kine_energy_range_p/F");
      T_tagger->Branch("ssm_daught_shw1_kine_energy_range_e", &tagger_info.ssm_daught_shw1_kine_energy_range_e, "ssm_daught_shw1_kine_energy_range_e/F");
      T_tagger->Branch("ssm_daught_shw1_kine_energy_cal", &tagger_info.ssm_daught_shw1_kine_energy_cal, "ssm_daught_shw1_kine_energy_cal/F");
      T_tagger->Branch("ssm_daught_shw1_kine_energy_best", &tagger_info.ssm_daught_shw1_kine_energy_best, "ssm_daught_shw1_kine_energy_best/F");
      T_tagger->Branch("ssm_daught_shw1_medium_dq_dx", &tagger_info.ssm_daught_shw1_medium_dq_dx, "ssm_daught_shw1_medium_dq_dx/F");
      T_tagger->Branch("ssm_daught_shw1_x_dir", &tagger_info.ssm_daught_shw1_x_dir, "ssm_daught_shw1_x_dir/F");
      T_tagger->Branch("ssm_daught_shw1_y_dir", &tagger_info.ssm_daught_shw1_y_dir, "ssm_daught_shw1_y_dir/F");
      T_tagger->Branch("ssm_daught_shw1_z_dir", &tagger_info.ssm_daught_shw1_z_dir, "ssm_daught_shw1_z_dir/F");
      T_tagger->Branch("ssm_daught_shw1_add_daught_track_counts_1", &tagger_info.ssm_daught_shw1_add_daught_track_counts_1, "ssm_daught_shw1_add_daught_track_counts_1/F");
      T_tagger->Branch("ssm_daught_shw1_add_daught_all_counts_1", &tagger_info.ssm_daught_shw1_add_daught_all_counts_1, "ssm_daught_shw1_add_daught_all_counts_1/F");
      T_tagger->Branch("ssm_daught_shw1_add_daught_track_counts_5", &tagger_info.ssm_daught_shw1_add_daught_track_counts_5, "ssm_daught_shw1_add_daught_track_counts_5/F");
      T_tagger->Branch("ssm_daught_shw1_add_daught_all_counts_5", &tagger_info.ssm_daught_shw1_add_daught_all_counts_5, "ssm_daught_shw1_add_daught_all_counts_5/F");
      T_tagger->Branch("ssm_daught_shw1_add_daught_track_counts_11", &tagger_info.ssm_daught_shw1_add_daught_track_counts_11, "ssm_daught_shw1_add_daught_track_counts_11/F");
      T_tagger->Branch("ssm_daught_shw1_add_daught_all_counts_11", &tagger_info.ssm_daught_shw1_add_daught_all_counts_11, "ssm_daught_shw1_add_daught_all_counts_11/F");
      //properties of sub-leading daughter shower
      T_tagger->Branch("ssm_daught_shw2_pdg", &tagger_info.ssm_daught_shw2_pdg, "ssm_daught_shw2_pdg/F");
      T_tagger->Branch("ssm_daught_shw2_score_mu_fwd", &tagger_info.ssm_daught_shw2_score_mu_fwd, "ssm_daught_shw2_score_mu_fwd/F");
      T_tagger->Branch("ssm_daught_shw2_score_p_fwd", &tagger_info.ssm_daught_shw2_score_p_fwd, "ssm_daught_shw2_score_p_fwd/F");
      T_tagger->Branch("ssm_daught_shw2_score_e_fwd", &tagger_info.ssm_daught_shw2_score_e_fwd, "ssm_daught_shw2_score_e_fwd/F");
      T_tagger->Branch("ssm_daught_shw2_score_mu_bck", &tagger_info.ssm_daught_shw2_score_mu_bck, "ssm_daught_shw2_score_mu_bck/F");
      T_tagger->Branch("ssm_daught_shw2_score_p_bck", &tagger_info.ssm_daught_shw2_score_p_bck, "ssm_daught_shw2_score_p_bck/F");
      T_tagger->Branch("ssm_daught_shw2_score_e_bck", &tagger_info.ssm_daught_shw2_score_e_bck, "ssm_daught_shw2_score_e_bck/F");
      T_tagger->Branch("ssm_daught_shw2_length", &tagger_info.ssm_daught_shw2_length, "ssm_daught_shw2_length/F");
      T_tagger->Branch("ssm_daught_shw2_direct_length", &tagger_info.ssm_daught_shw2_direct_length, "ssm_daught_shw2_direct_length/F");
      T_tagger->Branch("ssm_daught_shw2_length_ratio", &tagger_info.ssm_daught_shw2_length_ratio, "ssm_daught_shw2_length_ratio/F");
      T_tagger->Branch("ssm_daught_shw2_max_dev", &tagger_info.ssm_daught_shw2_max_dev, "ssm_daught_shw2_max_dev/F");
      T_tagger->Branch("ssm_daught_shw2_kine_energy_range", &tagger_info.ssm_daught_shw2_kine_energy_range, "ssm_daught_shw2_kine_energy_range/F");
      T_tagger->Branch("ssm_daught_shw2_kine_energy_range_mu", &tagger_info.ssm_daught_shw2_kine_energy_range_mu, "ssm_daught_shw2_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_daught_shw2_kine_energy_range_p", &tagger_info.ssm_daught_shw2_kine_energy_range_p, "ssm_daught_shw2_kine_energy_range_p/F");
      T_tagger->Branch("ssm_daught_shw2_kine_energy_range_e", &tagger_info.ssm_daught_shw2_kine_energy_range_e, "ssm_daught_shw2_kine_energy_range_e/F");
      T_tagger->Branch("ssm_daught_shw2_kine_energy_cal", &tagger_info.ssm_daught_shw2_kine_energy_cal, "ssm_daught_shw2_kine_energy_cal/F");
      T_tagger->Branch("ssm_daught_shw2_kine_energy_best", &tagger_info.ssm_daught_shw2_kine_energy_best, "ssm_daught_shw2_kine_energy_best/F");
      T_tagger->Branch("ssm_daught_shw2_medium_dq_dx", &tagger_info.ssm_daught_shw2_medium_dq_dx, "ssm_daught_shw2_medium_dq_dx/F");
      T_tagger->Branch("ssm_daught_shw2_x_dir", &tagger_info.ssm_daught_shw2_x_dir, "ssm_daught_shw2_x_dir/F");
      T_tagger->Branch("ssm_daught_shw2_y_dir", &tagger_info.ssm_daught_shw2_y_dir, "ssm_daught_shw2_y_dir/F");
      T_tagger->Branch("ssm_daught_shw2_z_dir", &tagger_info.ssm_daught_shw2_z_dir, "ssm_daught_shw2_z_dir/F");
      T_tagger->Branch("ssm_daught_shw2_add_daught_track_counts_1", &tagger_info.ssm_daught_shw2_add_daught_track_counts_1, "ssm_daught_shw2_add_daught_track_counts_1/F");
      T_tagger->Branch("ssm_daught_shw2_add_daught_all_counts_1", &tagger_info.ssm_daught_shw2_add_daught_all_counts_1, "ssm_pdaught_shw2_add_daught_all_counts_1/F");
      T_tagger->Branch("ssm_daught_shw2_add_daught_track_counts_5", &tagger_info.ssm_daught_shw2_add_daught_track_counts_5, "ssm_daught_shw2_add_daught_track_counts_5/F");
      T_tagger->Branch("ssm_daught_shw2_add_daught_all_counts_5", &tagger_info.ssm_daught_shw2_add_daught_all_counts_5, "ssm_pdaught_shw2_add_daught_all_counts_5/F");
      T_tagger->Branch("ssm_daught_shw2_add_daught_track_counts_11", &tagger_info.ssm_daught_shw2_add_daught_track_counts_11, "ssm_daught_shw2_add_daught_track_counts_11/F");
      T_tagger->Branch("ssm_daught_shw2_add_daught_all_counts_11", &tagger_info.ssm_daught_shw2_add_daught_all_counts_11, "ssm_pdaught_shw2_add_daught_all_counts_11/F");
      //event level properties
      T_tagger->Branch("ssm_nu_angle_z", &tagger_info.ssm_nu_angle_z, "ssm_nu_angle_z/F");
      T_tagger->Branch("ssm_nu_angle_target", &tagger_info.ssm_nu_angle_target, "ssm_nu_angle_target/F");
      T_tagger->Branch("ssm_nu_angle_absorber", &tagger_info.ssm_nu_angle_absorber, "ssm_nu_angle_absorber/F");
      T_tagger->Branch("ssm_nu_angle_vertical", &tagger_info.ssm_nu_angle_vertical, "ssm_nu_angle_vertical/F");
      T_tagger->Branch("ssm_con_nu_angle_z", &tagger_info.ssm_con_nu_angle_z, "ssm_con_nu_angle_z/F");
      T_tagger->Branch("ssm_con_nu_angle_target", &tagger_info.ssm_con_nu_angle_target, "ssm_con_nu_angle_target/F");
      T_tagger->Branch("ssm_con_nu_angle_absorber", &tagger_info.ssm_con_nu_angle_absorber, "ssm_con_nu_angle_absorber/F");
      T_tagger->Branch("ssm_con_nu_angle_vertical", &tagger_info.ssm_con_nu_angle_vertical, "ssm_con_nu_angle_vertical/F");
      T_tagger->Branch("ssm_prim_nu_angle_z", &tagger_info.ssm_prim_nu_angle_z, "ssm_prim_nu_angle_z/F");
      T_tagger->Branch("ssm_prim_nu_angle_target", &tagger_info.ssm_prim_nu_angle_target, "ssm_prim_nu_angle_target/F");
      T_tagger->Branch("ssm_prim_nu_angle_absorber", &tagger_info.ssm_prim_nu_angle_absorber, "ssm_prim_nu_angle_absorber/F");
      T_tagger->Branch("ssm_prim_nu_angle_vertical", &tagger_info.ssm_prim_nu_angle_vertical, "ssm_prim_nu_angle_vertical/F");
      T_tagger->Branch("ssm_track_angle_z", &tagger_info.ssm_track_angle_z, "ssm_track_angle_z/F");
      T_tagger->Branch("ssm_track_angle_target", &tagger_info.ssm_track_angle_target, "ssm_track_angle_target/F");
      T_tagger->Branch("ssm_track_angle_absorber", &tagger_info.ssm_track_angle_absorber, "ssm_track_angle_absorber/F");
      T_tagger->Branch("ssm_track_angle_vertical", &tagger_info.ssm_track_angle_vertical, "ssm_track_angle_vertical/F");
      T_tagger->Branch("ssm_vtxX", &tagger_info.ssm_vtxX, "ssm_vtxX/F");
      T_tagger->Branch("ssm_vtxY", &tagger_info.ssm_vtxY, "ssm_vtxY/F");
      T_tagger->Branch("ssm_vtxZ", &tagger_info.ssm_vtxZ, "ssm_vtxZ/F");

      //off vertex stuff
      T_tagger->Branch("ssm_offvtx_length",&tagger_info.ssm_offvtx_length,"ssm_offvtx_length/F");
      T_tagger->Branch("ssm_offvtx_energy",&tagger_info.ssm_offvtx_energy,"ssm_offvtx_energy/F");
      T_tagger->Branch("ssm_n_offvtx_tracks_1",&tagger_info.ssm_n_offvtx_tracks_1,"ssm_n_offvtx_tracks_1/F");
      T_tagger->Branch("ssm_n_offvtx_tracks_3",&tagger_info.ssm_n_offvtx_tracks_3,"ssm_n_offvtx_tracks_3/F");
      T_tagger->Branch("ssm_n_offvtx_tracks_5",&tagger_info.ssm_n_offvtx_tracks_5,"ssm_n_offvtx_tracks_5/F");
      T_tagger->Branch("ssm_n_offvtx_tracks_8",&tagger_info.ssm_n_offvtx_tracks_8,"ssm_n_offvtx_tracks_8/F");
      T_tagger->Branch("ssm_n_offvtx_tracks_11",&tagger_info.ssm_n_offvtx_tracks_11,"ssm_n_offvtx_tracks_11/F");
      T_tagger->Branch("ssm_n_offvtx_showers_1",&tagger_info.ssm_n_offvtx_showers_1,"ssm_n_offvtx_showers_1/F");
      T_tagger->Branch("ssm_n_offvtx_showers_3",&tagger_info.ssm_n_offvtx_showers_3,"ssm_n_offvtx_showers_3/F");
      T_tagger->Branch("ssm_n_offvtx_showers_5",&tagger_info.ssm_n_offvtx_showers_5,"ssm_n_offvtx_showers_5/F");
      T_tagger->Branch("ssm_n_offvtx_showers_8",&tagger_info.ssm_n_offvtx_showers_8,"ssm_n_offvtx_showers_8/F");
      T_tagger->Branch("ssm_n_offvtx_showers_11",&tagger_info.ssm_n_offvtx_showers_11,"ssm_n_offvtx_showers_11/F");
      //properties of leading off vertex track
      T_tagger->Branch("ssm_offvtx_track1_pdg",&tagger_info.ssm_offvtx_track1_pdg,"ssm_offvtx_track1_pdg/F");
      T_tagger->Branch("ssm_offvtx_track1_score_mu_fwd",&tagger_info.ssm_offvtx_track1_score_mu_fwd,"ssm_offvtx_track1_score_mu_fwd/F");
      T_tagger->Branch("ssm_offvtx_track1_score_p_fwd",&tagger_info.ssm_offvtx_track1_score_p_fwd,"ssm_offvtx_track1_score_p_fwd/F");
      T_tagger->Branch("ssm_offvtx_track1_score_e_fwd",&tagger_info.ssm_offvtx_track1_score_e_fwd,"ssm_offvtx_track1_score_e_fwd/F");
      T_tagger->Branch("ssm_offvtx_track1_score_mu_bck",&tagger_info.ssm_offvtx_track1_score_mu_bck,"ssm_offvtx_track1_score_mu_bck/F");
      T_tagger->Branch("ssm_offvtx_track1_score_p_bck",&tagger_info.ssm_offvtx_track1_score_p_bck,"ssm_offvtx_track1_score_p_bck/F");
      T_tagger->Branch("ssm_offvtx_track1_score_e_bck",&tagger_info.ssm_offvtx_track1_score_e_bck,"ssm_offvtx_track1_score_e_bck/F");
      T_tagger->Branch("ssm_offvtx_track1_length",&tagger_info.ssm_offvtx_track1_length,"ssm_offvtx_track1_length/F");
      T_tagger->Branch("ssm_offvtx_track1_direct_length",&tagger_info.ssm_offvtx_track1_direct_length,"ssm_offvtx_track1_direct_length/F");
      T_tagger->Branch("ssm_offvtx_track1_max_dev",&tagger_info.ssm_offvtx_track1_max_dev,"ssm_offvtx_track1_max_dev/F");
      T_tagger->Branch("ssm_offvtx_track1_kine_energy_range",&tagger_info.ssm_offvtx_track1_kine_energy_range,"ssm_offvtx_track1_kine_energy_range/F");
      T_tagger->Branch("ssm_offvtx_track1_kine_energy_range_mu",&tagger_info.ssm_offvtx_track1_kine_energy_range_mu,"ssm_offvtx_track1_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_offvtx_track1_kine_energy_range_p",&tagger_info.ssm_offvtx_track1_kine_energy_range_p,"ssm_offvtx_track1_kine_energy_range_p/F");
      T_tagger->Branch("ssm_offvtx_track1_kine_energy_range_e",&tagger_info.ssm_offvtx_track1_kine_energy_range_e,"ssm_offvtx_track1_kine_energy_range_e/F");
      T_tagger->Branch("ssm_offvtx_track1_kine_energy_cal",&tagger_info.ssm_offvtx_track1_kine_energy_cal,"ssm_offvtx_track1_kine_energy_cal/F");
      T_tagger->Branch("ssm_offvtx_track1_medium_dq_dx",&tagger_info.ssm_offvtx_track1_medium_dq_dx,"ssm_offvtx_track1_medium_dq_dx/F");
      T_tagger->Branch("ssm_offvtx_track1_x_dir",&tagger_info.ssm_offvtx_track1_x_dir,"ssm_offvtx_track1_x_dir/F");
      T_tagger->Branch("ssm_offvtx_track1_y_dir",&tagger_info.ssm_offvtx_track1_y_dir,"ssm_offvtx_track1_y_dir/F");
      T_tagger->Branch("ssm_offvtx_track1_z_dir",&tagger_info.ssm_offvtx_track1_z_dir,"ssm_offvtx_track1_z_dir/F");
      T_tagger->Branch("ssm_offvtx_track1_dist_mainvtx",&tagger_info.ssm_offvtx_track1_dist_mainvtx,"ssm_offvtx_track1_dist_mainvtx/F");
      //properties of leading off vertex shower
      T_tagger->Branch("ssm_offvtx_shw1_pdg_offvtx",&tagger_info.ssm_offvtx_shw1_pdg_offvtx,"ssm_offvtx_shw1_pdg_offvtx/F");
      T_tagger->Branch("ssm_offvtx_shw1_score_mu_fwd",&tagger_info.ssm_offvtx_shw1_score_mu_fwd,"ssm_offvtx_shw1_score_mu_fwd/F");
      T_tagger->Branch("ssm_offvtx_shw1_score_p_fwd",&tagger_info.ssm_offvtx_shw1_score_p_fwd,"ssm_offvtx_shw1_score_p_fwd/F");
      T_tagger->Branch("ssm_offvtx_shw1_score_e_fwd",&tagger_info.ssm_offvtx_shw1_score_e_fwd,"ssm_offvtx_shw1_score_e_fwd/F");
      T_tagger->Branch("ssm_offvtx_shw1_score_mu_bck",&tagger_info.ssm_offvtx_shw1_score_mu_bck,"ssm_offvtx_shw1_score_mu_bck/F");
      T_tagger->Branch("ssm_offvtx_shw1_score_p_bck",&tagger_info.ssm_offvtx_shw1_score_p_bck,"ssm_offvtx_shw1_score_p_bck/F");
      T_tagger->Branch("ssm_offvtx_shw1_score_e_bck",&tagger_info.ssm_offvtx_shw1_score_e_bck,"ssm_offvtx_shw1_score_e_bck/F");
      T_tagger->Branch("ssm_offvtx_shw1_length",&tagger_info.ssm_offvtx_shw1_length,"ssm_offvtx_shw1_length/F");
      T_tagger->Branch("ssm_offvtx_shw1_direct_length",&tagger_info.ssm_offvtx_shw1_direct_length,"ssm_offvtx_shw1_direct_length/F");
      T_tagger->Branch("ssm_offvtx_shw1_max_dev",&tagger_info.ssm_offvtx_shw1_max_dev,"ssm_offvtx_shw1_max_dev/F");
      T_tagger->Branch("ssm_offvtx_shw1_kine_energy_best",&tagger_info.ssm_offvtx_shw1_kine_energy_best,"ssm_offvtx_shw1_kine_energy_best/F");
      T_tagger->Branch("ssm_offvtx_shw1_kine_energy_range",&tagger_info.ssm_offvtx_shw1_kine_energy_range,"ssm_offvtx_shw1_kine_energy_range/F");
      T_tagger->Branch("ssm_offvtx_shw1_kine_energy_range_mu",&tagger_info.ssm_offvtx_shw1_kine_energy_range_mu,"ssm_offvtx_shw1_kine_energy_range_mu/F");
      T_tagger->Branch("ssm_offvtx_shw1_kine_energy_range_p",&tagger_info.ssm_offvtx_shw1_kine_energy_range_p,"ssm_offvtx_shw1_kine_energy_range_p/F");
      T_tagger->Branch("ssm_offvtx_shw1_kine_energy_range_e",&tagger_info.ssm_offvtx_shw1_kine_energy_range_e,"ssm_offvtx_shw1_kine_energy_range_e/F");
      T_tagger->Branch("ssm_offvtx_shw1_kine_energy_cal",&tagger_info.ssm_offvtx_shw1_kine_energy_cal,"ssm_offvtx_shw1_kine_energy_cal/F");
      T_tagger->Branch("ssm_offvtx_shw1_medium_dq_dx",&tagger_info.ssm_offvtx_shw1_medium_dq_dx,"ssm_offvtx_shw1_medium_dq_dx/F");
      T_tagger->Branch("ssm_offvtx_shw1_x_dir",&tagger_info.ssm_offvtx_shw1_x_dir,"ssm_offvtx_shw1_x_dir/F");
      T_tagger->Branch("ssm_offvtx_shw1_y_dir",&tagger_info.ssm_offvtx_shw1_y_dir,"ssm_offvtx_shw1_y_dir/F");
      T_tagger->Branch("ssm_offvtx_shw1_z_dir",&tagger_info.ssm_offvtx_shw1_z_dir,"ssm_offvtx_shw1_z_dir/F");
      T_tagger->Branch("ssm_offvtx_shw1_dist_mainvtx",&tagger_info.ssm_offvtx_shw1_dist_mainvtx,"ssm_offvtx_shw1_dist_mainvtx/F");
      // Sapcepoints
      T_tagger->Branch("ssmsp_Ntrack", &tagger_info.ssmsp_Ntrack, "ssmsp_Ntrack/I");
      T_tagger->Branch("ssmsp_Nsp", &tagger_info.ssmsp_Nsp);
      T_tagger->Branch("ssmsp_Nsp_tot", &tagger_info.ssmsp_Nsp_tot, "ssmsp_Nsp_tot/I");
      T_tagger->Branch("ssmsp_pdg", &tagger_info.ssmsp_pdg);
      T_tagger->Branch("ssmsp_id", &tagger_info.ssmsp_id);
      T_tagger->Branch("ssmsp_mother", &tagger_info.ssmsp_mother);
      T_tagger->Branch("ssmsp_x", &tagger_info.ssmsp_x);
      T_tagger->Branch("ssmsp_y", &tagger_info.ssmsp_y);
      T_tagger->Branch("ssmsp_z", &tagger_info.ssmsp_z);
      T_tagger->Branch("ssmsp_dx", &tagger_info.ssmsp_dx);
      T_tagger->Branch("ssmsp_dQ", &tagger_info.ssmsp_dQ);
      T_tagger->Branch("ssmsp_KE", &tagger_info.ssmsp_KE);
      T_tagger->Branch("ssmsp_containing_shower_id", &tagger_info.ssmsp_containing_shower_id);
      T_tagger->Branch("ssmsp_containing_shower_ke", &tagger_info.ssmsp_containing_shower_ke);
      T_tagger->Branch("ssmsp_containing_shower_flag", &tagger_info.ssmsp_containing_shower_flag);
      // Also add the kine vars to the KDAR BDT tagger
      WCPPID::KineInfo ssm_kine_tree;
      ssm_kine_tree.kine_pio_flag = 0;
      T_tagger->Branch("ssm_kine_reco_Enu",&ssm_kine_tree.kine_reco_Enu,"ssm_kine_reco_Enu/F");
      T_tagger->Branch("ssm_kine_reco_add_energy",&ssm_kine_tree.kine_reco_add_energy,"ssm_kine_reco_add_energy/F");
      T_tagger->Branch("ssm_kine_energy_particle",&ssm_kine_tree.kine_energy_particle);
      T_tagger->Branch("ssm_kine_energy_info",&ssm_kine_tree.kine_energy_info);
      T_tagger->Branch("ssm_kine_particle_type",&ssm_kine_tree.kine_particle_type);
      T_tagger->Branch("ssm_kine_energy_included",&ssm_kine_tree.kine_energy_included);
      T_tagger->Branch("ssm_kine_pio_mass",&ssm_kine_tree.kine_pio_mass,"ssm_kine_pio_mass/F");
      T_tagger->Branch("ssm_kine_pio_flag",&ssm_kine_tree.kine_pio_flag,"ssm_kine_pio_flag/I");
      T_tagger->Branch("ssm_kine_pio_vtx_dis",&ssm_kine_tree.kine_pio_vtx_dis,"ssm_kine_pio_vtx_dis/F");
      T_tagger->Branch("ssm_kine_pio_energy_1",&ssm_kine_tree.kine_pio_energy_1,"ssm_kine_pio_energy_1/F");
      T_tagger->Branch("ssm_kine_pio_theta_1",&ssm_kine_tree.kine_pio_theta_1,"ssm_kine_pio_theta_1/F");
      T_tagger->Branch("ssm_kine_pio_phi_1",&ssm_kine_tree.kine_pio_phi_1,"ssm_kine_pio_phi_1/F");
      T_tagger->Branch("ssm_kine_pio_dis_1",&ssm_kine_tree.kine_pio_dis_1,"ssm_kine_pio_dis_1/F");
      T_tagger->Branch("ssm_kine_pio_energy_2",&ssm_kine_tree.kine_pio_energy_2,"ssm_kine_pio_energy_2/F");
      T_tagger->Branch("ssm_kine_pio_theta_2",&ssm_kine_tree.kine_pio_theta_2,"ssm_kine_pio_theta_2/F");
      T_tagger->Branch("ssm_kine_pio_phi_2",&ssm_kine_tree.kine_pio_phi_2,"ssm_kine_pio_phi_2/F");
      T_tagger->Branch("ssm_kine_pio_dis_2",&ssm_kine_tree.kine_pio_dis_2,"ssm_kine_pio_dis_2/F");
      T_tagger->Branch("ssm_kine_pio_angle",&ssm_kine_tree.kine_pio_angle,"ssm_kine_pio_angle/F");

    //single photon shower
    T_tagger->Branch("shw_sp_flag",&tagger_info.shw_sp_flag,"shw_sp_flag/F");
    T_tagger->Branch("shw_sp_num_mip_tracks",&tagger_info.shw_sp_num_mip_tracks,"shw_sp_num_mip_tracks/F");
    T_tagger->Branch("shw_sp_num_muons",&tagger_info.shw_sp_num_muons,"shw_sp_num_muons/F");
    T_tagger->Branch("shw_sp_num_pions",&tagger_info.shw_sp_num_pions,"shw_sp_num_pions/F");
    T_tagger->Branch("shw_sp_num_protons",&tagger_info.shw_sp_num_protons,"shw_sp_num_protons/F");
    T_tagger->Branch("shw_sp_proton_length_1",&tagger_info.shw_sp_proton_length_1,"shw_sp_proton_length_1/F");
    T_tagger->Branch("shw_sp_proton_dqdx_1",&tagger_info.shw_sp_proton_dqdx_1,"shw_sp_proton_dqdx_1/F");
    T_tagger->Branch("shw_sp_proton_energy_1",&tagger_info.shw_sp_proton_energy_1,"shw_sp_proton_energy_1/F");
    T_tagger->Branch("shw_sp_proton_length_2",&tagger_info.shw_sp_proton_length_2,"shw_sp_proton_length_2/F");
    T_tagger->Branch("shw_sp_proton_dqdx_2",&tagger_info.shw_sp_proton_dqdx_2,"shw_sp_proton_dqdx_2/F");
    T_tagger->Branch("shw_sp_proton_energy_2",&tagger_info.shw_sp_proton_energy_2,"shw_sp_proton_energy_2/F");
    T_tagger->Branch("shw_sp_n_good_showers",&tagger_info.shw_sp_n_good_showers,"shw_sp_n_good_showers/F");
    T_tagger->Branch("shw_sp_n_20mev_showers",&tagger_info.shw_sp_n_20mev_showers,"shw_sp_n_20mev_showers/F");
    T_tagger->Branch("shw_sp_n_br1_showers",&tagger_info.shw_sp_n_br1_showers,"shw_sp_n_br1_showers/F");
    T_tagger->Branch("shw_sp_n_br2_showers",&tagger_info.shw_sp_n_br2_showers,"shw_sp_n_br2_showers/F");
    T_tagger->Branch("shw_sp_n_br3_showers",&tagger_info.shw_sp_n_br3_showers,"shw_sp_n_br3_showers/F");
    T_tagger->Branch("shw_sp_n_br4_showers",&tagger_info.shw_sp_n_br4_showers,"shw_sp_n_br4_showers/F");
    T_tagger->Branch("shw_sp_n_20br1_showers",&tagger_info.shw_sp_n_20br1_showers,"shw_sp_n_20br1_showers/F");
    T_tagger->Branch("shw_sp_20mev_showers",&tagger_info.shw_sp_20mev_showers);
    T_tagger->Branch("shw_sp_br1_showers",&tagger_info.shw_sp_br1_showers);
    T_tagger->Branch("shw_sp_br2_showers",&tagger_info.shw_sp_br2_showers);
    T_tagger->Branch("shw_sp_br3_showers",&tagger_info.shw_sp_br3_showers);
    T_tagger->Branch("shw_sp_br4_showers",&tagger_info.shw_sp_br4_showers);
    T_tagger->Branch("shw_sp_shw_vtx_dis",&tagger_info.shw_sp_shw_vtx_dis,"shw_sp_shw_vtx_dis/F");
    T_tagger->Branch("shw_sp_max_shw_dis",&tagger_info.shw_sp_max_shw_dis,"shw_sp_max_shw_dis/F");
    T_tagger->Branch("shw_sp_energy",&tagger_info.shw_sp_energy,"shw_sp_energy/F");

    T_tagger->Branch("shw_sp_vec_median_dedx",&tagger_info.shw_sp_vec_median_dedx,"shw_sp_vec_median_dedx/F");
    T_tagger->Branch("shw_sp_vec_mean_dedx",&tagger_info.shw_sp_vec_mean_dedx,"shw_sp_vec_mean_dedx/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_0",&tagger_info.shw_sp_vec_dQ_dx_0,"shw_sp_vec_dQ_dx_0/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_1",&tagger_info.shw_sp_vec_dQ_dx_1,"shw_sp_vec_dQ_dx_1/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_2",&tagger_info.shw_sp_vec_dQ_dx_2,"shw_sp_vec_dQ_dx_2/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_3",&tagger_info.shw_sp_vec_dQ_dx_3,"shw_sp_vec_dQ_dx_3/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_4",&tagger_info.shw_sp_vec_dQ_dx_4,"shw_sp_vec_dQ_dx_4/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_5",&tagger_info.shw_sp_vec_dQ_dx_5,"shw_sp_vec_dQ_dx_5/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_6",&tagger_info.shw_sp_vec_dQ_dx_6,"shw_sp_vec_dQ_dx_6/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_7",&tagger_info.shw_sp_vec_dQ_dx_7,"shw_sp_vec_dQ_dx_7/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_8",&tagger_info.shw_sp_vec_dQ_dx_8,"shw_sp_vec_dQ_dx_8/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_9",&tagger_info.shw_sp_vec_dQ_dx_9,"shw_sp_vec_dQ_dx_9/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_10",&tagger_info.shw_sp_vec_dQ_dx_10,"shw_sp_vec_dQ_dx_10/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_11",&tagger_info.shw_sp_vec_dQ_dx_11,"shw_sp_vec_dQ_dx_11/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_12",&tagger_info.shw_sp_vec_dQ_dx_12,"shw_sp_vec_dQ_dx_12/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_13",&tagger_info.shw_sp_vec_dQ_dx_13,"shw_sp_vec_dQ_dx_13/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_14",&tagger_info.shw_sp_vec_dQ_dx_14,"shw_sp_vec_dQ_dx_14/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_15",&tagger_info.shw_sp_vec_dQ_dx_15,"shw_sp_vec_dQ_dx_15/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_16",&tagger_info.shw_sp_vec_dQ_dx_16,"shw_sp_vec_dQ_dx_16/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_17",&tagger_info.shw_sp_vec_dQ_dx_17,"shw_sp_vec_dQ_dx_17/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_18",&tagger_info.shw_sp_vec_dQ_dx_18,"shw_sp_vec_dQ_dx_18/F");
    T_tagger->Branch("shw_sp_vec_dQ_dx_19",&tagger_info.shw_sp_vec_dQ_dx_19,"shw_sp_vec_dQ_dx_19/F");

    T_tagger->Branch("shw_sp_max_dQ_dx_sample",&tagger_info.shw_sp_max_dQ_dx_sample,"shw_sp_max_dQ_dx_sample/F");
    T_tagger->Branch("shw_sp_n_below_threshold",&tagger_info.shw_sp_n_below_threshold,"shw_sp_n_below_threshold/F");
    T_tagger->Branch("shw_sp_n_below_zero",&tagger_info.shw_sp_n_below_zero,"shw_sp_n_below_zero/F");
    T_tagger->Branch("shw_sp_n_lowest",&tagger_info.shw_sp_n_lowest,"shw_sp_n_lowest/F");
    T_tagger->Branch("shw_sp_n_highest",&tagger_info.shw_sp_n_highest,"shw_sp_n_highest/F");

    T_tagger->Branch("shw_sp_lowest_dQ_dx",&tagger_info.shw_sp_lowest_dQ_dx,"shw_sp_lowest_dQ_dx/F");
    T_tagger->Branch("shw_sp_highest_dQ_dx",&tagger_info.shw_sp_highest_dQ_dx,"shw_sp_highest_dQ_dx/F");
    T_tagger->Branch("shw_sp_medium_dQ_dx",&tagger_info.shw_sp_medium_dQ_dx,"shw_sp_medium_dQ_dx/F");
    T_tagger->Branch("shw_sp_stem_length",&tagger_info.shw_sp_stem_length,"shw_sp_stem_length/F");
    T_tagger->Branch("shw_sp_length_main",&tagger_info.shw_sp_length_main,"shw_sp_length_main/F");
    T_tagger->Branch("shw_sp_length_total",&tagger_info.shw_sp_length_total,"shw_sp_length_total/F");
    T_tagger->Branch("shw_sp_angle_beam",&tagger_info.shw_sp_angle_beam,"shw_sp_angle_beam/F");
    T_tagger->Branch("shw_sp_iso_angle",&tagger_info.shw_sp_iso_angle,"shw_sp_iso_angle/F");

    T_tagger->Branch("shw_sp_n_vertex",&tagger_info.shw_sp_n_vertex,"shw_sp_n_vertex/F");
    T_tagger->Branch("shw_sp_n_good_tracks",&tagger_info.shw_sp_n_good_tracks,"shw_sp_n_good_tracks/F");
    T_tagger->Branch("shw_sp_E_indirect_max_energy",&tagger_info.shw_sp_E_indirect_max_energy,"shw_sp_E_indirect_max_energy/F");
    T_tagger->Branch("shw_sp_flag_all_above",&tagger_info.shw_sp_flag_all_above,"shw_sp_flag_all_above/F");
    T_tagger->Branch("shw_sp_min_dQ_dx_5",&tagger_info.shw_sp_min_dQ_dx_5,"shw_sp_min_dQ_dx_5/F");
    T_tagger->Branch("shw_sp_n_other_vertex",&tagger_info.shw_sp_n_other_vertex,"shw_sp_n_other_vertex/F");
    T_tagger->Branch("shw_sp_n_stem_size",&tagger_info.shw_sp_n_stem_size,"shw_sp_n_stem_size/F");
    T_tagger->Branch("shw_sp_flag_stem_trajectory",&tagger_info.shw_sp_flag_stem_trajectory,"shw_sp_flag_stem_trajectory/F");
    T_tagger->Branch("shw_sp_min_dis",&tagger_info.shw_sp_min_dis,"shw_sp_min_dis/F");
    T_tagger->Branch("shw_sp_filled",&tagger_info.shw_sp_filled,"shw_sp_filled/F");
    // pio ...
    T_tagger->Branch("shw_sp_pio_flag",&tagger_info.shw_sp_pio_flag,"shw_sp_pio_flag/F");
    T_tagger->Branch("shw_sp_pio_mip_id",&tagger_info.shw_sp_pio_mip_id,"shw_sp_pio_mip_id/F");
    T_tagger->Branch("shw_sp_pio_filled",&tagger_info.shw_sp_pio_filled,"shw_sp_pio_filled/F");
    T_tagger->Branch("shw_sp_pio_flag_pio",&tagger_info.shw_sp_pio_flag_pio,"shw_sp_pio_flag_pio/F");

    T_tagger->Branch("shw_sp_pio_1_flag",&tagger_info.shw_sp_pio_1_flag,"shw_sp_pio_1_flag/F");
    T_tagger->Branch("shw_sp_pio_1_mass",&tagger_info.shw_sp_pio_1_mass,"shw_sp_pio_1_mass/F");
    T_tagger->Branch("shw_sp_pio_1_pio_type",&tagger_info.shw_sp_pio_1_pio_type,"shw_sp_pio_1_pio_type/F");
    T_tagger->Branch("shw_sp_pio_1_energy_1",&tagger_info.shw_sp_pio_1_energy_1,"shw_sp_pio_1_energy_1/F");
    T_tagger->Branch("shw_sp_pio_1_energy_2",&tagger_info.shw_sp_pio_1_energy_2,"shw_sp_pio_1_energy_2/F");
    T_tagger->Branch("shw_sp_pio_1_dis_1",&tagger_info.shw_sp_pio_1_dis_1,"shw_sp_pio_1_dis_1/F");
    T_tagger->Branch("shw_sp_pio_1_dis_2",&tagger_info.shw_sp_pio_1_dis_2,"shw_sp_pio_1_dis_2/F");

    T_tagger->Branch("shw_sp_pio_2_v_dis2",&tagger_info.shw_sp_pio_2_v_dis2);
    T_tagger->Branch("shw_sp_pio_2_v_angle2",&tagger_info.shw_sp_pio_2_v_angle2);
    T_tagger->Branch("shw_sp_pio_2_v_acc_length",&tagger_info.shw_sp_pio_2_v_acc_length);
    T_tagger->Branch("shw_sp_pio_2_v_flag",&tagger_info.shw_sp_pio_2_v_flag);

    T_tagger->Branch("shw_sp_br_filled",&tagger_info.shw_sp_br_filled,"shw_sp_br_filled/F");

    T_tagger->Branch("shw_sp_br1_flag",&tagger_info.shw_sp_br1_flag,"shw_sp_br1_flag/F");

    T_tagger->Branch("shw_sp_br1_1_flag",&tagger_info.shw_sp_br1_1_flag,"shw_sp_br1_1_flag/F");
    T_tagger->Branch("shw_sp_br1_1_shower_type",&tagger_info.shw_sp_br1_1_shower_type,"shw_sp_br1_1_shower_type/F");
    T_tagger->Branch("shw_sp_br1_1_vtx_n_segs",&tagger_info.shw_sp_br1_1_vtx_n_segs,"shw_sp_br1_1_vtx_n_segs/F");
    T_tagger->Branch("shw_sp_br1_1_energy",&tagger_info.shw_sp_br1_1_energy,"shw_sp_br1_1_energy/F");
    T_tagger->Branch("shw_sp_br1_1_n_segs",&tagger_info.shw_sp_br1_1_n_segs,"shw_sp_br1_1_n_segs/F");
    T_tagger->Branch("shw_sp_br1_1_flag_sg_topology",&tagger_info.shw_sp_br1_1_flag_sg_topology,"shw_sp_br1_1_flag_sg_topology/F");
    T_tagger->Branch("shw_sp_br1_1_flag_sg_trajectory",&tagger_info.shw_sp_br1_1_flag_sg_trajectory,"shw_sp_br1_1_flag_sg_trajectory/F");
    T_tagger->Branch("shw_sp_br1_1_sg_length",&tagger_info.shw_sp_br1_1_sg_length,"shw_sp_br1_1_sg_length/F");

    T_tagger->Branch("shw_sp_br1_2_flag",&tagger_info.shw_sp_br1_2_flag,"shw_sp_br1_2_flag/F");
    T_tagger->Branch("shw_sp_br1_2_energy",&tagger_info.shw_sp_br1_2_energy,"shw_sp_br1_2_energy/F");
    T_tagger->Branch("shw_sp_br1_2_n_connected",&tagger_info.shw_sp_br1_2_n_connected,"shw_sp_br1_2_n_connected/F");
    T_tagger->Branch("shw_sp_br1_2_max_length",&tagger_info.shw_sp_br1_2_max_length,"shw_sp_br1_2_max_length/F");
    T_tagger->Branch("shw_sp_br1_2_n_connected_1",&tagger_info.shw_sp_br1_2_n_connected_1,"shw_sp_br1_2_n_connected_1/F");
    T_tagger->Branch("shw_sp_br1_2_vtx_n_segs",&tagger_info.shw_sp_br1_2_vtx_n_segs,"shw_sp_br1_2_vtx_n_segs/F");
    T_tagger->Branch("shw_sp_br1_2_n_shower_segs",&tagger_info.shw_sp_br1_2_n_shower_segs,"shw_sp_br1_2_n_shower_segs/F");
    T_tagger->Branch("shw_sp_br1_2_max_length_ratio",&tagger_info.shw_sp_br1_2_max_length_ratio,"shw_sp_br1_2_max_length_ratio/F");
    T_tagger->Branch("shw_sp_br1_2_shower_length",&tagger_info.shw_sp_br1_2_shower_length,"shw_sp_br1_2_shower_length/F");

    T_tagger->Branch("shw_sp_br1_3_flag",&tagger_info.shw_sp_br1_3_flag,"shw_sp_br1_3_flag/F");
    T_tagger->Branch("shw_sp_br1_3_energy",&tagger_info.shw_sp_br1_3_energy,"shw_sp_br1_3_energy/F");
    T_tagger->Branch("shw_sp_br1_3_n_connected_p",&tagger_info.shw_sp_br1_3_n_connected_p,"shw_sp_br1_3_n_connected_p/F");
    T_tagger->Branch("shw_sp_br1_3_max_length_p",&tagger_info.shw_sp_br1_3_max_length_p,"shw_sp_br1_3_max_length_p/F");
    T_tagger->Branch("shw_sp_br1_3_n_shower_segs",&tagger_info.shw_sp_br1_3_n_shower_segs,"shw_sp_br1_3_n_shower_segs/F");
    T_tagger->Branch("shw_sp_br1_3_flag_sg_topology",&tagger_info.shw_sp_br1_3_flag_sg_topology,"shw_sp_br1_3_flag_sg_topology/F");
    T_tagger->Branch("shw_sp_br1_3_flag_sg_trajectory",&tagger_info.shw_sp_br1_3_flag_sg_trajectory,"shw_sp_br1_3_flag_sg_trajectory/F");
    T_tagger->Branch("shw_sp_br1_3_n_shower_main_segs",&tagger_info.shw_sp_br1_3_n_shower_main_segs,"shw_sp_br1_3_n_shower_main_segs/F");
    T_tagger->Branch("shw_sp_br1_3_sg_length",&tagger_info.shw_sp_br1_3_sg_length,"shw_sp_br1_3_sg_length/F");

    T_tagger->Branch("shw_sp_br2_flag",&tagger_info.shw_sp_br2_flag,"shw_sp_br2_flag/F");
    T_tagger->Branch("shw_sp_br2_flag_single_shower",&tagger_info.shw_sp_br2_flag_single_shower,"shw_sp_br2_flag_single_shower/F");
    T_tagger->Branch("shw_sp_br2_num_valid_tracks",&tagger_info.shw_sp_br2_num_valid_tracks,"shw_sp_br2_num_valid_tracks/F");
    T_tagger->Branch("shw_sp_br2_energy",&tagger_info.shw_sp_br2_energy,"shw_sp_br2_energy/F");
    T_tagger->Branch("shw_sp_br2_angle1",&tagger_info.shw_sp_br2_angle1,"shw_sp_br2_angle1/F");
    T_tagger->Branch("shw_sp_br2_angle2",&tagger_info.shw_sp_br2_angle2,"shw_sp_br2_angle2/F");
    T_tagger->Branch("shw_sp_br2_angle",&tagger_info.shw_sp_br2_angle,"shw_sp_br2_angle/F");
    T_tagger->Branch("shw_sp_br2_angle3",&tagger_info.shw_sp_br2_angle3,"shw_sp_br2_angle3/F");
    T_tagger->Branch("shw_sp_br2_n_shower_main_segs",&tagger_info.shw_sp_br2_n_shower_main_segs,"shw_sp_br2_n_shower_main_segs/F");
    T_tagger->Branch("shw_sp_br2_max_angle",&tagger_info.shw_sp_br2_max_angle,"shw_sp_br2_max_angle/F");
    T_tagger->Branch("shw_sp_br2_sg_length",&tagger_info.shw_sp_br2_sg_length,"shw_sp_br2_sg_length/F");
    T_tagger->Branch("shw_sp_br2_flag_sg_trajectory",&tagger_info.shw_sp_br2_flag_sg_trajectory,"shw_sp_br2_flag_sg_trajectory/F");


    T_tagger->Branch("shw_sp_lol_flag",&tagger_info.shw_sp_lol_flag,"shw_sp_lol_flag/F");

    T_tagger->Branch("shw_sp_lol_1_v_energy",&tagger_info.shw_sp_lol_1_v_energy);
    T_tagger->Branch("shw_sp_lol_1_v_vtx_n_segs",&tagger_info.shw_sp_lol_1_v_vtx_n_segs);
    T_tagger->Branch("shw_sp_lol_1_v_nseg",&tagger_info.shw_sp_lol_1_v_nseg);
    T_tagger->Branch("shw_sp_lol_1_v_angle",&tagger_info.shw_sp_lol_1_v_angle);
    T_tagger->Branch("shw_sp_lol_1_v_flag",&tagger_info.shw_sp_lol_1_v_flag);

    T_tagger->Branch("shw_sp_lol_2_v_length",&tagger_info.shw_sp_lol_2_v_length);
    T_tagger->Branch("shw_sp_lol_2_v_angle",&tagger_info.shw_sp_lol_2_v_angle);
    T_tagger->Branch("shw_sp_lol_2_v_type",&tagger_info.shw_sp_lol_2_v_type);
    T_tagger->Branch("shw_sp_lol_2_v_vtx_n_segs",&tagger_info.shw_sp_lol_2_v_vtx_n_segs);
    T_tagger->Branch("shw_sp_lol_2_v_energy",&tagger_info.shw_sp_lol_2_v_energy);
    T_tagger->Branch("shw_sp_lol_2_v_shower_main_length",&tagger_info.shw_sp_lol_2_v_shower_main_length);
    T_tagger->Branch("shw_sp_lol_2_v_flag_dir_weak",&tagger_info.shw_sp_lol_2_v_flag_dir_weak);
    T_tagger->Branch("shw_sp_lol_2_v_flag",&tagger_info.shw_sp_lol_2_v_flag);

    T_tagger->Branch("shw_sp_lol_3_angle_beam",&tagger_info.shw_sp_lol_3_angle_beam,"shw_sp_lol_3_angle_beam/F");
    T_tagger->Branch("shw_sp_lol_3_n_valid_tracks",&tagger_info.shw_sp_lol_3_n_valid_tracks,"shw_sp_lol_3_n_valid_tracks/F");
    T_tagger->Branch("shw_sp_lol_3_min_angle",&tagger_info.shw_sp_lol_3_min_angle,"shw_sp_lol_3_min_angle/F");
    T_tagger->Branch("shw_sp_lol_3_vtx_n_segs",&tagger_info.shw_sp_lol_3_vtx_n_segs,"shw_sp_lol_3_vtx_n_segs/F");
    T_tagger->Branch("shw_sp_lol_3_energy",&tagger_info.shw_sp_lol_3_energy,"shw_sp_lol_3_energy/F");
    T_tagger->Branch("shw_sp_lol_3_shower_main_length",&tagger_info.shw_sp_lol_3_shower_main_length,"shw_sp_lol_3_shower_main_length/F");
    T_tagger->Branch("shw_sp_lol_3_n_out",&tagger_info.shw_sp_lol_3_n_out,"shw_sp_lol_3_n_out/F");
    T_tagger->Branch("shw_sp_lol_3_n_sum",&tagger_info.shw_sp_lol_3_n_sum,"shw_sp_lol_3_n_sum/F");
    T_tagger->Branch("shw_sp_lol_3_flag",&tagger_info.shw_sp_lol_3_flag,"shw_sp_lol_3_flag/F");

    T_tagger->Branch("shw_sp_br3_1_energy",&tagger_info.shw_sp_br3_1_energy,"shw_sp_br3_1_energy/F");
    T_tagger->Branch("shw_sp_br3_1_n_shower_segments",&tagger_info.shw_sp_br3_1_n_shower_segments,"shw_sp_br3_1_n_shower_segments/F");
    T_tagger->Branch("shw_sp_br3_1_sg_flag_trajectory",&tagger_info.shw_sp_br3_1_sg_flag_trajectory,"shw_sp_br3_1_sg_flag_trajectory/F");
    T_tagger->Branch("shw_sp_br3_1_sg_direct_length",&tagger_info.shw_sp_br3_1_sg_direct_length,"shw_sp_br3_1_sg_direct_length/F");
    T_tagger->Branch("shw_sp_br3_1_sg_length",&tagger_info.shw_sp_br3_1_sg_length,"shw_sp_br3_1_sg_length/F");
    T_tagger->Branch("shw_sp_br3_1_total_main_length",&tagger_info.shw_sp_br3_1_total_main_length,"shw_sp_br3_1_total_main_length/F");
    T_tagger->Branch("shw_sp_br3_1_total_length",&tagger_info.shw_sp_br3_1_total_length,"shw_sp_br3_1_total_length/F");
    T_tagger->Branch("shw_sp_br3_1_iso_angle",&tagger_info.shw_sp_br3_1_iso_angle,"shw_sp_br3_1_iso_angle/F");
    T_tagger->Branch("shw_sp_br3_1_sg_flag_topology",&tagger_info.shw_sp_br3_1_sg_flag_topology,"shw_sp_br3_1_sg_flag_topology/F");
    T_tagger->Branch("shw_sp_br3_1_flag",&tagger_info.shw_sp_br3_1_flag,"shw_sp_br3_1_flag/F");

    T_tagger->Branch("shw_sp_br3_2_n_ele",&tagger_info.shw_sp_br3_2_n_ele,"shw_sp_br3_2_n_ele/F");
    T_tagger->Branch("shw_sp_br3_2_n_other",&tagger_info.shw_sp_br3_2_n_other,"shw_sp_br3_2_n_other/F");
    T_tagger->Branch("shw_sp_br3_2_energy",&tagger_info.shw_sp_br3_2_energy,"shw_sp_br3_2_energy/F");
    T_tagger->Branch("shw_sp_br3_2_total_main_length",&tagger_info.shw_sp_br3_2_total_main_length,"shw_sp_br3_2_total_main_length/F");
    T_tagger->Branch("shw_sp_br3_2_total_length",&tagger_info.shw_sp_br3_2_total_length,"shw_sp_br3_2_total_length/F");
    T_tagger->Branch("shw_sp_br3_2_other_fid",&tagger_info.shw_sp_br3_2_other_fid,"shw_sp_br3_2_other_fid/F");
    T_tagger->Branch("shw_sp_br3_2_flag",&tagger_info.shw_sp_br3_2_flag,"shw_sp_br3_2_flag/F");

    T_tagger->Branch("shw_sp_br3_3_v_energy",&tagger_info.shw_sp_br3_3_v_energy);
    T_tagger->Branch("shw_sp_br3_3_v_angle",&tagger_info.shw_sp_br3_3_v_angle);
    T_tagger->Branch("shw_sp_br3_3_v_dir_length",&tagger_info.shw_sp_br3_3_v_dir_length);
    T_tagger->Branch("shw_sp_br3_3_v_length",&tagger_info.shw_sp_br3_3_v_length);
    T_tagger->Branch("shw_sp_br3_3_v_flag",&tagger_info.shw_sp_br3_3_v_flag);

    T_tagger->Branch("shw_sp_br3_4_acc_length", &tagger_info.shw_sp_br3_4_acc_length, "shw_sp_br3_4_acc_length/F");
    T_tagger->Branch("shw_sp_br3_4_total_length", &tagger_info.shw_sp_br3_4_total_length, "shw_sp_br3_4_total_length/F");
    T_tagger->Branch("shw_sp_br3_4_energy", &tagger_info.shw_sp_br3_4_energy, "shw_sp_br3_4_energy/F");
    T_tagger->Branch("shw_sp_br3_4_flag", &tagger_info.shw_sp_br3_4_flag, "shw_sp_br3_4_flag/F");

    T_tagger->Branch("shw_sp_br3_5_v_dir_length", &tagger_info.shw_sp_br3_5_v_dir_length);
    T_tagger->Branch("shw_sp_br3_5_v_total_length", &tagger_info.shw_sp_br3_5_v_total_length);
    T_tagger->Branch("shw_sp_br3_5_v_flag_avoid_muon_check", &tagger_info.shw_sp_br3_5_v_flag_avoid_muon_check);
    T_tagger->Branch("shw_sp_br3_5_v_n_seg", &tagger_info.shw_sp_br3_5_v_n_seg);
    T_tagger->Branch("shw_sp_br3_5_v_angle", &tagger_info.shw_sp_br3_5_v_angle);
    T_tagger->Branch("shw_sp_br3_5_v_sg_length", &tagger_info.shw_sp_br3_5_v_sg_length);
    T_tagger->Branch("shw_sp_br3_5_v_energy", &tagger_info.shw_sp_br3_5_v_energy);
    T_tagger->Branch("shw_sp_br3_5_v_n_main_segs", &tagger_info.shw_sp_br3_5_v_n_main_segs);
    T_tagger->Branch("shw_sp_br3_5_v_n_segs", &tagger_info.shw_sp_br3_5_v_n_segs);
    T_tagger->Branch("shw_sp_br3_5_v_shower_main_length", &tagger_info.shw_sp_br3_5_v_shower_main_length);
    T_tagger->Branch("shw_sp_br3_5_v_shower_total_length", &tagger_info.shw_sp_br3_5_v_shower_total_length);
    T_tagger->Branch("shw_sp_br3_5_v_flag", &tagger_info.shw_sp_br3_5_v_flag);

    T_tagger->Branch("shw_sp_br3_6_v_angle",&tagger_info.shw_sp_br3_6_v_angle);
    T_tagger->Branch("shw_sp_br3_6_v_angle1",&tagger_info.shw_sp_br3_6_v_angle1);
    T_tagger->Branch("shw_sp_br3_6_v_flag_shower_trajectory",&tagger_info.shw_sp_br3_6_v_flag_shower_trajectory);
    T_tagger->Branch("shw_sp_br3_6_v_direct_length",&tagger_info.shw_sp_br3_6_v_direct_length);
    T_tagger->Branch("shw_sp_br3_6_v_length",&tagger_info.shw_sp_br3_6_v_length);
    T_tagger->Branch("shw_sp_br3_6_v_n_other_vtx_segs",&tagger_info.shw_sp_br3_6_v_n_other_vtx_segs);
    T_tagger->Branch("shw_sp_br3_6_v_energy",&tagger_info.shw_sp_br3_6_v_energy);
    T_tagger->Branch("shw_sp_br3_6_v_flag",&tagger_info.shw_sp_br3_6_v_flag);

    T_tagger->Branch("shw_sp_br3_7_energy",&tagger_info.shw_sp_br3_7_energy,"shw_sp_br3_7_energy/F");
    T_tagger->Branch("shw_sp_br3_7_min_angle",&tagger_info.shw_sp_br3_7_min_angle,"shw_sp_br3_7_min_angle/F");
    T_tagger->Branch("shw_sp_br3_7_sg_length",&tagger_info.shw_sp_br3_7_sg_length,"shw_sp_br3_7_sg_length/F");
    T_tagger->Branch("shw_sp_br3_7_main_length",&tagger_info.shw_sp_br3_7_shower_main_length,"shw_sp_br3_7_shower_main_length/F");
    T_tagger->Branch("shw_sp_br3_7_flag",&tagger_info.shw_sp_br3_7_flag,"shw_sp_br3_7_flag/F");

    T_tagger->Branch("shw_sp_br3_8_max_dQ_dx",&tagger_info.shw_sp_br3_8_max_dQ_dx,"shw_sp_br3_8_max_dQ_dx/F");
    T_tagger->Branch("shw_sp_br3_8_energy",&tagger_info.shw_sp_br3_8_energy,"shw_sp_br3_8_energy/F");
    T_tagger->Branch("shw_sp_br3_8_n_main_segs",&tagger_info.shw_sp_br3_8_n_main_segs,"shw_sp_br3_8_n_main_segs/F");
    T_tagger->Branch("shw_sp_br3_8_shower_main_length",&tagger_info.shw_sp_br3_8_shower_main_length,"shw_sp_br3_8_shower_main_length/F");
    T_tagger->Branch("shw_sp_br3_8_shower_length",&tagger_info.shw_sp_br3_8_shower_length,"shw_sp_br3_8_shower_length/F");
    T_tagger->Branch("shw_sp_br3_8_flag",&tagger_info.shw_sp_br3_8_flag,"shw_sp_br3_8_flag/F");

    T_tagger->Branch("shw_sp_br3_flag",&tagger_info.shw_sp_br3_flag,"shw_sp_br3_flag/F");


    T_tagger->Branch("shw_sp_br4_1_shower_main_length", &tagger_info.shw_sp_br4_1_shower_main_length, "shw_sp_br4_1_shower_main_length/F");
    T_tagger->Branch("shw_sp_br4_1_shower_total_length", &tagger_info.shw_sp_br4_1_shower_total_length, "shw_sp_br4_1_shower_total_length/F");
    T_tagger->Branch("shw_sp_br4_1_min_dis", &tagger_info.shw_sp_br4_1_min_dis, "shw_sp_br4_1_min_dis/F");
    T_tagger->Branch("shw_sp_br4_1_energy", &tagger_info.shw_sp_br4_1_energy, "shw_sp_br4_1_energy/F");
    T_tagger->Branch("shw_sp_br4_1_flag_avoid_muon_check", &tagger_info.shw_sp_br4_1_flag_avoid_muon_check, "shw_sp_br4_1_flag_avoid_muon_check/F");
    T_tagger->Branch("shw_sp_br4_1_n_vtx_segs", &tagger_info.shw_sp_br4_1_n_vtx_segs, "shw_sp_br4_1_n_vtx_segs/F");
    T_tagger->Branch("shw_sp_br4_1_n_main_segs", &tagger_info.shw_sp_br4_1_n_main_segs, "shw_sp_br4_1_n_main_segs/F");
    T_tagger->Branch("shw_sp_br4_1_flag", &tagger_info.shw_sp_br4_1_flag, "shw_sp_br4_1_flag/F");

    T_tagger->Branch("shw_sp_br4_2_ratio_45", &tagger_info.shw_sp_br4_2_ratio_45, "shw_sp_br4_2_ratio_45/F");
    T_tagger->Branch("shw_sp_br4_2_ratio_35", &tagger_info.shw_sp_br4_2_ratio_35, "shw_sp_br4_2_ratio_35/F");
    T_tagger->Branch("shw_sp_br4_2_ratio_25", &tagger_info.shw_sp_br4_2_ratio_25, "shw_sp_br4_2_ratio_25/F");
    T_tagger->Branch("shw_sp_br4_2_ratio_15", &tagger_info.shw_sp_br4_2_ratio_15, "shw_sp_br4_2_ratio_15/F");
    T_tagger->Branch("shw_sp_br4_2_energy",   &tagger_info.shw_sp_br4_2_energy, "shw_sp_br4_2_energy/F");
    T_tagger->Branch("shw_sp_br4_2_ratio1_45", &tagger_info.shw_sp_br4_2_ratio1_45, "shw_sp_br4_2_ratio1_45/F");
    T_tagger->Branch("shw_sp_br4_2_ratio1_35", &tagger_info.shw_sp_br4_2_ratio1_35, "shw_sp_br4_2_ratio1_35/F");
    T_tagger->Branch("shw_sp_br4_2_ratio1_25", &tagger_info.shw_sp_br4_2_ratio1_25, "shw_sp_br4_2_ratio1_25/F");
    T_tagger->Branch("shw_sp_br4_2_ratio1_15", &tagger_info.shw_sp_br4_2_ratio1_15, "shw_sp_br4_2_ratio1_15/F");
    T_tagger->Branch("shw_sp_br4_2_iso_angle", &tagger_info.shw_sp_br4_2_iso_angle, "shw_sp_br4_2_iso_angle/F");
    T_tagger->Branch("shw_sp_br4_2_iso_angle1", &tagger_info.shw_sp_br4_2_iso_angle1, "shw_sp_br4_2_iso_angle1/F");
    T_tagger->Branch("shw_sp_br4_2_angle", &tagger_info.shw_sp_br4_2_angle, "shw_sp_br4_2_angle/F");
    T_tagger->Branch("shw_sp_br4_2_flag", &tagger_info.shw_sp_br4_2_flag, "shw_sp_br4_2_flag/F");

    T_tagger->Branch("shw_sp_br4_flag", &tagger_info.shw_sp_br4_flag, "shw_sp_br4_flag/F");


    T_tagger->Branch("shw_sp_hol_1_n_valid_tracks", &tagger_info.shw_sp_hol_1_n_valid_tracks,"shw_sp_hol_1_n_valid_tracks/F");
    T_tagger->Branch("shw_sp_hol_1_min_angle", &tagger_info.shw_sp_hol_1_min_angle,"shw_sp_hol_1_min_angle/F");
    T_tagger->Branch("shw_sp_hol_1_energy", &tagger_info.shw_sp_hol_1_energy,"shw_sp_hol_1_energy/F");
    T_tagger->Branch("shw_sp_hol_1_flag_all_shower", &tagger_info.shw_sp_hol_1_flag_all_shower,"shw_sp_hol_1_flag_all_shower/F");
    T_tagger->Branch("shw_sp_hol_1_min_length", &tagger_info.shw_sp_hol_1_min_length,"shw_sp_hol_1_min_length/F");
    T_tagger->Branch("shw_sp_hol_1_flag", &tagger_info.shw_sp_hol_1_flag,"shw_sp_hol_1_flag/F");

    T_tagger->Branch("shw_sp_hol_2_min_angle", &tagger_info.shw_sp_hol_2_min_angle,"shw_sp_hol_2_min_angle/F");
    T_tagger->Branch("shw_sp_hol_2_medium_dQ_dx", &tagger_info.shw_sp_hol_2_medium_dQ_dx,"shw_sp_hol_2_medium_dQ_dx/F");
    T_tagger->Branch("shw_sp_hol_2_ncount", &tagger_info.shw_sp_hol_2_ncount,"shw_sp_hol_2_ncount/F");
    T_tagger->Branch("shw_sp_hol_2_energy", &tagger_info.shw_sp_hol_2_energy,"shw_sp_hol_2_energy/F");
    T_tagger->Branch("shw_sp_hol_2_flag", &tagger_info.shw_sp_hol_2_flag,"shw_sp_hol_2_flag/F");

    T_tagger->Branch("shw_sp_hol_flag", &tagger_info.shw_sp_hol_flag,"shw_sp_hol_flag/F");

    T_tagger->Branch("shw_sp_lem_shower_total_length",&tagger_info.shw_sp_lem_shower_total_length,"shw_sp_lem_shower_total_length/F");
    T_tagger->Branch("shw_sp_lem_shower_main_length",&tagger_info.shw_sp_lem_shower_main_length,"shw_sp_lem_shower_main_length/F");
    T_tagger->Branch("shw_sp_lem_n_3seg",&tagger_info.shw_sp_lem_n_3seg,"shw_sp_lem_n_3seg/F");
    T_tagger->Branch("shw_sp_lem_e_charge",&tagger_info.shw_sp_lem_e_charge,"shw_sp_lem_e_charge/F");
    T_tagger->Branch("shw_sp_lem_e_dQdx",&tagger_info.shw_sp_lem_e_dQdx,"shw_sp_lem_e_dQdx/F");
    T_tagger->Branch("shw_sp_lem_shower_num_segs",&tagger_info.shw_sp_lem_shower_num_segs,"shw_sp_lem_shower_num_segs/F");
    T_tagger->Branch("shw_sp_lem_shower_num_main_segs",&tagger_info.shw_sp_lem_shower_num_main_segs,"shw_sp_lem_shower_num_main_segs/F");
    T_tagger->Branch("shw_sp_lem_flag",&tagger_info.shw_sp_lem_flag,"shw_sp_lem_flag/F");


    // pio ...
    T_tagger->Branch("pio_flag",&tagger_info.pio_flag,"pio_flag/F");
    T_tagger->Branch("pio_mip_id",&tagger_info.pio_mip_id,"pio_mip_id/F");
    T_tagger->Branch("pio_filled",&tagger_info.pio_filled,"pio_filled/F");
    T_tagger->Branch("pio_flag_pio",&tagger_info.pio_flag_pio,"pio_flag_pio/F");

    T_tagger->Branch("pio_1_flag",&tagger_info.pio_1_flag,"pio_1_flag/F");
    T_tagger->Branch("pio_1_mass",&tagger_info.pio_1_mass,"pio_1_mass/F");
    T_tagger->Branch("pio_1_pio_type",&tagger_info.pio_1_pio_type,"pio_1_pio_type/F");
    T_tagger->Branch("pio_1_energy_1",&tagger_info.pio_1_energy_1,"pio_1_energy_1/F");
    T_tagger->Branch("pio_1_energy_2",&tagger_info.pio_1_energy_2,"pio_1_energy_2/F");
    T_tagger->Branch("pio_1_dis_1",&tagger_info.pio_1_dis_1,"pio_1_dis_1/F");
    T_tagger->Branch("pio_1_dis_2",&tagger_info.pio_1_dis_2,"pio_1_dis_2/F");

    T_tagger->Branch("pio_2_v_dis2",&tagger_info.pio_2_v_dis2);
    T_tagger->Branch("pio_2_v_angle2",&tagger_info.pio_2_v_angle2);
    T_tagger->Branch("pio_2_v_acc_length",&tagger_info.pio_2_v_acc_length);
    T_tagger->Branch("pio_2_v_flag",&tagger_info.pio_2_v_flag);



    // bad reconstruction ...
    T_tagger->Branch("stem_dir_flag",&tagger_info.stem_dir_flag,"stem_dir_flag/F");
    T_tagger->Branch("stem_dir_flag_single_shower",&tagger_info.stem_dir_flag_single_shower,"stem_dir_flag_single_shower/F");
    T_tagger->Branch("stem_dir_filled",&tagger_info.stem_dir_filled,"stem_dir_filled/F");
    T_tagger->Branch("stem_dir_angle",&tagger_info.stem_dir_angle,"stem_dir_angle/F");
    T_tagger->Branch("stem_dir_energy",&tagger_info.stem_dir_energy,"stem_dir_energy/F");
    T_tagger->Branch("stem_dir_angle1",&tagger_info.stem_dir_angle1,"stem_dir_angle1/F");
    T_tagger->Branch("stem_dir_angle2",&tagger_info.stem_dir_angle2,"stem_dir_angle2/F");
    T_tagger->Branch("stem_dir_angle3",&tagger_info.stem_dir_angle3,"stem_dir_angle3/F");
    T_tagger->Branch("stem_dir_ratio",&tagger_info.stem_dir_ratio,"stem_dir_ratio/F");

    T_tagger->Branch("br_filled",&tagger_info.br_filled,"br_filled/F");

    T_tagger->Branch("br1_flag",&tagger_info.br1_flag,"br1_flag/F");

    T_tagger->Branch("br1_1_flag",&tagger_info.br1_1_flag,"br1_1_flag/F");
    T_tagger->Branch("br1_1_shower_type",&tagger_info.br1_1_shower_type,"br1_1_shower_type/F");
    T_tagger->Branch("br1_1_vtx_n_segs",&tagger_info.br1_1_vtx_n_segs,"br1_1_vtx_n_segs/F");
    T_tagger->Branch("br1_1_energy",&tagger_info.br1_1_energy,"br1_1_energy/F");
    T_tagger->Branch("br1_1_n_segs",&tagger_info.br1_1_n_segs,"br1_1_n_segs/F");
    T_tagger->Branch("br1_1_flag_sg_topology",&tagger_info.br1_1_flag_sg_topology,"br1_1_flag_sg_topology/F");
    T_tagger->Branch("br1_1_flag_sg_trajectory",&tagger_info.br1_1_flag_sg_trajectory,"br1_1_flag_sg_trajectory/F");
    T_tagger->Branch("br1_1_sg_length",&tagger_info.br1_1_sg_length,"br1_1_sg_length/F");

    T_tagger->Branch("br1_2_flag",&tagger_info.br1_2_flag,"br1_2_flag/F");
    T_tagger->Branch("br1_2_energy",&tagger_info.br1_2_energy,"br1_2_energy/F");
    T_tagger->Branch("br1_2_n_connected",&tagger_info.br1_2_n_connected,"br1_2_n_connected/F");
    T_tagger->Branch("br1_2_max_length",&tagger_info.br1_2_max_length,"br1_2_max_length/F");
    T_tagger->Branch("br1_2_n_connected_1",&tagger_info.br1_2_n_connected_1,"br1_2_n_connected_1/F");
    T_tagger->Branch("br1_2_vtx_n_segs",&tagger_info.br1_2_vtx_n_segs,"br1_2_vtx_n_segs/F");
    T_tagger->Branch("br1_2_n_shower_segs",&tagger_info.br1_2_n_shower_segs,"br1_2_n_shower_segs/F");
    T_tagger->Branch("br1_2_max_length_ratio",&tagger_info.br1_2_max_length_ratio,"br1_2_max_length_ratio/F");
    T_tagger->Branch("br1_2_shower_length",&tagger_info.br1_2_shower_length,"br1_2_shower_length/F");

    T_tagger->Branch("br1_3_flag",&tagger_info.br1_3_flag,"br1_3_flag/F");
    T_tagger->Branch("br1_3_energy",&tagger_info.br1_3_energy,"br1_3_energy/F");
    T_tagger->Branch("br1_3_n_connected_p",&tagger_info.br1_3_n_connected_p,"br1_3_n_connected_p/F");
    T_tagger->Branch("br1_3_max_length_p",&tagger_info.br1_3_max_length_p,"br1_3_max_length_p/F");
    T_tagger->Branch("br1_3_n_shower_segs",&tagger_info.br1_3_n_shower_segs,"br1_3_n_shower_segs/F");
    T_tagger->Branch("br1_3_flag_sg_topology",&tagger_info.br1_3_flag_sg_topology,"br1_3_flag_sg_topology/F");
    T_tagger->Branch("br1_3_flag_sg_trajectory",&tagger_info.br1_3_flag_sg_trajectory,"br1_3_flag_sg_trajectory/F");
    T_tagger->Branch("br1_3_n_shower_main_segs",&tagger_info.br1_3_n_shower_main_segs,"br1_3_n_shower_main_segs/F");
    T_tagger->Branch("br1_3_sg_length",&tagger_info.br1_3_sg_length,"br1_3_sg_length/F");

    T_tagger->Branch("br2_flag",&tagger_info.br2_flag,"br2_flag/F");
    T_tagger->Branch("br2_flag_single_shower",&tagger_info.br2_flag_single_shower,"br2_flag_single_shower/F");
    T_tagger->Branch("br2_num_valid_tracks",&tagger_info.br2_num_valid_tracks,"br2_num_valid_tracks/F");
    T_tagger->Branch("br2_energy",&tagger_info.br2_energy,"br2_energy/F");
    T_tagger->Branch("br2_angle1",&tagger_info.br2_angle1,"br2_angle1/F");
    T_tagger->Branch("br2_angle2",&tagger_info.br2_angle2,"br2_angle2/F");
    T_tagger->Branch("br2_angle",&tagger_info.br2_angle,"br2_angle/F");
    T_tagger->Branch("br2_angle3",&tagger_info.br2_angle3,"br2_angle3/F");
    T_tagger->Branch("br2_n_shower_main_segs",&tagger_info.br2_n_shower_main_segs,"br2_n_shower_main_segs/F");
    T_tagger->Branch("br2_max_angle",&tagger_info.br2_max_angle,"br2_max_angle/F");
    T_tagger->Branch("br2_sg_length",&tagger_info.br2_sg_length,"br2_sg_length/F");
    T_tagger->Branch("br2_flag_sg_trajectory",&tagger_info.br2_flag_sg_trajectory,"br2_flag_sg_trajectory/F");


    T_tagger->Branch("lol_flag",&tagger_info.lol_flag,"lol_flag/F");

    T_tagger->Branch("lol_1_v_energy",&tagger_info.lol_1_v_energy);
    T_tagger->Branch("lol_1_v_vtx_n_segs",&tagger_info.lol_1_v_vtx_n_segs);
    T_tagger->Branch("lol_1_v_nseg",&tagger_info.lol_1_v_nseg);
    T_tagger->Branch("lol_1_v_angle",&tagger_info.lol_1_v_angle);
    T_tagger->Branch("lol_1_v_flag",&tagger_info.lol_1_v_flag);

    T_tagger->Branch("lol_2_v_length",&tagger_info.lol_2_v_length);
    T_tagger->Branch("lol_2_v_angle",&tagger_info.lol_2_v_angle);
    T_tagger->Branch("lol_2_v_type",&tagger_info.lol_2_v_type);
    T_tagger->Branch("lol_2_v_vtx_n_segs",&tagger_info.lol_2_v_vtx_n_segs);
    T_tagger->Branch("lol_2_v_energy",&tagger_info.lol_2_v_energy);
    T_tagger->Branch("lol_2_v_shower_main_length",&tagger_info.lol_2_v_shower_main_length);
    T_tagger->Branch("lol_2_v_flag_dir_weak",&tagger_info.lol_2_v_flag_dir_weak);
    T_tagger->Branch("lol_2_v_flag",&tagger_info.lol_2_v_flag);

    T_tagger->Branch("lol_3_angle_beam",&tagger_info.lol_3_angle_beam,"lol_3_angle_beam/F");
    T_tagger->Branch("lol_3_n_valid_tracks",&tagger_info.lol_3_n_valid_tracks,"lol_3_n_valid_tracks/F");
    T_tagger->Branch("lol_3_min_angle",&tagger_info.lol_3_min_angle,"lol_3_min_angle/F");
    T_tagger->Branch("lol_3_vtx_n_segs",&tagger_info.lol_3_vtx_n_segs,"lol_3_vtx_n_segs/F");
    T_tagger->Branch("lol_3_energy",&tagger_info.lol_3_energy,"lol_3_energy/F");
    T_tagger->Branch("lol_3_shower_main_length",&tagger_info.lol_3_shower_main_length,"lol_3_shower_main_length/F");
    T_tagger->Branch("lol_3_n_out",&tagger_info.lol_3_n_out,"lol_3_n_out/F");
    T_tagger->Branch("lol_3_n_sum",&tagger_info.lol_3_n_sum,"lol_3_n_sum/F");
    T_tagger->Branch("lol_3_flag",&tagger_info.lol_3_flag,"lol_3_flag/F");

    T_tagger->Branch("br3_1_energy",&tagger_info.br3_1_energy,"br3_1_energy/F");
    T_tagger->Branch("br3_1_n_shower_segments",&tagger_info.br3_1_n_shower_segments,"br3_1_n_shower_segments/F");
    T_tagger->Branch("br3_1_sg_flag_trajectory",&tagger_info.br3_1_sg_flag_trajectory,"br3_1_sg_flag_trajectory/F");
    T_tagger->Branch("br3_1_sg_direct_length",&tagger_info.br3_1_sg_direct_length,"br3_1_sg_direct_length/F");
    T_tagger->Branch("br3_1_sg_length",&tagger_info.br3_1_sg_length,"br3_1_sg_length/F");
    T_tagger->Branch("br3_1_total_main_length",&tagger_info.br3_1_total_main_length,"br3_1_total_main_length/F");
    T_tagger->Branch("br3_1_total_length",&tagger_info.br3_1_total_length,"br3_1_total_length/F");
    T_tagger->Branch("br3_1_iso_angle",&tagger_info.br3_1_iso_angle,"br3_1_iso_angle/F");
    T_tagger->Branch("br3_1_sg_flag_topology",&tagger_info.br3_1_sg_flag_topology,"br3_1_sg_flag_topology/F");
    T_tagger->Branch("br3_1_flag",&tagger_info.br3_1_flag,"br3_1_flag/F");

    T_tagger->Branch("br3_2_n_ele",&tagger_info.br3_2_n_ele,"br3_2_n_ele/F");
    T_tagger->Branch("br3_2_n_other",&tagger_info.br3_2_n_other,"br3_2_n_other/F");
    T_tagger->Branch("br3_2_energy",&tagger_info.br3_2_energy,"br3_2_energy/F");
    T_tagger->Branch("br3_2_total_main_length",&tagger_info.br3_2_total_main_length,"br3_2_total_main_length/F");
    T_tagger->Branch("br3_2_total_length",&tagger_info.br3_2_total_length,"br3_2_total_length/F");
    T_tagger->Branch("br3_2_other_fid",&tagger_info.br3_2_other_fid,"br3_2_other_fid/F");
    T_tagger->Branch("br3_2_flag",&tagger_info.br3_2_flag,"br3_2_flag/F");

    T_tagger->Branch("br3_3_v_energy",&tagger_info.br3_3_v_energy);
    T_tagger->Branch("br3_3_v_angle",&tagger_info.br3_3_v_angle);
    T_tagger->Branch("br3_3_v_dir_length",&tagger_info.br3_3_v_dir_length);
    T_tagger->Branch("br3_3_v_length",&tagger_info.br3_3_v_length);
    T_tagger->Branch("br3_3_v_flag",&tagger_info.br3_3_v_flag);

    T_tagger->Branch("br3_4_acc_length", &tagger_info.br3_4_acc_length, "br3_4_acc_length/F");
    T_tagger->Branch("br3_4_total_length", &tagger_info.br3_4_total_length, "br3_4_total_length/F");
    T_tagger->Branch("br3_4_energy", &tagger_info.br3_4_energy, "br3_4_energy/F");
    T_tagger->Branch("br3_4_flag", &tagger_info.br3_4_flag, "br3_4_flag/F");

    T_tagger->Branch("br3_5_v_dir_length", &tagger_info.br3_5_v_dir_length);
    T_tagger->Branch("br3_5_v_total_length", &tagger_info.br3_5_v_total_length);
    T_tagger->Branch("br3_5_v_flag_avoid_muon_check", &tagger_info.br3_5_v_flag_avoid_muon_check);
    T_tagger->Branch("br3_5_v_n_seg", &tagger_info.br3_5_v_n_seg);
    T_tagger->Branch("br3_5_v_angle", &tagger_info.br3_5_v_angle);
    T_tagger->Branch("br3_5_v_sg_length", &tagger_info.br3_5_v_sg_length);
    T_tagger->Branch("br3_5_v_energy", &tagger_info.br3_5_v_energy);
    T_tagger->Branch("br3_5_v_n_main_segs", &tagger_info.br3_5_v_n_main_segs);
    T_tagger->Branch("br3_5_v_n_segs", &tagger_info.br3_5_v_n_segs);
    T_tagger->Branch("br3_5_v_shower_main_length", &tagger_info.br3_5_v_shower_main_length);
    T_tagger->Branch("br3_5_v_shower_total_length", &tagger_info.br3_5_v_shower_total_length);
    T_tagger->Branch("br3_5_v_flag", &tagger_info.br3_5_v_flag);

    T_tagger->Branch("br3_6_v_angle",&tagger_info.br3_6_v_angle);
    T_tagger->Branch("br3_6_v_angle1",&tagger_info.br3_6_v_angle1);
    T_tagger->Branch("br3_6_v_flag_shower_trajectory",&tagger_info.br3_6_v_flag_shower_trajectory);
    T_tagger->Branch("br3_6_v_direct_length",&tagger_info.br3_6_v_direct_length);
    T_tagger->Branch("br3_6_v_length",&tagger_info.br3_6_v_length);
    T_tagger->Branch("br3_6_v_n_other_vtx_segs",&tagger_info.br3_6_v_n_other_vtx_segs);
    T_tagger->Branch("br3_6_v_energy",&tagger_info.br3_6_v_energy);
    T_tagger->Branch("br3_6_v_flag",&tagger_info.br3_6_v_flag);

    T_tagger->Branch("br3_7_energy",&tagger_info.br3_7_energy,"br3_7_energy/F");
    T_tagger->Branch("br3_7_min_angle",&tagger_info.br3_7_min_angle,"br3_7_min_angle/F");
    T_tagger->Branch("br3_7_sg_length",&tagger_info.br3_7_sg_length,"br3_7_sg_length/F");
    T_tagger->Branch("br3_7_main_length",&tagger_info.br3_7_shower_main_length,"br3_7_shower_main_length/F");
    T_tagger->Branch("br3_7_flag",&tagger_info.br3_7_flag,"br3_7_flag/F");

    T_tagger->Branch("br3_8_max_dQ_dx",&tagger_info.br3_8_max_dQ_dx,"br3_8_max_dQ_dx/F");
    T_tagger->Branch("br3_8_energy",&tagger_info.br3_8_energy,"br3_8_energy/F");
    T_tagger->Branch("br3_8_n_main_segs",&tagger_info.br3_8_n_main_segs,"br3_8_n_main_segs/F");
    T_tagger->Branch("br3_8_shower_main_length",&tagger_info.br3_8_shower_main_length,"br3_8_shower_main_length/F");
    T_tagger->Branch("br3_8_shower_length",&tagger_info.br3_8_shower_length,"br3_8_shower_length/F");
    T_tagger->Branch("br3_8_flag",&tagger_info.br3_8_flag,"br3_8_flag/F");

    T_tagger->Branch("br3_flag",&tagger_info.br3_flag,"br3_flag/F");


    T_tagger->Branch("br4_1_shower_main_length", &tagger_info.br4_1_shower_main_length, "br4_1_shower_main_length/F");
    T_tagger->Branch("br4_1_shower_total_length", &tagger_info.br4_1_shower_total_length, "br4_1_shower_total_length/F");
    T_tagger->Branch("br4_1_min_dis", &tagger_info.br4_1_min_dis, "br4_1_min_dis/F");
    T_tagger->Branch("br4_1_energy", &tagger_info.br4_1_energy, "br4_1_energy/F");
    T_tagger->Branch("br4_1_flag_avoid_muon_check", &tagger_info.br4_1_flag_avoid_muon_check, "br4_1_flag_avoid_muon_check/F");
    T_tagger->Branch("br4_1_n_vtx_segs", &tagger_info.br4_1_n_vtx_segs, "br4_1_n_vtx_segs/F");
    T_tagger->Branch("br4_1_n_main_segs", &tagger_info.br4_1_n_main_segs, "br4_1_n_main_segs/F");
    T_tagger->Branch("br4_1_flag", &tagger_info.br4_1_flag, "br4_1_flag/F");

    T_tagger->Branch("br4_2_ratio_45", &tagger_info.br4_2_ratio_45, "br4_2_ratio_45/F");
    T_tagger->Branch("br4_2_ratio_35", &tagger_info.br4_2_ratio_35, "br4_2_ratio_35/F");
    T_tagger->Branch("br4_2_ratio_25", &tagger_info.br4_2_ratio_25, "br4_2_ratio_25/F");
    T_tagger->Branch("br4_2_ratio_15", &tagger_info.br4_2_ratio_15, "br4_2_ratio_15/F");
    T_tagger->Branch("br4_2_energy",   &tagger_info.br4_2_energy, "br4_2_energy/F");
    T_tagger->Branch("br4_2_ratio1_45", &tagger_info.br4_2_ratio1_45, "br4_2_ratio1_45/F");
    T_tagger->Branch("br4_2_ratio1_35", &tagger_info.br4_2_ratio1_35, "br4_2_ratio1_35/F");
    T_tagger->Branch("br4_2_ratio1_25", &tagger_info.br4_2_ratio1_25, "br4_2_ratio1_25/F");
    T_tagger->Branch("br4_2_ratio1_15", &tagger_info.br4_2_ratio1_15, "br4_2_ratio1_15/F");
    T_tagger->Branch("br4_2_iso_angle", &tagger_info.br4_2_iso_angle, "br4_2_iso_angle/F");
    T_tagger->Branch("br4_2_iso_angle1", &tagger_info.br4_2_iso_angle1, "br4_2_iso_angle1/F");
    T_tagger->Branch("br4_2_angle", &tagger_info.br4_2_angle, "br4_2_angle/F");
    T_tagger->Branch("br4_2_flag", &tagger_info.br4_2_flag, "br4_2_flag/F");

    T_tagger->Branch("br4_flag", &tagger_info.br4_flag, "br4_flag/F");


    T_tagger->Branch("hol_1_n_valid_tracks", &tagger_info.hol_1_n_valid_tracks,"hol_1_n_valid_tracks/F");
    T_tagger->Branch("hol_1_min_angle", &tagger_info.hol_1_min_angle,"hol_1_min_angle/F");
    T_tagger->Branch("hol_1_energy", &tagger_info.hol_1_energy,"hol_1_energy/F");
    T_tagger->Branch("hol_1_flag_all_shower", &tagger_info.hol_1_flag_all_shower,"hol_1_flag_all_shower/F");
    T_tagger->Branch("hol_1_min_length", &tagger_info.hol_1_min_length,"hol_1_min_length/F");
    T_tagger->Branch("hol_1_flag", &tagger_info.hol_1_flag,"hol_1_flag/F");

    T_tagger->Branch("hol_2_min_angle", &tagger_info.hol_2_min_angle,"hol_2_min_angle/F");
    T_tagger->Branch("hol_2_medium_dQ_dx", &tagger_info.hol_2_medium_dQ_dx,"hol_2_medium_dQ_dx/F");
    T_tagger->Branch("hol_2_ncount", &tagger_info.hol_2_ncount,"hol_2_ncount/F");
    T_tagger->Branch("hol_2_energy", &tagger_info.hol_2_energy,"hol_2_energy/F");
    T_tagger->Branch("hol_2_flag", &tagger_info.hol_2_flag,"hol_2_flag/F");

    T_tagger->Branch("hol_flag", &tagger_info.hol_flag,"hol_flag/F");


    T_tagger->Branch("vis_1_filled",&tagger_info.vis_1_filled,"vis_1_filled/F");
    T_tagger->Branch("vis_1_n_vtx_segs",&tagger_info.vis_1_n_vtx_segs,"vis_1_n_vtx_segs/F");
    T_tagger->Branch("vis_1_energy",&tagger_info.vis_1_energy,"vis_1_energy/F");
    T_tagger->Branch("vis_1_num_good_tracks",&tagger_info.vis_1_num_good_tracks,"vis_1_num_good_tracks/F");
    T_tagger->Branch("vis_1_max_angle",&tagger_info.vis_1_max_angle,"vis_1_max_angle/F");
    T_tagger->Branch("vis_1_max_shower_angle",&tagger_info.vis_1_max_shower_angle,"vis_1_max_shower_angle/F");
    T_tagger->Branch("vis_1_tmp_length1",&tagger_info.vis_1_tmp_length1,"vis_1_tmp_length1/F");
    T_tagger->Branch("vis_1_tmp_length2",&tagger_info.vis_1_tmp_length2,"vis_1_tmp_length2/F");
    T_tagger->Branch("vis_1_particle_type",&tagger_info.vis_1_particle_type,"vis_1_particle_type/F");
    T_tagger->Branch("vis_1_flag",&tagger_info.vis_1_flag,"vis_1_flag/F");

    T_tagger->Branch("vis_2_filled",&tagger_info.vis_2_filled,"vis_2_filled/F");
    T_tagger->Branch("vis_2_n_vtx_segs",&tagger_info.vis_2_n_vtx_segs,"vis_2_n_vtx_segs/F");
    T_tagger->Branch("vis_2_min_angle",&tagger_info.vis_2_min_angle,"vis_2_min_angle/F");
    T_tagger->Branch("vis_2_min_weak_track",&tagger_info.vis_2_min_weak_track,"vis_2_min_weak_track/F");
    T_tagger->Branch("vis_2_angle_beam",&tagger_info.vis_2_angle_beam,"vis_2_angle_beam/F");
    T_tagger->Branch("vis_2_min_angle1",&tagger_info.vis_2_min_angle1,"vis_2_min_angle1/F");
    T_tagger->Branch("vis_2_iso_angle1",&tagger_info.vis_2_iso_angle1,"vis_2_iso_angle1/F");
    T_tagger->Branch("vis_2_min_medium_dQ_dx",&tagger_info.vis_2_min_medium_dQ_dx,"vis_2_min_medium_dQ_dx/F");
    T_tagger->Branch("vis_2_min_length",&tagger_info.vis_2_min_length,"vis_2_min_length/F");
    T_tagger->Branch("vis_2_sg_length",&tagger_info.vis_2_sg_length,"vis_2_sg_length/F");
    T_tagger->Branch("vis_2_max_angle",&tagger_info.vis_2_max_angle,"vis_2_max_angle/F");
    T_tagger->Branch("vis_2_max_weak_track",&tagger_info.vis_2_max_weak_track,"vis_2_max_weak_track/F");
    T_tagger->Branch("vis_2_flag",&tagger_info.vis_2_flag,"vis_2_flag/F");

    T_tagger->Branch("vis_flag",&tagger_info.vis_flag,"vis_flag/F");


    T_tagger->Branch("stem_len_energy", &tagger_info.stem_len_energy, "stem_len_energy/F");
    T_tagger->Branch("stem_len_length", &tagger_info.stem_len_length, "stem_len_length/F");
    T_tagger->Branch("stem_len_flag_avoid_muon_check", &tagger_info.stem_len_flag_avoid_muon_check, "stem_len_flag_avoid_muon_check/F");
    T_tagger->Branch("stem_len_num_daughters", &tagger_info.stem_len_num_daughters, "stem_len_num_daughters/F");
    T_tagger->Branch("stem_len_daughter_length", &tagger_info.stem_len_daughter_length, "stem_len_daughter_length/F");
    T_tagger->Branch("stem_len_flag", &tagger_info.stem_len_flag, "stem_len_flag/F");

    T_tagger->Branch("brm_n_mu_segs",&tagger_info.brm_n_mu_segs,"brm_n_mu_segs/F");
    T_tagger->Branch("brm_Ep",&tagger_info.brm_Ep,"brm_Ep/F");
    T_tagger->Branch("brm_energy",&tagger_info.brm_energy,"brm_energy/F");
    T_tagger->Branch("brm_acc_length",&tagger_info.brm_acc_length,"brm_acc_length/F");
    T_tagger->Branch("brm_shower_total_length",&tagger_info.brm_shower_total_length,"brm_shower_total_length/F");
    T_tagger->Branch("brm_connected_length",&tagger_info.brm_connected_length,"brm_connected_length/F");
    T_tagger->Branch("brm_n_size",&tagger_info.brm_n_size,"brm_n_size/F");
    T_tagger->Branch("brm_acc_direct_length",&tagger_info.brm_acc_direct_length,"brm_acc_direct_length/F");
    T_tagger->Branch("brm_n_shower_main_segs",&tagger_info.brm_n_shower_main_segs,"brm_n_shower_main_segs/F");
    T_tagger->Branch("brm_n_mu_main",&tagger_info.brm_n_mu_main,"brm_n_mu_main/F");
    T_tagger->Branch("brm_flag",&tagger_info.brm_flag,"brm_flag/F");

    T_tagger->Branch("cme_mu_energy",&tagger_info.cme_mu_energy,"cme_mu_energy/F");
    T_tagger->Branch("cme_energy",&tagger_info.cme_energy,"cme_energy/F");
    T_tagger->Branch("cme_mu_length",&tagger_info.cme_mu_length,"cme_mu_length/F");
    T_tagger->Branch("cme_length",&tagger_info.cme_length,"cme_length/F");
    T_tagger->Branch("cme_angle_beam",&tagger_info.cme_angle_beam,"cme_angle_beam/F");
    T_tagger->Branch("cme_flag",&tagger_info.cme_flag,"cme_flag/F");

    T_tagger->Branch("anc_energy",&tagger_info.anc_energy,"anc_energy/F");
    T_tagger->Branch("anc_angle",&tagger_info.anc_angle,"anc_angle/F");
    T_tagger->Branch("anc_max_angle",&tagger_info.anc_max_angle,"anc_max_angle/F");
    T_tagger->Branch("anc_max_length",&tagger_info.anc_max_length,"anc_max_length/F");
    T_tagger->Branch("anc_acc_forward_length",&tagger_info.anc_acc_forward_length,"anc_acc_forward_length/F");
    T_tagger->Branch("anc_acc_backward_length",&tagger_info.anc_acc_backward_length,"anc_acc_backward_length/F");
    T_tagger->Branch("anc_acc_forward_length1",&tagger_info.anc_acc_forward_length1,"anc_acc_forward_length1/F");
    T_tagger->Branch("anc_shower_main_length",&tagger_info.anc_shower_main_length,"anc_shower_main_length/F");
    T_tagger->Branch("anc_shower_total_length",&tagger_info.anc_shower_total_length,"anc_shower_total_length/F");
    T_tagger->Branch("anc_flag_main_outside",&tagger_info.anc_flag_main_outside,"anc_flag_main_outside/F");
    T_tagger->Branch("anc_flag",&tagger_info.anc_flag,"anc_flag/F");

    T_tagger->Branch("lem_shower_total_length",&tagger_info.lem_shower_total_length,"lem_shower_total_length/F");
    T_tagger->Branch("lem_shower_main_length",&tagger_info.lem_shower_main_length,"lem_shower_main_length/F");
    T_tagger->Branch("lem_n_3seg",&tagger_info.lem_n_3seg,"lem_n_3seg/F");
    T_tagger->Branch("lem_e_charge",&tagger_info.lem_e_charge,"lem_e_charge/F");
    T_tagger->Branch("lem_e_dQdx",&tagger_info.lem_e_dQdx,"lem_e_dQdx/F");
    T_tagger->Branch("lem_shower_num_segs",&tagger_info.lem_shower_num_segs,"lem_shower_num_segs/F");
    T_tagger->Branch("lem_shower_num_main_segs",&tagger_info.lem_shower_num_main_segs,"lem_shower_num_main_segs/F");
    T_tagger->Branch("lem_flag",&tagger_info.lem_flag,"lem_flag/F");

    T_tagger->Branch("stw_1_energy",&tagger_info.stw_1_energy,"stw_1_energy/F");
    T_tagger->Branch("stw_1_dis",&tagger_info.stw_1_dis,"stw_1_dis/F");
    T_tagger->Branch("stw_1_dQ_dx",&tagger_info.stw_1_dQ_dx,"stw_1_dQ_dx/F");
    T_tagger->Branch("stw_1_flag_single_shower",&tagger_info.stw_1_flag_single_shower,"stw_1_flag_single_shower/F");
    T_tagger->Branch("stw_1_n_pi0",&tagger_info.stw_1_n_pi0,"stw_1_n_pi0/F");
    T_tagger->Branch("stw_1_num_valid_tracks",&tagger_info.stw_1_num_valid_tracks,"stw_1_num_valid_tracks/F");
    T_tagger->Branch("stw_1_flag",&tagger_info.stw_1_flag,"stw_1_flag/F");

    T_tagger->Branch("stw_2_v_medium_dQ_dx", &tagger_info.stw_2_v_medium_dQ_dx);
    T_tagger->Branch("stw_2_v_energy", &tagger_info.stw_2_v_energy);
    T_tagger->Branch("stw_2_v_angle", &tagger_info.stw_2_v_angle);
    T_tagger->Branch("stw_2_v_dir_length", &tagger_info.stw_2_v_dir_length);
    T_tagger->Branch("stw_2_v_max_dQ_dx", &tagger_info.stw_2_v_max_dQ_dx);
    T_tagger->Branch("stw_2_v_flag", &tagger_info.stw_2_v_flag);

    T_tagger->Branch("stw_3_v_angle",&tagger_info.stw_3_v_angle);
    T_tagger->Branch("stw_3_v_dir_length",&tagger_info.stw_3_v_dir_length);
    T_tagger->Branch("stw_3_v_energy",&tagger_info.stw_3_v_energy);
    T_tagger->Branch("stw_3_v_medium_dQ_dx",&tagger_info.stw_3_v_medium_dQ_dx);
    T_tagger->Branch("stw_3_v_flag",&tagger_info.stw_3_v_flag);

    T_tagger->Branch("stw_4_v_angle",&tagger_info.stw_4_v_angle);
    T_tagger->Branch("stw_4_v_dis",&tagger_info.stw_4_v_dis);
    T_tagger->Branch("stw_4_v_energy",&tagger_info.stw_4_v_energy);
    T_tagger->Branch("stw_4_v_flag",&tagger_info.stw_4_v_flag);

    T_tagger->Branch("stw_flag", &tagger_info.stw_flag,"stw_flag/F");

    T_tagger->Branch("spt_flag_single_shower", &tagger_info.spt_flag_single_shower, "spt_flag_single_shower/F");
    T_tagger->Branch("spt_energy", &tagger_info.spt_energy, "spt_energy/F");
    T_tagger->Branch("spt_shower_main_length", &tagger_info.spt_shower_main_length, "spt_shower_main_length/F");
    T_tagger->Branch("spt_shower_total_length", &tagger_info.spt_shower_total_length, "spt_shower_total_length/F");
    T_tagger->Branch("spt_angle_beam", &tagger_info.spt_angle_beam, "spt_angle_beam/F");
    T_tagger->Branch("spt_angle_vertical", &tagger_info.spt_angle_vertical, "spt_angle_vertical/F");
    T_tagger->Branch("spt_max_dQ_dx", &tagger_info.spt_max_dQ_dx, "spt_max_dQ_dx/F");
    T_tagger->Branch("spt_angle_beam_1", &tagger_info.spt_angle_beam_1, "spt_angle_beam_1/F");
    T_tagger->Branch("spt_angle_drift", &tagger_info.spt_angle_drift, "spt_angle_drift/F");
    T_tagger->Branch("spt_angle_drift_1", &tagger_info.spt_angle_drift_1, "spt_angle_drift_1/F");
    T_tagger->Branch("spt_num_valid_tracks", &tagger_info.spt_num_valid_tracks, "spt_num_valid_tracks/F");
    T_tagger->Branch("spt_n_vtx_segs", &tagger_info.spt_n_vtx_segs, "spt_n_vtx_segs/F");
    T_tagger->Branch("spt_max_length", &tagger_info.spt_max_length, "spt_max_length/F");
    T_tagger->Branch("spt_flag", &tagger_info.spt_flag, "spt_flag/F");

    T_tagger->Branch("mgo_energy",&tagger_info.mgo_energy,"mgo_energy/F");
    T_tagger->Branch("mgo_max_energy",&tagger_info.mgo_max_energy,"mgo_max_energy/F");
    T_tagger->Branch("mgo_total_energy",&tagger_info.mgo_total_energy,"mgo_total_energy/F");
    T_tagger->Branch("mgo_n_showers",&tagger_info.mgo_n_showers,"mgo_n_showers/F");
    T_tagger->Branch("mgo_max_energy_1",&tagger_info.mgo_max_energy_1,"mgo_max_energy_1/F");
    T_tagger->Branch("mgo_max_energy_2",&tagger_info.mgo_max_energy_2,"mgo_max_energy_2/F");
    T_tagger->Branch("mgo_total_other_energy",&tagger_info.mgo_total_other_energy,"mgo_total_other_energy/F");
    T_tagger->Branch("mgo_n_total_showers",&tagger_info.mgo_n_total_showers,"mgo_n_total_showers/F");
    T_tagger->Branch("mgo_total_other_energy_1",&tagger_info.mgo_total_other_energy_1,"mgo_total_other_energy_1/F");
    T_tagger->Branch("mgo_flag",&tagger_info.mgo_flag,"mgo_flag/F");

    T_tagger->Branch("mgt_flag_single_shower",&tagger_info.mgt_flag_single_shower,"mgt_flag_single_shower/F");
    T_tagger->Branch("mgt_max_energy",&tagger_info.mgt_max_energy,"mgt_max_energy/F");
    T_tagger->Branch("mgt_energy",&tagger_info.mgt_energy,"mgt_energy/F");
    T_tagger->Branch("mgt_total_other_energy",&tagger_info.mgt_total_other_energy,"mgt_total_other_energy/F");
    T_tagger->Branch("mgt_max_energy_1",&tagger_info.mgt_max_energy_1,"mgt_max_energy_1/F");
    T_tagger->Branch("mgt_e_indirect_max_energy",&tagger_info.mgt_e_indirect_max_energy,"mgt_e_indirect_max_energy/F");
    T_tagger->Branch("mgt_e_direct_max_energy",&tagger_info.mgt_e_direct_max_energy,"mgt_e_direct_max_energy/F");
    T_tagger->Branch("mgt_n_direct_showers",&tagger_info.mgt_n_direct_showers,"mgt_n_direct_showers/F");
    T_tagger->Branch("mgt_e_direct_total_energy",&tagger_info.mgt_e_direct_total_energy,"mgt_e_direct_total_energy/F");
    T_tagger->Branch("mgt_flag_indirect_max_pio",&tagger_info.mgt_flag_indirect_max_pio,"mgt_flag_indirect_max_pio/F");
    T_tagger->Branch("mgt_e_indirect_total_energy",&tagger_info.mgt_e_indirect_total_energy,"mgt_e_indirect_total_energy/F");
    T_tagger->Branch("mgt_flag",&tagger_info.mgt_flag,"mgt_flag/F");

    T_tagger->Branch("sig_1_v_angle",&tagger_info.sig_1_v_angle);
    T_tagger->Branch("sig_1_v_flag_single_shower",&tagger_info.sig_1_v_flag_single_shower);
    T_tagger->Branch("sig_1_v_energy",&tagger_info.sig_1_v_energy);
    T_tagger->Branch("sig_1_v_energy_1",&tagger_info.sig_1_v_energy_1);
    T_tagger->Branch("sig_1_v_flag",&tagger_info.sig_1_v_flag);

    T_tagger->Branch("sig_2_v_energy",&tagger_info.sig_2_v_energy);
    T_tagger->Branch("sig_2_v_shower_angle",&tagger_info.sig_2_v_shower_angle);
    T_tagger->Branch("sig_2_v_flag_single_shower",&tagger_info.sig_2_v_flag_single_shower);
    T_tagger->Branch("sig_2_v_medium_dQ_dx",&tagger_info.sig_2_v_medium_dQ_dx);
    T_tagger->Branch("sig_2_v_start_dQ_dx",&tagger_info.sig_2_v_start_dQ_dx);
    T_tagger->Branch("sig_2_v_flag",&tagger_info.sig_2_v_flag);

    T_tagger->Branch("sig_flag",&tagger_info.sig_flag, "sig_flag/F");

    T_tagger->Branch("tro_1_v_particle_type",&tagger_info.tro_1_v_particle_type);
    T_tagger->Branch("tro_1_v_flag_dir_weak",&tagger_info.tro_1_v_flag_dir_weak);
    T_tagger->Branch("tro_1_v_min_dis",&tagger_info.tro_1_v_min_dis);
    T_tagger->Branch("tro_1_v_sg1_length",&tagger_info.tro_1_v_sg1_length);
    T_tagger->Branch("tro_1_v_shower_main_length",&tagger_info.tro_1_v_shower_main_length);
    T_tagger->Branch("tro_1_v_max_n_vtx_segs",&tagger_info.tro_1_v_max_n_vtx_segs);
    T_tagger->Branch("tro_1_v_tmp_length",&tagger_info.tro_1_v_tmp_length);
    T_tagger->Branch("tro_1_v_medium_dQ_dx",&tagger_info.tro_1_v_medium_dQ_dx);
    T_tagger->Branch("tro_1_v_dQ_dx_cut",&tagger_info.tro_1_v_dQ_dx_cut);
    T_tagger->Branch("tro_1_v_flag_shower_topology",&tagger_info.tro_1_v_flag_shower_topology);
    T_tagger->Branch("tro_1_v_flag",&tagger_info.tro_1_v_flag);

    T_tagger->Branch("tro_2_v_energy",&tagger_info.tro_2_v_energy);
    T_tagger->Branch("tro_2_v_stem_length",&tagger_info.tro_2_v_stem_length);
    T_tagger->Branch("tro_2_v_iso_angle",&tagger_info.tro_2_v_iso_angle);
    T_tagger->Branch("tro_2_v_max_length",&tagger_info.tro_2_v_max_length);
    T_tagger->Branch("tro_2_v_angle",&tagger_info.tro_2_v_angle);
    T_tagger->Branch("tro_2_v_flag",&tagger_info.tro_2_v_flag);

    T_tagger->Branch("tro_3_stem_length",&tagger_info.tro_3_stem_length,"tro_3_stem_length/F");
    T_tagger->Branch("tro_3_n_muon_segs",&tagger_info.tro_3_n_muon_segs,"tro_3_n_muon_segs/F");
    T_tagger->Branch("tro_3_energy",&tagger_info.tro_3_energy,"tro_3_energy/F");
    T_tagger->Branch("tro_3_flag",&tagger_info.tro_3_flag,"tro_3_flag/F");

    T_tagger->Branch("tro_4_v_dir2_mag",&tagger_info.tro_4_v_dir2_mag);
    T_tagger->Branch("tro_4_v_angle",&tagger_info.tro_4_v_angle);
    T_tagger->Branch("tro_4_v_angle1",&tagger_info.tro_4_v_angle1);
    T_tagger->Branch("tro_4_v_angle2",&tagger_info.tro_4_v_angle2);
    T_tagger->Branch("tro_4_v_length",&tagger_info.tro_4_v_length);
    T_tagger->Branch("tro_4_v_length1",&tagger_info.tro_4_v_length1);
    T_tagger->Branch("tro_4_v_medium_dQ_dx",&tagger_info.tro_4_v_medium_dQ_dx);
    T_tagger->Branch("tro_4_v_end_dQ_dx",&tagger_info.tro_4_v_end_dQ_dx);
    T_tagger->Branch("tro_4_v_energy",&tagger_info.tro_4_v_energy);
    T_tagger->Branch("tro_4_v_shower_main_length",&tagger_info.tro_4_v_shower_main_length);
    T_tagger->Branch("tro_4_v_flag_shower_trajectory",&tagger_info.tro_4_v_flag_shower_trajectory);
    T_tagger->Branch("tro_4_v_flag",&tagger_info.tro_4_v_flag);

    T_tagger->Branch("tro_5_v_max_angle",&tagger_info.tro_5_v_max_angle);
    T_tagger->Branch("tro_5_v_min_angle",&tagger_info.tro_5_v_min_angle);
    T_tagger->Branch("tro_5_v_max_length",&tagger_info.tro_5_v_max_length);
    T_tagger->Branch("tro_5_v_iso_angle",&tagger_info.tro_5_v_iso_angle);
    T_tagger->Branch("tro_5_v_n_vtx_segs",&tagger_info.tro_5_v_n_vtx_segs);
    T_tagger->Branch("tro_5_v_min_count",&tagger_info.tro_5_v_min_count);
    T_tagger->Branch("tro_5_v_max_count",&tagger_info.tro_5_v_max_count);
    T_tagger->Branch("tro_5_v_energy",&tagger_info.tro_5_v_energy);
    T_tagger->Branch("tro_5_v_flag",&tagger_info.tro_5_v_flag);

    T_tagger->Branch("tro_flag",&tagger_info.tro_flag,"tro_flag/F");


    // cosmic tagger ...
    T_tagger->Branch("cosmict_flag_1",&tagger_info.cosmict_flag_1,"cosmict_flag_1/F");
    T_tagger->Branch("cosmict_flag_2",&tagger_info.cosmict_flag_2,"cosmict_flag_2/F");
    T_tagger->Branch("cosmict_flag_3",&tagger_info.cosmict_flag_3,"cosmict_flag_3/F");
    T_tagger->Branch("cosmict_flag_4",&tagger_info.cosmict_flag_4,"cosmict_flag_4/F");
    T_tagger->Branch("cosmict_flag_5",&tagger_info.cosmict_flag_5,"cosmict_flag_5/F");
    T_tagger->Branch("cosmict_flag_6",&tagger_info.cosmict_flag_6,"cosmict_flag_6/F");
    T_tagger->Branch("cosmict_flag_7",&tagger_info.cosmict_flag_7,"cosmict_flag_7/F");
    T_tagger->Branch("cosmict_flag_8",&tagger_info.cosmict_flag_8,"cosmict_flag_8/F");
    T_tagger->Branch("cosmict_flag_9",&tagger_info.cosmict_flag_9,"cosmict_flag_9/F");
    T_tagger->Branch("cosmict_flag_10",&tagger_info.cosmict_flag_10);
    T_tagger->Branch("cosmict_flag",&tagger_info.cosmict_flag,"cosmict_flag/F");

    T_tagger->Branch("cosmict_2_filled",&tagger_info.cosmict_2_filled,"cosmict_2_filled/F");
    T_tagger->Branch("cosmict_2_particle_type",&tagger_info.cosmict_2_particle_type,"cosmict_2_particle_type/F");
    T_tagger->Branch("cosmict_2_n_muon_tracks",&tagger_info.cosmict_2_n_muon_tracks,"cosmict_2_n_muon_tracks/F");
    T_tagger->Branch("cosmict_2_total_shower_length",&tagger_info.cosmict_2_total_shower_length,"cosmict_2_total_shower_length/F");
    T_tagger->Branch("cosmict_2_flag_inside",&tagger_info.cosmict_2_flag_inside,"cosmict_2_flag_inside/F");
    T_tagger->Branch("cosmict_2_angle_beam",&tagger_info.cosmict_2_angle_beam,"cosmict_2_angle_beam/F");
    T_tagger->Branch("cosmict_2_flag_dir_weak",&tagger_info.cosmict_2_flag_dir_weak,"cosmict_2_flag_dir_weak/F");
    T_tagger->Branch("cosmict_2_dQ_dx_end",&tagger_info.cosmict_2_dQ_dx_end,"cosmict_2_dQ_dx_end/F");
    T_tagger->Branch("cosmict_2_dQ_dx_front",&tagger_info.cosmict_2_dQ_dx_front,"cosmict_2_dQ_dx_front/F");
    T_tagger->Branch("cosmict_2_theta",&tagger_info.cosmict_2_theta,"cosmict_2_theta/F");
    T_tagger->Branch("cosmict_2_phi",&tagger_info.cosmict_2_phi,"cosmict_2_phi/F");
    T_tagger->Branch("cosmict_2_valid_tracks",&tagger_info.cosmict_2_valid_tracks,"cosmict_2_valid_tracks/F");

    T_tagger->Branch("cosmict_3_filled",&tagger_info.cosmict_3_filled,"cosmict_3_filled/F");
    T_tagger->Branch("cosmict_3_flag_inside",&tagger_info.cosmict_3_flag_inside,"cosmict_3_flag_inside/F");
    T_tagger->Branch("cosmict_3_angle_beam",&tagger_info.cosmict_3_angle_beam,"cosmict_3_angle_beam/F");
    T_tagger->Branch("cosmict_3_flag_dir_weak",&tagger_info.cosmict_3_flag_dir_weak,"cosmict_3_flag_dir_weak/F");
    T_tagger->Branch("cosmict_3_dQ_dx_end",&tagger_info.cosmict_3_dQ_dx_end,"cosmict_3_dQ_dx_end/F");
    T_tagger->Branch("cosmict_3_dQ_dx_front",&tagger_info.cosmict_3_dQ_dx_front,"cosmict_3_dQ_dx_front/F");
    T_tagger->Branch("cosmict_3_theta",&tagger_info.cosmict_3_theta,"cosmict_3_theta/F");
    T_tagger->Branch("cosmict_3_phi",&tagger_info.cosmict_3_phi,"cosmict_3_phi/F");
    T_tagger->Branch("cosmict_3_valid_tracks",&tagger_info.cosmict_3_valid_tracks,"cosmict_3_valid_tracks/F");

    T_tagger->Branch("cosmict_4_filled",&tagger_info.cosmict_4_filled,"cosmict_4_filled/F");
    T_tagger->Branch("cosmict_4_flag_inside",&tagger_info.cosmict_4_flag_inside,"cosmict_4_flag_inside/F");
    T_tagger->Branch("cosmict_4_angle_beam",&tagger_info.cosmict_4_angle_beam,"cosmict_4_angle_beam/F");
    T_tagger->Branch("cosmict_4_connected_showers",&tagger_info.cosmict_4_connected_showers,"cosmict_4_connected_showers/F");

    T_tagger->Branch("cosmict_5_filled",&tagger_info.cosmict_5_filled,"cosmict_5_filled/F");
    T_tagger->Branch("cosmict_5_flag_inside",&tagger_info.cosmict_5_flag_inside,"cosmict_5_flag_inside/F");
    T_tagger->Branch("cosmict_5_angle_beam",&tagger_info.cosmict_5_angle_beam,"cosmict_5_angle_beam/F");
    T_tagger->Branch("cosmict_5_connected_showers",&tagger_info.cosmict_5_connected_showers,"cosmict_5_connected_showers/F");

    T_tagger->Branch("cosmict_6_filled",&tagger_info.cosmict_6_filled,"cosmict_6_filled/F");
    T_tagger->Branch("cosmict_6_flag_dir_weak",&tagger_info.cosmict_6_flag_dir_weak,"cosmict_6_flag_dir_weak/F");
    T_tagger->Branch("cosmict_6_flag_inside",&tagger_info.cosmict_6_flag_inside,"cosmict_6_flag_inside/F");
    T_tagger->Branch("cosmict_6_angle",&tagger_info.cosmict_6_angle,"cosmict_6_angle/F");


    T_tagger->Branch("cosmict_7_filled",&tagger_info.cosmict_7_filled,"cosmict_7_filled/F");
    T_tagger->Branch("cosmict_7_flag_sec",&tagger_info.cosmict_7_flag_sec,"cosmict_7_flag_sec/F");
    T_tagger->Branch("cosmict_7_n_muon_tracks",&tagger_info.cosmict_7_n_muon_tracks,"cosmict_7_n_muon_tracks/F");
    T_tagger->Branch("cosmict_7_total_shower_length",&tagger_info.cosmict_7_total_shower_length,"cosmict_7_total_shower_length/F");
    T_tagger->Branch("cosmict_7_flag_inside",&tagger_info.cosmict_7_flag_inside,"cosmict_7_flag_inside/F");
    T_tagger->Branch("cosmict_7_angle_beam",&tagger_info.cosmict_7_angle_beam,"cosmict_7_angle_beam/F");
    T_tagger->Branch("cosmict_7_flag_dir_weak",&tagger_info.cosmict_7_flag_dir_weak,"cosmict_7_flag_dir_weak/F");
    T_tagger->Branch("cosmict_7_dQ_dx_end",&tagger_info.cosmict_7_dQ_dx_end,"cosmict_7_dQ_dx_end/F");
    T_tagger->Branch("cosmict_7_dQ_dx_front",&tagger_info.cosmict_7_dQ_dx_front,"cosmict_7_dQ_dx_front/F");
    T_tagger->Branch("cosmict_7_theta",&tagger_info.cosmict_7_theta,"cosmict_7_theta/F");
    T_tagger->Branch("cosmict_7_phi",&tagger_info.cosmict_7_phi,"cosmict_7_phi/F");

    T_tagger->Branch("cosmict_8_filled",&tagger_info.cosmict_8_filled,"cosmict_8_filled/F");
    T_tagger->Branch("cosmict_8_flag_out",&tagger_info.cosmict_8_flag_out,"cosmict_8_flag_out/F");
    T_tagger->Branch("cosmict_8_muon_length",&tagger_info.cosmict_8_muon_length,"cosmict_8_muon_length/F");
    T_tagger->Branch("cosmict_8_acc_length",&tagger_info.cosmict_8_acc_length,"cosmict_8_acc_length/F");

    T_tagger->Branch("cosmict_10_flag_inside",&tagger_info.cosmict_10_flag_inside);
    T_tagger->Branch("cosmict_10_vtx_z",&tagger_info.cosmict_10_vtx_z);
    T_tagger->Branch("cosmict_10_flag_shower",&tagger_info.cosmict_10_flag_shower);
    T_tagger->Branch("cosmict_10_flag_dir_weak",&tagger_info.cosmict_10_flag_dir_weak);
    T_tagger->Branch("cosmict_10_angle_beam",&tagger_info.cosmict_10_angle_beam);
    T_tagger->Branch("cosmict_10_length",&tagger_info.cosmict_10_length);

    T_tagger->Branch("numu_cc_flag",&tagger_info.numu_cc_flag,"numu_cc_flag/F");
    T_tagger->Branch("numu_cc_flag_1",&tagger_info.numu_cc_flag_1);
    T_tagger->Branch("numu_cc_1_particle_type",&tagger_info.numu_cc_1_particle_type);
    T_tagger->Branch("numu_cc_1_length",&tagger_info.numu_cc_1_length);
    T_tagger->Branch("numu_cc_1_medium_dQ_dx",&tagger_info.numu_cc_1_medium_dQ_dx);
    T_tagger->Branch("numu_cc_1_dQ_dx_cut",&tagger_info.numu_cc_1_dQ_dx_cut);
    T_tagger->Branch("numu_cc_1_direct_length",&tagger_info.numu_cc_1_direct_length);
    T_tagger->Branch("numu_cc_1_n_daughter_tracks",&tagger_info.numu_cc_1_n_daughter_tracks);
    T_tagger->Branch("numu_cc_1_n_daughter_all",&tagger_info.numu_cc_1_n_daughter_all);

    T_tagger->Branch("numu_cc_flag_2",&tagger_info.numu_cc_flag_2);
    T_tagger->Branch("numu_cc_2_length",&tagger_info.numu_cc_2_length);
    T_tagger->Branch("numu_cc_2_total_length",&tagger_info.numu_cc_2_total_length);
    T_tagger->Branch("numu_cc_2_n_daughter_tracks",&tagger_info.numu_cc_2_n_daughter_tracks);
    T_tagger->Branch("numu_cc_2_n_daughter_all",&tagger_info.numu_cc_2_n_daughter_all);

    T_tagger->Branch("numu_cc_flag_3",&tagger_info.numu_cc_flag_3,"numu_cc_flag_3/F");
    T_tagger->Branch("numu_cc_3_particle_type",&tagger_info.numu_cc_3_particle_type,"numu_cc_3_particle_type/F");
    T_tagger->Branch("numu_cc_3_max_length",&tagger_info.numu_cc_3_max_length,"numu_cc_3_max_length/F");
    T_tagger->Branch("numu_cc_3_track_length",&tagger_info.numu_cc_3_acc_track_length,"numu_cc_3_acc_track_length/F");
    T_tagger->Branch("numu_cc_3_max_length_all",&tagger_info.numu_cc_3_max_length_all,"numu_cc_3_max_length_all/F");
    T_tagger->Branch("numu_cc_3_max_muon_length",&tagger_info.numu_cc_3_max_muon_length,"numu_cc_3_max_muon_length/F");
    T_tagger->Branch("numu_cc_3_n_daughter_tracks",&tagger_info.numu_cc_3_n_daughter_tracks,"numu_cc_3_n_daughter_tracks/F");
    T_tagger->Branch("numu_cc_3_n_daughter_all",&tagger_info.numu_cc_3_n_daughter_all,"numu_cc_3_n_daughter_all/F");

    // numu BDTs
    T_tagger->Branch("cosmict_2_4_score",&tagger_info.cosmict_2_4_score, "cosmict_2_4_score/F");
    T_tagger->Branch("cosmict_3_5_score",&tagger_info.cosmict_3_5_score, "cosmict_3_5_score/F");
    T_tagger->Branch("cosmict_6_score",&tagger_info.cosmict_6_score, "cosmict_6_score/F");
    T_tagger->Branch("cosmict_7_score",&tagger_info.cosmict_7_score, "cosmict_7_score/F");
    T_tagger->Branch("cosmict_8_score",&tagger_info.cosmict_8_score, "cosmict_8_score/F");
    T_tagger->Branch("cosmict_10_score",&tagger_info.cosmict_10_score, "cosmict_10_score/F");

    T_tagger->Branch("numu_1_score",&tagger_info.numu_1_score,"numu_1_score/F");
    T_tagger->Branch("numu_2_score",&tagger_info.numu_2_score,"numu_2_score/F");
    T_tagger->Branch("numu_3_score",&tagger_info.numu_3_score,"numu_3_score/F");

    T_tagger->Branch("cosmict_score",&tagger_info.cosmict_score,"cosmict_score/F");
    T_tagger->Branch("numu_score",&tagger_info.numu_score,"numu_score/F");


    // BDTs ...
    T_tagger->Branch("mipid_score",&tagger_info.mipid_score,"mipid_score/F");
    T_tagger->Branch("gap_score",&tagger_info.gap_score,"gap_score/F");
    T_tagger->Branch("hol_lol_score",&tagger_info.hol_lol_score,"hol_lol_score/F");
    T_tagger->Branch("cme_anc_score",&tagger_info.cme_anc_score,"cme_anc_score/F");
    T_tagger->Branch("mgo_mgt_score",&tagger_info.mgo_mgt_score,"mgo_mgt_score/F");
    T_tagger->Branch("br1_score",&tagger_info.br1_score,"br1_score/F");

    T_tagger->Branch("br3_score",&tagger_info.br3_score,"br3_score/F");
    T_tagger->Branch("br3_3_score",&tagger_info.br3_3_score,"br3_3_score/F");
    T_tagger->Branch("br3_5_score",&tagger_info.br3_5_score,"br3_5_score/F");
    T_tagger->Branch("br3_6_score",&tagger_info.br3_6_score,"br3_6_score/F");
    T_tagger->Branch("stemdir_br2_score",&tagger_info.stemdir_br2_score,"stemdir_br2_score/F");
    T_tagger->Branch("trimuon_score",&tagger_info.trimuon_score,"trimuon_score/F");

    T_tagger->Branch("br4_tro_score",&tagger_info.br4_tro_score,"br4_tro_score/F");
    T_tagger->Branch("mipquality_score",&tagger_info.mipquality_score,"mipquality_score/F");
    T_tagger->Branch("pio_1_score",&tagger_info.pio_1_score,"pio_1_score/F");
    T_tagger->Branch("pio_2_score",&tagger_info.pio_2_score,"pio_2_score/F");
    T_tagger->Branch("stw_spt_score",&tagger_info.stw_spt_score,"stw_spt_score/F");
    T_tagger->Branch("vis_1_score",&tagger_info.vis_1_score,"vis_1_score/F");

    T_tagger->Branch("vis_2_score",&tagger_info.vis_2_score,"vis_2_score/F");
    T_tagger->Branch("stw_2_score",&tagger_info.stw_2_score,"stw_2_score/F");
    T_tagger->Branch("stw_3_score",&tagger_info.stw_3_score,"stw_3_score/F");
    T_tagger->Branch("stw_4_score",&tagger_info.stw_4_score,"stw_4_score/F");
    T_tagger->Branch("sig_1_score",&tagger_info.sig_1_score,"sig_1_score/F");
    T_tagger->Branch("sig_2_score",&tagger_info.sig_2_score,"sig_2_score/F");

    T_tagger->Branch("lol_1_score",&tagger_info.lol_1_score,"lol_1_score/F");
    T_tagger->Branch("lol_2_score",&tagger_info.lol_2_score,"lol_2_score/F");
    T_tagger->Branch("tro_1_score",&tagger_info.tro_1_score,"tro_1_score/F");
    T_tagger->Branch("tro_2_score",&tagger_info.tro_2_score,"tro_2_score/F");
    T_tagger->Branch("tro_4_score",&tagger_info.tro_4_score,"tro_4_score/F");
    T_tagger->Branch("tro_5_score",&tagger_info.tro_5_score,"tro_5_score/F");
    T_tagger->Branch("nue_score",&tagger_info.nue_score,"nue_score/F");

    T_tagger->Branch("photon_flag", &tagger_info.photon_flag, "photon_flag/F");

    for (size_t i=0;i!=neutrino_vec.size();i++){
      WCPPID::Map_Proto_Vertex_Segments& map_vertex_segments = neutrino_vec.at(i)->get_map_vertex_segments();
      WCPPID::ProtoVertex *nu_vtx =  neutrino_vec.at(i)->get_main_vertex();

      auto it1 = map_vertex_segments.find(nu_vtx);
      Point vertex_point;
      if (it1 != map_vertex_segments.end() && it1->second.size()>0){
	WCPPID::ProtoSegment *sg = *map_vertex_segments[nu_vtx].begin();
	if (nu_vtx->get_wcpt().index == sg->get_wcpt_vec().front().index){
	  vertex_point = sg->get_point_vec().front();
	}else{
	  vertex_point = sg->get_point_vec().back();
	}
      }else{
	continue;
      }
      x_vtx = vertex_point.x/units::cm;
      y_vtx = vertex_point.y/units::cm;
      z_vtx = vertex_point.z/units::cm;
      tagger_info = neutrino_vec.at(i)->tagger_info;

      //      std::cout << tagger_info.cosmic_flag << " " << neutrino_vec.at(i)->tagger_info.cosmic_flag << std::endl;

      ssm_kine_tree = neutrino_vec.at(i)->get_kine_info();

      //Set numu_cc_flag to -2 if we pass KDAR gensel, but not regular gensel
      float temp_flag = tagger_info.numu_cc_flag;
      if(lm_cluster_length>1 && lm_cluster_length<10){tagger_info.numu_cc_flag=-2;}
      std::cout<<"Updating numu flag: lm_cluster_length "<<lm_cluster_length<<" New "<<tagger_info.numu_cc_flag<<" Origional "<<temp_flag<<std::endl;

      T_tagger->Fill();
    }


    // for (auto it = map_cluster_vertex.begin(); it!= map_cluster_vertex.end(); it++){
    //   WCPPID::ProtoVertex *vtx = it->second;
    //   x_vtx = vtx->get_fit_pt().x/units::cm;
    //   y_vtx = vtx->get_fit_pt().y/units::cm;
    //   z_vtx = vtx->get_fit_pt().z/units::cm;

    //   type_vtx = 3;
    //   flag_main_vtx = vtx->get_flag_neutrino_vertex();
    //   cluster_id_vtx = vtx->get_cluster_id();
    //   sub_cluster_ids_vtx->clear();
    //   //     for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
    //   //     	WCPPID::ProtoSegment *sg = *it1;
    //   //     	sub_cluster_ids_vtx->push_back(cluster_id_vtx*1000 + sg->get_id());
    //   //     }
    //   T_vtx->Fill();
    // }
  }


  TTree *TMC = new TTree("TMC","TMC");

  WCPPID::WCRecoTree reco_tree;
  reco_tree.mc_daughters = new std::vector<std::vector<int> >;
  reco_tree.mc_daughters->clear();
  reco_tree.mc_Ntrack = 0;

  TMC->Branch("mc_Ntrack", &reco_tree.mc_Ntrack);  // number of tracks in MC
  TMC->Branch("mc_id", &reco_tree.mc_id, "mc_id[mc_Ntrack]/I");  // track id; size == mc_Ntrack
  TMC->Branch("mc_pdg", &reco_tree.mc_pdg, "mc_pdg[mc_Ntrack]/I");  // track particle pdg; size == mc_Ntrack
  TMC->Branch("mc_process", &reco_tree.mc_process, "mc_process[mc_Ntrack]/I");  // track generation process code; size == mc_Ntrack
  TMC->Branch("mc_mother", &reco_tree.mc_mother, "mc_mother[mc_Ntrack]/I");  // mother id of this track; size == mc_Ntrack
  TMC->Branch("mc_included",&reco_tree.mc_included, "mc_included[mc_Ntrack]/I"); // flat labeling if this should be added to the neutrino energy ...

  TMC->Branch("mc_dir_weak",&reco_tree.mc_dir_weak,"mc_dir_weak[mc_Ntrack]/I");
  TMC->Branch("mc_kine_range",&reco_tree.mc_kine_range,"mc_kine_range[mc_Ntrack]/F");
  TMC->Branch("mc_kine_dQdx",&reco_tree.mc_kine_dQdx,"mc_kine_dQdx[mc_Ntrack]/F");
  TMC->Branch("mc_kine_charge",&reco_tree.mc_kine_charge,"mc_kine_charge[mc_Ntrack]/F");

  TMC->Branch("mc_length",&reco_tree.mc_length,"mc_length[mc_Ntrack]/F");
  TMC->Branch("mc_stopped",&reco_tree.mc_stopped,"mc_stopped[mc_Ntrack]/I");

  TMC->Branch("mc_daughters", reco_tree.mc_daughters);  // daughters id of this track; vector
  TMC->Branch("mc_startXYZT", &reco_tree.mc_startXYZT, "mc_startXYZT[mc_Ntrack][4]/F");  // start position of this track; size == mc_Ntrack
  TMC->Branch("mc_endXYZT", &reco_tree.mc_endXYZT, "mc_endXYZT[mc_Ntrack][4]/F");  // start position of this track; size == mc_Ntrack
  TMC->Branch("mc_startMomentum", &reco_tree.mc_startMomentum, "mc_startMomentum[mc_Ntrack][4]/F");  // start momentum of this track; size == mc_Ntrack
  TMC->Branch("mc_endMomentum", &reco_tree.mc_endMomentum, "mc_endMomentum[mc_Ntrack][4]/F");  // start momentum of this track; size == mc_Ntrack

  TMC->SetDirectory(file1);

  WCPPID::KineInfo kine_tree;
  kine_tree.kine_pio_flag = 0;

  TTree *T_kine = new TTree("T_kine","T_kine");
  T_kine->SetDirectory(file1);
  T_kine->Branch("kine_nu_x_corr",&kine_tree.kine_nu_x_corr,"kine_nu_x_corr/F");
  T_kine->Branch("kine_nu_y_corr",&kine_tree.kine_nu_y_corr,"kine_nu_y_corr/F");
  T_kine->Branch("kine_nu_z_corr",&kine_tree.kine_nu_z_corr,"kine_nu_z_corr/F");

  T_kine->Branch("kine_reco_Enu",&kine_tree.kine_reco_Enu,"kine_reco_Enu/F");
  T_kine->Branch("kine_reco_add_energy",&kine_tree.kine_reco_add_energy,"kine_reco_add_energy/F");
  T_kine->Branch("kine_energy_particle",&kine_tree.kine_energy_particle);
  T_kine->Branch("kine_energy_info",&kine_tree.kine_energy_info);
  T_kine->Branch("kine_particle_type",&kine_tree.kine_particle_type);
  T_kine->Branch("kine_energy_included",&kine_tree.kine_energy_included);

  T_kine->Branch("kine_pio_mass",&kine_tree.kine_pio_mass,"kine_pio_mass/F");
  T_kine->Branch("kine_pio_flag",&kine_tree.kine_pio_flag,"kine_pio_flag/I");
  T_kine->Branch("kine_pio_vtx_dis",&kine_tree.kine_pio_vtx_dis,"kine_pio_vtx_dis/F");

  T_kine->Branch("kine_pio_energy_1",&kine_tree.kine_pio_energy_1,"kine_pio_energy_1/F");
  T_kine->Branch("kine_pio_theta_1",&kine_tree.kine_pio_theta_1,"kine_pio_theta_1/F");
  T_kine->Branch("kine_pio_phi_1",&kine_tree.kine_pio_phi_1,"kine_pio_phi_1/F");
  T_kine->Branch("kine_pio_dis_1",&kine_tree.kine_pio_dis_1,"kine_pio_dis_1/F");

  T_kine->Branch("kine_pio_energy_2",&kine_tree.kine_pio_energy_2,"kine_pio_energy_2/F");
  T_kine->Branch("kine_pio_theta_2",&kine_tree.kine_pio_theta_2,"kine_pio_theta_2/F");
  T_kine->Branch("kine_pio_phi_2",&kine_tree.kine_pio_phi_2,"kine_pio_phi_2/F");
  T_kine->Branch("kine_pio_dis_2",&kine_tree.kine_pio_dis_2,"kine_pio_dis_2/F");

  T_kine->Branch("kine_pio_angle",&kine_tree.kine_pio_angle,"kine_pio_angle/F");

  for (size_t i=0; i!= neutrino_vec.size();i++){
    //std::cout<<std::endl;
    //std::cout<<"Filling nu "<<i<<" PF tree"<<std::endl;
    //    neutrino_vec.at(i)->fill_proto_main_tree(reco_tree);
    neutrino_vec.at(i)->fill_particle_tree(reco_tree );
    kine_tree = neutrino_vec.at(i)->get_kine_info();
    //    neutrino_vec.at(i)->fill_kine_tree(kine_tree );
  }
  //std::cout<<std::endl;
  TMC->Fill();
  T_kine->Fill();



  TTree *T_cluster ;
  Double_t x,y,z,q,nq;
  Int_t ncluster;
  Int_t temp_time_slice, ch_u, ch_v, ch_w;
  Int_t real_cluster_id;
  Int_t sub_cluster_id;
  T_cluster = new TTree("T_cluster","T_cluster");
  T_cluster->Branch("cluster_id",&ncluster,"cluster_id/I");
  T_cluster->Branch("real_cluster_id",&real_cluster_id,"real_cluster_id/I");
  T_cluster->Branch("sub_cluster_id",&sub_cluster_id,"sub_cluster_id/I");
  T_cluster->Branch("x",&x,"x/D");
  T_cluster->Branch("y",&y,"y/D");
  T_cluster->Branch("z",&z,"z/D");
  T_cluster->Branch("q",&q,"q/D");
  T_cluster->Branch("nq",&nq,"nq/D");
  T_cluster->Branch("time_slice",&temp_time_slice,"time_slice/I");
  T_cluster->Branch("ch_u",&ch_u,"ch_u/I");
  T_cluster->Branch("ch_v",&ch_v,"ch_v/I");
  T_cluster->Branch("ch_w",&ch_w,"ch_w/I");
  T_cluster->SetDirectory(file1);


  // save all the points, points are grouped by proto clusters ...
  for (auto it = live_clusters.begin(); it!=live_clusters.end(); it++){
    WCPPID::PR3DCluster* new_cluster = *it;
    ncluster = map_cluster_parent_id[new_cluster];
    real_cluster_id = new_cluster->get_cluster_id(); // for all other clusters ....
    ToyPointCloud *pcloud = new_cluster->get_point_cloud();
    if (pcloud!=0){
      WCP::WCPointCloud<double>& cloud = pcloud->get_cloud();
      std::vector<int>& point_sub_cluster_ids = new_cluster->get_point_sub_cluster_ids();
      for (size_t i=0;i!=cloud.pts.size();i++){
	x = cloud.pts[i].x/units::cm;
	y = cloud.pts[i].y/units::cm;
	z = cloud.pts[i].z/units::cm;

	if (point_sub_cluster_ids.size() == cloud.pts.size()){
	  if (point_sub_cluster_ids.at(i)==-1){
	    real_cluster_id = -1;
	  }else{
	    real_cluster_id =new_cluster->get_cluster_id()*1000 + point_sub_cluster_ids.at(i);
	  }
	}
	sub_cluster_id =  new_cluster->get_cluster_id();

	SlimMergeGeomCell *mcell = cloud.pts[i].mcell;
	ch_u = cloud.pts[i].index_u;
	ch_v = cloud.pts[i].index_v;
	ch_w = cloud.pts[i].index_w;

	if (mcell==0){
	  temp_time_slice = -1;
	  q = 1;
	  nq = 1;
	}else{
	  temp_time_slice = mcell->GetTimeSlice();
	  nq = 1;
	  const GeomWire *uwire = gds.by_planeindex(WirePlaneType_t(0),ch_u);
	  const GeomWire *vwire = gds.by_planeindex(WirePlaneType_t(1),ch_v);
	  const GeomWire *wwire = gds.by_planeindex(WirePlaneType_t(2),ch_w);
	  q = (mcell->Get_Wire_Charge(uwire) + mcell->Get_Wire_Charge(vwire) + mcell->Get_Wire_Charge(wwire))/3.;
	}
	T_cluster->Fill();
      }
    }
  }




  // also all points, charge can be used to save the track shower
  WCPPID::WCPointTree point_tree;
  TTree *t_rec_simple = new TTree("T_rec","T_rec");
  t_rec_simple->SetDirectory(file1);
  t_rec_simple->Branch("x",&point_tree.reco_x,"x/D");
  t_rec_simple->Branch("y",&point_tree.reco_y,"y/D");
  t_rec_simple->Branch("z",&point_tree.reco_z,"z/D");
  // hack charge to store the track or shower information ...
  t_rec_simple->Branch("q",&point_tree.reco_flag_track_shower_charge,"q/D");
  t_rec_simple->Branch("nq",&point_tree.reco_flag_track_shower,"nq/D");
  //
  t_rec_simple->Branch("cluster_id",&point_tree.reco_mother_cluster_id,"cluster_id/I");
  // later can save the particle cluster id ...
  t_rec_simple->Branch("real_cluster_id",&point_tree.reco_proto_cluster_id,"real_cluster_id/I");
  t_rec_simple->Branch("sub_cluster_id",&point_tree.reco_cluster_id,"sub_cluster_id/I");
  t_rec_simple->SetDirectory(file1);

  // save skeleton ...
  TTree *t_rec_charge = new TTree("T_rec_charge","T_rec_charge");
  t_rec_charge->SetDirectory(file1);
  t_rec_charge->Branch("x",&point_tree.reco_x,"x/D");
  t_rec_charge->Branch("y",&point_tree.reco_y,"y/D");
  t_rec_charge->Branch("z",&point_tree.reco_z,"z/D");
  t_rec_charge->Branch("q",&point_tree.reco_dQ,"q/D");
  t_rec_charge->Branch("nq",&point_tree.reco_dx,"nq/D");
  t_rec_charge->Branch("chi2",&point_tree.reco_chi2,"chi2/D");
  t_rec_charge->Branch("ndf",&point_tree.reco_ndf,"ndf/D");
  t_rec_charge->Branch("pu",&point_tree.reco_pu,"pu/D");
  t_rec_charge->Branch("pv",&point_tree.reco_pv,"pv/D");
  t_rec_charge->Branch("pw",&point_tree.reco_pw,"pw/D");
  t_rec_charge->Branch("pt",&point_tree.reco_pt,"pt/D");
  t_rec_charge->Branch("reduced_chi2",&point_tree.reco_reduced_chi2,"reduced_chi2/D");
  t_rec_charge->Branch("flag_vertex",&point_tree.reco_flag_vertex,"flag_vertex/I");
  t_rec_charge->Branch("flag_shower",&point_tree.reco_flag_track_shower,"flag_shower/I");
  t_rec_charge->Branch("rr",&point_tree.reco_rr,"rr/D");
  t_rec_charge->Branch("cluster_id",&point_tree.reco_mother_cluster_id,"cluster_id/I");
  t_rec_charge->Branch("real_cluster_id",&point_tree.reco_proto_cluster_id,"real_cluster_id/I");
  t_rec_charge->Branch("sub_cluster_id",&point_tree.reco_proto_cluster_id,"sub_cluster_id/I");

  // save skeleton informatio  ...
  TTree *t_rec_deblob = new TTree("T_rec_charge_blob","T_rec_charge_blob");
  t_rec_deblob->SetDirectory(file1);
  t_rec_deblob->Branch("x",&point_tree.reco_x,"x/D");
  t_rec_deblob->Branch("y",&point_tree.reco_y,"y/D");
  t_rec_deblob->Branch("z",&point_tree.reco_z,"z/D");
  t_rec_deblob->Branch("q",&point_tree.reco_dQ,"q/D");
  t_rec_deblob->Branch("nq",&point_tree.reco_dx,"nq/D");
  t_rec_deblob->Branch("chi2",&point_tree.reco_chi2,"chi2/D");
  t_rec_deblob->Branch("ndf",&point_tree.reco_ndf,"ndf/D");
  t_rec_deblob->Branch("flag_shower",&point_tree.reco_flag_track_shower,"flag_shower/I");
  t_rec_deblob->Branch("cluster_id",&point_tree.reco_mother_cluster_id,"cluster_id/I");
  t_rec_deblob->Branch("real_cluster_id",&point_tree.reco_particle_id,"real_cluster_id/I");
  t_rec_deblob->Branch("sub_cluster_id",&point_tree.reco_cluster_id,"sub_cluster_id/I");




  for (size_t i=0; i!= neutrino_vec.size();i++){
    int mother_cluster_id = neutrino_vec.at(i)->get_main_cluster()->get_cluster_id();
    neutrino_vec.at(i)->fill_skeleton_info_magnify(mother_cluster_id, point_tree, t_rec_charge, dQdx_scale, dQdx_offset);
    neutrino_vec.at(i)->fill_skeleton_info(mother_cluster_id, point_tree, t_rec_deblob, dQdx_scale, dQdx_offset, true);
    neutrino_vec.at(i)->fill_point_info(mother_cluster_id, point_tree, t_rec_simple);
  }





  TTree *T_proj_data = new TTree("T_proj_data","T_proj_data");
  std::vector<int> *proj_data_cluster_id = new std::vector<int>;
  std::vector<std::vector<int>> *proj_data_cluster_channel = new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *proj_data_cluster_timeslice= new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *proj_data_cluster_charge= new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *proj_data_cluster_charge_err= new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *proj_data_cluster_charge_pred= new std::vector<std::vector<int>>;

  T_proj_data->Branch("cluster_id",&proj_data_cluster_id);
  T_proj_data->Branch("channel",&proj_data_cluster_channel);
  T_proj_data->Branch("time_slice",&proj_data_cluster_timeslice);
  T_proj_data->Branch("charge",&proj_data_cluster_charge);
  T_proj_data->Branch("charge_err",&proj_data_cluster_charge_err);
  T_proj_data->Branch("charge_pred",&proj_data_cluster_charge_pred);
  T_proj_data->SetDirectory(file1);







  // original save ...
  for (auto it = live_clusters.begin(); it!=live_clusters.end(); it++){
    WCPPID::PR3DCluster* cluster = *it;
    int ndf_save= cluster->get_cluster_id();

    std::map<std::pair<int,int>, std::tuple<double,double,double> > & proj_data_u_map = cluster->get_proj_data_u_map();
    std::map<std::pair<int,int>, std::tuple<double,double,double> > & proj_data_v_map = cluster->get_proj_data_v_map();
    std::map<std::pair<int,int>, std::tuple<double,double,double> > & proj_data_w_map = cluster->get_proj_data_w_map();

    proj_data_cluster_id->push_back(ndf_save);
    std::vector<int> temp_channel;
    std::vector<int> temp_timeslice;
    std::vector<int> temp_charge;
    std::vector<int> temp_charge_err;
    std::vector<int> temp_charge_pred;
    for (auto it = proj_data_u_map.begin(); it!=proj_data_u_map.end(); it++){
      // if (it->first.first > 8256) std::cout << cluster->get_cluster_id() << " U " << it->first.first << std::endl;

      temp_channel.push_back(it->first.first);
      temp_timeslice.push_back(it->first.second);
      temp_charge.push_back(std::get<0>(it->second));
      temp_charge_err.push_back(std::get<1>(it->second));
      temp_charge_pred.push_back(std::get<2>(it->second));
    }
    for (auto it = proj_data_v_map.begin(); it!=proj_data_v_map.end(); it++){
      // if (it->first.first > 8256) std::cout << cluster->get_cluster_id() << " V " << it->first.first << std::endl;

      temp_channel.push_back(it->first.first);
      temp_timeslice.push_back(it->first.second);
      temp_charge.push_back(std::get<0>(it->second));
      temp_charge_err.push_back(std::get<1>(it->second));
      temp_charge_pred.push_back(std::get<2>(it->second));
    }
    for (auto it = proj_data_w_map.begin(); it!=proj_data_w_map.end(); it++){
      //  if (it->first.first > 8256) std::cout << cluster->get_cluster_id() << " W " << it->first.first << std::endl;

      temp_channel.push_back(it->first.first);
      temp_timeslice.push_back(it->first.second);
      temp_charge.push_back(std::get<0>(it->second));
      temp_charge_err.push_back(std::get<1>(it->second));
      temp_charge_pred.push_back(std::get<2>(it->second));
    }
    proj_data_cluster_channel->push_back(temp_channel);
    proj_data_cluster_timeslice->push_back(temp_timeslice);
    proj_data_cluster_charge->push_back(temp_charge);
    proj_data_cluster_charge_err->push_back(temp_charge_err);
    proj_data_cluster_charge_pred->push_back(temp_charge_pred);

  }
  T_proj_data->Fill();


  // now save the original projected charge information
  // fill the bad channels ...
  TTree *T_proj = new TTree("T_proj","T_proj");
  std::vector<int> *proj_cluster_id = new std::vector<int>;
  std::vector<std::vector<int>> *proj_cluster_channel = new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *proj_cluster_timeslice= new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *proj_cluster_charge= new std::vector<std::vector<int>>;
  std::vector<std::vector<int>> *proj_cluster_charge_err= new std::vector<std::vector<int>>;
  T_proj->Branch("cluster_id",&proj_cluster_id);
  T_proj->Branch("channel",&proj_cluster_channel);
  T_proj->Branch("time_slice",&proj_cluster_timeslice);
  T_proj->Branch("charge",&proj_cluster_charge);
  T_proj->Branch("charge_err",&proj_cluster_charge_err);
  T_proj->SetDirectory(file1);

  for (auto it = map_parentid_clusters.begin(); it!=map_parentid_clusters.end(); it++){
    int cluster_id = it->first;
    std::vector<int> proj_channel;
    std::vector<int> proj_timeslice;
    std::vector<int> proj_charge;
    std::vector<int> proj_charge_err;
    std::vector<int> proj_flag;
    for (auto it1 = it->second.begin(); it1!=it->second.end(); it1++){
      WCPPID::PR3DCluster *cluster = (*it1);
      cluster->get_projection(proj_channel,proj_timeslice,proj_charge, proj_charge_err, proj_flag, global_wc_map);
    }
    proj_cluster_id->push_back(cluster_id);
    proj_cluster_channel->push_back(proj_channel);
    proj_cluster_timeslice->push_back(proj_timeslice);
    proj_cluster_charge->push_back(proj_charge);
    proj_cluster_charge_err->push_back(proj_charge_err);
  }
  T_proj->Fill();


  TTree *t_bad = new TTree("T_bad","T_bad");
  t_bad->SetDirectory(file1);
  Int_t bad_npoints;
  Double_t bad_y[100],bad_z[100];
  t_bad->Branch("bad_npoints",&bad_npoints,"bad_npoints/I");
  t_bad->Branch("bad_y",bad_y,"bad_y[bad_npoints]/D");
  t_bad->Branch("bad_z",bad_z,"bad_z[bad_npoints]/D");

  WCP::SMGCSelection& dead_mcells = fid->get_Dead_mcells();
  for (auto it = dead_mcells.begin(); it!=dead_mcells.end(); it++){
    const SlimMergeGeomCell *cell = (SlimMergeGeomCell*)(*it);
    PointVector ps = cell->boundary();
    bad_npoints = ps.size();
    //    std::cout << bad_npoints << std::endl;
    for (int j=0;j!=bad_npoints;j++){
      bad_y[j] = ps.at(j).y/units::cm;
      bad_z[j] = ps.at(j).z/units::cm;
    }
    t_bad->Fill();
  }

  file1->Write();
  file1->Close();


  return 0;
}
