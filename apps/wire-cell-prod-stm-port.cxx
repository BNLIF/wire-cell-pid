#include "WCPSst/GeomDataSource.h"

#include "WCPData/SlimMergeGeomCell.h"
#include "WCPData/TPCParams.h"
#include "WCPData/Singleton.h"
#include "WCPData/ToyCTPointCloud.h"
#include "WCPData/PhotonLibrary.h"
#include "WCPData/Opflash.h"

#include "WCPPID/ToyFiducial.h"

#include "WCPPID/CalcPoints.h"
#include "WCPPID/PR3DCluster.h"

#include "WCPPID/ExecMon.h"
#include "WCPPID/ImprovePR3DCluster.h"

#include "TH1.h"
#include "TFile.h"
#include "TTree.h"

using namespace WCP;
using namespace std;

int main(int argc, char* argv[])
{
  if (argc < 4) {
    cerr << "usage: wire-cell-uboone /path/to/ChannelWireGeometry.txt /path/to/matching.root #entry -t[0,1? in time flash only] -c[0,1? main cluster only] -o[0,1? debug output]" << endl;
    return 1;
  }
  TH1::AddDirectory(kFALSE);
  
  bool flag_in_time_only = true; // default to not run all code
  bool flag_main_cluster_only = true; // default to run only on the main cluster
  bool flag_debug_output = true; // output
  int datatier = 0; // data=0, overlay=1, full mc=2

  int flag_glm = 0;// 0: no glm, 1: m1_glm, 2:m2_glm, 3:m1m2_glm
  int flag_calib_corr = 1; // default doing the dQ/dx correction

  bool flag_timestamp = false;
  bool flag_lifetime_corr = true;
  
  for (Int_t i=1;i!=argc;i++){
    switch(argv[i][1]){
    case 'a':
      flag_lifetime_corr = atoi(&argv[i][2]);
      break;
    case 't':
      flag_in_time_only = atoi(&argv[i][2]); 
      break;
    case 'c':
      flag_main_cluster_only = atoi(&argv[i][2]);
      break;
    case 'd':
      datatier = atoi(&argv[i][2]);
      break;
    case 'o':
      flag_debug_output = atoi(&argv[i][2]);
      break;
    case 'g':
      flag_glm = atoi(&argv[i][2]);
      break;
    case 'q':
      flag_calib_corr = atoi(&argv[i][2]);
      break;
    case 'z':
      flag_timestamp = atoi(&argv[i][2]);
      break;
    }
  }

  int flag_data = 1; // data, overlay, full MC
  bool flag_match_data = true;
  if (datatier == 2) flag_match_data = false; // if MC we do not take into account the dead PMT

  WCPPID::ExecMon em("starting");
  cout << em("load geometry") << endl;
  
  cout<<" ---> xpcheck glm (0: no glm, 1: m1_glm, 2:m2_glm, 3:m1m2_glm): "<<flag_glm<<endl;
  
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
  TString filename = argv[2];
  int entry_no = atoi(argv[3]);
  
  TFile *file = new TFile(filename);
  TTree *Trun = (TTree*)file->Get("Trun");

  bool image_fail=false;
  TTree* T_eval = (TTree*)file->Get("T_eval");
  if(T_eval->GetBranch("image_fail")){
    T_eval->SetBranchAddress("image_fail",&image_fail);
  }
  T_eval->GetEntry(entry_no);
  //imaging failed, create a minimal dummy file for the next step
  if(image_fail==true){
    double eventTime;
    int run_no, subrun_no, event_no;
    Trun->SetBranchAddress("eventNo",&event_no);
    Trun->SetBranchAddress("runNo",&run_no);
    Trun->SetBranchAddress("subRunNo",&subrun_no);
    Trun->GetEntry(entry_no);
    std::cout<<"Creating dummy file for entry "<<entry_no<<" run "<<run_no<<" subrun "<<subrun_no<<" event "<<event_no<<std::endl;
    TFile *file1 = new TFile(Form("stm_%d_%d_%d.root",run_no,subrun_no,event_no),"RECREATE");
    TTree *Trun1 = new TTree("Trun","Trun");
    Trun1->SetDirectory(file1);
    int time_offset=-1;
    int triggerbits=-1;
    Trun1->Branch("eventNo",&event_no,"eventNo/I");
    Trun1->Branch("runNo",&run_no,"runNo/I");
    Trun1->Branch("subRunNo",&subrun_no,"subRunNo/I");
    Trun1->Branch("triggerBits",&triggerbits,"triggerBits/i");
    Trun1->Branch("time_offset",&time_offset,"time_offset/I");
    Trun1->Fill();

    TTree *T_match1 = new TTree("T_match","T_match");
    T_match1->SetDirectory(file1);
    Int_t ncluster=0;
    Int_t flash_id=-1;
    Int_t event_type=-1;
    Double_t flash_time=-999;
    Double_t cluster_length=-999;
    T_match1->Branch("tpc_cluster_id",&ncluster,"tpc_cluster_id/I");
    T_match1->Branch("flash_id",&flash_id,"flash_id/I");
    T_match1->Branch("event_type",&event_type,"event_type/I");
    T_match1->Branch("flash_time",&flash_time,"flash_time/D");
    T_match1->Branch("cluster_length",&cluster_length,"cluster_length/D");
    T_match1->Fill();

    TTree *T_eval1 = new TTree("T_eval","T_eval");
    T_eval1->SetDirectory(file1);
    Float_t lm_cluster_length=-999;
    T_eval1->Branch("image_fail",&image_fail,"image_fail/O");
    T_eval1->Branch("lm_cluster_length",&lm_cluster_length,"lm_cluster_length/F");
    T_eval1->Fill();

    file1->Write();
    file1->Close();

    return 0;
  }

  if (entry_no >=Trun->GetEntries()) return 0;
  double eventTime;
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
  if((triggerbits>>12) & 1U){ lowerwindow=4.9295; upperwindow=16.6483; } // NUMI
  if(((triggerbits>>9) & 1U) && time_offset != 5) { lowerwindow=3.5625; upperwindow=5.34376; } // extbnb
  if (((triggerbits>>9) & 1U) && time_offset == 5){ lowerwindow=5.3045; upperwindow=17.0233;} // EXTNUMI
  
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

  TTree *T_flash = (TTree*)file->Get("T_flash");
  Double_t time;
  Int_t type;
  Int_t flash_id;
  Int_t temp_run_no, temp_subrun_no, temp_event_no;
  T_flash->SetBranchAddress("runNo",&temp_run_no);
  T_flash->SetBranchAddress("subRunNo",&temp_subrun_no);
  T_flash->SetBranchAddress("eventNo",&temp_event_no);
  T_flash->SetBranchAddress("time",&time);
  T_flash->SetBranchAddress("type",&type);
  T_flash->SetBranchAddress("flash_id",&flash_id);
  Double_t low_time, high_time, total_PE;
  Double_t temp_PE[32], temp_PE_err[32];
  std::vector<int> *fired_channels = new std::vector<int>;
  std::vector<double> *l1_fired_time = new std::vector<double>;
  std::vector<double> *l1_fired_pe = new std::vector<double>;
  T_flash->SetBranchAddress("low_time",&low_time);
  T_flash->SetBranchAddress("high_time",&high_time);
  T_flash->SetBranchAddress("total_PE",&total_PE);
  T_flash->SetBranchAddress("PE",temp_PE);
  T_flash->SetBranchAddress("PE_err",temp_PE_err);
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
  Double_t strength, pe_pred[32], pe_meas[32], pe_meas_err[32];
  //  Bool_t flag_close_to_PMT, flag_at_x_boundary;
  Double_t chi2, ks_dis, cluster_length;
  Int_t ndf;
  T_match->SetBranchAddress("strength",&strength);
  T_match->SetBranchAddress("pe_pred",pe_pred);
  T_match->SetBranchAddress("pe_meas",pe_meas);
  T_match->SetBranchAddress("pe_meas_err",pe_meas_err);
  //T_match->SetBranchAddress("flag_close_to_PMT",&flag_close_to_PMT);
  //T_match->SetBranchAddress("flag_at_x_boundary",&flag_at_x_boundary);
  T_match->SetBranchAddress("chi2",&chi2);
  T_match->SetBranchAddress("ks_dis",&ks_dis);
  T_match->SetBranchAddress("cluster_length",&cluster_length);
  T_match->SetBranchAddress("ndf",&ndf);
  

  
  std::map<int, Opflash* > map_flash_info;
  std::map<int, int> map_flash_tpc_ids;
  std::map<int, int> map_tpc_flash_ids;
  std::map<std::pair<int, int>, std::tuple<int, double, double, int> > map_flash_tpc_pair_type; // type, ks, chi2, ndf
  
  OpflashSelection flashes;
  for (int i=0;i!=T_flash->GetEntries();i++){
    T_flash->GetEntry(i);
    if (temp_run_no!=run_no || temp_subrun_no!=subrun_no || temp_event_no != event_no) continue;
    
    Opflash *flash = new Opflash(type, flash_id, low_time, high_time, time, total_PE, *fired_channels, temp_PE, temp_PE_err, *l1_fired_time, *l1_fired_pe);
    flashes.push_back(flash);

    // test ...
    // std::cout << flash->get_low_time() << " " << flash->get_l1_fired_time().size() << " " << flash->get_PE(3) << std::endl;
    
    map_flash_info[flash_id] = flash;//std::make_pair(type, time);
  }


  for (int i=0;i!=T_match->GetEntries();i++){
    T_match->GetEntry(i);
    if (temp_run_no!=run_no || temp_subrun_no!=subrun_no || temp_event_no != event_no) continue;
    if (flash_id==-1) continue;

    
    map_flash_tpc_ids[flash_id] = tpc_cluster_id;
    map_tpc_flash_ids[tpc_cluster_id] = flash_id;
    map_flash_tpc_pair_type[std::make_pair(flash_id, tpc_cluster_id)] = std::make_tuple(event_type, ks_dis, chi2, ndf);
  }

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
  ct_point_cloud.build_kdtree_index();

 
  

  TFile *file1 = new TFile(Form("stm_%d_%d_%d.root",run_no,subrun_no,event_no),"RECREATE");
  TTree *Trun1 = new TTree("Trun","Trun");
  Trun1->SetDirectory(file1);
  Trun1->Branch("eventNo",&event_no,"eventNo/I");
  Trun1->Branch("runNo",&run_no,"runNo/I");
  Trun1->Branch("subRunNo",&subrun_no,"subRunNo/I");
  
  Trun1->Branch("triggerBits",&triggerbits,"triggerBits/i");
  Trun1->Branch("unit_dis",&unit_dis,"unit_dis/F");
  Trun1->Branch("frame_length",&frame_length,"frame_length/I");
  Trun1->Branch("eve_num",&eve_num,"eve_num/I");
  Trun1->Branch("nrebin",&nrebin,"nrebin/I");
  Trun1->Branch("time_offset",&time_offset,"time_offset/I");
  
  Trun1->Branch("timesliceId",&timesliceId);
  Trun1->Branch("timesliceChannel",&timesliceChannel);
  Trun1->Branch("raw_charge",&raw_charge);
  Trun1->Branch("raw_charge_err",&raw_charge_err);

  if (elifetime < 1000){
    Trun1->Branch("elifetime",&elifetime,"elifetime/F");
  }
  
  Trun1->Fill();
  // Trun->CloneTree(-1,"fast");
  //if (T_bad_ch!=0){
  //   T_bad_ch->CloneTree(-1,"fast");
  //}

  // load photon library   

  
  TTree *T_match1 = new TTree("T_match","T_match");
  T_match1->SetDirectory(file1);
  Int_t ncluster;
  T_match1->Branch("tpc_cluster_id",&ncluster,"tpc_cluster_id/I");
  T_match1->Branch("flash_id",&flash_id,"flash_id/I");
  T_match1->Branch("event_type",&event_type,"event_type/I");
  Double_t flash_time;
  T_match1->Branch("flash_time",&flash_time,"flash_time/D");
  cluster_length = 0;
  T_match1->Branch("cluster_length",&cluster_length,"cluster_length/D");
  for (auto it = map_flash_tpc_ids.begin(); it!=map_flash_tpc_ids.end(); it++){
    flash_time = map_flash_info[it->first]->get_time();
    //std::cout << flash_time << " " << triggerbits << " " << lowerwindow << " " << upperwindow << std::endl;
    if (flag_in_time_only && (flash_time <= lowerwindow || flash_time >= upperwindow)) continue;

    event_type = std::get<0>(map_flash_tpc_pair_type[std::make_pair(it->first, it->second)]);
    
    int flag_tgm = (event_type >> 3) & 1U;
    int flag_low_energy = (event_type >> 4) & 1U;
    int flag_lm = (event_type >> 1) & 1U;

    // std::cout << flag_tgm << " " << flag_low_energy << " " << flag_lm << std::endl;
    
    ncluster = it->second;
    flash_id = it->first;
    
    double offset_x = (flash_time - time_offset)*2./nrebin*time_slice_width;
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

    // if (flag_tgm == 0 && flag_low_energy == 0 && flag_lm ==0 ){
    if (1){ // hack for now ...
      if (flag_main_cluster_only){
    	main_cluster->create_steiner_graph(ct_point_cloud, gds, nrebin, frame_length, unit_dis);
	//	main_cluster->recover_steiner_graph();
      }else{
        for (auto it1 = temp_clusters.begin(); it1!=temp_clusters.end();it1++){
          (*it1)->create_steiner_graph(ct_point_cloud, gds, nrebin, frame_length, unit_dis);
          //(*it1)->recover_steiner_graph();
        }
      }
      
      // // holder ...
      // if (main_cluster->get_point_cloud_steiner()!=0){
      // 	if (main_cluster->get_point_cloud_steiner()->get_num_points() >= 2){
      // 	  std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> wcps = main_cluster->get_two_boundary_wcps(2); 
      // 	  main_cluster->dijkstra_shortest_paths(wcps.first,2); 
      // 	  main_cluster->cal_shortest_path(wcps.second,2);
      // 	}
      // 	if (main_cluster->get_path_wcps().size()>=2){
      // 	  main_cluster->collect_charge_trajectory(ct_point_cloud);
      // 	  main_cluster->do_tracking(ct_point_cloud, global_wc_map, flash_time*units::microsecond);
      // 	}
      // }

      // if STM
      bool tag_stm = fid->check_stm(main_cluster, additional_clusters, offset_x, flash_time, ct_point_cloud, global_wc_map, event_type);
      int flag_stm = 0;

      if( flag_glm==1 || flag_glm==3 ) {
	cout<<" ---> xpcheck run GLM-1"<<endl;

	//debugging
	bool pass_pre_glm_cuts = (!flag_low_energy && !flag_lm && !flag_tgm && !tag_stm);

	WCP::Photon_Library pl(eventTime,run_no,flag_match_data);
	// run the geometric light mismatch tagger
	bool fully_contained = (event_type >> 2) & 1U;
	std::tuple<int, WCPPID::PR3DCluster*, WCP::Opflash*> glm_tagger_results = fid->glm_tagger(eventTime, flashes, main_cluster, additional_clusters, map_flash_info[flash_id], map_flash_tpc_pair_type[std::make_pair(flash_id, ncluster)], &pl, time_offset, nrebin, unit_dis, ct_point_cloud, run_no, subrun_no, event_no, fully_contained, flag_match_data, flag_timestamp, false);
	int tag_glm = std::get<0>(glm_tagger_results);

	// TGM by supplemental tagger ...
	if (tag_glm>=2) {
	  event_type |= 1UL << 3;
	  // STM candidate ...
	}else if (tag_glm==1){
	  double temp_flash_time = (std::get<2>(glm_tagger_results))->get_time();
	  double temp_offset_x = (temp_flash_time - time_offset)*2./nrebin*time_slice_width;
	  if (fid->check_stm_only(main_cluster, additional_clusters, temp_offset_x, temp_flash_time, ct_point_cloud, global_wc_map, event_type))
	    event_type |= 1UL << 5;
	}
	flag_tgm = (event_type >> 3) & 1U;
	flag_stm = (event_type >> 5) & 1U;

	//debugging glm tagger
	// if(pass_pre_glm_cuts){
	//   if(flag_tgm){
	//     fid->write_debug(run_no,subrun_no,event_no,tag_glm);
	//   } else if(flag_stm){
	//     fid->write_debug(run_no,subrun_no,event_no,tag_glm);
	//   } else{
	//     fid->write_debug(run_no,subrun_no,event_no,0);
	//   }
	// }

      }

      // if(!tag_stm){        
      // }
      if (flag_tgm==0 && flag_stm==0 && tag_stm)
	event_type |= 1UL << 5;
      if (fid->check_full_detector_dead())
	event_type |= 1UL << 6;



      /// M2
      if( flag_glm==2 || flag_glm==3 ) {
	int user_tgm = (event_type >> 3) & 1U;
	int user_stm = (event_type >> 5) & 1U;

	if( user_tgm==0 && user_stm==0 ) {
	  WCP::Photon_Library pl(eventTime,run_no,flag_match_data, false, flag_timestamp);

	  std::tuple<int, WCPPID::PR3DCluster*, WCP::Opflash*> M2_cosmic_tagger_results = fid->M2_cosmic_tagger(eventTime, flashes, main_cluster, additional_clusters, map_flash_info[flash_id], map_flash_tpc_pair_type[std::make_pair(flash_id, ncluster)], &pl, time_offset, nrebin, unit_dis, ct_point_cloud, run_no, subrun_no, event_no, flag_match_data, global_wc_map, flag_timestamp, false);
	  
	  int m2_type = std::get<0>(M2_cosmic_tagger_results);
	  if( m2_type==1 || m2_type==2 ) {
	    event_type |= 1UL << 3;
	  }
	  if( m2_type==3 ) {
	    event_type |= 1UL << 5;
	  }

	}

	cout<<" ---> xpcheck run GLM-2"<<endl;
      }

      // xpuser
      int user_tgm = (event_type >> 3) & 1U;
      int user_stm = (event_type >> 5) & 1U;
      //user_tgm = 1;
      if( user_tgm || user_stm ) {
	cout<<" xpresult "<<user_tgm<<"\t"<<user_stm<<endl;
      }

      // xpuser

      cluster_length = 0;
      std::vector<double>& dx = main_cluster->get_dx();
      {
	if (dx.size()==0 && main_cluster->get_point_cloud_steiner()!=0){
	  if (main_cluster->get_point_cloud_steiner()->get_num_points() >= 2){
	    std::pair<WCPointCloud<double>::WCPoint,WCPointCloud<double>::WCPoint> wcps = main_cluster->get_two_boundary_wcps(2); 
	    main_cluster->dijkstra_shortest_paths(wcps.first,2); 
	    main_cluster->cal_shortest_path(wcps.second,2);
	  }
	  if (main_cluster->get_path_wcps().size()>=2){
	    main_cluster->collect_charge_trajectory(ct_point_cloud);
	    main_cluster->do_tracking(ct_point_cloud, global_wc_map, flash_time*units::microsecond);
	  }
	}
	  
	for (size_t k=0;k!=dx.size();k++){
	  cluster_length += dx.at(k)/units::cm;
	}
      }
    }

    
    
    // std::cout << it->first << " " << flash_time << " " << it->second << " " << main_cluster << std::endl;
    T_match1->Fill();
  }
  T_match1->Write();
  cout << em("STM tagger") << std::endl;
    
  if (flag_debug_output){
    TTree *T_cluster ;
    Double_t x,y,z,q,nq;
    Int_t ncluster;
    Int_t temp_time_slice, ch_u, ch_v, ch_w;
    T_cluster = new TTree("T_cluster","T_cluster");
    T_cluster->Branch("cluster_id",&ncluster,"cluster_id/I");
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
    
    for (auto it = live_clusters.begin(); it!=live_clusters.end(); it++){
      WCPPID::PR3DCluster* new_cluster = *it;  
      ncluster = map_cluster_parent_id[new_cluster]; 
      ToyPointCloud *pcloud = new_cluster->get_point_cloud();
      if (pcloud!=0){
	WCP::WCPointCloud<double>& cloud = pcloud->get_cloud();
	for (size_t i=0;i!=cloud.pts.size();i++){
	  x = cloud.pts[i].x/units::cm;
	  y = cloud.pts[i].y/units::cm;
	  z = cloud.pts[i].z/units::cm;
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
    
    Double_t pu, pv, pw, pt;
    Double_t charge_save=1, ncharge_save=1, chi2_save=1, ndf_save=1;
    TTree *T_rec = new TTree("T_rec","T_rec");
    T_rec->Branch("x",&x,"x/D");
    T_rec->Branch("y",&y,"y/D");
    T_rec->Branch("z",&z,"z/D");
    T_rec->Branch("q",&charge_save,"q/D");
    T_rec->Branch("nq",&ncharge_save,"nq/D");
    T_rec->Branch("chi2",&chi2_save,"chi2/D");
    T_rec->Branch("ndf",&ndf_save,"ndf/D");
    T_rec->Branch("pu",&pu,"pu/D");
    T_rec->Branch("pv",&pv,"pv/D");
    T_rec->Branch("pw",&pw,"pw/D");
    T_rec->Branch("pt",&pt,"pt/D");
    T_rec->SetDirectory(file1);
    
    TTree *t_rec_charge = new TTree("T_rec_charge","T_rec_charge");
    t_rec_charge->SetDirectory(file1);
    t_rec_charge->Branch("x",&x,"x/D");
    t_rec_charge->Branch("y",&y,"y/D");
    t_rec_charge->Branch("z",&z,"z/D");
    t_rec_charge->Branch("q",&charge_save,"q/D");
    t_rec_charge->Branch("nq",&ncharge_save,"nq/D");
    t_rec_charge->Branch("chi2",&chi2_save,"chi2/D");
    t_rec_charge->Branch("ndf",&ndf_save,"ndf/D");
    t_rec_charge->Branch("pu",&pu,"pu/D");
    t_rec_charge->Branch("pv",&pv,"pv/D");
    t_rec_charge->Branch("pw",&pw,"pw/D");
    t_rec_charge->Branch("pt",&pt,"pt/D");
    Double_t reduced_chi2;
    t_rec_charge->Branch("reduced_chi2",&reduced_chi2,"reduced_chi2/D");
    
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
    for (auto it = live_clusters.begin(); it!=live_clusters.end(); it++){
      
      WCPPID::PR3DCluster* new_cluster = *it;
      ndf_save = new_cluster->get_cluster_id();
      charge_save = 0;
      ncharge_save = 0;
      chi2_save = 0;
      
      std::list<WCPointCloud<double>::WCPoint>& wcps_list = new_cluster->get_path_wcps();
      for (auto it = wcps_list.begin(); it!=wcps_list.end(); it++){
	x = (*it).x/units::cm;
	y = (*it).y/units::cm;
	z = (*it).z/units::cm;
	
	Point temp_p((*it).x,(*it).y,(*it).z);
	std::vector<int> time_chs = ct_point_cloud.convert_3Dpoint_time_ch(temp_p);
	pt = time_chs.at(0);
	pu = time_chs.at(1);
	pv = time_chs.at(2);
	pw = time_chs.at(3);
	T_rec->Fill();
      }
    }
    //    cout << em("shortest path ...") << std::endl;
    
    for (auto it = live_clusters.begin(); it!=live_clusters.end(); it++){
      WCPPID::PR3DCluster* cluster = *it;
      
      ndf_save = cluster->get_cluster_id();
      // original
      PointVector& pts = cluster->get_fine_tracking_path();
      //std::vector<double> dQ, dx;
      std::vector<double>& dQ = cluster->get_dQ();
      std::vector<double>& dx = cluster->get_dx();
      std::vector<double>& tpu = cluster->get_pu();
      std::vector<double>& tpv = cluster->get_pv();
      std::vector<double>& tpw = cluster->get_pw();
      std::vector<double>& tpt = cluster->get_pt();
      std::vector<double>& Vreduced_chi2 = cluster->get_reduced_chi2();
          
      if (pts.size()!=dQ.size() || pts.size()==0) continue;
      
      for (size_t i=0; i!=pts.size(); i++){
	x = pts.at(i).x/units::cm;
	y = pts.at(i).y/units::cm;
	z = pts.at(i).z/units::cm;
	charge_save = dQ.at(i);
	ncharge_save = dx.at(i)/units::cm;
	pu = tpu.at(i);
	pv = tpv.at(i);
	pw = tpw.at(i);
	pt = tpt.at(i);
	reduced_chi2 = Vreduced_chi2.at(i);
	t_rec_charge->Fill();
      }
      
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
	temp_channel.push_back(it->first.first);
	temp_timeslice.push_back(it->first.second);
	temp_charge.push_back(std::get<0>(it->second));
	temp_charge_err.push_back(std::get<1>(it->second));
	temp_charge_pred.push_back(std::get<2>(it->second));
      }
      for (auto it = proj_data_v_map.begin(); it!=proj_data_v_map.end(); it++){
	temp_channel.push_back(it->first.first);
	temp_timeslice.push_back(it->first.second);
	temp_charge.push_back(std::get<0>(it->second));
	temp_charge_err.push_back(std::get<1>(it->second));
	temp_charge_pred.push_back(std::get<2>(it->second));
      }
      for (auto it = proj_data_w_map.begin(); it!=proj_data_w_map.end(); it++){
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
  }

  {
  // figure out the T_eval tree ...
    TTree *T_eval = (TTree*)file->Get("T_eval");
    TTree *T_eval1 = new TTree("T_eval","T_eval");
    T_eval1->SetDirectory(file1);
    
    bool truth_isCC=false, truth_isEligible=false, truth_vtxInside=false;
    int truth_nuPdg=-1;
    float truth_nuTime=-1, truth_nuEnergy=-1., truth_energyInside=0., truth_electronInside=0.;
    float truth_vtxX=-1., truth_vtxY=-1., truth_vtxZ=-1.;
    float flash_time1=-1, flash_measPe=-1., flash_predPe=-1.;
    bool flash_found=false, match_found=false;
    float match_charge=0., match_energy=0.;
    // 3D matching
    float match_completeness=0./* matched/true */, match_completeness_energy=0., match_purity=0./* matched/reco */;
    float match_purity_xz=0, match_purity_xy=0;
    UInt_t match_type1=0;
    bool match_notFC_FV=false, match_notFC_SP=false, match_notFC_DC=false, match_isTgm=false, match_isFC=false;
    bool truth_isFC = true;

    // additional
    bool match_isSTM = false;
    bool match_isFullDetDead =false;

    //for kdar gensel
    float lm_cluster_length = 999999;
    bool image_fail = false;
    
    T_eval->SetBranchAddress("run", &temp_run_no);
    T_eval->SetBranchAddress("subrun", &temp_subrun_no);
    T_eval->SetBranchAddress("event", &temp_event_no);
    T_eval->SetBranchAddress("flash_found", &flash_found);
    T_eval->SetBranchAddress("flash_time", &flash_time1);
    T_eval->SetBranchAddress("flash_measPe", &flash_measPe);
    T_eval->SetBranchAddress("flash_predPe", &flash_predPe);
    T_eval->SetBranchAddress("match_found", &match_found);
    T_eval->SetBranchAddress("match_type", &match_type1);
    T_eval->SetBranchAddress("match_isFC", &match_isFC);
    T_eval->SetBranchAddress("match_isTgm", &match_isTgm);
    T_eval->SetBranchAddress("match_notFC_FV", &match_notFC_FV);
    T_eval->SetBranchAddress("match_notFC_SP", &match_notFC_SP);
    T_eval->SetBranchAddress("match_notFC_DC", &match_notFC_DC);
 
    T_eval1->Branch("run", &temp_run_no);
    T_eval1->Branch("subrun", &temp_subrun_no);
    T_eval1->Branch("event", &temp_event_no);
    T_eval1->Branch("flash_found", &flash_found);
    T_eval1->Branch("flash_time", &flash_time1);
    T_eval1->Branch("flash_measPe", &flash_measPe);
    T_eval1->Branch("flash_predPe", &flash_predPe);
    T_eval1->Branch("match_found", &match_found);
    T_eval1->Branch("match_type", &match_type1);
    T_eval1->Branch("match_isFC", &match_isFC);
    T_eval1->Branch("match_isTgm", &match_isTgm);
    T_eval1->Branch("match_notFC_FV", &match_notFC_FV);
    T_eval1->Branch("match_notFC_SP", &match_notFC_SP);
    T_eval1->Branch("match_notFC_DC", &match_notFC_DC);
    T_eval1->Branch("match_isSTM",&match_isSTM);
    T_eval1->Branch("match_isFullDetDead",&match_isFullDetDead);
      
    if(T_eval->GetBranch("truth_nuEnergy")){
      T_eval->SetBranchAddress("truth_nuEnergy", &truth_nuEnergy);
      T_eval->SetBranchAddress("truth_energyInside", &truth_energyInside);
      T_eval->SetBranchAddress("truth_electronInside", &truth_electronInside);
      T_eval->SetBranchAddress("truth_nuPdg", &truth_nuPdg);
      T_eval->SetBranchAddress("truth_isCC", &truth_isCC);
      T_eval->SetBranchAddress("truth_isEligible", &truth_isEligible);
      T_eval->SetBranchAddress("truth_isFC", &truth_isFC);
      T_eval->SetBranchAddress("truth_vtxInside", &truth_vtxInside);
      T_eval->SetBranchAddress("truth_vtxX", &truth_vtxX);
      T_eval->SetBranchAddress("truth_vtxY", &truth_vtxY);
      T_eval->SetBranchAddress("truth_vtxZ", &truth_vtxZ);
      T_eval->SetBranchAddress("truth_nuTime", &truth_nuTime);
      T_eval->SetBranchAddress("match_completeness", &match_completeness);
      T_eval->SetBranchAddress("match_completeness_energy", &match_completeness_energy);
      T_eval->SetBranchAddress("match_purity", &match_purity);
      T_eval->SetBranchAddress("match_purity_xz", &match_purity_xz);
      T_eval->SetBranchAddress("match_purity_xy", &match_purity_xy);
      T_eval->SetBranchAddress("match_charge", &match_charge);
      T_eval->SetBranchAddress("match_energy", &match_energy);
      
      T_eval1->Branch("truth_nuEnergy", &truth_nuEnergy);
      T_eval1->Branch("truth_energyInside", &truth_energyInside);
      T_eval1->Branch("truth_electronInside", &truth_electronInside);
      T_eval1->Branch("truth_nuPdg", &truth_nuPdg);
      T_eval1->Branch("truth_isCC", &truth_isCC);
      T_eval1->Branch("truth_isEligible", &truth_isEligible);
      T_eval1->Branch("truth_isFC", &truth_isFC);
      T_eval1->Branch("truth_vtxInside", &truth_vtxInside);
      T_eval1->Branch("truth_vtxX", &truth_vtxX);
      T_eval1->Branch("truth_vtxY", &truth_vtxY);
      T_eval1->Branch("truth_vtxZ", &truth_vtxZ);
      T_eval1->Branch("truth_nuTime", &truth_nuTime);
      T_eval1->Branch("match_completeness", &match_completeness);
      T_eval1->Branch("match_completeness_energy", &match_completeness_energy);
      T_eval1->Branch("match_purity", &match_purity);
      T_eval1->Branch("match_purity_xz", &match_purity_xz);
      T_eval1->Branch("match_purity_xy", &match_purity_xy);
      T_eval1->Branch("match_charge", &match_charge);
      T_eval1->Branch("match_energy", &match_energy);
    }

    if(T_eval->GetBranch("lm_cluster_length")){
      T_eval->SetBranchAddress("lm_cluster_length", &lm_cluster_length);
      T_eval->SetBranchAddress("image_fail", &image_fail);
      T_eval1->Branch("lm_cluster_length", &lm_cluster_length,"lm_cluster_length/F");
      T_eval1->Branch("image_fail", &image_fail);
    }

    for (int i=0;i!=T_eval->GetEntries();i++){
      T_eval->GetEntry(i);
      if (temp_run_no!=run_no || temp_subrun_no!=subrun_no || temp_event_no != event_no) continue;
      
      match_isSTM = false;
      match_isFullDetDead =false;

      for (int j=0;j!=T_match1->GetEntries();j++){
	T_match1->GetEntry(j);
	if (fabs(flash_time-flash_time1)<0.1){
	  match_isSTM = (event_type >> 5) & 1U;
	  match_isFullDetDead = (event_type >> 6) & 1U;
	}
      }
      
      T_eval1->Fill();
    }

  }
  file1->Write();
  file1->Close();
  
  
  return 0;
}
