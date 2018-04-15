//
//  twiddle.hpp
//
//  Created by Raúl Solera Rallo on 10/04/18.
//
//

#ifndef twiddle_hpp
#define twiddle_hpp

#include <stdio.h>
#include <vector>

#include "PID.h"

using namespace std;

class Twiddle {
public:

  /*
   * control if apply twiddle tune up
   */
  bool apply_twiddle;
  
  /*
   * Initialization flag
   */
  bool is_initialized = false;
  
  /*
   * Twiddle range
   */
  int free_init;
  int tuning_range;
  
  /*
   * Twiddle variables
   */
  double best_error;
  double actual_error;
  double rm_error;
  vector<double> best_Ks;
  
  /*
   * Twiddle parameters
   */
  double dp;
  double di;
  double dd;
  vector<double> ds;
  
  /*
   * Twiddle control parameters
   */
  double quality = 1;

  int param_controller;
  int phase_controller;

  double timestep;
  double distance;
  double longest_distance;
  
  double max_speed;
  double actual_max_speed;
  
  bool restart = false;
  
  /*
   * Writing file control
   */
  bool write_file = false;
  
  /*
   * Constructor
   */
  Twiddle();
  
  /*
   * Destructor.
   */
  virtual ~Twiddle();
  
  /*
   * Initialize Twiddle.
   */
  void Init(double dp, double di, double dd, int apply_twiddle, int free_init, int twiddle_range);
  
  /*
   * Update the PID parameters
   */
  void TrackError(double cte, double speed, PID &pid);

  void UdateParams(PID &pid);
  
  bool Restart();
  
  int NextPhase(int phase);

};


#endif /* twiddle_hpp */
