//
//  twiddle.cpp
//
//  Created by Raúl Solera Rallo on 10/04/18.
//
//

#include "twiddle.hpp"
#include <math.h>
#include <iostream>

using namespace std;

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

void Twiddle::Init(double dp, double di, double dd, int apply_twiddle, int free_init, int tuning_range) {
  
  best_error = 0;
  max_speed = 0;
  longest_distance = 0;
  actual_max_speed = 0;
  
  
  best_Ks.push_back(0);
  best_Ks.push_back(0);
  best_Ks.push_back(0);
  
  actual_error = 0;
  rm_error = 0;
  distance = 0;
  
  if(dp==0){dp = -0.01;}
  this->dp = dp;
  if(di==0){di = -0.0001;}
  this->di = di;
  if(dd==0){dd = -0.1;}
  this->dd = dd;
  
  ds.push_back(dp);
  ds.push_back(di);
  ds.push_back(dd);
  
  timestep = 0;
  distance = 0;
  
  this->free_init = free_init;
  this->tuning_range = tuning_range;
  
  param_controller = 0;
  
  this->apply_twiddle = false;
  if(apply_twiddle != 0){
    this->apply_twiddle = true;
  }
  
}

// Keep track of timestep and epoch error
// Once reached the experiment range call update params function
void Twiddle::TrackError(double cte, double speed, PID &pid){
  
  timestep += 1;

  // get track of total twiddle error
  if(timestep > free_init &&
     timestep <= tuning_range + free_init){
    actual_error += pow(cte, 2);
    rm_error = pow(actual_error/(timestep-free_init), 0.5);
    distance += speed;
    //cout << "\tDebug: " << actual_max_speed << "\t" << speed << endl;
    if(actual_max_speed<speed){actual_max_speed = speed;}
    
    // if error so far exceeds best error no need to finesh the experiment
    if(is_initialized && actual_error>best_error){
    // if(false){
    
      // go for next phase
      phase_controller = NextPhase(phase_controller);
      
      // and call the update parameter method (were the twiddle algorithm happens)
      UdateParams(pid);
      
    }
    
  }
  
  // Once reached the measurement range
  if(timestep > (tuning_range + free_init) ){
    
    // if still not initialized (not an initial error)
    if(!is_initialized){
      
      // Initialize best error and longest distance
      is_initialized = true;
      best_error = actual_error;
      max_speed = actual_max_speed;
      longest_distance = distance;
      phase_controller = 10;
      best_Ks = pid.Ks;
      
    }
    
    // call the update parameter method (were the twiddle algorithm happens)
    UdateParams(pid);
    
  }
  
}

// This is where the twiddle algorithm happens
void Twiddle::UdateParams(PID &pid){
  
  // only get out of the loop when need to calculate new error
  while(true){
    
    // phase 1.0: sum twiddle param and get new error
    if(phase_controller == 10){
      
      // change parameter
      pid.Ks[param_controller] += ds[param_controller];
      pid.UpdateParams(pid.Ks);
      
      // change twiddle phase
      phase_controller = NextPhase(phase_controller);
      
      // restart controller to get new error
      timestep = 0;
      actual_error = 0;
      actual_max_speed = 0;
      rm_error = 0;
      distance = 0;
      restart = true;
      return;
      
    }
    
    // phase 2.0: substract twiddle param and get new error
    if(phase_controller == 20){
      
      // change parameter
      pid.Ks[param_controller] -= 2*ds[param_controller];
      pid.UpdateParams(pid.Ks);
      
      // go for next twiddle phase
      phase_controller = NextPhase(phase_controller);
      
      // restart controller to get new error
      timestep = 0;
      actual_error = 0;
      actual_max_speed = 0;
      rm_error = 0;      
      distance = 0;
      restart = true;
      return;
      
    }
    
    // phase x.1: check if error has improved
    if(phase_controller%10 == 1){
      
      // if error has improved go tune next parameter
      if(actual_error < best_error){
      //if(distance > longest_distance){
        // if improved update best error
        best_error = actual_error;
        longest_distance = distance;
        best_Ks = pid.Ks;
        max_speed = actual_max_speed;
        
        // update twiddle parameters
        ds[param_controller] *= 1.1;
        quality *= 1.1;
        
        // set controller to tune next parameter
        param_controller = (param_controller + 1)%3;
        
        // init twiddle phase
        phase_controller = 10;
        
      }
      
      else{
        
        // go for next twiddle phase
        phase_controller = NextPhase(phase_controller);
        
      }
      
    }
    
    // phase 3.0: this change does not improve error --> reduce twiddle param
    // and go to tune next param
    if(phase_controller == 30){
      
      // update twiddle parameters
      pid.Ks[param_controller] += ds[param_controller];
      pid.UpdateParams(pid.Ks);
      
      // update twiddle parameters
      ds[param_controller] *= 0.9;
      quality *= 0.9;
      param_controller = (param_controller + 1)%3;
      
      // go for next twiddle phase
      phase_controller = NextPhase(phase_controller);
      
    }
    
  }
  
}


// Helper to check when the simulator should restart
bool Twiddle::Restart(){
  
  if(restart){
    restart = false;
    return true;
  }
  else{
    return false;
  }
  
}

// Helper to get next twiddle phase. What we understand by phase is:
// 10 -> sum twiddle param to pid param and restart sim to get new error
// 11 -> compare new error if better --> go for next param, else --> go for next phase
// 20 -> substract twiddle param to pid param and restart sim to get new error
// 21 -> compare new error if better --> go for next param, else --> go for next phase
// 30 -> neither sum nor substraction improve error --> reduce twiddle param and go for next param
int Twiddle::NextPhase(int phase){
  
  if(phase==10){return 11;}
  if(phase==11){return 20;}
  if(phase==20){return 21;}
  if(phase==21){return 30;}
  if(phase==30){return 10;}
  
  return 10;
  
}

