#include <iostream>
#include <chrono>
#include <algorithm>
#include <string>

#include <cadmium/modeling/coupled_model.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/concept/coupled_model_assert.hpp>
#include <cadmium/modeling/dynamic_coupled.hpp>
#include <cadmium/modeling/dynamic_atomic.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/tuple_to_ostream.hpp>
#include <cadmium/logger/common_loggers.hpp>


#include "../vendor/NDTime.hpp"
#include "../atomics/iestream.hpp"

#include "../data_structures/message.hpp"
#include "../atomics/digitalInput.hpp"
#include "../atomics/analogInput.hpp"
#include "../atomics/pwmOutput.hpp"
#include "../atomics/digitalOutput.hpp"

#include "../atomics/seeedBotDriver.hpp"

#ifdef ECADMIUM
  #include "../mbed.h"
#else
  const char* A0 = "A0_RightIR_In.txt";
  const char* A2 = "A2_CenterIR_In.txt";
  const char* D4 = "D4_LeftIR_In.txt";
  //#define D11 "D11_RightMotor2_Out.txt"
#endif

// SCARED OF THE DARK definition is for the analog sensor demo.
// If the analog sensor is not present then this definition should be commented out.
//#define SCARED_OF_THE_DARK

using namespace std;

using hclock=chrono::high_resolution_clock;
using TIME = NDTime;

// You must increase stack size for ECADMIUM. 
// The main functionality will be ran in a new thread with increased stack size
// See below for reference:
// https://os.mbed.com/questions/79584/Change-main-thread-stack-size/
#ifdef ECADMIUM
Thread app_thread(osPriorityNormal, 16*1024); // 16k stack
void run_app();
#endif

int main(int argc, char ** argv) {

  //This will end the main thread and create a new one with more stack.
  /********************************************************************/
  #ifdef ECADMIUM
    app_thread.start(&run_app);
      // Let the main thread die on the embedded platform. 
    }
    // run_app is only used for embedded threading, everything runs in main when simulated.
    void run_app(){
  #endif
/********************************************************************/

  // all simulation timing and I/O streams are ommited when running embedded 
  #ifndef ECADMIUM
    auto start = hclock::now(); //to measure simulation execution time

/*************** Loggers *******************/

  static std::ofstream out_data("seed_bot_test_output.txt");
  struct oss_sink_provider{
    static std::ostream& sink(){   
      return out_data;
    }
  };
  #else 
  //Logging is done over cout in ECADMIUM
  struct oss_sink_provider{
    static std::ostream& sink(){   
      return cout;
    }
  };
  #endif

  using info=cadmium::logger::logger<cadmium::logger::logger_info, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using debug=cadmium::logger::logger<cadmium::logger::logger_debug, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using state=cadmium::logger::logger<cadmium::logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using log_messages=cadmium::logger::logger<cadmium::logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using routing=cadmium::logger::logger<cadmium::logger::logger_message_routing, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using global_time=cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using local_time=cadmium::logger::logger<cadmium::logger::logger_local_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using log_all=cadmium::logger::multilogger<info, debug, state, log_messages, routing, global_time, local_time>;
  using logger_top=cadmium::logger::multilogger<log_messages, global_time>;


/*******************************************/



/********************************************/
/*********** APPLICATION GENERATOR **********/
/********************************************/
  using AtomicModelPtr=std::shared_ptr<cadmium::dynamic::modeling::model>;
  using CoupledModelPtr=std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>>;

/********************************************/
/********** SeedBotDriver *******************/
/********************************************/

  AtomicModelPtr seeedBotDriver = cadmium::dynamic::translate::make_dynamic_atomic_model<SeeedBotDriver, TIME>("seeedBotDriver");

/********************************************/
/****************** Input *******************/
/********************************************/
  AtomicModelPtr rightIR = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalInput, TIME>("rightIR" , A0);
  AtomicModelPtr centerIR = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalInput, TIME>("centerIR", A2);
  AtomicModelPtr leftIR = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalInput, TIME>("leftIR", D4);
  
  #ifdef SCARED_OF_THE_DARK
  #ifdef ECADMIUM
  AtomicModelPtr lightSensor = cadmium::dynamic::translate::make_dynamic_atomic_model<AnalogInput, TIME>("lightSensor", A5);
  #else
  AtomicModelPtr lightSensor = cadmium::dynamic::translate::make_dynamic_atomic_model<AnalogInput, TIME>("lightSensor");
  #endif
  #endif
/********************************************/
/***************** Output *******************/
/********************************************/
  #ifdef ECADMIUM
  AtomicModelPtr rightMotor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<PwmOutput, TIME>("rightMotor1", D8);
  #else
  AtomicModelPtr rightMotor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<PwmOutput, TIME>("rightMotor1");
  #endif
  
  #ifdef ECADMIUM
  AtomicModelPtr rightMotor2 = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalOutput, TIME>("rightMotor2", D11);
  #else
  AtomicModelPtr rightMotor2 = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalOutput, TIME>("rightMotor2");
  #endif
  
  #ifdef ECADMIUM
  AtomicModelPtr leftMotor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<PwmOutput, TIME>("leftMotor1", D12);
  #else
  AtomicModelPtr leftMotor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<PwmOutput, TIME>("leftMotor1");
  #endif

  #ifdef ECADMIUM
  AtomicModelPtr leftMotor2 = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalOutput, TIME>("leftMotor2", D13);
  #else
  AtomicModelPtr leftMotor2 = cadmium::dynamic::translate::make_dynamic_atomic_model<DigitalOutput, TIME>("leftMotor2");
  #endif

/************************/
/*******TOP MODEL********/
/************************/
  cadmium::dynamic::modeling::Ports iports_TOP = {};
  cadmium::dynamic::modeling::Ports oports_TOP = {};
  #ifdef SCARED_OF_THE_DARK
  cadmium::dynamic::modeling::Models submodels_TOP =  {lightSensor, seeedBotDriver, rightIR, centerIR, leftIR, rightMotor1, rightMotor2, leftMotor1, leftMotor2};
  #else
  cadmium::dynamic::modeling::Models submodels_TOP =  {seeedBotDriver, rightIR, centerIR, leftIR, rightMotor1, rightMotor2, leftMotor1, leftMotor2};
  #endif
  cadmium::dynamic::modeling::EICs eics_TOP = {};
  cadmium::dynamic::modeling::EOCs eocs_TOP = {};
  cadmium::dynamic::modeling::ICs ics_TOP = {
     cadmium::dynamic::translate::make_IC<seeedBotDriver_defs::rightMotor1, pwmOutput_defs::in>("seeedBotDriver","rightMotor1"),
     cadmium::dynamic::translate::make_IC<seeedBotDriver_defs::rightMotor2, digitalOutput_defs::in>("seeedBotDriver","rightMotor2"),
     cadmium::dynamic::translate::make_IC<seeedBotDriver_defs::leftMotor1, pwmOutput_defs::in>("seeedBotDriver","leftMotor1"),
     cadmium::dynamic::translate::make_IC<seeedBotDriver_defs::leftMotor2, digitalOutput_defs::in>("seeedBotDriver","leftMotor2"),
     cadmium::dynamic::translate::make_IC<digitalInput_defs::out, seeedBotDriver_defs::rightIR>("rightIR", "seeedBotDriver"),
     cadmium::dynamic::translate::make_IC<digitalInput_defs::out, seeedBotDriver_defs::leftIR>("leftIR", "seeedBotDriver"),
     cadmium::dynamic::translate::make_IC<digitalInput_defs::out, seeedBotDriver_defs::centerIR>("centerIR", "seeedBotDriver")
     #ifdef SCARED_OF_THE_DARK
     cadmium::dynamic::translate::make_IC<analogInput_defs::out, seeedBotDriver_defs::lightSensor>("lightSensor", "seeedBotDriver"),
     #endif
  };
  CoupledModelPtr TOP = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
   "TOP", 
   submodels_TOP, 
   iports_TOP, 
   oports_TOP, 
   eics_TOP, 
   eocs_TOP, 
   ics_TOP 
   );

///****************////

  #ifdef ECADMIUM
    //Enable the motors:
    DigitalOut rightMotorEn(D9);
    DigitalOut leftMotorEn(D10);
    rightMotorEn = 1;
    leftMotorEn = 1;
  #endif

  //cadmium::dynamic::engine::runner<NDTime, cadmium::logger::not_logger> r(TOP, {0});
  cadmium::dynamic::engine::runner<NDTime, log_messages> r(TOP, {0});
  r.run_until(NDTime("00:10:00:000"));

  #ifndef ECADMIUM
    return 0;
  #endif
}