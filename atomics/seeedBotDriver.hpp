/**
* 
*/

#ifndef BOOST_SIMULATION_PDEVS_BLINKY_HPP
#define BOOST_SIMULATION_PDEVS_BLINKY_HPP

#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <limits>
#include <math.h> 
#include <assert.h>
#include <memory>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <algorithm>
#include <limits>
#include <random>

#include "../data_structures/message.hpp"
//#define SCARED_OF_THE_DARK

using namespace cadmium;
using namespace std;
enum DriveState {right = 0, straight = 1, left = 2, stop = 3};
//Port definition
    struct seeedBotDriver_defs {
        //Output ports
        struct rightMotor1 : public out_port<Message_t> { };
        struct rightMotor2 : public out_port<Message_t> { };
        struct leftMotor1 : public out_port<Message_t> { };
        struct leftMotor2 : public out_port<Message_t> { };
        //Input ports
        struct rightIR : public in_port<Message_t> { };
        struct centerIR : public in_port<Message_t> { };
        struct leftIR : public in_port<Message_t> { };
        #ifdef SCARED_OF_THE_DARK
        struct lightSensor : public in_port<Message_t> { };
        #endif
    };

    template<typename TIME>
    class SeeedBotDriver {
        using defs=seeedBotDriver_defs; // putting definitions in context
        public:
            //Parameters to be overwriten when instantiating the atomic model
            TIME   slowToggleTime;
            TIME   fastToggleTime;
            // default constructor
            SeeedBotDriver() noexcept{
              state.dir = straight;
            }
            
            // state definition
            struct state_type{
              bool leftIR;
              bool centerIR;
              bool rightIR;
              DriveState dir;
              bool prop;
            }; 
            state_type state;
            // ports definition
            #ifdef SCARED_OF_THE_DARK
            using input_ports=std::tuple<typename defs::rightIR, typename defs::lightSensor, typename defs::centerIR, typename defs::leftIR>;
            #else
            using input_ports=std::tuple<typename defs::rightIR, typename defs::centerIR, typename defs::leftIR>;
            #endif
            using output_ports=std::tuple<typename defs::rightMotor1, typename defs::rightMotor2, typename defs::leftMotor1, typename defs::leftMotor2>;

            // internal transition
            void internal_transition() {
              state.prop = false;
              //Do nothing... 
            }

            // external transition
            void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) { 
              float light = 0;
              for(const auto &x : get_messages<typename defs::rightIR>(mbs)){
                state.rightIR = (x.value == 0);
              }
              for(const auto &x : get_messages<typename defs::centerIR>(mbs)){
                state.centerIR = (x.value == 0);
              }
              for(const auto &x : get_messages<typename defs::leftIR>(mbs)){
                state.leftIR = (x.value == 0);
              }
              #ifdef SCARED_OF_THE_DARK
              for(const auto &x : get_messages<typename defs::lightSensor>(mbs)){
                light = x.value;
              }
              #endif
              if((!(state.rightIR ^ state.leftIR ^ state.centerIR) && !(!state.rightIR && !state.leftIR && !state.centerIR)) || (state.rightIR && state.leftIR && state.centerIR)) {
                //This happens when no IR sensors see the line and if two or more IR sensors see the line.
                state.dir = DriveState::stop;
              } else if (state.rightIR) {
                state.dir = DriveState::left;
              } else if (state.leftIR) {
                state.dir = DriveState::right;
              } else {
                state.dir = DriveState::straight;
              }
              #ifdef SCARED_OF_THE_DARK
              if (light < 0.3) {
                state.dir = DriveState::stop;
              }
              #endif
              state.prop = true;
            }

            // confluence transition
            void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
              internal_transition();
              external_transition(TIME(), std::move(mbs));
            }

            // output function
            typename make_message_bags<output_ports>::type output() const {
              typename make_message_bags<output_ports>::type bags;
              Message_t rightMotorOut1;
              Message_t rightMotorOut2;
              Message_t leftMotorOut1;
              Message_t leftMotorOut2;  
              
              #define PWM_DRIVER
              #ifdef PWM_DRIVER

              switch(state.dir){
                case DriveState::right:
                  rightMotorOut1.value = 0.5;
                  rightMotorOut2.value = 0;
                  leftMotorOut1.value = 1;
                  leftMotorOut2.value = 1;                
                break;

                case DriveState::left:
                  rightMotorOut1.value = 1;
                  rightMotorOut2.value = 1;
                  leftMotorOut1.value = 0.5;
                  leftMotorOut2.value = 0;
                break;

                case DriveState::straight:
                  rightMotorOut1.value = 0.5;
                  rightMotorOut2.value = 0;
                  leftMotorOut1.value = 0.5;
                  leftMotorOut2.value = 0;
                break;

                case DriveState::stop:
                default:
                  rightMotorOut1.value = 0;
                  rightMotorOut2.value = 0;
                  leftMotorOut1.value = 0;
                  leftMotorOut2.value = 0;
                break;
              }

              #else

              switch(state.dir){
                case DriveState::right:
                  rightMotorOut1.value = 1;
                  rightMotorOut2.value = 0;
                  leftMotorOut1.value = 1;
                  leftMotorOut2.value = 1;                
                break;

                case DriveState::left:
                  rightMotorOut1.value = 1;
                  rightMotorOut2.value = 1;
                  leftMotorOut1.value = 1;
                  leftMotorOut2.value = 0;
                break;

                case DriveState::straight:
                  rightMotorOut1.value = 1;
                  rightMotorOut2.value = 0;
                  leftMotorOut1.value = 1;
                  leftMotorOut2.value = 0;
                break;

                case DriveState::stop:
                default:
                  rightMotorOut1.value = 0;
                  rightMotorOut2.value = 0;
                  leftMotorOut1.value = 0;
                  leftMotorOut2.value = 0;
                break;
              }

              #endif //PWM_DRIVER

              get_messages<typename defs::rightMotor1>(bags).push_back(rightMotorOut1);
              get_messages<typename defs::rightMotor2>(bags).push_back(rightMotorOut2);
              get_messages<typename defs::leftMotor1>(bags).push_back(leftMotorOut1);
              get_messages<typename defs::leftMotor2>(bags).push_back(leftMotorOut2);
                
              return bags;
            }

            // time_advance function
            TIME time_advance() const { 
              if(state.prop)
                return TIME("00:00:00");
              return std::numeric_limits<TIME>::infinity();
            }

            friend std::ostringstream& operator<<(std::ostringstream& os, const typename SeeedBotDriver<TIME>::state_type& i) {
              os << "Current state: " << i.dir; 
              return os;
            }
        };     


#endif // BOOST_SIMULATION_PDEVS_BLINKY_HPP