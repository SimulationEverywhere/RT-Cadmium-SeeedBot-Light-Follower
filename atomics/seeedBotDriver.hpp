/**
* or:
* Cadmium implementation of CD++ or atomic model
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

using namespace cadmium;
using namespace std;
enum DriveState {right, straight, left, stop};
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
            }; 
            state_type state;
            // ports definition

            using input_ports=std::tuple<typename defs::rightIR, typename defs::centerIR, typename defs::leftIR>;
            using output_ports=std::tuple<typename defs::rightMotor1, typename defs::rightMotor2, typename defs::leftMotor1, typename defs::leftMotor2>;

            // internal transition
            void internal_transition() {
              //Do nothing... 
            }

            // external transition
            void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) { 
              for(const auto &x : get_messages<typename defs::rightIR>(mbs)){
                state.rightIR = (x.value == 0);
              }
              for(const auto &x : get_messages<typename defs::centerIR>(mbs)){
                state.centerIR = (x.value == 0);
              }
              for(const auto &x : get_messages<typename defs::leftIR>(mbs)){
                state.leftIR = (x.value == 0);
              }
              if(!(state.rightIR ^ state.leftIR ^ state.centerIR) || (state.rightIR && state.leftIR && state.centerIR)) {
                state.dir = DriveState::stop;
              } else if (state.rightIR) {
                state.dir = DriveState::right;
              } else if (state.leftIR) {
                state.dir = DriveState::left;
              } else {
                state.dir = DriveState::straight;
              }
            }

            // confluence transition
            void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
              internal_transition();
              external_transition(TIME(), std::move(mbs));
            }

            // output function
            typename make_message_bags<output_ports>::type output() const {
              typename make_message_bags<output_ports>::type bags;
              Message_t rightMotorOut1, rightMotorOut2, leftMotorOut1, leftMotorOut2;  
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

              get_messages<typename defs::rightMotor1>(bags).push_back(rightMotorOut1);
              get_messages<typename defs::rightMotor2>(bags).push_back(rightMotorOut2);
              get_messages<typename defs::leftMotor1>(bags).push_back(leftMotorOut1);
              get_messages<typename defs::leftMotor2>(bags).push_back(leftMotorOut2);
                
              return bags;
            }

            // time_advance function
            TIME time_advance() const { 
              return std::numeric_limits<TIME>::infinity();
            }

            friend std::ostringstream& operator<<(std::ostringstream& os, const typename SeeedBotDriver<TIME>::state_type& i) {
              os << "Current state: " << i.dir; 
              return os;
            }
        };     


#endif // BOOST_SIMULATION_PDEVS_BLINKY_HPP