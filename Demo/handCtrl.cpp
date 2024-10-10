
#include "handCtrl.h"
#include <iostream>
#include <vector>



int test_actuator(std::string actuator_name,mjModel *m,mjData *d,double t){
    // ball_2_body_id = mj_name2id(mj_model, mjOBJ_BODY, "red_ball1");
    int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id == -1) {
        // std::cerr << "Error: Actuator with name " << actuator_name << " not found." << std::endl;
        throw("joint not found");
    } else {
        // std::cout << "Actuator ID for " << actuator_name << " is " << actuator_id << std::endl;
        d->ctrl[actuator_id] = 0.5* sin(t)+0.5 ; // Example control input

    }

    return 0;
}

int set_actuator(std::string actuator_name,mjModel *m,mjData *d,double v){
    // ball_2_body_id = mj_name2id(mj_model, mjOBJ_BODY, "red_ball1");
    int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id == -1) {
        // std::cerr << "Error: Actuator with name " << actuator_name << " not found." << std::endl;
        throw("joint not found");
    } else {
        // std::cout << "Actuator ID for " << actuator_name << " is " << actuator_id << std::endl;
        d->ctrl[actuator_id] = v ; // Example control input

    }

    return 0;
}


std::vector<std::string> hand_joint_motors={
    "M_J1",
    "M_J4",
    "M_J5",
    "M_J6",
    "M_J7",
    "M_J8",
    "M_J9",
    "M_J10",
    "M_J11",
    "M_J12",
    "M_J15",
    "M_J16",
    "M_J17",
    "M_J18",
    "M_J19",
    "M_J20",
    "M_J21",
    "M_J22"
};


int handCtrlTest(mjModel *m,mjData *d,double t){
    

    // for( int i=0;i<hand_joint_motors.size();i++){
    //     test_actuator(hand_joint_motors[i],m,d,t);
    // }



     for( int i=0;i<hand_joint_motors.size();i++){
        set_actuator(hand_joint_motors[i],m,d,0.5* sin(t)+0.5);
    }

    return 0;
}


int handCtrl(mjModel *m,mjData *d,float V[]){
  
     for( int i=0;i<hand_joint_motors.size();i++){
        set_actuator(hand_joint_motors[i],m,d,-V[i]);
    }

    return 0;
}