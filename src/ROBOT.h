
/*
  ESP32 MicroRobotArm
  Author: Alejandro Garcia*/
  
#ifndef _ROBOT_H
#define _ROBOT_H

#include <Arduino.h>

class ROBOT{
    public:
        ROBOT();
        ~ROBOT();
        void forwardKinematics(float desired_angles[5]);
        void inverseKinematics(float desired_position[4],float goal[4]);
        void setupLinks(float length_links[3]);
        //void setupMotors(MOTOR motors[5]);
        void setCurrentAngles(float current_angles[4]);
    private:
        //MOTOR _motors[5];
        float _length_links[3];
        float _current_angles[5];      
};

#endif