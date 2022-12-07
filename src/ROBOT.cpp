#include "ROBOT.h"
#include <iostream>
#include <cmath>

ROBOT::ROBOT(){}

ROBOT::~ROBOT(){}

void ROBOT::inverseKinematics(float desired_position[4],float goal[4]){
    float x2 = desired_position[0] - _length_links[2] * cos(desired_position[3]); //Assuming phi is in radians
    float y2 = desired_position[1] - _length_links[2] * sin(desired_position[3]); //Assuming phi is in radians
    float theta2 = acos((pow(x2, 2) + pow(y2, 2) - pow(_length_links[0], 2) - pow(_length_links[1], 2))/(2 * _length_links[0] * _length_links[1]));
    float c_theta1 = ((_length_links[0] + _length_links[1] * cos(theta2))*x2 + _length_links[1] * sin(theta2) * y2) / (pow(x2, 2) + pow(y2, 2));
    float s_theta1 = ((_length_links[0] + _length_links[1] * cos(theta2))*y2 - _length_links[1] * sin(theta2) * x2) / (pow(x2, 2) + pow(y2, 2));
    float theta1 = tan(s_theta1/c_theta1);
    float theta3 = desired_position[3] - (theta1 + theta2);
    float theta4 = atan(desired_position[1]/desired_position[0]);
    
    if (theta4 < 0) 
    {
        theta4 = -theta4;
    }

    if (desired_position[0] >= 0){
        if(desired_position[1] > 0)
        {
            theta4 = theta4;
        }
        else
        {
            theta4 = -theta4;
        }
    }
    else
    {
        if (desired_position[1] > 0)
        {
            theta4 = 3.14 - theta4;
        }
        else
        {
            theta4 = 3.14 + theta4;
        }
        
    }

    if(theta1 > 0.785398){
        theta1 = 0.785398;
    }
    else if(theta1 < -0.785398){
        theta1 = -0.785398;
    }

    if(theta2 > 0.785398){
        theta2 = 0.785398;
    }
    else if(theta2 < -0.785398){
        theta2 = -0.785398;
    }

    if(theta3 > 0.785398){
        theta3 = 0.785398;
    }
    else if(theta3 < -0.785398){
        theta3 = -0.785398;
    }

    float angles[] = {theta1, theta2, theta3, theta4};
    goal[0]= theta1;
    goal[1]= theta2;
    goal[2]= theta3;
    goal[3]= theta4;
    //return angles;
}

void ROBOT::setupLinks(float length_links[3]){
    _length_links[0] = length_links[0];
    _length_links[1] = length_links[1];
    _length_links[2] = length_links[2];
}