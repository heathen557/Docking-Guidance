//
// Created by luffy7n on 18-11-10.
//

#ifndef DOCKING_GUIDANCE2_TOOLS_FUNCTION_H
#define DOCKING_GUIDANCE2_TOOLS_FUNCTION_H
#include <iostream>
#include <vector>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include "framework.h"

using namespace std;

typedef struct Input_Num
{
    double x;
    double y;
}Input_Num;

typedef struct POSINFO
{
    std::string position;
    double offset;
}POSINFO;

POSINFO Plane_Straight_line_LR(const Input_Num &num1,const Input_Num &num2,Input_Num Rec_values);
POSINFO Plane_Straight_line_UD(const Input_Num &num1,const Input_Num &num2,Input_Num Rec_values);
float minkowsky(const std::vector<float > &v1, const std::vector<float> &v2, float m);
float calculateSimilarity(const std::vector<float> &v1, const std::vector<float> &v2);
double calculateVelocity(std::vector<double> &p);
float calculateLength(std::vector<float> &length);
void controlKey(int frame);

#endif //DOCKING_GUIDANCE2_TOOLS_FUNCTION_H
