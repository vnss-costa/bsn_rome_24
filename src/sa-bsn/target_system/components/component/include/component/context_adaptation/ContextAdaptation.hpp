#ifndef CONTEXTAPAPTATION_HPP
#define CONTEXTAPAPTATION_HPP

#include <ros/ros.h>
#include <ros/console.h>
#include <string>

#include "archlib/ROSComponent.hpp"
#include "messages/TargetSystemData.h"
#include "services/PatientAdapt.h"

class ContextAdaptation : public arch::ROSComponent {
    public:
        ContextAdaptation(int &argc, char **argv, std::string name);
        ~ContextAdaptation();

        void setUp();
        void tearDown();
        void body();

    private:

        void collect(const messages::TargetSystemData::ConstPtr& msg);
        ros::NodeHandle nh;

        bool setRisks(std::string vitalSign,float* lowRisk, float* MidRisk0, float* MidRisk1, float* highRisk0, float* highRisk1);

        void monitor();
        void analyze();
        void plan();
        void execute();

};

#endif