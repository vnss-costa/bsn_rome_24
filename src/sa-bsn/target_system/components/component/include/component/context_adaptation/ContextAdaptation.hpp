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

        void monitor();
        void analyze();
        void plan();
        void execute();

};

#endif