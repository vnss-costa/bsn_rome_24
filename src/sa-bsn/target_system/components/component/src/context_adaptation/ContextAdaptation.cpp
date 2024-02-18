#include "component/context_adaptation/ContextAdaptation.hpp"
#include "ros/ros.h"

ContextAdaptation::ContextAdaptation(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name) {}

ContextAdaptation::~ContextAdaptation() {}

void ContextAdaptation::setUp() {
    srand(time(NULL));
    float freq;
    nh.getParam("context_frequency", freq);
    ROS_INFO("Setting Up");
    ROS_INFO("Freq = %f", freq);
    rosComponentDescriptor.setFreq(freq);
}

void ContextAdaptation::body() {
    ros::Subscriber TargetSystemDataSub = nh.subscribe("TargetSystemData", 10, &ContextAdaptation::collect, this);
    ros::Rate loop_rate(rosComponentDescriptor.getFreq());
    while (ros::ok){
        ROS_INFO("Running");
        ros::spinOnce();
        loop_rate.sleep();    
            
    }   

    return;
}

void ContextAdaptation::tearDown() {
    ROS_INFO("Tearing down");
}

void ContextAdaptation::collect(const messages::TargetSystemData::ConstPtr& msg) {
    float heart_rate, spo2;
    heart_rate = msg->ecg_data;
    spo2 = msg->oxi_data;
}

void ContextAdaptation::monitor() {

}

void ContextAdaptation::analyze() {

}

void ContextAdaptation::plan() {

}

void ContextAdaptation::execute() {
    
}