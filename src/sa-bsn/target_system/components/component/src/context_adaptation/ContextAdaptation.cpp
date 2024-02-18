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

        float test[2];
        test[0] = 0;
        test[1] = 1;
        setHearRateRisks(test, test, test, test, test);

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

bool ContextAdaptation::setHearRateRisks(float* lowRisk, float* MidRisk0, float* MidRisk1, float* highRisk0, float* highRisk1) {
    ros::ServiceClient client = nh.serviceClient<services::PatientAdapt>("contextAdapt");
    services::PatientAdapt srv;
    srv.request.vitalSign = "heart_rate";
    srv.request.lowRisk_floor   = lowRisk[0];
    srv.request.lowRisk_ceil    = lowRisk[1];
    srv.request.MidRisk0_floor  = MidRisk0[0];
    srv.request.MidRisk0_ceil   = MidRisk0[1];
    srv.request.MidRisk1_floor  = MidRisk1[0];
    srv.request.MidRisk1_ceil   = MidRisk1[1];
    srv.request.highRisk0_floor = highRisk0[0];
    srv.request.highRisk0_ceil  = highRisk0[1];
    srv.request.highRisk1_floor = highRisk1[0];
    srv.request.highRisk1_ceil  = highRisk1[1];

    if (client.call(srv)) {
        ROS_INFO("Service is responsing... %d", srv.response.set);
    } else {
        ROS_INFO("Service is not respondg");
    }
}

void ContextAdaptation::monitor() {

}

void ContextAdaptation::analyze() {

}

void ContextAdaptation::plan() {

}

void ContextAdaptation::execute() {

}