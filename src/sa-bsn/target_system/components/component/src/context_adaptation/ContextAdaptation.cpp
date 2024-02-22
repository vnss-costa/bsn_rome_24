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

bool ContextAdaptation::setRisks(std::string vitalSign, float* lowRisk, float* MidRisk0, float* MidRisk1, float* highRisk0, float* highRisk1) {
    ros::ServiceClient client = nh.serviceClient<services::PatientAdapt>("contextAdapt");
    services::PatientAdapt srv;
    srv.request.vitalSign = vitalSign;
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
        if(srv.response.set) {
            ROS_INFO("Params %s set successfully", vitalSign.c_str());
            return true;
        }
        else ROS_INFO("Could not write params");
    } else {
        ROS_INFO("Service is not answering");
    }
    return false;
}

void ContextAdaptation::monitor() {
    // Valores de frequência cardíaca para diferentes contextos
    float sitting_low_risk_min = 85;
    float sitting_low_risk_max = 97;
    float sitting_mod_risk_min = 70;
    float sitting_mod_risk_max = 115;

    float running_low_risk_min = 100; // Valor estimado
    float running_mod_risk_min = 100; // Valor estimado
    float running_mod_risk_max = 150; // Valor estimado

    float sleeping_low_risk_max = 60;
    float sleeping_mod_risk_min = 60;
    float sleeping_mod_risk_max = 70;

    // Exemplo de dados de frequência cardíaca (substitua pelos dados reais)
    std::vector<float> heart_rate_data = {90, 110, 130, 70, 140, 55, 120};

    // Verifique os riscos com base nos valores de frequência cardíaca
    for (float hr : heart_rate_data) {
        if (hr >= sitting_low_risk_min && hr <= sitting_low_risk_max) {
            std::cout << "Sitting - Low Risk: Heart Rate = " << hr << " bpm\n";
        } else if ((hr >= sitting_mod_risk_min && hr < sitting_low_risk_min) ||
                    (hr > sitting_low_risk_max && hr <= sitting_mod_risk_max)) {
            std::cout << "Sitting - Moderate Risk: Heart Rate = " << hr << " bpm\n";
        } else {
            std::cout << "Sitting - High Risk: Heart Rate = " << hr << " bpm\n";
        }

        // Repita o mesmo processo para running e sleeping
        // ...

        // Exemplo: Verificação para running (valores estimados)
        if (hr >= running_low_risk_min && hr <= running_mod_risk_max) {
            std::cout << "Running - Low/Moderate Risk: Heart Rate = " << hr << " bpm\n";
        } else {
            std::cout << "Running - High Risk: Heart Rate = " << hr << " bpm\n";
        }

        // Exemplo: Verificação para sleeping
        if (hr <= sleeping_low_risk_max) {
            std::cout << "Sleeping - Low Risk: Heart Rate = " << hr << " bpm\n";
        } else if (hr >= sleeping_mod_risk_min && hr <= sleeping_mod_risk_max) {
            std::cout << "Sleeping - Moderate Risk: Heart Rate = " << hr << " bpm\n";
        } else {
            std::cout << "Sleeping - High Risk: Heart Rate = " << hr << " bpm\n";
        }
    }
}


void ContextAdaptation::analyze() {

}

void ContextAdaptation::plan() {

}

void ContextAdaptation::execute() {

}