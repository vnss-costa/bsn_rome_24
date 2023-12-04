#!/usr/bin/env python
import rospy
from messages import targetsystemdata

import pickle
import pandas as pd

def callback(data):
    
    # Abrir o arquivo pickle e carregar o modelo
    with open('src/beginner_tutorials/scripts/model_covid_prediction.pkl', 'rb') as file:
        print("Abriu")
        model = pickle.load(file)
    print("2")

    # Realiza a previsao de um unico caso de teste
    rospy.loginfo("trm_data %f" % (data.oxi_data))
    rospy.loginfo("trm_data %f" % (data.ecg_data))
    rospy.loginfo("trm_data C %f" % (data.trm_data))
    rospy.loginfo("trm_data F %f" % ((data.trm_data * 1.8) + 32))
    data_to_test = pd.DataFrame({'Oxygen': data.oxi_data, 'PulseRate': data.ecg_data, 'Temperature': ((data.trm_data * 1.8) + 32)}, index=[0])
    print(model.predict(data_to_test))

def listener():
    rospy.init_node('covid_detection_listener', anonymous=True)
    rospy.Subscriber("TargetSystemData", dados_bsn, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()