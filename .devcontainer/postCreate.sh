python-rospy
cd /workspaces/bsn_rome_24 &&\
    rosdep install --from-paths src --ignore-src -r -y &&\
    catkin_make &&\
    echo "source /workspaces/bsn_rome_24/devel/setup.sh" >> /root/.bashrc 