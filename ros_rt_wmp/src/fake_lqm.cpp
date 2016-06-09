#include "ros/ros.h"
#include "ros_rt_wmp_msgs/WMPInfo.h"
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <yaml-cpp/yaml.h>

char ** lqm;
int num_of_nodes;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_lqm", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    std::ostringstream oss1;
    oss1 << getenv("HOME") << "/.rt-wmp/fake_lqm.yaml";

    ros_rt_wmp_msgs::WMPInfo lqm;


    std::cout << "Reading config file " << oss1.str() << "..." << std::endl;
    try{

        YAML::Node config = YAML::LoadFile(oss1.str());
        if (YAML::Node parameter = config["num_of_nodes"]){
            num_of_nodes = parameter.as<int>();
            lqm.lqm.resize(num_of_nodes*num_of_nodes);
        }else{
            std::cerr <<"Parameter 'num_of_nodes' is mandatorty" << std::endl;
            exit(0);
        }
        if (YAML::Node links = config["links"]){
            for (int i = 0; i<links.size(); i++ ){
                YAML::Node link = links[i];
                char from = link["from"].as<int>();
                char to = link["to"].as<int>();
                int value = link["value"].as<int>();
                lqm.lqm[from * num_of_nodes + to] = value;
                lqm.lqm[to * num_of_nodes + from] = value;
            }
        }
        std::cout << "Done." << std::endl;
    }catch(std::exception e){
        std::cout << "Bad file or file not found..." << std::endl;
        exit(1);
    }

    ros::Publisher pub = n.advertise<ros_rt_wmp_msgs::WMPInfo>("/fake_lqm", 1);

    while (ros::ok()){
        ros::spinOnce();
        pub.publish(lqm);
        usleep(100000);
    }

    return 0;
}

