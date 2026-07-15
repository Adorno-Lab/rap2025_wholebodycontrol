#include "mainwindow.h"
#include <rclcpp/rclcpp.hpp>
#include <QApplication>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char *argv[])
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("taskviz_node");

    sas::DataConfiguration configuration;
    configuration.B1_1_topic_prefix = "sas_b1/b1_1";
    configuration.Z1_1_topic_prefix = "sas_z1/z1_1";
    configuration.B1_2_topic_prefix = "sas_b1/b1_2";
    configuration.Z1_2_topic_prefix = "sas_z1/z1_2";
    configuration.task_commander_topic_prefix = "task_commander";

    QApplication a(argc, argv);
    MainWindow w{node, configuration};
    w.show();
    return a.exec();
}
