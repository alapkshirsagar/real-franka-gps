#include "gps_agent_pkg/robotplugin.h"
#include "gps_agent_pkg/lingausscontroller.h"
#include "gps_agent_pkg/util.h"
#include <iostream>
#include <sstream>
#include <string>

using namespace gps_control;

// Constructor.
LinearGaussianController::LinearGaussianController()
: TrialController()
{
    is_configured_ = false;
}

// Destructor.
LinearGaussianController::~LinearGaussianController()
{
}


void LinearGaussianController::get_action(int t, const Eigen::VectorXd &X,
                                          const Eigen::VectorXd &obs,
                                          Eigen::VectorXd &U){
    // std::stringstream outstream;
    // std::string nextline;
    // outstream << "K_t:\n" << K_[t] << std::endl;
    // outstream << "X:\n" << X << std::endl;
    // outstream << "k_t:\n" << k_[t] << std::endl;
    // while (outstream.good())
    //   {
    //     std::getline(outstream, nextline);
    //     std::cout << "WW " << nextline << std::endl;
    //   }

    // Noise usually contained in k_
    //std::cout << "K_t:\n" << K_[t] << "\n";
    //std::cout << "X:\n" << X << "\n";
    //std::cout << "k_t:\n" << k_[t] << "\n";
    U = (K_[t] * X + k_[t]);
    //std::cout << "U:\n" << U << "\n";

    // std::cout << "Torques @ get_action: " << U.transpose() << std::endl;
}

// Configure the controller.
void LinearGaussianController::configure_controller(OptionsMap &options)
{
    //Call superclass
    TrialController::configure_controller(options);

    // TODO: Update K_
    int T = boost::get<int>(options["T"]);

    //TODO Don't do this hacky string indexing
    K_.resize(T);
    for(int i=0; i<T; i++){
        K_[i] = boost::get<Eigen::MatrixXd>(options["K_"+to_string(i)]);
    }

    k_.resize(T);
    for(int i=0; i<T; i++){
        k_[i] = boost::get<Eigen::VectorXd>(options["k_"+to_string(i)]);
    }
    ROS_INFO_STREAM("Set LG parameters");
    is_configured_ = true;
}
