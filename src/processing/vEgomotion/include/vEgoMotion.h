///*
// * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
// * Author: Valentina Vasco
// * email: valentina.vasco@iit.it
// * Permission is granted to copy, distribute, and/or modify this program
// * under the terms of the GNU General Public License, version 2 or any
// * later version published by the Free Software Foundation.
// *
// * A copy of the license can be found at
// * http://www.robotcub.org/icub/license/gpl.txt
// *
// * This program is distributed in the hope that it will be useful, but
// * WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// * Public License for more details
// */

//#ifndef __VEGOMOTION__
//#define __VEGOMOTION__

//#include <string.h>

//#include <fstream>

//#include <yarp/os/all.h>
//#include <yarp/sig/all.h>
//#include <yarp/dev/all.h>
//#include <yarp/math/Math.h>
//#include <iCub/eventdriven/all.h>
//#include <iCub/eventdriven/svm.h>


//class vEncObsThread : public::yarp::os::Thread
//{
//private:

//    yarp::dev::PolyDriver encdriver;
//    yarp::dev::IEncoders *iencs;
//    yarp::sig::Vector encvels;

//    std::string name;

//public:

//    vEncObsThread(std::string name);
//    bool threadInit();
//    void run();
//    yarp::sig::Vector getEncVels();
//    void threadRelease();


//};

//class vEgomotionManager : public yarp::os::BufferedPort<ev::vBottle>
//{
//private:

//    bool strictness;

//    //name of the module (rootname of ports)
//    std::string moduleName;

//    //thread to read velocities from encoders
//    vEncObsThread          *encsobserver;

//    //output port for the vBottle with the new events computed by the module
//    yarp::os::BufferedPort<ev::vBottle> outPort;
//    yarp::os::BufferedPort<yarp::os::Bottle> encPort;
//    yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

//    //for helping with timestamp wrap around
//    ev::vtsHelper unwrapper;

//    bool train;
//    double threshold;
//    yarp::os::Bottle param;
//    svm_parameter svm_param;

//    //pre-trained models
//    const char *mu_vx_file = "/usr/local/src/robot/event-driven/build/bin/muvx.model";
//    const char *mu_vy_file = "/usr/local/src/robot/event-driven/build/bin/muvy.model";
//    const char *sigma_vx_file = "/usr/local/src/robot/event-driven/build/bin/sigmavx.model";
//    const char *sigma_vy_file = "/usr/local/src/robot/event-driven/build/bin/sigmavy.model";
//    const char *sigma_vxvy_file = "/usr/local/src/robot/event-driven/build/bin/sigmavxvy.model";
////    const char *mu_vx_file = "/home/vvasco/dev/robotology/event-driven/build/bin/muvx.scale.model";
////    const char *mu_vy_file = "/home/vvasco/dev/robotology/event-driven/build/bin/muvy.scale.model";
////    const char *sigma_vx_file = "/home/vvasco/dev/robotology/event-driven/build/bin/sigmavx.scale.model";
////    const char *sigma_vy_file = "/home/vvasco/dev/robotology/event-driven/build/bin/sigmavy.scale.model";
////    const char *sigma_vxvy_file = "/home/vvasco/dev/robotology/event-driven/build/bin/sigmavxvy.scale.model";
//    struct svm_model *mu_vx, *mu_vy, *sigma_vx, *sigma_vy, *sigma_vxvy;

//    //encoders velocities
//    struct svm_node *encvel;

//    yarp::sig::Vector predict_mean(svm_node *encvel);
//    yarp::sig::Matrix predict_cov(svm_node *encvel);
//    bool detect_independent(ev::event<ev::FlowEvent> ofe, yarp::sig::Vector pred_meanv, yarp::sig::Matrix pred_covv);

//    uint x;
//    uint y;
//    float vx;
//    float vy;
//    uint ts;

//public:

//    vEgomotionManager(bool train, yarp::os::Bottle param, double threshold);

//    bool    open(const std::string moduleName, bool strictness = false);
//    void    close();
//    void    interrupt();

//    //this is the entry point to your main functionality
//    void    onRead(ev::vBottle &vbot);

//};


//class vEgomotionModule : public yarp::os::RFModule
//{

//    //the vbottle input and output handler
//    vEgomotionManager      *egomotionmanager;

//public:

//    //the virtual functions that need to be overloaded
//    virtual bool configure(yarp::os::ResourceFinder &rf);
//    virtual bool interruptModule();
//    virtual bool close();
//    virtual double getPeriod();
//    virtual bool updateModule();

//};

//#endif
////empty line to make gcc happy
