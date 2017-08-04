/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email: valentina.vasco@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __VEGOMOTION__
#define __VEGOMOTION__

#include <string.h>

#include <iostream>
#include <fstream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/svm.h>


class vEncObsThread : public::yarp::os::Thread
{
private:

    yarp::dev::PolyDriver encdriver;
    yarp::dev::IEncoders *iencs;
    yarp::sig::Vector encvels;

    std::string name;
    yarp::os::BufferedPort<yarp::os::Bottle> encPort;

public:

    vEncObsThread(std::string name);
    bool threadInit();
    void run();
    void getEncVels(yarp::sig::Vector &encvelscopy);
    void threadRelease();


};

class vEgomotionThread : public yarp::os::Thread
{
private:    

    bool strictness;

    //name of the module (rootname of ports)
    std::string name;

    //thread for queues of events
    ev::queueAllocator allocatorCallback;

    //thread to read velocities from encoders
    vEncObsThread          *velobs;

    //thread for the output
    ev::collectorPort outthread;

    //output port for the vBottle with the new events computed by the module
//    yarp::os::BufferedPort<ev::vBottle> outPort;
//    yarp::os::BufferedPort<yarp::os::Bottle> encPort;
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

    yarp::os::Stamp yarpstamp;

    bool train;
    double threshold;

    //pre-trained models
    const char *mu_vx_file = "/usr/local/src/robot/event-driven/build/bin/muvx.scale.model";
    const char *mu_vy_file = "/usr/local/src/robot/event-driven/build/bin/muvy.scale.model";
    const char *sigma_vx_file = "/usr/local/src/robot/event-driven/build/bin/sigmavx.scale.model";
    const char *sigma_vy_file = "/usr/local/src/robot/event-driven/build/bin/sigmavy.scale.model";
    const char *sigma_vxvy_file = "/usr/local/src/robot/event-driven/build/bin/sigmavxvy.scale.model";
//    const char *mu_vx_file = "/home/vvasco/dev/libsvm-3.22/muvx.model";
//    const char *mu_vy_file = "/home/vvasco/dev/libsvm-3.22/muvy.model";
//    const char *sigma_vx_file = "/home/vvasco/dev/libsvm-3.22/sigmavx.model";
//    const char *sigma_vy_file = "/home/vvasco/dev/libsvm-3.22/sigmavy.model";
//    const char *sigma_vxvy_file = "/home/vvasco/dev/libsvm-3.22/sigmavxvy.model";
    std::ifstream range_file;
    yarp::sig::Vector min_j, max_j, range_j;

    //files where we save data for training
    std::ofstream mu_vx_training;
    std::ofstream mu_vy_training;
    std::ofstream sigma_vx_training;
    std::ofstream sigma_vy_training;
    std::ofstream sigma_vxvy_training;

    //models
    struct svm_model *mu_vx, *mu_vy, *sigma_vx, *sigma_vy, *sigma_vxvy;

    //encoders velocities
    struct svm_node encvel;
    struct svm_node *encveltest;

    double cmx;
    double cmy;
    int ts;
    ev::vtsHelper unwrapper;

    yarp::sig::Vector predict_mean(svm_node *encvel);
    yarp::sig::Matrix predict_cov(svm_node *encvel);
    bool detect_independent(ev::event<ev::FlowEvent> ofe, yarp::sig::Vector pred_meanv, yarp::sig::Matrix pred_covv,
                            double &mahdist, double weight);

public:

    vEgomotionThread(std::string name, bool train, double threshold, vEncObsThread *velobs);
    bool threadInit();
    bool open(std::string portname);
    void onStop();
    void run();
};


class vEgomotionModule : public yarp::os::RFModule
{

    //thread for input flow events
    vEgomotionThread       *egomotionthread;

    //thread to read velocities from encoders
    vEncObsThread          *encsobserver;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

};

#endif
//empty line to make gcc happy
