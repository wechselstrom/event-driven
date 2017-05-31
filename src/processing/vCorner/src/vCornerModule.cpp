/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email:  valentina.vasco@iit.it
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

#include "vCornerModule.h"

//#include <iomanip>

using namespace ev;

/**********************************************************/
bool vCornerModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vCorner")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();
    bool realtime = rf.check("realtime", yarp::os::Value(true)).asBool();

    /* set parameters */
    int height = rf.check("height", yarp::os::Value(128)).asInt();
    int width = rf.check("width", yarp::os::Value(128)).asInt();
    int sobelsize = rf.check("filterSize", yarp::os::Value(5)).asInt();
    unsigned int qlen = rf.check("qsize", yarp::os::Value(36)).asInt();
    int windowRad = rf.check("spatial", yarp::os::Value(5)).asInt();
    double sigma = rf.check("sigma", yarp::os::Value(1.0)).asDouble();
//    int nEvents = rf.check("nEvents", yarp::os::Value(5000)).asInt();
    double thresh = rf.check("thresh", yarp::os::Value(8.0)).asDouble();
    bool repFlag = rf.check("repFlag", yarp::os::Value(true)).asBool();
    int nthreads = rf.check("nthreads", yarp::os::Value(1)).asInt();

    /* create the thread and pass pointers to the module parameters */
    if(!realtime) {
        cornerthread = 0;
        cornercallback = new vCornerCallback(height, width, sobelsize, windowRad, sigma, qlen, thresh, repFlag);
        return cornercallback->open(moduleName, strict);
    }
    else {
        cornercallback = 0;
        cornerthread = new vCornerThread(height, width, moduleName, strict, qlen, windowRad, sobelsize, sigma, thresh, nthreads);
        if(!cornerthread->start())
            return false;
    }

    return true;
}

/**********************************************************/
bool vCornerModule::interruptModule()
{
    if(cornercallback) cornercallback->interrupt();
    if(cornerthread) cornerthread->stop();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vCornerModule::close()
{
    if(cornercallback) {
        cornercallback->close();
        delete cornercallback;
    }
    if(cornerthread) delete cornerthread;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vCornerModule::updateModule()
{
    return true;
}

/**********************************************************/
double vCornerModule::getPeriod()
{
    return 1;
}

//empty line to make gcc happy