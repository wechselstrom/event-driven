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

#include "vCornerTrackingModule.h"

using namespace ev;

/**********************************************************/
bool vCornerTrackingModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vCornerTracking")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();

    /* set parameters */
    unsigned int height = rf.check("height", yarp::os::Value(128)).asInt();
    unsigned int width = rf.check("width", yarp::os::Value(128)).asInt();
    double mindistance = rf.check("mindist", yarp::os::Value(3.0)).asDouble();
    double maxdistance = rf.check("maxdist", yarp::os::Value(12.0)).asDouble();
    double trefresh = rf.check("trefresh", yarp::os::Value(1.0)).asDouble();
    int maxsize = rf.check("maxsize", yarp::os::Value(15)).asInt();
    int minevts = rf.check("minevts", yarp::os::Value(3)).asInt();
    bool callback = rf.check("callback", yarp::os::Value(true)).asBool();

    /* create the thread and pass pointers to the module parameters */
    if(callback) {
        cornertrackingthread = 0;
        cornertrackingcallback = new vCornerTrackingCallback(height, width, mindistance, maxdistance, trefresh, maxsize, minevts);
        return cornertrackingcallback->open(moduleName, strict);
    }
    else {
        cornertrackingcallback = 0;
        cornertrackingthread = new vCornerTrackingThread(height, width, moduleName, strict, mindistance, maxdistance, trefresh, maxsize, minevts);
        if(!cornertrackingthread->start())
            return false;
    }

    return true;

}

/**********************************************************/
bool vCornerTrackingModule::interruptModule()
{
    if(cornertrackingcallback) cornertrackingcallback->interrupt();
    if(cornertrackingthread) cornertrackingthread->stop();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vCornerTrackingModule::close()
{
    if(cornertrackingcallback) {
        cornertrackingcallback->close();
        delete cornertrackingcallback;
    }
    if(cornertrackingthread) delete cornertrackingthread;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vCornerTrackingModule::updateModule()
{
    return true;
}

/**********************************************************/
double vCornerTrackingModule::getPeriod()
{
    return 1;
}

/**********************************************************/

//empty line to make gcc happy
