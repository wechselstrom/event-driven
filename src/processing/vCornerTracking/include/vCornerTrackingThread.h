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

/// \defgroup Modules Modules
/// \defgroup vCornerTracking vCornerTracking
/// \ingroup Modules
/// \brief tracks corner events fitting 3D lines to cluster of corner events

#ifndef __VCORNERTRACKINGTHREAD__
#define __VCORNERTRACKINGTHREAD__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <iCub/eventdriven/vtsHelper.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <clusterPool.h>


class vCornerTrackingThread : public yarp::os::Thread
{
private:

    //thread for queues of events
    ev::queueAllocator allocatorCallback;

    //thread for the output
    ev::collectorPort outthread;

    yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

    ev::vtsHelper unwrapper;

//    //synchronising value
//    double cpudelay;
//    int prevstamp;
//    double t1;
//    double t2;
    yarp::os::Stamp yarpstamp;


    //parameters
    unsigned int height;
    unsigned int width;
    std::string name;
    bool strict;
    double mindistance;
    double maxdistance;
    double trefresh;
    int maxsize;
    int minevts;

    //set of clusters
    clusterPool *clusterSet;

//    std::ofstream outfile;

public:

    vCornerTrackingThread(unsigned int height, unsigned int width, std::string name, bool strict,
                          double mindistance, double maxdistance, double trefresh, int maxsize, int minevts);
    bool threadInit();
    bool open(std::string portname);
    void onStop();
    void run();
//    void threadRelease();

};


#endif
//empty line to make gcc happy
