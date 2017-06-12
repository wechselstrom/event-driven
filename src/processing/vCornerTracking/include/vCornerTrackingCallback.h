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

#ifndef __VCORNERTRACKINGCALLBACK__
#define __VCORNERTRACKINGCALLBACK__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <fstream>
#include <math.h>
#include <clusterPool.h>

class vCornerTrackingCallback : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    bool strictness;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<ev::vBottle> outPort;

    ev::vtsHelper unwrapper;

    //parameters
    int height;
    int width;
    int mindistance;
    double maxdistance;
    unsigned int trefresh;
    int maxsize;
    int minevts;

    //set of clusters
    clusterPool *clusterSet;

public:

    vCornerTrackingCallback(int height, int width, int mindistance, double maxdistance, unsigned int trefresh, int maxsize, int minevts);

    bool    open(const std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();

    //this is the entry point to your main functionality
    void    onRead(ev::vBottle &bot);

};

#endif
//empty line to make gcc happy
