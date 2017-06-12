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

#ifndef __CLUSTERPOOL__
#define __CLUSTERPOOL__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <math.h>
#include <utility>
#include <cluster.h>

class clusterPool {

private:

    //we store the pool of clusters in a vector
    std::vector <cluster> pool;

    //minimum number of events in the cluster to start tracking it
    int minevts;

    //minimum distance the cluster has to move before tracking it
    double mindistance;

    //maximum distance the cluster has to move before killing it
    double maxdistance;

    //time from the last update to kill the cluster
    double trefresh;

    //maximum size of the single cluster
    int maxsize;

    //check for the first event
    bool firstevent;

public:

    clusterPool(double mindistance, double maxdistance, double trefresh, int maxsize, int minevts);

    std::pair<double, double> update(ev::event<ev::LabelledAE> evt, double currt);
    std::pair<double, double> updateNew(ev::event<ev::LabelledAE> evt, double currt);
    void createNewCluster(ev::event<ev::LabelledAE> evt, double currt);
    void killOldCluster(int clusterID);
    std::vector <cluster> getPool();
//    cluster getCluster(int clusterID);

};

#endif
//empty line to make gcc happy
