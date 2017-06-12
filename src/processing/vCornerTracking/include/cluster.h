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

#ifndef __CLUSTER__
#define __CLUSTER__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/eigen/Eigen.h>
#include <math.h>
#include <utility>
//#include <eigen3/Eigen/Dense>


class cluster {

private:

    ev::vQueue cluster_;

    //maximum size of the cluster
    int maxsize;

    //last time at which the cluster was updated
    double tlast_update;

    //matrix to check if the event has been already added
    std::vector< std::vector < int > > checkevt;

    //cluster velocity
    std::pair <double, double> vel;

public:

    cluster() {}

    void initialise(int maxsize);
    double dist2event(ev::event<ev::LabelledAE> evt);
    double getSpatialDist(ev::event<ev::LabelledAE> evt);
    void addEvent(ev::event<ev::LabelledAE> evt, double currt);
    ev::event<> getLastEvent();
    ev::event<> getFirstEvent();
    void fitLine();
    unsigned int getLastUpdate();
    bool isInTriangle(ev::event<ev::LabelledAE> evt, unsigned int currt);
    int getClusterSize();
    double getVx();
    double getVy();

//    ev::vQueue getCluster();

};


#endif
//empty line to make gcc happy
