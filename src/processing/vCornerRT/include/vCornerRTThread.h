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
/// \defgroup vCorner vCorner
/// \ingroup Modules
/// \brief detects corner events using the Harris method

#ifndef __VCORNERRTTHREAD__
#define __VCORNERRTTHREAD__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/eventdriven/all.h>
#include <filters.h>
#include <fstream>
#include <math.h>

class vComputeThread : public yarp::os::Thread
{
private:

    int sobelsize;
    int windowRad;
    double sigma;
    double thresh;
    unsigned int qlen;
    ev::vQueue patch;
    filters convolution;
    ev::event<ev::AddressEvent> aep;
    ev::collectorPort *outthread;
    yarp::os::Stamp ystamp;
//    yarp::os::Mutex processing;
//    yarp::os::Mutex dataready;

    bool detectcorner(int x, int y);

public:

    bool taskassigned;
    vComputeThread(int sobelsize, int windowRad, double sigma, double thresh, unsigned int qlen, ev::collectorPort *outthread);
    void setData(ev::temporalSurface *cSurf, yarp::os::Stamp ystamp);
//    void setData(ev::historicalSurface *cSurf, yarp::os::Stamp ystamp);
    void assignTask(ev::temporalSurface *cSurf, yarp::os::Stamp ystamp);
    ev::event<ev::LabelledAE> getResponse();
    bool threadInit() { return true; }
    void run();
    void threadRelease() {}
};

class vCornerThread : public yarp::os::Thread
{
private:

    //thread for queues of events
    ev::queueAllocator allocatorCallback;

    //data structures
//    ev::historicalSurface surfaceleft;
//    ev::historicalSurface surfaceright;
    ev::temporalSurface *surfaceleft;
    ev::temporalSurface *surfaceright;

    //output port for the vBottle with the new events computed by the module
    yarp::os::BufferedPort<ev::vBottle> vBottleOut;
    yarp::os::BufferedPort<yarp::os::Bottle> debugPort;

    //list of thread for processing
    std::vector<vComputeThread *> computeThreads;

    //thread for the output
    ev::collectorPort outthread;

    //synchronising value
    double cpudelay;
    int prevstamp;
    double t1;
    double t2;
    yarp::os::Stamp yarpstamp;
    int k;

    //parameters
    unsigned int height;
    unsigned int width;
    std::string name;
    bool strict;
    int qlen;
    int temporalsize;
    int windowRad;
    int sobelsize;
    double sigma;
    double thresh;
    int nthreads;

    filters convolution;
    bool detectcorner(ev::vQueue patch, int x, int y);

public:

    vCornerThread(unsigned int height, unsigned int width, std::string name, bool strict, int qlen,
                  double temporalsize, int windowRad, int sobelsize, double sigma, double thresh, int nthreads);
    bool threadInit();
    bool open(std::string portname);
    void onStop();
    void run();

};


#endif
//empty line to make gcc happy
