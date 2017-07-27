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

#include "vCornerRTCallback.h"

using namespace ev;

vCornerCallback::vCornerCallback(int height, int width, int sobelsize, int windowRad, double temporalsize, double sigma, int qlen, double thresh)
{
    this->height = height;
    this->width = width;
    this->windowRad = windowRad;
    this->temporalsize = temporalsize / ev::vtsHelper::tsscaler;

    //ensure that sobel size is an odd number
    if(!(sobelsize % 2))
    {
        std::cout << "Warning: sobelsize should be odd" << std::endl;
        sobelsize--;
        std::cout << "sobelsize = " << sobelsize << " will be used" << std::endl;
    }

    this->qlen = qlen;
    this->thresh = thresh;

    this->cpudelay = 0.005;
    this->prevstamp = 0;
    this->t1 = this->t2 = 0;// = yarp::os::Time::now();

    int gaussiansize = 2*windowRad + 2 - sobelsize;
    convolution.configure(sobelsize, gaussiansize);
    convolution.setSobelFilters();
    convolution.setGaussianFilter(sigma);

    std::cout << "Using a " << sobelsize << "x" << sobelsize << " filter ";
    std::cout << "and a " << 2*windowRad + 1 << "x" << 2*windowRad + 1 << " spatial window" << std::endl;

    //create surface representations
    std::cout << "Creating surfaces..." << std::endl;
//    surfaceOfR.initialise(height, width);
//    surfaceOnR.initialise(height, width);
//    surfaceOfL.initialise(height, width);
//    surfaceOnL.initialise(height, width);
    surfaceOfR = new temporalSurface(width, height, this->temporalsize);
    surfaceOnR = new temporalSurface(width, height, this->temporalsize);
    surfaceOfL = new temporalSurface(width, height, this->temporalsize);
    surfaceOnL = new temporalSurface(width, height, this->temporalsize);

//    spfilter.initialise(width, height, 100000, 1);

}
/**********************************************************/
bool vCornerCallback::open(const std::string moduleName, bool strictness)
{
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<ev::vBottle>::open(inPortName);

    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);

    std::string debugPortName = "/" + moduleName + "/score:o";
    bool check3 = debugPort.open(debugPortName);

    outfile.open("/home/vvasco/dev/egomotion/affine/testing data/corners.txt");

    return check1 && check2 && check3;

}

/**********************************************************/
void vCornerCallback::close()
{
    //close ports
    debugPort.close();
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

    outfile.close();

    delete surfaceOfR;
    delete surfaceOnR;
    delete surfaceOfL;
    delete surfaceOnL;

}

/**********************************************************/
void vCornerCallback::interrupt()
{
    //pass on the interrupt call to everything needed
    debugPort.interrupt();
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();

}

/**********************************************************/
void vCornerCallback::onRead(ev::vBottle &bot)
{
    /*prepare output vBottle*/
//    ev::vBottle &outBottle = outPort.prepare();
//    outBottle.clear();
    yarp::os::Stamp st;
//    this->getEnvelope(st); outPort.setEnvelope(st);

    ev::vBottle fillerbottle;

    bool isc = false;

    /*get the event queue in the vBottle bot*/
    ev::vQueue q = bot.get<AE>();
    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        auto ae = is_event<AE>(*qi);
//        if(!spfilter.check(ae->x, ae->y, ae->polarity, ae->channel, ae->stamp))
//            continue;

//        double dt = ae->stamp - prevstamp;
//        if(dt < 0) dt += vtsHelper::max_stamp;
//        cpudelay -= dt * vtsHelper::tsscaler;
//        prevstamp = ae->stamp;

//        ev::historicalSurface *cSurf;
        ev::temporalSurface *cSurf;
        if(ae->getChannel()) {
//            continue;
            if(ae->polarity)
//                cSurf = &surfaceOfR;
                cSurf = surfaceOfR;
            else
//                cSurf = &surfaceOnR;
                cSurf = surfaceOnR;
        } else {
//            continue;
            if(ae->polarity)
//                cSurf = &surfaceOfL;
                cSurf = surfaceOfL;
            else
//                cSurf = &surfaceOnL;
                cSurf = surfaceOnL;
        }
//        cSurf->addEvent(*qi);
        cSurf->fastAddEvent(*qi);

//        if(cpudelay <= 0.0) {
//            if(t1 == 0.0)
//                t1 = yarp::os::Time::now();
//            t1 = yarp::os::Time::now();
            //get the roi and process
//            yarp::os::Time::delay(0.05);

            vQueue subsurf;
//            cSurf->getSurfaceN(subsurf, 0, qlen, windowRad, ae->x, ae->y);
            cSurf->getSurf(subsurf, windowRad);
            isc = detectcorner(subsurf, ae->x, ae->y);

            this->getEnvelope(st);
            outfile << ae->channel << " " << unwrapper(ae->stamp) << " " << ae->polarity << " "
                    << ae->x << " " << ae->y << " " << std::setprecision(15) << st.getTime() << " ";

            //if it's a corner, add it to the output bottle
            if(isc) {
                auto ce = make_event<LabelledAE>(ae);
                ce->ID = 1;
                fillerbottle.addEvent(ce);

                outfile << ce->ID << std::endl;
            }
            else
                outfile << 0 << std::endl;

//            //times it takes to process
////            cpudelay = 0.0;
//            cpudelay +=  yarp::os::Time::now() - t1;
//            t1 = yarp::os::Time::now();
//        }

        if(debugPort.getOutputCount()) {
            yarp::os::Bottle &scorebottleout = debugPort.prepare();
            scorebottleout.clear();
            scorebottleout.addDouble(cpudelay);
            debugPort.write();
        }

    }

    if( (yarp::os::Time::now() - t2) > 0.001 && fillerbottle.size() ) {
        yarp::os::Stamp st;
        this->getEnvelope(st);
        outPort.setEnvelope(st);
        ev::vBottle &eventsout = outPort.prepare();
        eventsout.clear();
        eventsout = fillerbottle;
        outPort.write(strictness);
        fillerbottle.clear();
        t2 = yarp::os::Time::now();
    }

}

/**********************************************************/
bool vCornerCallback::detectcorner(const vQueue subsurf, int x, int y)
{

    //set the final response to be centred on the curren event
    convolution.setResponseCenter(x, y);

    //update filter response
    for(unsigned int i = 0; i < subsurf.size(); i++)
    {
        //events are in the surface
        auto vi = is_event<AE>(subsurf[i]);
        convolution.applysobel(vi);

    }
    convolution.applygaussian();

    double score = convolution.getScore();

    //reset responses
    convolution.reset();

//    std::cout << score << std::endl;

    //if score > thresh tag ae as ce
    return score > thresh;

}

//empty line to make gcc happy

