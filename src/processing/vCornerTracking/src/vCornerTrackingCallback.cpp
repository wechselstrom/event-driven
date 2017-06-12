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

#include "vCornerTrackingCallback.h"

using namespace ev;

vCornerTrackingCallback::vCornerTrackingCallback(int height, int width, int mindistance, double maxdistance, unsigned int trefresh, int maxsize, int minevts)
{
    this->height = height;
    this->width = width;
    this->mindistance = mindistance;
    this->maxdistance = maxdistance;
    this->trefresh = trefresh;
    this->minevts = minevts;
    clusterSet = new clusterPool(mindistance, maxdistance, trefresh, maxsize, minevts);

}
/**********************************************************/
bool vCornerTrackingCallback::open(const std::string moduleName, bool strictness)
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

    return check1 && check2;

}

/**********************************************************/
void vCornerTrackingCallback::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

    delete clusterSet;

}

/**********************************************************/
void vCornerTrackingCallback::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

/**********************************************************/
void vCornerTrackingCallback::onRead(ev::vBottle &bot)
{
    /*prepare output vBottle*/
    ev::vBottle * outBottle = 0;

    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);

    std::pair <double, double> vel;

    /*get the event queue in the vBottle bot*/
    ev::vQueue q = bot.get<LabelledAE>();

    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        //current corner event
        auto cep = is_event<LabelledAE>(*qi);

        unsigned int currt = unwrapper(cep->stamp);

        //update cluster velocity
        vel = clusterSet->update(cep, currt);

        if(vel.first && vel.second) {
            //create new flow event and assign to it the velocity of the current cluster
            auto fe = make_event<FlowEvent>(cep);
            fe->vx = vel.first;
            fe->vy = vel.second;

            if(!outBottle) {
                outBottle = &outPort.prepare();
                outBottle->clear();
            }
            outBottle->addEvent(fe);
        }
    }

    outPort.write(strictness);

}

/**********************************************************/

//empty line to make gcc happy
