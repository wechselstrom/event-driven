/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

#ifndef __VCODEC__
#define __VCODEC__

#include <yarp/os/Bottle.h>
#include <memory>
#include <deque>
#include <math.h>

namespace ev {

//macros
class vEvent;

template<typename V = vEvent> using event = std::shared_ptr<V>;
template<typename V1, typename V2> event<V1> as_event(event<V2> orig_event) {
    return std::dynamic_pointer_cast<V1>(orig_event);
}
template<typename V1, typename V2> event<V1> is_event(event<V2> orig_event) {
    return std::static_pointer_cast<V1>(orig_event);
}
template<typename V> event<V> make_event(void) {
    return std::make_shared<V>();
}
template<typename V1, typename V2> event<V1> make_event(event<V2> orig_event) {
    return std::make_shared<V1>(*(orig_event.get()));
}
using vQueue = std::deque< event<vEvent> >;

void qsort(vQueue &q, bool respectWraps = false);
event<> createEvent(const std::string &type);

enum { VLEFT = 0, VRIGHT = 1 } ;

//event declarations
/// \brief base event class which defines the time the event occurs
class vEvent
{
public:
    static const std::string tag;
    unsigned int stamp:31;

    vEvent();
    virtual ~vEvent();

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel();
};

/// \brief an event with a pixel location, camera number and polarity
class AddressEvent : public vEvent
{
public:
    static const std::string tag;
    unsigned int x:10;
    unsigned int y:10;
    unsigned int channel:1;
    unsigned int polarity:1;

    AddressEvent();
    AddressEvent(const vEvent &v);
    AddressEvent(const AddressEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel(const int channel);
};
using AE = AddressEvent;

/// \brief an AddressEvent with a velocity in visual space
class FlowEvent : public AddressEvent
{
public:
    static const std::string tag;
    float vx;
    float vy;

    FlowEvent();
    FlowEvent(const vEvent &v);
    FlowEvent(const FlowEvent &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;

    int getDeath() const;
};

/// \brief an AddressEvent with an ID or class label
class LabelledAE : public AddressEvent
{
public:
    static const std::string tag;
    int ID;

    LabelledAE();
    LabelledAE(const vEvent &v);
    LabelledAE(const LabelledAE &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

/// \brief a LabelledAE with parameters that define a 2D gaussian
class GaussianAE : public LabelledAE
{
public:
    static const std::string tag;
    float sigx;
    float sigy;
    float sigxy;

    GaussianAE();
    GaussianAE(const vEvent &v);
    GaussianAE(const GaussianAE &v);

    virtual event<> clone();
    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

}

#endif


