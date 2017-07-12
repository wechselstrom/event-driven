/* event_driven.i */
%module event_driven
%{
#define SWIG_FILE_WITH_INIT
#include "iCub/eventdriven/vCodec.h"
#include "iCub/eventdriven/vBottle.h"
#include <yarp/os/Bottle.h>
#include <deque>
#include <vector>
#include <memory>
%}

%include "numpy.i"

%init %{
import_array();
%}

%apply (unsigned int* ARGOUT_ARRAY1, int DIM1) {(unsigned int *r1, int n1)};
%apply (unsigned int* ARGOUT_ARRAY1, int DIM1) {(unsigned int *r2, int n2)};
%apply (unsigned int* ARGOUT_ARRAY1, int DIM1) {(unsigned int *r3, int n3)};
%apply (unsigned int* ARGOUT_ARRAY1, int DIM1) {(unsigned int *r4, int n4)};
%apply (unsigned int* ARGOUT_ARRAY1, int DIM1) {(unsigned int *r5, int n5)};
%apply (unsigned int* IN_ARRAY2, int DIM1, int DIM2) {(unsigned int* data, int n, m)};

namespace ev {


class vEvent
{
public:
    static const std::string tag;
    vEvent();
    virtual ~vEvent();

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel();
};


class AddressEvent : public vEvent
{
public:
    AddressEvent();
    AddressEvent(const vEvent &v);
    AddressEvent(const AddressEvent &v);

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
    virtual int getChannel() const;
    virtual void setChannel(const int channel);
};
class FlowEvent : public AddressEvent
{
public:
    static const std::string tag;
    float vx;
    float vy;

    FlowEvent();
    FlowEvent(const vEvent &v);
    FlowEvent(const FlowEvent &v);

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;

    int getDeath() const;
};

class LabelledAE : public AddressEvent
{
public:
    static const std::string tag;
    int ID;

    LabelledAE();
    LabelledAE(const vEvent &v);
    LabelledAE(const LabelledAE &v);

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

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

    virtual void encode(yarp::os::Bottle &b) const;
    virtual bool decode(const yarp::os::Bottle &packet, int &pos);
    virtual yarp::os::Property getContent() const;
    virtual std::string getType() const;
};

class vBottle : public yarp::os::Bottle {

public:

    //constructors should not change from Bottle
    vBottle();

    //you can only modify contents by adding events and append other vBottles
    void append(vBottle &eb);

    template<class T> void append(vBottle &eb);

    //inherited
    void clear();

    bool read(yarp::os::ConnectionReader& reader);





};


%extend vBottle {
    int getSize() {
        auto q = $self->get<ev::vEvent>();
    	return q.size();
    }

    void getData(unsigned int *r1, int n1, unsigned int *r2, int n2,
		 unsigned int *r3, int n3, unsigned int *r4, int n4,
                 unsigned int *r5, int n5)
    {
        auto q = $self->get<ev::vEvent>();
        if ((unsigned int)n1!= q.size()) {
                std::cerr << "The size of the provided array does not match the expected size" << std::endl;
        	return;
        }
        unsigned int i;
	for (i=0; i< q.size(); i++) {
	    ev::vEvent *ev1 = &(*q.at(i));
	    ev::AddressEvent *m = dynamic_cast<ev::AddressEvent*>(ev1);
	    r1[i] = m->channel;
	    r2[i] = m->stamp;
	    r3[i] = m->x;
	    r4[i] = m->y;
	    r5[i] = m->polarity;
	}
    }

    void setData(unsigned int **data, int n, int m)
    {
        auto q = $self->get<ev::vEvent>();
        if ((unsigned int) n != q.size()) {
                std::cerr << "The length of the provided matrix does not match\
                              the number of events in a bottle (typically 512)" << std::endl;
        	return;
        }
        if ((unsigned int) m != 5) {
                std::cerr << "The provided matrix must be of shape (N, 5)" << std::endl;
        	return;
        }
        unsigned int i;
	for (i=0; i< q.size(); i++) {
	    ev::vEvent *ev1 = &(*q.at(i));
	    ev::AddressEvent *m = dynamic_cast<ev::AddressEvent*>(ev1);
	    m->channel  = data[i][0];
	    m->stamp    = data[i][1];
	    m->x        = data[i][2];
	    m->y        = data[i][3];
	    m->polarity = data[i][4];
        }
    }

}

}

