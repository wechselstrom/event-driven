#include "vControlLoopDelay.h"

/*////////////////////////////////////////////////////////////////////////////*/
// DELAYCONTROL
/*////////////////////////////////////////////////////////////////////////////*/

void delayControl::initFilter(int width, int height, int nparticles, int bins,
                              bool adaptive, int nthreads, double minlikelihood,
                              double inlierThresh, double randoms)
{
    vpf.initialise(width, height, nparticles, bins, adaptive, nthreads,
                   minlikelihood, inlierThresh, randoms);
    res.height = height;
    res.width = width;
}

void delayControl::initDelayControl(double gain, int maxtoproc, int positiveThreshold)
{
    this->gain = gain;
    this->minEvents = 1.0;
    qROI.setSize(maxtoproc);
    this->detectionThreshold = positiveThreshold;
}

bool delayControl::open(std::string name)
{
    if(!inputPort.open(name + "/vBottle:i"))
        return false;
    if(!outputPort.open(name + "/vBottle:o"))
        return false;
    if(!scopePort.open(name + "/scope:o"))
        return false;
    if(!debugPort.open(name + "/debug:o"))
        return false;

    return true;
}

void delayControl::onStop()
{
    inputPort.close();
    outputPort.close();
    scopePort.close();
    debugPort.close();
    inputPort.releaseDataLock();
}

void delayControl::run()
{

    double Tresample = 0;
    double Tpredict = 0;
    double Tlikelihood = 0;
    double Tgetwindow = 0;

    unsigned int targetproc = 0;
    unsigned int i = 0;
    yarp::os::Stamp ystamp;
    double stagnantstart = 0;
    bool detection = false;
    int channel;

    //START HERE!!
    ev::vQueue *q = 0;
    while(!q && !isStopping()) {
        q = inputPort.getNextQ(ystamp);
    }
    if(isStopping()) return;

    channel = q->front()->getChannel();

    while(true) {

        //calculate error
        unsigned int delay = inputPort.queryDelayN();
        targetproc = minEvents + (int)(delay * gain);

        //update the ROI with enough events
        Tgetwindow = yarp::os::Time::now();
        unsigned int addEvents = 0;
        while(addEvents < targetproc) {

            //if we ran out of events get a new queue
            if(i >= q->size()) {
                //if(inputPort.queryunprocessed() < 3) break;
                inputPort.scrapQ();
                q = 0; i = 0;
                while(!q && !isStopping()) {
                    q = inputPort.getNextQ(ystamp);
                }
                if(isStopping()) return;
            }

            auto v = is_event<AE>((*q)[i]);
            addEvents += qROI.add(v);
            i++;
        }
        Tgetwindow = yarp::os::Time::now() - Tgetwindow;

        //get the current time
        int currentstamp = 0;
        if(i >= q->size())
            currentstamp = (*q)[i-1]->stamp;
        else
            currentstamp = (*q)[i]->stamp;

        //do our update!!
        //yarp::os::Time::delay(0.005);
        Tlikelihood = yarp::os::Time::now();
        vpf.performObservation(qROI.q);
        Tlikelihood = yarp::os::Time::now() - Tlikelihood;
        vpf.extractTargetPosition(avgx, avgy, avgr);
        double roisize = avgr * 1.4;
        qROI.setROI(avgx - roisize, avgx + roisize, avgy - roisize, avgy + roisize);
        qROI.setSize(avgr * 4.0 * M_PI);

        Tresample = yarp::os::Time::now();
        vpf.performResample();
        Tresample = yarp::os::Time::now() - Tresample;

        Tpredict = yarp::os::Time::now();
        vpf.performPrediction(std::max(addEvents / (2.0 * avgr), 0.7));
        Tpredict = yarp::os::Time::now() - Tpredict;

        //check for stagnancy
        if(vpf.maxlikelihood < detectionThreshold) {

            if(!stagnantstart) {
                stagnantstart = yarp::os::Time::now();
            } else {
                if(yarp::os::Time::now() - stagnantstart > 1.0) {
                    vpf.resetToSeed();
                    detection = false;
                    stagnantstart = 0;
                    yInfo() << "Performing full resample";
                }
            }
        } else {
            detection = true;
            stagnantstart = 0;
        }


        //output our event
        if(outputPort.getOutputCount()) {
            auto ceg = make_event<GaussianAE>();
            ceg->stamp = currentstamp;
            ceg->setChannel(channel);
            ceg->x = avgx;
            ceg->y = avgy;
            ceg->sigx = avgr;
            ceg->sigy = avgr;
            ceg->sigxy = 1.0;
            if(vpf.maxlikelihood > detectionThreshold)
                ceg->polarity = 1.0;
            else
                ceg->polarity = 0.0;

            vBottle &outputbottle = outputPort.prepare();
            outputbottle.clear();
            outputbottle.addEvent(ceg);
            outputPort.write();

        }

        //write to our scope
        static double pscopetime = yarp::os::Time::now();
        if(scopePort.getOutputCount()) {

            static double val1 = -ev::vtsHelper::max_stamp;
            static double val2 = -ev::vtsHelper::max_stamp;
            static double val3 = -ev::vtsHelper::max_stamp;
            static double val4 = -ev::vtsHelper::max_stamp;
            static double val5 = -ev::vtsHelper::max_stamp;

            val1 = std::max(val1, (double)targetproc);
            val2 = std::max(val2, (double)inputPort.queryDelayN());
            val3 = std::max(val3, 0.0);
            val4 = std::max(val4, 0.0);
            val5 = std::max(val5, 0.0);

            double scopedt = yarp::os::Time::now() - pscopetime;
            if((scopedt > 0.05 || scopedt < 0)) {
                pscopetime += scopedt;

                yarp::os::Bottle &scopedata = scopePort.prepare();
                scopedata.clear();
                scopedata.addDouble(val1);
                scopedata.addDouble(val2);
                scopedata.addDouble(val3);
                scopedata.addDouble(val4);
                scopedata.addDouble(val5);

                val1 = -ev::vtsHelper::max_stamp;
                val2 = -ev::vtsHelper::max_stamp;
                val3 = -ev::vtsHelper::max_stamp;
                val4 = -ev::vtsHelper::max_stamp;
                val5 = -ev::vtsHelper::max_stamp;

                scopePort.write();
            }
        }

        //output a debug image
        static double pimagetime = yarp::os::Time::now();
        double pimagetimedt = yarp::os::Time::now() - pimagetime;
        if(debugPort.getOutputCount() && pimagetimedt > 0.04) {
            pimagetime += pimagetimedt;

            yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugPort.prepare();
            image.resize(res.width, res.height);
            image.zero();


            int px1 = avgx - roisize; if(px1 < 0) px1 = 0;
            int px2 = avgx + roisize; if(px2 >= res.width) px2 = res.width-1;
            int py1 = avgy - roisize; if(py1 < 0) py1 = 0;
            int py2 = avgy + roisize; if(py2 >= res.height) py2 = res.height-1;

            for(int x = px1; x <= px2; x+=2) {
                image(x, py1) = yarp::sig::PixelBgr(255, 255, 255);
                image(x, py2) = yarp::sig::PixelBgr(255, 255, 255);
            }
            for(int y = py1; y <= py2; y+=2) {
                image(px1, y) = yarp::sig::PixelBgr(255, 255, 255);
                image(px2, y) = yarp::sig::PixelBgr(255, 255, 255);
            }

            std::vector<vParticle> indexedlist = vpf.getps();

            for(unsigned int i = 0; i < indexedlist.size(); i++) {

                int py = indexedlist[i].gety();
                int px = indexedlist[i].getx();

                if(py < 0 || py >= res.height || px < 0 || px >= res.width) continue;
                image(px, py) = yarp::sig::PixelBgr(255, 255, 255);

            }
            drawEvents(image, qROI.q, currentstamp, 0, false);

            //drawcircle(image, avgx, avgy, avgr+0.5, 1);

            debugPort.write();
        }

    }

}

/*////////////////////////////////////////////////////////////////////////////*/
// ROIQ
/*////////////////////////////////////////////////////////////////////////////*/


roiq::roiq()
{
    roi.resize(4);
    n = 1000;
    roi[0] = 0; roi[1] = 1000;
    roi[2] = 0; roi[3] = 1000;
    use_TW = false;
}

void roiq::setSize(unsigned int value)
{
    //if TW n is in clock-ticks
    //otherwise n is in # events.
    n = value;
}

void roiq::setROI(int xl, int xh, int yl, int yh)
{
    roi[0] = xl; roi[1] = xh;
    roi[2] = yl; roi[3] = yh;
}

int roiq::add(event<AE> &v)
{

    if(v->x < roi[0] || v->x > roi[1] || v->y < roi[2] || v->y > roi[3])
        return 0;
    q.push_back(v);
    if(!use_TW) {
        if(q.size() > n)
            q.pop_front();
    } else {

        int dt = v->stamp - q.front()->stamp;
        if(dt < 0) dt += vtsHelper::max_stamp;
        while(dt > n) {
            q.pop_front();
            dt = v->stamp - q.front()->stamp;
            if(dt < 0) dt += vtsHelper::max_stamp;
        }
    }

    return 1;
}
