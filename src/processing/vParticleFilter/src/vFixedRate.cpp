#include "vFixedRate.h"

using namespace ev;

/*////////////////////////////////////////////////////////////////////////////*/
//particle reader (callback)
/*////////////////////////////////////////////////////////////////////////////*/
vParticleReader::vParticleReader()
{

    strict = false;
    pmax.resetWeight(0.0);
    srand(yarp::os::Time::now());

    avgx = 60;
    avgy = 60;
    avgr = 20;
    nparticles = 50;
    pwsum = 1.0;
    pwsumsq = nparticles * pow(1.0 / nparticles, 2.0);
    rate = 1000;
    seedx = 0; seedy = 0; seedr = 0;

    obsThresh = 30.0;
    obsInlier = 1.5;
    obsOutlier = 3.0;

    rbound_max = 50;
    rbound_min = 10;
    updatedvs = 0;
    vpstamp = 0;

    resTy = 0;
    predTy = 0;
    obsTy = 0;
    obsTv = 0;

}

void vParticleReader::initialise(unsigned int width , unsigned int height,
                                 unsigned int nParticles, unsigned int rate,
                                 double nRands, bool adaptive, double pVariance, int camera, bool useROI)
{
    //parameters
    res.width = width;
    res.height = height;

    this->camera = camera;
    this->useroi = useROI;

    rbound_min = res.width/17;
    rbound_max = res.width/8;

    avgx = res.width / 2.0;
    avgy = res.height / 2.0;
    avgr = (rbound_min + rbound_max) / 2.0;

    nparticles = nParticles;
    this->nRandomise = 1.0 + nRands;
    this->adaptive = adaptive;
    this->rate = rate;

    pwsumsq = nparticles * pow(1.0 / nparticles, 2.0);

    //initialise the particles
    std::cout << "Initialising Particles: " << nparticles << std::endl;
    vParticle p;

    indexedlist.clear();
    for(int i = 0; i < nparticles; i++) {
        p.initialiseParameters(i, obsThresh, obsOutlier, obsInlier, pVariance, 64);

        if(seedr)
            p.initialiseState(seedx, seedy, seedr);
        else
            p.initialiseState(res.width/2.0, res.height/2.0, (rbound_max + rbound_min)/2.0);

        p.resetWeight(1.0/nparticles);

        indexedlist.push_back(p);
    }



}

/******************************************************************************/
bool vParticleReader::open(const std::string &name, bool strictness)
{
    if(strictness) {
        this->strict = true;
        std::cout << "Setting " << name << " to strict" << std::endl;
        this->setStrict();
    }

    this->useCallback();
    if(!yarp::os::BufferedPort<ev::vBottle>::open(name + "/vBottle:i"))
        return false;
    if(!scopeOut.open(name + "/scope:o"))
        return false;
    if(!debugOut.open(name + "/debug:o"))
        return false;
    if(!resultOut.open(name + "/result:o"))
        return false;
    if(!vBottleOut.open(name + "/vBottle:o"))
        return false;

    return true;
}

/******************************************************************************/
void vParticleReader::close()
{
    //close ports
    scopeOut.close();
    debugOut.close();
    vBottleOut.close();
    resultOut.close();
    yarp::os::BufferedPort<ev::vBottle>::close();

}

/******************************************************************************/
void vParticleReader::interrupt()
{
    //pass on the interrupt call to everything needed
    scopeOut.interrupt();
    debugOut.interrupt();
    vBottleOut.interrupt();
    resultOut.interrupt();
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
}

bool vParticleReader::inbounds(vParticle &p)
{
    int r = p.getr();

    if(r < rbound_min) {
        p.resetRadius(rbound_min);
        r = rbound_min;
    }
    if(r > rbound_max) {
        p.resetRadius(rbound_max);
        r = rbound_max;
    }
    if(p.getx() < -r || p.getx() > res.width + r)
        return false;
    if(p.gety() < -r || p.gety() > res.height + r)
        return false;

    return true;
}


/******************************************************************************/
void vParticleReader::onRead(ev::vBottle &inputBottle)
{

    yarp::os::Stamp st;
    getEnvelope(st);
    if(st.getCount() != pstamp.getCount() +1) {
        std::cout << "Lost Bottle" << std::endl;
    }
    pstamp = st;

    //create event queue
   // tempT = yarp::os::Time::now();
    vQueue q = inputBottle.get<AE>();
   // obsTy += yarp::os::Time::now() - tempT;

    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        auto v = is_event<AE>(*qi);
        double vdt = v->stamp - vpstamp;
        if(vdt < 0) vdt += vtsHelper::max_stamp;
        obsTv += vdt;
        vpstamp = v->stamp;

        if((*qi)->getChannel() != camera) continue;

        //if(v->x < avgx - avgr *1.2 || v->x > avgx + avgr * 1.2 || v->y < avgy - avgr*1.2 || v->y > avgy + avgr*1.2) continue;

        tempT = yarp::os::Time::now();
        for(int i = 0; i < nparticles; i++)
            updatedvs = std::max(indexedlist[i].incrementalLikelihood(v->x, v->y), updatedvs);
        obsTy += yarp::os::Time::now() - tempT;

        if((double)updatedvs < 16) continue;


        tempT = yarp::os::Time::now();
        //NORMALISE
        double normval = 0.0;
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].concludeLikelihood(1.0 - updatedvs / 64.0);
            normval += indexedlist[i].getw();
        }
        for(int i = 0; i < nparticles; i++)
            indexedlist[i].updateWeightSync(normval);

        //FIND THE AVERAGE POSITION
        pwsum = 0;
        pwsumsq = 0;
        avgx = 0;
        avgy = 0;
        avgr = 0;

        for(int i = 0; i < nparticles; i ++) {
            double w = indexedlist[i].getw();
            pwsum += w;
            pwsumsq += pow(w, 2.0);
            avgx += indexedlist[i].getx() * w;
            avgy += indexedlist[i].gety() * w;
            avgr += indexedlist[i].getr() * w;

        }
        resTy += yarp::os::Time::now() - tempT;

        //RESAMPLE
        if(!adaptive || pwsumsq * nparticles > 2.0) {
            std::vector<vParticle> indexedSnap = indexedlist;
            for(int i = 0; i < nparticles; i++) {
                double rn = this->nRandomise * (double)rand() / RAND_MAX;
                if(rn > 1.0)
                    indexedlist[i].randomise(res.width, res.height, rbound_max);
                else {
                    double accum = 0.0; int j = 0;
                    for(j = 0; j < nparticles; j++) {
                        accum += indexedSnap[j].getw();
                        if(accum > rn) break;
                    }
                    indexedlist[i] = indexedSnap[j];
                }
            }
        }


        tempT = yarp::os::Time::now();
        //PREDICT
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].predict(updatedvs / 16.0);
            if(!inbounds(indexedlist[i])) {
                indexedlist[i].randomise(res.width, res.height, rbound_max);
            }
        }
        predTy += yarp::os::Time::now() - tempT;

        updatedvs = 0;
    }

    static int delay1 = 0;
    delay1++;
    if(scopeOut.getOutputCount() && delay1 > 5) {
        delay1 = 0;
        yarp::os::Bottle &sob = scopeOut.prepare();
        sob.clear();

        //double dt = q.back()->stamp - q.front()->stamp;
        //if(dt < 0) dt += ev::vtsHelper::max_stamp;
        sob.addDouble(obsTv * vtsHelper::tsscaler);
        sob.addDouble(obsTy);
        sob.addDouble(resTy);
        sob.addDouble(predTy);
        obsTv = 0;
        obsTy = 0;
        resTy = 0;
        predTy = 0;
        scopeOut.setEnvelope(st);
        scopeOut.write();
    }

    static int delay2 = 0;
    delay2++;
    if(vBottleOut.getOutputCount() && delay2 > 5) {
        delay2 = 0;
        ev::vBottle &eventsout = vBottleOut.prepare();
        eventsout.clear();
        auto ceg = make_event<GaussianAE>();
        ceg->stamp = q.back()->stamp;
        ceg->setChannel(camera);
        ceg->x = avgx;
        ceg->y = avgy;
        ceg->sigx = avgr;
        ceg->sigy = avgr;
        ceg->sigxy = 0;
        ceg->polarity = 1;
        //std::cout << ceg->getContent().toString() << std::endl;
        eventsout.addEvent(ceg);
        vBottleOut.setEnvelope(st);
        vBottleOut.write();

    }

    if(resultOut.getOutputCount()) {
        yarp::os::Bottle &trackBottle = resultOut.prepare();
        trackBottle.clear();
        resultOut.setEnvelope(st);
        //trackBottle.addInt(t);
        trackBottle.addDouble(avgx);
        trackBottle.addDouble(avgy);
        trackBottle.addDouble(avgr);
        resultOut.setEnvelope(st);
        resultOut.writeStrict();
    }


    if(debugOut.getOutputCount()) {
        yarp::sig::ImageOf< yarp::sig::PixelBgr> &image = debugOut.prepare();
        image.resize(res.width, res.height);
        image.zero();
        //stw = surfaceLeft.getSurf_Tlim(avgtw);

        for(unsigned int i = 0; i < indexedlist.size(); i++) {

            int py = indexedlist[i].gety();
            int px = indexedlist[i].getx();

            if(py < 0 || py >= res.height || px < 0 || px >= res.width) continue;

            //pcol = yarp::sig::PixelBgr(255*indexedlist[i].getw()/pmax.getw(), 255*indexedlist[i].getw()/pmax.getw(), 255);
            //image(res.width - px - 1, res.height - py - 1) = yarp::sig::PixelBgr(255, 255, 255);
            image(px, py) = yarp::sig::PixelBgr(255, 255, 255);
            //drawcircle(image, indexedlist[i].getx(), indexedlist[i].gety(), indexedlist[i].getr(), indexedlist[i].getid());
        }
        //drawEvents(image, stw, avgtw, false);
        //drawcircle(image, res.width - 1 - avgx, res.height - 1 - avgy, avgr, 1);
        drawcircle(image, avgx, avgy, avgr, 1);
        debugOut.setEnvelope(st);
        debugOut.write();
    }


}
