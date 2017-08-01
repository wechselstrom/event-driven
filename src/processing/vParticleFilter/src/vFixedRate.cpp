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
    standard_weight = 1.0 / nparticles;
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
    this->standard_weight = 1.0 / nparticles;

    pwsumsq = nparticles * pow(1.0 / nparticles, 2.0);

    //initialise the particles
    std::cout << "Initialising Particles: " << nparticles << std::endl;
    vParticle p;

    indexedlist.clear();
    for(int i = 0; i < nparticles; i++) {
        p.initialiseParameters(i, obsThresh, obsOutlier, obsInlier, pVariance, templatebins);

        if(seedr)
            p.initialiseState(seedx, seedy, seedr);
        else
            p.initialiseState(res.width/2.0, res.height/2.0, (rbound_max + rbound_min)/2.0);

        p.resetWeight(standard_weight);

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
    r = 0;
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
    static double maxlikelihood = 0, avglikelihood = 0, max_weight = 0;

    for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++) {

        auto v = is_event<AE>(*qi);
        double vdt = v->stamp - vpstamp;
        if(vdt < 0) vdt += vtsHelper::max_stamp;
        obsTv += vdt;
        vpstamp = v->stamp;

        if((*qi)->getChannel() != camera) continue;

        //if(v->x < avgx - avgr *1.5 || v->x > avgx + avgr * 1.5 || v->y < avgy - avgr*1.5 || v->y > avgy + avgr*1.5) continue;

        tempT = yarp::os::Time::now();
        for(int i = 0; i < nparticles; i++)
            updatedvs = std::max(indexedlist[i].incrementalLikelihood(v->x, v->y), updatedvs);
        obsTy += yarp::os::Time::now() - tempT;

        if((double)updatedvs < templatebins * 2.0 / M_PI) continue;

        // ///////////////// //
        // PERFORM AN UPDATE //
        // ///////////////// //


        //NORMALISE
        tempT = yarp::os::Time::now();
        double decay = 0.99; //exp((double)updatedvs / -32.0);
        double normval = 0.0;
        for(int i = 0; i < nparticles; i++) {
            indexedlist[i].concludeLikelihood(decay);
            normval += indexedlist[i].getw();
        }
        for(int i = 0; i < nparticles; i++)
            indexedlist[i].updateWeightSync(normval);

        //FIND THE AVERAGE POSITION
        pwsum = 0; pwsumsq = 0;
        avgx = 0; avgy = 0; avgr = 0;
        maxlikelihood = 0; avglikelihood = 0; max_weight = 0;
        int max_index = 0;

        for(int i = 0; i < nparticles; i ++) {
            double w = indexedlist[i].getw();
            pwsum += w;
            pwsumsq += pow(w, 2.0);
            avgx += indexedlist[i].getx() * w;
            avgy += indexedlist[i].gety() * w;
            avgr += indexedlist[i].getr() * w;
            if(i && indexedlist[i].getl() > maxlikelihood) {
                maxlikelihood = indexedlist[i].getl();
                max_index = i;
            }
            if(indexedlist[i].getw() > max_weight) {
                max_weight = indexedlist[i].getw();
            }
            if(i)
                avglikelihood += indexedlist[i].getl();
        }
        avglikelihood /= (nparticles-1);

        static int cc = 100;
        if(++cc >= 100) {
            //indexedlist[max_index].printL();
            cc = 0;
        }

        //RESAMPLE
        if(!adaptive || pwsumsq * nparticles > 2.0) {
            std::vector<vParticle> indexedSnap = indexedlist;
            for(int i = 1; i < nparticles; i++) {
                double rn = this->nRandomise * (double)rand() / RAND_MAX;
                if(rn > 1.0)
                    indexedlist[i].initialiseState(res.width/2.0, res.height/2.0, (rbound_max + rbound_min)/2.0);
                else {
                    double accum = 0.0; int j = 0;
                    for(j = 0; j < nparticles; j++) {
                        accum += indexedSnap[j].getw();
                        if(accum > rn) break;
                    }
                    indexedlist[i] = indexedSnap[j];
                }
            }
            //indexedlist[0].resetWeight(standard_weight);
        }
        resTy += yarp::os::Time::now() - tempT;

        //PREDICT
        tempT = yarp::os::Time::now();
        indexedlist[0].predict(0.0);
        for(int i = 1; i < nparticles; i++) {
            indexedlist[i].predict(4.0 * (M_PI/2.0) * updatedvs / (double)templatebins);
            if(!inbounds(indexedlist[i])) {
                indexedlist[i].randomise(res.width, res.height, rbound_max);
            }
        }
        predTy += yarp::os::Time::now() - tempT;

        updatedvs = 0;
    }

    static int delay2 = 0;
    if(vBottleOut.getOutputCount() && ++delay2 >= 1) {
        delay2 = 0;
        ev::vBottle &eventsout = vBottleOut.prepare();
        eventsout.clear();
//        auto ceg = make_event<GaussianAE>();
//        ceg->stamp = q.back()->stamp;
//        ceg->setChannel(camera);
//        ceg->x = avgx;
//        ceg->y = avgy;
//        ceg->sigx = avgr;
//        ceg->sigy = avgr;
//        ceg->sigxy = obsInlier;
//        if(maxlikelihood > templatebins / 2.0)
//            ceg->polarity = 1;
//        else
//            ceg->polarity = 0;
//        //std::cout << ceg->getContent().toString() << std::endl;
//        eventsout.addEvent(ceg);
        for(int i = 0; i < nparticles; i++) {
            auto ceg = make_event<GaussianAE>();
            ceg->stamp = q.back()->stamp;
            ceg->setChannel(camera);
            ceg->x = indexedlist[i].getx();
            ceg->y = indexedlist[i].gety();
            ceg->sigx = indexedlist[i].getr();
            ceg->sigy = indexedlist[i].getr();
            ceg->sigxy = obsInlier;
            ceg->ID = i;
            if(indexedlist[i].getl() > templatebins * 0.75)
                ceg->polarity = 1;
            else
                ceg->polarity = 0;
            //std::cout << ceg->getContent().toString() << std::endl;
            eventsout.addEvent(ceg);
        }
        vBottleOut.setEnvelope(st);
        vBottleOut.write();

    }

    static int delay1 = 0;
    const static int delayc1 = 5;
    if(scopeOut.getOutputCount() && ++delay1 >= delayc1) {
        delay1 = 0;

        yarp::os::Bottle &sob = scopeOut.prepare();
        sob.clear();


        //sob.addDouble(obsTv * vtsHelper::tsscaler / delayc1); //actual time passed
        //sob.addDouble(obsTy / delayc1); //time taken for processing events
        //sob.addDouble(delayc1 / (resTy + predTy + obsTy)); // actual update rate
        //sob.addDouble(delayc1 / (obsTv * vtsHelper::tsscaler)); // required update rate

        sob.addDouble(indexedlist[0].getl());
        sob.addDouble(avglikelihood);
        sob.addDouble(indexedlist[0].getw());
        sob.addDouble(max_weight);


        obsTv = 0;
        obsTy = 0;
        resTy = 0;
        predTy = 0;
        scopeOut.setEnvelope(st);
        scopeOut.write();
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
