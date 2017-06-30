#include "vParticle.h"
#include <cmath>
#include <limits>
#include <algorithm>

using ev::event;
using ev::AddressEvent;

double generateGaussianNoise(double mu, double sigma)
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do
     {
       u1 = rand() * (1.0 / RAND_MAX);
       u2 = rand() * (1.0 / RAND_MAX);
     }
    while ( u1 <= epsilon );

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, ev::vQueue &q, int currenttime, double tw, bool flip) {

    if(q.empty()) return;

    for(unsigned int i = 0; i < q.size(); i++) {
        if(tw) {
            double dt = currenttime - q[i]->stamp;
            if(dt < 0) dt += ev::vtsHelper::max_stamp;
            if(dt > tw) break;
        }

        auto v = is_event<AE>(q[i]);
        if(flip)
            image(image.width() - 1 - v->x, image.height() - 1 - v->y) = yarp::sig::PixelBgr(0, 255, 0);
        else
            image(v->x, v->y) = yarp::sig::PixelBgr(0, 255, 0);

    }
}

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy, int cr, int id)
{

    for(int y = -cr; y <= cr; y++) {
        for(int x = -cr; x <= cr; x++) {
            if(fabs(sqrt(pow(x, 2.0) + pow(y, 2.0)) - (double)cr) > 0.8) continue;
            int px = cx + x; int py = cy + y;
            if(py < 0 || py > image.height()-1 || px < 0 || px > image.width()-1) continue;
            switch(id) {
            case(0): //green
                image(px, py) = yarp::sig::PixelBgr(0, 255, 0);
                break;
            case(1): //blue
                image(px, py) = yarp::sig::PixelBgr(0, 0, 255);
                break;
            case(2): //red
                image(px, py) = yarp::sig::PixelBgr(255, 0, 0);
                break;
            default:
                image(px, py) = yarp::sig::PixelBgr(255, 255, 0);
                break;

            }

        }
    }

}

void drawDistribution(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, std::vector<vParticle> &indexedlist)
{

    double sum = 0;
    std::vector<double> weights;
    for(unsigned int i = 0; i < indexedlist.size(); i++) {
        weights.push_back(indexedlist[i].getw());
        sum += weights.back();
    }

    std::sort(weights.begin(), weights.end());


    image.resize(indexedlist.size(), 100);
    image.zero();
    for(unsigned int i = 0; i < weights.size(); i++) {
        image(weights.size() - 1 -  i, 99 - weights[i]*100) = yarp::sig::PixelBgr(255, 255, 255);
    }
}

double approxatan2(double y, double x) {

    double ax = std::abs(x); double ay = std::abs(y);
    double a = std::min (ax, ay) / std::max (ax, ay);
    //double s = pow(a, 2.0);
    //double r = ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a;

    double r = a * (M_PI_4 - (a - 1) * 0.273318560862312);

    if(ay > ax)
        r = 1.57079637 - r;

    if(x < 0)
        r = 3.14159274 - r;
    if(y < 0)
        r = -r;

    return r;

}

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLETRACKER
/*////////////////////////////////////////////////////////////////////////////*/

vParticle::vParticle()
{
    weight = 1.0;
    likelihood = 1.0;
    predlike = 1.0;
    minlikelihood = 20.0;
    inlierParameter = 1.5;
    outlierParameter = 3.0;
    variance = 0.5;
    stamp = 0;
    tw = 0;
    inlierCount = 0;
    maxtw = 0;
    outlierCount = 0;
    pcb = 0;
    angbuckets = 128;
    angdist.resize(angbuckets);
    negdist.resize(angbuckets);
    nupdates = 0;
}

void vParticle::initialiseParameters(int id, double minLikelihood,
                                     double outlierParam, double inlierParam,
                                     double variance, int angbuckets)
{
    this->id = id;
    this->minlikelihood = minLikelihood;
    this->outlierParameter = outlierParam;
    this->inlierParameter = inlierParam;
    this->variance = variance;
    this->angbuckets = angbuckets;
    angdist.resize(angbuckets, 0.0);
    negdist.resize(angbuckets, 0.0);
    likelihood = 1.0;
}

vParticle& vParticle::operator=(const vParticle &rhs)
{
    this->x = rhs.x;
    this->y = rhs.y;
    this->r = rhs.r;
    this->weight = rhs.weight;
    //this->angdist = rhs.angdist;
    //negdist.resize(angbuckets, 0.0);
    double lt = rhs.likelihood / angbuckets;
    angdist.resize(angbuckets, lt);
    //for(unsigned int i = 0; i < angbuckets; i++)
    //    angdist[i] = rhs.likelihood / angbuckets;

    //this->angdist.zero();
    //this->negdist = rhs.negdist;
    //this->likelihood = rhs.likelihood;

    return *this;
}

void vParticle::initialiseState(double x, double y, double r)
{
    this->x = x;
    this->y = y;
    this->r = r;

    for(int i = 0; i < angbuckets; i++) {
        angdist[i] = 0;
        negdist[i] = 0;
    }
    likelihood = 1.0;
    predlike = 1.0;
    //angdist.resize(angbuckets, 0.0);
    //negdist.resize(angbuckets, 1.0);
}

void vParticle::randomise(int x, int y, int r)
{
    initialiseState(rand()%x, rand()%y, rand()%r);

    //angdist.zero();
    //negdist.resize(angbuckets, 1.0);
}

void vParticle::resetWeight(double value)
{
    this->weight = weight;
}

void vParticle::resetRadius(double value)
{
    this->r = value;
}

void vParticle::predict(double sigma)
{
    if(!sigma) sigma = variance;
    double gx = generateGaussianNoise(0, sigma);
    double gy = generateGaussianNoise(0, sigma);
    double gr = generateGaussianNoise(0, sigma);

//    double inSigma2 = -0.5  / (sigma * sigma);
//    double px = exp(gx * gx * inSigma2);
//    double py = exp(gy * gy * inSigma2);
//    double pr = exp(gr * gr * inSigma2);
//    predlike = px * py * pr;

    //predlike = fabs(gx) + fabx(gy) + fabs(gr);
    predlike = exp((gx *gx + gy*gy + gr*gr) * -0.5 * 0.005);
    //predlike = 1.0 - sqrt(gx*gx + gy*gy + gr*gr) / 304;

    x += gx;
    y += gy;
    r += gr;

//    x = generateGaussianNoise(x, variance);
//    y = generateGaussianNoise(y, variance);
//    r = generateGaussianNoise(r, variance * 0.4);
}

void vParticle::concludeLikelihood()
{
//    double dtavg = 0;
//    double dtvar = 0;
//    int n = 0;

//    for(unsigned int i = 0; i < angdist.size(); i++) {
//        if(angdist[i] == 0 || angdist[i] > maxtw) continue;
//        dtavg += angdist[i];
//        n++;
//    }
//    if(n > minlikelihood) {
//        dtavg /= n;
//        for(unsigned int i = 0; i < angdist.size(); i++) {
//            if(angdist[i] == 0 || angdist[i] > maxtw) continue;
//            dtvar += (dtavg - angdist[i]) * (dtavg - angdist[i]);
//        }
//        dtvar /= n;
//        dtvar = 0.000001 + 1.0 / sqrt(dtvar * 2.0 * M_PI);
//    } else {
//        dtvar = 0.000001;
//    }

    nupdates = 0;
    likelihood = 0;
    for(int i = 0; i < angbuckets; i++) {
        likelihood += angdist[i];
        negdist[i] = 0;
    }

    if(likelihood > minlikelihood)
        weight = likelihood * weight;// * dtvar;// * predlike;
    else
        weight = minlikelihood * weight;


    for(int i = 0; i < angbuckets; i++) {
        angdist[i] *= 0.99;
    }
    //negdist.resize(angbuckets, 0.0);




}

void vParticle::updateWeightSync(double normval)
{
    weight = weight / normval;
}



