#ifndef __VPARTICLE__
#define __VPARTICLE__

#include <iCub/eventdriven/all.h>
#include <yarp/sig/all.h>

using namespace ev;

//TODO make template generation autonomous
yarp::sig::Matrix generateCircularTemplate( int radius, int thickness, int inflate );

enum ParticleType {
    Circle,
    Template
};

class vParticle;

void drawEvents(yarp::sig::ImageOf< yarp::sig::PixelBgr> &image, ev::vQueue &q, int currenttime, double tw = 0, bool flip = false);

void drawcircle(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, int cx, int cy, int cr, int id = 0);

void drawDistribution(yarp::sig::ImageOf<yarp::sig::PixelBgr> &image, std::vector<vParticle> &indexedlist);

class preComputedBins;

/*////////////////////////////////////////////////////////////////////////////*/
//VPARTICLETRACKER
/*////////////////////////////////////////////////////////////////////////////*/
class vParticle
{
protected:

    //static parameters
    int id;
    double minlikelihood;
    double inlierParameter;
    double outlierParameter;
    double variance;
    int angbuckets;
    preComputedBins *pcb;

    //temporary parameters (on update cycle)
    double likelihood;
    double predlike;
    int    outlierCount;
    int    inlierCount;
    double maxtw;
    yarp::sig::Vector angdist;
    yarp::sig::Vector negdist;

    //state and weight
    double x;
    double y;
    double r;
    double tw;
    double weight;

    //timing
    unsigned long int stamp;

public:


    vParticle();
    vParticle& operator=(const vParticle &rhs);
    virtual vParticle* clone () = 0; //Virtual constructor
    
    //initialise etc.
    void initialiseParameters(int id, double minLikelihood, double outlierParam, double inlierParam, double variance, int angbuckets);
    void attachPCB(preComputedBins *pcb) { this->pcb = pcb; }

    void initialiseState(double x, double y, double r, double tw);
    void randomise(int x, int y, int r, int tw);

    void resetStamp(unsigned long int value);
    void resetWeight(double value);
    void resetRadius(double value);


    //update
    void predict(unsigned long int stamp);

    virtual void initLikelihood() = 0;
    virtual int incrementalLikelihood(int vx, int vy, int dt) = 0;
    void concludeLikelihood();

    void updateWeightSync(double normval);

    //get
    int    getid() { return id; }
    double getx()  { return x; }
    double gety()  { return y; }
    double getr()  { return r; }
    double getw()  { return weight; }
    double getl()  { return likelihood; }
    double gettw() { return tw; }


};

class vParticleCircle : public vParticle
{
public:
    vParticleCircle() : vParticle(){};
    vParticle* clone() { return new vParticleCircle(*this);};
    
    void initLikelihood();
    int incrementalLikelihood(int vx, int vy, int dt);
};

class vParticleTemplate : public vParticle
{
private:
    yarp::sig::Matrix vTemplate;
    double score;
    
public:
    
    vParticleTemplate( yarp::sig::Matrix vTemplate)  : vParticle(), vTemplate(vTemplate){};
    vParticle* clone() { return new vParticleTemplate(*this);};
    
    void initLikelihood();
    int incrementalLikelihood(int vx, int vy, int dt);
};

class preComputedBins
{

private:

    yarp::sig::Matrix ds;
    yarp::sig::Matrix bs;
    int rows;
    int cols;
    int offsetx;
    int offsety;

public:

    preComputedBins()
    {
        rows = 0;
        cols = 0;
        offsetx = 0;
        offsety = 0;
    }

    void configure(int height, int width, double maxrad, int nBins)
    {
        rows = (height + maxrad) * 2 + 1;
        cols = (width + maxrad) * 2 + 1;
        offsety = rows/2;
        offsetx = cols/2;

        ds.resize(rows, cols);
        bs.resize(rows, cols);
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {

                int dy = i - offsety;
                int dx = j - offsetx;

                ds(i, j) = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
                bs(i, j) = (nBins-1) * (atan2(dy, dx) + M_PI) / (2.0 * M_PI);

            }
        }
    }

    double queryDistance(int dy, int dx)
    {
        dy += offsety; dx += offsetx;
//        if(dy < 0 || dy > rows || dx < 0 || dx > cols) {
//            std::cout << "preComputatedBins not large enough" << std::endl;
//            return 0.0;
//        }
        return ds(dy, dx);
    }

    int queryBinNumber(double dy, double dx)
    {
        dy += offsety; dx += offsetx;
//        if(dy < 0 || dy > rows || dx < 0 || dx > cols) {
//            std::cout << "preComputatedBins not large enough" << std::endl;
//            return 0.0;
//        }
        return (int)(bs(dy, dx) + 0.5);
    }



};


#endif
