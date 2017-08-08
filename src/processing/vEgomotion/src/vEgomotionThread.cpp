/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email: valentina.vasco@iit.it
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

#include "vEgomotionThread.h"

using namespace ev;

/**********************************************************/
bool vEgomotionModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vEgomotion")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

//    bool strict = rf.check("strict") &&
//            rf.check("strict", yarp::os::Value(true)).asBool();

    bool train = rf.check("train", yarp::os::Value(false)).asBool();
    double thresh_mag = rf.check("thresh_mag", yarp::os::Value(2.5)).asDouble();
    double thresh_angle = rf.check("thresh_angle", yarp::os::Value(0.3)).asDouble();
    int nthreads = rf.check("nthreads", yarp::os::Value(4)).asInt();

    //create the thread for reading joint velocities
    encsobserver = new vEncObsThread(moduleName);
    if(!encsobserver->start()) {
        std::cout << "could not open encoders observer" << std::endl;
    }

    //create the thread for reading the events
    egomotionthread = new vEgomotionThread(moduleName, train, thresh_mag, thresh_angle, nthreads, encsobserver);
    if(!egomotionthread->start()) {
        std::cout << "could not open egomotion thread" << std::endl;
        return false;
    }

    return true;

}

/**********************************************************/
bool vEgomotionModule::interruptModule()
{
//    encsobserver->stop();
    egomotionthread->stop();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vEgomotionModule::close()
{
    delete egomotionthread;
//    delete encsobserver;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vEgomotionModule::updateModule()
{
    return true;
}

/**********************************************************/
double vEgomotionModule::getPeriod()
{
    return 1;
}

/**********************************************************/
vEgomotionThread::vEgomotionThread(std::string name, bool train, double thresh_mag, double thresh_angle, int nthreads, vEncObsThread *velobs)
{
    this->name = name;
    this->train = train;
    this->velobs = velobs;
    this->nthreads = nthreads;

//    std::string folder = "/home/vvasco/dev/libsvm-3.22/";
    std::string folder = "/usr/local/src/robot/event-driven/build/bin/";
    int njoints = 6;

    //create smv node that will be our features (encoders velocities)
//    encveltest = new svm_node[njoints];
    encveltest = (struct svm_node *) malloc((njoints+1)*sizeof(struct svm_node));

    //open range
    range_file.open(folder + "range.txt", std::ios_base::in);
    min_j.resize(njoints);
    max_j.resize(njoints);
    range_j.resize(njoints);
    range_file >> min_j[0] >> max_j[0] >> min_j[1] >> max_j[1] >> min_j[2] >> max_j[2]
            >> min_j[3] >> max_j[3] >> min_j[4] >> max_j[4] >> min_j[5] >> max_j[5];
    for(int i = 0; i < njoints; i ++)
        range_j[i] = max_j[i] - min_j[i];

//    indpSurf = new temporalSurface(304, 240, 0.1 / ev::vtsHelper::tsscaler);

//    if(train) {

//        yInfo("STARTING TRAINING...");
//        yInfo("Please move only the iCub robot ");
//        yInfo("Files will be saved in: " + folder);
//        mu_vx_training.open(folder + "test1.txt", std::ios_base::out);
//        mu_vy_training.open(folder + "test2.txt", std::ios_base::out);
//        sigma_vx_training.open(folder + "test3.txt", std::ios_base::out);
//        sigma_vy_training.open(folder + "test4.txt", std::ios_base::out);
//        sigma_vxvy_training.open(folder + "test5.txt", std::ios_base::out);

//    }
//    else {

        yInfo("STARTING TESTING...");
        yInfo("Models are loaded from: " + folder);

        //load the trained models
        mu_vx = svm_load_model(mu_vx_file);
        if(!mu_vx)
            yError("muvx model could not be loaded");
        else
            yInfo("muvx model successfully loaded");

        mu_vy = svm_load_model(mu_vy_file);
        if(!mu_vy)
            yError("muvy model could not be loaded");
        else
            yInfo("muvy model successfully loaded");

        sigma_vx = svm_load_model(sigma_vx_file);
        if(!sigma_vx)
            yError("sigmavx model could not be loaded");
        else
            yInfo("sigmavx model successfully loaded");

        sigma_vy = svm_load_model(sigma_vy_file);
        if(!sigma_vy)
            yError("sigmavy model could not be loaded");
        else
            yInfo("sigmavy model successfully loaded");

        sigma_vxvy = svm_load_model(sigma_vxvy_file);
        if(!sigma_vxvy)
            yError("sigmavxvy model could not be loaded");
        else
            yInfo("sigmavxvy model successfully loaded");

        this->thresh_mag = thresh_mag;
        this->thresh_angle = thresh_angle;

//    }

        for(int i = 0; i < nthreads; i ++) {
            predictThreads.push_back(new vPredictThread(&outthread, mu_vx, mu_vy));
            predictThreads[i]->start();
        }

        outfile.open(folder + "events_ind.txt", std::ios_base::out);
}

/**********************************************************/
bool vEgomotionThread::threadInit()
{
    yInfo("Starting thread for training/testing...");

    if(!allocatorCallback.open("/" + name + "/vBottle:i")) {
    std::cout << "could not open vBottleIn port " << std::endl;
        return false;
    }

//    if(strictness) outPort.setStrict();
    if(!outthread.open("/" + name + "/vBottle:o")) {
        std::cout << "could not open vBottleOut port" << std::endl;
        return false;
    }
    if(!outthread.start())
        return false;

//    std::string outPortName = "/" + name + "/vBottle:o";
//    if(!outPort.open(outPortName)) {
//        std::cout << "could not open vBottleOut port " << std::endl;
//        return false;
//    }

    std::string debugPortName = "/" + name + "/score:o";
    if(!debugPort.open(debugPortName)) {
        std::cout << "could not open debug port " << std::endl;
        return false;
    }

    return true;

}

/**********************************************************/
void vEgomotionThread::onStop()
{
    allocatorCallback.close();
    allocatorCallback.releaseDataLock();

     //close ports
//    outPort.close();
//    encPort.close();
    debugPort.close();

    //remember to deallocate the allocated memory
    delete encveltest;
//    delete indpSurf;

    for(int i = 0; i < nthreads; i++)
        delete predictThreads[i];

    if(train) {
        mu_vx_training.close();
        mu_vy_training.close();
        sigma_vx_training.close();
        sigma_vy_training.close();
        sigma_vxvy_training.close();
    }
    else {
        range_file.close();
    }
    outfile.close();

}

/**********************************************************/
void vEgomotionThread::run()
{

    while(true) {

        //get queue of flow events
        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = allocatorCallback.getNextQ(yarpstamp);

//            if( (yarp::os::Time::now() - tcm) > 0.01) {
//                cmx = 0.0;
//                cmy = 0.0;
////                count = 0;
            //                std::cout << indpSurf->getEventCount() << " " << cmx << " " << cmy << std::endl;
            //            }
        }
        if(isStopping()) break;

        //get joints velocities, scale them and put them in a svm structure
        yarp::sig::Vector encvels;
        velobs->getEncVels(encvels);

        bool notmoving = false;
        unsigned int countNotMoving = 0;
        for(unsigned int b = 0; b < encvels.size(); b++)
        {
            encveltest[b].index = b + 1;
            if(abs(encvels[b]) < pow(10, -15)) {
                countNotMoving++;
            }
            encveltest[b].value = (encvels[b] - min_j[b]) / range_j[b];
        }
        //feature is meant to terminate with -1
        encveltest[encvels.size()].index = -1;
        //        encveltest[encvels.size()].value = 0;

        if(countNotMoving == encvels.size()) notmoving = true;

        yarp::sig::Vector pred_meanv(2);
        yarp::sig::Matrix pred_covv(2, 2);
        float avgvx = 0.0;
        float avgvy = 0.0;
        double mah_dist;
        double cos_dist;
        bool isindependent;
        double weight;
//        double cmx;
//        double cmy;
//        double cmvx;
//        double cmvy;

        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            //get the flow event
            auto ofp = ev::is_event<ev::FlowEvent>(*qi);
            avgvx += ofp->vx;
            avgvy += ofp->vy;

            //            //testing
            //            if(!train) {

            //predict egomotion using current encoder velocities
            //and the learnt models
            if(notmoving == true) {
                isindependent = true;
//                indpSurf->fastAddEvent(*qi);

//                //if it is independent motion, tag the event as independent
//                auto inde = make_event<LabelledAE>(ofp);
//                if(isindependent)
//                    inde->ID = 2;
//                else
//                    //if not, tag it as corner
//                    inde->ID = 1;

//                outthread.pushevent(inde, yarpstamp);

            }
            else {

//                //let the hread predict the mean vel
//                int k = 0;
//                while(true) {

//                    //assign a task to a thread that is not managing a task
//                    if(predictThreads[k]->available()) {
//                        predictThreads[k]->assignTask(ofp, encveltest, &yarpstamp);
//                        break;
//                    }
//                    if(++k == nthreads)
//                        k = 0;
//                }
//            }

                pred_meanv = predict_mean(encveltest);
                //                pred_covv = predict_cov(encveltest);

                pred_covv(0, 0) = 1;
                pred_covv(1, 0) = 0;
                pred_covv(0, 1) = 0;
                pred_covv(1, 1) = 1;

////                //velocity distance from known independently moving corners
////                double dx = ofp->x - cmx;
////                double dy = ofp->y - cmy;
////                double dvx = ofp->vx - cmvx;
////                double dvy = ofp->vy - cmvy;
////                double dists = sqrt(dx*dx + dy*dy);
////                double distv = sqrt(dvx*dvx + dvy*dvy);
//////                std::cout << dists << " " << distv << std::endl;
////                double dt = ofp->stamp - indpSurf->getMostRecent()->stamp;
////                if(dt < 0)
////                    dt += vtsHelper::max_stamp;

////                //give more weight to corners close to known independently moving corners
////                if(distv < 1.0 && dt < 1000000 && indpSurf->getEventCount() > 5) {
////                    std::cout << "weighting more " << indpSurf->getEventCount() << " " << distv << " " << dt << " ";
////                    weight = 10;
////                }
////                else
                    weight = 1;

                //compute metric
                isindependent = detect_independent(ofp, pred_meanv, pred_covv, mah_dist, cos_dist, weight);

////                if(weight > 1)
////                    std::cout << mah_dist/weight << " " << mah_dist << std::endl;

                outfile << ofp->channel << " " << ofp->stamp << " " << ofp->polarity << " " << ofp->x << " " << ofp->y
                        << " " << ofp->vx << " " << ofp->vy << " " << pred_meanv[0] << " " << pred_meanv[1] << " "
                        << mah_dist << " " << cos_dist << " " << isindependent << std::endl;

            }

            //if it is independent motion, tag the event as independent
            auto inde = make_event<LabelledAE>(ofp);
            if(isindependent)
                inde->ID = 2;
            else
                //if not, tag it as corner
                inde->ID = 1;

            outthread.pushevent(inde, yarpstamp);

            if(debugPort.getOutputCount()) {
                yarp::os::Bottle &scorebottleout = debugPort.prepare();
                scorebottleout.clear();
                scorebottleout.addDouble(pred_meanv[0]);
                scorebottleout.addDouble(pred_meanv[1]);
                scorebottleout.addDouble(pred_covv(0, 0));
                scorebottleout.addDouble(pred_covv(1, 1));
                scorebottleout.addDouble(cos_dist);
                scorebottleout.addDouble(ofp->vx);
                scorebottleout.addDouble(ofp->vy);
                debugPort.write();
            }

        }

//        cmx = 0.0;
//        cmy = 0.0;
//        cmvx = 0.0;
//        cmvy = 0.0;
//        vQueue ind = indpSurf->getEverything();
//        for(unsigned int k = 0; k < ind.size(); k ++) {
//            auto vi = is_event<FlowEvent>(ind[k]);
//            cmx += vi->x;
//            cmy += vi->y;
//            cmvx += vi->vx;
//            cmvy += vi->vy;
//            indpSurf->fastRemoveEvents(ind[k]);
//        }
//        cmx = cmx / ind.size();
//        cmy = cmy / ind.size();
//        cmvx = cmvx / ind.size();
//        cmvy = cmvy / ind.size();

        allocatorCallback.scrapQ();

    }

}

/**********************************************************/
bool vEgomotionThread::detect_independent(event<FlowEvent> ofe, yarp::sig::Vector pred_meanv, yarp::sig::Matrix pred_covv,
                                          double &mahdist, double &cosdist, double weight)
{

    yarp::sig::Vector flowvel(2);
    yarp::sig::Vector diff(2);
    yarp::sig::Matrix invcov(2, 2);
    yarp::sig::Matrix a(1, 2);

    flowvel[0] = ofe->vx;
    flowvel[1] = ofe->vy;

    diff[0] = flowvel[0] - pred_meanv[0];
    diff[1] = flowvel[1] - pred_meanv[1];
    invcov = yarp::math::luinv(pred_covv);

    a(0, 0) = diff[0]*invcov(0, 0) + diff[1]*invcov(1, 0);
    a(0, 1) = diff[0]*invcov(0, 1) + diff[1]*invcov(1, 1);

    //compute mahalanobis distance
    mahdist = weight * sqrt(a(0, 0)*diff[0] + a(0, 1)*diff[1]);

    double modflow = sqrt(flowvel[0]*flowvel[0] + flowvel[1]*flowvel[1]);
    double modpredmu = sqrt(pred_meanv[0]*pred_meanv[0] + pred_meanv[1]*pred_meanv[1]);
    double dotprod = flowvel[0]*pred_meanv[0] + flowvel[1]*pred_meanv[1];
    cosdist = dotprod / (modflow * modpredmu);

//    std::cout << cosdist << std::endl;

//    std::cout << pred_meanv[0] << " " << pred_meanv[1] << " " << flowvel[0] << " " << flowvel[1] << " " << mahdist << " " << threshold << std::endl;

    return (mahdist > thresh_mag | cosdist < thresh_angle);
}

/**********************************************************/
yarp::sig::Vector vEgomotionThread::predict_mean(svm_node *encvel)
{
    yarp::sig::Vector mu(2);

    //predictions
    double pred_mu_vx = 0;
    double pred_mu_vy = 0;

//    //predict the mean optical flow using the trained model
//    std::cout << "now " << encvel[0].value << " " << encvel[1].value << " " << encvel[2].value << " "
//                                 << encvel[3].value << " " << encvel[4].value << " " << encvel[5].value << " " << std::endl;

    pred_mu_vx = svm_predict(mu_vx, encvel);
    pred_mu_vy = svm_predict(mu_vy, encvel);

    mu[0] = pred_mu_vx;
    mu[1] = -pred_mu_vy;

//    std::cout << mu[0] << " " << mu[1] << std::endl;

    return mu;
}

//**********************************************************/
yarp::sig::Matrix vEgomotionThread::predict_cov(svm_node *encvel)
{
    yarp::sig::Matrix cov(2, 2);

    //predictions
    double pred_sigma_vx = 0.0;
    double pred_sigma_vy = 0.0;
    double pred_sigma_vxvy = 0.0;

    //predict the variance of optical flow using the trained model
    pred_sigma_vx = svm_predict(sigma_vx, encvel);
    pred_sigma_vy = svm_predict(sigma_vy, encvel);
    pred_sigma_vxvy = svm_predict(sigma_vxvy, encvel);

    cov(0, 0) = pred_sigma_vx;
    cov(0, 1) = pred_sigma_vxvy;
    cov(1, 0) = cov(0, 1);
    cov(1, 1) = pred_sigma_vy;

//    std::cout << cov(0, 0) << " " << cov(0, 1) << " " << cov(1, 1) << std::endl;

//    Mat cov = (Mat_<double>(2,2) << predicted_result_sigma_vx,
//                 predicted_result_sigma_vxvy, predicted_result_sigma_vxvy,
//                 predicted_result_sigma_vy);

    return cov;

}

//**********************************************************/
vEncObsThread::vEncObsThread(std::string name)
{
    this->name = name;
}

//**********************************************************/
bool vEncObsThread::threadInit()
{
    yInfo("Starting thread for reading velocities...");

//    yarp::os::Property options;

//    //CHANGE THIS ON THE REAL ROBOT!!!!!!
//    options.put("robot", "icubSim");
//    options.put("device", "remote_controlboard");
//    yarp::os::Value& robotname = options.find("robot");
//    options.put("local", "/" + robotname.asString() + "/local/head");
//    options.put("remote", "/" + robotname.asString() + "/head");

//    encdriver.open(options);
//    if(encdriver.isValid()) {

//        encdriver.view(iencs);

//        //get number of joints and resize encoder vector
//        int joints;
//        iencs->getAxes(&joints);
//        encvels.resize(joints);
//        return true;
//    }
//    else
//    {
//        std::cerr << "encoder driver not opened " << std::endl;
//        return false;
//    }

    std::string encPortName = "/" + name + "/encoders:i";
    if(!encPort.open(encPortName)) {
        std::cout << "could not open encoders port" << std::endl;
        return false;
    }

}

//**********************************************************/
void vEncObsThread::run()
{
    //loop until the thread is running
    while(true)
    {
        //read encoders velocities

        //get velocities from encoders
        //        iencs->getEncoderSpeeds(encvels.data());

        //get velocities from velocityObserver
        yarp::os::Bottle *bot = encPort.read();
        encvels.resize(bot->size());
        if(!bot->isNull()) {
            for(int b = 0; b < bot->size(); b++) {
                encvels[b] = bot->get(b).asDouble();
//                std::cout << encvels[b] << " ";
            }
        }
    }
}

//**********************************************************/
void vEncObsThread::getEncVels(yarp::sig::Vector &encvelscopy)
{
    for(unsigned int i = 0; i < encvels.size(); i++) {
//        std::cout << encvels[i] << " ";
        encvelscopy.push_back(encvels[i]);
//        std::cout << encvelscopy[i] << std::endl;
    }

}

//**********************************************************/
void vEncObsThread::threadRelease()
{
    std::cout << "Stopping thread..." << std::endl;
    encPort.close();
    encdriver.close();
}

//**********************************************************/
//               THREAD FOR PREDICTION                     //
//**********************************************************/
vPredictThread::vPredictThread(collectorPort *outthread, svm_model *mu_vx, svm_model *mu_vy)
{
    this->outthread = outthread;
    this->mu_vx = mu_vx;
    this->mu_vy = mu_vy;
    semaphore = new yarp::os::Semaphore(0);
    suspended = true;
}

void vPredictThread::suspend()
{
    suspended = true;
}

void vPredictThread::wakeup()
{
    suspended = false;
    semaphore->post();
}

bool vPredictThread::available()
{
    return suspended;
}

void vPredictThread::assignTask(ev::event<ev::FlowEvent> ofe, struct svm_node *encvel, yarp::os::Stamp *ystamp)
{
    this->ofe = ofe;
    this->encvel = encvel;
    ystamp_p = ystamp;
    wakeup();
}

void vPredictThread::run()
{

    while(true) {

        //if no task is assigned, wait
        if(suspended) {
            semaphore->wait();
        }
        else {

            yarp::sig::Vector pred_meanv(2);
            yarp::sig::Matrix pred_covv(2, 2);
            pred_meanv = predict_mean(encvel);
            pred_covv(0, 0) = 1;
            pred_covv(1, 0) = 0;
            pred_covv(0, 1) = 0;
            pred_covv(1, 1) = 1;

            //compute metric
            bool isindependent = detect_independent(ofe, pred_meanv, pred_covv, 1);

            //if it is independent motion, tag the event as independent
            auto inde = make_event<LabelledAE>(ofe);
            if(isindependent)
                inde->ID = 2;
            else
                //if not, tag it as corner
                inde->ID = 1;

            outthread->pushevent(inde, *ystamp_p);

            suspend();
        }
    }
}

bool vPredictThread::detect_independent(event<FlowEvent> ofe, yarp::sig::Vector pred_meanv, yarp::sig::Matrix pred_covv, double weight)
{

    yarp::sig::Vector flowvel(2);
    yarp::sig::Vector diff(2);
    yarp::sig::Matrix invcov(2, 2);
    yarp::sig::Matrix a(1, 2);

    flowvel[0] = ofe->vx;
    flowvel[1] = ofe->vy;

    diff[0] = flowvel[0] - pred_meanv[0];
    diff[1] = flowvel[1] - pred_meanv[1];
    invcov = yarp::math::luinv(pred_covv);

    a(0, 0) = diff[0]*invcov(0, 0) + diff[1]*invcov(1, 0);
    a(0, 1) = diff[0]*invcov(0, 1) + diff[1]*invcov(1, 1);

    //compute mahalanobis distance
    double mahdist = weight * sqrt(a(0, 0)*diff[0] + a(0, 1)*diff[1]);

    double modflow = sqrt(flowvel[0]*flowvel[0] + flowvel[1]*flowvel[1]);
    double modpredmu = sqrt(pred_meanv[0]*pred_meanv[0] + pred_meanv[1]*pred_meanv[1]);
    double dotprod = flowvel[0]*pred_meanv[0] + flowvel[1]*pred_meanv[1];
    double cosdist = dotprod / (modflow * modpredmu);

    return (mahdist > 5.0 | cosdist < 0.3);
}

yarp::sig::Vector vPredictThread::predict_mean(svm_node *encvel)
{
    yarp::sig::Vector mu(2);

    //predictions
    double pred_mu_vx = 0;
    double pred_mu_vy = 0;

    pred_mu_vx = svm_predict(mu_vx, encvel);
    pred_mu_vy = svm_predict(mu_vy, encvel);

    mu[0] = pred_mu_vx;
    mu[1] = -pred_mu_vy;

    return mu;
}

void vPredictThread::onStop()
{
    wakeup();
}

//empty line to make gcc happy
