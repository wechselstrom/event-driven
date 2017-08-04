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
    double threshold = rf.check("thresh", yarp::os::Value(2.5)).asDouble();

    //create the thread for reading joint velocities
    encsobserver = new vEncObsThread(moduleName);
    if(!encsobserver->start()) {
        std::cout << "could not open encoders observer" << std::endl;
    }

    //create the thread for reading the events
    egomotionthread = new vEgomotionThread(moduleName, train, threshold, encsobserver);
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
vEgomotionThread::vEgomotionThread(std::string name, bool train, double threshold, vEncObsThread *velobs)
{
    this->name = name;
    this->train = train;
    this->velobs = velobs;

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

    cmx = 0.0;
    cmy = 0.0;

    if(train) {

        yInfo("STARTING TRAINING...");
        yInfo("Please move only the iCub robot ");
        yInfo("Files will be saved in: " + folder);
        mu_vx_training.open(folder + "test1.txt", std::ios_base::out);
        mu_vy_training.open(folder + "test2.txt", std::ios_base::out);
        sigma_vx_training.open(folder + "test3.txt", std::ios_base::out);
        sigma_vy_training.open(folder + "test4.txt", std::ios_base::out);
        sigma_vxvy_training.open(folder + "test5.txt", std::ios_base::out);

    }
    else {

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

        this->threshold = threshold;

    }

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

}

/**********************************************************/
void vEgomotionThread::run()
{

    while(true) {

        //get queue of flow events
        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = allocatorCallback.getNextQ(yarpstamp);
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
        bool isindependent;
        double weight;
        for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

            //get the flow event
            auto ofp = ev::is_event<ev::FlowEvent>(*qi);
            avgvx += ofp->vx;
            avgvy += ofp->vy;
            int currt = unwrapper(ofp->stamp);

            //            //testing
            //            if(!train) {

            //predict egomotion using current encoder velocities
            //and the learnt models
            if(notmoving == true) {
                isindependent = true;
                cmx += ofp->x;
                cmy += ofp->y;
                ts = currt;
            }
            else {
                pred_meanv = predict_mean(encveltest);
//                pred_covv = predict_cov(encveltest);

                pred_covv(0, 0) = 1;
                pred_covv(1, 0) = 0;
                pred_covv(0, 1) = 0;
                pred_covv(1, 1) = 1;

//                //if something has moved
//                if(cmx != 0.0 && cmy != 0.0) {

//                }
                //distance from known independently moving corners
                double dx = ofp->x - cmx;
                double dy = ofp->y - cmy;
                double dist = sqrt(dx*dx + dy*dy);
                int dt = currt - ts;

                //give more weight to corners close to known independently moving corners
                if(dist < 5 && dt < 100000)
                    weight = 2;
                else
                    weight = 1;

                //compute metric
                isindependent = detect_independent(ofp, pred_meanv, pred_covv, mah_dist, weight);

                if(dt > 100000) {
                    cmx = 0.0;
                    cmy = 0.0;
                }

            }

            if(notmoving == true) {
                cmx = cmx/q->size();
                cmy = cmy/q->size();
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
                scorebottleout.addDouble(mah_dist);
                scorebottleout.addDouble(ofp->vx);
                scorebottleout.addDouble(ofp->vy);
                debugPort.write();
            }

            //            }
        }

//        if(debugPort.getOutputCount()) {
//            yarp::os::Bottle &scorebottleout = debugPort.prepare();
//            scorebottleout.clear();
//            scorebottleout.addDouble(pred_meanv[0]);
//            scorebottleout.addDouble(pred_meanv[1]);
//            scorebottleout.addDouble(pred_covv(0, 0));
//            scorebottleout.addDouble(pred_covv(1, 1));
//            scorebottleout.addDouble(mah_dist);
//            scorebottleout.addDouble(avgvx/count);
//            scorebottleout.addDouble(avgvy/count);
//            debugPort.write();
//        }

//        if(train) {

//        float svx = 0.0;
//        float svy = 0.0;
//        float svxvy = 0.0;

//            //compute statistics
////            std::cout << q->size() << std::endl;
//            avgvx = avgvx / q->size();
//            avgvy = avgvy / q->size();

//            for(ev::vQueue::iterator qi = q->begin(); qi != q->end(); qi++) {

//                //get the flow event
//                auto ofp = ev::is_event<ev::FlowEvent>(*qi);
//                svx += (ofp->vx - avgvx) * (ofp->vx - avgvx);
//                svy += (ofp->vy - avgvy) * (ofp->vy - avgvy);
//                svxvy += (ofp->vx - avgvx) * (ofp->vy - avgvy);
//            }
//            svx = sqrt(svx / (q->size() - 1));
//            svy = sqrt(svy / (q->size() - 1));
//            svxvy = sqrt(svxvy / (q->size() - 1));

//            //save to file
//            mu_vx_training << avgvx;
//            mu_vy_training << avgvy;
//            sigma_vx_training << svx;
//            sigma_vy_training << svy;
//            sigma_vxvy_training << svxvy;
//            for(unsigned int b = 0; b < encvels.size(); b++)
//            {
//                 mu_vx_training << " " << b + 1 << ":" << encvels[b];
//                 mu_vy_training << " " << b + 1 << ":" << encvels[b];
//                 sigma_vx_training << " " << b + 1 << ":" << encvels[b];
//                 sigma_vy_training << " " << b + 1 << ":" << encvels[b];
//                 sigma_vxvy_training << " " << b + 1 << ":" << encvels[b];
//            }
//            mu_vx_training << "\n";
//            mu_vy_training << "\n";
//            sigma_vx_training << "\n";
//            sigma_vy_training << "\n";
//            sigma_vxvy_training << "\n";

////            svm_problem prob_muvx;
////            prob_muvx.x = encveltrain;
////            prob_muvx.y = &avgvx;
////            mu_vx = svm_train(&prob_muvx, &svm_param);
////            if(svm_save_model(mu_vx_file_train, mu_vx))
////                std::cout << "saving model for mu_vx " << std::endl;

//        }

        allocatorCallback.scrapQ();

    }

}

/**********************************************************/
bool vEgomotionThread::detect_independent(event<FlowEvent> ofe, yarp::sig::Vector pred_meanv, yarp::sig::Matrix pred_covv,
                                          double &mahdist, double weight)
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

//    std::cout << pred_meanv[0] << " " << pred_meanv[1] << " " << flowvel[0] << " " << flowvel[1] << " " << mahdist << " " << threshold << std::endl;

//    if(debugPort.getOutputCount()) {
//        yarp::os::Bottle &scorebottleout = debugPort.prepare();
//        scorebottleout.clear();
//        scorebottleout.addDouble(mahdist);
//        debugPort.write();
//    }

    return mahdist > threshold;
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

//empty line to make gcc happy
