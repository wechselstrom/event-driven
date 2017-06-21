///*
// * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
// * Author: Valentina Vasco
// * email: valentina.vasco@iit.it
// * Permission is granted to copy, distribute, and/or modify this program
// * under the terms of the GNU General Public License, version 2 or any
// * later version published by the Free Software Foundation.
// *
// * A copy of the license can be found at
// * http://www.robotcub.org/icub/license/gpl.txt
// *
// * This program is distributed in the hope that it will be useful, but
// * WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// * Public License for more details
// */

//#include "vEgoMotion.h"

//using namespace ev;

///**********************************************************/
//bool vEgomotionModule::configure(yarp::os::ResourceFinder &rf)
//{
//    //set the name of the module
//    std::string moduleName =
//            rf.check("name", yarp::os::Value("vEgomotion")).asString();
//    yarp::os::RFModule::setName(moduleName.c_str());

//    bool strict = rf.check("strict") &&
//            rf.check("strict", yarp::os::Value(true)).asBool();

//    bool train = rf.check("train", yarp::os::Value(false)).asBool();
//    double threshold = rf.check("thresh", yarp::os::Value(4.0)).asDouble();
//    yarp::os::Bottle param = rf.findGroup("Parameters").tail();

//    //create the thread for reading the events
//    egomotionmanager = new vEgomotionManager(train, param, threshold);
//    if(!egomotionmanager->open(moduleName, strict)) {
//        std::cout << "could not open egomotion manager" << std::endl;
//        return false;
//    }

//    return true;

//}

///**********************************************************/
//bool vEgomotionModule::interruptModule()
//{
////    encsobserver->stop();
//    egomotionmanager->interrupt();
//    yarp::os::RFModule::interruptModule();
//    return true;
//}

///**********************************************************/
//bool vEgomotionModule::close()
//{
//    egomotionmanager->close();
//    delete egomotionmanager;
//    yarp::os::RFModule::close();
//    return true;
//}

///**********************************************************/
//bool vEgomotionModule::updateModule()
//{
//    return true;
//}

///**********************************************************/
//double vEgomotionModule::getPeriod()
//{
//    return 1;
//}

///**********************************************************/
//vEgomotionManager::vEgomotionManager(bool train, yarp::os::Bottle param, double threshold)
//{
//    this->train = train;
//    if(train == true) {

////        svm_param->svm_type = param[0];
//        svm_param.svm_type = NU_SVR;
//        svm_param.shrinking = 0;

//    }
//    else {

//        //load the trained models
//        mu_vx = svm_load_model(mu_vx_file);
//        if(!mu_vx)
//            std::cerr << "muvx model could not be loaded" << std::endl;
//        else
//            std::cout << "muvx model successfully loaded" << std::endl;

//        mu_vy = svm_load_model(mu_vy_file);
//        if(!mu_vy)
//            std::cerr << "muvy model could not be loaded" << std::endl;
//        else
//            std::cout << "muvy model successfully loaded" << std::endl;

//        sigma_vx = svm_load_model(sigma_vx_file);
//        if(!sigma_vx)
//            std::cerr << "sigmavx model could not be loaded" << std::endl;
//        else
//            std::cout << "sigmavx model successfully loaded" << std::endl;

//        sigma_vy = svm_load_model(sigma_vy_file);
//        if(!sigma_vy)
//            std::cerr << "sigmavy model could not be loaded" << std::endl;
//        else
//            std::cout << "sigmavy model successfully loaded" << std::endl;

//        sigma_vxvy = svm_load_model(sigma_vxvy_file);
//        if(!sigma_vxvy)
//            std::cerr << "sigmavxvy model could not be loaded" << std::endl;
//        else
//            std::cout << "sigmavxvy model successfully loaded" << std::endl;

//        this->threshold = threshold;

//    }

//    //create smv node that will be our features (encoders velocities)
//    encvel = new svm_node[6];

//    //    //create the thread for reading encoders velocities
//    //    encsobserver = new vEncObsThread(moduleName);
//    //    if(!encsobserver->start()) {
//    //        std::cout << "could not open encoders observer" << std::endl;
//    //    }

//}

///**********************************************************/
//bool vEgomotionManager::open(const std::string moduleName, bool strictness)
//{
//    this->strictness = strictness;
//    if(strictness) {
//        std::cout << "Setting " << moduleName << " to strict" << std::endl;
//        this->setStrict();
//    }
//    this->useCallback();

//    std::string inPortName = "/" + moduleName + "/vBottle:i";
//    bool check1 = BufferedPort<ev::vBottle>::open(inPortName);

//    if(strictness) outPort.setStrict();
//    std::string outPortName = "/" + moduleName + "/vBottle:o";
//    bool check2 = outPort.open(outPortName);

//    std::string encPortName = "/" + moduleName + "/encoders:i";
//    bool check3 = encPort.open(encPortName);

//    std::string debugPortName = "/" + moduleName + "/score:o";
//    bool check4 = debugPort.open(debugPortName);

//    return check1 && check2 && check3 && check4;

//}

///**********************************************************/
//void vEgomotionManager::close()
//{
//    //close ports
//    yarp::os::BufferedPort<ev::vBottle>::close();
//    outPort.close();
//    encPort.close();
//    debugPort.close();

//    //remember to deallocate the allocated memory
//    delete encsobserver;
//    delete encvel;

//}

///**********************************************************/
//void vEgomotionManager::interrupt()
//{
//    //pass on the interrupt call to everything needed
//    yarp::os::BufferedPort<ev::vBottle>::interrupt();
//    outPort.interrupt();
//    encPort.interrupt();
//    debugPort.interrupt();

//    encsobserver->stop();
//}

///**********************************************************/
//void vEgomotionManager::onRead(ev::vBottle &vbot)
//{

//    /*prepare output vBottle*/
//    ev::vBottle * outBottle = 0;

//    yarp::os::Stamp st;
//    this->getEnvelope(st); outPort.setEnvelope(st);

//    /*get the event queue in the vBottle bot*/
//    ev::vQueue q = vbot.get<AE>();

//    yarp::sig::Vector pred_meanv(2);
//    yarp::sig::Matrix pred_covv(2, 2);

//    yarp::os::Bottle *bot = encPort.read();
//    double encvels[bot->size()];
//    if(!bot->isNull())
//    {
//        for(int b = 0; b < bot->size(); b++)
//        {
//            encvels[b] = bot->get(b).asDouble();
//            encvel[b].index = b + 1;
//            encvel[b].value = encvels[b];
////            std::cout << encvels[b] << " ";
//            //            std::cout << encvel[b].value << " ";
//        }
//        encvel[bot->size()].index = -1;
//        encvel[bot->size()].value = 0;

////        std::cout << std::endl;

//        //    //vector where we store the velocities from the encoders
//        //    yarp::sig::Vector encvels = encsobserver->getEncVels();
//        //    encvel = new svm_node[encvels.size()];

//        //    //get the current encoders velocities and create the svm vector
//        //    for(unsigned int i = 0; i < encvels.size(); i++)
//        //    {
//        //        encvel[i].index = i + 1;
//        //        encvel[i].value = encvels[i];
//        //    }

//        for(ev::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
//        {
//            //get the current optical flow event
//            auto ofp = is_event<FlowEvent>(*qi);
////            if(!ofp->getChannel()) continue;

//            if(train) {

//                svm_train();

//            }
//            else {

//                //scale encoders velocities for libsvm
//                //TODO

//                //predict egomotion using current encoder velocities
//                //and the learnt models
//                pred_meanv = predict_mean(encvel);
//                pred_covv = predict_cov(encvel);

//                //            std::cout << pred_meanv[0] << " " << pred_meanv[1] << " " << ofp->vx << " " << ofp->vy << std::endl;

//                //compute metric
//                bool isindependent = detect_independent(ofp, pred_meanv, pred_covv);

//                auto inde = make_event<LabelledAE>(ofp);
//                //if it is independent motion, tag the event as independent
//                if(isindependent) {
//                    inde->ID = 2;
//                    //                std::cout << "independent " << std::endl;
//                }
//                else
//                    //if not, tag it as corner
//                    inde->ID = 1;

//                if(debugPort.getOutputCount()) {
//                    yarp::os::Bottle &scorebottleout = debugPort.prepare();
//                    scorebottleout.clear();
//                    scorebottleout.addDouble(pred_meanv[0]);
//                    scorebottleout.addDouble(pred_meanv[1]);
//                    scorebottleout.addDouble(ofp->vx);
//                    scorebottleout.addDouble(ofp->vy);
//                    debugPort.write();
//                }

//                if(!outBottle) {
//                    outBottle = &outPort.prepare();
//                    outBottle->clear();
//                }
//                outBottle->addEvent(inde);

//            }
//        }
//    }

//    outPort.write(strictness);

//}

///**********************************************************/
//bool vEgomotionManager::detect_independent(event<FlowEvent> ofe, yarp::sig::Vector pred_meanv, yarp::sig::Matrix pred_covv)
//{

//    yarp::sig::Vector flowvel(2);
//    yarp::sig::Vector diff(2);
//    yarp::sig::Matrix invcov(2, 2);
//    yarp::sig::Matrix a(1, 2);

//    flowvel[0] = ofe->vx;
//    flowvel[1] = ofe->vy;

//    diff[0] = flowvel[0] - pred_meanv[0];
//    diff[1] = flowvel[1] - pred_meanv[1];
//    invcov = yarp::math::luinv(pred_covv);

//    a(0, 0) = diff[0]*invcov(0, 0) + diff[1]*invcov(1, 0);
//    a(0, 1) = diff[0]*invcov(0, 1) + diff[1]*invcov(1, 1);

//    //compute mahalanobis distance
//    double mahdist = sqrt(a(0, 0)*diff[0] + a(0, 1)*diff[1]);

////    std::cout << mahdist << " " << threshold << std::endl;

////    if(debugPort.getOutputCount()) {
////        yarp::os::Bottle &scorebottleout = debugPort.prepare();
////        scorebottleout.clear();
////        scorebottleout.addDouble(mahdist);
////        debugPort.write();
////    }

//    return mahdist > threshold;
//}

///**********************************************************/
//yarp::sig::Vector vEgomotionManager::predict_mean(svm_node *encvel)
//{
//    yarp::sig::Vector mu(2);

//    //predictions
//    double pred_mu_vx = 0;
//    double pred_mu_vy = 0;

////    //predict the mean optical flow using the trained model
////    std::cout << "now " << encvel[0].value << " " << encvel[1].value << " " << encvel[2].value << " "
////                                 << encvel[3].value << " " << encvel[4].value << " " << encvel[5].value << " " << std::endl;

//    pred_mu_vx = svm_predict(mu_vx, encvel);
//    pred_mu_vy = svm_predict(mu_vy, encvel);

//    mu[0] = pred_mu_vx;
//    mu[1] = pred_mu_vy;

////    std::cout << mu[0] << " " << mu[1] << std::endl;

//    return mu;
//}

////**********************************************************/
//yarp::sig::Matrix vEgomotionManager::predict_cov(svm_node *encvel)
//{
//    yarp::sig::Matrix cov(2, 2);

//    //predictions
//    double pred_sigma_vx = 0.0;
//    double pred_sigma_vy = 0.0;
//    double pred_sigma_vxvy = 0.0;

//    //predict the variance of optical flow using the trained model
//    pred_sigma_vx = svm_predict(sigma_vx, encvel);
//    pred_sigma_vy = svm_predict(sigma_vy, encvel);
//    pred_sigma_vxvy = svm_predict(sigma_vxvy, encvel);

//    cov(0, 0) = pred_sigma_vx;
//    cov(0, 1) = pred_sigma_vxvy;
//    cov(1, 0) = cov(0, 1);
//    cov(1, 1) = pred_sigma_vy;

////    std::cout << cov(0, 0) << " " << cov(0, 1) << " " << cov(1, 1) << std::endl;

////    Mat cov = (Mat_<double>(2,2) << predicted_result_sigma_vx,
////                 predicted_result_sigma_vxvy, predicted_result_sigma_vxvy,
////                 predicted_result_sigma_vy);

//    return cov;

//}

////**********************************************************/
//vEncObsThread::vEncObsThread(std::string name)
//{
//    this->name = name;
//}

////**********************************************************/
//bool vEncObsThread::threadInit()
//{
//    std::cout << "Starting thread for reading velocities..." << std::endl;

//    yarp::os::Property options;
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
//}

////**********************************************************/
//void vEncObsThread::run()
//{
//    //loop until the thread is running
//    while(!isStopping())
//    {
//        //read encoders velocities
////        std::cout << "Reading encoders velocities..." << std::endl;
//        iencs->getEncoderSpeeds(encvels.data());
////        std::cout << encvels[0] << " " << encvels[1] << " " << encvels[2] << " "
////                                 << encvels[3] << " " << encvels[4] << " " << encvels[5] << std::endl;
//    }
//}

////**********************************************************/
//yarp::sig::Vector vEncObsThread::getEncVels()
//{
//    return encvels;
//}

////**********************************************************/
//void vEncObsThread::threadRelease()
//{
//    std::cout << "Stopping thread..." << std::endl;
//    encdriver.close();
//}

////empty line to make gcc happy
