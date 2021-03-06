#vAutosaccadeDemo App

Introduction
------------
This module is used to generate events in static scenes. When the event rate drops below a certain threshold, a 
circular motion of the eye is triggered. The resulting event stream gives an idea about the shapes of the objects in 
the scene. Instead, if there are enough events, then the robot gazes to their center of mass. The app launches the 
zynqGrabber which sends the events both to the vFramer for visualization and to the autosaccade module which decides 
how to move the robot eyes.

![builder_view](images/autoSaccadeApp_builder.png)
  
 Dependencies
 ------------
 This app needs the robot (or the [simulator](http://wiki.icub.org/brain/group__icub__Simulation.html)) to be up and 
 running together with the [yarprobotinterface](http://www.yarp.it/yarprobotinterface.html) and the [iKinGazeCtrl](http://wiki.icub.org/brain/group__iKinGazeCtrl.html) .
 
 Instantiated Modules
 --------------------
 * **zynqGrabber**
 * **autosaccade**
 * **vFramer**
 * **yarpview**
 
 Opened ports
 ------------
 * `/zynqGrabber/vBottle:o`
 * `/autosaccade/vBottle:i`
 * `/vFramer/AE:i`
 * `/vFramer/left`
 * `/vFramer/right`
 * `/viewleft`
 * `/viewright`

How to run the application
--------------------------

On a console, run yarpserver (if not already running).

You can now run yarpmanager.

Inside the Application folder in the yarpmanager gui, you should see an entry called vAutosaccadeDemo. Double click and 
open it.

Run the robot (or the iCubSim), the yarprobotinterface and the iKinGazeCtrl. Make sure to specify the correct robot 
name passing the proper parameter to the autosaccade module.

Now you are ready to run the application! Hit the run button and then connect on the yarpmanager gui.

You will now see the robot (or the simulator) gazing to the center of mass of the events or, if the event rate is not
 high enough, performing a circular eyes motion to generate events.