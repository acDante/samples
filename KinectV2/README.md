Name  
====  
KinectV2  

## Overview  
 Client side service provider of the Sample project to move avator with KinectV2, as Visual Studio 2013 C++ project.  

## Description
 Client side service provider of the Sample project to move avator with KinectV2, as Visual Studio 2013 C++ project.  
 This is a sample project to move avator in SIGVerse with KinectV2.  
 When you move in front of KinectV2 sensor, an avator in the SIGVerse  
 moves along with your movement. 

## Demo  

## VS. 

## Requirement  
 Please install SIGServer-2.2.0 and SIGViewer-2.2.0 as below:  
 <http://www.sigverse.com/wiki/en/index.php?Tutorial>  
 Please install Visual Studio 2013  
   
 include  
 ../include: SIGVerse header files and OpenNI header files needed  
 library  
 ../lib: SIGVerse library file and OpenNI library file needed  
 rutime  
 ../Release: OpenCV dynamic link library files and a config file needed  

you can download from below:  
<**********>  
Please merge to the project directory.  
  
## Usage  
 1. boot up Ubuntu12.04 on VMware.  
 2. start SIGServer on Ubuntu. SIGVerse must be SIGVerse-2.2.0.  
   $ cd ***  
   $ sigserver.sh -w ./kinectWorld.xml(filename) -p 9000(port number)  
 3. start SIGViewer on Windows8.  
 4. push start button on SIGViewer to start simulation.  
 5. edit KinectV2_2013_.bat, set your IPadress and port number of Ubuntu.  
 6. execute KinectV2.bat.  
 7. move your body in front fo Kinect sensor.  
 
## Install
 set any place  

## Contribution  

## Licence  

## Author  
National Institute of Informatics  
 