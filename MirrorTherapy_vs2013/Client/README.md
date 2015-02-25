Name  
====  
MirrorTherapy_vs2013 

## Overview  
 Rehabilitation project of phantom limb pain with Kinect V2, as Visual Studio 2013 C++ project.  

## Description  
 If you lost your limbs or arms, you might suffer from pain of them even they doesn't exist.  
 That is called "phantom limb pain". You might relieve your pain with "MirrorTherapy".  
 "MirrorTherapy" is that you move existing limb or arm in the  mirror, recognizing it as a lost limb or arm.  
 MirrorTherapy project make the therapy efficient and easy with Kinect V2 and Oculus Rift. 

## Demo  

## VS.  

## Requirement  
 Please install SIGServer-2.2.0 and SIGViewer-2.2.0 as below:  
 <http://www.sigverse.com/wiki/en/index.php?Tutorial>  
 Please install Visual Studio 2013  
   
 Project settings below:  
 
 include  
  ./include: SIGVerse and OpenNI header files (.h) needed  
 
 runtime  
 ./Release: OpenCV dynamic link library file(.dll) , SIGVerse config file(.ini) needed.
  1. config.ini
  2. opencv_core2410.dll
  3. opencv_highgui2410.dll
  4. opencv_imgproc2410.dll
  
 

 You can download from below:  
 SIGViewer-2.2.0 of SIGVerse-2.2.0   
 <http://sigverse.org/sigverse/main/download/>  
 Please install it as below:  
 <http://www.sigverse.com/wiki/en/index.php?Tutorial>   
 OpenCV2.4.10  
 <http://opencv.org/downloads.html>  


 Please merge to the project directory.  

## Usage  
 1. boot up Ubuntu12.04.  
 2. start SIGServer on Ubuntu. SIGVerse must be sigverse-2.2.0.  
    $ cd /EmbodiedBrain/MirrorTherapy/Server/Controller  
    $ sigserver.sh -w ./MirrorTherapy.xml(filename)
 3. start SIGViewer on Windows8.  
 4. push start button on SIGViewer to start simulation.
 5. push Services -> Start -> MirrorTherapy_vs2013.
 6. move your body in front fo Kinect sensor.  

## Install  
 set any place  

## Contribution  

## Licence  

## Author  
 National Institute of Informatics

wrote by Nozaki.
Date:  Feb. 25th, 2015.
 