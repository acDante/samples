SIGVerse/samples/textbook
========================
RobotController for textbook

Please perform the following steps.

1. Create SIGServer for Windows.
2. Create work directory.

About this directory
------------------------
This directory include sample robot controller for textbook.
Please perform "sigmake.bat", and start SIGServer with attached WinWorld.xml.


Difinition of the wall
------------------------
You can set the wall of the maze in "Configuration.txt".
The definitions of each axial wall are as follows.

xWall (-> x) number (Position)
 ---+---+---+---+--- 5 (450)
|            3,4 4,4|
 ---+---+---+---+--- 4 (350)
|                4,3|
 ---+---+---+---+--- 3 (250)
|                   |
 ---+---+---+---+--- 2 (150)
|0,1                |
 ---+---+---+---+--- 1 (50)
|0,0 1,0            |
 ---+---+---+---+--- 0 (-50)
  0   1   2   3   4  
  0  100 200 300 400

zWall (Å´ z) number (Position)
 ---+---+---+---+--- 
|            3,4 4,4| 4 -400
 ---+---+---+---+--- 
|                4,3| 3 -300
 ---+---+---+---+--- 
|                   | 2 -200
 ---+---+---+---+--- 
|0,1                | 1 -100
 ---+---+---+---+--- 
|0,0 1,0            | 0 0
 ---+---+---+---+--- 
0   1   2   3   4   5 
-50 50  150 250 350 450