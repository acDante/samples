Chap9 粒子フィルタ
===


### ParticleFilter.h
---
各種パラメータを設定します．


### Moderator.cpp
---
迷路の環境を設定します．



### ParticleFilter.cpp
---
ベイズフィルタを基にしつつ，モンテカルロ近似とSIRを導入することで実現されるベイズフィルタの近似手法です．

Robotは送られてきたメッセージに沿った行動を行います．
上：n, 右：e, 下：s, 左：w


RobotまたはModeratorにinitialとメッセージを送ると，エピソードが変わります．


===
WinWorld.xml内のRobotに実装するdllを変更することで，コントローラを変更できます．
