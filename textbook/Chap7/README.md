Chap7 Q-Learning
===


### QLearning.h
---
各種パラメータを設定します．


### Moderator.cpp
---
迷路の環境を設定し，ロボットのスタート位置を指定します．

また，ロボットに報酬を返します．


### QLearning.cpp
---
Q学習を用いて経路を学習します．

ε-Greedy法を用いて行動を選択します．

エピソードが終了するごとに Q-values.txt にQ値が保存されます．


### RobotController.cpp
---
得られたQ値に従って移動します．



===
WinWorld.xml内のRobotに実装するdllを変更することで，コントローラを変更できます．
