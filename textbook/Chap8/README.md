Chap8 Bayesian Filter
===


### BayesianFilter.h
---
各種パラメータを設定します．


### Moderator.cpp
---
迷路の環境を設定します．

ロボットが選択した行動に合わせて存在確率を計算します．


### BayesianFilter.cpp
---
POMDPを前提としながら，自分が行った行動から自己位置を推定します．

メッセージに沿った行動を行います．



===
WinWorld.xml内のRobotに実装するdllを変更することで，コントローラを変更できます．
