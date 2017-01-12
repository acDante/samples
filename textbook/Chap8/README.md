Chap8 Bayesian Filter
===


### BayesianFilter.h
---
各種パラメータを設定します．


### Moderator.cpp
---
迷路の環境を設定します．



### BayesianFilter.cpp
---
POMDPを前提としながら，自分が行った行動から自己位置を推定します．

Robotは送られてきたメッセージに沿った行動を行います．
上：n, 右：e, 下：s, 左：w

行動を基に，以下の項目の計算を行います．

- 1) 移動後の確率分布
- 2) センサ情報
- 3) 移動と観測の情報統合
- 4) 正規化した存在確率

RobotまたはModeratorにinitialとメッセージを送ると，エピソードが変わります．


===
WinWorld.xml内のRobotに実装するdllを変更することで，コントローラを変更できます．
