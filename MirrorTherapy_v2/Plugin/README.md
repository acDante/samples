#Plugin#

Plugin は，主に Windows の PC に接続したデバイスの情報を SIGServer の Controller へ送信する機能を持ちます．  

##KinectV2\_vs2013ディレクトリ##

KinectV2 から得られる情報（全身のクォータニオン）を Controller へ送信するプログラムのソースコードを，  
VisualStudio2013 のプロジェクト形式で格納しています．  
プロジェクトのビルド後には，同ディレクトリ内の Release ディレクトリに KinectV2\_vs2013.sig が生成されます．  

##OculusRiftDK1_vs2010##

OculusRiftDK1 から得られる情報（Oculusの姿勢：首のオイラー角）を Controller へ送信するプログラムのソースコードを，  
Visual C++ 2010 のプロジェクト形式で格納しています．  
プロジェクトのビルド後には，同ディレクトリ内の Release/Debug ディレクトリに OculusRiftDK1\_vs2010.sig が生成されます．  

##PluginCommon##

上記2つの Plugin に共通する機能（クラス）を実装しているプログラムのソースコードを格納しています．  
