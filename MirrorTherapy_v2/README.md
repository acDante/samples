MirrorTherapy_v2
=============

Controller
----------
コントローラに必要なソースコードのディレクトリです．

* 今のところ，UbuntuでSIGServerを起動し，man-niiをコントロールする実装が含まれています．  
* 具体的には，Pluginから送信されるメッセージを受け取って，SIGViewer上のman-niiを動かします．  

Plugin
------
サービスプロバイダのソースコードのディレクトリです．

* 今のところ，Windowsで動作するKinectV2とOculusRiftDK1の情報をControllerへ送信する実装が含まれています．  
* KinectV2用のサービスプロバイダでは，KinectV2のSDKで算出されるクォータニオンを送信します．  
* OculusRiftDK1用のサービスプロバイダでは，OculusのSDKで算出される首のオイラー角を送信します．


Common
------
ControllerとPluginの両方に必要なソースコードのディレクトリです．

* SensorDataクラスの実装が含まれています．


SensorDataクラスについて
------------------------
SensorDataクラスは，デバイスから得られる情報のうち，Controllerへ送信する情報のみを扱う役割を持ちます．  
SensorDataクラスのサブクラスでは，次の2つのメソッドを実装する必要があります．
1. デバイスから得られるセンサの情報（姿勢など）をメッセージ（文字列）へ変換・生成するメソッド： encodeSensorData2Message()
2. メッセージ（文字列）をセンサの情報（姿勢など）へ変換するメソッド：decodeMessage2SensorData(std::string &message)

