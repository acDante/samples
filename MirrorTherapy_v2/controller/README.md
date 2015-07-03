#Controller#

Controller の実装例を格納しています．  
SIGVerse の man-nii のアバターを，KinectV2 と OculusRiftDK1 で操作する Controller の実装例です．  

（Ubunutuの端末で）  
各ディレクトリ内で make すると，各 Controller でリンクする共有オブジェクトライブラリ（.so）を生成します．  
その後， sigserver.sh -w ./world.xml とコマンドを入力すると，SIGServer が起動し，  
 Plugin からのメッセージの受信待ち状態になります．  
その後，SIGViewer で Connect ボタンをクリックして START ボタンをクリックし， Services から各 Plugin をスタートさせると，  
各 Controller が動作します．

##ManNiiAvatarControllerByKinectV2##

KinectV2 を使ってアバターを操作するControllerの実装例です．  
Plugin から送られるメッセージを受け取った時の処理は，主に次の通りです．

* KinectV2_vs2013.sig を使用して送信されるメッセージを受け取る．
* 受け取ったメッセージに基づいて KinectV2SensorData クラスのインスタンスを生成（decode）する．
* KinectV2SensorData （KinectV2 で計測された全身のクォータニオン）を，man-nii のクォータニオンへ変換する．
* man-nii のクォータニオンを，SIGViewer 上のアバターへセットする．


##ManNiiAvatarControllerByOculus##

OculusRiftDK1 を使ってアバターを操作するControllerの実装例です．  
Plugin から送られるメッセージを受け取った時の処理は，主に次の通りです．

* OculusRiftDK1_vs2010.sig を使用して送信されるメッセージを受け取る．
* 受け取ったメッセージに基づいて OculusRiftDK1SensorData クラスのインスタンスを生成（decode）する．
* OculusRiftDK1SensorData （Oculus の姿勢を示すオイラー角）を，man-nii の首のクォータニオンへセットする．
* man-nii の首のクォータニオンを，SIGViewer 上のアバターへセットする．

##MirrorTherapyController##

KinectV2 と OculusRiftDK1 の両方（あるいはそのどちらか）を使ってアバターを操作し，MirrorTherapy を行う実装例です．  
上記2つのコントローラが持つ機能に加えて，次のような機能を主に実装しています．

* 実世界での右腕の動作を反転させてアバターの左腕へ伝える機能（実世界での左腕の動作は無視される）
* 実世界での左腕の動作を反転させてアバターの右腕へ伝える機能（実世界での右腕の動作は無視される）
* 実世界でのどちらかの腕の動作を反転させて伝える際に，指定の秒[ms]だけ遅らせて伝える機能
* 動作の反転モード（右腕の動作を左腕へ，左腕の動作を右腕へ，実世界の動作をそのまま伝える（反転しない），の3つ）を  
SIGViewer から送信するメッセージによって変更する機能
* 動作を遅らせる秒数（動作の遅延時間）を，SIGViewer から送信するメッセージによって変更する機能

##ControllerCommon##

上記3つの Controller で共通する機能を持つクラスを実装したソースコードを格納しています．

* AvatarController クラスの実装が含まれています．
 * 上記3つのControllerは，AvatarController クラスを継承しています．
* Posture クラスの実装が含まれています．
* Posture クラスを継承する ManNiiPosture クラスの実装が含まれています．
 * man-nii の姿勢の情報は， ManNiiPosture クラスで扱います．