MirrorTherapy_v2
================

初めに，(1) Controller, (2) Plugin, (3) Commonの3つのディレクトリについて説明します．  
その後，(4) 基本的な使い方（ソースコードのビルド方法と実行方法の例）について説明します．

(1) Controller
--------------
コントローラのソースコードのディレクトリです．  
UbuntuでSIGServerを起動し，Pluginから送信されるメッセージを受け取って，SIGViewer上のman-niiを操作することができます．

(2) Plugin
----------
サービスプロバイダのソースコードのディレクトリです．
実装したサービスプロバイダの機能は次のようなものです．

* Windowsで動作するKinectV2とOculusRiftDK1の情報をControllerへ送信する．  
 * KinectV2用のサービスプロバイダでは，KinectV2のSDKで算出されるクォータニオンを送信する．  
 * OculusRiftDK1用のサービスプロバイダでは，OculusのSDKで算出される首のオイラー角を送信する．

(3) Common
----------
ControllerとPluginの両方に必要なソースコードのディレクトリです．  
SensorDataクラスの実装が含まれています．  
注意点などを次に列挙します．

* Window と Ubuntu の両方で使用する．
* 実装には boost が含まれているので，ソースコードのビルドのために Ubuntu へ boost をインストールする必要がある．
* Ubuntu への boost のインストール例（端末で打つコマンド）を次に示す．

` sudo apt-get install boost-all-dev


(4) 基本的な使い方
-----------------

リポジトリを Clone し，MirrorTherapy_v2 の全ソースコードをビルドすると，次の表の「実行ファイル名」に示す実行ファイルが得られます．  
同表に，実行ファイルの種類（PluginはWindows用，ControllerはUbuntu用）と簡単な概要も記載しました．

| - | 実行ファイル名  | Plugin / Controller | 概要 |
| ---------- | ------------- | ------------- | ------------- |
|a| KinectV2_vs2013.sig  | Plugin  | KinectV2のクォータニオンをControllerへ送信する |
|b| OculusRiftDK1_vs2010.sig  | Plugin  | OculusRiftDK1のオイラー角をControllerへ送信する |
|c| ManNiiAvatarControllerByKinectV2.so  | Controller | KinectV2のクォータニオンを受信してSIGViewer上のアバターを操作する |
|d| ManNiiAvatarControllerByOculus.so  | Controller | Oculusのオイラー角を受信してSIGViewer上のアバターを操作する |
|e| MirrorTherapyController.so  | Controller | KinectV2とOculusを同時に利用してSIGViewer上のアバターの動作を加工する |

基本的には，次のような手順でビルドして実行します．

1. 使用する機器（Kinect V2 と OculusRiftDK1）を PC に接続する（Oculusはキャリブレーションをしておくと better）．
2. Windows側で，C:\SIGVerse\src\ 直下にリポジトリを Clone する．
3. Windows側で，上の表の a と b を生成（ビルド）する．
4. Windows側で，SIGViewer を起動し，上の表の a と b を SIGViewer の Services に Add する．
5. Ubuntu側で，/home/sigverse/ 以下の任意のディレクトリにリポジトリを Clone する．
6. Ubuntu側で，上の表の c, d, e を生成（ビルド）する．
7. Ubuntu側で，SIGServer を起動する（SIGServer のワールドファイルに c, d, e のいずれかをリンクする）．
8. Windows側で，SIGViewer で IP Address と Port Number を指定して Connect する．
9. Windows側で，SIGViewer で START をクリックして Services -> Start で a か b のいずれか，あるいは両方をクリックして Plugin を実行する．
10. （以降は終了時の手順） Windows側で，SIGViewer で STOP をクリックする．
11. Windows側で，Plugin 実行時に開いたコマンドプロンプトをクリックして（アクティブにして），キーボードで Ctrl+C を押下する．
12. Windows側で， SIGViewer で Quit Simulation をクリックする．

手順の 3 と 6 でビルドしますが，ビルドの例はそれぞれの Plugin / Controller のディレクトリ内に記載していますので，ビルドの際はそちらを参考にしてください．  
実行ファイル群（.sig と .so）をビルド済みの場合は，手順の 2 から 6 は必要ありません．