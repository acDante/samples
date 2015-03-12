Name  
====  
KinectV2_vs2013  

## Overview  
 KinectV2で計測されるQuaternionをサーバーへ送信します．  
 以前は.batを実行する必要がありましたが，現在はSIGViewerから直接起動できます．  
 （KinectV2_vs2013.sigを生成します．）  

## Requirement  
 ソリューション構成はReleaseにしてください．  
 boost以外で使用しているライブラリは次の通りです．  
 ・SIGService.lib  
 ・kinect20.lib  
 ・opencv_highgui2410.lib  
 ・opencv_core2410.lib  
 ・opencv_imgproc2410.lib  
 
 インクルードするヘッダファイルのうち，Clientフォルダ内にないものは次の通りです．  

 (1) Kinect.h  
	私の環境では，KinectV2のSDKをインストール後に
	「C:\Program Files\Microsoft SDKs\Kinect\v2.0-PublicPreview1409\inc」内に生
	成されていました．

 (2) SIGService.h  
	共通の環境を構築していれば，
	「C:\SIGVerse\GitHub\Client\SIGService\Windows\SIGService」内にあると思いま
	す．

## Author  
 National Institute of Informatics  

## Wrote by  
 Nozaki, 12th 2015 March. 