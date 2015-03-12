Name  
====  
MirrorTherapy_vs2013  

## Overview  
 Rehabilitation project of phantom limb pain with Kinect V2, as Visual Studio 2013 C++ project.  
 Also with OculusRift DK1, as Visual Studio 2010.  

## Description  
 If you lost your limbs or arms, you might suffer from pain of them even they doesn't exist.  
 That is called "phantom limb pain". You might relieve your pain with "MirrorTherapy".  
 "MirrorTherapy" is that you move existing limb or arm in the  mirror, recognizing it as a lost limb or arm.  
 MirrorTherapy project make the therapy efficient and easy with Kinect V2 and Oculus Rift. 

 MirrorTherapy_vs2013では，サービスプロバイダとしてKinectV2_vs2013.sigとOculusDK1_vs2010.sigを使い，サーバー側のコントローラとしてMirrorTherapyController.cppを使います．  
 詳しくはソースコードなどを見てもらうしかございませんが，MirrorTherapyController.cppは，ManNiiAvatarController.cppの内容を継承しています．  
 MirrorTherapy用の構成ではありますが，KinectV2_vs2013.sigとOculusDK1_vs2010.sigとManNiiAvatarController.cppは汎用的に使えると思います．  

## Requirement  
 KinectV2の方はVS2013でビルドし，Oculusの方はVS2010でビルドします．

 それに伴って，使用するライブラリファイルのバージョンが異なるので注意が必要です．  
 つまり，KinectV2でもOculusでも使う「SIGService.lib」は，KinectV2のプロジェクトではVS2013用のものをリンクし，OculusのプロジェクトではVS2010用のものをリンクする必要があります．  
 VS2010用のSIGService.libがビルドできる前提(*)で，VS2013用のSIGService.libをビルドする方法から説明します．  

	(1) C:\SIGVerse\GitHub\Client\SIGService\Windows\ の SIGService_2010.sln を同フォルダ内にコピーして「SIGService_2013.sln」などに名前を変更する．
	(2) SIGService_2013.sln を VS2013 で開く．
	(3) ソリューション構成を「Release」にする．
	(4) 「SIGService」のプロジェクト（test_libではない）のプロパティで，出力先のフォルダを設定し直す．
		例えば，「$(SolutionDir)$(Configuration)_2010\」を「$(SolutionDir)$(Configuration)_2013\」にする（その下の中間ファイル？の出力先も同様に変えておく）．
	(5) 「SIGService」のプロジェクト（test_libではない）のプロパティで，追加のライブラリディレクトリから，boostのライブラリ群のリンク先をVS2013用のフォルダに変更する． 
		例えば，「C:\SIGVerse\src\boost_1_55_0\lib32-msvc-10.0」を「C:\SIGVerse\src\boost_1_55_0\lib32-msvc-12.0」に変更する．
	(6) 以上の状態で，「SIGService」のプロジェクトをビルドする．
	(7) KinectV2_vs2013.slnのビルド時に，(5)で生成したSIGService.libをリンクする．


 (*)C:\SIGVerse\GitHub\ の直下にhttps://github.com/SIGVerse/Client.gitをクローンして，C:\SIGVerse\GitHub\Client\SIGService\Windows\ の SIGService_2010.sln がビルドできていて，VS2010用の SIGService.lib が Debug_2010\ か Release_2010\ のどちらかのフォルダに生成されていること，というのが前提です．

## Author  
 National Institute of Informatics  

## Wrote by  
 Nozaki, 12th 2015 March. 