#KinectV2_vs2013#

KinectV2_vs2013.sig を生成する手順を説明します．  

##必要な環境など##

KinectV2_vs2013.sln を使って KinectV2_vs2013.sig をビルドするために，次の環境などが必要です．

* Windows8 （またはそれ以上のWindows OS）
* Visual Studio Express 2013 for Desktop （以降はVS2013と表記します）
* VS2013 用にビルドされた SIGService.lib  
※ VS2013 用の SIGService.lib をビルドする手順は，末尾に記述してあります．
* OpenCV がインストールされていること（本リポジトリでは 2.4.10 を使用していますが，2.4系ならどれでもOKだと思います）
* Kinect for Windows SDK 2.0 がインストールされていること

なお，「VS2013 用の～～」と表記しているものは，筆者の環境で「VS2013 を使ってビルドしたもの」という意味を含んでいます．  
おそらくは，VS2012 でビルドしたものでもOKだと思います．  

##ビルドの手順##

ビルドする場合は，次の手順を参考にしてください．  

1. GitHub からクローンしてきた KinectV2_vs2013.sln を VS2013 で開く．
2. プロジェクトのアクティブな構成を Release にする．
3. プロジェクトのプロパティを開いて，次の4～7に従ってインクルードとリンクの設定を行う．
4. 追加のインクルードディレクトリに，次のディレクトリを指定する．
 * SIGService.h があるディレクトリ
 * MirrorTherapy_v2 ディレクトリ
 * boost のディレクトリ
 * KinectV2 の SDK がインストールされているディレクトリ内にある「inc」ディレクトリ  
  ※ SDK のバージョンによって，インストールされる（作成される）ディレクトリ名が変わる様です．  
基本的には，「C:\Program Files\Microsoft SDKs\Kinect\」ディレクトリ以下のどこかにあると思います．
 * OpenCV の include ディレクトリ
5. 追加のライブラリディレクトリに，次のディレクトリを指定する．  
 なお，すべて VS2013 用のライブラリがあるディレクトリを指定してください．
 * SIGService.lib があるディレクトリ
 * boost のライブラリ群があるディレクトリ
 * OpenCV のライブラリ群があるディレクトリ
 * KinectV2 の SDK のライブラリ（kinect20.lib）があるディレクトリ
6. 追加の依存ファイルに，次の文字列を追加する．
 * SIGService.lib
 * kinect20.lib
 * opencv\_highgui*.lib
 * opencv\_core*.lib
 * opencv\_imgproc*.lib  
 ※「\*」の部分は，インストールしてある OpenCV のバージョンに合わせて変更してください．
7. ターゲットの拡張子を「.exe」から「.sig」へ変更する．
8. ビルドする．
9. OpenCV がインストールされているディレクトリ（デフォルトではC:\opencv\build\x86\vc12\bin）から，  
次の3つのファイルを Release ディレクトリへコピーする．
 * opencv\_core\*.dll
 * opencv\_highgui\*.dll
 * opencv\_imgproc\*.dll  
 ※「\*」の部分は，インストールしてある OpenCV のバージョンに合わせて変更してください．

##VS2013用のSIGService.libについて##
KinectV2 を使うプラグインをビルドする場合， VS2013 で使える SIGService.lib が必要です．  

稲邑先生の研究室の関係者の方は，VS2013 でビルドした SIGService.lib を以下からコピーできます．

* \\tera2.iir.nii.ac.jp\share\tome-nozaki\SIGServiceLibs

SIGService.lib を VS2013 で改めてビルドする際は，以下をご覧ください．

###VS2013用のSIGService.libをビルドする###
  
SIGService.lib をビルドするためには， boost のライブラリも VS2013 用のものが必要です．  
SIGService.lib をビルドする例は次の通りです．

1. http://sourceforge.net/projects/boost/files/boost-binaries/1.55.0-build2/  
の「boost\_1\_55\_0-msvc-12.0-32.exe」を DL してインストールする．
2. https://github.com/SIGVerse/Client リポジトリを C:\SIGVerse\GitHub\ 直下に Clone する．
3. SIGService_2010.sln を VS2013 で開いて，　boost 周りの設定を 1. でインストールした boost のものにする．
4. （VS2010 用の SIGService.lib をビルドしたことがある場合は，既存のものを上書きしてしまわないように，  
出力先の Release ディレクトリを退避しておく．）
5. ビルドする．
6. ビルド結果が Release ディレクトリに出力されていることを確認する．
7. Release ディレクトリの名称を Release_2013 へ変更する．

私がGitへあげたプロジェクトファイルには，この手順で生成した SIGService.lib を使用する設定になっています．
