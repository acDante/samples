#OculusRiftDK1_vs2010#

OculuRiftDK1_vs2010.sig を生成する手順を説明します．  

##必要な環境など##

OculuRiftDK1_vs2010.sln を使って OculuRiftDK1\_vs2010.sig をビルドするために，次の環境などが必要です．

* Microsoft Visual C++ 2010 Express（以降はVS2010と表記します）
* VS2010 用にビルドされた SIGService.lib  
* Oculus 用の SDK（OculusSDK）がインストールされていること（動作確認したバージョンは0.2.5）

##ビルドの手順##

ビルドする場合は，次の手順を参考にしてください．  

1. GitHub からクローンしてきた OculusRiftDK1\_vs2010.sln を VS2010 で開く．
2. プロジェクトのアクティブな構成を Release にする．
3. プロジェクトのプロパティを開いて，次の4～7に従ってインクルードとリンクの設定を行う．
4. 追加のインクルードディレクトリに，次のディレクトリを指定する．
 * SIGService.h があるディレクトリ
 * MirrorTherapy_v2 ディレクトリ
 * boost のディレクトリ
 * Oculus の SDK がインストールされているディレクトリ内にある「OculusSDK\LibOVR\Include」ディレクトリ  
5. 追加のライブラリディレクトリに，次のディレクトリを指定する．  
 なお，すべて VS2010 用のライブラリがあるディレクトリを指定してください．
 * SIGService.lib があるディレクトリ
 * boost のライブラリ群があるディレクトリ
 * Oculus の SDK がインストールされているディレクトリ内にある「OculusSDK\LibOVR\Lib\Win32」ディレクトリ
6. 追加の依存ファイルに，次の文字列を追加する．
 * SIGService.lib
 * libovr.lib （Debug 構成の場合は libovrd.lib）
 * winmm.lib （VS2010が正しくインストール済なら，ファイル自体は  
 C:\Program Files\Microsoft SDKs\Windows\ 以下のどこかにあります）
7. ターゲットの拡張子を「.exe」から「.sig」へ変更する．
8. ビルドする．

##VS2010用のSIGService.libについて##
OculusRiftDK1 を使うプラグインをビルドする場合， VS2010 で使える SIGService.lib が必要です．  

VS2010 用の SIGService.lib は，[ここ](http://www.sigverse.org/wiki/jp/?%E3%82%B5%E3%83%BC%E3%83%93%E3%82%B9%E3%83%A9%E3%82%A4%E3%83%96%E3%83%A9%E3%83%AA "サービスライブラリ")から DL できます．  
