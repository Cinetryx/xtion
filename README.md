えとしゃん
==========

このプログラムは人間のジェスチャー・ポーズを検出するプログラムです。  

  

概要
----
デジタルデバイスの操作には様々な方法があります。パソコンの操作にはキーボード(CUI)・マウス(GUI)などが代表的な例です。
しかし、これらのデバイスは人間の自然な動きとは言えません。キーボードはブラインドタッチなどができるようになるととても便利ですが、子供やパソコンを使い慣れていない人が使うのには不便です。

そこでCUI・GUIをさらに発展させ多ものとして登場したものがNUI（ナチュラルユーザーインターフェイス）です。身振り手振りや音といった、人間が日常生活の中で自然に行う動作でデバイスを操作するというものです。直観的で自然な動作なため、子供やパソコンに使い慣れていない人でも簡単にデバイスを操作することができます。

このプログラムはそのNUIの中でも、人間が行うジェスチャー・ポーズを検出する、というプログラムです。


機能
----
カメラの前に立った人間の頭や手などを追尾して、対象者があらかじめ登録しておいた指定のジェスチャー・ポーズをしたときにアクションを行います。
このプログラムでは、対象者のポーズに応じて、そのポーズに対応した絵本の表紙を表示します。


動作環境
--------
・Lubuntu12.04 (Ubuntu12.04, Xubuntu12.04)  
・Intel 64bit CPU の PC  
・Xtion Pro LIVE (Kinect)   
・OpenNI2  
・NiTE2  
・OpenCV  
※（）内のでも代用が可能です。  

使い方
------
OpenNI2とNiTE2をインストールしたPCでビルドを行います。  
xtion/でmakeを実行してください。  
xtion/Bin/x64-Release/下に作成されたSkeltonが実行ファイルです。  
XtionまたはKinectをPCにつないで実行してください。  
現時点では、腰より両手を上にあげた場合と、頭より両手を上にあげた場合で反応します。  

編集方法
--------
登録されたジェスチャー・ポーズを変更するためには、プログラムを変更します。  
main.cppのPose checkPose()関数に追記してください。  
```lang
        Pose checkPose(float *x3, float *y3, float *z3, cv::Mat& depthImage)
        {
            char checkedPose_c[15] = "NONE";
            Pose checkedPose = NONE;
            std::stringstream ss;

            if (y3[0] < y3[6] && y3[0] < y3[7]) {
                strcpy(checkedPose_c, "MOUNTAIN");
                checkedPose = MOUNTAIN;
                ehonnImage = cv::imread("./DaredaOre.jpg");
                imshow("Ehon", ehonnImage);
            }
            else if ((y3[8] < y3[6]) && (y3[8] < y3[7]) && (y3[2] > y3[6]) && (y3[3] > y3[7])) {
                strcpy(checkedPose_c, "OBAKE");
                checkedPose = OBAKE;
                ehonnImage = cv::imread("./Dareda.jpg");
                imshow("Ehon", ehonnImage);
            }
            else {
                cv::destroyWindow("Ehon");
            }
            
            ss << "Pose:" << checkedPose_c;
            cv::putText(depthImage, ss.str(), cv::Point(0, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
            return checkedPose;
        }
```
引数のx3[]、y3[], z3[]に各関節のxyz座標があります。  
これらを比較するなどして、ジェスチャーを登録してください。  
引数の配列の順番は次に示す配列と同じ順番になっています。  
```lang
char joint_name[15][15] = {"HEAD_HEAD",
                           "NECK_NECK",
                           "LEFT_SHOULDER",
                           "RIGT_SHOULDER",
                           "LEFT_ELBOW",
                           "RIGT_ELBOW",
                           "LEFT_HAND",
                           "RIGT_HAND",
                           "TORSO_TORSO",
                           "LEFT_HIP",
                           "RIGT_HIP",
                           "LEFT_KNEE",
                           "RIGT_KNEE",
                           "LEFT_FOOT",
                           "RIGT_FOOT"};
```

作成者アカウント
----------------
https://twitter.com/KTokunn  

作成者サイト
-------------
http://tokunn.hateblo.jp/  
