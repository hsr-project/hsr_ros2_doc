######
概要
######

本章では、 `yolox_ros <https://github.com/Ar-Ray-code/YOLOX-ROS>`__ による物体認識と、 `graspnet_ros <https://github.com/hsr-project/graspnet_ros>`__ による把持姿勢推定とを組み合わせ、
推定された把持位置・姿勢を用いて `hsrb_interface <https://github.com/hsr-project/hsrb_interfaces/tree/humble>`__ でPick & Placeを実行する環境一式について説明します。

本環境は ``yolox_ws`` と ``hsrb_pnp_ws`` の2つのDocker環境から構成されます。
それぞれの役割を以下に示します。

yolox_ws
===========

+ ``yolox_ros`` を用いて、画像シーケンスから把持対象物を検出します。

+ ``compressedImage`` 、 ``compressedDepth`` を入力として取得することで、データ伝送の高速化を図っています。

+ ``yolox_ros`` の検出結果を ``graspnet_ros`` に入力し、把持位置・姿勢を推定して出力します。

hsrb_pnp_ws
==============

+ 推定された把持位置・姿勢をもとに、 ``hsrb_interface`` を用いてPick & Placeを実行します。