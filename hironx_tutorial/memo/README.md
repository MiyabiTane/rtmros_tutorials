# 学んだことメモ<br>

## hironxjskがお弁当を詰めるデモ<br>
### [EusLisp内で完結しているデモ](https://gist.github.com/MiyabiTane/6d19dd7c44314a350444fd2f8b88fd75)<br>
＜実行の仕方＞<br>
```
$ roscore
```
```
$ source ~/ros/jsk_rtmros_ws/devel/setup.bash 
$ roscd hiro_lunch_box/src
$ roseus demo.l
$ (main)
```
### gazebo内におかずを出現させる<br>
[このページ](https://github.com/MiyabiTane/jsk_model_tools/tree/add_by_tanemoto/eusurdf/new_models)を参考にしてeusモデルをgazebo内に出せるかたちに変換する。<br>
次に、package.xmlの\<export>の部分に書き加える。<br>
```
<export>
  <rosdoc config="rosdoc.yaml" />
  <!-- use my models -->
  <gazebo_ros gazebo_model_path="${prefix}/models" />
</export>
```
.最後に、worldファイルに物体を書き加える。<br>
```
<include>
    <name>takosan-wiener</name>
    <uri>model://takosan-wiener</uri>
    <!-- my models appear -->
    <pose>0.0 0 0  0 0 0</pose>
</include>
```

## hironxjskが積み木をするデモ<br>
```
$ source ~/ros/jsk_rtmros_ws/devel/setup.bash
$ roslaunch hironx_tutorial demo_tanemoto.launch
```
```
$ source ~/ros/jsk_rtmros_ws/devel/setup.bash
$ rosrun hironx_tutorial demo_tanemoto.l
$ (main)
```
### gazebo空間の物体設定とhsiフィルターの適応
demo_tanemoto.launch内に追記する。<br>
まず、物体を出現させるにはdemo_cube_red（nextageが赤い物体を取る、参照）をつくって<br>
```
<node name="spawn_demo_cube_red" pkg="gazebo_ros" type="spawn_model"
          args="-file $(find hironx_tutorial)/models/demo_cube_red/model.sdf -sdf
                -x 0.5 -y 0.3 -z 0.82 -model demo_cube_red" />
```
とすれば良い。また、hsiフィルターをかけるには
```
<!-- add launch -->
  <include file="$(find jsk_pcl_ros)/launch/hsi_color_filter.launch">
    <arg name="INPUT" value="/head_camera/depth_registered/points"/>
  </include>
```
とすればよい。<br>

## nextageが赤い物体をとる<br>
＜コード＞https://gist.github.com/MiyabiTane/704f6cba572669c33362fda13ec76984<br>
### nextageのgazebo環境をつくる<br>
```
$ mkdir -p ~/ros/ws_nextage_tutorials/src
$ cd ~/ros/ws_nextage_tutorials/src
$ wstool init
$ wstool merge https://gist.githubusercontent.com/k-okada/94b7140e4c9fbf800f77c506a2ddb8f6/raw/4f869623bbf87eb8e8cf4cb5fd0d84ac3f25d5c2/.rosinstall
$ wstool update
$ cd ..
$ rosdep install -r --from-paths src --ignore-src
$ catkin build
```
```
$ source ~/ros/ws_nextage_tutorials/devel/setup.bash
$ roslaunch nextage_tutorials demo_tanemoto.launch
```
```
$ source ~/ros/ws_nextage_tutorials/devel/setup.bash
$ rosrun nextage_tutorials demo_tanemoto.l  
```

### gazebo内に物体を出現させる<br>
~/ros/ws_nextage_tutorials/src/rtmros_tutorials/nextage_tutorials/launch/demo_tanemoto.launch内の<br>
``` launch
 <include file="$(find nextage_gazebo)/launch/nextage_world.launch" >
    <arg name="gzpose" value="-x 0.01 -y 0.01 -z 0.1" />
    <arg name="model" value="$(find nextage_tutorials)/model/NextageKinect.xacro" />
    <arg name="world_file" value="$(find nextage_tutorials)/model/demo_tanemoto.world" />
  </include>
```
部分に注目する。../models/demo_tanemoto.worldがgazebo内の環境を作っているファイルなのでさらにその中を見てみる。<br>
```
<include>
    <uri>model://demo_cube_red</uri>
    <pose>1.0 0.005 0.83 0 0 0</pose>
</include>
```
のようにすれば新たな物体demo_cube_redを出現させることができる。<br>
demo_cube_redについては~/ros/ws_nextage_tutorials/src/rtmros_nextage/nextage_gazebo/models/demo_cube_redフォルダを作成し、中にmodel.configとmodel.sdfを入れればよい。<br>
物体の色や形状を変えるにはmodel.sdfをいじればよい。<br>

### BoundingBoxを出現させる<br>
```
$ source ~/ros/ws_nextage_tutorials/devel/setup.bash
$ roslaunch nextage_tutorials demo_tanemoto.launch
```
```
$ roslaunch jsk_pcl_ros hsi_color_filter.launch INPUT:=/head_camera/depth_registered/points
```
```
$ source ~/ros/ws_nextage_tutorials/devel/setup.bash
$ rosrun nextage_tutorials demo_tanemoto.l  
```
とすれば/HSI_color_filter/boxesに結果が出力される。<br>

特定の色のboxが欲しい時は
```
rosrun rqt_reconfigure rqt_reconfigure
```
で調節する。<br>

### gazebo空間を明るくする
gazeboを照らさないと色付き点群にならないことがあるのでgazebo空間を明るくしておく。<br>
先程の.worldファイルに以下を追加する。
```
<scene>
	<shadows>false</shadows>	
</scene>	
```

## コマンドメモ<br>
<Emacs立ち上げ><br>
・M-x shell -> euslisp使うとき<br>
・Ctrl+c b -> 新規作成<br>
<ターミナルの終了><br>
・Ctrl+z -> 今すぐ強制終了せよ（危険）<br>
・Ctrl+c -> 適切に処理して終了<br>
<便利なもの><br>
・Ctrl+R -> これをしてから入力すると最近打った文字がでてくる<br>

## Gitメモ<br>
https://github.com/start-jsk/rtmros_hrp2
のページをForkしたあと、
```
cd ~/ros/jsk_hiro_ws/src/rtm-ros-robotics/rtmros_tutorials
git remote add MiyabiTane https://github.com/MiyabiTane/rtmros_tutorials
```
すれば自分のところにリポジトリを作ることができる。<br>
あとはいつも通りgit add, git commit,して<br>
```
git push MiyabiTane <ブランチ名>
```





