歩行者情報を持ったレイヤーを追加します。


```
$ cd catkin_ws/src
$ git clone [THIS REPO]

$ touch .rosinstall
$ wstool merge pedestrian_layers/.rosinstall
$ wstool update
$ rosdep install -i --from-path .
$ catkin build
```