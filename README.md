# 多激光雷达自主建图及实时定位

## 目录

* [amcl对多激光雷达的修改](#amcl对多激光雷达的修改)
* [一些操作](#一些操作)
    * [如何编译？](#如何编译？)
    * [如何显示gazebo的GUI？](#如何显示gazebo的GUI？)
    * [如何运行？](#如何运行？)
* [遗留问题](#遗留问题)



## amcl对多激光雷达的修改

修改激光雷达接收话题scan_1和scan_2数据，在amcl观测更新部分对两个激光雷达数据处理。
采用两个雷达数据共同更新粒子权重。


## 劫持检测与全局重定位实现

订阅amcl_pose话题，在amcl_pose回调函数中处理数据，首先拿最新的两次激光对地图进行扫描匹配，看匹配的残差大小，
残差过大，则认定定位失效，采用分支定界算法进行全局重定位。分支定界算法找到的位置如果score大于0.7，则发布b_b_pose话题发布扫描匹配位置，
amcl里面订阅这个话题，得到位置，在位置附近重新初始化粒子


## 一些操作

### 如何编译？

首先配置好环境，cartographer和amcl的环境

```
mkdir -r ~/ros1_ws/src
cd ~/ros1_ws/src
git clone https://github.com/adayimaxiga/double_laser_mapping_location.git
cd ..
catkin_make_isolate
```

### 如何显示gazebo的GUI？


```
~/ros1_ws/src/gazebo_environment/turtlebot3_gazebo/launch
```
目录下有turtlebot3_world.launch
这个文档，打开这个文档，中间有一行gui选项改称true
```
<arg name="gui" value="true"/>
```

### 如何运行？

当然是不用一个一个run了，机智的我已经洗干净....
哦不，写好了脚本文件，文件名就是功能无需介绍
在哪呢？
```
~/ros1_ws/src/tools/scripts
```




## 遗留问题

！！！！最关键的遗留问题: loop_closer_check单词打错了..............羞耻羞耻，丢了我交的人，我交还是很强的，是我弱。。。
代码里面已经改成了loop closure，但是这里改了的话麻烦太多，
还要改package.xml和我写的脚本文件，麻烦麻烦

### 定位失效检测数据不同步问题

之前为了图省事，直接拿最新的激光数据来做扫描匹配了，这样就有激光超前与amclpose
的问题，修改方法就如下:

1. loop节点里接收激光数据放在一个队列里，激光数据要保存时间戳
2. amcl_pose应该发布laser的时间
3. loop节点里根据amcl_pose时间来在激光序列中寻找时间最近的激光做扫描匹配
4. 如果失效，全局重定位用最新的激光数据来做branch and bound


2018.8.31 LD