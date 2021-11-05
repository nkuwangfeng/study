### 标题: LiDAR Inertial Odometry Aided Robust LiDAR Localization System in Changing City Scenes

### 作者: Wendong Ding, Shenhua Hou, Hang Gao, Guowei Wan, Shiyu Song

### 来源: 2020 IEEE International Conference on Robotics and Automation (ICRA) 

### 摘要:  介绍一篇关于激光-惯性里程计（LIO）的文章，亮点是使用LIO提高激光全局定位的精度和长期定位能力, 尤其针对动态变化的城市场景.

### 目录

* 主要内容
* 研究背景
* 方法
  * 激光惯性里程计LIO
  * 位姿图融合
  * 环境变化检测

* 实验
* 总结

#### 一, 主要内容:

* 本文整合了LIO和基于LiDAR定位模块。两个模块的测量都被融合到一个位姿图优化框架中。

* 二者的互补性质使得在环境产生变化或地图有误差的情况下也能够实现鲁棒的定位。

#### 二, 研究背景与相关工作

* 在动态变化的环境中实现准确的长期定位是一件困难的事情。

* 之前的工作[2]-[5]证实了一些环境的特定变化可以被用现存技术克服，如人行横道重铺，水坑，雪地等。但是在一般的动态场景中实现长期定位仍然具有很大挑战。

* 本文通过融合互补的LIO和LiDAR全局定位两个模块实现动态场景中的长期定位。

#### 2.1 长期定位(GMM等, 直方图滤波等)

#### 2.2 LIO(LOAM,icp,ndt,csm等)

#### 2.3 定位融合(eskf, graph-based, 松耦合, 紧耦合等)

#### 三, 方法

* 方法框图如下所示，包含四个模块：激光惯性里程计（LIO），激光全局匹配（LGM），位姿图融合（PGF）和环境变换检测（ECD）。

* LGM模块基于Wan等人的工作[5]，其能够基于先验地图得到（x,y,yaw）的三自由度估计结果，剩余三个自由度（roll，pitch和z）通过IMU重力测量读数和数字高程模型（DEM）地图得到。

* LIO模块基于Cartographer，但是进行了大量的扩展以提高精度。

![image-20211124153532476](/home/wf/.config/Typora/typora-user-images/image-20211124153532476.png)

#### 3.1 激光惯性里程计

* 激光惯性里程计（LIO）模块是基于Cartographer的改进。

* 首先，作者扩展2D占据栅格到3D占据栅格以实现6DoF的里程计。

* 其次，作者使用IMU提供运动预测以克服点云的运动畸变，并使用IMU预积分作为两帧之间的运动约束。

* 最后，考虑到车道线和路牌提供的丰富信息，作者在栅格中添加激光intensity信息。

* 为了实现精度和速度的平衡，作者采用了多分辨率栅格以从粗到精的方式进行优化。

作者构建LIO模块为一个MAP估计问题：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+P%5Cleft%28%5Cmathbf%7Bx%7D_%7Bk%7D%5E%7BL%7D+%5Cmid+%5Cmathbf%7Bz%7D_%7Bk%7D%2C+%5Cmathbf%7Bx%7D_%7Bk-1%7D%5E%7BL%7D%2C+%5Cmathcal%7BS%7D_%7Bk-1%7D%5Cright%29+%5Cpropto+%26+P%5Cleft%28%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BP%7D+%5Cmid+%5Cmathbf%7Bx%7D_%7Bk%7D%5E%7BL%7D%2C+%5Cmathcal%7BS%7D_%7Bk-1%7D%5Cright%29+%5C%5C+%26+P%5Cleft%28%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BI%7D+%5Cmid+%5Cmathbf%7Bx%7D_%7Bk%7D%5E%7BL%7D%2C+%5Cmathbf%7Bx%7D_%7Bk-1%7D%5E%7BL%7D%5Cright%29+%5Cend%7Baligned%7D) (1)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Bz%7D_%7Bk%7D%3D%5Cleft%5C%7B%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BP%7D%2C+%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BI%7D%5Cright%5C%7D) ， ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BP%7D) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BI%7D) 分别是LiDAR点云和IMU惯性测量。 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Bx%7D_%7Bk%7D%5E%7BL%7D) 表示 ![[公式]](https://www.zhihu.com/equation?tex=k) 时刻的状态，包含6DoF位姿，速度和IMU偏置。

假设零均值高斯噪声，测量的似然函数可以被定义如下：

![[公式]](https://www.zhihu.com/equation?tex=P%5Cleft%28%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BI%7D+%5Cmid+%5Cmathbf%7Bx%7D_%7Bk%7D%5E%7BL%7D%2C+%5Cmathbf%7Bx%7D_%7Bk-1%7D%5E%7BL%7D%5Cright%29+%5Cpropto+%5Cexp+-%5Cfrac%7B1%7D%7B2%7D%5Cleft%5C%7C%5Cmathbf%7Br%7D_%7Bk%7D%5E%7BI%7D%5Cright%5C%7C_%7B%5CLambda_%7Bk%7D%5E%7BI%7D%7D%5E%7B2%7D) (2)

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+P%5Cleft%28%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BP%7D+%5Cmid+%5Cmathbf%7Bx%7D_%7Bk%7D%5E%7BL%7D%2C+%5Cmathcal%7BS%7D_%7Bk-1%7D%5Cright%29+%5Cpropto+%26+%5Cprod_%7Bi%7D+%5Cprod_%7Bj%7D+%5Cexp+-%5Cfrac%7B1%7D%7B2+%5Csigma_%7Bo_%7Bi%7D%7D%5E%7B2%7D%7D%5C%7C%5Cmathrm%7BSSOP%7D%5C%7C%5E%7B2%7D+%5C%5C+%26+%5Cprod_%7Bi%7D+%5Cprod_%7Bj%7D+%5Cexp+-%5Cfrac%7B1%7D%7B2+%5Csigma_%7Br_%7Bi%7D%7D%5E%7B2%7D%7D%5C%7C%5Coperatorname%7BSSID%7D%5C%7C%5E%7B2%7D+%5Cend%7Baligned%7D) (3)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Br%7D_%7Bk%7D%5E%7BI%7D) 是IMU预积分残差。

![[公式]](https://www.zhihu.com/equation?tex=i) 遍历栅格的所有分辨率， ![[公式]](https://www.zhihu.com/equation?tex=j) 遍历所有的激光点。占据概率的平方和（Sum of Squared Occupancy Probability, SSOP）和密度差的平方和（Sum of Squared Intensity Difference, SSID）的具体定义如下：

![[公式]](https://www.zhihu.com/equation?tex=%5Cleft%5C%7B%5Cbegin%7Barray%7D%7Bl%7D+%5Cmathrm%7BSSOP%7D%3D1-P%28s%29+%5C%5C+%5Cmathrm%7BSSID%7D%3D%5Cfrac%7Bu_%7Bs%7D-%5Cmathrm%7BI%7D%5Cleft%28%5Cmathbf%7Bp%7D_%7Bj%7D%5Cright%29%7D%7B%5Csigma_%7Bs%7D%7D+%5Cend%7Barray%7D%5Cright.) (4)

其中 ![[公式]](https://www.zhihu.com/equation?tex=P%28s%29) 是子地图中hit栅格的占据概率， ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathrm%7BI%7D%5Cleft%28%5Cmathbf%7Bp%7D_%7Bj%7D%5Cright%29) 是激光点 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Bp%7D_%7Bj%7D) 的intensity， ![[公式]](https://www.zhihu.com/equation?tex=u_%7Bs%7D) 和 ![[公式]](https://www.zhihu.com/equation?tex=%5Csigma_%7Bs%7D) 是hit栅格中LiDAR点的intensity的均值和方差。

* 具体通过三次插值来从submap里获取概率, 另外![image-20211124163451863](/home/wf/.config/Typora/typora-user-images/image-20211124163451863.png) 是一个不同分辨率下的权重.

* 最大后验概率后会插入当前帧完成submap的更新. 增量更新使用二值贝叶斯下inverse measurement model and the log odds ratio 来完成更新.

* 对最大后验问题取负对数后，可以转化为非线性最小二乘问题，本文使用非线性最小二乘求解器Ceres[32]求解。

#### 3.2  位姿图融合

位姿图融合（PGF）模块融合LIO和激光全局定位（LGM）模块到一个位姿图中，对应的贝叶斯网络如下所示：

![image-20211124164556036](/home/wf/.config/Typora/typora-user-images/image-20211124164556036.png)

数学上，该模块可以被构建为一个MAP问题如下：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+P%28%5Cmathcal%7BX%7D+%5Cmid+%5Cmathcal%7BZ%7D%29+%5Cpropto+%5Cprod_%7Bk%2C+s%7D+P%5Cleft%28%5Cmathbf%7Bz%7D_%7Bk+s%7D%5E%7BO%7D+%5Cmid+%5Cmathbf%7Bx%7D_%7Bk%7D%5E%7BL%7D%2C+%5Cmathbf%7Bx%7D_%7Bs%7D%5E%7BS%7D%5Cright%29+%26+%5Cprod_%7Bk%7D+P%5Cleft%28%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BI%7D+%5Cmid+%5Cmathbf%7Bx%7D_%7Bk%7D%5E%7BL%7D%2C+%5Cmathbf%7Bx%7D_%7Bk-1%7D%5E%7BL%7D%5Cright%29+%5C%5C+%26+%5Cprod_%7Bk%7D+P%5Cleft%28%5Cmathbf%7Bz%7D_%7Bk%7D%5E%7BG%7D+%5Cmid+%5Cmathbf%7Bx%7D_%7Bk%7D%5E%7BL%7D%2C+%5Cmathbf%7Bx%7D_%7BL%7D%5E%7BG%7D%5Cright%29+%5Cend%7Baligned%7D) (5)

激光和IMU测量似然，LGM测量似然如下：

![image-20211124164836991](/home/wf/.config/Typora/typora-user-images/image-20211124164836991.png)

其中 ![[公式]](https://www.zhihu.com/equation?tex=%5Cmathbf%7Br%7D_%7Bk+s%7D%5E%7BO%7D)是全局定位残差。

##### 3.2.1 全局定位残差与协方差计算

![image-20211124174007749](/home/wf/.config/Typora/typora-user-images/image-20211124174007749.png)



估计协方差可以用来自适应融合. z和角度的协方差设置为常数, x,y的协方差计算来自于LGM.

##### 3.2.2 里程计的残差和协方差计算

![image-20211124174748672](/home/wf/.config/Typora/typora-user-images/image-20211124174748672.png)

协方差均设为常数.

##### 3.2.3 IMU预积分残差和协方差

![image-20211124164450116](/home/wf/.config/Typora/typora-user-images/image-20211124164450116.png)

#### 3.3 环境变化检测

LIO模块产生的子地图投影到地面平面，产成2D栅格子地图后全局2D栅格地图进行比较以检测变化。

记 ![[公式]](https://www.zhihu.com/equation?tex=u_%7Bs%7D%2C+%5Csigma_%7Bs%7D%2C+a_%7Bs%7D) 和 ![[公式]](https://www.zhihu.com/equation?tex=u_%7Bm%7D%2C+%5Csigma_%7Bm%7D%2Ca_m) 分别为LIO构建产生的2D栅格子地图和全局2D栅格地图中对应栅格的intensity均值，方差和高度均值。基于此得到

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Barray%7D%7Bl%7D+z_%7Bs%7D%28r%29%3D%5Cfrac%7B%5Cleft%28u_%7Bs%7D-u_%7Bm%7D%5Cright%29%5E%7B2%7D%5Cleft%28%5Csigma_%7Bs%7D%5E%7B2%7D%2B%5Csigma_%7Bm%7D%5E%7B2%7D%5Cright%29%7D%7B%5Csigma_%7Bs%7D%5E%7B2%7D+%5Csigma_%7Bm%7D%5E%7B2%7D%7D+%5C%5C+z_%7Bs%7D%28a%29%3D%5Cleft%28a_%7Bs%7D-a_%7Bm%7D%5Cright%29%5E%7B2%7D+%5Cend%7Barray%7D) (7)

用于评估栅格内的环境变化程度。作者构建每个栅格内的变化为一个二值状态估计问题，并使用二值贝叶斯滤波器解决[31]。其中d是一个栅格改变的二值状态参数.

![image-20211124175459053](/home/wf/.config/Typora/typora-user-images/image-20211124175459053.png)

### 四, 实验

* 实验使用了内部数据集和Apollo-SouthBay数据集[33][34]，包含Velodyne HDL-64E LiDAR，GNSS  RTK和一个IMU。

* 真值轨迹由基于全局最小二乘优化的离线LiDAR  SLAM方法得到。

* 评测指标包含水平和朝向误差，其中水平误差又进一步被划分为纵向（Long.）和横向（Lat.）误差。

* 此外，水平误差和朝向误差低于0.1m，0.2m，0.3m和0.1度的帧比例也作为评测指标。

![image-20211124180646787](/home/wf/.config/Typora/typora-user-images/image-20211124180646787.png)

![image-20211124180850023](/home/wf/.config/Typora/typora-user-images/image-20211124180850023.png)

![image-20211124181208625](/home/wf/.config/Typora/typora-user-images/image-20211124181208625.png)

* 实时性: LGM，LIO和PFG模块的平均运行时间分别为44.9ms，79.4ms和2.8ms。注意到LGM(FPGA)和LIO并行运行，因此整个系统大概每帧需要运行82ms，满足实时性要求。

* LIO模块的精度评估采用KITTI里程计的评估标准。平移和旋转误差分别为0.9309%和0.0057deg/m。(定量判断了多分辨率和强度信息的作用)

![image-20211124181021754](/home/wf/.config/Typora/typora-user-images/image-20211124181021754.png)

* 环境变化检测与地图与更新

  ![image-20211124181332343](/home/wf/.config/Typora/typora-user-images/image-20211124181332343.png)

### 五, 总结

* 核心思想是利用位姿图优化框架融合LIO和LiDAR全局定位模块，以完成动态变化场景中的长期定位。
* LiDAR全局定位的精度高于LIO，但是在环境发生变化时，LiDAR全局定位模块可能产生较差的定位精度；而LIO虽然定位精度低于LiDAR全局定位，但是它不依赖先验地图，因此精度不随着环境变化而降低。
* 作者正是利用了二者的互补性质，解决了动态变化场景中的准确定位问题。
* 此外，作者在总结中提到了未来可能探索基于学习的环境变化检测方法。











