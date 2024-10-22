# 开发记录

### 2024.04.27: 
DONE: 

搭建了亮度计算的基本框架, 完成了以下函数的编写:

- 观测相角函数
- 散射定律中的相函数
- LSL散射定律的散射函数
- 基于入射角和出射角的可见性判断函数
- 形状面积与法向量计算函数
- DAMIT格式形状文件读取函数
- stl模型输出函数
- 基准系转固联系的旋转矩阵计算函数
- DAMIT格式的光变曲线文件读取函数

下一步的TODO:(按照顺序完成)

- [x] 完成基于形状的理论光变计算过程, 并进行验证
- [] 增加基于地平线判断的可见性判断函数, 并与上述的可见性判断函数共同使用
- [] ~~基于g2o的自转周期拟合程序, 使用自动求导, 并进行验证~~将g2o替换为ceres, 在函数拟合任务上ceres比g2o更合适
- [x] 理论光变计算过程代码优化, 使得代码可读性和封装性更高
- [ ] 增加基于obj模型的亮度计算模块

#### 2024.05.05
ceres换成线性求解器后, 能够在合适的区间内得到比较合理的自转周期结果, 这是一件很好的事. 

下一步TODO:

- [] 考虑如何将亮度计算部分以自动微分的方式实现(ceres::Jet), 有着自动微分, 后续想优化什么参数就优化什么参数.
- [] 对代码进行一些更改, 去掉没用的
- [x] 实现lambda、beta、P的同时优化(已完成, 但如果误差较大的话, 三者耦合在一起, 只能得到很不好的结果, 因此通常是固定某个参数， 然后再进行优化)
- [] 研究如何固定某个参数, 在优化时可以选择优化的参数

TinyOBJ好像不能将src和h分开, 会冲突

#### 2024.10.02
已完成射线与形状的交点计算, 并可以初步地可视化.

下一步TODO:

- [] 根据相机模型, 自动生成射线簇, 计算任意观测位置的可见性
- [] 根据太阳模型, 自动生成射线簇, 计算形状表面任意位置的是否可被照射到
- [x] ~~看看有没有什么加速的办法, 现在太慢了.~~ (已经实现BVH加速, 还可以进一步考虑更快的方法)