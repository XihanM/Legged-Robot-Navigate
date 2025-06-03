# RL训练GO2翻越梅花桩
测试测试
## 安装

我的环境是Ubuntu 20.04.6 LTS，pytroch版本是2.4.1+cu121，主要编程环境为VS Code。

### 安装准备

从库中拉开源项目文件，已经放到github上了，可以用clone，失败的话也可以直接"Download ZIP"

```sh
git clone git@github.com:XihanM/Legged-Robot-Navigate.git
```

进入legged-robot-navigate文件夹，打开终端，启动VS Code

### 安装legged_gym，isaacgym

这里我们要开始安装isaacgym，作为本项目的仿真环境。

```sh
conda create -n legged_robot_parkour python=3.8
conda activate legged_robot_parkour
cd rsl_rl && pip install -e .
cd .. 
cd isaacgym/python && pip install -e .
cd .. 
cd ..
cd legged_gym && pip install -e .
cd ..
```

## 快速入门，运行第一个demo

主程序在下面的文件夹下

```sh
cd legged_gym/legged_gym/scripts
```

运行train.py文件，

```python
conda activate legged_robot_parkour
cd legged_gym/legged_gym/scripts
python train.py --task=go2 --num_envs=64 --headless --max_iterations=50
```

就说明开始正常训练了，可以通过设置--max_iterations来决定训练的轮数，这里方便起见选了--max_iterations=50。num_envs表示同时训练的数量，量力而行。

训练完成后，我们可以在legged_gym/logs/rough_go2找到这次训练保存的模型，这里的‘rough_go2’是默认的项目名称，要修改只需要添加--experiment_name修改为这次的名称，一般我会写本轮实验我要测试的奖励函数或者其他影响因素的名字。然后在-run_name写这个奖励值的具体数值。

对于已经训练好的模型，除了评估reward等量化指标，还需要用play.py在仿真环境中再测试一次。测试的代码如下

```sh
python play.py --task=go2 --num_envs=1  --checkpoint=50 --load_run=/path/to/your/project/legged_gym/logs/rough_go2/yourtime
```

现在，**你已经是个成熟的RLer了，能自己完成这个项目了**。

## 可视化训练

训练过程可以用tensorboard来可视化所有训练参数

```sh
tensorboard --logdir=/path/to/your/project/legged_gym/logs/xxx
```