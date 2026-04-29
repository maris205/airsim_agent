# 第七章 多无人机Agent协同

## 6.1 多Agent系统简介

### 6.1.1 什么是智能体（Agent）

在人工智能领域，"智能体"（Agent）是一个能够感知环境、做出决策并采取行动的自主实体。一个完整的智能体通常包含以下几个核心组件：

- **感知模块**：从环境中获取信息。对于无人机而言，这包括GPS定位、惯性测量单元（IMU）、摄像头、激光雷达等传感器数据。
- **决策模块**：根据感知到的信息和内部目标，选择下一步的行动。传统方法使用规则引擎或强化学习策略；而在本章中，我们使用大型语言模型（LLM）作为决策核心。
- **执行模块**：将决策转化为具体的物理动作，例如控制电机转速、调整飞行姿态、改变航向等。
- **通信模块**：与其他智能体或人类操作员交换信息。

用一个生活中的类比来理解：一个外卖骑手就是一个智能体。他通过手机App感知订单信息和导航路线（感知），根据距离和时间决定先送哪一单（决策），骑着电动车前往目的地（执行），并通过电话与顾客沟通（通信）。

当我们把大型语言模型引入智能体的决策模块时，就得到了所谓的"LLM Agent"。LLM的优势在于它能够理解自然语言描述的任务目标，进行推理和规划，并生成结构化的执行指令。例如，当我们告诉一个LLM Agent"去检查那座风力发电机的叶片状况"时，它能够自主规划飞行路径、选择合适的观察角度，并用自然语言汇报检查结果。

### 6.1.2 从单Agent到多Agent系统

单个智能体的能力终究有限。一架无人机的电池续航通常只有20~40分钟，摄像头视野有限，一次只能执行一个任务。当面对大范围搜索、多目标同时监控、复杂环境巡检等任务时，单架无人机往往力不从心。

多智能体系统（Multi-Agent System, MAS）通过让多个智能体协同工作来突破单体的局限。这就像从"一个人干活"变成"一个团队协作"——每个成员各司其职，整体效率远超个体之和。

多无人机协同的典型应用场景包括：

**（1）搜索与救援**

灾难发生后，需要在最短时间内搜索大面积区域。如果只派一架无人机，可能需要数小时才能覆盖整个区域；而派出一个由5~10架无人机组成的编队，每架负责一个子区域，搜索时间可以缩短到原来的几分之一。2023年土耳其地震救援中，多架无人机协同搜索就发挥了重要作用。

**（2）农业植保**

大面积农田的喷洒作业是多无人机协同的成熟应用。多架植保无人机按照预设航线分区作业，通过实时通信避免重复喷洒和遗漏区域。中国的大疆、极飞等企业已经在这一领域实现了商业化运营。

**（3）物流配送**

在偏远地区或紧急情况下，多架无人机可以组成配送网络。每架无人机负责一段航程，通过"接力"的方式将货物送达远距离目的地，突破单架无人机航程限制。

**（4）工业巡检**

电力线路、风力发电场、石油管道等基础设施的巡检工作量巨大。多架无人机可以同时对不同设施进行检查，一架在高空全局监控，其余在低空近距离拍照取证。这正是本章实验所模拟的场景。

**（5）编队表演**

大型活动中的无人机灯光秀需要数百甚至上千架无人机精确协同。每架无人机都是一个智能体，需要在保持编队队形的同时避免碰撞。

### 6.1.3 多Agent协同的核心挑战

要让多个智能体真正协同工作，而不是各自为政甚至互相干扰，必须解决以下几个根本性挑战：

#### 1. 任务分配与分解

如何将一个高层级的任务目标分解为一系列可执行的子任务，并将这些子任务合理地分配给各个智能体？

PLACEHOLDER_MORE_CONTENT

以"搜索一片10平方公里的森林寻找失踪徒步者"为例：系统需要将整个区域划分为若干子区域，然后根据每架无人机的电量、位置、传感器能力等因素，将子区域分配给最合适的无人机。这个问题在数学上属于组合优化问题，当无人机数量和任务数量增大时，最优分配方案的搜索空间会呈指数级增长。

在传统方法中，任务分配通常使用拍卖算法（Auction Algorithm）、匈牙利算法等优化方法。而在本章中，我们将展示一种全新的思路：让LLM理解任务描述，直接输出分配方案。虽然LLM的方案不一定是数学上的最优解，但它能够处理自然语言描述的模糊任务，这是传统算法难以做到的。

#### 2. 通信与信息共享

智能体之间需要交换哪些信息？交换的频率如何？通信带宽有限时如何取舍？

在理想情况下，每架无人机都能实时共享自己的完整状态（位置、速度、电量、传感器数据等）。但在现实中，无线通信带宽有限、存在延迟和丢包。因此，通信协议的设计需要在"信息完整性"和"通信开销"之间找到平衡。

本章的实验中，我们用一个Python列表（消息板）来模拟无人机之间的无线通信。虽然简化了很多，但核心思想是一致的：每架无人机将自己的关键状态发布到共享空间，其他无人机可以读取这些信息来辅助决策。

#### 3. 冲突消解与安全

当两架无人机同时决定飞往同一个目标时怎么办？当飞行路径交叉时如何避免碰撞？

冲突消解是多无人机系统中最关键的安全问题。常见的策略包括：

- **优先级机制**：为每架无人机分配优先级，低优先级的无人机主动避让。
- **协商机制**：冲突双方通过通信协商，一方改变路径或等待。
- **地理围栏**：为每架无人机划定专属的飞行区域，从物理上避免冲突。

在本章的分布式协同实验中，我们通过消息板实现了一种简单但有效的冲突消解：Drone2在决策时能看到Drone1已经选择了某个目标，因此会自动避开，选择另一个目标。

#### 4. 可扩展性

系统能否从2架无人机平滑扩展到20架、200架？

这是架构设计中的重要考量。中心化架构在小规模时简单高效，但当无人机数量增加时，中央节点会成为瓶颈。分布式架构天然具有更好的可扩展性，但协调的复杂度也随之增加。层级式架构则试图在两者之间找到平衡——通过分层管理来兼顾全局协调和局部自主。

### 6.1.4 三种协同架构

根据决策权的分布方式，多无人机协同架构可以分为三大类：

#### 模式一：中心化协同（Centralized）

一个"指挥官"统一决策，所有无人机听从指挥。

```
人类操作员 → [指挥官LLM] → 任务分解 → 分别指挥Drone1、Drone2...
```

这种架构类似于军队中的指挥体系：指挥官掌握全局信息，制定作战计划，各部队按命令执行。优点是决策质量高（因为有全局信息）、实现简单；缺点是指挥官一旦失效，整个系统瘫痪（单点故障），而且所有通信都要经过指挥官，容易形成瓶颈。

#### 模式二：分布式协同（Distributed）

每架无人机有自己的"大脑"，通过共享信息协调行动。

```
Drone1 [自己的LLM] ←→ 消息板 ←→ [自己的LLM] Drone2
```

这种架构类似于一个自组织的团队：没有固定的领导者，每个成员根据共享的信息独立做出决策。优点是没有单点故障、可扩展性好；缺点是难以保证全局最优，可能出现决策冲突。

#### 模式三：层级协同（Hierarchical）

结合前两种模式的优点：上层有指挥官负责全局规划，下层的执行者有一定的自主权。

```
        [监控指挥官]
       ↙          ↘
[巡检员A]      [巡检员B]
  自主执行        自主执行
```

这种架构类似于公司的组织结构：CEO制定战略方向，部门经理在各自领域内自主决策。它在全局协调和局部灵活性之间取得了平衡，是工业巡检、搜救等实际应用中最常用的架构。

三种架构的对比如下：

| 对比项 | 中心化 | 分布式 | 层级式 |
|--------|--------|--------|--------|
| 决策方式 | 一个LLM统一规划 | 每架无人机各自决策 | 分层决策 |
| 通信模式 | 指挥官→工作者（单向） | 消息板（双向共享） | 上下级双向 |
| 容错性 | 低 | 高 | 中等 |
| 可扩展性 | 差 | 好 | 较好 |
| 全局最优 | 容易实现 | 较难保证 | 较好 |
| 实现复杂度 | 简单 | 中等 | 中等偏高 |
| 典型应用 | 小规模编队 | 大规模蜂群 | 工业巡检 |

### 6.1.5 LLM驱动的多Agent系统：新范式

传统的多无人机协同系统依赖于预编程的规则和算法。例如，任务分配使用拍卖算法，路径规划使用A*或RRT算法，编队控制使用虚拟结构法或领航-跟随法。这些方法在结构化、可预测的环境中表现出色，但面对开放性、模糊性的任务描述时往往束手无策。

大型语言模型的出现为多Agent系统带来了一种全新的范式。LLM的核心优势在于：

**（1）自然语言理解**

传统系统需要将任务编码为数学形式（如坐标列表、代价函数等），而LLM可以直接理解"去检查那座风力发电机有没有损坏"这样的自然语言指令，并将其转化为具体的飞行计划。

**（2）常识推理**

LLM在训练过程中积累了大量的世界知识。当被告知"天气恶劣"时，它能推断出应该降低飞行高度、缩短巡检时间；当被告知"发现可疑人员"时，它能推断出应该保持安全距离并通知其他无人机协助监控。

**（3）灵活的任务分解**

面对一个复杂的任务目标，LLM能够像人类指挥官一样，将其分解为合理的子任务。而且，当情况发生变化时（例如一架无人机电量不足），LLM可以动态调整计划，这是硬编码的规则系统难以做到的。

**（4）自然语言协商**

在分布式场景中，多个LLM Agent可以通过自然语言进行"对话"来协商任务分配。例如，Drone1可以告诉Drone2："我已经在检查A区了，你去B区吧。"这种协商方式直观且灵活。

当然，LLM驱动的多Agent系统也面临着独特的挑战：LLM可能产生"幻觉"（生成不存在的坐标或不合理的指令）、推理速度相对较慢（相比传统算法）、以及难以保证行为的确定性和可预测性。这些问题在本章的高级主题部分会进一步讨论。

### 6.1.6 多Agent协同的研究进展

多智能体系统是人工智能领域的经典研究方向，近年来随着LLM的兴起又焕发了新的活力。以下简要介绍几个重要的研究方向，帮助读者建立更完整的知识图景。

#### 1. 多智能体强化学习（MARL）

多智能体强化学习是让多个智能体通过与环境的交互来学习协作策略的方法。每个智能体根据自己的观察和奖励信号来更新策略，同时需要考虑其他智能体的行为。

代表性工作包括OpenAI Five（多个AI协作打Dota 2游戏）和DeepMind的AlphaStar（星际争霸中的多单位协调）。在无人机领域，MARL被用于学习编队飞行、协同搜索等策略。

MARL的优势在于能够通过大量模拟训练发现人类难以设计的协作策略；挑战在于训练不稳定（多个智能体同时学习导致环境非平稳）、样本效率低、以及从模拟到现实的迁移困难。

#### 2. 群体智能与蜂群算法

群体智能（Swarm Intelligence）从自然界的群体行为中汲取灵感。蚂蚁觅食时通过信息素进行间接通信，蜜蜂通过舞蹈传递花蜜位置信息，鸟群通过简单的局部规则实现复杂的编队飞行。

在无人机领域，群体智能的思想被用于设计大规模无人机蜂群。每架无人机只需遵循几条简单的规则（如保持与邻居的距离、朝向邻居的平均方向飞行、避免碰撞），就能涌现出复杂的集群行为。这种方法的优势在于极强的可扩展性和鲁棒性——即使部分个体失效，整个群体仍能正常运作。

#### 3. LLM Agent框架的发展

2023年以来，基于LLM的多Agent框架迅速发展。AutoGen提出了"对话驱动"的协作范式，LangGraph引入了"状态图"的编排方式，CrewAI则用"角色扮演"简化了多Agent的定义。

在机器人领域，SayCan（Google, 2022）首次展示了LLM如何将自然语言指令转化为机器人可执行的动作序列；ChatGPT for Robotics（Microsoft, 2023）进一步探索了LLM在机器人任务规划中的应用；SMART-LLM（2024）则专门研究了LLM在多机器人任务分配中的能力。

这些工作共同指向一个趋势：LLM正在成为多智能体系统的"认知中枢"，负责高层次的理解、规划和协调，而传统的控制算法则负责底层的运动执行。

#### 4. 人机协同与人在回路

在实际部署中，完全自主的多无人机系统仍然面临安全和法规方面的限制。因此，"人在回路"（Human-in-the-Loop）的设计理念非常重要：人类操作员通过自然语言与无人机集群交互，提供高层指令和关键决策，而无人机负责具体的执行。

本章的实验正是这种人机协同模式的体现：人类操作员输入任务目标（如"搜索这两个区域"），LLM Agent负责分解和执行，最后向人类汇报结果。

### 6.1.7 多智能体程序框架选择

在动手编写代码之前，有必要了解当前主流的多Agent程序框架。这些框架封装了智能体的创建、通信、任务编排等底层细节，让开发者可以专注于业务逻辑。

#### 1. Microsoft AutoGen：对话式协作引擎

AutoGen的设计哲学根植于一个核心理念：复杂的多智能体协作可以被有效地建模为一场结构化的对话。它提供了一个灵活的框架，用于创建可定制的、"可对话的"（Conversable）智能体。

想象一个微信群聊：群里有产品经理、设计师、程序员，他们通过聊天来协作完成一个项目。AutoGen的GroupChat机制就是这个思路——多个Agent在一个"群聊"中交流，由一个管理者决定谁来"发言"。

#### 2. LangChain LangGraph：状态驱动的编排框架

LangGraph将工作流建模为一个有状态的图（Stateful Graph）。图中的节点代表智能体或函数，边定义了它们之间的交互逻辑。

这就像一条工厂的流水线：每个工位（节点）负责一道工序，产品（状态）沿着传送带（边）从一个工位流向下一个。LangGraph的优势在于流程可预测、可调试，适合生产环境。

#### 3. CrewAI：基于角色的任务执行框架

CrewAI将智能体协作类比为一个现实世界的工作团队。你只需要定义三个概念：Agent（团队成员）、Task（工作任务）、Crew（团队），然后调用`crew.kickoff()`就能启动整个协作流程。

这是学习曲线最低的框架，非常适合快速原型验证。本章的后半部分将使用CrewAI来演示框架化的多Agent编排。

| 框架 | 核心理念 | 特点 | 无人机适用性 |
|------|----------|------|------------|
| LangGraph | 状态驱动的图 | 确定性控制流、生产级可靠性 | 适合确定性流程 |
| AutoGen | 对话驱动 | 灵活的群聊机制、动态协作 | 适合动态协商场景 |
| CrewAI | 角色扮演 | 直观的团队协作、低学习曲线 | 适合快速原型验证 |

### 6.1.8 本章方案与实验路线图

为了让读者能够一行行看懂代码、真正理解原理，本章采用循序渐进的方案：

1. **先理解原理**（6.3~6.4节）：用纯Python + OpenAI SDK + AirSim实现中心化和分布式两种协同模式，不使用任何Agent框架，总代码量不超过100行
2. **再使用框架**（6.5~6.6节）：用CrewAI框架简化多Agent编排，体会框架带来的工程效率提升
3. **综合实战**（6.7节）：用三无人机层级协同完成工业巡检任务，结合真实场景的目标检测与拍照

整个实验路线图如下：

| 实验 | 内容 | 协同模式 | 技术栈 | 无人机数量 |
|------|------|---------|--------|-----------|
| 6.3节 | 指挥官分解任务 | 中心化 | 纯Python | 2架 |
| 6.4节 | 消息板协商 | 分布式 | 纯Python | 2架 |
| 6.5节 | CrewAI入门 | — | CrewAI框架 | 无（纯LLM） |
| 6.6节 | CrewAI+AirSim | 中心化 | CrewAI框架 | 2架 |
| 6.7节 | 工业巡检 | 层级式 | 纯Python | 3架 |

理解了原理之后再去学习框架，就会有深刻的体会——框架帮你做了什么，你自己又能控制什么。

---

PLACEHOLDER_REST_OF_DOC

## 6.2 环境搭建与工具准备

### 6.2.1 配置AirSim

首先，我们需要配置AirSim模拟器以生成一个多无人机环境。这通过修改AirSim的`settings.json`文件来完成。该文件通常位于您的`文档/AirSim`目录下。

由于不同实验所需的无人机数量不同，本章提供了两个配置文件：

| 配置文件 | 无人机数量 | 适用实验 |
|----------|-----------|---------|
| `1-6-settings.json` | 2架（Drone1、Drone2） | 6.2~6.6节 |
| `7-settings.json` | 3架（Drone1、Drone2、Drone3） | 6.7节（层级协同） |

使用时，将对应的配置文件复制到`~/Documents/AirSim/settings.json`，然后启动（或重启）AirSim即可。

以下是2架无人机的基础配置（`1-6-settings.json`）：

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "CameraDefaults": {
    "CaptureSettings": [
      {"ImageType": 0, "Width": 640, "Height": 480, "FOV_Degrees": 90}
    ]
  },
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 0, "Y": 0, "Z": 0, "Yaw": 0
    },
    "Drone2": {
      "VehicleType": "SimpleFlight",
      "X": 5, "Y": 0, "Z": 0, "Yaw": 0
    }
  }
}
```

其中`CameraDefaults`配置了640x480分辨率的机载相机，用于后续的无人机拍照功能。3架无人机的配置（`7-settings.json`）在此基础上增加了Drone3。

### 6.2.2 安装依赖

本章需要以下Python包：

```bash
pip install openai airsim    # 基础部分（6.2~6.4节）
pip install crewai           # 框架部分（6.5~6.6节，需Python 3.10+）
```

### 6.2.3 连接与验证

用一个简单的Python脚本来验证设置是否成功：

```python
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
vehicles = client.listVehicles()
print(f"连接成功！发现无人机: {vehicles}")
# 预期输出: 连接成功！发现无人机: ['Drone1', 'Drone2']
```

### 6.2.4 工具模块：airsim_tools.py

我们将所有工具函数封装在`airsim_tools.py`中，包含两部分。

#### Part 1：AirSim无人机控制函数

三个核心函数，全部是纯Python函数，没有任何框架依赖：

| 函数 | 功能 | 参数 |
|------|------|------|
| `takeoff(client, drone_id)` | 起飞 | AirSim客户端, 无人机名称 |
| `fly_to(client, drone_id, x, y, z)` | 飞到指定坐标 | 客户端, 名称, NED坐标 |
| `get_state(client, drone_id)` | 获取当前位置 | 客户端, 名称 |

> 注意：AirSim使用NED坐标系（North-East-Down），z为负表示向上。例如z=-10代表距地面10米高度。这与日常生活中"高度为正"的直觉相反，初学者需要特别注意。

完整实现代码：

```python
def connect_airsim():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    return client

def takeoff(client, drone_id):
    client.enableApiControl(True, vehicle_name=drone_id)
    client.armDisarm(True, vehicle_name=drone_id)
    client.takeoffAsync(vehicle_name=drone_id).join()

def fly_to(client, drone_id, x, y, z, speed=5):
    client.moveToPositionAsync(x, y, z, speed, vehicle_name=drone_id).join()

def get_state(client, drone_id):
    state = client.getMultirotorState(vehicle_name=drone_id)
    pos = state.kinematics_estimated.position
    return {"x": round(pos.x_val, 2), "y": round(pos.y_val, 2), "z": round(pos.z_val, 2)}
```

每个函数都很简短——`takeoff`先解锁API控制和电机，然后异步起飞并等待完成；`fly_to`命令无人机以指定速度飞往目标坐标；`get_state`读取无人机的当前位置并返回字典。

#### Part 2：LLM调用封装

一个`call_llm(prompt)`函数，使用OpenAI SDK调用豆包（Doubao）模型：

```python
def call_llm(prompt, system="你是一个无人机任务规划助手，请简洁回答。"):
    client = OpenAI(base_url=LLM_BASE_URL, api_key=LLM_API_KEY)
    response = client.chat.completions.create(
        model=LLM_MODEL, temperature=0.1,
        messages=[{"role": "system", "content": system},
                  {"role": "user", "content": prompt}]
    )
    return response.choices[0].message.content
```

`temperature=0.1`确保LLM的输出尽可能确定和一致，这对于需要解析JSON格式的任务分解场景尤为重要。

---

PLACEHOLDER_63_ONWARDS

## 6.3 实验一：中心化协同

### 6.3.1 核心思想

中心化协同是最直观的多无人机协作模式：一个LLM充当"指挥官"，接收高层任务目标，将其分解为具体的飞行指令，然后依次控制每架无人机完成任务。

这种模式的工作流程可以用一句话概括：**人类说目标，LLM做计划，无人机去执行**。整个实现不到50行代码，非常适合作为理解多Agent协同的第一个实验。

### 6.3.2 指挥官LLM分解任务

给LLM一个高层任务目标，让它自动分解为每架无人机的具体航点任务。关键技巧是要求LLM以JSON格式输出，这样我们可以直接用`json.loads()`解析，无需复杂的文本处理：

```python
mission = "派Drone1去(-10, 10, -10)侦察，派Drone2去(10, 10, -10)侦察"

plan_prompt = f"""你是无人机集群指挥官。请将以下任务分解为具体飞行指令。
任务：{mission}
要求：
- 输出纯JSON列表，不要其他文字
- 每个任务格式：{{"drone_id": "Drone1", "target": [x, y, z]}}
- z坐标为负表示高度（如-10表示10米高）
"""

plan_text = call_llm(plan_prompt)
tasks = json.loads(plan_text)
```

这里有一个重要的工程细节：我们在提示词中明确要求"输出纯JSON列表，不要其他文字"。这是因为LLM有时会在JSON前后添加解释性文字，导致`json.loads()`解析失败。在实际工程中，还需要添加更多的容错处理（如正则提取JSON、重试机制等）。

### 6.3.3 顺序执行所有任务

指挥官分配好了任务，现在逐一执行：

```python
results = []
for i, task in enumerate(tasks):
    drone_id = task["drone_id"]
    target = task["target"]
    print(f"--- 执行任务 {i+1}: {drone_id} → {target} ---")
    takeoff(client, drone_id)
    fly_to(client, drone_id, target[0], target[1], target[2])
    pos = get_state(client, drone_id)
    results.append({"drone_id": drone_id, "target": target, "actual_pos": pos})
```

注意这里是**顺序执行**——Drone1完成后才轮到Drone2。这是中心化模式的一个局限：即使两架无人机的任务互不相关，也无法并行执行（除非我们使用多线程，但那会增加代码复杂度）。

### 6.3.4 指挥官LLM生成总结报告

所有任务完成后，让LLM生成一份简洁的执行报告。这一步展示了LLM的另一个优势——它不仅能做计划，还能分析执行结果并生成人类可读的报告：

```python
summary_prompt = f"""以下是无人机任务执行结果，请生成简洁的中文报告：
原始任务：{mission}
执行结果：{json.dumps(results, ensure_ascii=False)}
请分析每架无人机是否到达目标位置，总结任务完成情况。"""

report = call_llm(summary_prompt)
```

### 6.3.5 流程总结

中心化协同的完整流程只有4步：

```
1. 人类输入任务目标
      ↓
2. LLM指挥官 → 分解为JSON任务列表
      ↓
3. for循环 → 逐一执行（takeoff + fly_to）
      ↓
4. LLM指挥官 → 生成总结报告
```

**优点**：代码极简，一目了然；指挥官有全局信息，可以做出全局最优分配。

**局限**：指挥官是单点故障；无人机之间没有直接通信；任务是预先规划好的，无法应对执行过程中的动态变化。

---

## 6.4 实验二：分布式协同

### 6.4.1 核心思想

分布式协同的核心区别：**没有中央指挥官**，每架无人机拥有自己的LLM"大脑"，通过一个**共享消息板**交换信息、协调行动。

每架无人机的决策流程为：
1. 读取消息板上其他无人机的状态
2. 结合自己的任务目标，让LLM做出决策
3. 执行飞行动作
4. 将结果发布到消息板

这模拟了真实场景中无人机通过无线通信交换信息的过程。

### 6.4.2 共享消息板和无人机大脑

**消息板**是一个简单的Python列表，所有无人机都可以读写。在真实系统中，这对应的是无人机之间的无线通信链路（如Wi-Fi、4G/5G、或专用数据链）。

**drone_brain**是每架无人机的"大脑"函数——它读取消息板、调LLM决策、执行动作、发布结果：

```python
message_board = []

def drone_brain(client, drone_id, mission, message_board):
    # 1. 读取消息板
    others_info = [m for m in message_board if m["from"] != drone_id]

    # 2. LLM独立决策
    decision_prompt = f"""你是无人机 {drone_id}，正在执行分布式协同任务。
    总体任务：{mission}
    其他无人机的状态：
    {json.dumps(others_info, ensure_ascii=False) if others_info else "暂无"}
    请决定飞往哪个坐标。不要和其他无人机去同一个位置。
    输出纯JSON：{{"target": [x, y, z], "reason": "简短理由"}}"""

    decision = json.loads(call_llm(decision_prompt))
    target = decision["target"]

    # 3. 执行飞行
    takeoff(client, drone_id)
    fly_to(client, drone_id, target[0], target[1], target[2])

    # 4. 发布到消息板
    message_board.append({
        "from": drone_id, "target": target,
        "actual_pos": get_state(client, drone_id),
        "reason": decision.get("reason", "")
    })
```

### 6.4.3 执行分布式协同

两架无人机依次执行。关键观察点：**Drone2能看到Drone1的消息，从而做出不同的决策**：

```python
mission = "搜索区域：需要侦察(-10,10,-10)附近和(10,10,-10)附近，两架无人机分工协作。"
message_board = []

drone_brain(client, "Drone1", mission, message_board)  # 消息板为空
drone_brain(client, "Drone2", mission, message_board)  # 能看到Drone1的消息
```

当Drone1执行时，消息板为空，它只能根据任务描述自行选择一个目标。当Drone2执行时，它能从消息板上看到Drone1已经去了某个位置，因此会自动选择另一个位置——这就是分布式协同的核心机制。

### 6.4.4 流程总结

| 对比 | 中心化（6.3节） | 分布式（本节） |
|------|----------------|---------------|
| 决策者 | 1个指挥官LLM | 每架无人机各自的LLM |
| 通信 | 指挥官→工作者（单向） | 消息板（双向共享） |
| Drone2的信息 | 由指挥官分配 | 自己从消息板读取 |
| 适应性 | 低（预先规划好） | 高（可根据实时信息调整） |

**核心观察**：Drone2在决策时能看到Drone1已经去了某个位置，因此会自动选择去另一个位置——这就是分布式协同的魅力。无需任何人为干预，两架无人机通过信息共享实现了自发的分工协作。

---

## 6.5 CrewAI框架入门

前面我们用纯Python实现了中心化和分布式两种协同模式。代码虽然简洁，但所有的流程编排、结果传递、错误处理都需要手动管理。在实际工程中，通常会使用多Agent框架来简化这些工作。

本节介绍**CrewAI**——学习曲线最低的多Agent框架，只需掌握3个核心概念即可上手。

### 6.5.1 CrewAI是什么

CrewAI将多Agent协作类比为**现实中的工作团队**。就像组建一个项目组一样，你只需要：

| 概念 | 类比 | 作用 |
|------|------|------|
| **Agent** | 团队成员 | 定义角色（role）、目标（goal）、背景（backstory） |
| **Task** | 工作任务 | 定义任务描述和期望输出，指定由哪个Agent执行 |
| **Crew** | 团队 | 把Agent和Task组合在一起，启动执行 |

```
┌─────────── Crew（团队）───────────┐
│                                    │
│  Agent1（研究员）→ Task1（调研）    │
│         ↓                          │
│  Agent2（写手）  → Task2（撰写）    │
│                                    │
└────────────────────────────────────┘
          crew.kickoff() → 结果
```

### 6.5.2 配置LLM

CrewAI内置了`LLM`类，可以直接连接任何OpenAI兼容的API：

```python
from crewai import LLM

llm = LLM(
    model="doubao-seed-2-0-pro-260215",
    base_url="https://ark.cn-beijing.volces.com/api/v3",
    api_key="your-api-key",
    temperature=0.1
)
```

### 6.5.3 示例：单Agent Hello World

最简单的CrewAI程序——一个Agent、一个Task、一个Crew：

```python
from crewai import Agent, Task, Crew

greeter = Agent(
    role="问候专家",
    goal="用有趣的方式向用户打招呼",
    backstory="你是一个热情友好的AI助手。",
    llm=llm, verbose=True
)

greet_task = Task(
    description="请用中文向小明打招呼，并给出今天的一句鼓励话语。",
    expected_output="一段友好的中文问候语。",
    agent=greeter
)

crew = Crew(agents=[greeter], tasks=[greet_task], verbose=True)
result = crew.kickoff()
```

就这么简单！定义Agent→定义Task→组建Crew→`kickoff()`执行。

### 6.5.4 示例：双Agent协作

两个Agent按顺序协作：**研究员**先收集信息，**报告员**再整理成报告。Task1的输出自动传递给Task2作为上下文，这就是CrewAI的顺序协作模式。

### 6.5.5 自定义工具（@tool）

Agent不仅能"思考"，还能"行动"——通过**工具（Tool）**与外部世界交互。CrewAI提供了`@tool`装饰器，可以将任意Python函数变成Agent可调用的工具：

> 注意：`@tool`的名称参数必须使用英文，中文名称会被CrewAI内部的sanitize逻辑过滤为空字符串导致报错。

```python
from crewai.tools import tool

@tool("get_weather")
def get_weather(city: str) -> str:
    """查询指定城市的天气情况（模拟数据）。"""
    weather_data = {"北京": "晴天 25°C", "上海": "下雨 20°C"}
    return weather_data.get(city, f"{city}: 暂无数据")
```

Agent会自动决定何时调用哪个工具——这就是**ReAct（推理+行动）**模式的体现。LLM先"思考"需要什么信息，再"行动"调用工具获取，最后整合输出。

---

## 6.6 CrewAI + AirSim：多无人机协同任务

本节将CrewAI框架与AirSim模拟器结合，用Agent框架编排多无人机协同搜索任务。

### 6.6.1 将AirSim API封装为CrewAI工具

用`@tool`装饰器将无人机控制函数包装成Agent可调用的工具：

```python
@tool("drone_takeoff")
def drone_takeoff(drone_id: str) -> str:
    """命令指定的无人机起飞。"""
    airsim_client.enableApiControl(True, vehicle_name=drone_id)
    airsim_client.armDisarm(True, vehicle_name=drone_id)
    airsim_client.takeoffAsync(vehicle_name=drone_id).join()
    return f"{drone_id} 已成功起飞"

@tool("drone_fly_to")
def drone_fly_to(drone_id: str, x: float, y: float, z: float) -> str:
    """命令无人机飞往指定NED坐标。"""
    airsim_client.moveToPositionAsync(x, y, z, 5, vehicle_name=drone_id).join()
    return f"{drone_id} 已到达坐标 ({x}, {y}, {z})"

@tool("drone_get_position")
def drone_get_position(drone_id: str) -> str:
    """获取指定无人机的当前位置坐标。"""
    state = airsim_client.getMultirotorState(vehicle_name=drone_id)
    pos = state.kinematics_estimated.position
    return f"{drone_id} 当前位置: x={pos.x_val:.1f}, y={pos.y_val:.1f}, z={pos.z_val:.1f}"
```

### 6.6.2 定义Agent与Crew

两个角色：**任务规划师**负责理解任务目标并分解为飞行指令；**侦察员**负责执行飞行任务，使用AirSim工具控制无人机。

```python
crew = Crew(agents=[planner, scout], tasks=[plan_task, exec_task], verbose=True)
result = crew.kickoff()
```

整个流程由CrewAI框架自动编排：规划师先输出飞行指令，侦察员再根据指令调用工具执行。Agent会自动决定何时调用哪个工具——开发者无需手动编排调用顺序。

### 6.6.3 与纯Python实现的对比

| 对比 | 纯Python（6.3~6.4节） | CrewAI（本节） |
|------|----------------------|----------------|
| 任务分解 | `call_llm()` + 手动解析JSON | Agent自动推理 |
| 工具调用 | 手动 `takeoff()` + `fly_to()` | Agent根据任务自动选择工具 |
| 流程编排 | for循环/消息板 | Crew自动管理Task顺序 |
| 错误处理 | 需要自己写 | 框架内置重试机制 |
| 代码可读性 | 直接但代码多 | 声明式、更清晰 |

先理解原理，再使用框架——这是学习多Agent系统的最佳路径。

---

## 6.7 实验三：层级协同——三无人机工业巡检

前面的实验中，无人机要么听从统一指挥（中心化），要么各自为政（分布式）。在真实的工业巡检场景中，更常见的是一种**层级指挥**模式：一架无人机在高空担任监控指挥角色，统筹协调多架巡检无人机分头执行任务，巡检完成后向指挥汇报。

这种模式兼具中心化的全局协调能力和分布式的执行灵活性，是工业巡检、搜救、安防等领域的主流架构。

> **AirSim配置**：本节使用`7-settings.json`（3架无人机），需重启AirSim。

### 6.7.1 场景与角色设计

本节利用AirSim工业巡检场景中的真实设施作为巡检目标。与前面硬编码坐标不同，我们通过AirSim的`simListSceneObjects()`和`simGetObjectPose()` API**动态获取**场景中设施的真实位置。

| 无人机 | 角色 | 职责 |
|--------|------|------|
| Drone3 | 监控指挥 | 高空悬停，获取全局视野，制定计划，汇总报告 |
| Drone1 | 巡检员A | 飞往风力发电机，近距离检查并拍照 |
| Drone2 | 巡检员B | 飞往变电站区域，近距离检查并拍照 |

### 6.7.2 动态获取巡检目标

通过AirSim API自动发现场景中的巡检设施：

```python
def get_inspection_targets(client):
    targets = {}
    objects = client.simListSceneObjects()
    for obj in objects:
        if 'Wind_Turbine' in obj:
            pose = client.simGetObjectPose(obj)
            targets['风力发电机'] = {
                'name': obj,
                'x': round(pose.position.x_val, 1),
                'y': round(pose.position.y_val, 1)
            }
    trellis_positions = []
    for obj in objects:
        if 'Electric_trellis' in obj:
            pose = client.simGetObjectPose(obj)
            trellis_positions.append((pose.position.x_val, pose.position.y_val))
    if trellis_positions:
        avg_x = sum(p[0] for p in trellis_positions) / len(trellis_positions)
        avg_y = sum(p[1] for p in trellis_positions) / len(trellis_positions)
        targets['变电站'] = {'name': f'电塔群({len(trellis_positions)}座)',
            'x': round(avg_x, 1), 'y': round(avg_y, 1)}
    return targets
```

同时定义拍照函数，让无人机在到达目标后拍摄现场照片。AirSim的图像获取是通过API直接从UE4渲染引擎内部读取数据，不是操作系统截屏，因此窗口最小化也能正常拍照。

### 6.7.3 监控无人机升空与任务规划

监控无人机（Drone3）首先起飞到高空中心位置，获取全局视野。然后通过API获取巡检目标的坐标，交给LLM制定任务分配计划。LLM会根据目标位置自动将两个巡检任务分配给不同的无人机。

### 6.7.4 巡检无人机执行任务

两架巡检无人机按照指挥官的计划，分别飞往各自的目标，拍照并记录状态：

```python
reports = []
for task in plan['tasks']:
    drone_id = task['drone_id']
    x, y, z = task['x'], task['y'], task['z']
    takeoff(client, drone_id)
    fly_to(client, drone_id, x, y, z)
    capture_image(client, drone_id, f"nb7_{drone_id}.png")
    pos = get_state(client, drone_id)
    reports.append({'drone_id': drone_id, 'target': task['target'],
        'actual_position': pos, 'status': '到达目标，拍照完成'})
```

### 6.7.5 监控无人机汇总报告

所有巡检任务完成后，监控无人机收集汇报信息，交给LLM生成最终的巡检总结报告。信息自下而上汇报，决策自上而下分配，符合真实世界中的指挥链结构。

### 6.7.6 三种协同模式对比

| 对比项 | 中心化（6.3节） | 分布式（6.4节） | 层级指挥（本节） |
|--------|----------------|----------------|-----------------|
| 决策方式 | 1个LLM统一规划 | 每架无人机各自决策 | 指挥官规划 + 执行者汇报 |
| 通信模式 | 指挥官→工作者 | 消息板（双向） | 指挥官↔巡检员（层级） |
| 角色分工 | 无明确分工 | 平等协商 | 监控/巡检明确分层 |
| 场景感知 | 硬编码坐标 | 共享状态 | API获取真实目标 + 拍照确认 |
| 无人机数量 | 2架 | 2架 | 3架（1监控 + 2巡检） |
| 适用场景 | 简单任务 | 动态环境 | 工业巡检、搜救等 |

---

## 6.8 高级主题与未来方向

本章所介绍的系统仅仅是通往真正自主无人机集群之路的开端。在将这些系统从模拟推向现实的过程中，还有许多值得关注的问题和方向。

### 6.8.1 从模拟到现实的迁移

在AirSim中完美运行的系统，在部署到物理硬件上时可能会遇到各种问题。真实世界的传感器存在噪声，无线通信存在延迟和丢包，电机和电池的物理特性也与模拟模型有差异。

解决"模拟到现实"（Sim-to-Real）的鸿沟是机器人学领域一个长期存在的核心挑战。常见的策略包括：在模拟中引入随机噪声来增强鲁棒性（Domain Randomization）、使用更高保真度的物理引擎、以及在真实环境中进行小规模验证后逐步扩大部署范围。

### 6.8.2 LLM在机器人技术中的局限性

**接地问题（Grounding）**：LLM的知识来自文本训练数据，它对物理世界的"理解"是间接的。当LLM说"飞到建筑物旁边"时，它并不真正知道建筑物在哪里、有多大。因此，需要将LLM的符号化输出与传感器的实时数据进行"接地"——用真实的感知数据来验证和修正LLM的计划。

**幻觉与安全**：LLM可能生成不存在的坐标、不合理的飞行指令，甚至"幻觉"出并不存在的障碍物。在无人机场景中，这种错误可能导致碰撞或坠毁。因此，必须设计多层安全保障机制：

- 在执行前对LLM生成的指令进行合理性验证（如坐标是否在合法范围内）
- 设置地理围栏等硬性安全边界
- 实现可靠的"人在回路"监督和紧急制动机制

### 6.8.3 机载LLM vs. 云端LLM

这是一个关键的架构权衡：

- **云端LLM**：可以利用最强大的大语言模型（如GPT-4、Claude等），具备更强的推理和规划能力，但会引入显著的通信延迟（通常100ms~1s），且依赖网络连接。
- **机载LLM（边缘计算）**：将经过优化的轻量级模型部署在无人机上，可实现低延迟的本地决策，但受限于机载计算能力和能耗。

未来的发展趋势可能是**云-边协同的混合架构**：云端负责长期战略规划和复杂推理，边缘端负责实时响应和安全控制。例如，云端LLM制定整体巡检计划，机载小模型负责实时避障和姿态控制。

### 6.8.4 研究前沿

**动态组织架构选择**：使用LLM根据实时任务参数（如任务复杂度、集群规模、通信质量）动态地为无人机集群选择最优的协作模式。例如，通信良好时采用中心化模式以获得全局最优，通信中断时自动切换为分布式模式以保持运作。

**LLM与多智能体强化学习的融合**：构建混合系统，结合LLM的语义理解与任务分解能力，以及强化学习在密集奖励信号下优化协作策略的能力。LLM负责"做什么"（高层规划），强化学习负责"怎么做"（底层控制）。

**大规模蜂群智能**：当无人机数量达到数百甚至上千架时，传统的通信和协调机制将面临严峻挑战。受自然界蜂群、鸟群启发的涌现式协调方法，结合LLM的语义理解能力，可能是解决大规模协同问题的关键。

这些前沿探索正在不断弥合学术研究与工业实践之间的差距，推动自主无人机系统向更高层次的智能化、自适应性和可靠性发展。




