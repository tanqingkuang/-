# 一、叶类型

叶类型本质是一堆结构体、枚举的定义

## 1.1 枚举

```
typedef enum {
    "None",
    "集结",
    "编队保持",
    "编队重构"
} FormationStageCmdE, FormationStaE; /* 编队指令，编队状态 */

typedef enum {
    "None",
    "三角编队"
} FormationPatE;
```



## 1.2 结构体

```
typedef struct {
    float vEast;
    float vNorth;
    float vUp;
    float vTheta; // 航迹倾角
    float vPsi; // 航迹偏航角
    float vd; // =sqrt(vEast^2 + vNotrh^2)
} VdInEarthS; // 在地球平面坐标系下的地速 

typedef struct {
    float east; // 单位：m
    float north; // 单位：m
    float h; // 单位：m
} PosInEarthS; // 在地球平面坐标系下的位置

typedef struct {
    u8 idx;  // 航点编号，从0开始
		PosInEarthS pos;
} WayPointS; // 航点

typedef struct {
		u8 idx; // 航段编号，0-1的航点组成航段0
    WayPointS start;
    WayPointS end;
    float vdCmd; // 单位 m/s 
    float radius; // 曲率，0代表直线
} WayLineS; // 航段



FormationStageE cmd; // 外部给出的编队指令
FormationStageE state[N]; // 各个飞机的
FormationPatStateE pattern; // 队形



typedef struct {
    
    FormationStageE mode; // 任务编排给出的编队指令
    FormationStaE state[N]; // 各个飞机的编队飞行状态
    
} FormationInfoS; // 编队信息

```





# 二、context