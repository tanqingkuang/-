# 3D 态势模型资产

- `PredatorUAV.glb`:低多边形捕食者式固定翼无人机(434 三角形,36KB),
  来自 Poly Pizza(原 Google Poly),作者 sftwr314r8,CC-BY 3.0 许可。
  来源页:https://poly.pizza/m/3Eio09miiAF
  注意:该资产机头朝 +Z,Python 侧策略使用 +90° 偏航校正。
  模型单位翼展约 1.76,对应真实翼展约 15m。
  仅用于仿真验证的态势显示,QML 侧通过 `RuntimeLoader` 运行时加载。
  机型参数现由 `aircraft_model_style.py` 的策略类提供。
- `BayraktarTB2.glb`: Bayraktar TB2 察打一体无人机(9300 三角面),
  来自 Sketchfab,作者 42manako,CC-BY 4.0 许可。
  来源页:https://sketchfab.com/3d-models/ukrainian-bayraktar-tb2-9738e42238894e5bbc51646c79eadec7
  注意:该资产机头朝 +Z,Y 向上,翼展沿 X 轴,Python 侧策略使用 +90° 偏航校正。
  按真实尺寸建模,模型单位翼展约 11.957,对应真实翼展约 12m,原点位于地面。
  仅用于仿真验证的态势显示,QML 侧通过 `RuntimeLoader` 运行时加载。
- `RQ4GlobalHawk.glb`: RQ-4 全球鹰无人机(52876 三角面),
  来自 Sketchfab,作者 kio00,CC-BY 许可。
  来源页:https://sketchfab.com/3d-models/rq4-global-hawk-space-ver-36560c332dda4a2bbf5c6f0877c023c2
  注意:该资产机头朝 +Z,Y 向上,翼展沿 X 轴,Python 侧策略使用 +90° 偏航校正。
  模型单位翼展约 0.469,对应真实翼展约 39.9m。
  仅用于仿真验证的态势显示,QML 侧通过 `RuntimeLoader` 运行时加载。
- `terrain_detail_normal.png`: 四方无缝的岩面切线空间法线贴图，用于补足高度网格采样间的细裂隙。
- `terrain_detail_albedo.png`: 四方无缝的低饱和岩石反照率乘色图，只添加灰岩颗粒与暗裂隙，不覆盖顶点色的海拔分层。
- 两张地形贴图均由 `scripts/generate_terrain_detail_normal.py` 使用固定种子生成，可重复构建；正式全量版打包脚本会显式携带二者。
