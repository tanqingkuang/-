# 3D 山地地形风格样张

本目录用于生成两张 Qt Quick 3D 渲染样张：

- `output/style_a.png`：写实地形 + 科幻氛围
- `output/style_b.png`：暗色线框指挥风

生成逻辑独立于正式 `src/` 与现有 QML。高度场、航线、风险区线层均由 `generate_previews.py` 动态生成，并通过本目录内的 `preview_scene.qml` 渲染。

## 运行命令

在仓库根目录执行：

```powershell
python scripts/terrain_style_preview/generate_previews.py
```

脚本默认输出：

```text
scripts/terrain_style_preview/output/style_a.png
scripts/terrain_style_preview/output/style_b.png
```

## 可调参数

```powershell
python scripts/terrain_style_preview/generate_previews.py `
  --grid-size 641 `
  --seed 20260711 `
  --contour-step 185 `
  --wait-ms 1500 `
  --styles a,b `
  --live-ms 0 `
  --output-dir scripts/terrain_style_preview/output
```

- `--grid-size`：高度场采样边长，默认 `641`。默认轴向为中心高分辨率、外围低分辨率；核心 `24km x 24km` 区域约 `46.9m` 一个采样点。低配机器可降到 `384`，但近景山脊会明显变软。
- `--seed`：地形细节随机种子，默认 `20260711`。主峰布局固定沿航线走廊两侧分布，种子主要影响脊状噪声、域扭曲、走廊内小山头和外围连接丘陵。
- `--contour-step`：风格 B 普通等高线间隔，单位米，默认 `185`。数值越小，线框越密。
- `--wait-ms`：每张图显示后等待渲染稳定的毫秒数，默认 `1500`。若导出偶发黑屏，可调大到 `2500`。
- `--styles`：选择要渲染的风格，默认 `a,b`。第四轮只精修 style A，可用 `--styles a` 避免覆盖冻结的 style B。
- `--live-ms`：实时显示 style A 窗口的毫秒数，不抓图，默认 `0`。用于非截图模式下观察窗口运行状态。
- `--output-dir`：输出目录，默认是本目录下 `output/`。

## 实现说明

- 预览画布为 `120km x 120km`，核心 `24km x 24km` 走廊使用高分辨率采样，外围远山使用同一张 mesh 的低密度裙边延伸到雾终点之外，避免看到几何边缘。
- 地形共用同一份高度几何：8 座主峰布置在蜿蜒航线走廊两侧，主峰相对高度约 `2300m~3000m`。
- 山体由椭圆主峰、随机旋转/偏移的多倍频程 ridged fBm、domain warping、曲率沟壑、外围连接丘陵和走廊内 `300m~600m` 小山头叠加生成。固定方向正弦刀脊已移除，避免平行鳍片状瑕疵。
- 风格 A 的地表顶点色由海拔、坡度、低角度冷暖光照和曲率 AO 共同决定，风险区叠加暗红半透明面片、红橙发光线框和淡青色安全缓冲虚线。
- 风格 B 的地表低亮度显示，额外叠加青绿色发光等高线、贴地经纬网格、红色风险区线框和安全缓冲虚线。
- 航线使用两层青色三角带：外层宽而透明模拟辉光，内层窄而高亮保证路径清晰；沿线包含约 `2km` 间隔的发光圆环航点和简化无人机线框。
- 红色虚线原始航线从风险区前截断，并在进入风险区处叠加红圈叉号。

## 注意事项

脚本默认设置 `QSG_RHI_BACKEND=d3d11`，适配 Windows 11 桌面会话。若当前机器的 Qt Quick 3D 抓图异常，可以先关闭其他占用 GPU 的窗口，再把 `--wait-ms` 调大后重试。

## 第四轮实测

测试命令：

```powershell
python scripts/terrain_style_preview/generate_previews.py --styles a --live-ms 6000
```

本机实测：

- 默认 `--grid-size 641` 时，地形网格为 `410881` 个顶点、`819200` 个三角形。
- 第五轮最终 style A 渲染实测：高度场生成耗时约 `3.54s`，场景数据总构建耗时约 `14.10s`。
- style A 实时窗口非截图模式运行 `6000ms`，窗口正常显示并退出，未出现黑屏、卡死或 QML 报错。
- style B 已冻结，不作为第四轮视觉调参对象；需要重出图时仍可用 `--styles b` 单独生成。
