"""llt 全局测试配置。注意：只影响 tests/llt,不影响 ST 与生产运行。"""

import os

# 地形高度场生成成本随分辨率平方增长(641 约 5s/次,257 约 0.6s/次),
# 而 llt 一次全量要生成 15~20 次,是套件耗时的绝对大头。
# 这里统一压低分辨率上限提速;分辨率语义本身由专项用例在无上限下验证
# (见 test_situation3d_scene 的航线净空用例)。用 setdefault 保留手工覆盖入口。
os.environ.setdefault("SIM3D_TERRAIN_RESOLUTION_CAP", "257")
