"""配置文件加载和校验模块。注意：格式变更需同步控制器加载逻辑。"""


def load_config(path: str) -> dict[str, object]:
    """读取并解析仿真配置文件。注意：文件路径由调用方保证存在且可读。"""
    raise NotImplementedError

