"""生成地形近景法线与反照率贴图。注意：一次性离线生成，产物 PNG 随仓库提交。"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parents[1]
# 默认输出到正式 QML 资产目录，Situation3DView.qml 以相对路径引用。
DEFAULT_OUTPUT = PROJECT_ROOT / "src" / "ui" / "gui" / "situation3d" / "qml" / "assets" / "terrain_detail_normal.png"
DEFAULT_ALBEDO_OUTPUT = PROJECT_ROOT / "src" / "ui" / "gui" / "situation3d" / "qml" / "assets" / "terrain_detail_albedo.png"


def _periodic_fbm(size: int, octaves: int, seed: int) -> np.ndarray:
    """生成四方连续的分形噪声高度图。注意：频域合成天然无缝，不需要边缘拼接。"""

    rng = np.random.default_rng(seed)
    freq_y = np.fft.fftfreq(size)[:, None]
    freq_x = np.fft.fftfreq(size)[None, :]
    radius = np.sqrt(freq_x * freq_x + freq_y * freq_y)
    radius[0, 0] = 1.0
    height = np.zeros((size, size), dtype=np.float64)
    for octave in range(octaves):
        # 每个倍频程一段环形频带，幅度按 1/f 衰减出岩石类粗糙度。
        low = 3.0 * (2.0 ** octave) / size
        high = low * 2.0
        band = ((radius >= low) & (radius < high)).astype(np.float64)
        phase = rng.uniform(0.0, 2.0 * np.pi, (size, size))
        spectrum = band * np.exp(1j * phase) / (radius ** 1.15)
        layer = np.real(np.fft.ifft2(spectrum))
        layer /= max(np.abs(layer).max(), 1e-9)
        height += layer * (0.62 ** octave)
    # 脊化:折叠负半区制造棱线,更接近岩面而不是云雾。
    height = 1.0 - np.abs(height / max(np.abs(height).max(), 1e-9))
    return height.astype(np.float32)


def build_normal_map(size: int, strength: float, seed: int) -> np.ndarray:
    """把噪声高度图转成切线空间法线贴图。注意：输出 uint8 RGB，Z 轴朝外。"""

    height = _periodic_fbm(size, octaves=5, seed=seed)
    # 周期梯度:相邻差分配合 roll,保证贴图边缘法线也无缝。
    grad_x = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * 0.5 * size * strength
    grad_y = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * 0.5 * size * strength
    normal_z = np.ones_like(height)
    length = np.sqrt(grad_x * grad_x + grad_y * grad_y + 1.0)
    normal = np.stack((-grad_x / length, -grad_y / length, normal_z / length), axis=-1)
    return ((normal * 0.5 + 0.5) * 255.0).clip(0, 255).astype(np.uint8)


def _periodic_blur(field: np.ndarray, radius: int, *, passes: int = 1) -> np.ndarray:
    """周期模糊二维贴图。注意：使用 roll 保持四方无缝。"""

    result = field.astype(np.float32, copy=True)
    for _ in range(passes):
        horizontal = sum(np.roll(result, shift, axis=1) for shift in range(-radius, radius + 1))
        horizontal /= radius * 2 + 1
        result = sum(np.roll(horizontal, shift, axis=0) for shift in range(-radius, radius + 1))
        result /= radius * 2 + 1
    return result


def _standardize(field: np.ndarray) -> np.ndarray:
    """把纹理信号标准化到稳定对比度。注意：极小方差时返回零场。"""

    centered = field - float(np.mean(field))
    deviation = float(np.std(centered))
    return centered / max(deviation, 1e-6)


def build_albedo_map(size: int, seed: int) -> np.ndarray:
    """生成低饱和岩石反照率乘色图。注意：亮度以接近白色为主，避免覆盖顶点配色。"""

    coarse = _periodic_fbm(size, octaves=5, seed=seed)
    fine = _periodic_fbm(size, octaves=6, seed=seed + 97)
    coarse_variation = _standardize(_periodic_blur(coarse, 7, passes=2))
    grain = _standardize(fine - _periodic_blur(fine, 2, passes=1))
    # 两条等值带形成非规则裂隙网；宽窄错开，避免单一云噪声的塑料斑块感。
    crack_a = np.exp(-((fine - 0.46) / 0.026) ** 2)
    crack_b = np.exp(-((coarse - 0.66) / 0.019) ** 2)
    cracks = np.clip(0.68 * crack_a + 0.42 * crack_b, 0.0, 1.0)
    luminance = 0.982 + 0.007 * coarse_variation + 0.005 * grain - 0.22 * cracks
    luminance = np.clip(luminance, 0.68, 1.0)
    # 微弱冷灰通道差只提供岩性，不改变深绿灰/暖灰的宏观分区。
    color = np.stack((luminance * 0.985, luminance * 0.997, luminance * 1.012), axis=-1)
    return (np.clip(color, 0.0, 1.0) * 255.0).astype(np.uint8)


def main() -> int:
    """脚本入口。注意：默认参数即正式产物，重跑可完全复现。"""

    parser = argparse.ArgumentParser(description="生成四方连续的地形细节材质贴图。")
    parser.add_argument("--size", type=int, default=512, help="贴图边长像素，默认 512。")
    parser.add_argument("--strength", type=float, default=0.0135, help="法线扰动强度，默认 0.0135。")
    parser.add_argument("--seed", type=int, default=1949, help="随机种子，与地形布局保持一致。")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="PNG 输出路径。")
    parser.add_argument("--albedo-output", type=Path, default=DEFAULT_ALBEDO_OUTPUT, help="岩石反照率 PNG 输出路径。")
    args = parser.parse_args()
    normal = build_normal_map(int(args.size), float(args.strength), int(args.seed))
    albedo = build_albedo_map(int(args.size), int(args.seed))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.albedo_output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(normal, mode="RGB").save(args.output, "PNG")
    Image.fromarray(albedo, mode="RGB").save(args.albedo_output, "PNG")
    print(f"{args.output}  {args.output.stat().st_size / 1024.0:.1f} KB")
    print(f"{args.albedo_output}  {args.albedo_output.stat().st_size / 1024.0:.1f} KB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
