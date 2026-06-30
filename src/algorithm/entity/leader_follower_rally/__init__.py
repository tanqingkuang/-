"""领航跟随集结实体包。注意：长机与僚机实体保持独立导出，避免影响既有保持实体。"""

from src.algorithm.context.context import FormContextS
from src.algorithm.context.leaf_types import PosTrackDiagS, copy_motion, copy_pos_track_diag
from src.algorithm.entity.types import EntityOutputS


def fill_output(cxt: FormContextS, diag: PosTrackDiagS, outbox: list, y: EntityOutputS) -> None:
    """将 Context 中的计算结果回填到实体输出边界。"""
    if y.selfAccCmd is None:
        y.selfAccCmd = cxt.selfAccCmd
    else:
        y.selfAccCmd.accEast = cxt.selfAccCmd.accEast
        y.selfAccCmd.accNorth = cxt.selfAccCmd.accNorth
        y.selfAccCmd.accUp = cxt.selfAccCmd.accUp
    if y.selfCmd is None:
        y.selfCmd = cxt.selfCmd
    else:
        copy_motion(cxt.selfCmd, y.selfCmd)
    if y.controlDiag is None:
        y.controlDiag = diag
    else:
        copy_pos_track_diag(diag, y.controlDiag)
    y.outbox.clear()
    y.outbox.extend(outbox)
