"""Shared look for the result figures: palette, fonts, recessive axes.

One place for the things every figure in rl/ must agree on, so the
training-progress graph and the town event map read as one report.
Colours are the same validated set the dashboard uses. The font stack
puts a CJK face first because every label in this repo is EN + JA and
DejaVu Sans (matplotlib's default) has no kana or kanji.

rl/ の図が共有する見た目：パレット・フォント・控えめな軸。学習進捗
グラフと町のイベント図が同じ体裁になるよう一箇所にまとめる。色は
ダッシュボードと同じ検証済みの組。本リポジトリのラベルは全て EN+JA
なので、かな・漢字を持たない DejaVu Sans（matplotlib 既定）の前に
CJK フォントを置く。
"""

import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager, rcParams  # noqa: E402

C_TRAIN = '#3987e5'    # training curves, arm A / 学習曲線・アームA
C_EVAL = '#199e70'     # evaluation, arm B / 評価・アームB
C_WARN = '#c98500'     # lane crossings / 車線逸脱
C_DANGER = '#e66767'   # collisions / 衝突
C_PURPLE = '#a64ac9'   # red lights / 信号無視
C_INK = '#333333'
C_MUTED = '#777777'
C_ROAD = '#d9d9d9'

# Study arms (docs/RESEARCH_PROTOCOL.md §3) -> colour + label. Labels are
# English only because they are printed inside the figures.
# 研究アーム → 色とラベル（A: 既定交通, B: 人間エラー交通, C: 自動操縦の
# 参照基線）。図中に印字されるため英語のみ。
ARM_STYLE = {
    'A': (C_TRAIN, 'A: default traffic'),
    'B': (C_EVAL, 'B: human-error traffic'),
    'C': (C_MUTED, 'C: autopilot baseline'),
}

_CJK_CANDIDATES = ('Noto Sans CJK JP', 'Noto Sans JP', 'IPAexGothic',
                   'IPAGothic', 'TakaoGothic', 'VL Gothic',
                   'Droid Sans Fallback')


def setup_fonts():
    """Prefer an installed CJK sans; fall back silently to DejaVu Sans.

    導入済みの CJK サンセリフを優先し、無ければ黙って DejaVu Sans。
    """
    installed = {f.name for f in font_manager.fontManager.ttflist}
    stack = [n for n in _CJK_CANDIDATES if n in installed]
    rcParams['font.family'] = 'sans-serif'
    rcParams['font.sans-serif'] = stack + ['DejaVu Sans']
    rcParams['axes.unicode_minus'] = False
    return stack[0] if stack else None


def style_axis(ax, title=None, xlabel=None, ylabel=None):
    """Recessive axes: no top/right spines, faint grid, muted labels.

    控えめな軸：上下枠なし・薄いグリッド・落ち着いたラベル。
    """
    if title is not None:
        ax.set_title(title, fontsize=10, color=C_INK, loc='left', pad=8)
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=8, color=C_MUTED)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=8, color=C_MUTED)
    ax.grid(True, alpha=0.25, linewidth=0.5)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color('#cccccc')
    ax.tick_params(colors=C_MUTED, labelsize=8)


setup_fonts()
