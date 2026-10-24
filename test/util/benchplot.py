# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Chart style shared by the scripts that plot benchmark results.

A validated categorical palette in fixed slot order, a diverging pair with a
neutral midpoint, and the ink and chrome of a light surface; bars at most 24px
thick with 2px surface gaps between touching segments; hairline solid grids;
text in ink colors, never in a series color. Needs matplotlib (uv environment).
"""

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948")
NEUTRAL = "#b4b2a9"       # a part that is no series, e.g. a stack's base
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
# Diverging: red below, blue above, gray for nothing
DIVERGING = LinearSegmentedColormap.from_list("diverging", ["#e34948", "#f0efec", "#2a78d6"])

DPI = 150
PT_PER_PX = 72 / DPI
ROW_IN = 0.3            # one row of a horizontal bar chart
BAR_PX = 24             # bars at most this thick
FONT = 8

plt.rcParams["font.size"] = FONT


def style(ax, axis="x"):
    ax.set_facecolor(SURFACE)
    ax.grid(axis=axis, color=GRID, linewidth=PT_PER_PX, linestyle="-")
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(AXIS)
    ax.spines["bottom"].set_linewidth(PT_PER_PX)
    ax.tick_params(colors=INK_SECONDARY, labelsize=FONT, length=0)


def ink_on(color):
    """Ink or white, whichever reads on a fill: a hex color or an RGBA tuple."""
    if isinstance(color, str):
        rgb = [int(color[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    else:
        rgb = color[:3]
    lum = 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]
    return INK if lum > 0.5 else "white"


def titles(fig, title, subtitle):
    fig.text(0.01, 0.995, title, ha="left", va="top", fontsize=FONT + 3, color=INK,
             fontweight="bold")
    fig.text(0.01, 0.995 - 0.3 / fig.get_figheight(), subtitle, ha="left", va="top",
             fontsize=FONT, color=INK_SECONDARY)


def row_figure(nrows, width=9.0, header_in=1.0, left=0.17):
    height = header_in + nrows * ROW_IN + 0.5
    fig, ax = plt.subplots(figsize=(width, height), dpi=DPI)
    fig.patch.set_facecolor(SURFACE)
    fig.subplots_adjust(left=left, right=0.97, top=1 - header_in / height, bottom=0.45 / height)
    return fig, ax


def bar_height(ax):
    """Bar thickness in data units (one row = 1) for at most BAR_PX."""
    row_px = ax.get_window_extent().height / abs(ax.get_ylim()[0] - ax.get_ylim()[1])
    return min(0.6, BAR_PX / row_px)


def rows_layout(n, header_rows=1):
    """y positions of n rows: the first header_rows on top, a gap, then the rest."""
    return [i if i < header_rows else i + 0.6 for i in range(n)]


def stacked(ax, names, parts, colors, xmax, header_rows=1, value_format=lambda w: f"{100 * w:.0f}"):
    """Horizontal stacked bars: parts[k][i] is segment k of row i. Values are
    written inside the segments they fit in."""
    ys = rows_layout(len(names), header_rows)
    ax.set_ylim(ys[-1] + 0.8, -0.8)
    ax.set_xlim(0, xmax)
    style(ax)
    h = bar_height(ax)
    px_per_unit = ax.get_window_extent().width / xmax
    left = [0.0] * len(names)
    for seg, color in zip(parts, colors):
        ax.barh(ys, seg, left=left, height=h, color=color, edgecolor=SURFACE,
                linewidth=2 * PT_PER_PX)
        for y, x0, w in zip(ys, left, seg):
            text = value_format(w)
            if w * px_per_unit >= len(text) * FONT * 0.75 + 10:
                ax.text(x0 + w / 2, y, text, ha="center", va="center", fontsize=FONT - 1,
                        color=ink_on(color))
        left = [a + b for a, b in zip(left, seg)]
    ax.set_yticks(ys, names)
    for label in ax.get_yticklabels()[:header_rows]:
        label.set_fontweight("bold")
    return ys, left


def legend(fig, labels, colors, ncol, top_in=0.62):
    handles = [Patch(facecolor=c, edgecolor="none") for c in colors]
    fig.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.01, 1 - top_in / fig.get_figheight()),
               ncol=ncol, frameon=False, fontsize=FONT, labelcolor=INK, handlelength=1,
               handleheight=1, columnspacing=1.5)


def save(fig, out):
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)
    return out
