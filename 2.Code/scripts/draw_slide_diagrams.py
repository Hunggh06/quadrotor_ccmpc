"""Ve cac so do minh hoa cho slide thuyet trinh CC-MPC.

Phong cach: nen trang, khoi mau tim, chu tieng Viet.

He toa do: moi so do dung mot khung logic rong x cao, va ti le o vuong duoc
giu dung nen hinh tron / goc bo tron khong bi meo.

Cach dung:
    python scripts/draw_slide_diagrams.py --out "<thu muc anh>"

Script chi ve hinh, khong dung den mo phong.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import textwrap

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Ellipse, FancyArrowPatch, FancyBboxPatch, Polygon

PURPLE = "#7C3AED"
PURPLE_D = "#5B21B6"
PURPLE_M = "#C4B5FD"
PURPLE_L = "#EDE9FE"
DARK = "#1E1B4B"
GREY = "#6B7280"
GREY_L = "#D1D5DB"
RED = "#DC2626"
GREEN = "#15803D"
AMBER = "#D97706"
TEAL = "#0E7490"

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    }
)


class Board:
    """Khung ve phang, ti le o vuong (20 don vi = 1 inch)."""

    SCALE = 20.0

    def __init__(self, lw: float, lh: float) -> None:
        self.lw, self.lh = lw, lh
        self.fig = plt.figure(figsize=(lw / self.SCALE, lh / self.SCALE), dpi=190)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, lw)
        self.ax.set_ylim(0, lh)
        self.ax.axis("off")
        self.ax.set_facecolor("white")

    # -- khoi --------------------------------------------------------------
    def box(
        self,
        cx,
        cy,
        w,
        h,
        text="",
        fc=PURPLE_L,
        ec=PURPLE,
        tc=DARK,
        fs=11.5,
        lw=2.0,
        bold=False,
        wrap=26,
        rounding=2.4,
        zorder=3,
        alpha=1.0,
    ):
        corner = min(rounding, w / 2.4, h / 2.4)
        self.ax.add_patch(
            FancyBboxPatch(
                (cx - w / 2, cy - h / 2),
                w,
                h,
                boxstyle=f"round,pad=0,rounding_size={corner}",
                facecolor=fc,
                edgecolor=ec,
                lw=lw,
                zorder=zorder,
                alpha=alpha,
            )
        )
        if text:
            self.ax.text(
                cx,
                cy,
                self._wrap(text, wrap),
                ha="center",
                va="center",
                fontsize=fs,
                color=tc,
                zorder=zorder + 1,
                fontweight="bold" if bold else "normal",
                linespacing=1.42,
            )

    def diamond(self, cx, cy, w, h, text, fs=11, fc=PURPLE_L, ec=PURPLE_M, tc=DARK, wrap=22):
        self.ax.add_patch(
            Polygon(
                [(cx, cy + h / 2), (cx + w / 2, cy), (cx, cy - h / 2), (cx - w / 2, cy)],
                closed=True,
                facecolor=fc,
                edgecolor=ec,
                lw=2.0,
                zorder=3,
            )
        )
        self.ax.text(
            cx,
            cy,
            self._wrap(text, wrap),
            ha="center",
            va="center",
            fontsize=fs,
            color=tc,
            zorder=4,
            linespacing=1.35,
        )

    def note(self, cx, cy, text, fs=10.5, color=GREY, wrap=40, ha="center", bold=False):
        self.ax.text(
            cx,
            cy,
            self._wrap(text, wrap),
            ha=ha,
            va="center",
            fontsize=fs,
            color=color,
            zorder=6,
            linespacing=1.45,
            fontweight="bold" if bold else "normal",
        )

    def arrow(self, p, q, color=PURPLE, lw=2.2, rad=0.0, style="-|>", ms=16, ls="-", zorder=2):
        self.ax.add_patch(
            FancyArrowPatch(
                p,
                q,
                arrowstyle=style,
                mutation_scale=ms,
                lw=lw,
                color=color,
                connectionstyle=f"arc3,rad={rad}",
                zorder=zorder,
                linestyle=ls,
                shrinkA=1.5,
                shrinkB=1.5,
            )
        )

    def save(self, path: pathlib.Path) -> None:
        self.fig.savefig(path)
        plt.close(self.fig)

    @staticmethod
    def _wrap(text: str, wrap: int) -> str:
        lines: list[str] = []
        for para in str(text).split("\n"):
            lines.extend(textwrap.wrap(para, wrap) or [""])
        return "\n".join(lines)


def path_line(p, q, n=160):
    t = np.linspace(0, 1, n)
    return np.column_stack([p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]), t


def unit_normal(p, q):
    d = np.array([q[0] - p[0], q[1] - p[1]], dtype=float)
    d /= np.linalg.norm(d)
    return np.array([-d[1], d[0]])


# --------------------------------------------------------------------------
# 1. Bai toan tranh va cham
# --------------------------------------------------------------------------


def diagram_problem(out: pathlib.Path) -> None:
    b = Board(200, 100)
    ax = b.ax

    start = np.array([24.0, 28.0])
    goal = np.array([182.0, 66.0])
    static_c = np.array([98.0, 46.0])
    static_r = 9.5
    move_a = np.array([152.0, 74.0])
    move_b = np.array([158.0, 22.0])
    move_r = 6.0

    line, t = path_line(start, goal)
    ax.plot(line[:, 0], line[:, 1], color=GREY, lw=1.6, ls="--", zorder=1)
    b.note(120.0, 62.0, "đường bay thẳng", fs=9.5, color=GREY, wrap=24)

    n = unit_normal(start, goal)
    curve = line - 19.0 * np.sin(math.pi * t)[:, None] * n[None, :]
    ax.plot(curve[:, 0], curve[:, 1], color=PURPLE, lw=3.2, zorder=5)
    b.note(128.0, 19.0, "quỹ đạo thực tế\n(vòng qua vật cản)", fs=10, color=PURPLE_D, wrap=30)

    ax.add_patch(
        Circle(static_c, static_r, facecolor=RED, alpha=0.20, edgecolor=RED, lw=2.0, zorder=3)
    )
    b.note(static_c[0], static_c[1], "vật cản\ntĩnh", fs=9.5, color=RED, wrap=12)

    ax.plot([move_a[0], move_b[0]], [move_a[1], move_b[1]], color=AMBER, lw=1.6,
            ls=":", zorder=2)
    ax.add_patch(
        Circle(move_b, move_r, facecolor=AMBER, alpha=0.26, edgecolor=AMBER, lw=2.0, zorder=3)
    )
    ax.add_patch(
        Circle(move_a, move_r, facecolor=AMBER, alpha=0.09, edgecolor=AMBER, lw=1.2, ls="--",
               zorder=2)
    )
    b.arrow((move_a[0] + 6.0, move_a[1]), (move_a[0] + 6.0, move_a[1] - 15.0), color=AMBER,
            lw=2.4, ms=15)
    b.note(170.0, 38.0, "vật cản động\nv = 0,38 m/s", fs=9.5, color=AMBER, wrap=16, ha="left")

    ell_c = curve[40] + np.array([0.0, -3.0])
    ax.add_patch(
        Ellipse(ell_c, 16.0, 11.0, angle=-24, facecolor=TEAL, alpha=0.17, edgecolor=TEAL,
                lw=1.4, ls="--", zorder=4)
    )
    b.note(38.0, 13.0, "vùng bất định vị trí", fs=9.5, color=TEAL, wrap=22)
    b.arrow((52.0, 14.5), (ell_c[0] - 8.0, ell_c[1] - 3.0), color=TEAL, lw=1.3, ms=12, ls="--",
            rad=-0.15)

    ax.add_patch(Circle(start, 3.0, facecolor=GREEN, edgecolor="white", lw=2.0, zorder=7))
    b.note(24.0, 19.0, "xuất phát", fs=10, color=GREEN, wrap=14)
    ax.plot(*goal, marker="*", ms=24, color=AMBER, mec="white", mew=1.4, zorder=7, ls="none")
    b.note(189.0, 68.0, "đích", fs=10.5, color=AMBER, wrap=10, ha="left")

    b.box(100.0, 89.0, 186.0, 17.0, "", fc="#FAFAFF", ec=GREY_L, lw=1.4, rounding=2.2)
    b.note(
        100.0,
        89.0,
        "Drone bay từ điểm xuất phát tới đích, tránh vật cản tĩnh và vật cản động,\n"
        "trong khi vị trí của chính nó và của vật cản đều chỉ biết một cách KHÔNG CHẮC CHẮN.",
        fs=11.5,
        color=DARK,
        wrap=110,
    )
    b.save(out / "so_do_bai_toan.png")


# --------------------------------------------------------------------------
# 2. Nguyen ly MPC
# --------------------------------------------------------------------------


def diagram_mpc(out: pathlib.Path) -> None:
    b = Board(200, 100)
    ax = b.ax

    box_w, box_h = 44.0, 20.0
    a = (30.0, 74.0)
    bb = (30.0, 26.0)
    c = (80.0, 26.0)
    d = (80.0, 74.0)

    b.box(*a, box_w, box_h, "1. Đọc trạng thái hiện tại\n(có nhiễu, đã ước lượng)",
          fc=PURPLE_L, ec=PURPLE, fs=11, wrap=30)
    b.box(*bb, box_w, box_h, "2. Dự đoán 20 bước tới\ncho drone và vật cản",
          fc="#F5F3FF", ec=PURPLE_M, fs=11, wrap=30)
    b.box(*c, box_w, box_h, "3. Tối ưu cả chuỗi 20\nbước điều khiển một lượt",
          fc=PURPLE_L, ec=PURPLE, fs=11, wrap=30)
    b.box(*d, box_w, box_h, "4. CHỈ thực hiện bước\nđầu tiên, vứt 19 bước sau",
          fc="#DDD6FE", ec=PURPLE_D, fs=11, wrap=30, lw=2.4)

    b.arrow((a[0], a[1] - box_h / 2), (bb[0], bb[1] + box_h / 2))
    b.arrow((bb[0] + box_w / 2, bb[1]), (c[0] - box_w / 2, c[1]))
    b.arrow((c[0], c[1] + box_h / 2), (d[0], d[1] - box_h / 2))
    b.arrow((d[0] - box_w / 2, d[1]), (a[0] + box_w / 2, a[1]), color=GREY, lw=2.0, rad=-0.22)
    b.note(55.0, 50.0, "dịch cửa sổ lên 1 bước\n(k+1) rồi lặp lại", fs=9.5, color=GREY, wrap=24)
    b.note(55.0, 88.0, "Mỗi chu kỳ 50 ms (20 Hz)", fs=11.5, color=PURPLE_D, wrap=32, bold=True)

    ax.plot([100.0, 100.0], [8.0, 92.0], color=GREY_L, lw=1.6, ls="--")

    x0, bar_w, bar_h = 116.0, 62.0, 9.0
    labels = ("bước k", "bước k+1", "bước k+2")
    ys = (70.0, 52.0, 34.0)
    for i, (label, y) in enumerate(zip(labels, ys)):
        sx = x0 + i * 6.0
        ax.add_patch(
            FancyBboxPatch(
                (sx, y), bar_w, bar_h,
                boxstyle="round,pad=0,rounding_size=1.4",
                facecolor=PURPLE_M, edgecolor=PURPLE, lw=1.3, alpha=0.55, zorder=3,
            )
        )
        ax.add_patch(
            FancyBboxPatch(
                (sx, y), 6.5, bar_h,
                boxstyle="round,pad=0,rounding_size=1.2",
                facecolor=PURPLE_D, edgecolor=PURPLE_D, lw=1.0, zorder=4,
            )
        )
        b.note(sx - 3.0, y + bar_h / 2, label, fs=10, color=DARK, wrap=12, ha="right")

    b.note(150.0, 87.0, "Cửa sổ dự đoán trượt dần về phía trước", fs=11, color=PURPLE_D, wrap=44,
           bold=True)
    b.note(116.0, 24.0, "ô đậm = bước thực sự được đưa xuống động cơ", fs=10, color=PURPLE_D,
           wrap=48, ha="left")
    b.note(116.0, 13.0, "Vì chỉ tin 1 bước, MPC đổi được kế hoạch\nngay khi vật cản trở mặt.",
           fs=10, color=GREY, wrap=48, ha="left")
    b.save(out / "so_do_mpc.png")


# --------------------------------------------------------------------------
# 3. Danh dinh so voi co xet bat dinh
# --------------------------------------------------------------------------


def diagram_nominal_vs_cc(out: pathlib.Path) -> None:
    b = Board(200, 100)
    ax = b.ax

    sigma_z = 2.935
    sigma_demo = 4.0

    panels = (
        dict(ox=0.0, keepout=11.0, cautious=False),
        dict(ox=102.0, keepout=11.0 + sigma_z * sigma_demo, cautious=True),
    )

    for panel in panels:
        ox = panel["ox"]
        pill = np.array([ox + 20.0, 74.0])
        goal = np.array([ox + 88.0, 26.0])
        obs = np.array([ox + 64.0, 58.0])
        base_r = 11.0
        keepout = panel["keepout"]

        line, t = path_line(pill, goal)
        n = unit_normal(pill, goal)

        ax.add_patch(
            Circle(obs, base_r, facecolor=RED, alpha=0.18, edgecolor=RED, lw=1.8, zorder=3)
        )
        ax.add_patch(
            Circle(obs, keepout, facecolor="none", edgecolor=RED if not panel["cautious"]
                   else PURPLE, lw=1.8, ls=":", zorder=2)
        )

        if panel["cautious"]:
            ax.add_patch(
                Circle(obs, keepout, facecolor=PURPLE, alpha=0.09, edgecolor=PURPLE, lw=2.0,
                       zorder=2)
            )
            dip = 21.0
            curve = line - dip * np.sin(math.pi * t)[:, None] * n[None, :]
            ax.plot(curve[:, 0], curve[:, 1], color=PURPLE, lw=3.2, zorder=6)
            ax.add_patch(
                Ellipse(pill + np.array([3.0, -3.0]), 11.0, 15.0, angle=-30, facecolor=TEAL,
                        alpha=0.18, edgecolor=TEAL, lw=1.4, ls="--", zorder=6)
            )
            ax.text(ox + 54.0, 92.0, "CC-MPC — xét cả độ bất định", ha="center", va="center",
                    fontsize=12.5, color=DARK, fontweight="bold")
            ax.text(ox + 54.0, 83.5,
                    "$P(\\mathrm{va\\ ch\\!\\!am}_k) \\leq \\delta$  được ép buộc mỗi bước",
                    ha="center", va="center", fontsize=11.5, color=PURPLE_D)
            b.note(ox + 54.0, 10.0,
                   "Vùng tránh = bán kính vật cản + $z_{1-\\delta}\\,\\sigma$.\n"
                   "Quỹ đạo bị đẩy ra xa đúng bằng phần phình thêm đó.",
                   fs=10, color=PURPLE_D, wrap=52)
        else:
            ax.plot(line[:, 0], line[:, 1], color=PURPLE, lw=3.2, zorder=6)
            ax.text(ox + 54.0, 92.0, "MPC thông thường — chỉ xét vị trí danh định",
                    ha="center", va="center", fontsize=12.5, color=DARK, fontweight="bold")
            ax.text(ox + 54.0, 83.5, "$P(\\mathrm{va\\ ch\\!\\!am}) \\leq \\delta$  không được kiểm tra",
                    ha="center", va="center", fontsize=11.5, color=RED)
            b.note(ox + 54.0, 10.0,
                   "Vùng tránh = đúng bán kính vật cản. Đường bay cắt sát mép,\n"
                   "nhìn thì hợp lệ nhưng chỉ đúng nếu vị trí là CHÍNH XÁC.",
                   fs=10, color=RED, wrap=52)

        ax.add_patch(Circle(pill, 2.8, facecolor=PURPLE if panel["cautious"] else GREEN,
                            edgecolor="white", lw=1.8, zorder=7))
        ax.plot(*goal, marker="*", ms=22, color=AMBER, mec="white", mew=1.2, ls="none",
                zorder=7)

    ax.plot([100.0, 100.0], [6.0, 96.0], color=GREY_L, lw=1.8)
    b.save(out / "so_do_danh_dinh_vs_bat_dinh.png")


# --------------------------------------------------------------------------
# 4. Kien truc vong kin
# --------------------------------------------------------------------------


def diagram_loop(out: pathlib.Path) -> None:
    b = Board(200, 100)

    w, h = 56.0, 24.0
    b.box(32.0, 80.0, w, h,
          "MuJoCo\nmô hình động học drone\n+ 3 vật cản", fc=PURPLE_L, ec=PURPLE, fs=10.5,
          wrap=26)
    b.box(100.0, 80.0, w, h,
          "Cảm biến có nhiễu\nvị trí ±3 cm\nvận tốc ±6 cm/s\nvị trí vật cản ±5 cm",
          fc="#F5F3FF", ec=PURPLE_M, fs=10.5, wrap=26)
    b.box(168.0, 80.0, w, h,
          "Bộ ước lượng\nESEKF cho drone\n+ lọc Kalman cho vật cản", fc=PURPLE_L, ec=PURPLE,
          fs=10.5, wrap=26)

    b.box(168.0, 22.0, w, h,
          "Bộ điều khiển CC-MPC\n13 trạng thái, tầm nhìn 20 bước\nràng buộc xác suất va chạm",
          fc="#DDD6FE", ec=PURPLE_D, fs=10.5, wrap=26, lw=2.6)
    b.box(100.0, 22.0, w, h,
          "Giám sát an toàn\n8 cổng kiểm tra + dự phòng PD\n(từ chối lệnh chưa chứng nhận)",
          fc="#F5F3FF", ec=PURPLE_M, fs=10.5, wrap=26)
    b.box(32.0, 22.0, w, h,
          "Bộ trộn động cơ\nlực đẩy + 3 mô-men\n→ 4 động cơ", fc=PURPLE_L, ec=PURPLE, fs=10.5,
          wrap=26)

    b.arrow((60.0, 80.0), (72.0, 80.0))
    b.arrow((128.0, 80.0), (140.0, 80.0))
    b.arrow((168.0, 68.0), (168.0, 34.0), lw=2.4)
    b.arrow((140.0, 22.0), (128.0, 22.0))
    b.arrow((72.0, 22.0), (60.0, 22.0))
    b.arrow((32.0, 34.0), (32.0, 68.0), lw=2.8, color=PURPLE_D)

    b.note(172.5, 51.0, "trạng thái +\nhiệp phương sai", fs=9, color=GREY, wrap=18, ha="left")
    b.note(27.5, 51.0, "lực đẩy\n+ mô-men", fs=9, color=GREY, wrap=16)

    b.box(100.0, 51.0, 76.0, 20.0, "", fc="#FFF7ED", ec=AMBER, lw=1.6, rounding=2.2)
    b.note(100.0, 51.0,
           "Vòng kín, mỗi 50 ms (20 Hz)\nToàn bộ hệ chạy trong mô phỏng MuJoCo:\n"
           "không có camera thật, không có ROS2, không có VIO.",
           fs=10.5, color=AMBER, wrap=46)
    b.save(out / "so_do_kien_truc.png")


# --------------------------------------------------------------------------
# 5. Chu trinh hoat dong va xu ly tinh huong
# --------------------------------------------------------------------------


def diagram_cycle(out: pathlib.Path) -> None:
    b = Board(200, 124)
    ax = b.ax

    x = 50.0           # truc chinh
    w = 64.0
    xl = 10.0          # truc vong lap phan hoi ben trai
    xr = 152.0         # cot tinh huong ben phai
    wr = 84.0

    b.box(x, 112.0, w, 11.0, "Bắt đầu bước điều khiển (mỗi 50 ms)", fc=PURPLE_D, ec=PURPLE_D,
          tc="white", fs=10.5, wrap=32, bold=True)
    b.box(x, 97.0, w, 12.0, "Ước lượng trạng thái drone + vật cản\n(ESEKF và lọc Kalman)",
          fc=PURPLE_L, ec=PURPLE, fs=10.5, wrap=32)
    b.box(x, 81.0, w, 12.0, "Dự đoán quỹ đạo và lan truyền\nhiệp phương sai 20 bước tới",
          fc=PURPLE_L, ec=PURPLE, fs=10.5, wrap=32)
    b.diamond(x, 62.0, 70.0, 18.0, "Quỹ đạo khả thi\nvà biên an toàn > 0 ?", fs=10.5, wrap=22)
    b.box(x, 42.0, w, 12.0, "Giải bài toán tối ưu bằng IPOPT\n(≈ 22 vòng lặp, tối đa 60)",
          fc=PURPLE_L, ec=PURPLE, fs=10.5, wrap=32)
    b.diamond(x, 22.0, 74.0, 18.0, "Chứng nhận rủi ro\nCERTIFIED ?", fs=10.5, wrap=22)
    b.box(x, 5.0, w, 10.0, "Gửi lực đẩy + mô-men → sang bước k+1", fc="#DDD6FE", ec=PURPLE_D,
          fs=10.5, wrap=34, bold=True)

    b.arrow((x, 106.5), (x, 103.0))
    b.arrow((x, 91.0), (x, 87.0))
    b.arrow((x, 75.0), (x, 71.0))
    b.arrow((x, 53.0), (x, 48.0))
    b.arrow((x, 36.0), (x, 31.0))
    b.arrow((x, 13.0), (x, 10.0))

    b.note(x + 4.0, 105.0, "có", fs=9.5, color=GREEN, wrap=8, ha="left")
    b.note(x + 4.0, 33.5, "có", fs=9.5, color=GREEN, wrap=8, ha="left")

    # nhanh "khong co nghiem": tang slack roi giai lai (vong lap phan hoi cuc bo)
    ax.plot([x + 35.0, x + 46.0], [62.0, 58.0], color=RED, lw=1.9, ls="--")
    ax.plot([x + 46.0, x + 46.0], [58.0, 42.0], color=RED, lw=1.9, ls="--")
    b.arrow((x + 46.0, 42.0), (x + w / 2 + 1.0, 42.0), color=RED, ls="--", lw=1.9, ms=14)
    b.note(x + 50.0, 52.0, "không → nới bán kính\n(slack), hạ tốc độ,\ngiải lại", fs=9.0,
           color=RED, wrap=20, ha="left")

    # vong lap dieu khien chinh
    ax.plot([x - w / 2, xl], [5.0, 5.0], color=PURPLE_D, lw=2.2)
    ax.plot([xl, xl], [5.0, 112.0], color=PURPLE_D, lw=2.2)
    b.arrow((xl, 112.0), (x - w / 2, 112.0), color=PURPLE_D, lw=2.2)
    ax.text(xl - 3.5, 58.0, "lặp lại mỗi 50 ms", rotation=90, ha="center", va="center",
            fontsize=9.5, color=PURPLE_D)

    ax.text(xr, 117.0, "Ba tình huống phải xử lý", ha="center", va="center", fontsize=11.5,
            color=DARK, fontweight="bold")

    b.box(xr, 101.0, wr, 18.0,
          "Vật cản đổi hướng\nlọc cập nhật vận tốc → bán kính siết đổi\n→ quỹ đạo mới",
          fc="#ECFEFF", ec=TEAL, fs=9.6, wrap=40)
    b.arrow((xr - wr / 2, 101.0), (x + w / 2 + 2.0, 97.0), color=TEAL, ls="--", lw=1.7, ms=13)

    b.box(xr, 78.0, wr, 18.0,
          "Nhiễu tăng\nσ tăng → bán kính siết rộng hơn\n→ bay thận trọng hơn",
          fc="#FFF7ED", ec=AMBER, fs=9.6, wrap=40)
    b.arrow((xr - wr / 2, 78.0), (x + w / 2 + 2.0, 81.0), color=AMBER, ls="--", lw=1.7, ms=13)

    b.box(xr, 22.0, wr, 26.0,
          "Chưa chứng nhận được rủi ro\n→ từ chối lệnh, dự phòng PD giữ lệnh cũ\n"
          "20 lần liên tiếp → chế độ khẩn cấp",
          fc="#FEF2F2", ec=RED, fs=9.6, wrap=40)
    b.arrow((x + 37.0, 22.0), (xr - wr / 2, 22.0), color=RED, ls="--", lw=1.7, ms=13)
    b.note(x + 41.0, 26.5, "không", fs=9.5, color=RED, wrap=8, ha="left")

    b.save(out / "so_do_chu_trinh.png")


BUILDERS = {
    "bai_toan": diagram_problem,
    "mpc": diagram_mpc,
    "danh_dinh_vs_bat_dinh": diagram_nominal_vs_cc,
    "kien_truc": diagram_loop,
    "chu_trinh": diagram_cycle,
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, help="thu muc ghi anh")
    parser.add_argument("--only", nargs="*", default=None)
    args = parser.parse_args()

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for name in args.only or list(BUILDERS):
        BUILDERS[name](out)
        print(f"  da ve so_do_{name}.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
