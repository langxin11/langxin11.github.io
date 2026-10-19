import matplotlib.pyplot as plt
import numpy as np

def get_pub_config(column="single", lang="chinese"):
    width = 3.5 if column == "single" else 7.0
    height = 2.4 if column == "single" else 2.8

    serif_fonts = (
        ["SimSun", "Times New Roman", "DejaVu Serif"]
        if lang == "chinese"
        else ["Times New Roman", "DejaVu Serif"]
    )

    return {
        "figure.figsize": (width, height),
        "font.family": "serif",
        "font.serif": serif_fonts,
        "mathtext.fontset": "stix",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "font.size": 8.0,
        "axes.labelsize": 8.5,
        "axes.titlesize": 9.0,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.5,
        "legend.frameon": True,
        "legend.edgecolor": "0.85",
        "legend.fancybox": False,
        "legend.borderaxespad": 0.5,
        "lines.linewidth": 1.2,
        "lines.markersize": 3.5,
        "axes.linewidth": 0.8,
        "xtick.direction": "in",
        "ytick.direction": "in",
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "xtick.major.size": 3.0,
        "ytick.major.size": 3.0,
        "axes.unicode_minus": False,
        "axes.formatter.use_mathtext": False,
        "figure.dpi": 150,
        "savefig.dpi": 300,
    }

t = np.linspace(0, 8, 200)
ref = np.sin(t)
prop = ref * (1 - np.exp(-2.0 * t)) + 0.015 * np.sin(6 * t)
base = ref * (1 - np.exp(-1.0 * t)) + 0.06 * np.sin(4 * t)

# 1. Single-column figure with inset
cfg_single = get_pub_config(column="single", lang="chinese")
with plt.rc_context(cfg_single):
    fig, ax = plt.subplots(figsize=(3.5, 2.4), layout="constrained")
    ax.plot(t, ref, "k--", label=r"期望轨迹 $r(t)$", linewidth=1.0)
    ax.plot(t, prop, color="#0072BD", linestyle="-", marker="o", markevery=25,
            label=r"提出算法 (MPC)", linewidth=1.2)
    ax.plot(t, base, color="#D95319", linestyle="-.", marker="s", markevery=25,
            label=r"对比算法 (PID)", linewidth=1.2)

    # 规范国际标准单位表达：物理量用斜体，单位用正体 \mathrm{...}
    ax.set_xlabel(r"时间 $t$ / $\mathrm{s}$")
    ax.set_ylabel(r"位置 $p(t)$ / $\mathrm{m}$")
    ax.grid(True, linestyle=":", alpha=0.6, linewidth=0.5)
    ax.legend(loc="lower right")

    axins = ax.inset_axes([0.18, 0.55, 0.38, 0.38])
    axins.plot(t, ref, "k--", linewidth=1.0)
    axins.plot(t, prop, color="#0072BD", linestyle="-", marker="o", markevery=5, linewidth=1.2)
    axins.plot(t, base, color="#D95319", linestyle="-.", marker="s", markevery=5, linewidth=1.2)
    axins.set_xlim(1.2, 2.2)
    axins.set_ylim(0.85, 1.05)
    axins.grid(True, linestyle=":", alpha=0.5, linewidth=0.4)
    axins.tick_params(labelsize=6.5)
    ax.indicate_inset_zoom(axins, edgecolor="0.5", alpha=0.8)

    fig.savefig("posts/知乎迁入/matplotlib-chinese-mathtext/pub-single-column.png", dpi=300)
    plt.close(fig)

# 2. Two-panel subplots
cfg_double = get_pub_config(column="single", lang="chinese")
with plt.rc_context(cfg_double):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(3.5, 3.2), sharex=True, layout="constrained")

    # Subplot (a)
    ax1.plot(t, ref, "k--", label=r"期望 $r(t)$", linewidth=0.9)
    ax1.plot(t, prop, color="#0072BD", linestyle="-", label="MPC", linewidth=1.1)
    ax1.plot(t, base, color="#D95319", linestyle="-.", label="PID", linewidth=1.1)
    ax1.set_ylabel(r"状态 $x(t)$")
    ax1.grid(True, linestyle=":", alpha=0.6, linewidth=0.5)
    ax1.legend(loc="lower right", ncol=3)
    ax1.text(0.03, 0.88, "(a)", transform=ax1.transAxes, fontweight="bold", fontsize=8.5)

    # Subplot (b)
    u_prop = np.gradient(prop, t)
    u_base = np.gradient(base, t)
    ax2.plot(t, u_prop, color="#0072BD", linestyle="-", label="MPC 输入", linewidth=1.1)
    ax2.plot(t, u_base, color="#D95319", linestyle="-.", label="PID 输入", linewidth=1.1)
    ax2.axhline(1.5, color="gray", linestyle=":", linewidth=0.8, label="输入约束")
    ax2.axhline(-1.5, color="gray", linestyle=":", linewidth=0.8)
    ax2.set_xlabel(r"时间 $t$ / $\mathrm{s}$")
    ax2.set_ylabel(r"控制量 $u(t)$ / $\mathrm{N}$")
    ax2.grid(True, linestyle=":", alpha=0.6, linewidth=0.5)
    ax2.legend(loc="upper right", ncol=3)
    ax2.text(0.03, 0.88, "(b)", transform=ax2.transAxes, fontweight="bold", fontsize=8.5)

    fig.savefig("posts/知乎迁入/matplotlib-chinese-mathtext/pub-multi-panel.png", dpi=300)
    plt.close(fig)

print("Regenerated figures with standardized spaced upright units!")
