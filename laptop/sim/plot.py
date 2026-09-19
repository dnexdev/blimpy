"""Live top-down view of the simulated world (matplotlib, persistent artists so it stays cheap)."""
import math
from .. import config


class Plot:
    def __init__(self, world, title="Blimpy simulator"):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle
        self.plt = plt
        plt.ion()
        self.fig = plt.figure(figsize=(9.5, 6))
        self.fig.canvas.manager.set_window_title(title)
        gs = self.fig.add_gridspec(1, 2, width_ratios=[5, 1])
        ax = self.fig.add_subplot(gs[0]); self.ax = ax
        (x0, y0), (x1, y1) = world.arena
        ax.set_xlim(x0 - 0.3, x1 + 0.3); ax.set_ylim(y0 - 0.3, y1 + 0.3); ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, lw=2, color="k"))
        for cx, cy, cr in world.obstacles:
            ax.add_patch(Circle((cx, cy), cr, color="0.55"))
        if config.JUDGES_XY:
            ax.plot(*config.JUDGES_XY, "k*", ms=11, label="judges spot")
        self.trail, = ax.plot([], [], "-", color="tab:cyan", lw=1)
        self.body = Circle((0, 0), config.R_BALLOON, color="tab:blue", alpha=0.35); ax.add_patch(self.body)
        self.head, = ax.plot([], [], "-", color="tab:blue", lw=2.5)
        self.vel, = ax.plot([], [], "-", color="tab:green", lw=1.5)
        self.per, = ax.plot([], [], "^", color="tab:red", ms=12, label="person")
        self.txt = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", family="monospace", fontsize=9,
                           bbox=dict(boxstyle="round", fc="w", alpha=0.8))
        ax.legend(loc="lower right", fontsize=8)
        axz = self.fig.add_subplot(gs[1]); self.axz = axz
        axz.set_xlim(-1, 1); axz.set_ylim(0, 3); axz.set_xticks([]); axz.set_ylabel("height of balloon centre (m)")
        axz.axhline(config.FOLLOW["Z_HOLD"], color="g", ls="--", lw=1)
        axz.axhspan(0, 0.6, color="0.8"); axz.axhspan(2.8, 3, color="0.8")
        self.zdot, = axz.plot([0], [1.5], "o", color="tab:blue", ms=16)
        self.fig.tight_layout()
        self.pts = []

    def update(self, world):
        b, p, m, w = world.b, world.person.p, world.cur, world.wind_now
        self.pts = (self.pts + [(b.x, b.y)])[-400:]
        self.trail.set_data([q[0] for q in self.pts], [q[1] for q in self.pts])
        self.body.center = (b.x, b.y)
        self.head.set_data([b.x, b.x + 0.7 * math.cos(b.psi)], [b.y, b.y + 0.7 * math.sin(b.psi)])
        self.vel.set_data([b.x, b.x + 2.0 * b.vx], [b.y, b.y + 2.0 * b.vy])
        self.per.set_data([p[0]], [p[1]])
        self.zdot.set_data([0], [b.z])
        self.txt.set_text(f"t={world.t:6.1f}s   {'ARMED' if world.armed else 'motors off'}\n"
                          f"z={b.z:.2f} m  v={b.v:.2f} m/s  heading={math.degrees(b.psi):+4.0f} deg\n"
                          f"L={m[0]:+.2f} R={m[1]:+.2f} S={m[2]:+.2f} V={m[3]:+.2f}\n"
                          f"wind=({w[0]:+.2f},{w[1]:+.2f}) m/s   bumps={world.collisions}")
        self.plt.pause(0.001)
