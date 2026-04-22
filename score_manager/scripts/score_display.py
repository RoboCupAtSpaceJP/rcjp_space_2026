#!/usr/bin/env python3
import threading
import os
import yaml
import rospkg
import rospy
import tkinter as tk
from std_msgs.msg import Float64

# (label, yaml_key, max_points)
SCORE_ITEMS = [
    ("タスク開始",        "start_task",                   10),
    ("往路  航行",        "navigation_outbound",           20),
    ("往路  障害物回避",  "obstacle_avoidance_outbound",   20),
    ("往路  安全ボーナス","safety_bonus_outbound",         20),
    ("撮影タスク",        "search_task",                   40),
    ("復路  航行",        "navigation_return",             20),
    ("復路  障害物回避",  "obstacle_avoidance_return",     20),
    ("復路  安全ボーナス","safety_bonus_return",           20),
    ("ドッキング",        "docking_task",                  20),
    ("時間ボーナス",      "time_bonus",                    10),
]
MAX_TOTAL = sum(m for _, _, m in SCORE_ITEMS)

BG      = "#0d1117"
HEADER  = "#161b22"
FG      = "#e6edf3"
GREEN   = "#3fb950"
DIMGRAY = "#484f58"
YELLOW  = "#d29922"
RED     = "#f85149"
BLUE    = "#58a6ff"


class ScoreDisplay:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboCup@Space 2026")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        self.remaining = None
        self.time_limit = rospy.get_param('/competition/time_limit', 900.0)

        try:
            team = rospy.get_param('/competition/team_name', 'team')
            pkg = rospkg.RosPack().get_path('score_manager')
            self.yaml_path = os.path.join(pkg, 'scores', team, 'score.yaml')
        except Exception:
            self.yaml_path = None

        self.current_stage = rospy.get_param('/competition/stage', 1)
        self.current_trial = rospy.get_param('/competition/trial_number', 1)

        self._build_ui()
        rospy.Subscriber('/competition/remaining_time', Float64, self._cb_time)
        self._update()

    def _sep(self, pady=4):
        tk.Frame(self.root, bg=DIMGRAY, height=1).pack(fill='x', padx=20, pady=pady)

    def _build_ui(self):
        # ── ヘッダー ──────────────────────────────
        hdr = tk.Frame(self.root, bg=HEADER, pady=12)
        hdr.pack(fill='x')
        tk.Label(hdr, text="RoboCup@Space 2026", font=("Noto Sans CJK JP",20, "bold"),
                 bg=HEADER, fg=BLUE).pack()

        # ── 残り時間 ──────────────────────────────
        tf = tk.Frame(self.root, bg=BG, pady=10)
        tf.pack(fill='x', padx=20)
        tk.Label(tf, text="残り時間", font=("Noto Sans CJK JP",13),
                 bg=BG, fg=FG, anchor='w').pack(side='left')
        self.timer_label = tk.Label(tf, text="--:--",
                                     font=("Noto Sans CJK JP",26, "bold"), bg=BG, fg=GREEN)
        self.timer_label.pack(side='right')

        self._sep()

        # ── スコア行 ──────────────────────────────
        self.val_labels = {}
        for label, key, max_pt in SCORE_ITEMS:
            row = tk.Frame(self.root, bg=BG, pady=3)
            row.pack(fill='x', padx=24)
            tk.Label(row, text=label, font=("Noto Sans CJK JP",12),
                     bg=BG, fg=FG, width=20, anchor='w').pack(side='left')
            tk.Label(row, text=f"/ {max_pt}", font=("Noto Sans CJK JP",12),
                     bg=BG, fg=DIMGRAY, width=6, anchor='e').pack(side='right')
            val = tk.Label(row, text="0", font=("Noto Sans CJK JP",12, "bold"),
                           bg=BG, fg=DIMGRAY, width=4, anchor='e')
            val.pack(side='right')
            self.val_labels[key] = val

        self._sep(pady=8)

        # ── 合計点 ────────────────────────────────
        total_frame = tk.Frame(self.root, bg=BG, pady=8)
        total_frame.pack(fill='x', padx=20)
        tk.Label(total_frame,
                 text=f"合 計 点  (Stage {self.current_stage} / Trial {self.current_trial})",
                 font=("Noto Sans CJK JP",14), bg=BG, fg=FG).pack()
        self.total_label = tk.Label(total_frame, text=f"0",
                                     font=("Noto Sans CJK JP",64, "bold"), bg=BG, fg=BLUE)
        self.total_label.pack()
        self.max_label = tk.Label(total_frame, text=f"/ {MAX_TOTAL}",
                                   font=("Noto Sans CJK JP",16), bg=BG, fg=DIMGRAY)
        self.max_label.pack()

        tk.Frame(self.root, bg=BG, height=12).pack()

    def _cb_time(self, msg):
        self.remaining = msg.data

    def _read_latest(self):
        if not self.yaml_path or not os.path.exists(self.yaml_path):
            return {}
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            trials = (data or {}).get('trials', [])
            matched = [t for t in trials
                       if t.get('stage') == self.current_stage
                       and t.get('trial_number') == self.current_trial]
            return matched[-1] if matched else {}
        except Exception:
            return {}

    def _update(self):
        # タイマー更新
        if self.remaining is not None:
            secs = max(0, int(self.remaining))
            m, s = divmod(secs, 60)
            color = RED if secs < 60 else YELLOW if secs < 180 else GREEN
            self.timer_label.config(text=f"{m:02d}:{s:02d}", fg=color)

        # スコア更新
        entry = self._read_latest()
        total = 0
        for _, key, _ in SCORE_ITEMS:
            val = int(entry.get(key, 0) or 0)
            total += val
            self.val_labels[key].config(
                text=str(val),
                fg=GREEN if val > 0 else DIMGRAY
            )
        self.total_label.config(text=str(total))

        self.root.after(500, self._update)


def main():
    rospy.init_node('score_display', anonymous=True)
    root = tk.Tk()
    ScoreDisplay(root)

    t = threading.Thread(target=rospy.spin, daemon=True)
    t.start()

    root.mainloop()


if __name__ == '__main__':
    main()
