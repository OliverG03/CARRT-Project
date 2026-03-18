import tkinter as tk
from tkinter import font as tkfont
import time

# ── Color Palette ──────────────────────────────────────────────────────────────
BG_DARK     = "#0D0F14"
BG_PANEL    = "#141720"
BG_CARD     = "#1C2030"
ACCENT_CYAN = "#00D4FF"
ACCENT_GOLD = "#FFB800"
ACCENT_RED  = "#FF4455"
ACCENT_GREEN= "#00E5A0"
TEXT_PRIMARY= "#E8EAF0"
TEXT_DIM    = "#5A6075"
BORDER      = "#252A3A"

# ── Root Window ────────────────────────────────────────────────────────────────
root = tk.Tk()
root.title("Kinova Gen 3 — Control Interface")
root.geometry("1100x680")
root.configure(bg=BG_DARK)
root.resizable(False, False)

# ── Status tracking ────────────────────────────────────────────────────────────
status_var   = tk.StringVar(value="System Ready")
last_cmd_var = tk.StringVar(value="—")
connected_var= tk.StringVar(value="● CONNECTED")

# ── Canvas-based rounded rectangle helper ─────────────────────────────────────
def rounded_rect(canvas, x1, y1, x2, y2, r=12, **kwargs):
    pts = [
        x1+r, y1,   x2-r, y1,
        x2,   y1,   x2,   y1+r,
        x2,   y2-r, x2,   y2,
        x2-r, y2,   x1+r, y2,
        x1,   y2,   x1,   y2-r,
        x1,   y1+r, x1,   y1,
        x1+r, y1,
    ]
    return canvas.create_polygon(pts, smooth=True, **kwargs)

# ── Animated status dot ────────────────────────────────────────────────────────
dot_colors   = [ACCENT_GREEN, "#00BB80", "#009960"]
dot_index    = [0]
dot_label    = None  # assigned later

def pulse_dot():
    if dot_label:
        dot_label.config(fg=dot_colors[dot_index[0] % len(dot_colors)])
        dot_index[0] += 1
    root.after(700, pulse_dot)

# ── Command handler ────────────────────────────────────────────────────────────
def run_command(cmd, color):
    ts = time.strftime("%H:%M:%S")
    last_cmd_var.set(f"{cmd}  [{ts}]")
    status_var.set(f"Executing: {cmd}…")
    root.after(1800, lambda: status_var.set("System Ready"))
    log_event(cmd, ts, color)

def log_event(cmd, ts, color):
    log_canvas.config(state="normal")
    log_canvas.insert("end", f"  {ts}  ", ("dim",))
    log_canvas.insert("end", f"{cmd}\n", ("cmd",))
    log_canvas.see("end")
    log_canvas.config(state="disabled")

# ── Layout ─────────────────────────────────────────────────────────────────────

# ── Header bar ────────────────────────────────────────────────────────────────
header = tk.Frame(root, bg=BG_PANEL, height=64)
header.pack(fill="x", side="top")
header.pack_propagate(False)

tk.Label(header, text="KINOVA", bg=BG_PANEL, fg=ACCENT_CYAN,
         font=("Courier", 22, "bold")).pack(side="left", padx=(28,0), pady=14)
tk.Label(header, text=" GEN 3  /  CONTROL PANEL", bg=BG_PANEL, fg=TEXT_DIM,
         font=("Courier", 13)).pack(side="left", pady=14)

# connection badge
badge_frame = tk.Frame(header, bg=BG_PANEL)
badge_frame.pack(side="right", padx=28)
dot_label = tk.Label(badge_frame, text="●", bg=BG_PANEL, fg=ACCENT_GREEN,
                     font=("Courier", 10))
dot_label.pack(side="left")
tk.Label(badge_frame, textvariable=connected_var, bg=BG_PANEL, fg=TEXT_DIM,
         font=("Courier", 10)).pack(side="left", padx=(3,0))

# thin accent line under header
tk.Frame(root, bg=ACCENT_CYAN, height=1).pack(fill="x")

# ── Body ──────────────────────────────────────────────────────────────────────
body = tk.Frame(root, bg=BG_DARK)
body.pack(fill="both", expand=True, padx=28, pady=20)

# Left column: command buttons
left = tk.Frame(body, bg=BG_DARK, width=460)
left.pack(side="left", fill="y")
left.pack_propagate(False)

tk.Label(left, text="C O M M A N D S", bg=BG_DARK, fg=TEXT_DIM,
         font=("Courier", 9, "bold")).pack(anchor="w", pady=(0,14))

# ── Button factory ─────────────────────────────────────────────────────────────
def make_cmd_button(parent, label, sublabel, icon, accent, cmd_name):
    card = tk.Frame(parent, bg=BG_CARD, bd=0, highlightthickness=1,
                    highlightbackground=BORDER)
    card.pack(fill="x", pady=6, ipady=0)

    inner = tk.Frame(card, bg=BG_CARD)
    inner.pack(fill="x", padx=16, pady=14)

    # color swatch
    swatch = tk.Frame(inner, bg=accent, width=4)
    swatch.pack(side="left", fill="y", pady=2, padx=(0,14))

    # text block
    text_col = tk.Frame(inner, bg=BG_CARD)
    text_col.pack(side="left", fill="both", expand=True)
    tk.Label(text_col, text=f"{icon}  {label}", bg=BG_CARD, fg=TEXT_PRIMARY,
             font=("Courier", 13, "bold"), anchor="w").pack(fill="x")
    tk.Label(text_col, text=sublabel, bg=BG_CARD, fg=TEXT_DIM,
             font=("Courier", 9), anchor="w").pack(fill="x")

    # action button
    btn = tk.Button(inner, text="EXECUTE", bg=accent, fg=BG_DARK,
                    font=("Courier", 9, "bold"), relief="flat", cursor="hand2",
                    width=9, padx=6, pady=6,
                    command=lambda: run_command(cmd_name, accent))
    btn.pack(side="right", padx=(10,0))

    # hover effect
    def on_enter(e):
        card.config(highlightbackground=accent)
    def on_leave(e):
        card.config(highlightbackground=BORDER)
    card.bind("<Enter>", on_enter)
    card.bind("<Leave>", on_leave)
    for w in [inner, text_col, swatch]:
        w.bind("<Enter>", on_enter)
        w.bind("<Leave>", on_leave)

    return btn

make_cmd_button(left, "Clear Table",      "Remove all items from workspace",     "⬜", ACCENT_CYAN,  "Clear Table",callback="")
make_cmd_button(left, "Pick Up Item",     "Grasp target from designated zone",   "✋", ACCENT_GOLD,  "Pick Up Item",callback="")
make_cmd_button(left, "Give Medication",  "Dispense to patient hand position",   "💊", ACCENT_GREEN, "Give Medication",callback="")
make_cmd_button(left, "Return to Home",   "Move arm to neutral resting pose",    "🏠", TEXT_DIM,     "Return to Home",callback="")

# ── Divider ───────────────────────────────────────────────────────────────────
tk.Frame(body, bg=BORDER, width=1).pack(side="left", fill="y", padx=24)

# ── Right column: status + log ────────────────────────────────────────────────
right = tk.Frame(body, bg=BG_DARK)
right.pack(side="left", fill="both", expand=True)

tk.Label(right, text="STATUS", bg=BG_DARK, fg=TEXT_DIM,
         font=("Courier", 9, "bold")).pack(anchor="w", pady=(0,14))

# Status card
status_card = tk.Frame(right, bg=BG_CARD, bd=0,
                        highlightthickness=1, highlightbackground=BORDER)
status_card.pack(fill="x", pady=(0,14))

stat_inner = tk.Frame(status_card, bg=BG_CARD)
stat_inner.pack(fill="x", padx=16, pady=14)

tk.Label(stat_inner, text="SYSTEM STATUS", bg=BG_CARD, fg=TEXT_DIM,
         font=("Courier", 8)).pack(anchor="w")
tk.Label(stat_inner, textvariable=status_var, bg=BG_CARD, fg=ACCENT_CYAN,
         font=("Courier", 14, "bold")).pack(anchor="w", pady=(4,12))

tk.Frame(stat_inner, bg=BORDER, height=1).pack(fill="x", pady=(0,10))

tk.Label(stat_inner, text="LAST COMMAND", bg=BG_CARD, fg=TEXT_DIM,
         font=("Courier", 8)).pack(anchor="w")
tk.Label(stat_inner, textvariable=last_cmd_var, bg=BG_CARD, fg=TEXT_PRIMARY,
         font=("Courier", 11)).pack(anchor="w", pady=(4,0))

# Log
tk.Label(right, text="EVENT LOG", bg=BG_DARK, fg=TEXT_DIM,
         font=("Courier", 9, "bold")).pack(anchor="w", pady=(8,8))

log_frame = tk.Frame(right, bg=BG_CARD, bd=0,
                      highlightthickness=1, highlightbackground=BORDER)
log_frame.pack(fill="both", expand=True)

log_canvas = tk.Text(log_frame, bg=BG_CARD, fg=TEXT_DIM, relief="flat",
                      font=("Courier", 10), state="disabled",
                      insertbackground=ACCENT_CYAN, wrap="word",
                      padx=12, pady=10, cursor="arrow")
log_canvas.tag_config("dim", foreground=TEXT_DIM)
log_canvas.tag_config("cmd", foreground=ACCENT_GREEN)
log_canvas.pack(fill="both", expand=True)

# Scrollbar
sb = tk.Scrollbar(log_frame, command=log_canvas.yview, bg=BG_CARD,
                   troughcolor=BG_CARD, width=8)
sb.pack(side="right", fill="y")
log_canvas.config(yscrollcommand=sb.set)

# ── Footer ────────────────────────────────────────────────────────────────────
tk.Frame(root, bg=BORDER, height=1).pack(fill="x")
footer = tk.Frame(root, bg=BG_PANEL, height=32)
footer.pack(fill="x")
footer.pack_propagate(False)
tk.Label(footer, text="Kinova Gen 3  ·  7-DOF Robotic Arm  ·  v1.0.0",
         bg=BG_PANEL, fg=TEXT_DIM, font=("Courier", 8)).pack(side="left", padx=28, pady=8)
tk.Label(footer, text="Monroe, GA  ·  Lab Station 01",
         bg=BG_PANEL, fg=TEXT_DIM, font=("Courier", 8)).pack(side="right", padx=28, pady=8)

# ── Kickoff ───────────────────────────────────────────────────────────────────
pulse_dot()
log_event("System initialized", time.strftime("%H:%M:%S"), ACCENT_GREEN)
root.mainloop()

