#!/usr/bin/env python3
import os
import re
import threading
import queue
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .dump import _read_metadata, dump_rosbag_to_single_csv   # reuse your logic

def list_topics_and_types(bag_dir: str):
    """Open a rosbag2 directory and return [(topic, type), ...]."""
    storage_id, ser = _read_metadata(bag_dir)
    storage_options = StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format=ser,
        output_serialization_format=ser,
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    tts = reader.get_all_topics_and_types()
    # Close underlying handle by letting reader go out of scope
    return sorted([(t.name, t.type) for t in tts], key=lambda x: x[0])

class App(tk.Tk):
    def __init__(self):
        from pathlib import Path
        super().__init__()
        self.title("rosbag_to_csv (ROS 2)")
        self.geometry("780x520")
        self.minsize(700, 480)

        HOME = str(Path.home())
        self.default_bag_root = os.path.join(HOME, "bags") if os.path.isdir(os.path.join(HOME, "bags")) else HOME
        self.bag_dir_var = tk.StringVar(value=self.default_bag_root)
        self.out_dir_var = tk.StringVar(value=os.path.join(HOME, "csvs"))
        self.stamp_mode_var = tk.StringVar(value="ns")

        self._topics = []       # [(topic, type)]
        self._checks = {}       # topic -> tk.IntVar()
        self._worker = None
        self._q = queue.Queue()

        self._build_ui()

    # ---------------- UI ----------------
    def _build_ui(self):
        frm_top = ttk.Frame(self, padding=10)
        frm_top.pack(fill="x")

        # Bag folder
        ttk.Label(frm_top, text="Bag directory:").grid(row=0, column=0, sticky="w")
        e_bag = ttk.Entry(frm_top, textvariable=self.bag_dir_var, width=80)
        e_bag.grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(frm_top, text="Browse…", command=self._browse_bag).grid(row=0, column=2)
        frm_top.columnconfigure(1, weight=1)

        # Output folder
        ttk.Label(frm_top, text="Output directory:").grid(row=1, column=0, sticky="w", pady=(6,0))
        e_out = ttk.Entry(frm_top, textvariable=self.out_dir_var, width=80)
        e_out.grid(row=1, column=1, sticky="ew", padx=6, pady=(6,0))
        ttk.Button(frm_top, text="Browse…", command=self._browse_out).grid(row=1, column=2, pady=(6,0))

        # Stamp mode
        frm_opts = ttk.Frame(self, padding=(10,0,10,0))
        frm_opts.pack(fill="x", pady=(4,0))
        ttk.Label(frm_opts, text="Timestamp:").grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(frm_opts, text="ns",  value="ns",  variable=self.stamp_mode_var).grid(row=0, column=1)
        ttk.Radiobutton(frm_opts, text="sec", value="sec", variable=self.stamp_mode_var).grid(row=0, column=2)

        # Topic list (with filter)
        frm_mid = ttk.Frame(self, padding=10)
        frm_mid.pack(fill="both", expand=True)

        topbar = ttk.Frame(frm_mid)
        topbar.pack(fill="x")
        ttk.Label(topbar, text="Filter topics (regex):").pack(side="left")
        self.filter_var = tk.StringVar(value="")
        e_filter = ttk.Entry(topbar, textvariable=self.filter_var, width=40)
        e_filter.pack(side="left", padx=6)
        ttk.Button(topbar, text="Apply", command=self._apply_filter).pack(side="left")
        ttk.Button(topbar, text="Select All", command=lambda: self._select_all(True)).pack(side="left", padx=(12,0))
        ttk.Button(topbar, text="Clear All", command=lambda: self._select_all(False)).pack(side="left", padx=6)

        cols = ("topic", "type")
        self.tree = ttk.Treeview(frm_mid, columns=cols, show="headings", selectmode="none")
        self.tree.heading("topic", text="Topic")
        self.tree.heading("type", text="Type")
        self.tree.column("topic", width=450)
        self.tree.column("type", width=280)

        # Add a checkbutton per row in column #0 using a separate listframe
        self.listframe = ttk.Frame(frm_mid)
        self.listframe.pack(fill="both", expand=True, pady=(6,0))
        self.canvas = tk.Canvas(self.listframe, highlightthickness=0)
        self.scroll = ttk.Scrollbar(self.listframe, orient="vertical", command=self.canvas.yview)
        self.inner = ttk.Frame(self.canvas)
        self.inner.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.create_window((0,0), window=self.inner, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scroll.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scroll.pack(side="right", fill="y")

        # Bottom bar (Load / Export)
        frm_bot = ttk.Frame(self, padding=10)
        frm_bot.pack(fill="x")
        self.progress = ttk.Progressbar(frm_bot, mode="indeterminate")
        self.progress.pack(side="left", fill="x", expand=True, padx=(0,10))
        ttk.Button(frm_bot, text="Load Topics", command=self._load_topics).pack(side="left")
        ttk.Button(frm_bot, text="Export Selected → CSV", command=self._export).pack(side="left", padx=(10,0))

    # ------------- Actions --------------
    def _browse_bag(self):
        initial = self.default_bag_root  # start at ~/bags or ~
        d = filedialog.askdirectory(
            title="Choose rosbag2 directory (contains metadata.yaml)",
            initialdir=initial
        )
        if d:
            self.bag_dir_var.set(d)

    def _browse_out(self):
        initial = os.path.join(os.path.expanduser("~"), "csvs")
        # Ensure ~/csvs exists (optional convenience)
        os.makedirs(initial, exist_ok=True)
        d = filedialog.askdirectory(
            title="Choose output directory",
            initialdir=initial
        )
        if d:
            self.out_dir_var.set(d)

    def _clear_topic_checks(self):
        for child in self.inner.winfo_children():
            child.destroy()
        self._checks.clear()

    def _populate_topic_checks(self, rows):
        self._clear_topic_checks()
        for topic, typ in rows:
            var = tk.IntVar(value=0)
            self._checks[topic] = var
            row = ttk.Frame(self.inner)
            row.pack(fill="x", padx=4, pady=2)
            ttk.Checkbutton(row, variable=var).pack(side="left")
            ttk.Label(row, text=topic).pack(side="left", padx=6)
            ttk.Label(row, text=typ, foreground="#666").pack(side="right")

    def _load_topics(self):
        bag = self.bag_dir_var.get().strip()
        if not bag or not os.path.exists(os.path.join(bag, "metadata.yaml")):
            messagebox.showerror("Error", "Please choose a valid rosbag2 directory (must contain metadata.yaml).")
            return
        try:
            rows = list_topics_and_types(bag)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read topics:\n{e}")
            return
        self._topics = rows
        self._populate_topic_checks(rows)

    def _apply_filter(self):
        if not self._topics:
            return
        pat = self.filter_var.get().strip()
        if not pat:
            filtered = self._topics
        else:
            try:
                rx = re.compile(pat)
            except re.error as e:
                messagebox.showerror("Regex error", f"{e}")
                return
            filtered = [r for r in self._topics if rx.search(r[0])]
        self._populate_topic_checks(filtered)

    def _select_all(self, state: bool):
        for v in self._checks.values():
            v.set(1 if state else 0)

    def _export(self):
        bag = self.bag_dir_var.get().strip()
        out = self.out_dir_var.get().strip()
        os.makedirs(out, exist_ok=True)
        if not bag or not os.path.exists(os.path.join(bag, "metadata.yaml")):
            messagebox.showerror("Error", "Please choose a valid rosbag2 directory (must contain metadata.yaml).")
            return
        if not out:
            messagebox.showerror("Error", "Please choose an output directory.")
            return
        selected = [t for t, v in self._checks.items() if v.get() == 1]
        if not selected:
            messagebox.showwarning("Nothing selected", "Pick at least one topic.")
            return

        # Build include patterns that match exactly the chosen topics
        includes = [f"^{re.escape(t)}$" for t in selected]
        stamp_mode = self.stamp_mode_var.get()

        # Run export in a background thread to keep UI responsive
        self.progress.start(10)
        self._worker = threading.Thread(
            target=self._export_worker,
            args=(bag, out, stamp_mode, includes),
            daemon=True
        )
        self._worker.start()
        self.after(100, self._poll_worker)

    def _export_worker(self, bag, out, stamp_mode, includes):
        try:
            out_csv = dump_rosbag_to_single_csv(
                bag_path=bag,
                output_dir=out,
                stamp_mode=stamp_mode,
                include_patterns=includes,
                exclude_patterns=[],  # user explicitly chose topics
            )
            self._q.put(("ok", out_csv))
        except Exception as e:
            self._q.put(("err", str(e)))

    def _poll_worker(self):
        try:
            kind, payload = self._q.get_nowait()
        except queue.Empty:
            self.after(100, self._poll_worker)
            return
        finally:
            self.progress.stop()

        if kind == "ok":
            messagebox.showinfo("Done", f"Merged CSV written to:\n{payload}")
        else:
            messagebox.showerror("Export failed", payload)

def main():
    app = App()
    app.mainloop()
