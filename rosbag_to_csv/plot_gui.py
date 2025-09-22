#!/usr/bin/env python3
import os
import csv
import math
import tkinter as tk
from tkinter import ttk, filedialog, messagebox


def _default_csv_dir() -> str:
    home = os.path.expanduser("~")
    d = os.path.join(home, "csvs")
    os.makedirs(d, exist_ok=True)
    return d


def _list_csvs(path: str):
    try:
        return sorted([f for f in os.listdir(path) if f.lower().endswith('.csv')])
    except Exception:
        return []


def _find_time_column(header):
    if 'timestamp_sec' in header:
        return 'timestamp_sec', 'sec'
    if 'timestamp_ns' in header:
        return 'timestamp_ns', 'ns'
    # Fallback: try common names
    for c in header:
        if c.lower() in ('time', 'timestamp'):
            return c, 'sec'
    return None, None


def _find_accel_cmd_column(header):
    # Kept for backward compatibility (not used in new selection flow)
    lower = [h.lower() for h in header]
    for i, h in enumerate(lower):
        if h.endswith(':command'):
            return header[i]
    for i, h in enumerate(lower):
        if 'accel_cmd' in h:
            return header[i]
    for i, h in enumerate(lower):
        if 'accel' in h and 'cmd' in h:
            return header[i]
    return None


def _find_vehicle_speed_column(header):
    # Kept for backward compatibility (not used in new selection flow)
    lower = [h.lower() for h in header]
    for i, h in enumerate(lower):
        if ':vehicle_speed' in h:
            return header[i]
    for i, h in enumerate(lower):
        if 'speed' in h and 'steer' not in h and 'angular' not in h and 'wheel' not in h:
            return header[i]
    return None


def _find_lat_lon_columns(header):
    # Prefer explicit latitude/longitude
    lat = None
    lon = None
    lower = [h.lower() for h in header]
    for i, h in enumerate(lower):
        if h.endswith(':latitude') or h.endswith('.latitude') or h.endswith('/latitude') or h == 'latitude':
            lat = header[i]
        if h.endswith(':longitude') or h.endswith('.longitude') or h.endswith('/longitude') or h == 'longitude':
            lon = header[i]
    if lat and lon:
        return lat, lon
    # Try shortened lat/lon, but avoid 'longitudinal' matches
    for i, h in enumerate(lower):
        # Column part after last delimiter
        tail = header[i].split(':')[-1].split('.')[-1].split('/')[-1]
        tl = tail.lower()
        if tl == 'lat' or tl.endswith('_lat'):
            lat = header[i]
        if tl == 'lon' or tl.endswith('_lon'):
            lon = header[i]
    return lat, lon


def _convert_latlon_to_xy(lat_list, lon_list):
    # Simple equirectangular projection around first valid point
    # Returns x (east, m) and y (north, m)
    # Skip None values by carrying last known (to align lengths); use NaN for breaks
    n = len(lat_list)
    x = [math.nan] * n
    y = [math.nan] * n
    # Find origin
    origin_idx = None
    for i, (la, lo) in enumerate(zip(lat_list, lon_list)):
        if la is not None and lo is not None:
            origin_idx = i
            break
    if origin_idx is None:
        return x, y
    lat0 = math.radians(lat_list[origin_idx])
    lon0 = math.radians(lon_list[origin_idx])
    R_earth = 6378137.0  # meters
    for i, (la, lo) in enumerate(zip(lat_list, lon_list)):
        if la is None or lo is None:
            continue
        latr = math.radians(la)
        lonr = math.radians(lo)
        x[i] = (lonr - lon0) * math.cos((latr + lat0) / 2.0) * R_earth
        y[i] = (latr - lat0) * R_earth
    return x, y


class PlotApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CSV Plotter")
        self.geometry("720x420")
        self.minsize(640, 380)

        self.csv_dir_var = tk.StringVar(value=_default_csv_dir())
        self._current_csv_path = None
        self._time_col = None
        self._time_mode = None  # 'sec' or 'ns'
        self._signal_checks = {}  # All available signals (except timestamp)
        self._derived_options = {}  # key -> IntVar (e.g., 'pos_x', 'pos_y')
        self._lat_col = None
        self._lon_col = None
        self._build_ui()
        self._refresh_list()

    def _build_ui(self):
        frm = ttk.Frame(self, padding=10)
        frm.pack(fill='both', expand=True)

        # Directory selector
        ttk.Label(frm, text="CSV directory:").grid(row=0, column=0, sticky='w')
        e = ttk.Entry(frm, textvariable=self.csv_dir_var, width=70)
        e.grid(row=0, column=1, sticky='ew', padx=6)
        ttk.Button(frm, text="Browse…", command=self._browse_dir).grid(row=0, column=2)
        ttk.Button(frm, text="Refresh", command=self._refresh_list).grid(row=0, column=3, padx=(6,0))
        frm.columnconfigure(1, weight=1)

        # CSV listbox
        self.listbox = tk.Listbox(frm, height=15, selectmode='browse')
        self.scroll = ttk.Scrollbar(frm, orient='vertical', command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=self.scroll.set)
        self.listbox.grid(row=1, column=0, columnspan=3, sticky='nsew', pady=(8,0))
        self.scroll.grid(row=1, column=3, sticky='ns', pady=(8,0))
        self.listbox.bind('<<ListboxSelect>>', self._on_csv_selected)
        frm.rowconfigure(1, weight=1)

        # Signal selection panel (all signals)
        self.signals_frame = ttk.LabelFrame(frm, text="Available signals")
        self.signals_frame.grid(row=2, column=0, columnspan=4, sticky='ew', pady=(8,0))
        self.signals_canvas = tk.Canvas(self.signals_frame, height=180, highlightthickness=0)
        self.signals_scroll = ttk.Scrollbar(self.signals_frame, orient='vertical', command=self.signals_canvas.yview)
        self.signals_inner = ttk.Frame(self.signals_canvas)
        self.signals_inner.bind('<Configure>', lambda e: self.signals_canvas.configure(scrollregion=self.signals_canvas.bbox('all')))
        self.signals_canvas.create_window((0,0), window=self.signals_inner, anchor='nw')
        self.signals_canvas.configure(yscrollcommand=self.signals_scroll.set)
        self.signals_canvas.grid(row=0, column=0, sticky='nsew')
        self.signals_scroll.grid(row=0, column=1, sticky='ns')
        self.signals_frame.columnconfigure(0, weight=1)

        # Derived options (pos_x/pos_y)
        self.derived_frame = ttk.Frame(frm)
        self.derived_frame.grid(row=3, column=0, columnspan=4, sticky='ew', pady=(6,0))
        self.derived_label = ttk.Label(self.derived_frame, text="Derived:")
        self.derived_label.pack(side='left')
        # Derived checkboxes will be added dynamically

        # Overlay controls
        overlay_frame = ttk.Frame(frm)
        overlay_frame.grid(row=4, column=0, columnspan=4, sticky='ew')
        ttk.Label(overlay_frame, text="Overlay signals (comma-separated exact names):").pack(side='left')
        self.overlay_var = tk.StringVar(value="")
        ttk.Entry(overlay_frame, textvariable=self.overlay_var, width=60).pack(side='left', padx=(6,0))

        # Select all/none controls
        sel_ctrl = ttk.Frame(frm)
        sel_ctrl.grid(row=5, column=0, columnspan=2, sticky='ew', pady=(6,0))
        ttk.Button(sel_ctrl, text="Select All", command=lambda: self._select_all(self._signal_checks, True)).pack(side='left')
        ttk.Button(sel_ctrl, text="Clear", command=lambda: self._select_all(self._signal_checks, False)).pack(side='left', padx=(6,0))

            # Plot button
        btn = ttk.Button(frm, text="Plot", command=self._plot_selected)
        btn.grid(row=6, column=0, sticky='w', pady=(10,0))

        self.status = ttk.Label(frm, text="")
        self.status.grid(row=6, column=1, columnspan=3, sticky='e', pady=(10,0))

    def _browse_dir(self):
        d = filedialog.askdirectory(title="Choose CSV directory", initialdir=self.csv_dir_var.get())
        if d:
            self.csv_dir_var.set(d)
            self._refresh_list()

    def _refresh_list(self):
        self.listbox.delete(0, tk.END)
        for f in _list_csvs(self.csv_dir_var.get()):
            self.listbox.insert(tk.END, f)
        self.status.configure(text=f"Found {self.listbox.size()} CSV files")

    def _clear_signal_checks(self):
        for child in self.signals_inner.winfo_children():
            child.destroy()
        for child in self.derived_frame.winfo_children():
            # keep the label, remove others
            if child is not self.derived_label:
                child.destroy()
        self._signal_checks.clear()
        self._derived_options.clear()

    def _select_all(self, mapping, state: bool):
        for var in mapping.values():
            var.set(1 if state else 0)

    def _to_bool(self, v):
        if v is None:
            return None
        if isinstance(v, (int, float)):
            if v == 0:
                return 0
            if v == 1:
                return 1
        s = str(v).strip().lower()
        if s in ("1", "true", "yes", "on", "enable", "enabled"): return 1
        if s in ("0", "false", "no", "off", "disable", "disabled"): return 0
        return None

    def _to_float(self, v):
        if v is None or v == "":
            return None
        try:
            return float(v)
        except Exception:
            return None

    def _scan_signals(self, path: str):
        # Return all columns (except time) as available signals; also detect lat/lon
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            header = reader.fieldnames or []
            time_col, tmode = _find_time_column(header)
            if not time_col:
                raise RuntimeError("No timestamp column found (expected 'timestamp_ns' or 'timestamp_sec').")
            lat_col, lon_col = _find_lat_lon_columns(header)
            self._lat_col, self._lon_col = lat_col, lon_col
            signals = [h for h in header if h != time_col]
        return time_col, tmode, signals

    def _on_csv_selected(self, event=None):
        sel = self.listbox.curselection()
        if not sel:
            return
        f = self.listbox.get(sel[0])
        path = os.path.join(self.csv_dir_var.get(), f)
        self._current_csv_path = path
        try:
            time_col, tmode, signals = self._scan_signals(path)
        except Exception as e:
            self._clear_signal_checks()
            self.status.configure(text=f"Scan error: {e}")
            return

        self._time_col = time_col
        self._time_mode = tmode
        self._clear_signal_checks()

        # Populate signal checks (all)
        for col in sorted(signals):
            var = tk.IntVar(value=0)
            self._signal_checks[col] = var
            ttk.Checkbutton(self.signals_inner, variable=var, text=col).pack(anchor='w')

        # Add derived options if lat/lon available
        if self._lat_col and self._lon_col:
            for key, label in (("pos_x", "pos_x (from lat/lon, m)"), ("pos_y", "pos_y (from lat/lon, m)")):
                var = tk.IntVar(value=0)
                self._derived_options[key] = var
                ttk.Checkbutton(self.derived_frame, variable=var, text=label).pack(side='left', padx=(8,0))

        self.status.configure(text=f"Scanned: {len(signals)} signals available")

    def _plot_selected(self):
        if not self._current_csv_path:
            messagebox.showwarning("No CSV", "Please select a CSV file to plot.")
            return
        # Gather selections
        selected = [k for k, v in self._signal_checks.items() if v.get() == 1]
        derived_sel = [k for k, v in self._derived_options.items() if v.get() == 1]
        if not selected and not derived_sel:
            messagebox.showwarning("Nothing selected", "Pick at least one signal to plot.")
            return
        # Parse overlay list (exact names)
        overlay_list = []
        if hasattr(self, 'overlay_var') and self.overlay_var.get().strip():
            overlay_list = [s for s in (x.strip() for x in self.overlay_var.get().split(',')) if s]

        try:
            self._plot_csv(self._current_csv_path, selected, derived_sel, overlay_list)
        except ImportError:
            messagebox.showerror("Missing dependency", "matplotlib is required to plot. Install with 'pip install matplotlib' or your distro's package manager.")
        except Exception as e:
            messagebox.showerror("Plot error", str(e))

    def _plot_csv(self, path: str, selected_cols, derived_sel, overlay_cols):
        import matplotlib.pyplot as plt

        # Read all rows and extract selected columns
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            header = reader.fieldnames or []
            if self._time_col not in header:
                raise RuntimeError("Timestamp column missing from selected CSV.")

            t_vals = []
            # Prepare storage for signals (collect both float and bool candidates; classify later)
            numeric_data = {col: [] for col in selected_cols}
            boolean_data = {col: [] for col in selected_cols}
            lats = [] if ('pos_x' in derived_sel or 'pos_y' in derived_sel) else None
            lons = [] if ('pos_x' in derived_sel or 'pos_y' in derived_sel) else None

            for row in reader:
                ts = row.get(self._time_col, "")
                if ts == "" or ts is None:
                    continue
                t = float(ts) if self._time_mode == 'sec' else float(ts) / 1e9
                t_vals.append(t)

                for col in selected_cols:
                    v = row.get(col)
                    fv = self._to_float(v)
                    bv = self._to_bool(v)
                    numeric_data[col].append(fv)
                    boolean_data[col].append(bv if bv is not None else math.nan)

                if lats is not None:
                    lats.append(self._to_float(row.get(self._lat_col)) if self._lat_col else None)
                if lons is not None:
                    lons.append(self._to_float(row.get(self._lon_col)) if self._lon_col else None)

        if not t_vals:
            raise RuntimeError("No data rows found in CSV.")

        # Normalize time start to 0
        t0 = t_vals[0]
        t_vals = [t - t0 for t in t_vals]

        # Classify signals: numeric if any float present, else boolean if any 0/1 present
        numeric_cols = []
        boolean_cols = []
        for col in selected_cols:
            has_num = any(x is not None for x in numeric_data[col])
            has_bool = any((isinstance(x, (int, float)) and not math.isnan(x)) for x in boolean_data[col])
            if has_num:
                numeric_cols.append(col)
            elif has_bool:
                boolean_cols.append(col)
        # Prune data maps to only classified columns
        numeric_data = {c: numeric_data[c] for c in numeric_cols}
        boolean_data = {c: boolean_data[c] for c in boolean_cols}

        # Derived positions
        derived_data = {}
        if ('pos_x' in derived_sel or 'pos_y' in derived_sel) and lats is not None and lons is not None:
            xs, ys = _convert_latlon_to_xy(lats, lons)
            if 'pos_x' in derived_sel:
                derived_data['pos_x'] = xs
            if 'pos_y' in derived_sel:
                derived_data['pos_y'] = ys

        # Prepare overlay/remaining groups
        overlay_set = set(overlay_cols or [])
        # Restrict to selected (ignore unknown names)
        overlay_numeric = [c for c in numeric_data.keys() if c in overlay_set]
        overlay_boolean = [c for c in boolean_data.keys() if c in overlay_set]
        overlay_derived = [c for c in derived_data.keys() if c in overlay_set]

        remaining_numeric = [c for c in numeric_data.keys() if c not in overlay_set]
        remaining_boolean = [c for c in boolean_data.keys() if c not in overlay_set]
        remaining_derived = [c for c in derived_data.keys() if c not in overlay_set]

        # Count subplots
        n_subplots = (1 if (overlay_numeric or overlay_boolean or overlay_derived) else 0) \
                     + len(remaining_numeric) + len(remaining_boolean) + len(remaining_derived)

        if n_subplots == 0:
            raise RuntimeError("Nothing to plot after classification.")

        # Create figure and axes
        fig, axes = plt.subplots(n_subplots, 1, sharex=True, figsize=(10, max(4, 2 + n_subplots*2)), constrained_layout=True)
        if n_subplots == 1:
            axes = [axes]

        fig.suptitle(os.path.basename(path))

        ax_index = 0
        # Overlay subplot (mix numeric lines and boolean steps with offsets)
        if overlay_numeric or overlay_boolean or overlay_derived:
            ax = axes[ax_index]
            # Numeric overlays
            for col in overlay_numeric:
                ax.plot(t_vals, numeric_data[col], label=col)
            for col in overlay_derived:
                ax.plot(t_vals, derived_data[col], label=col)
            # Boolean overlays with offsets
            offset = 0.0
            for col in overlay_boolean:
                vals = boolean_data[col]
                y = [((v if isinstance(v, (int, float)) and not math.isnan(v) else math.nan) + offset) for v in vals]
                ax.step(t_vals, y, where='post', label=f"{col} (off {offset:.0f})")
                offset += 1.2
            ax.set_ylabel('overlay')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize='small', ncol=2)
            ax_index += 1

        # Remaining numeric signals (each in own subplot)
        for col in remaining_numeric:
            ax = axes[ax_index]
            ax.plot(t_vals, numeric_data[col], label=col)
            ax.set_ylabel(col)
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize='small')
            ax_index += 1

        # Remaining derived signals (each in own subplot)
        for col in remaining_derived:
            ax = axes[ax_index]
            ax.plot(t_vals, derived_data[col], label=col)
            ax.set_ylabel(col)
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize='small')
            ax_index += 1

        # Remaining boolean signals (each in own subplot)
        for col in remaining_boolean:
            ax = axes[ax_index]
            vals = boolean_data[col]
            y = [(v if isinstance(v, (int, float)) and not math.isnan(v) else math.nan) for v in vals]
            ax.step(t_vals, y, where='post', label=col)
            ax.set_ylabel(f"{col} (bool)")
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize='small')
            ax_index += 1

        # Label x-axis on the last subplot
        axes[-1].set_xlabel('time (s)')

        # Save figure as PDF next to the CSV
        pdf_path = os.path.splitext(path)[0] + ".pdf"
        try:
            fig.savefig(pdf_path, format='pdf', bbox_inches='tight')
            self.status.configure(text=f"Saved: {os.path.basename(pdf_path)}")
            try:
                self.after(0, lambda: messagebox.showinfo("Saved", f"Plot saved to:\n{pdf_path}"))
            except Exception:
                pass
        except Exception as e:
            try:
                self.after(0, lambda: messagebox.showerror("Save error", f"Failed to save PDF: {e}"))
            except Exception:
                pass

        plt.show()


def main():
    app = PlotApp()
    app.mainloop()
