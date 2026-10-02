"""
MANTA UAV Companion Computer — Remote File Explorer & Video Manager
GUI to explore Raspberry Pi files, download flight recordings,
monitor disk space, and manage live FPV streams.
"""

import os
import sys
import stat
import threading
import time
import json
import shutil
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from datetime import datetime
import paramiko

DEFAULT_HOST = "manta.local"
DEFAULT_USER = "pc"
DEFAULT_PASS = "134679"
DEFAULT_REMOTE_DIR = "/home/pc/flight_videos"

class MantaExplorerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MANTA UAV — Remote File Explorer (Raspberry Pi)")
        self.geometry("980x680")
        self.minsize(800, 500)

        self.ssh = None
        self.sftp = None
        self.current_remote_dir = "/home/pc"
        self.is_connected = False
        self.is_recording_on_pi = False

        self.fpv_config_file = os.path.join(os.path.dirname(__file__), "fpv_config.json")
        init_vflip, init_hflip = self.load_fpv_config()
        self.var_vflip = tk.BooleanVar(value=init_vflip)
        self.var_hflip = tk.BooleanVar(value=init_hflip)
        self.fpv_process = None

        self.setup_styles()
        self.build_ui()

        self.bind("<F5>", lambda e: self.navigate_to(self.current_remote_dir))
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.after(200, self.connect_ssh)

    def load_fpv_config(self):
        if os.path.isfile(self.fpv_config_file):
            try:
                with open(self.fpv_config_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    return data.get("vflip", True), data.get("hflip", True)
            except Exception:
                pass
        return True, True

    def save_fpv_config(self):
        try:
            with open(self.fpv_config_file, "w", encoding="utf-8") as f:
                json.dump({
                    "vflip": bool(self.var_vflip.get()),
                    "hflip": bool(self.var_hflip.get())
                }, f, indent=4)
        except Exception as e:
            print(f"Error saving {self.fpv_config_file}: {e}")

    def setup_styles(self):
        self.bg_color = "#181a1b"
        self.panel_bg = "#22252a"
        self.accent_color = "#00b4d8"
        self.accent_hover = "#48cae4"
        self.text_color = "#f8f9fa"
        self.text_muted = "#adb5bd"
        self.success_color = "#2ec4b6"
        self.danger_color = "#e63946"

        self.configure(bg=self.bg_color)
        
        style = ttk.Style(self)
        style.theme_use("clam")

        style.configure(".", background=self.bg_color, foreground=self.text_color)
        style.configure("Panel.TFrame", background=self.panel_bg)
        
        style.configure("Treeview", 
                        background="#1e2124", 
                        foreground="#ffffff", 
                        fieldbackground="#1e2124", 
                        rowheight=28,
                        font=("Segoe UI", 10))
        style.configure("Treeview.Heading", 
                        background="#2b2f35", 
                        foreground="#00b4d8", 
                        font=("Segoe UI", 10, "bold"))
        style.map("Treeview", 
                  background=[("selected", "#0077b6")], 
                  foreground=[("selected", "#ffffff")])

        style.configure("TProgressbar", thickness=14, troughcolor="#2b2f35", background="#00b4d8")

    def build_ui(self):
        # 1. Top bar: Connection state & shortcuts
        top_bar = tk.Frame(self, bg=self.panel_bg, height=54, padx=12, pady=8)
        top_bar.pack(fill=tk.X, side=tk.TOP)

        self.lbl_status_led = tk.Label(top_bar, text="●", fg="#ffb703", bg=self.panel_bg, font=("Segoe UI", 16))
        self.lbl_status_led.pack(side=tk.LEFT, padx=(0, 6))

        self.lbl_status = tk.Label(top_bar, text="Connecting to manta.local...", fg=self.text_color, bg=self.panel_bg, font=("Segoe UI", 11, "bold"))
        self.lbl_status.pack(side=tk.LEFT)

        tk.Label(top_bar, text="IP/Host:", bg=self.panel_bg, fg=self.text_muted, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(14, 4))
        self.entry_host = tk.Entry(top_bar, bg="#1e2124", fg="#ffffff", insertbackground="#ffffff", relief=tk.FLAT, font=("Consolas", 10), width=16)
        self.entry_host.insert(0, DEFAULT_HOST)
        self.entry_host.pack(side=tk.LEFT, padx=(0, 6))
        self.entry_host.bind("<Return>", lambda e: self.connect_ssh())

        self.btn_reconnect = tk.Button(top_bar, text="🔄 Reconnect", bg="#2b2f35", fg="#ffffff", activebackground="#3d424b", activeforeground="#ffffff", relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9), command=self.connect_ssh)
        self.btn_reconnect.pack(side=tk.LEFT, padx=4)

        btn_box = tk.Frame(top_bar, bg=self.panel_bg)
        btn_box.pack(side=tk.RIGHT)

        tk.Button(btn_box, text="🎥 Flight Videos", bg="#0077b6", fg="#ffffff", activebackground=self.accent_hover, relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9, "bold"), command=lambda: self.navigate_to("/home/pc/flight_videos")).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_box, text="📁 MANTA Code", bg="#2b2f35", fg="#ffffff", activebackground="#3d424b", relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9), command=lambda: self.navigate_to("/home/pc/MANTA")).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_box, text="🏠 Home (/home/pc)", bg="#2b2f35", fg="#ffffff", activebackground="#3d424b", relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9), command=lambda: self.navigate_to("/home/pc")).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_box, text="📂 Open in Windows", bg="#38b000", fg="#ffffff", activebackground="#70e000", relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9, "bold"), command=self.open_windows_share).pack(side=tk.LEFT, padx=4)

        # 2. Directory Navigation Bar
        nav_bar = tk.Frame(self, bg="#2b2f35", padx=12, pady=6)
        nav_bar.pack(fill=tk.X, side=tk.TOP)

        tk.Button(nav_bar, text="⬆️ Up Folder", bg="#1e2124", fg="#ffffff", relief=tk.FLAT, padx=8, pady=2, font=("Segoe UI", 9), command=self.navigate_up).pack(side=tk.LEFT, padx=(0, 8))
        tk.Label(nav_bar, text="Remote Path:", bg="#2b2f35", fg=self.accent_color, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))
        
        self.entry_path = tk.Entry(nav_bar, bg="#1e2124", fg="#ffffff", insertbackground="#ffffff", relief=tk.FLAT, font=("Consolas", 10))
        self.entry_path.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))
        self.entry_path.bind("<Return>", lambda e: self.navigate_to(self.entry_path.get().strip()))

        tk.Button(nav_bar, text="🔄 Refresh (F5)", bg="#4361ee", fg="#ffffff", activebackground="#4895ef", relief=tk.FLAT, padx=10, pady=2, font=("Segoe UI", 9, "bold"), command=lambda: self.navigate_to(self.current_remote_dir)).pack(side=tk.RIGHT, padx=(6, 0))
        tk.Button(nav_bar, text="Go ➔", bg="#00b4d8", fg="#000000", relief=tk.FLAT, padx=10, pady=2, font=("Segoe UI", 9, "bold"), command=lambda: self.navigate_to(self.entry_path.get().strip())).pack(side=tk.RIGHT)

        # 3. File Table (Treeview)
        table_frame = tk.Frame(self, bg=self.bg_color, padx=12, pady=8)
        table_frame.pack(fill=tk.BOTH, expand=True)

        columns = ("name", "type", "size", "modified")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", selectmode="extended")
        
        self.tree.heading("name", text=" File / Folder Name", anchor=tk.W)
        self.tree.heading("type", text=" Type", anchor=tk.CENTER)
        self.tree.heading("size", text=" Size", anchor=tk.E)
        self.tree.heading("modified", text=" Last Modified", anchor=tk.CENTER)

        self.tree.column("name", width=420, anchor=tk.W)
        self.tree.column("type", width=120, anchor=tk.CENTER)
        self.tree.column("size", width=130, anchor=tk.E)
        self.tree.column("modified", width=190, anchor=tk.CENTER)

        scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree.bind("<Double-1>", self.on_item_double_click)
        self.tree.bind("<Button-3>", self.show_context_menu)

        self.context_menu = tk.Menu(self, tearoff=0, bg="#22252a", fg="#ffffff", activebackground="#0077b6")
        self.context_menu.add_command(label="⬇️ Download to PC", command=self.download_selected)
        self.context_menu.add_command(label="▶️ Play Video (ffplay)", command=self.play_selected_video)
        self.context_menu.add_separator()
        self.context_menu.add_command(label="🗑️ Delete File", command=self.delete_selected)

        # 4. Bottom Action Bar
        bottom_panel = tk.Frame(self, bg=self.panel_bg, padx=12, pady=10)
        bottom_panel.pack(fill=tk.X, side=tk.BOTTOM)

        actions_box = tk.Frame(bottom_panel, bg=self.panel_bg)
        actions_box.pack(fill=tk.X, side=tk.TOP, pady=(0, 6))

        tk.Button(actions_box, text="⬇️ Download Selected", bg="#00b4d8", fg="#000000", activebackground=self.accent_hover, relief=tk.FLAT, padx=14, pady=6, font=("Segoe UI", 10, "bold"), command=self.download_selected).pack(side=tk.LEFT, padx=(0, 8))
        tk.Button(actions_box, text="▶️ Play Video (ffplay)", bg="#0077b6", fg="#ffffff", activebackground="#0096c7", relief=tk.FLAT, padx=12, pady=6, font=("Segoe UI", 10, "bold"), command=self.play_selected_video).pack(side=tk.LEFT, padx=(0, 8))

        fpv_group = tk.Frame(actions_box, bg=self.panel_bg)
        fpv_group.pack(side=tk.LEFT, padx=(0, 8))

        self.btn_fpv = tk.Button(
            fpv_group,
            text="🔴 Live FPV",
            bg="#d90429",
            fg="#ffffff",
            activebackground="#ef233c",
            relief=tk.FLAT,
            padx=12,
            pady=6,
            font=("Segoe UI", 10, "bold"),
            command=self.toggle_fpv_stream
        )
        self.btn_fpv.pack(side=tk.LEFT, padx=(0, 4))

        fpv_toggles = tk.Frame(fpv_group, bg=self.panel_bg)
        fpv_toggles.pack(side=tk.LEFT, padx=(0, 2))

        self.chk_vflip = tk.Checkbutton(
            fpv_toggles,
            text="↕ Flip V",
            variable=self.var_vflip,
            command=self.save_fpv_config,
            bg=self.panel_bg,
            fg="#f8f9fa",
            selectcolor="#181a1b",
            activebackground=self.panel_bg,
            activeforeground="#00b4d8",
            font=("Segoe UI", 8, "bold"),
            padx=2,
            pady=0,
            cursor="hand2"
        )
        self.chk_vflip.pack(anchor=tk.W)

        self.chk_hflip = tk.Checkbutton(
            fpv_toggles,
            text="↔ Flip H",
            variable=self.var_hflip,
            command=self.save_fpv_config,
            bg=self.panel_bg,
            fg="#f8f9fa",
            selectcolor="#181a1b",
            activebackground=self.panel_bg,
            activeforeground="#00b4d8",
            font=("Segoe UI", 8, "bold"),
            padx=2,
            pady=0,
            cursor="hand2"
        )
        self.chk_hflip.pack(anchor=tk.W)

        self.btn_record_pi = tk.Button(actions_box, text="⏺️ Record on Pi", bg="#7b2cbf", fg="#ffffff", activebackground="#9d4edd", relief=tk.FLAT, padx=12, pady=6, font=("Segoe UI", 10, "bold"), command=self.toggle_record_on_pi)
        self.btn_record_pi.pack(side=tk.LEFT, padx=(0, 8))
        tk.Button(actions_box, text="🗑️ Delete", bg="#495057", fg="#ff4d6d", activebackground="#6c757d", relief=tk.FLAT, padx=10, pady=6, font=("Segoe UI", 9), command=self.delete_selected).pack(side=tk.LEFT, padx=(0, 8))

        self.lbl_disk = tk.Label(actions_box, text="SD Space: Loading...", fg=self.text_muted, bg=self.panel_bg, font=("Segoe UI", 9))
        self.lbl_disk.pack(side=tk.RIGHT, padx=6)

        prog_box = tk.Frame(bottom_panel, bg=self.panel_bg)
        prog_box.pack(fill=tk.X, side=tk.BOTTOM)

        self.lbl_prog_info = tk.Label(prog_box, text="Ready.", fg=self.text_muted, bg=self.panel_bg, font=("Segoe UI", 9))
        self.lbl_prog_info.pack(side=tk.LEFT)

        self.progressbar = ttk.Progressbar(prog_box, style="TProgressbar", mode="determinate", length=260)
        self.progressbar.pack(side=tk.RIGHT, padx=(8, 0))

    def connect_ssh(self):
        target_host = self.entry_host.get().strip() if hasattr(self, 'entry_host') and self.entry_host.get().strip() else DEFAULT_HOST
        self.lbl_status.config(text=f"Connecting to {target_host}...", fg="#ffb703")
        self.lbl_status_led.config(fg="#ffb703")

        def _do_connect():
            if target_host in ["manta.local", "10.32.198.35", ""]:
                candidates = ["10.32.198.35", "manta.local"]
            else:
                candidates = [target_host, "10.32.198.35", "manta.local"]

            last_error = "Unknown"
            client = None
            connected_host = None

            for h in candidates:
                try:
                    c = paramiko.SSHClient()
                    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                    c.connect(h, username=DEFAULT_USER, password=DEFAULT_PASS, timeout=4)
                    client = c
                    connected_host = h
                    break
                except Exception as e:
                    last_error = str(e)

            if client and connected_host:
                try:
                    transport = client.get_transport()
                    if transport:
                        transport.default_window_size = 4 * 1024 * 1024
                        transport.default_max_packet_size = 64 * 1024
                        transport.set_keepalive(5)

                    sftp = client.open_sftp()
                    self.ssh = client
                    self.sftp = sftp
                    self.is_connected = True
                    self.active_host = connected_host

                    self.after(0, lambda h=connected_host: self._on_connected(h))
                except Exception as e:
                    err_msg = str(e)
                    self.after(0, lambda err=err_msg, h=target_host: self._on_connection_error(err, h))
            else:
                self.after(0, lambda err=last_error, h=target_host: self._on_connection_error(err, h))

        threading.Thread(target=_do_connect, daemon=True).start()

    def _on_connected(self, host):
        self.entry_host.delete(0, tk.END)
        self.entry_host.insert(0, host)
        self.lbl_status.config(text=f"Connected to MANTA UAV ({host})", fg=self.success_color)
        self.lbl_status_led.config(fg=self.success_color)
        self.update_disk_usage()
        self.check_recording_status()
        self.navigate_to(DEFAULT_REMOTE_DIR)

    def _on_connection_error(self, err_msg, host):
        self.is_connected = False
        self.lbl_status.config(text=f"Disconnected ({host})", fg=self.danger_color)
        self.lbl_status_led.config(fg=self.danger_color)
        self.lbl_prog_info.config(text=f"Connection error: {err_msg}")
        messagebox.showerror("Connection Error", f"Could not connect to Raspberry Pi ({host}).\n\nDetails:\n{err_msg}\n\nCheck the IP address on your hotspot and enter it in 'IP/Host' above.")

    def navigate_to(self, path):
        if not self.is_connected or not self.sftp:
            return

        def _do_list():
            try:
                try:
                    self.sftp.stat(path)
                    target_dir = path
                except IOError:
                    if path == DEFAULT_REMOTE_DIR:
                        try:
                            self.sftp.mkdir(DEFAULT_REMOTE_DIR)
                            target_dir = path
                        except Exception:
                            target_dir = "/home/pc"
                    else:
                        target_dir = "/home/pc"

                entries = self.sftp.listdir_attr(target_dir)
                
                items = []
                for attr in entries:
                    if attr.filename.startswith('.'):
                        continue
                    mtime_str = datetime.fromtimestamp(attr.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
                    is_dir = stat.S_ISDIR(attr.st_mode) if attr.st_mode is not None else False
                    
                    if is_dir:
                        items.append((f"📁 {attr.filename}", "Folder", "", mtime_str, attr.filename, True, attr.st_mtime))
                    else:
                        size_str = self.format_size(attr.st_size)
                        ext = os.path.splitext(attr.filename)[1].lower()
                        icon = "🎬 " if ext in ('.mkv', '.mp4', '.h264') else "📄 "
                        items.append((f"{icon}{attr.filename}", ext[1:].upper() or "File", size_str, mtime_str, attr.filename, False, attr.st_mtime))

                items.sort(key=lambda x: (not x[5], -x[6]))
                self.after(0, lambda td=target_dir, it=items: self._update_tree(td, it))
            except Exception as e:
                err_msg = str(e)
                self.after(0, lambda err=err_msg: self.lbl_prog_info.config(text=f"Error reading folder: {err}"))

        threading.Thread(target=_do_list, daemon=True).start()

    def _update_tree(self, path, items):
        self.current_remote_dir = path
        self.entry_path.delete(0, tk.END)
        self.entry_path.insert(0, path)

        for row in self.tree.get_children():
            self.tree.delete(row)

        for item in items:
            display_name, file_type, size, mtime, raw_name, is_dir = item[0], item[1], item[2], item[3], item[4], item[5]
            self.tree.insert("", tk.END, values=(display_name, file_type, size, mtime), tags=(raw_name, "dir" if is_dir else "file"))

        if not items:
            self.lbl_prog_info.config(text=f"Empty folder: 0 files in {path}")
        else:
            self.lbl_prog_info.config(text=f"{len(items)} item(s) in folder {path}")

    def navigate_up(self):
        parent = os.path.dirname(self.current_remote_dir.rstrip("/"))
        if parent and parent != self.current_remote_dir:
            self.navigate_to(parent)

    def on_item_double_click(self, event):
        item_id = self.tree.focus()
        if not item_id:
            return
        tags = self.tree.item(item_id, "tags")
        if not tags:
            return
        raw_name, item_type = tags[0], tags[1]
        
        if item_type == "dir":
            new_path = f"{self.current_remote_dir.rstrip('/')}/{raw_name}"
            self.navigate_to(new_path)
        else:
            if raw_name.lower().endswith(('.mkv', '.mp4', '.h264')):
                self.play_selected_video()

    def show_context_menu(self, event):
        item = self.tree.identify_row(event.y)
        if item:
            self.tree.selection_set(item)
            self.context_menu.post(event.x_root, event.y_root)

    def download_selected(self):
        selected = self.tree.selection()
        if not selected:
            messagebox.showinfo("Notice", "Please select one or more files to download.")
            return

        dest_dir = filedialog.askdirectory(title="Select destination folder on PC", initialdir=os.getcwd())
        if not dest_dir:
            return

        items_to_download = []
        for item_id in selected:
            tags = self.tree.item(item_id, "tags")
            if tags and tags[1] == "file":
                filename = tags[0]
                items_to_download.append(filename)

        if not items_to_download:
            messagebox.showinfo("Notice", "No files selected (only folders).")
            return

        def _do_download():
            self.progressbar["value"] = 0
            total_files = len(items_to_download)

            for idx, filename in enumerate(items_to_download, 1):
                remote_path = f"{self.current_remote_dir.rstrip('/')}/{filename}"
                local_path = os.path.join(dest_dir, filename)

                max_retries = 5
                success = False

                for attempt in range(1, max_retries + 1):
                    try:
                        stat_info = self.sftp.stat(remote_path)
                        total_bytes = stat_info.st_size

                        if filename.lower().endswith('.mkv'):
                            base_name, _ = os.path.splitext(filename)
                            mp4_local_path = os.path.join(dest_dir, f"{base_name}.mp4")
                            if os.path.exists(mp4_local_path) and os.path.getsize(mp4_local_path) > 1024:
                                success = True
                                break

                        existing_bytes = 0
                        if os.path.exists(local_path):
                            existing_bytes = os.path.getsize(local_path)
                            if existing_bytes == total_bytes and total_bytes > 0:
                                success = True
                                break
                            elif existing_bytes > total_bytes:
                                os.remove(local_path)
                                existing_bytes = 0

                        chunk_size = 256 * 1024
                        mode = 'ab' if existing_bytes > 0 else 'wb'

                        with self.sftp.open(remote_path, 'rb') as remote_file:
                            if existing_bytes > 0:
                                remote_file.seek(existing_bytes)

                            with open(local_path, mode) as local_file:
                                transferred = existing_bytes
                                start_time = time.time()
                                last_ui_update = 0
                                bytes_session = 0

                                while transferred < total_bytes:
                                    to_read = min(chunk_size, total_bytes - transferred)
                                    chunk = remote_file.read(to_read)
                                    if not chunk:
                                        break
                                    local_file.write(chunk)
                                    transferred += len(chunk)
                                    bytes_session += len(chunk)

                                    now = time.time()
                                    if (now - last_ui_update >= 0.2) or (transferred >= total_bytes):
                                        elapsed = now - start_time
                                        speed = (bytes_session / (1024 * 1024)) / elapsed if elapsed > 0 else 0
                                        pct = int((transferred / total_bytes) * 100) if total_bytes > 0 else 0
                                        rem_sec = int((total_bytes - transferred) / (bytes_session / elapsed)) if elapsed > 0 and bytes_session > 0 else 0
                                        speed_str = f"{speed:.1f} MB/s"
                                        rem_str = f"{rem_sec}s" if rem_sec < 60 else f"{rem_sec//60}m{rem_sec%60:02d}s"

                                        info_text = f"Downloading ({idx}/{total_files}): {filename} — {pct}% ({speed_str}, {rem_str} left)"
                                        self.after(0, lambda p=pct, t=info_text: (
                                             self.progressbar.configure(value=p),
                                             self.lbl_prog_info.config(text=t)
                                        ))
                                        last_ui_update = now

                                if transferred >= total_bytes:
                                    success = True
                                    break

                    except Exception as e:
                        err_msg = str(e)
                        if attempt < max_retries:
                            info_text = f"Warning ({attempt}/{max_retries}): Connection unstable ({err_msg}). Reconnecting..."
                            self.after(0, lambda t=info_text: self.lbl_prog_info.config(text=t))
                            time.sleep(2)
                            try:
                                if self.ssh and self.ssh.get_transport() and self.ssh.get_transport().is_active():
                                    self.sftp = self.ssh.open_sftp()
                            except Exception:
                                pass
                        else:
                            self.after(0, lambda err=err_msg: messagebox.showerror("Transfer Error", f"Failed downloading {filename}: {err}"))
                            return

                if not success:
                    return

                if filename.lower().endswith('.mkv') and os.path.exists(local_path):
                    self.after(0, lambda fn=filename: self.lbl_prog_info.config(text=f"Converting to MP4 and cleaning local .mkv: {fn}..."))
                    try:
                        from convert_videos import convert_mkv_to_mp4, is_ffmpeg_available
                        if is_ffmpeg_available():
                            convert_mkv_to_mp4(local_path, delete_original=True, verbose=False)
                    except Exception as e:
                        print(f"MP4 conversion error: {e}")

            self.after(0, lambda: self._on_download_complete(dest_dir, len(items_to_download)))

        threading.Thread(target=_do_download, daemon=True).start()

    def _on_download_complete(self, dest_dir, count):
        self.progressbar["value"] = 100
        self.lbl_prog_info.config(text=f"Success! {count} file(s) downloaded.")
        if messagebox.askyesno("Download Complete", f"{count} file(s) downloaded successfully to:\n{dest_dir}\n\nOpen destination folder now?"):
            os.startfile(dest_dir)

    def play_selected_video(self):
        selected = self.tree.selection()
        if not selected:
            messagebox.showinfo("Notice", "Select a video file (.mkv) to play.")
            return

        tags = self.tree.item(selected[0], "tags")
        if not tags or tags[1] != "file":
            return
        filename = tags[0]
        if not filename.lower().endswith(('.mkv', '.mp4', '.h264')):
            messagebox.showinfo("Format", "Selected file is not a supported video format.")
            return

        remote_path = f"{self.current_remote_dir.rstrip('/')}/{filename}"
        
        ffplay_bin = shutil.which("ffplay")
        if not ffplay_bin:
            messagebox.showwarning("ffplay not found", "ffplay executable was not found in PATH.\nPlease download the file to play with VLC or Windows Media Player.")
            return

        self.lbl_prog_info.config(text=f"Launching remote playback of {filename} with ffplay...")

        def _do_stream():
            try:
                proc = subprocess.Popen(
                    [ffplay_bin, "-probesize", "64", "-fflags", "nobuffer", "-window_title", f"MANTA Flight Video: {filename}", "-i", "-"],
                    stdin=subprocess.PIPE
                )
                with self.sftp.open(remote_path, 'rb') as remote_file:
                    while True:
                        chunk = remote_file.read(65536)
                        if not chunk:
                            break
                        try:
                            proc.stdin.write(chunk)
                            proc.stdin.flush()
                        except (BrokenPipeError, OSError):
                            break
                try:
                    proc.stdin.close()
                except Exception:
                    pass
            except Exception as e:
                err_msg = str(e)
                self.after(0, lambda err=err_msg: messagebox.showerror("Playback Error", f"Failed playing video: {err}"))

        threading.Thread(target=_do_stream, daemon=True).start()

    def toggle_fpv_stream(self):
        if self.fpv_process and self.fpv_process.poll() is None:
            self.stop_fpv_stream()
        else:
            self.launch_stream_receiver()

    def launch_stream_receiver(self):
        if self.fpv_process and self.fpv_process.poll() is None:
            self.stop_fpv_stream()
            time.sleep(0.4)

        receiver_script = os.path.join(os.path.dirname(__file__), "stream_receiver.py")
        if not os.path.isfile(receiver_script):
            messagebox.showerror("Error", f"FPV stream receiver script not found:\n{receiver_script}")
            return

        target_host = getattr(self, 'active_host', None)
        if not target_host:
            target_host = self.entry_host.get().strip() if hasattr(self, 'entry_host') and self.entry_host.get().strip() else DEFAULT_HOST

        self.save_fpv_config()

        cmd = [
            sys.executable,
            receiver_script,
            "--host", target_host,
            "--vflip" if self.var_vflip.get() else "--no-vflip",
            "--hflip" if self.var_hflip.get() else "--no-hflip"
        ]

        try:
            self.fpv_process = subprocess.Popen(cmd)
            self.btn_fpv.config(text="⏹️ Stop FPV", bg="#e63946", activebackground="#ff4d6d")

            orient_parts = []
            if self.var_vflip.get():
                orient_parts.append("V-Flip")
            if self.var_hflip.get():
                orient_parts.append("H-Flip")
            orient_desc = f" [{', '.join(orient_parts)}]" if orient_parts else " [Normal]"

            self.lbl_prog_info.config(text=f"Live FPV stream launched{orient_desc} ({target_host})...")
            self._monitor_fpv_process()
        except Exception as e:
            messagebox.showerror("FPV Launch Error", f"Could not launch FPV stream:\n{e}")

    def stop_fpv_stream(self):
        if self.fpv_process and self.fpv_process.poll() is None:
            try:
                self.fpv_process.terminate()
                self.fpv_process.wait(timeout=2)
            except Exception:
                try:
                    self.fpv_process.kill()
                except Exception:
                    pass
        self.fpv_process = None
        self.btn_fpv.config(text="🔴 Live FPV", bg="#d90429", activebackground="#ef233c")
        self.lbl_prog_info.config(text="FPV stream stopped.")

        if self.is_connected and self.ssh:
            def _clean_remote():
                try:
                    self.ssh.exec_command("pkill -SIGINT -f 'rpicam-vid.*baseline'")
                except Exception:
                    pass
            threading.Thread(target=_clean_remote, daemon=True).start()

    def _monitor_fpv_process(self):
        def _check():
            while self.fpv_process and self.fpv_process.poll() is None:
                time.sleep(0.5)
            self.fpv_process = None
            self.after(0, lambda: self.btn_fpv.config(text="🔴 Live FPV", bg="#d90429", activebackground="#ef233c"))
            self.after(0, lambda: self.lbl_prog_info.config(text="FPV stream stopped."))
        threading.Thread(target=_check, daemon=True).start()

    def on_closing(self):
        if self.fpv_process and self.fpv_process.poll() is None:
            self.stop_fpv_stream()
        self.destroy()

    def delete_selected(self):
        selected = self.tree.selection()
        if not selected:
            messagebox.showinfo("Notice", "Select one or more files to delete.")
            return

        items_to_delete = []
        for item_id in selected:
            tags = self.tree.item(item_id, "tags")
            if tags:
                filename = tags[0]
                is_dir = (tags[1] == "dir")
                items_to_delete.append((filename, is_dir))

        if not items_to_delete:
            return

        filenames_str = "\n".join(f"- {name}" for name, _ in items_to_delete[:5])
        if len(items_to_delete) > 5:
            filenames_str += f"\n... and {len(items_to_delete) - 5} more file(s)"

        if not messagebox.askyesno("Confirm Delete", f"Are you sure you want to permanently delete:\n\n{filenames_str}\n\nThis cannot be undone."):
            return

        def _do_delete():
            for filename, is_dir in items_to_delete:
                remote_path = f"{self.current_remote_dir.rstrip('/')}/{filename}"
                try:
                    if is_dir:
                        self.sftp.rmdir(remote_path)
                    else:
                        self.sftp.remove(remote_path)
                except Exception as e:
                    err_msg = str(e)
                    self.after(0, lambda err=err_msg, fn=filename: messagebox.showerror("Delete Error", f"Could not delete {fn}:\n{err}"))
                    return

            self.after(0, lambda: self.lbl_prog_info.config(text=f"{len(items_to_delete)} item(s) deleted."))
            self.after(0, lambda: self.navigate_to(self.current_remote_dir))
            self.after(0, self.update_disk_usage)

        threading.Thread(target=_do_delete, daemon=True).start()

    def check_recording_status(self):
        if not self.ssh:
            return
        def _check():
            try:
                stdin, stdout, stderr = self.ssh.exec_command('pgrep -f "rpicam-vid.*record_flight"')
                pids = stdout.read().decode().strip()
                if pids:
                    self.is_recording_on_pi = True
                    self.after(0, lambda: self.btn_record_pi.config(text="⏹️ Stop Recording (Pi)", bg="#e63946", activebackground="#ff4d6d"))
                else:
                    self.is_recording_on_pi = False
                    self.after(0, lambda: self.btn_record_pi.config(text="⏺️ Record on Pi", bg="#7b2cbf", activebackground="#9d4edd"))
            except Exception:
                pass
        threading.Thread(target=_check, daemon=True).start()

    def toggle_record_on_pi(self):
        if not self.is_connected or not self.ssh:
            messagebox.showwarning("Notice", "Not connected to Raspberry Pi.")
            return

        if not self.is_recording_on_pi:
            def _start():
                try:
                    stdin, stdout, stderr = self.ssh.exec_command('pgrep -f "rpicam-vid"')
                    pids = stdout.read().decode().strip()
                    if pids:
                        self.after(0, lambda: messagebox.showwarning("Camera Busy", "Raspberry Pi camera is already in use by another process (stream or recording).\nStop FPV stream before recording onboard."))
                        return
                    self.ssh.exec_command('nohup /home/pc/MANTA/Code/MANTA_PI/record_flight.sh > /home/pc/flight_videos/record.log 2>&1 &')
                    self.is_recording_on_pi = True
                    self.after(0, lambda: self.btn_record_pi.config(text="⏹️ Stop Recording (Pi)", bg="#e63946", activebackground="#ff4d6d"))
                    self.after(0, lambda: self.lbl_prog_info.config(text="🔴 Flight recording started on Raspberry Pi (/home/pc/flight_videos)..."))
                except Exception as e:
                    err_msg = str(e)
                    self.after(0, lambda err=err_msg: messagebox.showerror("Recording Error", f"Could not start recording:\n{err}"))
            threading.Thread(target=_start, daemon=True).start()
        else:
            self._stop_pi_recording()

    def _stop_pi_recording(self):
        self.lbl_prog_info.config(text="Flushing buffers to SD card on Pi...")
        def _stop():
            try:
                self.ssh.exec_command('pkill -SIGINT -f rpicam-vid')
                import time
                time.sleep(2)
            except Exception:
                pass
            self.is_recording_on_pi = False
            self.after(0, lambda: self.btn_record_pi.config(text="⏺️ Record on Pi", bg="#7b2cbf", activebackground="#9d4edd"))
            self.after(0, lambda: self.lbl_prog_info.config(text="Recording finished and saved on Raspberry Pi!"))
            self.after(0, lambda: self.navigate_to(DEFAULT_REMOTE_DIR))
            self.after(0, self.update_disk_usage)

        threading.Thread(target=_stop, daemon=True).start()

    def open_windows_share(self):
        host = getattr(self, 'active_host', DEFAULT_HOST)
        share_path = f"\\\\{host}\\MANTA"
        try:
            os.startfile(share_path)
            self.lbl_prog_info.config(text=f"Opening Windows share: {share_path}")
        except Exception as e:
            messagebox.showerror("Share Error", f"Could not open {share_path} in Windows.\n\nDetails:\n{e}\n\nYou can manually connect by pressing Win+R and typing: \\\\manta.local\\MANTA")

    def update_disk_usage(self):
        if not self.ssh:
            return

        def _do_df():
            try:
                stdin, stdout, stderr = self.ssh.exec_command("df -h /home/pc | awk 'NR==2 {print $2, $3, $4, $5}'")
                out = stdout.read().decode().strip().split()
                if len(out) == 4:
                    total, used, free, pct = out
                    text = f"SD: {free} free ({pct} used of {total})"
                    self.after(0, lambda: self.lbl_disk.config(text=text))
            except Exception:
                pass

        threading.Thread(target=_do_df, daemon=True).start()

    @staticmethod
    def format_size(size_bytes):
        if size_bytes == 0:
            return "0 B"
        units = ["B", "KB", "MB", "GB", "TB"]
        idx = 0
        s = float(size_bytes)
        while s >= 1024 and idx < len(units) - 1:
            s /= 1024.0
            idx += 1
        return f"{s:.1f} {units[idx]}"

def main():
    app = MantaExplorerApp()
    app.mainloop()

if __name__ == "__main__":
    main()
