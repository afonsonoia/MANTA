"""
MANTA UAV Companion Computer — Explorador de Ficheiros Remoto & Gestor de Vídeos
Interface gráfica para explorar ficheiros no Raspberry Pi, descarregar gravações
de voo em 1 clique, ver espaço em disco e reproduzir transmissões.
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

# Credenciais e configuração padrão
DEFAULT_HOST = "manta.local"
DEFAULT_USER = "pc"
DEFAULT_PASS = "134679"
DEFAULT_REMOTE_DIR = "/home/pc/flight_videos"

class MantaExplorerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MANTA UAV — Explorador de Ficheiros Remoto (Raspberry Pi)")
        self.geometry("980x680")
        self.minsize(800, 500)

        # Estado da ligação SSH/SFTP
        self.ssh = None
        self.sftp = None
        self.current_remote_dir = "/home/pc"
        self.is_connected = False
        self.is_recording_on_pi = False

        # Configuração de orientação FPV (Inversão vertical e horizontal)
        self.fpv_config_file = os.path.join(os.path.dirname(__file__), "fpv_config.json")
        init_vflip, init_hflip = self.load_fpv_config()
        self.var_vflip = tk.BooleanVar(value=init_vflip)
        self.var_hflip = tk.BooleanVar(value=init_hflip)
        self.fpv_process = None

        # Configurar tema e estilos visuais modernos
        self.setup_styles()

        # Construir interface
        self.build_ui()

        # Tecla de atalho F5 para atualizar
        self.bind("<F5>", lambda e: self.navigate_to(self.current_remote_dir))

        # Gestão de encerramento da aplicação
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Iniciar ligação automática em background
        self.after(200, self.connect_ssh)

    def load_fpv_config(self):
        """Carrega as opções guardadas de inversão de imagem FPV."""
        if os.path.isfile(self.fpv_config_file):
            try:
                with open(self.fpv_config_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    return data.get("vflip", True), data.get("hflip", True)
            except Exception:
                pass
        return True, True

    def save_fpv_config(self):
        """Guarda as opções de inversão de imagem FPV em ficheiro JSON."""
        try:
            with open(self.fpv_config_file, "w", encoding="utf-8") as f:
                json.dump({
                    "vflip": bool(self.var_vflip.get()),
                    "hflip": bool(self.var_hflip.get())
                }, f, indent=4)
        except Exception as e:
            print(f"Erro ao guardar {self.fpv_config_file}: {e}")

    def setup_styles(self):
        """Define paleta de cores escura e moderna para visual premium."""
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
        # 1. BARRA SUPERIOR — Estado de Conexão e Atalhos
        top_bar = tk.Frame(self, bg=self.panel_bg, height=54, padx=12, pady=8)
        top_bar.pack(fill=tk.X, side=tk.TOP)

        # Indicador de estado
        self.lbl_status_led = tk.Label(top_bar, text="●", fg="#ffb703", bg=self.panel_bg, font=("Segoe UI", 16))
        self.lbl_status_led.pack(side=tk.LEFT, padx=(0, 6))

        self.lbl_status = tk.Label(top_bar, text="A ligar a manta.local...", fg=self.text_color, bg=self.panel_bg, font=("Segoe UI", 11, "bold"))
        self.lbl_status.pack(side=tk.LEFT)

        # Campo Host/IP configurável
        tk.Label(top_bar, text="IP/Host:", bg=self.panel_bg, fg=self.text_muted, font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(14, 4))
        self.entry_host = tk.Entry(top_bar, bg="#1e2124", fg="#ffffff", insertbackground="#ffffff", relief=tk.FLAT, font=("Consolas", 10), width=16)
        self.entry_host.insert(0, DEFAULT_HOST)
        self.entry_host.pack(side=tk.LEFT, padx=(0, 6))
        self.entry_host.bind("<Return>", lambda e: self.connect_ssh())

        # Botão Reconectar
        self.btn_reconnect = tk.Button(top_bar, text="🔄 Reconectar", bg="#2b2f35", fg="#ffffff", activebackground="#3d424b", activeforeground="#ffffff", relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9), command=self.connect_ssh)
        self.btn_reconnect.pack(side=tk.LEFT, padx=4)

        # Atalhos rápidos
        btn_box = tk.Frame(top_bar, bg=self.panel_bg)
        btn_box.pack(side=tk.RIGHT)

        tk.Button(btn_box, text="🎥 Vídeos de Voo", bg="#0077b6", fg="#ffffff", activebackground=self.accent_hover, relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9, "bold"), command=lambda: self.navigate_to("/home/pc/flight_videos")).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_box, text="📁 Código MANTA", bg="#2b2f35", fg="#ffffff", activebackground="#3d424b", relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9), command=lambda: self.navigate_to("/home/pc/MANTA")).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_box, text="🏠 Home (/home/pc)", bg="#2b2f35", fg="#ffffff", activebackground="#3d424b", relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9), command=lambda: self.navigate_to("/home/pc")).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_box, text="📂 Abrir no Windows", bg="#38b000", fg="#ffffff", activebackground="#70e000", relief=tk.FLAT, padx=10, pady=4, font=("Segoe UI", 9, "bold"), command=self.open_windows_share).pack(side=tk.LEFT, padx=4)

        # 2. BARRA DE NAVEGAÇÃO DE DIRETÓRIO
        nav_bar = tk.Frame(self, bg="#2b2f35", padx=12, pady=6)
        nav_bar.pack(fill=tk.X, side=tk.TOP)

        tk.Button(nav_bar, text="⬆️ Subir Pasta", bg="#1e2124", fg="#ffffff", relief=tk.FLAT, padx=8, pady=2, font=("Segoe UI", 9), command=self.navigate_up).pack(side=tk.LEFT, padx=(0, 8))
        tk.Label(nav_bar, text="Pasta Remota:", bg="#2b2f35", fg=self.accent_color, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))
        
        self.entry_path = tk.Entry(nav_bar, bg="#1e2124", fg="#ffffff", insertbackground="#ffffff", relief=tk.FLAT, font=("Consolas", 10))
        self.entry_path.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))
        self.entry_path.bind("<Return>", lambda e: self.navigate_to(self.entry_path.get().strip()))

        tk.Button(nav_bar, text="🔄 Atualizar (F5)", bg="#4361ee", fg="#ffffff", activebackground="#4895ef", relief=tk.FLAT, padx=10, pady=2, font=("Segoe UI", 9, "bold"), command=lambda: self.navigate_to(self.current_remote_dir)).pack(side=tk.RIGHT, padx=(6, 0))
        tk.Button(nav_bar, text="Ir ➔", bg="#00b4d8", fg="#000000", relief=tk.FLAT, padx=10, pady=2, font=("Segoe UI", 9, "bold"), command=lambda: self.navigate_to(self.entry_path.get().strip())).pack(side=tk.RIGHT)

        # 3. TABELA DE FICHEIROS REMOTOS (TREEVIEW)
        table_frame = tk.Frame(self, bg=self.bg_color, padx=12, pady=8)
        table_frame.pack(fill=tk.BOTH, expand=True)

        columns = ("name", "type", "size", "modified")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", selectmode="extended")
        
        self.tree.heading("name", text=" Nome do Ficheiro / Pasta", anchor=tk.W)
        self.tree.heading("type", text=" Tipo", anchor=tk.CENTER)
        self.tree.heading("size", text=" Tamanho", anchor=tk.E)
        self.tree.heading("modified", text=" Última Modificação", anchor=tk.CENTER)

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

        # Menu de contexto (botão direito)
        self.context_menu = tk.Menu(self, tearoff=0, bg="#22252a", fg="#ffffff", activebackground="#0077b6")
        self.context_menu.add_command(label="⬇️ Descarregar para o PC", command=self.download_selected)
        self.context_menu.add_command(label="▶️ Reproduzir Vídeo (ffplay)", command=self.play_selected_video)
        self.context_menu.add_separator()
        self.context_menu.add_command(label="🗑️ Eliminar Ficheiro", command=self.delete_selected)

        # 4. BARRA DE AÇÕES INFERIOR (DOWNLOADS, STREAM, ESPAÇO NO SD)
        bottom_panel = tk.Frame(self, bg=self.panel_bg, padx=12, pady=10)
        bottom_panel.pack(fill=tk.X, side=tk.BOTTOM)

        actions_box = tk.Frame(bottom_panel, bg=self.panel_bg)
        actions_box.pack(fill=tk.X, side=tk.TOP, pady=(0, 6))

        tk.Button(actions_box, text="⬇️ Descarregar Selecionados", bg="#00b4d8", fg="#000000", activebackground=self.accent_hover, relief=tk.FLAT, padx=14, pady=6, font=("Segoe UI", 10, "bold"), command=self.download_selected).pack(side=tk.LEFT, padx=(0, 8))
        tk.Button(actions_box, text="▶️ Reproduzir Vídeo (ffplay)", bg="#0077b6", fg="#ffffff", activebackground="#0096c7", relief=tk.FLAT, padx=12, pady=6, font=("Segoe UI", 10, "bold"), command=self.play_selected_video).pack(side=tk.LEFT, padx=(0, 8))

        # Bloco FPV Direto com botões de alternância de Inversão Vertical e Horizontal
        fpv_group = tk.Frame(actions_box, bg=self.panel_bg)
        fpv_group.pack(side=tk.LEFT, padx=(0, 8))

        self.btn_fpv = tk.Button(
            fpv_group,
            text="🔴 FPV Direto",
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
            text="↕ Inverter V",
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
            text="↔ Inverter H",
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

        self.btn_record_pi = tk.Button(actions_box, text="⏺️ Gravar no Pi", bg="#7b2cbf", fg="#ffffff", activebackground="#9d4edd", relief=tk.FLAT, padx=12, pady=6, font=("Segoe UI", 10, "bold"), command=self.toggle_record_on_pi)
        self.btn_record_pi.pack(side=tk.LEFT, padx=(0, 8))
        tk.Button(actions_box, text="🗑️ Eliminar", bg="#495057", fg="#ff4d6d", activebackground="#6c757d", relief=tk.FLAT, padx=10, pady=6, font=("Segoe UI", 9), command=self.delete_selected).pack(side=tk.LEFT, padx=(0, 8))

        # Espaço em disco SD
        self.lbl_disk = tk.Label(actions_box, text="Espaço no SD: A carregar...", fg=self.text_muted, bg=self.panel_bg, font=("Segoe UI", 9))
        self.lbl_disk.pack(side=tk.RIGHT, padx=6)

        # Barra de progresso de transferências
        prog_box = tk.Frame(bottom_panel, bg=self.panel_bg)
        prog_box.pack(fill=tk.X, side=tk.BOTTOM)

        self.lbl_prog_info = tk.Label(prog_box, text="Pronto.", fg=self.text_muted, bg=self.panel_bg, font=("Segoe UI", 9))
        self.lbl_prog_info.pack(side=tk.LEFT)

        self.progressbar = ttk.Progressbar(prog_box, style="TProgressbar", mode="determinate", length=260)
        self.progressbar.pack(side=tk.RIGHT, padx=(8, 0))

    # ==========================================================================
    # LÓGICA DE CONEXÃO SSH / SFTP
    # ==========================================================================
    def connect_ssh(self):
        """Estabelece ligação SSH e SFTP com o Raspberry Pi."""
        target_host = self.entry_host.get().strip() if hasattr(self, 'entry_host') and self.entry_host.get().strip() else DEFAULT_HOST
        self.lbl_status.config(text=f"A ligar a {target_host}...", fg="#ffb703")
        self.lbl_status_led.config(fg="#ffb703")

        def _do_connect():
            # Lista de candidatos a testar: IP direto primeiro para evitar bloqueios de mDNS no hotspot
            if target_host in ["manta.local", "10.32.198.35", ""]:
                candidates = ["10.32.198.35", "manta.local"]
            else:
                candidates = [target_host, "10.32.198.35", "manta.local"]

            last_error = "Desconhecido"
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
        self.lbl_status.config(text=f"Conectado ao MANTA UAV ({host})", fg=self.success_color)
        self.lbl_status_led.config(fg=self.success_color)
        self.update_disk_usage()
        self.check_recording_status()
        
        # Se a pasta de vídeos existir, navegar logo para lá
        self.navigate_to(DEFAULT_REMOTE_DIR)

    def _on_connection_error(self, err_msg, host):
        self.is_connected = False
        self.lbl_status.config(text=f"Desconectado ({host})", fg=self.danger_color)
        self.lbl_status_led.config(fg=self.danger_color)
        self.lbl_prog_info.config(text=f"Erro ao ligar: {err_msg}")
        messagebox.showerror("Erro de Ligação", f"Não foi possível ligar ao Raspberry Pi ({host}).\n\nDetalhes:\n{err_msg}\n\nVerifique o IP no hotspot do telemóvel e introduza no campo 'IP/Host' acima.")

    # ==========================================================================
    # NAVEGAÇÃO E LISTAGEM DE FICHEIROS
    # ==========================================================================
    def navigate_to(self, path):
        if not self.is_connected or not self.sftp:
            return

        def _do_list():
            try:
                # Se não existir, tenta o caminho pai ou /home/pc
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
                
                # Separar pastas e ficheiros
                items = []
                for attr in entries:
                    if attr.filename.startswith('.'):
                        continue
                    mtime_str = datetime.fromtimestamp(attr.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
                    is_dir = stat.S_ISDIR(attr.st_mode) if attr.st_mode is not None else False
                    
                    if is_dir:
                        items.append((f"📁 {attr.filename}", "Pasta", "", mtime_str, attr.filename, True, attr.st_mtime))
                    else:
                        size_str = self.format_size(attr.st_size)
                        ext = os.path.splitext(attr.filename)[1].lower()
                        icon = "🎬 " if ext in ('.mkv', '.mp4', '.h264') else "📄 "
                        items.append((f"{icon}{attr.filename}", ext[1:].upper() or "Ficheiro", size_str, mtime_str, attr.filename, False, attr.st_mtime))

                # Ordenar: pastas primeiro, depois por data decrescente (mais recentes no topo)
                items.sort(key=lambda x: (not x[5], -x[6]))

                self.after(0, lambda td=target_dir, it=items: self._update_tree(td, it))
            except Exception as e:
                err_msg = str(e)
                self.after(0, lambda err=err_msg: self.lbl_prog_info.config(text=f"Erro ao ler pasta: {err}"))

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
            self.lbl_prog_info.config(text=f"Pasta vazia: 0 ficheiros em {path}")
        else:
            self.lbl_prog_info.config(text=f"{len(items)} item(ns) na pasta {path}")

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
            # Se for vídeo, reproduzir automaticamente
            if raw_name.lower().endswith(('.mkv', '.mp4', '.h264')):
                self.play_selected_video()

    def show_context_menu(self, event):
        item = self.tree.identify_row(event.y)
        if item:
            self.tree.selection_set(item)
            self.context_menu.post(event.x_root, event.y_root)

    # ==========================================================================
    # DOWNLOAD E REPRODUÇÃO DE VÍDEOS
    # ==========================================================================
    def download_selected(self):
        selected = self.tree.selection()
        if not selected:
            messagebox.showinfo("Aviso", "Por favor selecione um ou mais ficheiros para descarregar.")
            return

        dest_dir = filedialog.askdirectory(title="Selecione a pasta no seu PC para guardar os ficheiros", initialdir=os.getcwd())
        if not dest_dir:
            return

        items_to_download = []
        for item_id in selected:
            tags = self.tree.item(item_id, "tags")
            if tags and tags[1] == "file":
                filename = tags[0]
                items_to_download.append(filename)

        if not items_to_download:
            messagebox.showinfo("Aviso", "Nenhum ficheiro individual selecionado (apenas pastas).")
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

                        # Se for .mkv, verificar se já temos o .mp4 correspondente completo
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

                        chunk_size = 256 * 1024  # 256 KB chunk estável (sem prefetch para evitar estouro de buffers no RPi 3 A+)
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

                                        info_text = f"A descarregar ({idx}/{total_files}): {filename} — {pct}% ({speed_str}, restam {rem_str})"
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
                            info_text = f"Aviso ({attempt}/{max_retries}): Ligação instável ({err_msg}). A reconectar e retomar..."
                            self.after(0, lambda t=info_text: self.lbl_prog_info.config(text=t))
                            time.sleep(2)
                            # Tentar restabelecer SFTP se necessário
                            try:
                                if self.ssh and self.ssh.get_transport() and self.ssh.get_transport().is_active():
                                    self.sftp = self.ssh.open_sftp()
                            except Exception:
                                pass
                        else:
                            self.after(0, lambda err=err_msg: messagebox.showerror("Erro de Transferência", f"Falha ao descarregar {filename}: {err}"))
                            return

                if not success:
                    return

                # Pós-processamento automático: converter .mkv para .mp4 e remover o .mkv local
                if filename.lower().endswith('.mkv') and os.path.exists(local_path):
                    self.after(0, lambda fn=filename: self.lbl_prog_info.config(text=f"A converter para MP4 e a limpar .mkv local: {fn}..."))
                    try:
                        from convert_videos import convert_mkv_to_mp4, is_ffmpeg_available
                        if is_ffmpeg_available():
                            convert_mkv_to_mp4(local_path, delete_original=True, verbose=False)
                    except Exception as e:
                        print(f"Erro na conversão MP4: {e}")

            self.after(0, lambda: self._on_download_complete(dest_dir, len(items_to_download)))

        threading.Thread(target=_do_download, daemon=True).start()

    def _on_download_complete(self, dest_dir, count):
        self.progressbar["value"] = 100
        self.lbl_prog_info.config(text=f"Sucesso! {count} ficheiro(s) transferido(s).")
        if messagebox.askyesno("Download Concluído", f"{count} ficheiro(s) descarregados com sucesso para:\n{dest_dir}\n\nDeseja abrir a pasta agora?"):
            os.startfile(dest_dir)

    def play_selected_video(self):
        """Reproduz o vídeo selecionado diretamente através do ffplay ou leitor local."""
        selected = self.tree.selection()
        if not selected:
            messagebox.showinfo("Aviso", "Selecione um ficheiro de vídeo (.mkv) para reproduzir.")
            return

        tags = self.tree.item(selected[0], "tags")
        if not tags or tags[1] != "file":
            return
        filename = tags[0]
        if not filename.lower().endswith(('.mkv', '.mp4', '.h264')):
            messagebox.showinfo("Formato", "O ficheiro selecionado não aparenta ser um ficheiro de vídeo suportado.")
            return

        remote_path = f"{self.current_remote_dir.rstrip('/')}/{filename}"
        
        # Verifica se temos o ffplay
        ffplay_bin = shutil.which("ffplay")
        if not ffplay_bin:
            messagebox.showwarning("ffplay não encontrado", "O executável ffplay não foi encontrado no PATH.\nPor favor faça o download do ficheiro para reproduzir com o VLC ou leitor do Windows.")
            return

        self.lbl_prog_info.config(text=f"A iniciar reprodução remota de {filename} com ffplay...")

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
                self.after(0, lambda err=err_msg: messagebox.showerror("Erro de Reprodução", f"Falha ao reproduzir vídeo: {err}"))

        threading.Thread(target=_do_stream, daemon=True).start()

    def toggle_fpv_stream(self):
        """Alterna entre iniciar e parar a transmissão FPV em direto."""
        if self.fpv_process and self.fpv_process.poll() is None:
            self.stop_fpv_stream()
        else:
            self.launch_stream_receiver()

    def launch_stream_receiver(self):
        """Inicia a transmissão em direto de baixa latência com a orientação (vflip/hflip) configurada."""
        # Se já existir transmissão em curso, encerra-a antes de relançar
        if self.fpv_process and self.fpv_process.poll() is None:
            self.stop_fpv_stream()
            time.sleep(0.4)

        receiver_script = os.path.join(os.path.dirname(__file__), "stream_receiver.py")
        if not os.path.isfile(receiver_script):
            messagebox.showerror("Erro", f"Script de receção FPV não encontrado:\n{receiver_script}")
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
            self.btn_fpv.config(text="⏹️ Parar FPV", bg="#e63946", activebackground="#ff4d6d")

            orient_parts = []
            if self.var_vflip.get():
                orient_parts.append("V-Flip")
            if self.var_hflip.get():
                orient_parts.append("H-Flip")
            orient_desc = f" [{', '.join(orient_parts)}]" if orient_parts else " [Normal]"

            self.lbl_prog_info.config(text=f"Transmissão FPV em direto iniciada{orient_desc} ({target_host})...")
            self._monitor_fpv_process()
        except Exception as e:
            messagebox.showerror("Erro ao Iniciar FPV", f"Não foi possível iniciar o fluxo de vídeo FPV:\n{e}")

    def stop_fpv_stream(self):
        """Encerra a transmissão FPV e liberta a câmara no Raspberry Pi."""
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
        self.btn_fpv.config(text="🔴 FPV Direto", bg="#d90429", activebackground="#ef233c")
        self.lbl_prog_info.config(text="Transmissão FPV terminada.")

        # Garantir limpeza remota de processos rpicam-vid no Pi se SSH estiver ativo
        if self.is_connected and self.ssh:
            def _clean_remote():
                try:
                    self.ssh.exec_command("pkill -SIGINT -f 'rpicam-vid.*baseline'")
                except Exception:
                    pass
            threading.Thread(target=_clean_remote, daemon=True).start()

    def _monitor_fpv_process(self):
        """Monitoriza em background o fecho do ffplay/stream para restaurar o botão de FPV."""
        def _check():
            while self.fpv_process and self.fpv_process.poll() is None:
                time.sleep(0.5)
            self.fpv_process = None
            self.after(0, lambda: self.btn_fpv.config(text="🔴 FPV Direto", bg="#d90429", activebackground="#ef233c"))
            self.after(0, lambda: self.lbl_prog_info.config(text="Transmissão FPV terminada."))
        threading.Thread(target=_check, daemon=True).start()

    def on_closing(self):
        """Ao fechar a janela do explorador, para o FPV se estiver a correr e fecha."""
        if self.fpv_process and self.fpv_process.poll() is None:
            self.stop_fpv_stream()
        self.destroy()

    def delete_selected(self):
        """Elimina os ficheiros selecionados no Raspberry Pi após confirmação."""
        selected = self.tree.selection()
        if not selected:
            messagebox.showinfo("Aviso", "Selecione um ou mais ficheiros para eliminar.")
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
            filenames_str += f"\n... e mais {len(items_to_delete) - 5} ficheiro(s)"

        if not messagebox.askyesno("Confirmar Eliminação", f"Tem a certeza de que deseja eliminar permanentemente:\n\n{filenames_str}\n\nEsta ação não pode ser desfeita."):
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
                    self.after(0, lambda err=err_msg, fn=filename: messagebox.showerror("Erro ao Eliminar", f"Não foi possível eliminar {fn}:\n{err}"))
                    return

            self.after(0, lambda: self.lbl_prog_info.config(text=f"{len(items_to_delete)} item(ns) eliminado(s)."))
            self.after(0, lambda: self.navigate_to(self.current_remote_dir))
            self.after(0, self.update_disk_usage)

        threading.Thread(target=_do_delete, daemon=True).start()

    def check_recording_status(self):
        """Verifica se há gravação a decorrer no Pi e atualiza o botão."""
        if not self.ssh:
            return
        def _check():
            try:
                stdin, stdout, stderr = self.ssh.exec_command('pgrep -f "rpicam-vid.*record_flight"')
                pids = stdout.read().decode().strip()
                if pids:
                    self.is_recording_on_pi = True
                    self.after(0, lambda: self.btn_record_pi.config(text="⏹️ Parar Gravação (Pi)", bg="#e63946", activebackground="#ff4d6d"))
                else:
                    self.is_recording_on_pi = False
                    self.after(0, lambda: self.btn_record_pi.config(text="⏺️ Gravar no Pi", bg="#7b2cbf", activebackground="#9d4edd"))
            except Exception:
                pass
        threading.Thread(target=_check, daemon=True).start()

    def toggle_record_on_pi(self):
        """Inicia ou para a gravação segura de voo (.mkv) no Raspberry Pi via SSH."""
        if not self.is_connected or not self.ssh:
            messagebox.showwarning("Aviso", "Não está conectado ao Raspberry Pi.")
            return

        if not self.is_recording_on_pi:
            def _start():
                try:
                    stdin, stdout, stderr = self.ssh.exec_command('pgrep -f "rpicam-vid"')
                    pids = stdout.read().decode().strip()
                    if pids:
                        self.after(0, lambda: messagebox.showwarning("Câmara Ocupada", "A câmara do Raspberry Pi já está em uso por outro processo (transmissão ou gravação).\nPare a transmissão FPV antes de iniciar a gravação a bordo."))
                        return
                    self.ssh.exec_command('nohup /home/pc/MANTA/Code/MANTA_PI/record_flight.sh > /home/pc/flight_videos/record.log 2>&1 &')
                    self.is_recording_on_pi = True
                    self.after(0, lambda: self.btn_record_pi.config(text="⏹️ Parar Gravação (Pi)", bg="#e63946", activebackground="#ff4d6d"))
                    self.after(0, lambda: self.lbl_prog_info.config(text="🔴 Gravação de voo iniciada no Raspberry Pi (/home/pc/flight_videos)..."))
                except Exception as e:
                    err_msg = str(e)
                    self.after(0, lambda err=err_msg: messagebox.showerror("Erro ao Iniciar", f"Não foi possível iniciar gravação:\n{err}"))
            threading.Thread(target=_start, daemon=True).start()
        else:
            self._stop_pi_recording()

    def _stop_pi_recording(self):
        self.lbl_prog_info.config(text="A finalizar e descarregar buffers no cartão SD do Pi...")
        def _stop():
            try:
                self.ssh.exec_command('pkill -SIGINT -f rpicam-vid')
                import time
                time.sleep(2)
            except Exception:
                pass
            self.is_recording_on_pi = False
            self.after(0, lambda: self.btn_record_pi.config(text="⏺️ Gravar no Pi", bg="#7b2cbf", activebackground="#9d4edd"))
            self.after(0, lambda: self.lbl_prog_info.config(text="Gravação finalizada e guardada no Raspberry Pi!"))
            self.after(0, lambda: self.navigate_to(DEFAULT_REMOTE_DIR))
            self.after(0, self.update_disk_usage)

        threading.Thread(target=_stop, daemon=True).start()

    def open_windows_share(self):
        """Abre diretamente a pasta partilhada Samba no Explorador do Windows."""
        host = getattr(self, 'active_host', DEFAULT_HOST)
        share_path = f"\\\\{host}\\MANTA"
        try:
            os.startfile(share_path)
            self.lbl_prog_info.config(text=f"A abrir partilha no Windows: {share_path}")
        except Exception as e:
            messagebox.showerror("Erro ao abrir partilha", f"Não foi possível abrir {share_path} no Windows.\n\nDetalhes:\n{e}\n\nPode tentar aceder manualmente pressionando Win+R e digitando: \\\\manta.local\\MANTA")

    def update_disk_usage(self):
        """Obtém a utilização do disco no Raspberry Pi."""
        if not self.ssh:
            return

        def _do_df():
            try:
                stdin, stdout, stderr = self.ssh.exec_command("df -h /home/pc | awk 'NR==2 {print $2, $3, $4, $5}'")
                out = stdout.read().decode().strip().split()
                if len(out) == 4:
                    total, used, free, pct = out
                    text = f"SD: {free} livres ({pct} ocupado de {total})"
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
