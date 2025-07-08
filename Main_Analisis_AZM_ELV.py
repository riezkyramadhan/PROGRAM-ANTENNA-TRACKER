import tkinter as tk
from tkinter import filedialog, ttk, messagebox
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

class ResponAnalyzerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Analisis Respon Sistem Kendali Antena")
        # --- PERUBAHAN --- Jendela diperlebar untuk mengakomodasi kolom baru
        self.root.geometry("1400x700")

        # Style
        style = ttk.Style(self.root)
        style.theme_use("clam")

        # Main Frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Top Frame untuk kontrol file
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.X, pady=5)

        self.btn_load = ttk.Button(top_frame, text="Buka File Log (.txt / .csv)", command=self.load_file)
        self.btn_load.pack(side=tk.LEFT, padx=5)
        
        self.lbl_file = ttk.Label(top_frame, text="Belum ada file yang dimuat", anchor="w")
        self.lbl_file.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Frame untuk tombol kontrol grafik
        graph_controls_frame = ttk.Frame(main_frame)
        graph_controls_frame.pack(fill=tk.X, pady=(5, 10))
        
        self.btn_pos_graph = ttk.Button(graph_controls_frame, text="Tampilkan Grafik Posisi", command=self.create_position_graph, state=tk.DISABLED)
        self.btn_pos_graph.pack(side=tk.LEFT, padx=5)
        
        self.btn_err_graph = ttk.Button(graph_controls_frame, text="Tampilkan Grafik Kontrol & Error", command=self.create_control_error_graph, state=tk.DISABLED)
        self.btn_err_graph.pack(side=tk.LEFT, padx=5)

        # Results Frame
        results_frame = ttk.Frame(main_frame, padding="5")
        results_frame.pack(fill=tk.BOTH, expand=True)

        # --- PERUBAHAN --- Menambahkan kolom baru ke Treeview
        columns = ('Axis', 'Phase', 'RiseTime', 'SettlingTime', 'PeakTime', 'Overshoot', 'SSE', 'IAE', 'ISE', 'ITAE', 'ITSE')
        self.tree = ttk.Treeview(results_frame, columns=columns, show='headings')
        
        self.tree.heading('Axis', text='Sumbu')
        self.tree.heading('Phase', text='Fase')
        self.tree.heading('RiseTime', text='Rise Time (ms)')
        self.tree.heading('SettlingTime', text='Settling Time (ms)')
        self.tree.heading('PeakTime', text='Peak Time (ms)')
        self.tree.heading('Overshoot', text='Overshoot (%)')
        self.tree.heading('SSE', text='SSE (°)')
        self.tree.heading('IAE', text='IAE')
        self.tree.heading('ISE', text='ISE')
        self.tree.heading('ITAE', text='ITAE')
        self.tree.heading('ITSE', text='ITSE')
        
        self.tree.column('Axis', width=50, anchor='center')
        self.tree.column('Phase', width=140)
        self.tree.column('RiseTime', width=120, anchor='center')
        self.tree.column('SettlingTime', width=120, anchor='center')
        self.tree.column('PeakTime', width=120, anchor='center')
        self.tree.column('Overshoot', width=100, anchor='center')
        self.tree.column('SSE', width=100, anchor='center')
        self.tree.column('IAE', width=100, anchor='center')
        self.tree.column('ISE', width=100, anchor='center')
        self.tree.column('ITAE', width=100, anchor='center')
        self.tree.column('ITSE', width=100, anchor='center')
        
        tree_scrollbar = ttk.Scrollbar(results_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=tree_scrollbar.set)
        
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.df = None

    def load_file(self):
        file_path = filedialog.askopenfilename(
            title="Pilih file log",
            filetypes=(("Text files", "*.txt"), ("CSV files", "*.csv"), ("All files", "*.*"))
        )
        if not file_path:
            return

        try:
            # --- BARU --- Memastikan semua kolom numerik penting ada
            required_cols = ['Timestamp(ms)', 'Test_Phase', 'Target_Azm', 'Actual_Azm', 'Error_Azm', 'Target_Elv', 'Actual_Elv', 'Error_Elv']
            self.df = pd.read_csv(file_path)
            for col in required_cols:
                if col not in self.df.columns:
                    raise ValueError(f"Kolom yang dibutuhkan '{col}' tidak ditemukan di file.")
            
            self.lbl_file.config(text=file_path.split('/')[-1])
            self.analyze_and_display()
        except Exception as e:
            messagebox.showerror("Error Membaca File", f"Gagal memuat atau memproses file.\nPastikan format file dan nama kolom sudah benar.\n\nError: {e}")
            self.btn_pos_graph.config(state=tk.DISABLED)
            self.btn_err_graph.config(state=tk.DISABLED)

    def analyze_and_display(self):
        if self.df is None:
            return
            
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        results = self.analyze_step_responses()
        for res in results:
            # --- PERUBAHAN --- Menambahkan nilai dari metrik baru ke tabel
            self.tree.insert('', tk.END, values=(
                res['axis'], 
                res['phase_name'],
                f"{res['rise_time']:.2f}" if res['rise_time'] is not None else "N/A",
                f"{res['settling_time']:.2f}" if res['settling_time'] is not None else "N/A",
                f"{res['peak_time']:.2f}" if res['peak_time'] is not None else "N/A",
                f"{res['overshoot']:.2f}" if res['overshoot'] is not None else "N/A",
                f"{res['steady_state_error']:.3f}" if res['steady_state_error'] is not None else "N/A",
                f"{res['iae']:.3f}" if res['iae'] is not None else "N/A",
                f"{res['ise']:.3f}" if res['ise'] is not None else "N/A",
                f"{res['itae']:.3f}" if res['itae'] is not None else "N/A",
                f"{res['itse']:.3f}" if res['itse'] is not None else "N/A",
            ))

        self.btn_pos_graph.config(state=tk.NORMAL)
        self.btn_err_graph.config(state=tk.NORMAL)

    def analyze_step_responses(self):
        results = []
        move_phases = self.df['Test_Phase'][self.df['Test_Phase'].str.contains("MOVE", na=False)].unique()
        last_phase_df = None
        for phase in move_phases:
            phase_df = self.df[self.df['Test_Phase'] == phase].copy().reset_index(drop=True)
            if phase_df.empty or len(phase_df) < 2:
                continue
            az_results = self._calculate_metrics(phase_df, 'Azm', last_phase_df)
            az_results['phase_name'] = phase
            results.append(az_results)
            el_results = self._calculate_metrics(phase_df, 'Elv', last_phase_df)
            el_results['phase_name'] = phase
            results.append(el_results)
            last_phase_df = phase_df
        return results

    # --- PERUBAHAN BESAR --- Fungsi ini sekarang menghitung semua metrik
    def _calculate_metrics(self, phase_df, axis_prefix, last_phase_df=None):
        target_col = f'Target_{axis_prefix}'
        actual_col = f'Actual_{axis_prefix}'
        error_col = f'Error_{axis_prefix}'
        time_col = 'Timestamp(ms)'

        # Inisialisasi semua metrik ke None
        rise_time, settling_time, peak_time, overshoot, sse = None, None, None, None, None
        iae, ise, itae, itse = None, None, None, None

        try:
            # Nilai dasar
            target_val = phase_df[target_col].iloc[0]
            initial_val = last_phase_df[actual_col].iloc[-1] if last_phase_df is not None and not last_phase_df.empty else phase_df[actual_col].iloc[0]
            step_amplitude = target_val - initial_val

            # Waktu
            start_time = phase_df[time_col].iloc[0]
            relative_time_ms = phase_df[time_col] - start_time
            relative_time_s = relative_time_ms / 1000.0
            dt_s = relative_time_s.diff().fillna(0) # delta t dalam detik

            # Error
            error = phase_df[error_col]

            # --- Steady-State Error (SSE) ---
            steady_state_slice = phase_df.iloc[int(len(phase_df) * 0.8):]
            steady_state_val = steady_state_slice[actual_col].mean()
            sse = target_val - steady_state_val

            # Hanya hitung metrik dinamis jika ada pergerakan signifikan
            if abs(step_amplitude) > 1e-3:
                # --- Rise Time (10% to 90%) ---
                if step_amplitude > 0:
                    t10_df = phase_df[phase_df[actual_col] >= initial_val + 0.1 * step_amplitude]
                    t90_df = phase_df[phase_df[actual_col] >= initial_val + 0.9 * step_amplitude]
                else:
                    t10_df = phase_df[phase_df[actual_col] <= initial_val + 0.1 * step_amplitude]
                    t90_df = phase_df[phase_df[actual_col] <= initial_val + 0.9 * step_amplitude]
                
                t10 = t10_df[time_col].iloc[0] if not t10_df.empty else start_time
                t90 = t90_df[time_col].iloc[0] if not t90_df.empty else phase_df[time_col].iloc[-1]
                rise_time = t90 - t10

                # --- Overshoot & Peak Time ---
                if step_amplitude > 0:
                    peak_idx = phase_df[actual_col].idxmax()
                    peak_val = phase_df.loc[peak_idx, actual_col]
                    overshoot = ((peak_val - steady_state_val) / abs(step_amplitude)) * 100 if peak_val > steady_state_val else 0.0
                else: # Step Down
                    peak_idx = phase_df[actual_col].idxmin()
                    peak_val = phase_df.loc[peak_idx, actual_col]
                    overshoot = ((steady_state_val - peak_val) / abs(step_amplitude)) * 100 if peak_val < steady_state_val else 0.0
                
                peak_time = phase_df.loc[peak_idx, time_col] - start_time
                
                # --- Settling Time (within 2% of steady-state value) ---
                settling_band = 0.02 * abs(step_amplitude)
                outside_band_df = phase_df[abs(phase_df[actual_col] - steady_state_val) > settling_band]
                
                if outside_band_df.empty:
                    settling_time = 0.0 # Langsung berada di dalam band
                else:
                    last_outside_time = outside_band_df[time_col].iloc[-1]
                    # Cek apakah sistem benar-benar settle di akhir
                    if phase_df.iloc[-1][actual_col] > steady_state_val + settling_band or \
                       phase_df.iloc[-1][actual_col] < steady_state_val - settling_band:
                        settling_time = None # Tidak settle
                    else:
                        settling_time = last_outside_time - start_time

            # --- Integral Performance Criteria ---
            iae = np.sum(np.abs(error) * dt_s)
            ise = np.sum(error**2 * dt_s)
            itae = np.sum(relative_time_s * np.abs(error) * dt_s)
            itse = np.sum(relative_time_s * (error**2) * dt_s)

        except (IndexError, KeyError) as e:
            print(f"Peringatan: Gagal menghitung metrik untuk {axis_prefix} di fase ini. Error: {e}")
            # Biarkan nilai default (None)
        
        return {
            'axis': axis_prefix, 'rise_time': rise_time, 'settling_time': settling_time,
            'peak_time': peak_time, 'overshoot': overshoot, 'steady_state_error': sse,
            'iae': iae, 'ise': ise, 'itae': itae, 'itse': itse
        }

    def _show_graph_in_new_window(self, fig, title):
        graph_window = tk.Toplevel(self.root)
        graph_window.title(title)
        graph_window.geometry("1000x800")
        frame = ttk.Frame(graph_window)
        frame.pack(fill=tk.BOTH, expand=True)
        canvas = FigureCanvasTkAgg(fig, master=frame)
        canvas.draw()
        toolbar = NavigationToolbar2Tk(canvas, frame)
        toolbar.update()
        toolbar.pack(side=tk.BOTTOM, fill=tk.X)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def create_position_graph(self):
        f = plt.Figure(figsize=(10, 8), dpi=100)
        ax1 = f.add_subplot(211)
        ax2 = f.add_subplot(212)
        time_sec = self.df['Timestamp(ms)'] / 1000.0
        ax1.plot(time_sec, self.df['Target_Azm'], 'r--', label='Referensi', linewidth=2)
        ax1.plot(time_sec, self.df['Actual_Azm'], 'b-', label='Output Sistem')
        ax1.set_title('Grafik Respon Sistem Motor Sumbu Azimut')
        ax1.set_ylabel('Posisi Azimut (°)')
        ax1.legend()
        ax1.grid(True)
        plt.setp(ax1.get_xticklabels(), visible=False)
        ax2.plot(time_sec, self.df['Target_Elv'], 'r--', label='Referensi', linewidth=2)
        ax2.plot(time_sec, self.df['Actual_Elv'], 'b-', label='Output Sistem')
        ax2.set_title('Grafik Respon Sistem Motor Sumbu Elevasi')
        ax2.set_xlabel('Waktu (detik)')
        ax2.set_ylabel('Posisi Elevasi (°)')
        ax2.legend()
        ax2.grid(True)
        f.tight_layout(pad=3.0)
        self._show_graph_in_new_window(f, "Grafik Analisis Posisi")

    def create_control_error_graph(self):
        f = plt.Figure(figsize=(10, 8), dpi=100)
        ax1 = f.add_subplot(211)
        ax2 = f.add_subplot(212)
        time_sec = self.df['Timestamp(ms)'] / 1000.0
        ax1.set_title('Grafik Kontrol & Error Motor Sumbu Azimut')
        ax1_twin = ax1.twinx()
        ax1.plot(time_sec, self.df['Error_Azm'], 'g-', label='Error Azimut')
        ax1_twin.plot(time_sec, self.df['Control_Azm'], 'm-', alpha=0.6, label='Control Signal Azimut')
        ax1.set_ylabel('Error (°)', color='g')
        ax1_twin.set_ylabel('Control Signal', color='m')
        ax1.legend(loc='upper left')
        ax1_twin.legend(loc='upper right')
        ax1.grid(True)
        plt.setp(ax1.get_xticklabels(), visible=False)
        ax2.set_title('Grafik Kontrol & Error Motor Sumbu Elevasi')
        ax2_twin = ax2.twinx()
        ax2.plot(time_sec, self.df['Error_Elv'], 'g-', label='Error Elevasi')
        ax2_twin.plot(time_sec, self.df['Control_Elv'], 'm-', alpha=0.6, label='Control Signal Elevasi')
        ax2.set_xlabel('Waktu (detik)')
        ax2.set_ylabel('Error (°)', color='g')
        ax2_twin.set_ylabel('Control Signal', color='m')
        ax2.legend(loc='upper left')
        ax2_twin.legend(loc='upper right')
        ax2.grid(True)
        f.tight_layout(pad=3.0)
        self._show_graph_in_new_window(f, "Grafik Kontrol & Error")

if __name__ == "__main__":
    root = tk.Tk()
    app = ResponAnalyzerApp(root)
    root.mainloop()