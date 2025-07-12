import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
import os
import math

class ModernOpticsCalculator:
    def __init__(self, root):
        self.root = root
        self.root.title("X-ray Optics Calculator")
        self.root.geometry("1200x1000")
        self.root.configure(bg='#2c3e50')
        
        # Configuration file
        self.config_file = "optics_config.json"
        
        # Configure modern styling
        self.setup_styles()
        
        # Default configurations - these will be loaded/saved to JSON
        self.default_configs = {
            "zone_plates": {
                "60nm ZP": {"drn": 60, "diameter": 180, "smallest_feature": 30, "efficiency": 20, "thickness": 500},
                "50nm ZP": {"drn": 50, "diameter": 180, "smallest_feature": 25, "efficiency": 25, "thickness": 500},
                "40nm ZP": {"drn": 40, "diameter": 180, "smallest_feature": 20, "efficiency": 30, "thickness": 500},
                "30nm ZP": {"drn": 30, "diameter": 300, "smallest_feature": 30, "efficiency": 35, "thickness": 500},
                "16nm ZP": {"drn": 16, "diameter": 150, "smallest_feature": 16, "efficiency": 35, "thickness": 500}
            },
            "condensers": {
                "Sigray": {"source_dist": 70, "inner_dia": 450, "outer_dia": 750, 
                          "length": 80, "focal_length": 43.2, "na_tip": 5.208, "position_offset": 100},
                "Sigray2": {"source_dist": 35, "inner_dia": 475, "outer_dia": 749,
                           "length": 145, "focal_length": 96.9, "na_tip": 2.451, "position_offset": 150}
            },
            "cameras": {
                "ORX-10G-310S9M": {"pixel_size": 3.45, "pixels_v": 6464, "pixels_h": 4852,
                                             "scintillator": "GGG:Ce", "efficiency": 15},
                "Generic Camera": {"pixel_size": 5.0, "pixels_v": 1024, "pixels_h": 1024,
                                 "scintillator": "CsI:Tl", "efficiency": 10}
            },
            "optical_magnifications": {
                "2x Objective": {"magnification": 2.0, "na": 0.055, "working_distance": 31.0},
                "5x Objective": {"magnification": 5.0, "na": 0.13, "working_distance": 17.0},
                "9x Objective": {"magnification": 9.0, "na": 0.28, "working_distance": 34.0},
            },
            "user_parameters": {
                "camera_distance": 3500.0,
                "binning": 2,
                "exposure_time": 0.001,
                "sample_detector_distance": 3500.0,
                "beam_current": 200.0
            }
        }
        
        # Load configuration first
        self.load_config()
        
        # Then initialize variables for user parameters
        self.init_user_variables()
        
        self.setup_ui()
        self.calculate()
        
    def load_config(self):
        """Load configuration from JSON file"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    loaded_config = json.load(f)
                # Merge with defaults (in case new parameters were added)
                self.configs = self.merge_configs(self.default_configs, loaded_config)
            else:
                self.configs = self.default_configs.copy()
                self.save_config()  # Create initial config file
        except Exception as e:
            print(f"Error loading config: {e}")
            self.configs = self.default_configs.copy()
    
    def merge_configs(self, default, loaded):
        """Merge loaded config with defaults to handle new parameters"""
        result = default.copy()
        for key, value in loaded.items():
            if key in result and isinstance(value, dict):
                result[key] = self.merge_configs(result[key], value)
            else:
                result[key] = value
        return result
    
    def save_config_with_feedback(self):
        """Save config and show user feedback"""
        if self.save_config():
            messagebox.showinfo("Save Complete", "✓ Configuration saved successfully!")
    
    def save_config(self):
        """Save current configuration to JSON file"""
        try:
            # Update user parameters from UI only if variables exist
            if hasattr(self, 'camera_distance_var'):
                self.configs["user_parameters"]["camera_distance"] = float(self.camera_distance_var.get())
                self.configs["user_parameters"]["binning"] = int(self.binning_var.get())
                self.configs["user_parameters"]["exposure_time"] = float(self.exposure_time_var.get())
                self.configs["user_parameters"]["sample_detector_distance"] = float(self.sample_det_distance_var.get())
                self.configs["user_parameters"]["beam_current"] = float(self.beam_current_var.get())
            
            with open(self.config_file, 'w') as f:
                json.dump(self.configs, f, indent=2)
            
            if hasattr(self, 'status_var'):
                self.status_var.set("✓ Configuration saved successfully")
            return True
        except ValueError as e:
            if hasattr(self, 'status_var'):
                self.status_var.set(f"❌ Invalid value: {e}")
            else:
                messagebox.showerror("Save Error", f"Invalid value: {e}")
            return False
        except Exception as e:
            if hasattr(self, 'status_var'):
                self.status_var.set(f"❌ Save error: {e}")
            else:
                messagebox.showerror("Save Error", f"Failed to save config: {e}")
            return False
    
    def init_user_variables(self):
        """Initialize user parameter variables"""
        user_params = self.configs["user_parameters"]
        self.camera_distance_var = tk.StringVar(value=str(user_params.get("camera_distance", 3500.0)))
        self.binning_var = tk.StringVar(value=str(user_params.get("binning", 2)))
        self.exposure_time_var = tk.StringVar(value=str(user_params.get("exposure_time", 0.001)))
        self.sample_det_distance_var = tk.StringVar(value=str(user_params.get("sample_detector_distance", 3500.0)))
        self.beam_current_var = tk.StringVar(value=str(user_params.get("beam_current", 200.0)))
        
    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        # Configure styles for modern look
        style.configure('Title.TLabel', 
                       font=('Segoe UI', 24, 'bold'),
                       background='#2c3e50',
                       foreground='white')
        
        style.configure('Heading.TLabel',
                       font=('Segoe UI', 12, 'bold'),
                       background='#34495e',
                       foreground='white',
                       padding=10)
        
        style.configure('Modern.TEntry',
                       font=('Segoe UI', 10),
                       fieldbackground='white',
                       borderwidth=2,
                       relief='flat')
        
        style.configure('Modern.TCombobox',
                       font=('Segoe UI', 10),
                       fieldbackground='white',
                       borderwidth=2,
                       relief='flat')
        
    def setup_ui(self):
        # Main container
        main_container = tk.Frame(self.root, bg='#2c3e50')
        main_container.pack(fill='both', expand=True, padx=20, pady=20)
        
        # Title
        title_label = ttk.Label(main_container, text="X-ray Optics Calculator", style='Title.TLabel')
        title_label.pack(pady=(0, 20))
        
        # Create notebook for tabbed interface
        notebook = ttk.Notebook(main_container)
        notebook.pack(fill='both', expand=True)
        
        # Main calculation tab
        calc_frame = tk.Frame(notebook, bg='#2c3e50')
        notebook.add(calc_frame, text="Calculator")
        
        # Configuration tab
        config_frame = tk.Frame(notebook, bg='#2c3e50')
        notebook.add(config_frame, text="Configuration")
        
        # Setup main calculator interface
        self.setup_calculator_tab(calc_frame)
        
        # Setup configuration interface
        self.setup_config_tab(config_frame)
        
    def setup_calculator_tab(self, parent):
        # Input section
        self.create_input_section(parent)
        
        # User parameters section
        self.create_user_params_section(parent)
        
        # Results section
        self.create_results_section(parent)
        
        # Control buttons
        self.create_control_section(parent)
        
        # Status bar
        self.create_status_section(parent)
        
    def create_input_section(self, parent):
        input_frame = tk.Frame(parent, bg='#34495e', relief='raised', bd=2)
        input_frame.pack(fill='x', pady=(0, 10))
        
        # Input header
        header_label = ttk.Label(input_frame, text="Input Parameters", style='Heading.TLabel')
        header_label.pack(fill='x')
        
        # Input content
        content_frame = tk.Frame(input_frame, bg='#ecf0f1', padx=20, pady=15)
        content_frame.pack(fill='x')
        
        # Energy input row
        energy_frame = tk.Frame(content_frame, bg='#ecf0f1')
        energy_frame.pack(fill='x', pady=(0, 10))
        
        tk.Label(energy_frame, text="Energy (eV):", font=('Segoe UI', 11), 
                bg='#ecf0f1', fg='#2c3e50').pack(side='left')
        
        self.energy_var = tk.StringVar(value="8000")
        energy_entry = ttk.Entry(energy_frame, textvariable=self.energy_var, 
                               style='Modern.TEntry', width=12)
        energy_entry.pack(side='left', padx=(10, 20))
        energy_entry.bind('<KeyRelease>', self.on_value_change)
        
        tk.Label(energy_frame, text="Wavelength:", font=('Segoe UI', 11),
                bg='#ecf0f1', fg='#2c3e50').pack(side='left')
        
        self.wavelength_var = tk.StringVar()
        wavelength_label = tk.Label(energy_frame, textvariable=self.wavelength_var,
                                  font=('Segoe UI', 11, 'bold'), bg='#ecf0f1', fg='#3498db')
        wavelength_label.pack(side='left', padx=(10, 5))
        
        tk.Label(energy_frame, text="nm", font=('Segoe UI', 11),
                bg='#ecf0f1', fg='#2c3e50').pack(side='left')
        
        # Selections row
        select_frame = tk.Frame(content_frame, bg='#ecf0f1')
        select_frame.pack(fill='x')
        
        # Zone plate selection
        zp_frame = tk.Frame(select_frame, bg='#ecf0f1')
        zp_frame.pack(side='left', fill='x', expand=True, padx=(0, 8))
        
        tk.Label(zp_frame, text="Zone Plate:", font=('Segoe UI', 11),
                bg='#ecf0f1', fg='#2c3e50').pack(anchor='w')
        
        self.zp_var = tk.StringVar(value="30nm ZP")
        zp_combo = ttk.Combobox(zp_frame, textvariable=self.zp_var,
                               values=list(self.configs["zone_plates"].keys()),
                               style='Modern.TCombobox', state='readonly')
        zp_combo.pack(fill='x', pady=(5, 0))
        zp_combo.bind('<<ComboboxSelected>>', self.on_selection_change)
        
        # Condenser selection
        cond_frame = tk.Frame(select_frame, bg='#ecf0f1')
        cond_frame.pack(side='left', fill='x', expand=True, padx=(8, 8))
        
        tk.Label(cond_frame, text="Condenser:", font=('Segoe UI', 11),
                bg='#ecf0f1', fg='#2c3e50').pack(anchor='w')
        
        self.condenser_var = tk.StringVar(value="Sigray")
        cond_combo = ttk.Combobox(cond_frame, textvariable=self.condenser_var,
                                 values=list(self.configs["condensers"].keys()),
                                 style='Modern.TCombobox', state='readonly')
        cond_combo.pack(fill='x', pady=(5, 0))
        cond_combo.bind('<<ComboboxSelected>>', self.on_selection_change)
        
        # Camera selection
        cam_frame = tk.Frame(select_frame, bg='#ecf0f1')
        cam_frame.pack(side='left', fill='x', expand=True, padx=(8, 8))
        
        tk.Label(cam_frame, text="Camera:", font=('Segoe UI', 11),
                bg='#ecf0f1', fg='#2c3e50').pack(anchor='w')
        
        self.camera_var = tk.StringVar(value="ORX-10G-310S9M")
        cam_combo = ttk.Combobox(cam_frame, textvariable=self.camera_var,
                                values=list(self.configs["cameras"].keys()),
                                style='Modern.TCombobox', state='readonly')
        cam_combo.pack(fill='x', pady=(5, 0))
        cam_combo.bind('<<ComboboxSelected>>', self.on_selection_change)
        
        # Optical Magnification selection
        opt_mag_frame = tk.Frame(select_frame, bg='#ecf0f1')
        opt_mag_frame.pack(side='left', fill='x', expand=True, padx=(8, 0))
        
        tk.Label(opt_mag_frame, text="Optical Mag:", font=('Segoe UI', 11),
                bg='#ecf0f1', fg='#2c3e50').pack(anchor='w')
        
        self.optical_mag_var = tk.StringVar(value="2x Objective")
        opt_mag_combo = ttk.Combobox(opt_mag_frame, textvariable=self.optical_mag_var,
                                    values=list(self.configs["optical_magnifications"].keys()),
                                    style='Modern.TCombobox', state='readonly')
        opt_mag_combo.pack(fill='x', pady=(5, 0))
        opt_mag_combo.bind('<<ComboboxSelected>>', self.on_selection_change)
        
    def create_user_params_section(self, parent):
        user_frame = tk.Frame(parent, bg='#34495e', relief='raised', bd=2)
        user_frame.pack(fill='x', pady=(0, 10))
        
        # Header
        header_label = ttk.Label(user_frame, text="User Parameters", style='Heading.TLabel')
        header_label.pack(fill='x')
        
        # Content
        content_frame = tk.Frame(user_frame, bg='#ecf0f1', padx=20, pady=15)
        content_frame.pack(fill='x')
        
        # Create parameter fields
        params_grid = tk.Frame(content_frame, bg='#ecf0f1')
        params_grid.pack(fill='x')
        
        # Camera Distance
        self.create_param_field(params_grid, 0, 0, "Camera Distance:", self.camera_distance_var, "mm")
        
        # Binning
        self.create_param_field(params_grid, 0, 1, "Binning:", self.binning_var, "")
        
        # Exposure Time  
        self.create_param_field(params_grid, 0, 2, "Exposure Time:", self.exposure_time_var, "s")
        
        # Sample-Detector Distance
        self.create_param_field(params_grid, 1, 0, "Sample-Det Distance:", self.sample_det_distance_var, "mm")
        
        # Beam Current
        self.create_param_field(params_grid, 1, 1, "Beam Current:", self.beam_current_var, "mA")
        
        # Save button with improved callback
        save_btn = tk.Button(params_grid, text="Save Config", font=('Segoe UI', 10, 'bold'),
                           bg='#f39c12', fg='white', relief='flat', padx=20, pady=5,
                           command=self.save_config_with_feedback, cursor='hand2')
        save_btn.grid(row=1, column=2, padx=10, pady=5, sticky='ew')
        
        # Configure grid
        for i in range(3):
            params_grid.grid_columnconfigure(i, weight=1)
            
    def create_param_field(self, parent, row, col, label, var, unit):
        frame = tk.Frame(parent, bg='#ecf0f1')
        frame.grid(row=row, column=col, padx=10, pady=5, sticky='ew')
        
        tk.Label(frame, text=label, font=('Segoe UI', 10),
                bg='#ecf0f1', fg='#2c3e50').pack(anchor='w')
        
        entry_frame = tk.Frame(frame, bg='#ecf0f1')
        entry_frame.pack(fill='x')
        
        entry = ttk.Entry(entry_frame, textvariable=var, style='Modern.TEntry', width=12)
        entry.pack(side='left', fill='x', expand=True)
        entry.bind('<KeyRelease>', self.on_value_change)
        
        if unit:
            tk.Label(entry_frame, text=unit, font=('Segoe UI', 10),
                    bg='#ecf0f1', fg='#7f8c8d').pack(side='left', padx=(5, 0))
        
    def create_results_section(self, parent):
        results_container = tk.Frame(parent, bg='#2c3e50')
        results_container.pack(fill='both', expand=True, pady=(0, 10))
        
        # Zone Plate Results
        self.create_result_card(results_container, "Zone Plate Calculations", 0, 0, [
            ("Diameter", "zp_diameter", "μm", "value"),
            ("Outer Ring (Δrn)", "zp_drn", "nm", "value"),
            ("Smallest Feature", "zp_smallest", "nm", "value"),
            ("Focal Length", "zp_focal", "mm", "value"),
            ("Numerical Aperture", "zp_na", "mrad", "value"),
            ("Number of Zones", "zp_zones", "", "value"),
            ("Motor Position", "zp_position", "mm", "motor")
        ])
        
        # Camera Results  
        self.create_result_card(results_container, "Camera & Imaging", 0, 1, [
            ("Pixel Size", "cam_pixel", "μm", "value"),
            ("Resolution", "cam_pixels", "px", "value"),
            ("Effective Resolution", "cam_eff_pixels", "px", "value"),
            ("Optical Magnification", "opt_mag", "×", "value"),
            ("Total Magnification", "total_mag", "×", "value"),
            ("Effective Pixel", "cam_eff_pixel", "nm", "value"),
            ("Field of View", "cam_fov", "μm", "value"),
            ("Distance (User)", "cam_distance_user", "mm", "motor")
        ])
        
        # Condenser Results
        self.create_result_card(results_container, "Condenser", 0, 2, [
            ("Type", "cond_type", "", "value"),
            ("Source Distance", "cond_source", "m", "value"),
            ("Inner Diameter", "cond_inner", "μm", "value"),
            ("Outer Diameter", "cond_outer", "μm", "value"),
            ("Length", "cond_length", "mm", "value"),
            ("Focal Length", "cond_focal", "mm", "value"),
            ("Motor Position", "cond_position", "mm", "motor")
        ])
        
        # Configure grid weights
        results_container.grid_columnconfigure(0, weight=1)
        results_container.grid_columnconfigure(1, weight=1)
        results_container.grid_columnconfigure(2, weight=1)
        
    def create_result_card(self, parent, title, row, col, fields):
        # Card frame
        card_frame = tk.Frame(parent, bg='#ecf0f1', relief='raised', bd=3)
        card_frame.grid(row=row, column=col, sticky='nsew', padx=8, pady=5)
        
        # Card header
        header_frame = tk.Frame(card_frame, bg='#34495e', height=35)
        header_frame.pack(fill='x')
        header_frame.pack_propagate(False)
        
        header_label = tk.Label(header_frame, text=title, font=('Segoe UI', 11, 'bold'),
                               bg='#34495e', fg='white')
        header_label.pack(expand=True)
        
        # Card content
        content_frame = tk.Frame(card_frame, bg='#ecf0f1', padx=12, pady=12)
        content_frame.pack(fill='both', expand=True)
        
        # Create fields
        for i, (label, var_name, unit, style_type) in enumerate(fields):
            field_frame = tk.Frame(content_frame, bg='#ecf0f1')
            field_frame.pack(fill='x', pady=2)
            
            # Label
            label_widget = tk.Label(field_frame, text=f"{label}:", font=('Segoe UI', 9),
                                  bg='#ecf0f1', fg='#2c3e50', width=14, anchor='w')
            label_widget.pack(side='left')
            
            # Value
            var = tk.StringVar()
            setattr(self, f"{var_name}_var", var)
            
            color = '#e74c3c' if style_type == 'motor' else '#3498db'
            value_widget = tk.Label(field_frame, textvariable=var,
                                  font=('Segoe UI', 9, 'bold'),
                                  bg='#ecf0f1', fg=color, anchor='w', width=10)
            value_widget.pack(side='left', padx=(5, 5))
            
            # Unit
            if unit:
                unit_widget = tk.Label(field_frame, text=unit, font=('Segoe UI', 9),
                                     bg='#ecf0f1', fg='#7f8c8d', anchor='w')
                unit_widget.pack(side='left')
    
    def setup_config_tab(self, parent):
        # Configuration editor - simplified for now
        config_label = tk.Label(parent, text="Configuration Editor", 
                               font=('Segoe UI', 16, 'bold'),
                               bg='#2c3e50', fg='white')
        config_label.pack(pady=20)
        
        # Buttons for config management
        button_frame = tk.Frame(parent, bg='#2c3e50')
        button_frame.pack(pady=20)
        
        tk.Button(button_frame, text="Export Config", font=('Segoe UI', 11, 'bold'),
                 bg='#3498db', fg='white', relief='flat', padx=20, pady=8,
                 command=self.export_config, cursor='hand2').pack(side='left', padx=10)
        
        tk.Button(button_frame, text="Import Config", font=('Segoe UI', 11, 'bold'),
                 bg='#9b59b6', fg='white', relief='flat', padx=20, pady=8,
                 command=self.import_config, cursor='hand2').pack(side='left', padx=10)
        
        tk.Button(button_frame, text="Reset to Defaults", font=('Segoe UI', 11, 'bold'),
                 bg='#e67e22', fg='white', relief='flat', padx=20, pady=8,
                 command=self.reset_to_defaults, cursor='hand2').pack(side='left', padx=10)
    
    def create_control_section(self, parent):
        control_frame = tk.Frame(parent, bg='#2c3e50')
        control_frame.pack(fill='x', pady=(0, 10))
        
        button_frame = tk.Frame(control_frame, bg='#2c3e50')
        button_frame.pack()
        
        # Calculate button
        calc_btn = tk.Button(button_frame, text="Calculate", font=('Segoe UI', 12, 'bold'),
                           bg='#3498db', fg='white', relief='flat', padx=30, pady=10,
                           command=self.calculate, cursor='hand2')
        calc_btn.pack(side='left', padx=(0, 10))
        
        # Reset button  
        reset_btn = tk.Button(button_frame, text="Reset", font=('Segoe UI', 12, 'bold'),
                            bg='#95a5a6', fg='white', relief='flat', padx=30, pady=10,
                            command=self.reset, cursor='hand2')
        reset_btn.pack(side='left', padx=(0, 10))
        
        # Export button
        export_btn = tk.Button(button_frame, text="Export Results", font=('Segoe UI', 12, 'bold'),
                             bg='#27ae60', fg='white', relief='flat', padx=30, pady=10,
                             command=self.export_results, cursor='hand2')
        export_btn.pack(side='left')
        
    def create_status_section(self, parent):
        status_frame = tk.Frame(parent, bg='#34495e', relief='sunken', bd=1)
        status_frame.pack(fill='x')
        
        self.status_var = tk.StringVar(value="Ready - Configuration loaded")
        status_label = tk.Label(status_frame, textvariable=self.status_var,
                              font=('Segoe UI', 10), bg='#34495e', fg='white',
                              anchor='w', padx=10, pady=5)
        status_label.pack(fill='x')
    
    def on_value_change(self, event=None):
        self.root.after(100, self.calculate)
    
    def on_selection_change(self, event=None):
        # Update parameters when selection changes
        self.root.after(50, self.calculate)
    
    def calculate(self):
        try:
            # Get energy and calculate wavelength
            energy = float(self.energy_var.get())
            wavelength = 1240 / energy
            self.wavelength_var.set(f"{wavelength:.3f}")
            
            # Get configurations - with error handling for missing keys
            zp_config = self.configs["zone_plates"][self.zp_var.get()]
            camera_config = self.configs["cameras"][self.camera_var.get()]
            cond_config = self.configs["condensers"][self.condenser_var.get()]
            
            # Check if optical magnification exists
            if self.optical_mag_var.get() in self.configs["optical_magnifications"]:
                opt_mag_config = self.configs["optical_magnifications"][self.optical_mag_var.get()]
                optical_magnification = opt_mag_config["magnification"]
            else:
                # Default to first available option
                first_opt_mag = list(self.configs["optical_magnifications"].keys())[0]
                self.optical_mag_var.set(first_opt_mag)
                opt_mag_config = self.configs["optical_magnifications"][first_opt_mag]
                optical_magnification = opt_mag_config["magnification"]
            
            # Get user parameters
            camera_distance = float(self.camera_distance_var.get())
            binning = int(self.binning_var.get())
            
            # Zone plate values
            diameter = zp_config["diameter"]
            drn = zp_config["drn"]
            smallest_feature = zp_config["smallest_feature"]
            
            focal_length = (diameter * drn) / wavelength / 1000
            na = (1000 * wavelength) / (2 * drn)
            num_zones = (1000 * diameter) / (4 * drn)
            zp_position = focal_length
            
            # Camera calculations with user parameters and optical magnification
            pixel_size = camera_config["pixel_size"]
            pixels_v = camera_config["pixels_v"]
            pixels_h = camera_config["pixels_h"]
            
            # Apply binning
            eff_pixels_v = pixels_v // binning
            eff_pixels_h = pixels_h // binning
            eff_pixel_size = pixel_size * binning
            
            # Total magnification: Zone plate magnification × Optical magnification
            zp_magnification = camera_distance / focal_length
            total_magnification = zp_magnification * optical_magnification
            
            # Effective pixel size at sample (accounting for total magnification)
            effective_pixel = (eff_pixel_size * 1000) / total_magnification  # nm
            
            chip_size_v = eff_pixels_v * eff_pixel_size / 1000
            chip_size_h = eff_pixels_h * eff_pixel_size / 1000
            fov_v = (chip_size_v * 1000) / total_magnification
            fov_h = (chip_size_h * 1000) / total_magnification
            
            # Update displays
            self.zp_diameter_var.set(f"{diameter}")
            self.zp_drn_var.set(f"{drn}")
            self.zp_smallest_var.set(f"{smallest_feature}")
            self.zp_focal_var.set(f"{focal_length:.2f}")
            self.zp_na_var.set(f"{na:.2f}")
            self.zp_zones_var.set(f"{num_zones:.0f}")
            self.zp_position_var.set(f"{zp_position:.2f}")
            
            self.cam_pixel_var.set(f"{pixel_size}")
            self.cam_pixels_var.set(f"{pixels_v}×{pixels_h}")
            self.cam_eff_pixels_var.set(f"{eff_pixels_v}×{eff_pixels_h}")
            self.opt_mag_var.set(f"{optical_magnification:.1f}")
            self.total_mag_var.set(f"{total_magnification:.1f}")
            self.cam_eff_pixel_var.set(f"{effective_pixel:.0f}")
            self.cam_fov_var.set(f"{fov_v:.0f}×{fov_h:.0f}")
            self.cam_distance_user_var.set(f"{camera_distance}")
            
            self.cond_type_var.set(self.condenser_var.get())
            self.cond_source_var.set(f"{cond_config['source_dist']}")
            self.cond_inner_var.set(f"{cond_config['inner_dia']}")
            self.cond_outer_var.set(f"{cond_config['outer_dia']}")
            self.cond_length_var.set(f"{cond_config['length']}")
            self.cond_focal_var.set(f"{cond_config['focal_length']}")
            self.cond_position_var.set(f"{cond_config['focal_length']}")
            
            self.status_var.set(f"✓ Calculated for {energy} eV - λ = {wavelength:.3f} nm - Binning: {binning}×")
            
        except KeyError as e:
            self.status_var.set(f"❌ Configuration error: Missing {e}")
        except ValueError as e:
            self.status_var.set(f"❌ Error: Invalid input value - {e}")
        except Exception as e:
            self.status_var.set(f"❌ Calculation error: {str(e)}")
    
    def reset(self):
        self.energy_var.set("8000")
        self.zp_var.set("30nm ZP")
        self.condenser_var.set("Sigray")
        self.camera_var.set("ORX-10G-310S9M")
        self.optical_mag_var.set("2x Objective")
        self.calculate()
        self.status_var.set("🔄 Reset to test values")
    
    def export_config(self):
        filename = filedialog.asksaveasfilename(
            title="Export Configuration",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            defaultextension=".json"
        )
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.configs, f, indent=2)
                messagebox.showinfo("Export Complete", f"✓ Configuration exported to {filename}")
            except Exception as e:
                messagebox.showerror("Export Error", f"❌ Failed to export: {e}")
    
    def import_config(self):
        filename = filedialog.askopenfilename(
            title="Import Configuration",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            try:
                with open(filename, 'r') as f:
                    imported_config = json.load(f)
                self.configs = self.merge_configs(self.default_configs, imported_config)
                
                # Update UI with new config
                self.init_user_variables()
                self.update_dropdowns()
                self.calculate()
                
                messagebox.showinfo("Import Complete", f"✓ Configuration imported from {filename}")
                self.status_var.set("✓ Configuration imported successfully")
            except Exception as e:
                messagebox.showerror("Import Error", f"❌ Failed to import: {e}")
    
    def reset_to_defaults(self):
        if messagebox.askyesno("Reset Configuration", 
                              "Are you sure you want to reset all configurations to defaults?"):
            self.configs = self.default_configs.copy()
            self.init_user_variables()
            self.update_dropdowns()
            self.save_config()
            self.calculate()
            messagebox.showinfo("Reset Complete", "✓ Configuration reset to defaults")
            self.status_var.set("✓ Configuration reset to defaults")
    
    def update_dropdowns(self):
        """Update dropdown values when configuration changes"""
        # Update zone plate dropdown
        zp_combo = None
        cond_combo = None
        cam_combo = None
        
        # Find the comboboxes and update their values
        # This is a simplified approach - in practice you might want to store references
        for widget in self.root.winfo_children():
            if hasattr(widget, 'winfo_children'):
                self.find_and_update_combos(widget)
    
    def find_and_update_combos(self, widget):
        """Recursively find and update comboboxes"""
        for child in widget.winfo_children():
            if isinstance(child, ttk.Combobox):
                if child['textvariable'] == str(self.zp_var):
                    child['values'] = list(self.configs["zone_plates"].keys())
                elif child['textvariable'] == str(self.condenser_var):
                    child['values'] = list(self.configs["condensers"].keys())
                elif child['textvariable'] == str(self.camera_var):
                    child['values'] = list(self.configs["cameras"].keys())
                elif child['textvariable'] == str(self.optical_mag_var):
                    child['values'] = list(self.configs["optical_magnifications"].keys())
            elif hasattr(child, 'winfo_children'):
                self.find_and_update_combos(child)
    
    def export_results(self):
        try:
            # Get current values for export
            energy = self.energy_var.get()
            wavelength = self.wavelength_var.get()
            
            results = f"""X-ray Optics Calculation Results
================================
Timestamp: {self.get_timestamp()}
Configuration: {self.config_file}

Input Parameters:
- Energy: {energy} eV
- Wavelength: {wavelength} nm
- Zone Plate: {self.zp_var.get()}
- Condenser: {self.condenser_var.get()}
- Camera: {self.camera_var.get()}
- Optical Magnification: {self.optical_mag_var.get()}

User Parameters:
- Camera Distance: {self.camera_distance_var.get()} mm
- Binning: {self.binning_var.get()}×
- Exposure Time: {self.exposure_time_var.get()} s
- Sample-Detector Distance: {self.sample_det_distance_var.get()} mm
- Beam Current: {self.beam_current_var.get()} mA

Zone Plate Results:
- Diameter: {self.zp_diameter_var.get()} μm
- Outer Ring (Δrn): {self.zp_drn_var.get()} nm
- Smallest Feature: {self.zp_smallest_var.get()} nm
- Focal Length: {self.zp_focal_var.get()} mm
- Numerical Aperture: {self.zp_na_var.get()} mrad
- Number of Zones: {self.zp_zones_var.get()}
- Motor Position: {self.zp_position_var.get()} mm

Camera & Imaging Results:
- Pixel Size: {self.cam_pixel_var.get()} μm
- Resolution: {self.cam_pixels_var.get()} px
- Effective Resolution: {self.cam_eff_pixels_var.get()} px
- Optical Magnification: {self.opt_mag_var.get()}×
- Total Magnification: {self.total_mag_var.get()}×
- Effective Pixel Size: {self.cam_eff_pixel_var.get()} nm
- Field of View: {self.cam_fov_var.get()} μm
- Camera Distance: {self.cam_distance_user_var.get()} mm

Condenser Results:
- Type: {self.cond_type_var.get()}
- Source Distance: {self.cond_source_var.get()} m
- Inner Diameter: {self.cond_inner_var.get()} μm
- Outer Diameter: {self.cond_outer_var.get()} μm
- Length: {self.cond_length_var.get()} mm
- Focal Length: {self.cond_focal_var.get()} mm
- Motor Position: {self.cond_position_var.get()} mm

Notes:
- All motor positions are calculated values
- Camera distance is user-defined parameter
- ZP Magnification = Camera Distance / ZP Focal Length
- Total Magnification = ZP Magnification × Optical Magnification
- Effective pixel = (Pixel Size × Binning × 1000) / Total Magnification
"""
            
            # Save to file
            filename = f"optics_results_{self.get_timestamp(True)}.txt"
            with open(filename, "w", encoding='utf-8') as f:
                f.write(results)
            
            messagebox.showinfo("Export Complete", f"✓ Results saved to {filename}")
            self.status_var.set(f"✓ Results exported to {filename}")
            
        except Exception as e:
            messagebox.showerror("Export Error", f"❌ Failed to export: {str(e)}")
    
    def get_timestamp(self, for_filename=False):
        """Get current timestamp"""
        import datetime
        now = datetime.datetime.now()
        if for_filename:
            return now.strftime("%Y%m%d_%H%M%S")
        else:
            return now.strftime("%Y-%m-%d %H:%M:%S")

def main():
    root = tk.Tk()
    app = ModernOpticsCalculator(root)
    root.mainloop()

if __name__ == "__main__":
    main()
