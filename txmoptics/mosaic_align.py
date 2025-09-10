import tkinter as tk
from tkinter import ttk, messagebox
import pvaccess as pva
import epics
import matplotlib.pyplot as plt
import numpy as np
import time
import threading
import json
import os
import subprocess

class CameraStitchGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Camera Grid Stitching")
        self.root.geometry("500x450")
        self.config_file = "camera_stitch_config.json"
        
        # Load saved settings
        self.load_config()
        
        # Create GUI elements
        self.create_widgets()
        
        # Status
        self.acquiring = False
        self.stop_requested = False
        
        # Save settings on window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Update mode info on startup
        self.update_mode_info()
        
    def create_widgets(self):
        # Create notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Tab 1: Grid Parameters
        self.grid_frame = ttk.Frame(notebook)
        notebook.add(self.grid_frame, text="Grid Setup")
        
        # Tab 2: PVs
        self.pv_frame = ttk.Frame(notebook)
        notebook.add(self.pv_frame, text="PVs")
        
        # Create widgets in each tab
        self.create_grid_widgets()
        self.create_pv_widgets()
        
        # Buttons and status (outside tabs)
        self.create_buttons_and_status()
    
    def create_grid_widgets(self):
        # Camera parameters
        frame1 = ttk.LabelFrame(self.grid_frame, text="Camera Parameters", padding=10)
        frame1.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(frame1, text="Pixel size (µm):").grid(row=0, column=0, sticky="w", pady=2)
        self.pixel_size_var = tk.StringVar(value=str(self.config.get("pixel_size", 1.0)))
        ttk.Entry(frame1, textvariable=self.pixel_size_var, width=10).grid(row=0, column=1, pady=2)
        
        # Acquisition mode
        frame_acq = ttk.LabelFrame(self.grid_frame, text="Acquisition Mode", padding=10)
        frame_acq.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(frame_acq, text="Mode:").grid(row=0, column=0, sticky="w", pady=2)
        self.acq_mode_var = tk.StringVar(value=self.config.get("acq_mode", "180"))
        mode_combo = ttk.Combobox(frame_acq, textvariable=self.acq_mode_var, values=["180", "360"], state="readonly", width=10)
        mode_combo.grid(row=0, column=1, pady=2, sticky="w")
        
        ttk.Label(frame_acq, text="deg").grid(row=0, column=2, sticky="w", pady=2, padx=(5,0))
        
        # Info label for mode
        self.mode_info = tk.StringVar(value="Standard grid acquisition")
        ttk.Label(frame_acq, textvariable=self.mode_info, foreground="blue", font=("Arial", 8)).grid(row=1, column=0, columnspan=3, sticky="w", pady=2)
        
        # Bind mode change to update info
        mode_combo.bind("<<ComboboxSelected>>", self.update_mode_info)
        
        # Grid parameters
        frame2 = ttk.LabelFrame(self.grid_frame, text="Grid Parameters", padding=10)
        frame2.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(frame2, text="Horizontal steps:").grid(row=0, column=0, sticky="w", pady=2)
        self.h_steps_var = tk.StringVar(value=str(self.config.get("h_steps", 3)))
        ttk.Entry(frame2, textvariable=self.h_steps_var, width=10).grid(row=0, column=1, pady=2)
        
        ttk.Label(frame2, text="Vertical steps:").grid(row=1, column=0, sticky="w", pady=2)
        self.v_steps_var = tk.StringVar(value=str(self.config.get("v_steps", 3)))
        ttk.Entry(frame2, textvariable=self.v_steps_var, width=10).grid(row=1, column=1, pady=2)
        
        ttk.Label(frame2, text="Horizontal step size (mm):").grid(row=0, column=2, sticky="w", padx=(20,0), pady=2)
        self.h_step_size_var = tk.StringVar(value=str(self.config.get("h_step_size", 0.2)))
        ttk.Entry(frame2, textvariable=self.h_step_size_var, width=10).grid(row=0, column=3, pady=2)
        
        ttk.Label(frame2, text="Vertical step size (mm):").grid(row=1, column=2, sticky="w", padx=(20,0), pady=2)
        self.v_step_size_var = tk.StringVar(value=str(self.config.get("v_step_size", 0.2)))
        ttk.Entry(frame2, textvariable=self.v_step_size_var, width=10).grid(row=1, column=3, pady=2)
        
        # Show calculated overlap
        ttk.Label(frame2, text="Calculated overlap:").grid(row=2, column=0, sticky="w", pady=2)
        self.overlap_display = tk.StringVar(value="0%")
        ttk.Label(frame2, textvariable=self.overlap_display, foreground="blue").grid(row=2, column=1, sticky="w", pady=2)
        
        ttk.Button(frame2, text="Update Overlap", command=self.calculate_overlap).grid(row=2, column=2, padx=(20,0), pady=2)
    
    def create_pv_widgets(self):
        # PVs
        frame1 = ttk.LabelFrame(self.pv_frame, text="EPICS Process Variables", padding=10)
        frame1.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(frame1, text="Detector PV:").grid(row=0, column=0, sticky="w", pady=2)
        self.detector_var = tk.StringVar(value=self.config.get("detector_pv", "32idbSP1:Pva1:Image"))
        ttk.Entry(frame1, textvariable=self.detector_var, width=40).grid(row=0, column=1, pady=2, sticky="ew")
        
        ttk.Label(frame1, text="Size X PV:").grid(row=1, column=0, sticky="w", pady=2)
        self.size_x_var = tk.StringVar(value=self.config.get("size_x_pv", "32idbSP1:cam1:ArraySizeX_RBV"))
        ttk.Entry(frame1, textvariable=self.size_x_var, width=40).grid(row=1, column=1, pady=2, sticky="ew")
        
        ttk.Label(frame1, text="Size Y PV:").grid(row=2, column=0, sticky="w", pady=2)
        self.size_y_var = tk.StringVar(value=self.config.get("size_y_pv", "32idbSP1:cam1:SizeY_RBV"))
        ttk.Entry(frame1, textvariable=self.size_y_var, width=40).grid(row=2, column=1, pady=2, sticky="ew")
        
        ttk.Label(frame1, text="X Motor PV:").grid(row=3, column=0, sticky="w", pady=2)
        self.x_motor_var = tk.StringVar(value=self.config.get("x_motor_pv", "32idbSP1:m1"))
        ttk.Entry(frame1, textvariable=self.x_motor_var, width=40).grid(row=3, column=1, pady=2, sticky="ew")
        
        ttk.Label(frame1, text="Y Motor PV:").grid(row=4, column=0, sticky="w", pady=2)
        self.y_motor_var = tk.StringVar(value=self.config.get("y_motor_pv", "32idbSP1:m2"))
        ttk.Entry(frame1, textvariable=self.y_motor_var, width=40).grid(row=4, column=1, pady=2, sticky="ew")
        
        ttk.Label(frame1, text="Rotation PV:").grid(row=5, column=0, sticky="w", pady=2)
        self.rotation_var = tk.StringVar(value=self.config.get("rotation_pv", "32idbSP1:m3"))
        ttk.Entry(frame1, textvariable=self.rotation_var, width=40).grid(row=5, column=1, pady=2, sticky="ew")
        
        # Configure column weight for resizing
        frame1.columnconfigure(1, weight=1)
    
    def create_buttons_and_status(self):
        # Buttons
        frame4 = ttk.Frame(self.root)
        frame4.pack(fill="x", padx=10, pady=10)
        
        # Create styled buttons
        style = ttk.Style()
        style.configure('Green.TButton', foreground='white', background='green')
        style.configure('Orange.TButton', foreground='white', background='orange')
        style.configure('Red.TButton', foreground='white', background='red')
        
        # First row of buttons
        self.preview_btn = ttk.Button(frame4, text="Start Preview", 
                                     command=self.start_preview, 
                                     style='Orange.TButton')
        self.preview_btn.pack(side="left", padx=(0,10))
        
        # Second row - new Start Acquisition button
        self.acquire_btn = ttk.Button(frame4, text="Start Acquisition", 
                                     command=self.start_mosaic_script, 
                                     style='Green.TButton')
        self.acquire_btn.pack(side="left", padx=(0,10))
        
        self.stop_btn = ttk.Button(frame4, text="Stop Acquisition", 
                                  command=self.stop_acquisition,
                                  style='Red.TButton',
                                  state="disabled")
        self.stop_btn.pack(side="left", padx=(0,10))
        
        ttk.Button(frame4, text="Test Single Image", command=self.test_image).pack(side="left")
        
        ttk.Button(frame4, text="Save Settings", command=self.force_save_config).pack(side="left", padx=(10,0))
        
        # Status
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(self.root, textvariable=self.status_var, relief="sunken").pack(fill="x", padx=10, pady=(0,10))
        
        # Progress
        self.progress = ttk.Progressbar(self.root, mode='determinate')
        self.progress.pack(fill="x", padx=10, pady=(0,10))
    
    def update_mode_info(self, event=None):
        """Update info text based on acquisition mode"""
        mode = self.acq_mode_var.get()
        if mode == "180":
            self.mode_info.set("Standard grid acquisition at current rotation")
        else:  # 360
            self.mode_info.set("Double field of view: Grid doubled, half at 0°, half at 180°")
    
    def get_image_size(self):
        """Get image dimensions from PVs"""
        try:
            size_x = epics.caget(self.size_x_var.get())
            size_y = epics.caget(self.size_y_var.get())
            return int(size_y), int(size_x)  # height, width
        except:
            return 2426, 3232  # fallback
    
    def calculate_overlap(self):
        """Calculate overlap based on pixel size, motor step, and image size"""
        try:
            pixel_size = float(self.pixel_size_var.get())  # µm
            h_step_mm = float(self.h_step_size_var.get())  # mm
            v_step_mm = float(self.v_step_size_var.get())  # mm
            
            img_h, img_w = self.get_image_size()
            
            # Convert motor steps to µm
            h_step_um = h_step_mm * 1000  # mm to µm
            v_step_um = v_step_mm * 1000  # mm to µm
            
            # Calculate image size in µm
            img_w_um = img_w * pixel_size
            img_h_um = img_h * pixel_size
            
            # Calculate overlap percentages
            h_overlap = max(0, (img_w_um - h_step_um) / img_w_um)
            v_overlap = max(0, (img_h_um - v_step_um) / img_h_um)
            
            self.overlap_display.set(f"H: {h_overlap:.1%}, V: {v_overlap:.1%}")
            self.calculated_overlap = (h_overlap, v_overlap)
            
        except Exception as e:
            self.overlap_display.set("Error")
            self.calculated_overlap = (0.15, 0.15)  # fallback
    
    def get_image(self):
        """Get single image from camera"""
        detector_pv = self.detector_var.get()
        pv = pva.Channel(detector_pv)
        img_h, img_w = self.get_image_size()
        return pv.get()['value'][0]['ushortValue'].reshape(img_h, img_w)
    
    def move_motors(self, x_pos, y_pos):
        """Move motors to position and wait"""
        x_motor = self.x_motor_var.get()
        y_motor = self.y_motor_var.get()
        epics.caput(x_motor, x_pos, wait=True)
        epics.caput(y_motor, y_pos, wait=True)
        time.sleep(0.1)
    
    def move_rotation(self, angle):
        """Move rotation motor to specified angle"""
        rotation_pv = self.rotation_var.get()
        epics.caput(rotation_pv, angle, wait=True)
        time.sleep(0.2)  # Extra settling time for rotation
    
    def test_image(self):
        """Test single image acquisition"""
        try:
            self.status_var.set("Acquiring test image...")
            self.root.update()
            
            img = self.get_image()
            
            plt.figure(figsize=(10, 8))
            vmin, vmax = np.percentile(img[img > 0], [1, 99])
            plt.imshow(img, cmap='gray', vmin=vmin, vmax=vmax)
            plt.title("Test Image")
            plt.colorbar()
            plt.show()
            
            self.status_var.set("Test image displayed")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to acquire test image: {str(e)}")
            self.status_var.set("Error")
    
    def start_preview(self):
        """Start grid preview in separate thread"""
        if self.acquiring:
            return
            
        try:
            # Validate inputs
            h_steps = int(self.h_steps_var.get())
            v_steps = int(self.v_steps_var.get())
            h_step_size = float(self.h_step_size_var.get())
            v_step_size = float(self.v_step_size_var.get())
            acq_mode = self.acq_mode_var.get()
            
            if h_steps < 1 or v_steps < 1:
                raise ValueError("Steps must be >= 1")
                
        except ValueError as e:
            messagebox.showerror("Input Error", str(e))
            return
        
        # Reset stop flag and update button states
        self.stop_requested = False
        self.preview_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        
        # Start preview thread
        thread = threading.Thread(target=self.acquire_grid)
        thread.daemon = True
        thread.start()
    
    def start_mosaic_script(self):
        """Run the mosaic.sh bash script with parameters"""
        try:
            self.status_var.set("Running mosaic.sh script...")
            self.root.update()
            
            # Get parameters from GUI
            h_steps = self.h_steps_var.get()
            v_steps = self.v_steps_var.get()
            h_step_size = self.h_step_size_var.get()
            v_step_size = self.v_step_size_var.get()
            
            # Get tomoscan prefix from detector PV (extract prefix part)
            detector_pv = self.detector_var.get()
            # Extract prefix (e.g., "32idbSP1:Pva1:Image" -> "32id:TomoScan:")
            tomoscan_prefix = "32id:TomoScan:"  # You may want to make this configurable
            
            # Validate parameters
            try:
                h_steps = int(h_steps)
                v_steps = int(v_steps)
                h_step_size = float(h_step_size)
                v_step_size = float(v_step_size)
                
                if h_steps < 1 or v_steps < 1:
                    raise ValueError("Steps must be >= 1")
            except ValueError as e:
                messagebox.showerror("Parameter Error", f"Invalid parameters: {str(e)}")
                self.status_var.set("Error: Invalid parameters")
                return
            
            # Get the directory where this Python script is located
            script_dir = os.path.dirname(os.path.abspath(__file__))
            mosaic_path = os.path.join(script_dir, 'mosaic.sh')
            
            # Check if the script exists
            if not os.path.exists(mosaic_path):
                raise FileNotFoundError(f"mosaic.sh not found at: {mosaic_path}")
            
            # Prepare command with parameters
            cmd = ['bash', mosaic_path, str(h_steps), str(v_steps), 
                   str(h_step_size), str(v_step_size), tomoscan_prefix]
            
            # Run the bash script with parameters
            result = subprocess.run(cmd, 
                                  capture_output=True, 
                                  text=True, 
                                  cwd=script_dir)
            
            if result.returncode == 0:
                self.status_var.set("Mosaic script completed successfully")
                if result.stdout:
                    messagebox.showinfo("Script Output", f"Script completed:\n{result.stdout}")
            else:
                self.status_var.set("Mosaic script failed")
                error_msg = result.stderr if result.stderr else "Unknown error"
                messagebox.showerror("Script Error", f"Script failed with error:\n{error_msg}")
                
        except FileNotFoundError as e:
            messagebox.showerror("Script Error", f"mosaic.sh script not found:\n{str(e)}")
            self.status_var.set("Error: mosaic.sh not found")
        except Exception as e:
            messagebox.showerror("Script Error", f"Failed to run mosaic.sh:\n{str(e)}")
            self.status_var.set("Error running script")
    
    def stop_acquisition(self):
        """Request to stop the current acquisition"""
        self.stop_requested = True
        self.status_var.set("Stop requested - finishing current image...")
        self.stop_btn.config(state="disabled")
    
    def acquire_grid(self):
        """Acquire grid images"""
        try:
            self.acquiring = True
            self.preview_btn.config(state="disabled")
            
            # Get parameters
            h_steps = int(self.h_steps_var.get())
            v_steps = int(self.v_steps_var.get())
            h_step_size = float(self.h_step_size_var.get())
            v_step_size = float(self.v_step_size_var.get())
            acq_mode = self.acq_mode_var.get()
            
            # Calculate overlap
            self.calculate_overlap()
            h_overlap, v_overlap = getattr(self, 'calculated_overlap', (0.15, 0.15))
            
            # Determine actual grid size based on mode
            if acq_mode == "360":
                actual_h_steps = h_steps * 2
                actual_v_steps = v_steps
                self.status_var.set("Starting 360° double field of view acquisition...")
            else:
                actual_h_steps = h_steps
                actual_v_steps = v_steps
                self.status_var.set("Starting 180° standard acquisition...")
            
            total_images = h_steps * v_steps * (2 if acq_mode == "360" else 1)
            self.progress['maximum'] = total_images
            
            # Get image dimensions
            test_img = self.get_image()
            img_h, img_w = test_img.shape
            
            # Calculate output size
            eff_h = int(img_h * (1 - v_overlap))
            eff_w = int(img_w * (1 - h_overlap))
            out_h = eff_h * actual_v_steps + int(img_h * v_overlap)
            out_w = eff_w * actual_h_steps + int(img_w * h_overlap)
            
            stitched = np.zeros((out_h, out_w), dtype=test_img.dtype)
            stitched_rgb = np.zeros((out_h, out_w, 3), dtype=np.uint8)
            
            # Create and show the plot window immediately
            plt.ion()  # Turn on interactive mode
            fig, ax = plt.subplots(figsize=(15, 12))
            if acq_mode == "360":
                ax.set_title(f'Double Field of View {actual_h_steps}x{actual_v_steps} (White borders)')
            else:
                ax.set_title(f'Stitched Grid {actual_h_steps}x{actual_v_steps} (White borders)')
            
            im = ax.imshow(stitched_rgb)
            plt.show(block=False)
            plt.draw()
            
            self.status_var.set(f"Acquiring {total_images} images ({acq_mode}° mode)...")
            
            # Acquire images based on mode
            count = 0
            
            if acq_mode == "360":
                # 360° mode: Right half for 0°, left half for 180°
                for rot_idx, rotation in enumerate([0, 180]):
                    # Move to rotation angle
                    self.status_var.set(f"Moving to {rotation}° rotation...")
                    self.root.update()
                    self.move_rotation(rotation)
                    
                    # Acquire grid at this rotation
                    for i in range(v_steps):
                        for j in range(h_steps):
                            # Check for stop request
                            if self.stop_requested:
                                self.status_var.set("Acquisition stopped by user")
                                return
                                
                            count += 1
                            
                            # Motor positions (always same grid)
                            x_pos = j * h_step_size
                            y_pos = i * v_step_size
                            
                            self.status_var.set(f"Image {count}/{total_images}: ({j+1},{i+1}) at {rotation}°")
                            self.root.update()
                            
                            # Move and acquire
                            self.move_motors(x_pos, y_pos)
                            img = self.get_image()
                            
                            # Position in big image: 0° on right, 180° on left
                            # For 180°, flip only horizontally to account for rotation
                            if rotation == 0:
                                big_i = i
                                big_j = j + h_steps  # 0° on right side
                            else:  # 180°
                                big_i = i  # Keep vertical order same
                                big_j = (h_steps - 1 - j)  # Flip only horizontal, left side
                            
                            start_y = big_i * eff_h
                            start_x = big_j * eff_w
                            end_y = min(start_y + img_h, out_h)
                            end_x = min(start_x + img_w, out_w)
                            
                            # Put image in big image
                            stitched[start_y:end_y, start_x:end_x] = img[:end_y-start_y, :end_x-start_x]
                            
                            # Put in RGB
                            img_norm = ((img[:end_y-start_y, :end_x-start_x] / img.max() * 255) if img.max() > 0 else img[:end_y-start_y, :end_x-start_x]).astype(np.uint8)
                            stitched_rgb[start_y:end_y, start_x:end_x] = np.stack([img_norm, img_norm, img_norm], axis=2)
                            
                            # Draw thick WHITE frame around this image
                            border_color = (255, 255, 255)  # WHITE
                            thickness = 5  # Thick border
                            
                            # Draw thick borders
                            for t in range(thickness):
                                # Top border
                                if start_y + t < end_y:
                                    stitched_rgb[start_y + t, start_x:end_x] = border_color
                                # Bottom border
                                if end_y - 1 - t >= start_y:
                                    stitched_rgb[end_y - 1 - t, start_x:end_x] = border_color
                                # Left border
                                if start_x + t < end_x:
                                    stitched_rgb[start_y:end_y, start_x + t] = border_color
                                # Right border
                                if end_x - 1 - t >= start_x:
                                    stitched_rgb[start_y:end_y, end_x - 1 - t] = border_color
                            
                            # Update the display immediately
                            im.set_array(stitched_rgb)
                            fig.canvas.draw()
                            fig.canvas.flush_events()
                            
                            self.progress['value'] = count
                            self.root.update()
            else:
                # 180° mode: standard acquisition
                for i in range(v_steps):
                    for j in range(h_steps):
                        # Check for stop request
                        if self.stop_requested:
                            self.status_var.set("Acquisition stopped by user")
                            return
                            
                        count += 1
                        
                        # Motor positions
                        x_pos = j * h_step_size
                        y_pos = i * v_step_size
                        
                        self.status_var.set(f"Image {count}/{total_images}: ({j+1},{i+1})")
                        self.root.update()
                        
                        # Move and acquire
                        self.move_motors(x_pos, y_pos)
                        img = self.get_image()
                        
                        # Position in big image
                        start_y = i * eff_h
                        start_x = j * eff_w
                        end_y = min(start_y + img_h, out_h)
                        end_x = min(start_x + img_w, out_w)
                        
                        # Put image in big image
                        stitched[start_y:end_y, start_x:end_x] = img[:end_y-start_y, :end_x-start_x]
                        
                        # Put in RGB
                        img_norm = ((img[:end_y-start_y, :end_x-start_x] / img.max() * 255) if img.max() > 0 else img[:end_y-start_y, :end_x-start_x]).astype(np.uint8)
                        stitched_rgb[start_y:end_y, start_x:end_x] = np.stack([img_norm, img_norm, img_norm], axis=2)
                        
                        # Draw thick WHITE frame around this image
                        border_color = (255, 255, 255)  # WHITE
                        thickness = 5  # Thick border
                        
                        # Draw thick borders
                        for t in range(thickness):
                            # Top border
                            if start_y + t < end_y:
                                stitched_rgb[start_y + t, start_x:end_x] = border_color
                            # Bottom border
                            if end_y - 1 - t >= start_y:
                                stitched_rgb[end_y - 1 - t, start_x:end_x] = border_color
                            # Left border
                            if start_x + t < end_x:
                                stitched_rgb[start_y:end_y, start_x + t] = border_color
                            # Right border
                            if end_x - 1 - t >= start_x:
                                stitched_rgb[start_y:end_y, end_x - 1 - t] = border_color
                        
                        # Update the display immediately
                        im.set_array(stitched_rgb)
                        fig.canvas.draw()
                        fig.canvas.flush_events()
                        
                        self.progress['value'] = count
                        self.root.update()
            
            # Save final result
            np.savetxt('stitched_image.txt', stitched, fmt='%d', delimiter='\t')
            
            # Turn off interactive mode and keep final plot
            plt.ioff()
            
            self.status_var.set(f"Complete! Saved stitched_image.txt ({out_w}x{out_h}) - {acq_mode}° mode")
            
        except Exception as e:
            messagebox.showerror("Acquisition Error", str(e))
            self.status_var.set("Error")
            
        finally:
            self.acquiring = False
            self.stop_requested = False
            self.preview_btn.config(state="normal")
            self.stop_btn.config(state="disabled")
            self.progress['value'] = 0
    
    def force_save_config(self):
        """Force save current settings to file with user feedback"""
        try:
            self.save_config()
            messagebox.showinfo("Settings Saved", f"Settings saved to:\n{self.config_file}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save settings:\n{str(e)}")
    
    def load_config(self):
        """Load configuration from file"""
        self.config = {}
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    self.config = json.load(f)
            except:
                self.config = {}
    
    def save_config(self):
        """Save current settings to file"""
        def safe_float(value, default):
            try:
                return float(value)
            except:
                return default
        
        def safe_int(value, default):
            try:
                return int(value)
            except:
                return default
        
        config = {
            "detector_pv": self.detector_var.get(),
            "size_x_pv": self.size_x_var.get(),
            "size_y_pv": self.size_y_var.get(),
            "x_motor_pv": self.x_motor_var.get(),
            "y_motor_pv": self.y_motor_var.get(),
            "rotation_pv": self.rotation_var.get(),
            "pixel_size": safe_float(self.pixel_size_var.get(), 1.0),
            "acq_mode": self.acq_mode_var.get(),
            "h_steps": safe_int(self.h_steps_var.get(), 3),
            "v_steps": safe_int(self.v_steps_var.get(), 3),
            "h_step_size": safe_float(self.h_step_size_var.get(), 0.2),
            "v_step_size": safe_float(self.v_step_size_var.get(), 0.2)
        }
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
        except:
            pass
    
    def on_closing(self):
        """Handle window closing"""
        self.save_config()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = CameraStitchGUI(root)
    root.mainloop()
