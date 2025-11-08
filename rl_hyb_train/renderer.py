"""Renderer module for visualizing environment state."""
import numpy as np
from typing import Dict, Optional, Deque
from collections import deque

from .config import RendererConfig


class RenderBuffer:
    """Ring buffer for storing render data."""
    
    def __init__(self, maxlen: int):
        self.buffer = deque(maxlen=maxlen)
    
    def append(self, data: Dict):
        """Append data to buffer."""
        self.buffer.append(data)
    
    def clear(self):
        """Clear buffer."""
        self.buffer.clear()
    
    def __len__(self):
        return len(self.buffer)
    
    def __iter__(self):
        return iter(self.buffer)
    
    def __getitem__(self, idx):
        return self.buffer[idx]


class Renderer:
    """Renderer for environment visualization."""
    
    def __init__(self, config: RendererConfig):
        self.config = config
        self.fig: Optional[object] = None
        self.axes = {}
        self.artists = {}
        self._initialized = False
        self._matplotlib_imported = False
        self._full_buffer = deque()  # Store full episode for scrubbing
        self._slider = None  # Time slider widget (matplotlib)
        self._pygame_initialized = False
        self._pygame_screen = None
        self._pygame_clock = None
        self._slider_pos = 1.0  # Current slider position (0.0 to 1.0)
        self._slider_dragging = False
        self._paused = False  # Pause state
        
        # Always use Agg backend - we'll use pygame for interactivity
        import os
        if config.mode != "ansi":
            os.environ['MPLBACKEND'] = 'Agg'
        
        # Only import matplotlib if not in ansi mode
        if config.mode != "ansi":
            self._import_matplotlib()
        
        # Import pygame if interactive mode
        if config.interactive:
            self._import_pygame()
    
    def _import_pygame(self):
        """Import pygame for interactive rendering."""
        try:
            import pygame
            self.pygame = pygame
            self._pygame_available = True
        except ImportError:
            self._pygame_available = False
            self.config.interactive = False
            print("Warning: pygame not available. Install with: uv pip install pygame")
    
    def _import_matplotlib(self):
        """Lazy import matplotlib."""
        if self._matplotlib_imported:
            return
        
        try:
            import matplotlib
            # Always use Agg backend - pygame handles interactivity
            import os
            os.environ['MPLBACKEND'] = 'Agg'
            matplotlib.use('Agg', force=True)
            
            import matplotlib.pyplot as plt
            from matplotlib.figure import Figure
            from matplotlib.backends.backend_agg import FigureCanvasAgg
            
            self.plt = plt
            self.Figure = Figure
            self.FigureCanvasAgg = FigureCanvasAgg
            self._matplotlib_imported = True
            self._backend = 'Agg'
        except ImportError as e:
            raise ImportError(
                "matplotlib is required for rendering. "
                "Install it with: uv pip install matplotlib"
            ) from e
    
    def _initialize_figure(self):
        """Initialize matplotlib figure and axes."""
        if self._initialized:
            return
        
        if not self._matplotlib_imported:
            self._import_matplotlib()
        
        self.fig = self.plt.figure(figsize=self.config.figsize, dpi=self.config.dpi)
        self.fig.suptitle("RL Hybrid Train Environment", fontsize=16, fontweight='bold')
        
        # Create grid layout: 3 rows, 3 columns (power diagram takes middle column)
        # Adjust bottom margin if interactive mode (need space for slider in pygame, not matplotlib)
        bottom_margin = 0.07  # Keep same margin - pygame slider is separate
        gs = self.fig.add_gridspec(3, 3, hspace=0.4, wspace=0.4,
                                     left=0.07, right=0.96, top=0.93, bottom=bottom_margin)
        
        # Top bar: timeline (spans all columns)
        ax_timeline = self.fig.add_subplot(gs[0, :])
        self.axes['timeline'] = ax_timeline
        
        # Panel A: Power diagram (left)
        ax_power_diagram = self.fig.add_subplot(gs[1, 0])
        self.axes['power_diagram'] = ax_power_diagram
        
        # Panel B: Power flow time series (spans middle and right columns - bigger!)
        ax_power = self.fig.add_subplot(gs[1, 1:])
        self.axes['power'] = ax_power
        
        # Panel C: Speed tracking (moved to bottom row, left)
        ax_speed = self.fig.add_subplot(gs[2, 0])
        self.axes['speed'] = ax_speed
        
        # Panel D: Stores (bottom-middle)
        ax_stores = self.fig.add_subplot(gs[2, 1])
        self.axes['stores'] = ax_stores
        
        # Panel E: Cost rate (bottom-right)
        ax_cost = self.fig.add_subplot(gs[2, 2])
        self.axes['cost'] = ax_cost
        
        # Slider will be created dynamically in render_human when we have data
        
        # Initialize artists (pre-allocate for performance)
        self._initialize_artists()
        
        self._initialized = True
    
    def _initialize_artists(self):
        """Pre-allocate matplotlib artists."""
        # Timeline artists
        ax = self.axes['timeline']
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis('off')
        self.artists['timeline_progress'] = ax.barh(0, 0, height=0.3, color='blue', alpha=0.5)[0]
        self.artists['timeline_text'] = ax.text(0.5, 0.5, '', ha='center', va='center', fontsize=10)
        
        # Speed tracking artists
        ax = self.axes['speed']
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.grid(True, alpha=0.3)
        self.artists['speed_line'] = ax.plot([], [], label='Speed', color='blue')[0]
        self.artists['target_speed_line'] = ax.plot([], [], label='Target', color='red', linestyle='--')[0]
        self.artists['speed_delay_text'] = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                                                    va='top', ha='left', fontsize=9,
                                                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        ax.legend(loc='upper right', fontsize=8)
        
        # Power flow artists - bigger and clearer!
        ax = self.axes['power']
        ax.set_title('Power Flow Over Time', fontsize=13, fontweight='bold', pad=10)
        ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Power (kW)', fontsize=12, fontweight='bold')
        ax.tick_params(labelsize=10)
        ax.grid(True, alpha=0.4, linewidth=0.8)
        ax.axhline(y=0, color='black', linestyle='-', linewidth=1.0)

        self.artists['power_stack'] = []
        self.artists['p_batt_chg_area'] = None
        self.artists['p_req_line'] = ax.plot([], [], label='P_req', color=self.config.colors['demand'], linewidth=3.0)[0]
        self.artists['p_supply_line'] = ax.plot([], [], label='Supply', color='magenta', linewidth=2.5)[0]
        self.artists['p_aux_line'] = ax.plot([], [], label='Aux Load', color='gray', linestyle=':', linewidth=2.0)[0]
        self.artists['p_unmet_line'] = ax.plot([], [], label='Unmet', color=self.config.colors['unmet'], linewidth=2.5)[0]
        self.artists['p_dem_line'] = ax.plot([], [], label='P_dem', color='#6a1b9a', linestyle='--', linewidth=2.0)[0]
        self.artists['p_loss_line'] = ax.plot([], [], label='Loss', color='#8d6e63', linestyle='-.', linewidth=1.5)[0]

        from matplotlib.patches import Patch
        from matplotlib.lines import Line2D
        handles = [
            Patch(color=self.config.colors['fc'], alpha=0.6, label='FC'),
            Patch(color=self.config.colors['batt_dis'], alpha=0.6, label='Batt Dis'),
            Patch(color=self.config.colors['batt_chg'], alpha=0.6, label='Batt Chg'),
            Line2D([0], [0], color=self.config.colors['demand'], linewidth=3.0, label='P_req'),
            Line2D([0], [0], color='magenta', linewidth=2.5, label='Supply'),
            Line2D([0], [0], color='gray', linestyle=':', linewidth=2.0, label='Aux Load'),
            Line2D([0], [0], color=self.config.colors['unmet'], linewidth=2.5, label='Unmet'),
            Line2D([0], [0], color='#6a1b9a', linestyle='--', linewidth=2.0, label='P_dem'),
            Line2D([0], [0], color='#8d6e63', linestyle='-.', linewidth=1.5, label='Loss'),
        ]
        ax.legend(handles=handles, loc='upper right', fontsize=10, framealpha=0.9)
        
        # Stores artists
        ax = self.axes['stores']
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Level (%)')
        ax.set_ylim(0, 100)
        ax.grid(True, alpha=0.3)
        # SOC corridor bands
        soc_min = 20.0
        soc_max = 90.0
        ax.axhspan(soc_min, soc_max, alpha=0.2, color=self.config.colors['soc_band'], label='SOC Corridor')
        self.artists['soc_line'] = ax.plot([], [], label='SOC', color='blue', linewidth=2)[0]
        self.artists['tank_line'] = ax.plot([], [], label='H2 Tank', color='green', linewidth=2)[0]
        ax.axhline(y=soc_min, color='orange', linestyle='--', linewidth=1, alpha=0.5)
        ax.axhline(y=soc_max, color='orange', linestyle='--', linewidth=1, alpha=0.5)
        ax.legend(loc='upper right', fontsize=8)
        
        # Cost rate artists
        ax = self.axes['cost']
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Cost Rate (€/s)')
        ax.grid(True, alpha=0.3)
        self.artists['cost_h2'] = ax.fill_between([], [], [], label='H2', color='blue', alpha=0.6)
        self.artists['cost_grid'] = ax.fill_between([], [], [], label='Grid', color='green', alpha=0.6)
        self.artists['cost_delay'] = ax.fill_between([], [], [], label='Delay', color='orange', alpha=0.6)
        self.artists['cost_unmet'] = ax.fill_between([], [], [], label='Unmet', color='red', alpha=0.6)
        self.artists['cost_smooth'] = ax.fill_between([], [], [], label='Smooth', color='purple', alpha=0.6)
        self.artists['reward_line'] = ax.plot([], [], label='Reward', color='black', linewidth=1.5)[0]
        ax.legend(loc='upper right', fontsize=7, ncol=2)
        
        # Power diagram artists
        ax = self.axes['power_diagram']
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.axis('off')
        ax.set_title('Power Flow Diagram', fontsize=10, fontweight='bold')
        
        # Initialize power diagram components (will be updated dynamically)
        self.artists['power_diagram_components'] = {}
        
        # Footer text for shield flags (adjust position if interactive mode)
        footer_y = 0.04 if (self.config.interactive and self._pygame_available) else 0.02
        self.artists['footer_text'] = self.fig.text(0.5, footer_y, '', ha='center', va='bottom',
                                                     fontsize=9, family='monospace')
    
    def _initialize_pygame(self):
        """Initialize pygame window for interactive mode."""
        if self._pygame_initialized or not self._pygame_available:
            return
        
        self.pygame.init()
        # Calculate window size from figure size
        width = int(self.config.figsize[0] * self.config.dpi)
        height = int(self.config.figsize[1] * self.config.dpi) + 60  # Extra space for slider
        self._pygame_screen = self.pygame.display.set_mode((width, height))
        self.pygame.display.set_caption("RL Hybrid Train Environment - Interactive")
        self._pygame_clock = self.pygame.time.Clock()
        self._pygame_initialized = True
    
    def _matplotlib_to_pygame_surface(self):
        """Convert matplotlib figure to pygame surface."""
        if not self._matplotlib_imported or self.fig is None:
            return None
        
        # Render matplotlib figure to buffer
        canvas = self.FigureCanvasAgg(self.fig)
        canvas.draw()
        buf = canvas.buffer_rgba()
        img = np.asarray(buf)
        
        # Convert RGBA to RGB
        rgb = img[:, :, :3]
        
        # Convert numpy array to pygame surface
        # Matplotlib gives us (height, width, 3)
        height, width = rgb.shape[:2]
        rgb_bytes = rgb.tobytes()
        
        # Use frombuffer for better compatibility
        try:
            surface = self.pygame.image.frombuffer(rgb_bytes, (width, height), 'RGB')
        except AttributeError:
            # Fallback for older pygame versions
            surface = self.pygame.image.fromstring(rgb_bytes, (width, height), 'RGB')
        
        return surface
    
    def _handle_pygame_events(self):
        """Handle pygame events for interactive controls."""
        if not self._pygame_initialized:
            return False
        
        for event in self.pygame.event.get():
            if event.type == self.pygame.QUIT:
                return False
            
            # Handle slider dragging
            mouse_x, mouse_y = self.pygame.mouse.get_pos()
            screen_width = self._pygame_screen.get_width()
            screen_height = self._pygame_screen.get_height()
            
            # Slider area: bottom 40 pixels, full width
            slider_y = screen_height - 40
            slider_height = 30
            
            # Pause button area
            button_x = 20
            button_y = slider_y - 25
            button_width = 80
            button_height = 25
            
            if event.type == self.pygame.MOUSEBUTTONDOWN:
                # Check if clicking on pause button
                if button_x <= mouse_x <= button_x + button_width and button_y <= mouse_y <= button_y + button_height:
                    self._paused = not self._paused
                # Check if clicking on slider
                elif slider_y <= mouse_y <= screen_height:
                    self._slider_dragging = True
            
            elif event.type == self.pygame.MOUSEBUTTONUP:
                self._slider_dragging = False
            
            elif event.type == self.pygame.MOUSEMOTION and self._slider_dragging:
                # Update slider position based on mouse x
                slider_x = max(20, min(screen_width - 20, mouse_x))
                self._slider_pos = (slider_x - 20) / (screen_width - 40)
                self._slider_pos = max(0.0, min(1.0, self._slider_pos))
            
            # Keyboard controls
            elif event.type == self.pygame.KEYDOWN:
                if event.key == self.pygame.K_LEFT:
                    self._slider_pos = max(0.0, self._slider_pos - 0.01)
                elif event.key == self.pygame.K_RIGHT:
                    self._slider_pos = min(1.0, self._slider_pos + 0.01)
                elif event.key == self.pygame.K_HOME:
                    self._slider_pos = 0.0
                elif event.key == self.pygame.K_END:
                    self._slider_pos = 1.0
                elif event.key == self.pygame.K_SPACE:
                    # Toggle pause
                    self._paused = not self._paused
        
        return True
    
    def _draw_pygame_slider(self):
        """Draw the time slider and pause button in pygame."""
        if not self._pygame_initialized:
            return
        
        screen_width = self._pygame_screen.get_width()
        screen_height = self._pygame_screen.get_height()
        slider_y = screen_height - 40
        slider_height = 30
        slider_x_start = 20
        slider_x_end = screen_width - 20
        slider_width = slider_x_end - slider_x_start
        
        # Draw slider track
        self.pygame.draw.rect(self._pygame_screen, (100, 100, 100), 
                             (slider_x_start, slider_y + slider_height // 2 - 2, 
                              slider_width, 4))
        
        # Draw slider handle
        handle_x = slider_x_start + int(self._slider_pos * slider_width)
        handle_y = slider_y + slider_height // 2
        self.pygame.draw.circle(self._pygame_screen, (200, 200, 200), 
                               (handle_x, handle_y), 10)
        self.pygame.draw.circle(self._pygame_screen, (50, 50, 50), 
                               (handle_x, handle_y), 10, 2)
        
        # Draw pause button
        button_x = slider_x_start
        button_y = slider_y - 25
        button_width = 80
        button_height = 25
        button_color = (100, 150, 100) if self._paused else (150, 100, 100)
        self.pygame.draw.rect(self._pygame_screen, button_color,
                             (button_x, button_y, button_width, button_height))
        self.pygame.draw.rect(self._pygame_screen, (255, 255, 255),
                             (button_x, button_y, button_width, button_height), 2)
        
        # Draw pause/play text
        font = self.pygame.font.Font(None, 20)
        button_text = "PAUSED" if self._paused else "PAUSE"
        text_surface = font.render(button_text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=(button_x + button_width // 2, 
                                                  button_y + button_height // 2))
        self._pygame_screen.blit(text_surface, text_rect)
        
        # Draw time label
        if len(self._full_buffer) > 0:
            max_time = max(d.get('t', 0.0) for d in self._full_buffer)
            current_time = self._slider_pos * max_time
            time_font = self.pygame.font.Font(None, 24)
            time_text = f"Time: {current_time:.1f}s / {max_time:.1f}s"
            time_surface = time_font.render(time_text, True, (255, 255, 255))
            self._pygame_screen.blit(time_surface, (slider_x_start + button_width + 10, slider_y - 20))
            
            # Instructions
            inst_font = self.pygame.font.Font(None, 18)
            inst_text = "Drag slider | ← → arrows | Home/End | Space: pause | Q: quit"
            inst_surface = inst_font.render(inst_text, True, (200, 200, 200))
            self._pygame_screen.blit(inst_surface, (slider_x_start, slider_y + slider_height + 5))
    
    def keep_alive(self):
        """Keep the pygame window open and handle events after episode ends."""
        if not self.config.interactive or not self._pygame_available or not self._pygame_initialized:
            return
        
        print("\nEpisode ended. Window will stay open for interactive exploration.")
        print("Close the window or press 'q' to exit.")
        
        running = True
        while running:
            # Handle events
            for event in self.pygame.event.get():
                if event.type == self.pygame.QUIT:
                    running = False
                    break
                
                # Handle slider dragging
                mouse_x, mouse_y = self.pygame.mouse.get_pos()
                screen_width = self._pygame_screen.get_width()
                screen_height = self._pygame_screen.get_height()
                
                slider_y = screen_height - 40
                button_x = 20
                button_y = slider_y - 25
                button_width = 80
                button_height = 25
                
                if event.type == self.pygame.MOUSEBUTTONDOWN:
                    # Check if clicking on pause button
                    if button_x <= mouse_x <= button_x + button_width and button_y <= mouse_y <= button_y + button_height:
                        self._paused = not self._paused
                    # Check if clicking on slider
                    elif slider_y <= mouse_y <= screen_height:
                        self._slider_dragging = True
                
                elif event.type == self.pygame.MOUSEBUTTONUP:
                    self._slider_dragging = False
                
                elif event.type == self.pygame.MOUSEMOTION and self._slider_dragging:
                    slider_x = max(20, min(screen_width - 20, mouse_x))
                    self._slider_pos = (slider_x - 20) / (screen_width - 40)
                    self._slider_pos = max(0.0, min(1.0, self._slider_pos))
                
                elif event.type == self.pygame.KEYDOWN:
                    if event.key == self.pygame.K_LEFT:
                        self._slider_pos = max(0.0, self._slider_pos - 0.01)
                    elif event.key == self.pygame.K_RIGHT:
                        self._slider_pos = min(1.0, self._slider_pos + 0.01)
                    elif event.key == self.pygame.K_HOME:
                        self._slider_pos = 0.0
                    elif event.key == self.pygame.K_END:
                        self._slider_pos = 1.0
                    elif event.key == self.pygame.K_SPACE:
                        self._paused = not self._paused
                    elif event.key == self.pygame.K_q:
                        running = False
                        break
            
            # Update and render
            if len(self._full_buffer) > 0:
                max_time = max(d.get('t', 0.0) for d in self._full_buffer)
                selected_time = self._slider_pos * max_time
                display_buffer = deque([d for d in self._full_buffer if d.get('t', 0.0) <= selected_time])
                
                self._update_plots(display_buffer)
                
                # Render to pygame
                plot_surface = self._matplotlib_to_pygame_surface()
                if plot_surface:
                    self._pygame_screen.fill((0, 0, 0))
                    self._pygame_screen.blit(plot_surface, (0, 0))
                    self._draw_pygame_slider()
                    self.pygame.display.flip()
            
            self._pygame_clock.tick(30)
        
        print("Window closed.")
    
    def render_human(self, buffer: Deque):
        """Render to live window (human mode)."""
        if len(buffer) == 0:
            return
        
        # Store full buffer for interactive scrubbing
        if self.config.interactive and self._pygame_available:
            # Update full buffer with new data (unless paused)
            if not self._paused:
                for item in buffer:
                    if len(self._full_buffer) == 0 or item.get('t', 0.0) > self._full_buffer[-1].get('t', 0.0):
                        self._full_buffer.append(item)
                
                # Update slider to max if at end (auto-follow)
                if len(self._full_buffer) > 0:
                    max_time = max(d.get('t', 0.0) for d in self._full_buffer)
                    if self._slider_pos >= 0.99:  # Was near max, keep at max
                        self._slider_pos = 1.0
        
        if not self._initialized:
            self._initialize_figure()
        
        # Initialize pygame if interactive mode
        if self.config.interactive and self._pygame_available and not self._pygame_initialized:
            self._initialize_pygame()
        
        # Handle pygame events if interactive
        if self.config.interactive and self._pygame_available and self._pygame_initialized:
            if not self._handle_pygame_events():
                return  # User closed window
        
        # Use full buffer for interactive mode, otherwise use provided buffer
        display_buffer = self._full_buffer if (self.config.interactive and self._pygame_available and len(self._full_buffer) > 0) else buffer
        
        # If interactive and pygame available, filter by slider position
        if self.config.interactive and self._pygame_available and len(self._full_buffer) > 0:
            max_time = max(d.get('t', 0.0) for d in self._full_buffer)
            selected_time = self._slider_pos * max_time
            display_buffer = deque([d for d in self._full_buffer if d.get('t', 0.0) <= selected_time])
        
        self._update_plots(display_buffer)
        
        # Render to pygame if interactive, otherwise save to file
        if self.config.interactive and self._pygame_available and self._pygame_initialized:
            # Convert matplotlib figure to pygame surface
            plot_surface = self._matplotlib_to_pygame_surface()
            if plot_surface:
                # Clear screen
                self._pygame_screen.fill((0, 0, 0))
                # Blit plot surface
                self._pygame_screen.blit(plot_surface, (0, 0))
                # Draw slider
                self._draw_pygame_slider()
                # Update display
                self.pygame.display.flip()
                # Limit framerate
                self._pygame_clock.tick(30)
        else:
            # Non-interactive: save to file
            self.fig.canvas.draw()
            output_file = "env0_render.png"
            try:
                self.fig.savefig(output_file, dpi=self.config.dpi, bbox_inches='tight')
                if not hasattr(self, '_saved_message_shown'):
                    print(f"Figure saved to {output_file} (refresh file to see updates)")
                    self._saved_message_shown = True
            except PermissionError:
                if not hasattr(self, '_save_error_shown'):
                    print(f"Warning: cannot write {output_file} (permission denied); continuing without saving frames.")
                    self._save_error_shown = True
    
    def render_rgb_array(self, buffer: Deque) -> np.ndarray:
        """Render to RGB array (for video recording)."""
        if len(buffer) == 0:
            # Return blank frame
            return np.zeros((int(self.config.figsize[1] * self.config.dpi),
                           int(self.config.figsize[0] * self.config.dpi), 3), dtype=np.uint8)
        
        if not self._initialized:
            self._initialize_figure()
        
        self._update_plots(buffer)
        
        # Convert to RGB array
        canvas = self.FigureCanvasAgg(self.fig)
        canvas.draw()
        buf = canvas.buffer_rgba()
        img = np.asarray(buf)
        
        # Convert RGBA to RGB
        rgb = img[:, :, :3]
        
        return rgb.astype(np.uint8)
    
    def render_ansi(self, data: Dict) -> str:
        """Render single-line ANSI text output."""
        if not data:
            return ""
        
        # Format power values
        p_req = data.get('p_req_kw', 0.0)
        p_fc = data.get('p_fc_kw', 0.0)
        p_batt = data.get('p_batt_kw', 0.0)
        p_batt_dis = max(0.0, p_batt)
        p_batt_chg = max(0.0, -p_batt)
        
        # Format speeds
        speed = data.get('speed_mps', 0.0)
        target_speed = data.get('target_speed_mps', 0.0)
        
        # Format stores
        soc = data.get('soc', 0.0) * 100.0
        tank = data.get('tank_level_norm', 0.0) * 100.0
        
        # Format costs
        cost_h2 = data.get('cost_h2_eur', 0.0)
        cost_grid = data.get('cost_grid_eur', 0.0)
        cost_delay = data.get('penalty_delay_eur', 0.0)
        cost_unmet = data.get('penalty_unmet_eur', 0.0)
        cost_smooth = data.get('penalty_smooth_eur', 0.0)
        reward = data.get('r_step', 0.0)
        
        # Format shield flags
        flags = []
        if data.get('soc_low', False):
            flags.append('SOC_LOW')
        if data.get('soc_high', False):
            flags.append('SOC_HIGH')
        if data.get('fc_ramp_limited', False):
            flags.append('FC_RAMP')
        if data.get('crate_capped', False):
            flags.append('C_RATE')
        if data.get('regen_clipped', False):
            flags.append('REGEN')
        flags_str = ' '.join(flags) if flags else '[      ]'
        
        t = data.get('t', 0.0)
        delay = data.get('delay_seconds', 0.0)
        delay_str = f"+{delay:.1f}s" if delay > 0 else f"{delay:.1f}s" if delay < 0 else "on-time"
        
        line = (
            f"t={t:6.1f}s  v={speed:4.1f}/{target_speed:4.1f} m/s  "
            f"P:req {p_req:5.0f}  FC {p_fc:4.0f}  Batt +{p_batt_dis:4.0f}/-{p_batt_chg:4.0f} kW  "
            f"SOC {soc:3.0f}%  H2 {tank:3.0f}%  "
            f"Δ€/s: H2 {cost_h2:5.2f}  Grid {cost_grid:5.2f}  Delay {cost_delay:4.1f}  "
            f"Unmet {cost_unmet:5.2f}  Smooth {cost_smooth:5.2f}  | r={reward:6.2f}  "
            f"Flags: {flags_str}"
        )
        
        return line
    
    
    def _update_plots(self, buffer: Deque, max_time: Optional[float] = None):
        """Update all plots with buffer data."""
        if len(buffer) == 0:
            return
        
        # Convert buffer to arrays
        data_list = list(buffer)
        
        # Apply time filter if specified
        if max_time is not None:
            data_list = [d for d in data_list if d.get('t', 0.0) <= max_time]
        
        if len(data_list) == 0:
            return
        
        t = np.array([d['t'] for d in data_list])
        
        if len(t) == 0:
            return
        
        # Timeline update
        if len(data_list) > 0:
            latest = data_list[-1]
            episode_step = latest.get('episode_step', 0)
            episode_length = latest.get('episode_length', 1)
            progress = episode_step / max(1, episode_length)
            current_time = latest.get('t', 0.0)
            delay = latest.get('delay_seconds', 0.0)
            distance = latest.get('distance_km', 0.0)
            
            # Calculate cumulative cost per km
            total_cost = sum(d.get('cost_h2_eur', 0.0) + d.get('cost_grid_eur', 0.0) +
                           d.get('penalty_delay_eur', 0.0) + d.get('penalty_unmet_eur', 0.0) +
                           d.get('penalty_smooth_eur', 0.0) for d in data_list)
            cost_per_km = total_cost / max(0.001, distance) if distance > 0 else 0.0
            
            ax = self.axes['timeline']
            self.artists['timeline_progress'].set_width(progress)
            timeline_text = (
                f"Time: {current_time:.0f}s  "
                f"Delay: {delay:+.1f}s  "
                f"€/km: {cost_per_km:.2f}"
            )
            self.artists['timeline_text'].set_text(timeline_text)
        
        # Speed tracking update
        speed = np.array([d.get('speed_mps', 0.0) for d in data_list])
        target_speed = np.array([d.get('target_speed_mps', 0.0) for d in data_list])
        at_stop = np.array([d.get('at_stop', False) for d in data_list])
        
        ax = self.axes['speed']
        self.artists['speed_line'].set_data(t, speed)
        self.artists['target_speed_line'].set_data(t, target_speed)
        if len(t) > 0:
            ax.set_xlim(t[0], t[-1])
            if len(speed) > 0:
                ax.set_ylim(0, max(50.0, max(speed) * 1.1))
            else:
                ax.set_ylim(0, 50.0)
        else:
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 50.0)
        
        # Shade dwell periods (clear old patches first)
        for patch in ax.patches:
            if patch.get_alpha() == 0.2 and patch.get_facecolor()[0] == 0.5:  # Gray patches
                try:
                    patch.remove()
                except:
                    pass
        
        if len(at_stop) > 1:
            for i in range(len(at_stop) - 1):
                if at_stop[i]:
                    ax.axvspan(t[i], t[min(i+1, len(t)-1)], alpha=0.2, color='gray')
        
        delay_val = data_list[-1].get('delay_seconds', 0.0) if len(data_list) > 0 else 0.0
        delay_str = f"+{delay_val:.1f}s" if delay_val > 0 else f"{delay_val:.1f}s" if delay_val < 0 else "on-time"
        self.artists['speed_delay_text'].set_text(f"Delay: {delay_str}")
        
        # Power flow update
        p_fc = np.array([d.get('p_fc_kw', 0.0) for d in data_list])
        p_fc_pos = np.clip(p_fc, 0.0, None)
        p_batt_dis = np.array([
            d.get('p_batt_discharge_kw', max(d.get('p_batt_kw', 0.0), 0.0))
            for d in data_list
        ])
        p_batt_bus = np.array([
            d.get(
                'p_batt_delivered_kw',
                d.get('p_batt_discharge_kw', max(d.get('p_batt_kw', 0.0), 0.0))
            )
            for d in data_list
        ])
        p_batt_chg = np.array([
            d.get('p_batt_charge_kw', max(-d.get('p_batt_kw', 0.0), 0.0))
            for d in data_list
        ])
        p_req = np.array([d.get('p_req_kw', 0.0) for d in data_list])
        p_dem = np.array([
            d.get('p_dem_kw', d.get('p_req_kw', 0.0) + d.get('p_aux_kw', 0.0))
            for d in data_list
        ])
        p_unmet = np.array([d.get('p_unmet_kw', 0.0) for d in data_list])
        p_aux = np.array([d.get('p_aux_kw', 0.0) for d in data_list])
        p_loss = np.array([d.get('p_loss_kw', 0.0) for d in data_list])
        p_supply = np.array([
            d.get(
                'p_supply_kw',
                d.get('p_fc_kw', 0.0) + d.get(
                    'p_batt_delivered_kw',
                    d.get('p_batt_discharge_kw', max(d.get('p_batt_kw', 0.0), 0.0))
                )
            )
            for d in data_list
        ])

        ax = self.axes['power']

        for artist in self.artists.get('power_stack', []):
            try:
                artist.remove()
            except Exception:
                pass
        self.artists['power_stack'] = []

        if self.artists.get('p_batt_chg_area') is not None:
            try:
                self.artists['p_batt_chg_area'].remove()
            except Exception:
                pass
            self.artists['p_batt_chg_area'] = None

        if len(t) > 0:
            stack_artists = ax.stackplot(
                t,
                p_fc_pos,
                p_batt_bus,
                colors=[self.config.colors['fc'], self.config.colors['batt_dis']],
                alpha=0.7,  # Increased for better visibility
            )
            self.artists['power_stack'] = stack_artists

            if np.any(p_batt_chg > 0):
                self.artists['p_batt_chg_area'] = ax.fill_between(
                    t,
                    0,
                    -p_batt_chg,
                    color=self.config.colors['batt_chg'],
                    alpha=0.7,  # Increased for better visibility
                )

        self.artists['p_req_line'].set_data(t, p_req)
        self.artists['p_supply_line'].set_data(t, p_supply)
        self.artists['p_dem_line'].set_data(t, p_dem)
        self.artists['p_aux_line'].set_data(t, p_aux)
        self.artists['p_unmet_line'].set_data(t, p_unmet)
        self.artists['p_loss_line'].set_data(t, p_loss)

        if len(t) > 0:
            ax.set_xlim(t[0], t[-1])
            p_max = max(np.max(p_supply), np.max(p_req), np.max(p_dem), np.max(p_aux), np.max(p_loss), 10.0)
            p_min_candidates = [
                -np.max(p_batt_chg) if np.size(p_batt_chg) else -10.0,
                np.min(p_req),
                -10.0,
            ]
            p_min = min(p_min_candidates)
            ax.set_ylim(p_min * 1.1, p_max * 1.1)
        else:
            ax.set_xlim(0, 1)
            ax.set_ylim(-50, 100)
        
        # Stores update
        soc = np.array([d.get('soc', 0.0) * 100.0 for d in data_list])
        tank = np.array([d.get('tank_level_norm', 0.0) * 100.0 for d in data_list])
        
        ax = self.axes['stores']
        self.artists['soc_line'].set_data(t, soc)
        self.artists['tank_line'].set_data(t, tank)
        if len(t) > 0:
            ax.set_xlim(t[0], t[-1])
        else:
            ax.set_xlim(0, 1)
        
        # Cost rate update
        cost_h2 = np.array([d.get('cost_h2_eur', 0.0) for d in data_list])
        cost_grid = np.array([d.get('cost_grid_eur', 0.0) for d in data_list])
        cost_delay = np.array([d.get('penalty_delay_eur', 0.0) for d in data_list])
        cost_unmet = np.array([d.get('penalty_unmet_eur', 0.0) for d in data_list])
        cost_smooth = np.array([d.get('penalty_smooth_eur', 0.0) for d in data_list])
        reward = np.array([d.get('r_step', 0.0) for d in data_list])
        
        ax = self.axes['cost']
        # Clear fill_between areas and recreate (they can't be updated directly)
        for key in ['cost_h2', 'cost_grid', 'cost_delay', 'cost_unmet', 'cost_smooth']:
            if key in self.artists:
                try:
                    self.artists[key].remove()
                except:
                    pass
        
        if len(t) > 0:
            bottom = np.zeros_like(cost_h2)
            self.artists['cost_h2'] = ax.fill_between(t, bottom, cost_h2, color='blue', alpha=0.6, label='H2')
            bottom += cost_h2
            self.artists['cost_grid'] = ax.fill_between(t, bottom, bottom + cost_grid, color='green', alpha=0.6, label='Grid')
            bottom += cost_grid
            self.artists['cost_delay'] = ax.fill_between(t, bottom, bottom + cost_delay, color='orange', alpha=0.6, label='Delay')
            bottom += cost_delay
            self.artists['cost_unmet'] = ax.fill_between(t, bottom, bottom + cost_unmet, color='red', alpha=0.6, label='Unmet')
            bottom += cost_unmet
            self.artists['cost_smooth'] = ax.fill_between(t, bottom, bottom + cost_smooth, color='purple', alpha=0.6, label='Smooth')
        
        self.artists['reward_line'].set_data(t, reward)
        ax.set_xlim(t[0] if len(t) > 0 else 0, t[-1] if len(t) > 0 else 1)
        if len(cost_h2) > 0:
            bottom = cost_h2 + cost_grid + cost_delay + cost_unmet + cost_smooth
            cost_max = max(np.max(bottom), np.max(reward), 0.1)
            cost_min = min(np.min(reward), -0.1)
            ax.set_ylim(cost_min * 1.1, cost_max * 1.1)
        else:
            ax.set_ylim(-0.1, 0.1)
        
        # Power diagram update
        if len(data_list) > 0:
            latest = data_list[-1]
            p_fc = max(latest.get('p_fc_kw', 0.0), 0.0)
            p_batt_kw = latest.get('p_batt_kw', 0.0)
            p_batt_dis = latest.get('p_batt_discharge_kw', max(p_batt_kw, 0.0))
            p_batt_chg = latest.get('p_batt_charge_kw', max(-p_batt_kw, 0.0))
            p_batt_delivered = latest.get('p_batt_delivered_kw', p_batt_dis)
            p_req = latest.get('p_req_kw', 0.0)
            p_dem = latest.get('p_dem_kw', p_req + latest.get('p_aux_kw', 0.0))
            p_aux = latest.get('p_aux_kw', 0.0)
            p_unmet = latest.get('p_unmet_kw', 0.0)
            p_loss = latest.get('p_loss_kw', 0.0)
            p_supply = latest.get('p_supply_kw', p_fc + p_batt_delivered)
            p_charge_regen = latest.get('p_batt_charge_regen_kw', 0.0)
            p_charge_fc = latest.get('p_batt_charge_fc_kw', 0.0)
            p_regen_post_aux = latest.get('p_regen_post_aux_kw', 0.0)
            
            ax = self.axes['power_diagram']
            
            for key in list(self.artists.get('power_diagram_components', {}).keys()):
                try:
                    comp = self.artists['power_diagram_components'][key]
                    if isinstance(comp, list):
                        for item in comp:
                            try:
                                item.remove()
                            except Exception:
                                pass
                    else:
                        comp.remove()
                except Exception:
                    pass
            
            ax.clear()
            ax.set_xlim(0, 10)
            ax.set_ylim(0, 11)
            ax.axis('off')
            ax.set_title('Power Flow Diagram', fontsize=12, fontweight='bold', pad=10)
            self.artists['power_diagram_components'] = {}
            
            from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
            
            max_power = max(
                p_fc,
                p_batt_dis,
                p_batt_chg,
                abs(p_req),
                p_aux,
                p_unmet,
                p_loss,
                1.0,
            )
            arrow_scale = lambda p: max(1.5, min(8.0, 1.5 + 6.5 * (p / max_power)))
            
            demand_y = 8.4
            aux_y = 6.3
            loss_y = 3.8
            unmet_y = 1.4
            box_width = 2.0
            box_height = 1.5
            
            fc_box = FancyBboxPatch(
                (0.5, 8.0),
                box_width,
                box_height,
                boxstyle="round,pad=0.2",
                facecolor=self.config.colors['fc'],
                edgecolor='black',
                linewidth=2.5,
                alpha=0.8,
            )
            ax.add_patch(fc_box)
            ax.text(1.5, 8.85, 'Fuel Cell', ha='center', va='center', fontsize=11, fontweight='bold')
            ax.text(1.5, 8.25, f'{p_fc:.1f} kW', ha='center', va='center', fontsize=10, fontweight='bold')
            
            batt_color = self.config.colors['batt_dis'] if p_batt_dis > 0.05 else self.config.colors['batt_chg']
            batt_box = FancyBboxPatch(
                (0.5, 4.8),
                box_width,
                box_height,
                boxstyle="round,pad=0.2",
                facecolor=batt_color,
                edgecolor='black',
                linewidth=2.5,
                alpha=0.8,
            )
            ax.add_patch(batt_box)
            ax.text(1.5, 5.55, 'Battery', ha='center', va='center', fontsize=11, fontweight='bold')
            if p_batt_dis > 0.05:
                ax.text(1.5, 4.95, f'+{p_batt_dis:.1f} kW', ha='center', va='center', fontsize=10, fontweight='bold')
            elif p_batt_chg > 0.05:
                ax.text(1.5, 4.95, f'-{p_batt_chg:.1f} kW', ha='center', va='center', fontsize=10, fontweight='bold')
            else:
                ax.text(1.5, 4.95, '0.0 kW', ha='center', va='center', fontsize=10)
            
            charge_notes = []
            if p_charge_regen > 0.05:
                charge_notes.append(f"regen {p_charge_regen:.0f} kW")
            if p_charge_fc > 0.05:
                charge_notes.append(f"FC {p_charge_fc:.0f} kW")
            if charge_notes:
                ax.text(
                    1.5,
                    4.35,
                    "Charge: " + " | ".join(charge_notes),
                    ha='center',
                    va='top',
                    fontsize=9,
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.9),
                )
            
            bus_line = Rectangle((4.0, 3.0), 0.6, 6.5, facecolor='#FFD700', edgecolor='black', linewidth=3.5, alpha=0.6)
            ax.add_patch(bus_line)
            ax.text(
                4.3,
                6.25,
                'POWER\nBUS',
                ha='center',
                va='center',
                fontsize=10,
                fontweight='bold',
                rotation=90,
                bbox=dict(boxstyle='round,pad=0.4', facecolor='white', alpha=0.9),
            )
            summary = f"Supply {p_supply:.0f} kW\nDemand {p_dem:.0f} kW"
            if p_unmet > 0.1:
                summary += f"\nUnmet {p_unmet:.0f} kW"
            ax.text(
                4.25,
                9.6,
                summary,
                ha='center',
                va='top',
                fontsize=10,
                bbox=dict(boxstyle='round,pad=0.4', facecolor='white', alpha=0.85),
            )
            
            demand_box = FancyBboxPatch(
                (7.5, demand_y - box_height / 2),
                box_width,
                box_height,
                boxstyle="round,pad=0.2",
                facecolor=self.config.colors['demand'],
                edgecolor='black',
                linewidth=2.5,
                alpha=0.85,
            )
            ax.add_patch(demand_box)
            demand_label = 'Traction' if p_req >= 0 else 'Regen'
            ax.text(8.5, demand_y + 0.25, demand_label, ha='center', va='center', fontsize=11, fontweight='bold')
            ax.text(8.5, demand_y - 0.25, f'{p_req:.1f} kW', ha='center', va='center', fontsize=10, fontweight='bold')
            if p_req < -0.1 and p_regen_post_aux > 0.1:
                ax.text(
                    8.5,
                    demand_y - 0.75,
                    f"regen avail {p_regen_post_aux:.0f} kW",
                    ha='center',
                    va='center',
                    fontsize=9,
                    color='green',
                )
            
            aux_box = FancyBboxPatch(
                (7.5, aux_y - box_height / 2),
                box_width,
                box_height,
                boxstyle="round,pad=0.2",
                facecolor='#808080',
                edgecolor='black',
                linewidth=2.5,
                alpha=0.85,
            )
            ax.add_patch(aux_box)
            ax.text(8.5, aux_y + 0.25, 'Auxiliary', ha='center', va='center', fontsize=11, fontweight='bold')
            ax.text(8.5, aux_y - 0.25, f'{p_aux:.1f} kW', ha='center', va='center', fontsize=10, fontweight='bold')
            
            loss_box = FancyBboxPatch(
                (7.5, loss_y - box_height / 2),
                box_width,
                box_height,
                boxstyle="round,pad=0.2",
                facecolor='#bdb09f',
                edgecolor='#8d6e63',
                linewidth=2.3,
                alpha=0.85,
            )
            ax.add_patch(loss_box)
            ax.text(8.5, loss_y + 0.25, 'Losses', ha='center', va='center', fontsize=11, fontweight='bold')
            ax.text(8.5, loss_y - 0.25, f'{p_loss:.1f} kW', ha='center', va='center', fontsize=10, fontweight='bold')
            
            unmet_color = self.config.colors['unmet'] if p_unmet > 0.1 else '#CCCCCC'
            unmet_box = FancyBboxPatch(
                (7.5, unmet_y - box_height / 2),
                box_width,
                box_height,
                boxstyle="round,pad=0.2",
                facecolor=unmet_color,
                edgecolor='red' if p_unmet > 0.1 else 'gray',
                linewidth=2.3 if p_unmet > 0.1 else 1.5,
                alpha=0.9,
            )
            ax.add_patch(unmet_box)
            ax.text(8.5, unmet_y + 0.25, 'Unmet', ha='center', va='center', fontsize=11, fontweight='bold')
            ax.text(
                8.5,
                unmet_y - 0.25,
                f'{p_unmet:.1f} kW',
                ha='center',
                va='center',
                fontsize=10,
                fontweight='bold' if p_unmet > 0.1 else 'normal',
            )
            
            components = {
                'fc_box': fc_box,
                'batt_box': batt_box,
                'bus_line': bus_line,
                'demand_box': demand_box,
                'aux_box': aux_box,
                'loss_box': loss_box,
                'unmet_box': unmet_box,
                'arrows': [],
                'texts': [],
            }
            
            if p_fc > 0.05:
                arrow_fc = FancyArrowPatch(
                    (2.5, 8.5),
                    (4.0, 8.5),
                    arrowstyle='->',
                    lw=arrow_scale(p_fc),
                    color=self.config.colors['fc'],
                    alpha=0.9,
                    zorder=10,
                )
                ax.add_patch(arrow_fc)
                components['arrows'].append(arrow_fc)
                components['texts'].append(
                    ax.text(
                        3.25,
                        9.0,
                        f'{p_fc:.1f} kW',
                        ha='center',
                        va='bottom',
                        fontsize=9,
                        fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=self.config.colors['fc'], alpha=0.95),
                    )
                )
            
            if p_charge_fc > 0.05:
                arrow_fc_batt = FancyArrowPatch(
                    (1.0, 8.0),
                    (1.0, 6.3),
                    arrowstyle='->',
                    lw=arrow_scale(p_charge_fc),
                    linestyle='--',
                    color=self.config.colors['fc'],
                    alpha=0.8,
                    zorder=10,
                )
                ax.add_patch(arrow_fc_batt)
                components['arrows'].append(arrow_fc_batt)
            
            if p_batt_dis > 0.05:
                arrow_batt_dis = FancyArrowPatch(
                    (2.5, 5.55),
                    (4.0, 5.55),
                    arrowstyle='->',
                    lw=arrow_scale(p_batt_dis),
                    color=self.config.colors['batt_dis'],
                    alpha=0.9,
                    zorder=10,
                )
                ax.add_patch(arrow_batt_dis)
                components['arrows'].append(arrow_batt_dis)
                components['texts'].append(
                    ax.text(
                        3.25,
                        6.0,
                        f'+{p_batt_dis:.1f} kW',
                        ha='center',
                        va='bottom',
                        fontsize=9,
                        fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=self.config.colors['batt_dis'], alpha=0.95),
                    )
                )
            elif p_batt_chg > 0.05:
                arrow_batt_chg = FancyArrowPatch(
                    (4.0, 5.55),
                    (2.5, 5.55),
                    arrowstyle='->',
                    lw=arrow_scale(p_batt_chg),
                    color=self.config.colors['batt_chg'],
                    alpha=0.9,
                    zorder=10,
                )
                ax.add_patch(arrow_batt_chg)
                components['arrows'].append(arrow_batt_chg)
                components['texts'].append(
                    ax.text(
                        3.25,
                        6.0,
                        f'-{p_batt_chg:.1f} kW',
                        ha='center',
                        va='bottom',
                        fontsize=9,
                        fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=self.config.colors['batt_chg'], alpha=0.95),
                    )
                )
            
            if abs(p_req) > 0.05:
                if p_req >= 0:
                    arrow_demand = FancyArrowPatch(
                        (4.6, demand_y),
                        (7.5, demand_y),
                        arrowstyle='->',
                        lw=arrow_scale(abs(p_req)),
                        color=self.config.colors['demand'],
                        alpha=0.9,
                        zorder=10,
                    )
                else:
                    arrow_demand = FancyArrowPatch(
                        (7.5, demand_y),
                        (4.6, demand_y),
                        arrowstyle='->',
                        lw=arrow_scale(abs(p_req)),
                        color='green',
                        alpha=0.9,
                        zorder=10,
                    )
                ax.add_patch(arrow_demand)
                components['arrows'].append(arrow_demand)
            
            if p_aux > 0.05:
                arrow_aux = FancyArrowPatch(
                    (4.6, aux_y),
                    (7.5, aux_y),
                    arrowstyle='->',
                    lw=arrow_scale(p_aux),
                    color='#606060',
                    alpha=0.9,
                    zorder=10,
                )
                ax.add_patch(arrow_aux)
                components['arrows'].append(arrow_aux)
            
            if p_loss > 0.05:
                arrow_loss = FancyArrowPatch(
                    (4.6, loss_y),
                    (7.5, loss_y),
                    arrowstyle='->',
                    lw=arrow_scale(p_loss),
                    color='#8d6e63',
                    alpha=0.9,
                    zorder=10,
                )
                ax.add_patch(arrow_loss)
                components['arrows'].append(arrow_loss)
            
            if p_unmet > 0.05:
                arrow_unmet = FancyArrowPatch(
                    (4.6, unmet_y),
                    (7.5, unmet_y),
                    arrowstyle='->',
                    lw=arrow_scale(p_unmet),
                    color='red',
                    alpha=0.9,
                    zorder=10,
                )
                ax.add_patch(arrow_unmet)
                components['arrows'].append(arrow_unmet)
            
            self.artists['power_diagram_components'] = components
        
        # Footer: shield flags
        if len(data_list) > 0:
            latest = data_list[-1]
            flags = []
            if latest.get('soc_low', False):
                flags.append('SOC_LOW')
            if latest.get('soc_high', False):
                flags.append('SOC_HIGH')
            if latest.get('fc_ramp_limited', False):
                flags.append('FC_RAMP')
            if latest.get('crate_capped', False):
                flags.append('C_RATE')
            if latest.get('regen_clipped', False):
                flags.append('REGEN_CLIP')
            
            flags_str = ' | '.join(flags) if flags else 'No active constraints'
            footer_text = f"Shield Flags: {flags_str}"
            
            if self.config.show_hidden_debug:
                mass = latest.get('passenger_mass_t', 0.0)
                aux_bias = latest.get('aux_bias_kw', 0.0)
                footer_text += f" | Mass: {mass:.1f}t | Aux Bias: {aux_bias:.2f}kW"
            
            self.artists['footer_text'].set_text(footer_text)
    
    def close(self):
        """Clean up matplotlib and pygame resources."""
        if self.fig is not None and self._matplotlib_imported:
            self.plt.close(self.fig)
            self.fig = None
            self._initialized = False
        
        if self._pygame_initialized and self._pygame_available:
            self.pygame.quit()
            self._pygame_initialized = False
        
        self._full_buffer.clear()
        self._slider = None
