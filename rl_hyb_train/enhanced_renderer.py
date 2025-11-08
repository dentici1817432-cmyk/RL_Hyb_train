"""
Enhanced Renderer for RL Hybrid Train Environment.

This module extends the existing Renderer with advanced features including:
- 3D energy flow visualization with animated power flows
- Smart performance dashboard with real-time KPIs
- Intelligent recommendations engine
- Environmental impact metrics
- Comparative analysis capabilities
- Advanced interactive controls
"""

import numpy as np
import math
from typing import Dict, List, Optional, Deque, Tuple, Any
from collections import deque
import warnings

from .config import RendererConfig
from .renderer import Renderer, RenderBuffer


class EnhancedRenderer(Renderer):
    """Enhanced renderer with 3D visualization and smart analytics."""
    
    def __init__(self, config: RendererConfig):
        super().__init__(config)
        self._performance_history = deque(maxlen=3600)  # Store 1 hour of data
        self._efficiency_scores = []
        self._co2_emissions_kg = 0.0
        self._energy_balance_score = 0.0
        self._recommendations = []
        self._anomaly_detector = AnomalyDetector()
        self._comparative_buffer = []  # For multi-episode comparison
        self._enhanced_3d_enabled = True
        self._animation_enabled = True
        self._smart_dashboard_enabled = True
        self._environmental_impact_enabled = True
        
        # Import matplotlib 3D capabilities
        self._import_matplotlib_3d()
        
        # 3D animation state
        self._animation_step = 0
        self._particle_systems = {}
        
        # Performance tracking
        self._episode_start_time = None
        self._cumulative_metrics = {
            'total_energy_kwh': 0.0,
            'total_cost_eur': 0.0,
            'total_h2_kg': 0.0,
            'total_distance_km': 0.0,
            'efficiency_score': 0.0,
            'co2_emissions_kg': 0.0,
        }
    
    def _import_matplotlib_3d(self):
        """Import matplotlib 3D capabilities."""
        try:
            from mpl_toolkits.mplot3d import Axes3D
            from mpl_toolkits.mplot3d.art3d import Line3DCollection
            self.mplot3d = __import__('mpl_toolkits.mplot3d', fromlist=['Axes3D'])
            self._3d_available = True
        except ImportError:
            self._3d_available = False
            if not hasattr(self, '_3d_warning_shown'):
                print("Warning: matplotlib 3D not available. Some enhanced features will be disabled.")
                self._3d_warning_shown = True
    
    def _initialize_enhanced_figure(self):
        """Initialize enhanced figure with new panels and 3D visualization."""
        if not self._initialized:
            # Call parent initialization first
            super()._initialize_figure()
            
            # Add new enhanced panels
            self._add_smart_dashboard_panel()
            self._add_3d_energy_flow_panel()
            self._add_environmental_impact_panel()
            self._add_recommendations_panel()
            
            self._enhanced_initialized = True
    
    def _add_smart_dashboard_panel(self):
        """Add clean smart performance dashboard panel."""
        if not self._matplotlib_imported:
            return
            
        # ONLY add to timeline - clean and simple
        ax_timeline = self.axes.get('timeline')
        if ax_timeline is not None:
            # Add ONE clean text element in the timeline area for key metrics
            self.artists['smart_info_text'] = ax_timeline.text(0.02, 0.85, '', 
                                                              transform=ax_timeline.transAxes,
                                                              fontsize=9, va='top', ha='left',
                                                              bbox=dict(boxstyle='round,pad=0.4', 
                                                                       facecolor='lightgreen', alpha=0.8,
                                                                       edgecolor='darkgreen', linewidth=1))
    
    def _add_3d_energy_flow_panel(self):
        """Add enhanced power flow visualization panel."""
        # Keep the old power_diagram for compatibility, but make it enhanced
        if 'power_diagram' in self.axes:
            ax_power = self.axes['power_diagram']
            # Add a text annotation for enhanced info
            self.artists['enhanced_power_info'] = ax_power.text(0.02, 0.98, '', 
                                                               transform=ax_power.transAxes,
                                                               fontsize=8, va='top', ha='left',
                                                               bbox=dict(boxstyle='round,pad=0.3', 
                                                                        facecolor='lightyellow', alpha=0.8))
            
            self._enhanced_power_enabled = True
    
    def _add_environmental_impact_panel(self):
        """Add environmental impact visualization panel."""
        # This will be handled by the environmental_impact axes we created above
        pass
    
    def _add_recommendations_panel(self):
        """Add recommendations panel."""
        # This will be handled by the recommendations axes we created above
        pass
    
    def _update_enhanced_plots(self, buffer: Deque, max_time: Optional[float] = None):
        """Update enhanced plots with new features."""
        if len(buffer) == 0:
            return
        
        # Call parent update first
        super()._update_plots(buffer, max_time)
        
        # Update our enhanced features (only if they exist)
        self._update_smart_dashboard(buffer)
        self._update_enhanced_power_flow(buffer)
        self._update_environmental_impact(buffer)
        self._update_recommendations(buffer)
    
    def _update_smart_dashboard(self, buffer: Deque):
        """Update clean smart performance dashboard with key metrics."""
        if not self._smart_dashboard_enabled or len(buffer) == 0:
            return
        
        data_list = list(buffer)
        latest = data_list[-1]
        
        # Calculate real-time KPIs
        kpis = self._calculate_performance_kpis(data_list)
        
        # Update ONLY the timeline smart info text - clean and simple
        artist = self.artists.get('smart_info_text')
        if artist is not None:
            eff_score = kpis['efficiency_score']
            cost_per_km = kpis['cost_per_km']
            delay = latest.get('delay_seconds', 0.0)
            
            # Create a clean, concise summary
            smart_text = f"Smart Insights:\n"
            smart_text += f"• Efficiency: {eff_score:.1f}%\n"
            smart_text += f"• Cost/km: €{cost_per_km:.2f}\n"
            smart_text += f"• Schedule: {delay:+.1f}s"
            
            artist.set_text(smart_text)
        
        # Update performance history
        self._performance_history.append(kpis)
    
    def _update_enhanced_power_flow(self, buffer: Deque):
        """Update enhanced power flow information."""
        if not getattr(self, '_enhanced_power_enabled', False) or len(buffer) == 0:
            return
        
        data_list = list(buffer)
        latest = data_list[-1]
        
        # Update the enhanced power info text
        artist = self.artists.get('enhanced_power_info')
        if artist is not None:
            p_fc = max(latest.get('p_fc_kw', 0.0), 0.0)
            p_batt = latest.get('p_batt_kw', 0.0)
            p_req = latest.get('p_req_kw', 0.0)
            p_unmet = latest.get('p_unmet_kw', 0.0)
            
            # Calculate power split efficiency
            total_supply = p_fc + abs(p_batt)
            if total_supply > 0:
                fc_efficiency = p_fc / total_supply
                batt_efficiency = abs(p_batt) / total_supply
                power_info = f"Power Split:\nFC: {fc_efficiency:.1%}\nBatt: {batt_efficiency:.1%}\nUnmet: {p_unmet:.1f}kW"
            else:
                power_info = f"Power Split:\nFC: 0%\nBatt: 0%\nUnmet: {p_unmet:.1f}kW"
            
            artist.set_text(power_info)
    

    
    def _update_3d_energy_flow(self, buffer: Deque):
        """Update 3D energy flow visualization."""
        if not self._3d_enabled or not self._3d_available or len(buffer) == 0:
            return
        
        data_list = list(buffer)
        latest = data_list[-1]
        
        ax = self.axes.get('3d_energy_flow')
        if ax is None:
            return
        
        # Clear previous components
        for artist in self.artists.get('3d_components', {}).values():
            try:
                if isinstance(artist, list):
                    for item in artist:
                        item.remove()
                else:
                    artist.remove()
            except:
                pass
        
        self.artists['3d_components'] = {}
        
        # 3D positions for components
        components = {
            'fuel_cell': (2, 5, 8),
            'battery': (2, 5, 5),
            'power_bus': (5, 5, 6.5),
            'traction': (8, 7, 7),
            'auxiliary': (8, 5, 5),
            'loss': (8, 3, 3),
            'unmet': (8, 1, 1)
        }
        
        # Get current power values
        p_fc = max(latest.get('p_fc_kw', 0.0), 0.0)
        p_batt = latest.get('p_batt_kw', 0.0)
        p_batt_dis = max(p_batt, 0.0)
        p_batt_chg = max(-p_batt, 0.0)
        p_req = latest.get('p_req_kw', 0.0)
        p_aux = latest.get('p_aux_kw', 0.0)
        p_unmet = latest.get('p_unmet_kw', 0.0)
        p_loss = latest.get('p_loss_kw', 0.0)
        
        # Draw 3D components (boxes)
        from matplotlib.patches import Rectangle3D
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        for name, (x, y, z) in components.items():
            if name == 'fuel_cell' and p_fc > 0.1:
                # 3D box for fuel cell
                box = Rectangle3D([x-0.3, y-0.3, z-0.2], [x+0.3, y+0.3, z+0.2], 
                                facecolor=self.config.colors['fc'], alpha=0.8)
                ax.add_collection3d(box)
                self.artists['3d_components'][name] = box
                
                # Power flow animation particles
                if self._animation_enabled:
                    self._create_flow_particles(ax, (2, 5, 8), (5, 5, 6.5), p_fc, 'fc')
            
            elif name == 'battery' and (p_batt_dis > 0.1 or p_batt_chg > 0.1):
                color = self.config.colors['batt_dis'] if p_batt_dis > p_batt_chg else self.config.colors['batt_chg']
                box = Rectangle3D([x-0.3, y-0.3, z-0.2], [x+0.3, y+0.3, z+0.2], 
                                facecolor=color, alpha=0.8)
                ax.add_collection3d(box)
                self.artists['3d_components'][name] = box
                
                # Flow particles
                if self._animation_enabled:
                    flow_color = self.config.colors['batt_dis'] if p_batt_dis > p_batt_chg else self.config.colors['batt_chg']
                    flow_power = max(p_batt_dis, p_batt_chg)
                    direction = (5, 5, 6.5) if p_batt_dis > p_batt_chg else (2, 5, 8)
                    self._create_flow_particles(ax, (x, y, z), direction, flow_power, 'batt')
            
            elif name == 'power_bus':
                # Central power bus
                box = Rectangle3D([x-0.2, y-0.5, z-0.3], [x+0.2, y+0.5, z+0.3], 
                                facecolor='#FFD700', alpha=0.7)
                ax.add_collection3d(box)
                self.artists['3d_components'][name] = box
        
        # Update animation
        if self._animation_enabled:
            self._update_particle_animation(ax, data_list)
    
    def _create_flow_particles(self, ax, start_pos, end_pos, power, particle_type):
        """Create animated flow particles for 3D visualization."""
        if power < 1.0:  # Don't animate very small flows
            return
            
        # Create multiple particles along the flow path
        n_particles = min(int(power / 10), 10)  # Scale with power
        
        for i in range(n_particles):
            # Calculate particle position along path
            t = (self._animation_step * 0.1 + i * 0.2) % 1.0
            x = start_pos[0] + t * (end_pos[0] - start_pos[0])
            y = start_pos[1] + t * (end_pos[1] - start_pos[1])
            z = start_pos[2] + t * (end_pos[2] - start_pos[2])
            
            # Particle color based on type
            if particle_type == 'fc':
                color = self.config.colors['fc']
            elif particle_type == 'batt':
                color = self.config.colors['batt_dis'] if power > 0 else self.config.colors['batt_chg']
            else:
                color = 'yellow'
            
            # Draw particle
            ax.scatter([x], [y], [z], c=color, s=50, alpha=0.8, depthshade=True)
    
    def _update_particle_animation(self, ax, data_list):
        """Update particle animation."""
        self._animation_step += 1
        
        # Clear old particles
        for collection in ax.collections:
            try:
                collection.remove()
            except:
                pass
    
    def _update_environmental_impact(self, buffer: Deque):
        """Update environmental impact metrics (integrated into smart info)."""
        if not self._environmental_impact_enabled or len(buffer) == 0:
            return
        
        data_list = list(buffer)
        
        # Calculate environmental metrics
        env_metrics = self._calculate_environmental_metrics(data_list)
        
        # Update the smart info text with environmental data
        artist = self.artists.get('smart_info_text')
        if artist is not None:
            co2_per_km = env_metrics['co2_per_km']
            energy_eff = env_metrics['energy_efficiency']
            
            # Add environmental info to the smart text
            current_text = artist.get_text()
            env_info = f"\n[ECO] CO2/km: {co2_per_km:.3f} kg/km\n[EFF] Energy eff: {energy_eff:.1f}%"
            
            # Only add if not already there (avoid duplication)
            if "CO2/km" not in current_text:
                artist.set_text(current_text + env_info)
    
    def _update_recommendations(self, buffer: Deque):
        """Update smart recommendations (integrated into smart info)."""
        if len(buffer) == 0:
            return
        
        data_list = list(buffer)
        
        # Generate recommendations
        recommendations = self._generate_recommendations(data_list)
        
        # Update the smart info text with recommendations
        artist = self.artists.get('smart_info_text')
        if artist is not None and recommendations:
            # Get the most important recommendation (highest priority)
            high_priority_recs = [r for r in recommendations if r.get('priority') == 'high']
            if high_priority_recs:
                rec = high_priority_recs[0]
                current_text = artist.get_text()
                
                # Add recommendation with appropriate icon
                if rec.get('type') == 'soc':
                    icon = "[BATT]"
                elif rec.get('type') == 'hydrogen':
                    icon = "[H2]"
                elif rec.get('type') == 'power':
                    icon = "[PWR]"
                else:
                    icon = "[TIP]"
                
                rec_info = f"\n{icon} {rec['text']}"
                
                # Only add if not already there (avoid duplication)
                if rec['text'] not in current_text:
                    artist.set_text(current_text + rec_info)
    
    def _calculate_performance_kpis(self, data_list: List[Dict]) -> Dict[str, float]:
        """Calculate real-time performance KPIs."""
        if not data_list:
            return {}
        
        latest = data_list[-1]
        
        # Energy efficiency calculation
        total_distance = latest.get('distance_km', 0.0)
        total_fc_energy = sum(d.get('p_fc_kw', 0.0) for d in data_list) / 3600.0  # kWh
        total_batt_energy = sum(abs(d.get('p_batt_kw', 0.0)) for d in data_list) / 3600.0  # kWh
        total_energy = total_fc_energy + total_batt_energy
        
        # Efficiency score (0-100)
        if total_distance > 0 and total_energy > 0:
            theoretical_min_energy = total_distance * 2.0  # Assume 2 kWh/km minimum
            actual_energy = total_energy
            efficiency = max(0, min(100, 100 * (theoretical_min_energy / actual_energy)))
        else:
            efficiency = 50.0
        
        # Energy balance (how well supply matches demand)
        supply_times = [d.get('p_supply_kw', 0.0) for d in data_list]
        demand_times = [d.get('p_dem_kw', 0.0) for d in data_list]
        
        if supply_times and demand_times:
            balance_errors = [abs(s - d) for s, d in zip(supply_times, demand_times)]
            avg_error = np.mean(balance_errors) if balance_errors else 0
            max_demand = max(max(supply_times), max(demand_times), 1.0)
            energy_balance = max(0, min(100, 100 * (1 - avg_error / max_demand)))
        else:
            energy_balance = 50.0
        
        # Cost per km
        total_cost = sum(d.get('cost_h2_eur', 0.0) + d.get('cost_grid_eur', 0.0) for d in data_list)
        cost_per_km = total_cost / max(total_distance, 0.001)
        
        return {
            'efficiency_score': efficiency,
            'energy_balance': energy_balance,
            'cost_per_km': cost_per_km,
            'total_energy_kwh': total_energy,
            'total_cost_eur': total_cost,
        }
    
    def _calculate_environmental_metrics(self, data_list: List[Dict]) -> Dict[str, float]:
        """Calculate environmental impact metrics."""
        if not data_list:
            return {}
        
        latest = data_list[-1]
        total_distance = latest.get('distance_km', 0.0)
        
        # CO2 emissions (simplified calculation)
        total_h2_kg = sum(d.get('h2_consumed_kg', 0.0) for d in data_list)
        total_grid_kwh = sum(d.get('grid_charge_kwh', 0.0) for d in data_list)
        
        # Emission factors (kg CO2 per unit)
        h2_emission_factor = 0.0  # Green hydrogen
        grid_emission_factor = 0.4  # kg CO2 per kWh (grid average)
        
        co2_emissions = total_h2_kg * h2_emission_factor + total_grid_kwh * grid_emission_factor
        co2_per_km = co2_emissions / max(total_distance, 0.001)
        
        # Energy efficiency
        total_energy = sum(d.get('p_fc_kw', 0.0) + abs(d.get('p_batt_kw', 0.0)) for d in data_list) / 3600.0
        energy_efficiency = max(0, min(100, 100 * (total_distance / max(total_energy, 0.001) / 2.0)))
        
        # Environmental score (combined metric)
        env_score = (energy_efficiency + max(0, 100 - co2_per_km * 10)) / 2.0
        
        return {
            'co2_emissions_kg': co2_emissions,
            'co2_per_km': co2_per_km,
            'energy_efficiency': energy_efficiency,
            'environmental_score': env_score,
        }
    
    def _generate_recommendations(self, data_list: List[Dict]) -> List[Dict[str, Any]]:
        """Generate smart recommendations for energy optimization."""
        recommendations = []
        
        if not data_list:
            return recommendations
        
        latest = data_list[-1]
        soc = latest.get('soc', 0.5)
        tank_level = latest.get('tank_level_norm', 0.5)
        
        # SOC-based recommendations
        if soc < 0.3:
            recommendations.append({
                'text': 'Low SOC - consider charging strategy',
                'priority': 'high',
                'type': 'soc'
            })
        elif soc > 0.9:
            recommendations.append({
                'text': 'High SOC - could optimize discharge',
                'priority': 'medium',
                'type': 'soc'
            })
        
        # Tank level recommendations
        if tank_level < 0.2:
            recommendations.append({
                'text': 'Low H2 - plan refueling stop',
                'priority': 'high',
                'type': 'hydrogen'
            })
        
        # Power flow optimization
        unmet_power = latest.get('p_unmet_kw', 0.0)
        if unmet_power > 10:
            recommendations.append({
                'text': f'High unmet demand ({unmet_power:.0f} kW) - adjust power split',
                'priority': 'high',
                'type': 'power'
            })
        
        # Schedule performance
        delay = latest.get('delay_seconds', 0.0)
        if delay > 30:
            recommendations.append({
                'text': f'Significant delay ({delay:.0f}s) - consider energy-saving mode',
                'priority': 'medium',
                'type': 'schedule'
            })
        
        return recommendations
    
    def render_human(self, buffer: Deque):
        """Enhanced human rendering with all new features."""
        if len(buffer) == 0:
            return
        
        # Initialize enhanced components if not done
        if not hasattr(self, '_enhanced_initialized'):
            self._initialize_enhanced_figure()
        
        # Call parent method for base functionality
        super().render_human(buffer)
        
        # Update enhanced features
        self._update_enhanced_plots(buffer)
    
    def render_rgb_array(self, buffer: Deque) -> np.ndarray:
        """Enhanced RGB array rendering."""
        # Initialize enhanced components
        if not hasattr(self, '_enhanced_initialized'):
            self._initialize_enhanced_figure()
        
        # Update enhanced plots
        self._update_enhanced_plots(buffer)
        
        # Call parent method
        return super().render_rgb_array(buffer)
    
    def start_comparison_mode(self, max_episodes: int = 4):
        """Start multi-episode comparison mode."""
        self._comparison_mode = True
        self._max_comparison_episodes = max_episodes
        self._comparative_buffer = []
    
    def add_episode_to_comparison(self, buffer: Deque, episode_name: str = ""):
        """Add an episode buffer to comparison mode."""
        if not hasattr(self, '_comparison_mode') or not self._comparison_mode:
            return
        
        episode_data = {
            'name': episode_name or f"Episode {len(self._comparative_buffer) + 1}",
            'buffer': list(buffer),
            'metrics': self._calculate_comparison_metrics(list(buffer))
        }
        
        self._comparative_buffer.append(episode_data)
        
        # Keep only the most recent episodes
        if len(self._comparative_buffer) > self._max_comparison_episodes:
            self._comparative_buffer.pop(0)
    
    def _calculate_comparison_metrics(self, data_list: List[Dict]) -> Dict[str, float]:
        """Calculate metrics for episode comparison."""
        if not data_list:
            return {}
        
        latest = data_list[-1]
        total_distance = latest.get('distance_km', 0.0)
        total_cost = sum(d.get('cost_h2_eur', 0.0) + d.get('cost_grid_eur', 0.0) for d in data_list)
        
        return {
            'total_cost': total_cost,
            'cost_per_km': total_cost / max(total_distance, 0.001),
            'efficiency': self._calculate_performance_kpis(data_list)['efficiency_score'],
            'final_soc': latest.get('soc', 0.0),
            'total_delay': latest.get('delay_seconds', 0.0),
        }
    
    def close(self):
        """Clean up enhanced renderer resources."""
        super().close()
        
        # Clear enhanced state
        self._performance_history.clear()
        self._efficiency_scores.clear()
        self._comparative_buffer.clear()
        self._particle_systems.clear()


class AnomalyDetector:
    """Simple anomaly detection for energy patterns."""
    
    def __init__(self, window_size: int = 100, threshold: float = 2.0):
        self.window_size = window_size
        self.threshold = threshold
        self.history = deque(maxlen=window_size)
    
    def is_anomaly(self, value: float) -> bool:
        """Check if a value is anomalous."""
        if len(self.history) < 10:  # Need minimum data
            self.history.append(value)
            return False
        
        # Calculate z-score
        values = list(self.history)
        mean_val = np.mean(values)
        std_val = np.std(values)
        
        if std_val == 0:
            anomaly = False
        else:
            z_score = abs((value - mean_val) / std_val)
            anomaly = z_score > self.threshold
        
        self.history.append(value)
        return anomaly
