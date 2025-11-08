"""Timetable generation for realistic train scheduling."""
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any
import numpy as np


@dataclass
class Stop:
    """Train stop information."""
    position_m: float  # Position along route (meters)
    dwell_time_s: float  # Dwell duration (seconds)
    arrival_time_s: float  # Scheduled arrival time (seconds)
    departure_time_s: float  # Scheduled departure time (seconds)


@dataclass
class RouteSegment:
    """Route segment with grade and speed limit."""
    start_position_m: float
    end_position_m: float
    grade_percent: float  # Grade as percentage (e.g., 2.0 for +2%)
    speed_limit_mps: float  # Maximum speed for this segment


@dataclass
class Timetable:
    """Complete timetable with route and schedule information."""
    route_segments: List[RouteSegment]
    stops: List[Stop]
    total_distance_m: float
    total_time_s: float


class TimetableGenerator:
    """Generates timetables based on configuration parameters."""
    
    def __init__(
        self,
        route_config,
        stops_config,
        rng: np.random.Generator,
        segment_length_m: float = 1000.0  # Default 1km per segment
    ):
        self.route_config = route_config
        self.stops_config = stops_config
        self.rng = rng
        self.segment_length_m = segment_length_m
    
    def generate(self) -> Timetable:
        """Generate a complete timetable."""
        # Generate route segments
        route_segments = self._generate_route_segments()
        
        # Generate stops
        stops = self._generate_stops(route_segments)
        
        # Calculate totals
        total_distance_m = sum(
            seg.end_position_m - seg.start_position_m 
            for seg in route_segments
        )
        total_time_s = stops[-1].departure_time_s if stops else 0.0
        
        return Timetable(
            route_segments=route_segments,
            stops=stops,
            total_distance_m=total_distance_m,
            total_time_s=total_time_s
        )
    
    def _generate_route_segments(self) -> List[RouteSegment]:
        """Generate route segments with grades and speed limits."""
        segments = []
        position_m = 0.0
        
        # Create segments based on grade profile
        grade_profile = self.route_config.grade_segments_percent
        speed_limits = self.route_config.speed_limits_mps
        
        for i, grade_percent in enumerate(grade_profile):
            speed_limit = speed_limits[i] if i < len(speed_limits) else speed_limits[-1]
            
            segment = RouteSegment(
                start_position_m=position_m,
                end_position_m=position_m + self.segment_length_m,
                grade_percent=grade_percent,
                speed_limit_mps=speed_limit
            )
            segments.append(segment)
            position_m += self.segment_length_m
        
        return segments
    
    def _generate_stops(self, route_segments: List[RouteSegment]) -> List[Stop]:
        """Generate stops along the route with dwell times."""
        stops = []
        total_distance = sum(
            seg.end_position_m - seg.start_position_m 
            for seg in route_segments
        )
        
        if self.stops_config.count <= 1:
            # No intermediate stops, just start and end
            return stops
        
        # Distribute stops evenly along the route
        stop_positions = np.linspace(0, total_distance, self.stops_config.count)
        current_time_s = 0.0
        avg_speed_mps = 20.0  # Average speed for initial timing
        
        for i, position_m in enumerate(stop_positions):
            # Add dwell time jitter
            base_dwell = self.stops_config.dwell_mean_s
            jitter_frac = self.rng.uniform(
                -self.stops_config.dwell_jitter_frac,
                self.stops_config.dwell_jitter_frac
            )
            dwell_time = base_dwell * (1.0 + jitter_frac)
            
            # Calculate arrival time (simple approximation)
            if i > 0:
                travel_time = (position_m - stops[-1].position_m) / avg_speed_mps
                current_time_s += travel_time
            
            # Create stop
            stop = Stop(
                position_m=position_m,
                dwell_time_s=dwell_time,
                arrival_time_s=current_time_s,
                departure_time_s=current_time_s + dwell_time
            )
            stops.append(stop)
            
            # Update time for next segment
            current_time_s += dwell_time
        
        return stops
    
    def get_segment_at_position(self, position_m: float, route_segments: List[RouteSegment]) -> RouteSegment:
        """Get route segment at given position."""
        for segment in route_segments:
            if segment.start_position_m <= position_m < segment.end_position_m:
                return segment
        return route_segments[-1]  # Return last segment if past end
    
    def get_upcoming_stop_distance(self, position_m: float, stops: List[Stop]) -> float:
        """Get distance to next stop from current position."""
        for stop in stops:
            if stop.position_m > position_m:
                return stop.position_m - position_m
        return float('inf')  # No upcoming stops
