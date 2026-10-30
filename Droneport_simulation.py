from __future__ import annotations
import argparse
import copy
import csv
import heapq
import json
import math
import random
import statistics
import tempfile
from collections import defaultdict
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple
from copy import deepcopy
import hashlib
DEFAULT_CONFIG: Dict[str, Any] = {'field': {'area_ha': 27.0}, 'fleet': {'total_drones': 2, 'max_active_drones': 2}, 'station': {'cells_count': 3, 'doors_count': 3, 'gantries_count': 1, 'reserve_cell_policy': 'preferred_two', 'third_cell_active': 0}, 'flight': {'transit_speed_m_s': 10.0, 'work_speed_m_s': 7.0, 'swath_m': 10.0, 'transit_distance_m': 1000.0, 'tank_capacity_l': 40.0, 'application_rate_l_ha': 10.0, 'tank_reserve_fraction': 0.05, 'aircraft_mass_without_battery_kg': 39.9, 'liquid_density_kg_l': 1.0, 'fit_t50_power_proxy': True, 'reference_energy_wh': 1566.6, 'reference_empty_mass_kg': 52.0, 'reference_loaded_mass_kg': 92.0, 'reference_empty_endurance_min': 16.5, 'reference_loaded_endurance_min': 7.0, 'hover_power_coefficient_kw': 0.0, 'hover_power_exponent': 0.0, 'max_flight_speed_m_s': 10.0, 'max_spray_flow_l_min': 16.0, 'max_takeoff_mass_kg': 92.0, 'power_calibration_factor': 1.0, 'outbound_power_factor': 1.0, 'work_power_factor': 1.0, 'return_power_factor': 1.0}, 'timing': {'door_open_s': 20.0, 'door_close_s': 20.0, 'landing_s': 15.0, 'identification_s': 5.0, 'fixation_s': 5.0, 'platform_in_s': 25.0, 'platform_out_s': 25.0, 'manipulator_move_s': 10.0, 'service_check_s': 10.0, 'manipulator_clear_s': 5.0, 'refill_s': 120.0, 'battery_swap_s': 120.0, 'final_check_s': 20.0, 'takeoff_s': 15.0}, 'battery_rack': {'battery_count': 6, 'capacity': 4}, 'battery_model': {'capacity_ah': 30.0, 'nominal_voltage_v': 52.22, 'battery_mass_kg': 12.1, 'initial_soc': 0.95, 'dispatch_soc': 0.95, 'reserve_soc': 0.15, 'emergency_soc': 0.05, 'charge_target_soc': 0.95, 'charge_reference_from_soc': 0.3, 'charge_reference_to_soc': 0.95, 'charge_reference_time_s': 540.0, 'charger_power_kw': 9.0, 'charger_channels': 2, 'charge_efficiency': 0.92, 'cooling_slots': 2, 'cooling_reference_s': 120.0, 'minimum_cooling_s': 30.0, 'ambient_temperature_c': 25.0, 'hot_battery_temperature_c': 45.0, 'charge_start_max_temperature_c': 35.0, 'charge_end_temperature_c': 32.0, 'max_install_temperature_c': 40.0, 'hot_ambient_threshold_c': 35.0, 'hot_charge_time_multiplier': 1.2, 'hot_cooling_time_multiplier': 1.5, 'temperature_rise_at_full_discharge_c': 25.0, 'initial_soh': 1.0, 'minimum_soh': 0.8, 'soh_loss_per_equivalent_cycle': 0.0002, 'parameter_notes': {'capacity_and_mass': 'DJI DB1560 T50 nominal specification: 30 Ah, 52.22 V, 12.1 kg', 'charge': '9 min from 30 to 95%, specified DJI fast-charge conditions; other SoC intervals are extrapolated', 'reserve': '15% planning reserve and thermal timings are model assumptions', 'power_curve': 'T50 two-point laboratory endurance fit, inferred from nominal energy; not a validated flight-power curve'}}, 'communication': {'server_confirmation_required_for_new_mission': False}, 'simulation': {'max_time_s': 259200.0}, 'communication_outages': [], 'failures': [], 'randomness': {'enabled': False, 'seed': 42, 'operation_variation': 0.1, 'delay_probability': 0.08, 'delay_min_s': 30.0, 'delay_max_s': 90.0, 'service_delays': False}, 'weather': {'mode': 'none', 'random_intervals': True, 'normal_min_s': 900.0, 'normal_max_s': 2100.0, 'strong_min_s': 300.0, 'strong_max_s': 720.0, 'steady_power_factor': 1.08, 'steady_transit_speed_factor': 0.95, 'steady_work_rate_factor': 0.95, 'steady_landing_factor': 1.1, 'strong_power_factor': 1.15, 'strong_transit_speed_factor': 0.9, 'strong_work_rate_factor': 0.85, 'strong_landing_factor': 1.2, 'flight_update_s': 10.0, 'return_margin_wh': 2.0}, 'plots': {'enabled': True, 'formats': ['png', 'svg'], 'dpi': 600, 'width_mm': 180.0, 'timeline_minutes': 40.0}, 'battery_sweep': {'counts': [4, 5, 6, 7, 8, 9, 10], 'time_tolerance_fraction': 0.02, 'maximum_battery_wait_s': 1e-06}, 'experiment': {'replications': 10, 'scenario': 'configured', 'compare_fleets': [2, 3], 'output_dir': 'simulation_v8_results'}, 'scenarios': {'baseline': {'randomness': {'enabled': False, 'service_delays': False}, 'weather': {'mode': 'none'}}, 'operation_variation': {'randomness': {'enabled': True, 'service_delays': False}, 'weather': {'mode': 'none'}}, 'steady_wind': {'randomness': {'enabled': False, 'service_delays': False}, 'weather': {'mode': 'steady'}}, 'wind_episodes': {'randomness': {'enabled': True, 'operation_variation': 0.0, 'service_delays': False}, 'weather': {'mode': 'episodes'}}, 'service_delays': {'randomness': {'enabled': True, 'operation_variation': 0.0, 'service_delays': True}, 'weather': {'mode': 'none'}}, 'combined': {'randomness': {'enabled': True, 'service_delays': True}, 'weather': {'mode': 'episodes'}}}}
EPS = 1e-09
class SimulationError(RuntimeError):
    pass
@dataclass(order=True)
class ScheduledEvent:
    time_s: float
    priority: int
    sequence: int
    kind: str = field(compare=False)
    payload: Dict[str, Any] = field(compare=False, default_factory=dict)
@dataclass
class EventRecord:
    simulation_time_s: float
    event_id: str
    category: str
    operation: str
    object_type: str
    object_id: str
    old_state: str
    new_state: str
    related_object_id: str
    message: str
    details: Dict[str, Any] = field(default_factory=dict)
    def csv_row(self) -> Dict[str, Any]:
        row = asdict(self)
        row['details'] = json.dumps(self.details, ensure_ascii=False, sort_keys=True)
        return row
@dataclass
class Stateful:
    state: str
    state_since_s: float = 0.0
    state_durations_s: Dict[str, float] = field(default_factory=lambda: defaultdict(float))
    def transition(self, now_s: float, new_state: str) -> str:
        old_state = self.state
        self.state_durations_s[old_state] += max(0.0, now_s - self.state_since_s)
        self.state = new_state
        self.state_since_s = now_s
        return old_state
    def duration_in(self, state: str, now_s: float) -> float:
        total = self.state_durations_s.get(state, 0.0)
        if self.state == state:
            total += max(0.0, now_s - self.state_since_s)
        return total
@dataclass
class Drone(Stateful):
    drone_id: str = ''
    current_mission_id: Optional[str] = None
    assigned_cell_id: Optional[str] = None
    battery_id: Optional[str] = None
    position: str = 'STATION'
    assigned_area_ha: float = 0.0
    remaining_area_ha: float = 0.0
    remaining_liquid_l: float = 0.0
    remaining_energy_wh: float = 0.0
    flight_time_s: float = 0.0
    waiting_cell_time_s: float = 0.0
    cell_allocation_delay_s: float = 0.0
    waiting_gantry_time_s: float = 0.0
    cell_request_since_s: Optional[float] = None
    waiting_cell_since_s: Optional[float] = None
    waiting_gantry_since_s: Optional[float] = None
    ground_cycle_started_s: Optional[float] = None
    arrived_at_station: bool = False
    final_return: bool = False
    refill_complete: bool = False
    battery_service_complete: bool = False
    pending_battery_id: Optional[str] = None
    waiting_for_new_mission: bool = False
    missions_completed: int = 0
    arrival_time_s: float = 0.0
    queue_battery_wait_s: float = 0.0
    queue_manipulator_wait_s: float = 0.0
@dataclass
class GarageCell(Stateful):
    cell_id: str = ''
    door_id: str = ''
    platform_position: str = 'INSIDE'
    occupied_by: Optional[str] = None
    is_preferred: bool = True
    fault_state: bool = False
    reservation_count: int = 0
@dataclass
class Door(Stateful):
    door_id: str = ''
    cell_id: str = ''
    operation_end_time_s: Optional[float] = None
    fault_state: bool = False
    operation_token: int = 0
    open_count: int = 0
@dataclass
class ServiceGantry(Stateful):
    gantry_id: str = 'GANTRY-01'
    current_operation: Optional[str] = None
    target_cell_id: Optional[str] = None
    target_drone_id: Optional[str] = None
    operation_end_time_s: Optional[float] = None
    operation_token: int = 0
    paused_remaining_s: float = 0.0
    fault_state: bool = False
@dataclass
class Battery(Stateful):
    battery_id: str = ''
    assigned_drone_id: Optional[str] = None
    soc: float = 0.95
    soh: float = 1.0
    temperature_c: float = 25.0
    cycle_count: int = 0
    installation_count: int = 0
    equivalent_full_cycles: float = 0.0
    energy_throughput_wh: float = 0.0
    charged_energy_wh: float = 0.0
    location: str = 'RACK'
    cooling_wait_started_s: Optional[float] = None
    charge_wait_started_s: Optional[float] = None
    charge_started_soc: Optional[float] = None
    ready_at_s: Optional[float] = None
    fault_state: bool = False
@dataclass
class BatteryRack:
    rack_id: str
    capacity: int
    battery_ids: List[str] = field(default_factory=list)
@dataclass
class CoolingSlot(Stateful):
    slot_id: str = ''
    battery_id: Optional[str] = None
    operation_end_time_s: Optional[float] = None
    operation_token: int = 0
    completed_operations: int = 0
@dataclass
class ChargerChannel(Stateful):
    charger_id: str = ''
    battery_id: Optional[str] = None
    operation_end_time_s: Optional[float] = None
    operation_token: int = 0
    completed_operations: int = 0
    delivered_energy_wh: float = 0.0
    grid_energy_wh: float = 0.0
@dataclass
class Mission(Stateful):
    mission_id: str = ''
    assigned_drone_id: str = ''
    area_ha: float = 0.0
    start_time_s: Optional[float] = None
    work_finish_time_s: Optional[float] = None
    finish_time_s: Optional[float] = None
    priority: int = 0
    return_reason: str = ''
    outbound_time_s: float = 0.0
    work_time_s: float = 0.0
    return_time_s: float = 0.0
    energy_used_wh: float = 0.0
    outbound_energy_wh: float = 0.0
    work_energy_wh: float = 0.0
    return_energy_wh: float = 0.0
    liquid_used_l: float = 0.0
    battery_id: str = ''
    battery_soc_start: float = 0.0
    battery_soc_end_planned: float = 0.0
    planned_area_ha: float = 0.0
    planned_energy_wh: float = 0.0
    battery_soc_end_actual: float = 0.0
    early_return: bool = False
    early_return_weather_level: str = ''
@dataclass
class LocalController(Stateful):
    communication_state: str = 'ONLINE'
    field_model: Dict[str, Any] = field(default_factory=dict)
    active_missions: List[str] = field(default_factory=list)
    total_processed_area_ha: float = 0.0
def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result
def mean_or_zero(values: Sequence[float]) -> float:
    return statistics.fmean(values) if values else 0.0
class DroneportSimulation:
    def __init__(self, config: Dict[str, Any], scenario_name: str='base'):
        self.config = deep_merge(DEFAULT_CONFIG, config)
        self.scenario_name = scenario_name
        self._resolve_physical_parameters(config)
        self._validate_config()
        seed = self.config['randomness']['seed']
        self.weather_rng = random.Random(seed + 1000003)
        self.operation_rng = random.Random(seed + 2000003)
        self.delay_rng = random.Random(seed + 3000003)
        self.weather_timeline = self._build_weather_timeline()
        self.weather_starts = [row['start_s'] for row in self.weather_timeline]
        self.flight_runtime: Dict[str, Dict[str, Any]] = {}
        self.sampled_operations: List[Dict[str, Any]] = []
        self.service_delay_events = 0
        self.time_s = 0.0
        self._event_sequence = 0
        self._log_sequence = 0
        self._mission_sequence = 0
        self._queue_sequence = 0
        self._scheduled: List[ScheduledEvent] = []
        self.records: List[EventRecord] = []
        self.resource_conflicts = 0
        self.invariant_checks = 0
        self.invariant_violations = 0
        self.communication_loss_count = 0
        self.processing_completion_time_s: Optional[float] = None
        self.return_completion_time_s: Optional[float] = None
        self.ground_cycle_durations_s: List[float] = []
        self.final_recovery_durations_s: List[float] = []
        self.cell_waits_s: List[float] = []
        self.cell_allocation_delays_s: List[float] = []
        self.gantry_waits_s: List[float] = []
        self.cooling_waits_s: List[float] = []
        self.charger_waits_s: List[float] = []
        self.cooling_durations_s: List[float] = []
        self.charge_durations_s: List[float] = []
        self.battery_shortage_events = 0
        self.cell_queue: List[Tuple[float, int, str]] = []
        self.gantry_queue: List[Tuple[float, int, str]] = []
        self.cooling_queue: List[Tuple[float, int, str]] = []
        self.charger_queue: List[Tuple[float, int, str]] = []
        self._queued_for_cell: set[str] = set()
        self._queued_for_gantry: set[str] = set()
        self._queued_for_cooling: set[str] = set()
        self._queued_for_charging: set[str] = set()
        self.controller = LocalController(state='RUNNING', field_model=copy.deepcopy(self.config['field']))
        self.drones: Dict[str, Drone] = {}
        self.cells: Dict[str, GarageCell] = {}
        self.doors: Dict[str, Door] = {}
        self.batteries: Dict[str, Battery] = {}
        self.missions: Dict[str, Mission] = {}
        self.cooling_slots: Dict[str, CoolingSlot] = {}
        self.chargers: Dict[str, ChargerChannel] = {}
        self.gantry = ServiceGantry(state='IDLE')
        self.rack = BatteryRack(rack_id='RACK-01', capacity=int(self.config['battery_rack']['capacity']))
        self.active_drone_ids: List[str] = []
        self.battery_inventory: List[Dict[str, Any]] = []
        self._create_entities()
        self._capture_battery_inventory()
        self._schedule_configured_events()
        self._schedule(0.0, 'simulation_start', priority=20)
    def _resolve_physical_parameters(self, supplied: Dict[str, Any]) -> None:
        fc, bc = (self.config['flight'], self.config['battery_model'])
        legacy = supplied.get('flight', {})
        if 'speed_m_s' in legacy:
            if 'transit_speed_m_s' not in legacy:
                fc['transit_speed_m_s'] = legacy['speed_m_s']
            if 'work_speed_m_s' not in legacy:
                fc['work_speed_m_s'] = legacy['speed_m_s']
            fc.pop('speed_m_s', None)
        energy = float(bc['capacity_ah']) * float(bc['nominal_voltage_v'])
        if 'nominal_capacity_wh' in supplied.get('battery_model', {}):
            if not math.isclose(float(bc['nominal_capacity_wh']), energy, rel_tol=1e-06):
                raise ValueError('Set battery capacity_ah and nominal_voltage_v; nominal_capacity_wh must match their product')
        bc['nominal_capacity_wh'] = energy
        if fc['fit_t50_power_proxy']:
            m0, m1 = (fc['reference_empty_mass_kg'], fc['reference_loaded_mass_kg'])
            t0, t1 = (fc['reference_empty_endurance_min'], fc['reference_loaded_endurance_min'])
            if not 0 < m0 < m1 or not 0 < t1 < t0:
                raise ValueError('Invalid T50 power calibration anchors')
            exponent = math.log(t0 / t1) / math.log(m1 / m0)
            fc['hover_power_exponent'] = exponent
            fc['hover_power_coefficient_kw'] = fc['reference_energy_wh'] / 1000 / (t0 / 60) / m0 ** exponent
        if not 0 < fc['transit_speed_m_s'] <= fc['max_flight_speed_m_s']:
            raise ValueError('Transit speed exceeds T50 model limit')
        if not 0 < fc['work_speed_m_s'] <= fc['max_flight_speed_m_s']:
            raise ValueError('Work speed exceeds T50 model limit')
        requested_flow = fc['work_speed_m_s'] * fc['swath_m'] * fc['application_rate_l_ha'] * 60 / 10000
        if requested_flow > fc['max_spray_flow_l_min'] + EPS:
            raise ValueError('Requested application flow exceeds installed spraying system')
        full_mass = fc['aircraft_mass_without_battery_kg'] + bc['battery_mass_kg'] + fc['tank_capacity_l'] * fc['liquid_density_kg_l']
        if full_mass > fc['max_takeoff_mass_kg'] + EPS:
            raise ValueError('Full tank takeoff mass exceeds configured aircraft limit')
        for section in (fc, bc):
            if any((isinstance(v, (float, int)) and (not math.isfinite(v)) for v in section.values())):
                raise ValueError('Physical parameters must be finite')
        if bc['capacity_ah'] <= 0 or bc['nominal_voltage_v'] <= 0:
            raise ValueError('Battery capacity and voltage must be positive')
    def _capture_battery_inventory(self) -> None:
        states = [b.state for b in self.batteries.values()]
        row = {'time_s': self.time_s, 'total': len(states), 'in_drones': states.count('IN_DRONE'), 'reserved': states.count('RESERVED'), 'ready': len(self._ready_battery_ids()), 'cooling': states.count('COOLING'), 'waiting_cooling': states.count('WAITING_FOR_COOLING'), 'charging': states.count('CHARGING'), 'waiting_charger': states.count('WAITING_FOR_CHARGER')}
        row['other_unavailable'] = len(states) - sum((v for k, v in row.items() if k not in {'time_s', 'total'}))
        if row['other_unavailable'] < 0 or sum((v for k, v in row.items() if k not in {'time_s', 'total'})) != len(states):
            raise SimulationError('Battery inventory balance failed')
        if self.battery_inventory and self.battery_inventory[-1]['time_s'] == self.time_s:
            self.battery_inventory[-1] = row
        else:
            self.battery_inventory.append(row)
    def _battery_usage_metrics(self) -> Dict[str, Any]:
        used = {m.battery_id for m in self.missions.values() if m.start_time_s is not None}
        swaps = sum((r.operation == 'BATTERY_SWAP_COMPLETE' for r in self.records))
        max_state = lambda key: max((r[key] for r in self.battery_inventory), default=0)
        return {'unique_batteries_used': len(used), 'used_battery_ids': sorted(used), 'unused_batteries_count': len(self.batteries) - len(used), 'battery_swap_count': swaps, 'battery_flight_uses': sum((b.cycle_count for b in self.batteries.values())), 'battery_charge_completions': sum((c.completed_operations for c in self.chargers.values())), 'battery_efc_total': round(sum((b.equivalent_full_cycles for b in self.batteries.values())), 6), 'minimum_ready_batteries': min((r['ready'] for r in self.battery_inventory), default=0), 'peak_charging_batteries': max_state('charging'), 'peak_cooling_batteries': max_state('cooling'), 'peak_waiting_for_charger': max_state('waiting_charger'), 'peak_waiting_for_cooling': max_state('waiting_cooling'), 'peak_batteries_outside_drones': max((r['total'] - r['in_drones'] for r in self.battery_inventory), default=0), 'installed_charger_output_kw': self.config['battery_model']['charger_channels'] * self.config['battery_model']['charger_power_kw'], 'stock_sizing_note': 'Unique packs used is not the minimum required inventory; use --battery-sweep under the selected scenario.'}
    def _validate_config(self) -> None:
        required = ('field', 'fleet', 'station', 'flight', 'timing', 'battery_rack', 'battery_model', 'communication', 'simulation')
        missing = [name for name in required if name not in self.config]
        if missing:
            raise ValueError(f'Missing configuration sections: {missing}')
        field_cfg = self.config['field']
        fleet = self.config['fleet']
        station = self.config['station']
        flight = self.config['flight']
        timing = self.config['timing']
        rack = self.config['battery_rack']
        battery_cfg = self.config['battery_model']
        if float(field_cfg['area_ha']) <= 0:
            raise ValueError('field.area_ha must be positive')
        total_drones = int(fleet['total_drones'])
        active_drones = int(fleet['max_active_drones'])
        cells_count = int(station['cells_count'])
        if not 1 <= active_drones <= total_drones:
            raise ValueError('fleet.max_active_drones must be between 1 and total_drones')
        mode = station['third_cell_active']
        if type(mode) is not int or mode not in (0, 1):
            raise ValueError('station.third_cell_active must be integer 0 or 1')
        if cells_count not in (2, 3):
            raise ValueError('station.cells_count must be 2 or 3')
        if cells_count == 2 and mode:
            raise ValueError('third_cell_active=1 requires three installed cells')
        if active_drones > (3 if mode else 2):
            raise ValueError('Active drones exceed working cells; enable cell 3 or reduce fleet')
        if cells_count < total_drones:
            raise ValueError('station.cells_count must be at least fleet.total_drones so every drone can be safely accommodated at the station')
        if int(station['doors_count']) != cells_count:
            raise ValueError('v6.0 requires one door for every garage cell')
        if int(station['gantries_count']) != 1:
            raise ValueError('v6.0 currently supports exactly one gantry')
        if station['reserve_cell_policy'] not in {'dynamic', 'preferred_two'}:
            raise ValueError('Unknown reserve_cell_policy')
        for key, value in timing.items():
            if not isinstance(value, (int, float)) or value < 0:
                raise ValueError(f'timing.{key} must be non-negative')
        for key in ('transit_speed_m_s', 'work_speed_m_s', 'swath_m', 'transit_distance_m', 'tank_capacity_l', 'application_rate_l_ha', 'aircraft_mass_without_battery_kg', 'liquid_density_kg_l', 'hover_power_coefficient_kw', 'hover_power_exponent', 'power_calibration_factor', 'outbound_power_factor', 'work_power_factor', 'return_power_factor'):
            if float(flight[key]) <= 0:
                raise ValueError(f'flight.{key} must be positive')
        if not 0 <= float(flight['tank_reserve_fraction']) < 1:
            raise ValueError('flight.tank_reserve_fraction must be in [0, 1)')
        for key in ('nominal_capacity_wh', 'battery_mass_kg', 'charge_reference_time_s', 'charger_power_kw', 'charge_efficiency', 'cooling_reference_s', 'minimum_cooling_s', 'initial_soh', 'minimum_soh'):
            if float(battery_cfg[key]) <= 0:
                raise ValueError(f'battery_model.{key} must be positive')
        for key in ('initial_soc', 'dispatch_soc', 'reserve_soc', 'emergency_soc', 'charge_target_soc', 'charge_reference_from_soc', 'charge_reference_to_soc', 'initial_soh', 'minimum_soh'):
            if not 0 < float(battery_cfg[key]) <= 1:
                raise ValueError(f'battery_model.{key} must be in (0, 1]')
        emergency_soc = float(battery_cfg['emergency_soc'])
        reserve_soc = float(battery_cfg['reserve_soc'])
        dispatch_soc = float(battery_cfg['dispatch_soc'])
        target_soc = float(battery_cfg['charge_target_soc'])
        if not emergency_soc < reserve_soc < dispatch_soc <= target_soc:
            raise ValueError('Expected emergency_soc < reserve_soc < dispatch_soc <= charge_target_soc')
        if float(battery_cfg['initial_soc']) < dispatch_soc:
            raise ValueError('battery_model.initial_soc must allow initial dispatch')
        if float(battery_cfg['minimum_soh']) > float(battery_cfg['initial_soh']):
            raise ValueError('battery_model.minimum_soh exceeds initial_soh')
        if float(battery_cfg['charge_reference_to_soc']) <= float(battery_cfg['charge_reference_from_soc']):
            raise ValueError('charge reference SoC interval must be positive')
        if float(battery_cfg['charge_efficiency']) > 1.0:
            raise ValueError('battery_model.charge_efficiency must not exceed 1')
        if float(battery_cfg['soh_loss_per_equivalent_cycle']) < 0:
            raise ValueError('battery_model.soh_loss_per_equivalent_cycle is negative')
        if int(battery_cfg['charger_channels']) < 1:
            raise ValueError('battery_model.charger_channels must be at least 1')
        if int(battery_cfg['cooling_slots']) < 1:
            raise ValueError('battery_model.cooling_slots must be at least 1')
        battery_count = int(rack['battery_count'])
        if battery_count < total_drones + 1:
            raise ValueError('battery_rack.battery_count must include at least one spare')
        if int(rack['capacity']) < battery_count - total_drones:
            raise ValueError('Battery rack capacity is too small for spare batteries')
        self._validate_stochastic_config()
        if float(self.config['simulation']['max_time_s']) <= 0:
            raise ValueError('simulation.max_time_s must be positive')
    def _validate_stochastic_config(self) -> None:
        rc, wc = (self.config['randomness'], self.config['weather'])
        if type(rc['seed']) is not int:
            raise ValueError('randomness.seed must be integer')
        if not 0 <= rc['operation_variation'] < 1:
            raise ValueError('operation_variation must be in [0, 1)')
        if not 0 <= rc['delay_probability'] <= 1:
            raise ValueError('delay_probability must be in [0, 1]')
        if not all((math.isfinite(rc[k]) for k in ('delay_min_s', 'delay_max_s'))) or not 0 <= rc['delay_min_s'] <= rc['delay_max_s']:
            raise ValueError('Invalid service delay bounds')
        if wc['mode'] not in {'none', 'steady', 'episodes'}:
            raise ValueError('weather.mode must be none, steady or episodes')
        for prefix in ('normal', 'strong'):
            if not all((math.isfinite(wc[prefix + suffix]) for suffix in ('_min_s', '_max_s'))) or not 0 < wc[prefix + '_min_s'] <= wc[prefix + '_max_s']:
                raise ValueError('Weather interval bounds must be positive and ordered')
        for prefix in ('steady', 'strong'):
            for suffix in ('power_factor', 'transit_speed_factor', 'work_rate_factor', 'landing_factor'):
                if not math.isfinite(wc[prefix + '_' + suffix]) or wc[prefix + '_' + suffix] <= 0:
                    raise ValueError('Weather factors must be finite and positive')
        if not math.isfinite(wc['flight_update_s']) or wc['flight_update_s'] <= 0:
            raise ValueError('flight_update_s must be finite and positive')
        if not math.isfinite(wc['return_margin_wh']) or wc['return_margin_wh'] < 0:
            raise ValueError('return_margin_wh must be finite and nonnegative')
        if type(self.config['experiment']['replications']) is not int or self.config['experiment']['replications'] < 1:
            raise ValueError('experiment.replications must be a positive integer')
        for key, value in self.config['timing'].items():
            if not math.isfinite(value):
                raise ValueError(f'timing.{key} must be finite')
    def _create_entities(self) -> None:
        fleet = self.config['fleet']
        station = self.config['station']
        flight = self.config['flight']
        battery_cfg = self.config['battery_model']
        field_area = float(self.config['field']['area_ha'])
        active_count = int(fleet['max_active_drones'])
        total_count = int(fleet['total_drones'])
        initial_soc = float(battery_cfg['initial_soc'])
        initial_soh = float(battery_cfg['initial_soh'])
        nominal_capacity_wh = float(battery_cfg['nominal_capacity_wh'])
        initial_energy_wh = nominal_capacity_wh * initial_soh * initial_soc
        for index in range(1, int(station['cells_count']) + 1):
            cell_id = f'CELL-{index:02d}'
            door_id = f'DOOR-{index:02d}'
            self.cells[cell_id] = GarageCell(state='FREE', cell_id=cell_id, door_id=door_id, is_preferred=index <= 2)
            self.doors[door_id] = Door(state='CLOSED', door_id=door_id, cell_id=cell_id)
        per_active_drone = field_area / active_count
        for index in range(1, total_count + 1):
            drone_id = f'DRONE-{index:03d}'
            is_active = index <= active_count
            self.drones[drone_id] = Drone(state='READY' if is_active else 'PARKED', drone_id=drone_id, assigned_area_ha=per_active_drone if is_active else 0.0, remaining_area_ha=per_active_drone if is_active else 0.0, remaining_liquid_l=float(flight['tank_capacity_l']), remaining_energy_wh=initial_energy_wh)
            if is_active:
                self.active_drone_ids.append(drone_id)
        battery_count = int(self.config['battery_rack']['battery_count'])
        ambient = float(battery_cfg['ambient_temperature_c'])
        for index in range(1, battery_count + 1):
            battery_id = f'BAT-{index:03d}'
            if index <= total_count:
                drone_id = f'DRONE-{index:03d}'
                battery = Battery(state='IN_DRONE', battery_id=battery_id, assigned_drone_id=drone_id, installation_count=1, soc=initial_soc, soh=initial_soh, temperature_c=ambient, location=drone_id)
                self.drones[drone_id].battery_id = battery_id
            else:
                battery = Battery(state='READY', battery_id=battery_id, soc=initial_soc, soh=initial_soh, temperature_c=ambient, location=self.rack.rack_id, ready_at_s=0.0)
                self.rack.battery_ids.append(battery_id)
            self.batteries[battery_id] = battery
        for index in range(1, int(battery_cfg['cooling_slots']) + 1):
            slot_id = f'COOLING-{index:02d}'
            self.cooling_slots[slot_id] = CoolingSlot(state='IDLE', slot_id=slot_id)
        for index in range(1, int(battery_cfg['charger_channels']) + 1):
            charger_id = f'CHARGER-{index:02d}'
            self.chargers[charger_id] = ChargerChannel(state='IDLE', charger_id=charger_id)
        inactive = [drone for drone in self.drones.values() if drone.drone_id not in self.active_drone_ids]
        available_cells = list(reversed(list(self.cells.values())))
        for drone, cell in zip(inactive, available_cells):
            cell.state = 'OCCUPIED'
            cell.occupied_by = drone.drone_id
            drone.assigned_cell_id = cell.cell_id
    def _schedule_configured_events(self) -> None:
        if self.config['weather']['mode'] != 'none':
            for weather in self.weather_timeline:
                self._schedule(weather['start_s'], 'weather_changed', dict(weather), 0)
        for outage in self.config.get('communication_outages', []):
            start = float(outage['start_s'])
            duration = float(outage['duration_s'])
            self._schedule(start, 'communication_lost', {'outage': outage}, 0)
            self._schedule(start + duration, 'communication_restored', {'outage': outage}, 0)
        for failure in self.config.get('failures', []):
            start = float(failure['start_s'])
            duration = float(failure['duration_s'])
            payload = {'failure': failure}
            self._schedule(start, 'failure_start', payload, 0)
            self._schedule(start + duration, 'failure_end', payload, 0)
    def _schedule(self, time_s: float, kind: str, payload: Optional[Dict[str, Any]]=None, priority: int=10) -> None:
        if time_s < self.time_s - EPS:
            raise SimulationError(f'Cannot schedule {kind} in the past')
        self._event_sequence += 1
        heapq.heappush(self._scheduled, ScheduledEvent(time_s=float(time_s), priority=priority, sequence=self._event_sequence, kind=kind, payload=payload or {}))
    def _log(self, category: str, operation: str, object_type: str, object_id: str, old_state: str='', new_state: str='', related_object_id: str='', message: str='', details: Optional[Dict[str, Any]]=None) -> None:
        self._log_sequence += 1
        self.records.append(EventRecord(simulation_time_s=round(self.time_s, 6), event_id=f'EVT-{self._log_sequence:07d}', category=category, operation=operation, object_type=object_type, object_id=object_id, old_state=old_state, new_state=new_state, related_object_id=related_object_id, message=message, details=details or {}))
    def _change_state(self, entity: Stateful, new_state: str, object_type: str, object_id: str, related_object_id: str='', message: str='', details: Optional[Dict[str, Any]]=None) -> None:
        old_state = entity.transition(self.time_s, new_state)
        self._log('STATE', 'STATE_TRANSITION', object_type, object_id, old_state, new_state, related_object_id, message, details)
    def _command(self, target_type: str, target_id: str, command: str, related_id: str='', details: Optional[Dict[str, Any]]=None) -> None:
        self._log('COMMAND', 'COMMAND_SENT', target_type, target_id, related_object_id=related_id, message=command, details=details)
    def _effective_battery_capacity_wh(self, battery: Battery) -> float:
        nominal = float(self.config['battery_model']['nominal_capacity_wh'])
        return nominal * battery.soh
    def _flight_power_kw(self, total_mass_kg: float, phase: str) -> float:
        flight = self.config['flight']
        coefficient = float(flight['hover_power_coefficient_kw'])
        exponent = float(flight['hover_power_exponent'])
        calibration = float(flight['power_calibration_factor'])
        phase_factor = float(flight[f'{phase}_power_factor'])
        return coefficient * total_mass_kg ** exponent * calibration * phase_factor
    def _mean_work_power_kw(self, mass0: float, mass1: float) -> float:
        fc = self.config['flight']
        exponent = float(fc['hover_power_exponent'])
        if abs(mass0 - mass1) < 1e-10:
            return self._flight_power_kw(mass0, 'work')
        scale = fc['hover_power_coefficient_kw'] * fc['power_calibration_factor'] * fc['work_power_factor']
        if abs(exponent + 1) < 1e-10:
            return scale * math.log(mass0 / mass1) / (mass0 - mass1)
        return scale * (mass0 ** (exponent + 1) - mass1 ** (exponent + 1)) / ((exponent + 1) * (mass0 - mass1))
    def _sortie_energy_components(self, area_ha: float) -> Tuple[float, float, float, float, float]:
        flight = self.config['flight']
        speed = float(flight['transit_speed_m_s'])
        swath = float(flight['swath_m'])
        transit_s = float(flight['transit_distance_m']) / speed
        coverage_rate_ha_s = float(flight['work_speed_m_s']) * swath / 10000.0
        work_s = area_ha / coverage_rate_ha_s
        liquid_used_l = area_ha * float(flight['application_rate_l_ha'])
        initial_liquid_l = float(flight['tank_capacity_l'])
        final_liquid_l = max(0.0, initial_liquid_l - liquid_used_l)
        average_work_liquid_l = 0.5 * (initial_liquid_l + final_liquid_l)
        density = float(flight['liquid_density_kg_l'])
        aircraft_mass = float(flight['aircraft_mass_without_battery_kg']) + float(self.config['battery_model']['battery_mass_kg'])
        outbound_mass = aircraft_mass + initial_liquid_l * density
        work_mass = aircraft_mass + average_work_liquid_l * density
        return_mass = aircraft_mass + final_liquid_l * density
        outbound_wh = self._flight_power_kw(outbound_mass, 'outbound') * transit_s * 1000.0 / 3600.0
        work_wh = self._mean_work_power_kw(outbound_mass, return_mass) * work_s * 1000.0 / 3600.0
        return_wh = self._flight_power_kw(return_mass, 'return') * transit_s * 1000.0 / 3600.0
        return (outbound_wh, work_wh, return_wh, work_s, liquid_used_l)
    def _build_weather_timeline(self) -> List[Dict[str, Any]]:
        wc = self.config['weather']
        horizon = float(self.config['simulation']['max_time_s'])
        if wc['mode'] in {'none', 'steady'}:
            return [{'start_s': 0.0, 'end_s': horizon, 'level': 'normal' if wc['mode'] == 'none' else 'steady'}]
        rows, now, level = ([], 0.0, 'normal')
        while now < horizon:
            lo, hi = (wc[level + '_min_s'], wc[level + '_max_s'])
            duration = self.weather_rng.uniform(lo, hi) if self.config['randomness']['enabled'] and wc['random_intervals'] else (lo + hi) / 2
            end = min(horizon, now + duration)
            rows.append({'start_s': now, 'end_s': end, 'level': level})
            now, level = (end, 'strong' if level == 'normal' else 'normal')
        return rows
    def _weather_at(self, time_s: float) -> Dict[str, Any]:
        from bisect import bisect_right
        index = max(0, bisect_right(self.weather_starts, time_s) - 1)
        row = self.weather_timeline[index]
        level, wc = (row['level'], self.config['weather'])
        result = dict(row)
        for key in ('power_factor', 'transit_speed_factor', 'work_rate_factor', 'landing_factor'):
            result[key] = 1.0 if level == 'normal' else float(wc[level + '_' + key])
        return result
    def _sample_duration(self, duration: float, operation: str) -> float:
        rc = self.config['randomness']
        value = duration
        if rc['enabled'] and rc['operation_variation'] > 0 and (duration > 0):
            spread = rc['operation_variation']
            value *= self.operation_rng.triangular(1 - spread, 1 + spread, 1)
        self.sampled_operations.append({'time_s': self.time_s, 'operation': operation, 'duration_s': value})
        return value
    def _duration(self, key: str) -> float:
        value = float(self.config['timing'][key])
        if key in {'landing_s', 'takeoff_s'}:
            value *= self._weather_at(self.time_s)['landing_factor']
        return self._sample_duration(value, key)
    def _flight_mass(self, area_ha: float) -> float:
        fc = self.config['flight']
        liquid = max(0.0, fc['tank_capacity_l'] - area_ha * fc['application_rate_l_ha'])
        return fc['aircraft_mass_without_battery_kg'] + self.config['battery_model']['battery_mass_kg'] + liquid * fc['liquid_density_kg_l']
    def _return_envelope_wh(self, area_ha: float, distance_m: float) -> float:
        wc, fc = (self.config['weather'], self.config['flight'])
        levels = ['steady'] if wc['mode'] == 'steady' else ['strong']
        worst_ratio = max([1.0] + [wc[x + '_power_factor'] / wc[x + '_transit_speed_factor'] for x in levels])
        return self._flight_power_kw(self._flight_mass(area_ha), 'return') * 1000 / 3600 * distance_m / fc['transit_speed_m_s'] * worst_ratio
    def _start_weather_flight(self, drone: Drone, mission: Mission) -> None:
        mission.area_ha = mission.energy_used_wh = mission.liquid_used_l = 0.0
        mission.outbound_energy_wh = mission.work_energy_wh = mission.return_energy_wh = 0.0
        mission.outbound_time_s = mission.work_time_s = mission.return_time_s = 0.0
        self.flight_runtime[mission.mission_id] = {'phase': 'outbound', 'distance_m': 0.0}
        self._queue_flight_step(mission)
    def _flight_delta(self, mission: Mission, phase: str, duration: float, weather: Dict[str, Any]) -> Tuple[float, float, float]:
        fc = self.config['flight']
        mass0 = self._flight_mass(mission.area_ha)
        if phase == 'work':
            rate = fc['work_speed_m_s'] * fc['swath_m'] / 10000 * weather['work_rate_factor']
            area = duration * rate
            mass1 = self._flight_mass(mission.area_ha + area)
            mean_power = self._mean_work_power_kw(mass0, mass1)
            distance = 0.0
        else:
            area = 0.0
            distance = duration * fc['transit_speed_m_s'] * weather['transit_speed_factor']
            mean_power = self._flight_power_kw(mass0, phase)
        energy = mean_power * weather['power_factor'] * duration * 1000 / 3600
        return (energy, area, distance)
    def _queue_flight_step(self, mission: Mission) -> None:
        runtime = self.flight_runtime[mission.mission_id]
        phase, wc, fc = (runtime['phase'], self.config['weather'], self.config['flight'])
        weather = self._weather_at(self.time_s)
        dt = min(float(wc['flight_update_s']), max(0.0, weather['end_s'] - self.time_s))
        if dt <= 0:
            raise SimulationError('Weather horizon exhausted')
        if phase == 'work':
            left = mission.planned_area_ha - mission.area_ha
            if left <= 1e-08:
                self._finish_weather_work(mission, early=False)
                return
            rate = fc['work_speed_m_s'] * fc['swath_m'] / 10000 * weather['work_rate_factor']
            dt = min(dt, left / rate)
        else:
            left = fc['transit_distance_m'] - runtime['distance_m'] if phase == 'outbound' else runtime['distance_m']
            dt = min(dt, max(0.0, left) / (fc['transit_speed_m_s'] * weather['transit_speed_factor']))
        early = False
        if phase != 'return':
            battery = self.batteries[mission.battery_id]
            budget = (mission.battery_soc_start - self.config['battery_model']['reserve_soc']) * self._effective_battery_capacity_wh(battery) - wc['return_margin_wh']
            def committed(seconds: float) -> float:
                energy, area, distance = self._flight_delta(mission, phase, seconds, weather)
                return mission.energy_used_wh + energy + self._return_envelope_wh(mission.area_ha + area, runtime['distance_m'] + distance)
            if committed(dt) >= budget:
                low, high = (0.0, dt)
                if committed(0.0) > budget + 1e-06:
                    raise SimulationError('Online return guard has insufficient energy')
                for _ in range(48):
                    mid = (low + high) / 2
                    if committed(mid) <= budget:
                        low = mid
                    else:
                        high = mid
                dt, early = (low, True)
        energy, area, distance = self._flight_delta(mission, phase, dt, weather)
        self._schedule(self.time_s + dt, 'flight_step', {'mission_id': mission.mission_id, 'duration_s': dt, 'energy_wh': energy, 'area_ha': area, 'distance_m': distance, 'phase': phase, 'early': early})
    def _on_flight_step(self, payload: Dict[str, Any]) -> None:
        mission = self.missions[payload['mission_id']]
        runtime = self.flight_runtime[mission.mission_id]
        phase = payload['phase']
        mission.energy_used_wh += payload['energy_wh']
        setattr(mission, phase + '_energy_wh', getattr(mission, phase + '_energy_wh') + payload['energy_wh'])
        setattr(mission, phase + '_time_s', getattr(mission, phase + '_time_s') + payload['duration_s'])
        mission.area_ha += payload['area_ha']
        mission.liquid_used_l = mission.area_ha * self.config['flight']['application_rate_l_ha']
        runtime['distance_m'] += (-1 if phase == 'return' else 1) * payload['distance_m']
        if payload['early']:
            self._finish_weather_work(mission, early=True)
            return
        if phase == 'outbound' and runtime['distance_m'] >= self.config['flight']['transit_distance_m'] - 1e-06:
            runtime['phase'] = 'work'
        elif phase == 'return' and runtime['distance_m'] <= 1e-06:
            self._on_return_arrival({'drone_id': mission.assigned_drone_id, 'mission_id': mission.mission_id})
            del self.flight_runtime[mission.mission_id]
            return
        self._queue_flight_step(mission)
    def _finish_weather_work(self, mission: Mission, early: bool) -> None:
        drone = self.drones[mission.assigned_drone_id]
        mission.early_return = early and mission.area_ha < mission.planned_area_ha - 1e-07
        if mission.area_ha <= 1e-07:
            raise SimulationError('No productive area: selected weather/energy envelope is infeasible')
        if mission.early_return:
            mission.return_reason = 'wind_energy'
            mission.early_return_weather_level = self._weather_at(self.time_s)['level']
            self._log('FLIGHT', 'EARLY_RETURN', 'DRONE', drone.drone_id, related_object_id=mission.mission_id, details={'planned_area_ha': mission.planned_area_ha, 'actual_area_ha': mission.area_ha, 'energy_used_wh': mission.energy_used_wh, 'weather_level': self._weather_at(self.time_s)['level']})
        drone.final_return = drone.remaining_area_ha <= mission.area_ha + 1e-07
        self.flight_runtime[mission.mission_id]['phase'] = 'return'
        self._on_mission_work_complete({'drone_id': drone.drone_id, 'mission_id': mission.mission_id})
        self._queue_flight_step(mission)
    def _plan_sortie(self, drone: Drone) -> Mission:
        if not drone.battery_id:
            raise SimulationError(f'{drone.drone_id} has no installed battery')
        battery = self.batteries[drone.battery_id]
        if battery.state != 'IN_DRONE' or battery.assigned_drone_id != drone.drone_id:
            raise SimulationError(f'Battery assignment is invalid for {drone.drone_id}')
        flight = self.config['flight']
        battery_cfg = self.config['battery_model']
        tank_capacity = float(flight['tank_capacity_l'])
        liquid_rate = float(flight['application_rate_l_ha'])
        tank_reserve_l = tank_capacity * float(flight['tank_reserve_fraction'])
        area_by_liquid = (tank_capacity - tank_reserve_l) / liquid_rate
        upper_area = min(drone.remaining_area_ha, area_by_liquid)
        effective_capacity_wh = self._effective_battery_capacity_wh(battery)
        reserve_soc = float(battery_cfg['reserve_soc'])
        available_energy_wh = max(0.0, (battery.soc - reserve_soc) * effective_capacity_wh)
        def energy_for(area: float) -> float:
            outbound, work, returning, _, _ = self._sortie_energy_components(area)
            return outbound + work + returning
        transit_only_wh = energy_for(0.0)
        if transit_only_wh >= available_energy_wh - EPS:
            raise SimulationError(f'{battery.battery_id} cannot cover transit and reserve: available={available_energy_wh:.1f} Wh, transit={transit_only_wh:.1f} Wh')
        energy_limited = energy_for(upper_area) > available_energy_wh + 1e-07
        if energy_limited:
            low, high = (0.0, upper_area)
            for _ in range(64):
                midpoint = 0.5 * (low + high)
                if energy_for(midpoint) <= available_energy_wh:
                    low = midpoint
                else:
                    high = midpoint
            area_ha = low
        else:
            area_ha = upper_area
        if area_ha <= EPS:
            raise SimulationError(f'No positive sortie area for {drone.drone_id}')
        outbound_wh, work_wh, return_wh, work_s, liquid_used_l = self._sortie_energy_components(area_ha)
        energy_used_wh = outbound_wh + work_wh + return_wh
        soc_end = battery.soc - energy_used_wh / effective_capacity_wh
        if soc_end < reserve_soc - 1e-07:
            raise SimulationError(f'Mission planner violated reserve SoC for {battery.battery_id}')
        limiting: List[str] = []
        if drone.remaining_area_ha <= area_ha + 1e-07:
            limiting.append('task_complete')
        if energy_limited:
            limiting.append('battery')
        if area_by_liquid <= area_ha + 1e-07:
            limiting.append('liquid')
        speed = float(flight['transit_speed_m_s'])
        outbound_s = float(flight['transit_distance_m']) / speed
        self._mission_sequence += 1
        return Mission(state='PLANNED', mission_id=f'MISSION-{self._mission_sequence:05d}', assigned_drone_id=drone.drone_id, area_ha=area_ha, priority=0, return_reason='+'.join(limiting), outbound_time_s=outbound_s, work_time_s=work_s, return_time_s=outbound_s, energy_used_wh=energy_used_wh, outbound_energy_wh=outbound_wh, work_energy_wh=work_wh, return_energy_wh=return_wh, liquid_used_l=liquid_used_l, battery_id=battery.battery_id, battery_soc_start=battery.soc, battery_soc_end_planned=soc_end, planned_area_ha=area_ha, planned_energy_wh=energy_used_wh)
    def _start_next_mission(self, drone: Drone) -> None:
        if drone.remaining_area_ha <= EPS:
            return
        communication = self.config['communication']
        if self.controller.communication_state == 'OFFLINE' and bool(communication['server_confirmation_required_for_new_mission']):
            drone.waiting_for_new_mission = True
            self._change_state(drone, 'PARKED', 'DRONE', drone.drone_id, message='New mission waits for server confirmation')
            self._log('SAFETY', 'MISSION_START_BLOCKED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', related_object_id=drone.drone_id, message='Satellite link is offline')
            return
        mission = self._plan_sortie(drone)
        self.missions[mission.mission_id] = mission
        drone.current_mission_id = mission.mission_id
        drone.final_return = 'task_complete' in mission.return_reason
        drone.position = 'FIELD'
        drone.arrived_at_station = False
        mission.start_time_s = self.time_s
        self.controller.active_missions.append(mission.mission_id)
        self._log('DISPATCH', 'MISSION_ASSIGNED', 'MISSION', mission.mission_id, 'PLANNED', 'ASSIGNED', drone.drone_id, 'Local controller assigned the next field sortie', {'area_ha': mission.area_ha, 'return_reason': mission.return_reason, 'battery_id': mission.battery_id, 'battery_soc_start': mission.battery_soc_start, 'battery_soc_end_planned': mission.battery_soc_end_planned, 'energy_used_wh': mission.energy_used_wh})
        mission.state = 'ASSIGNED'
        self._command('DRONE', drone.drone_id, 'TAKE_OFF_AND_EXECUTE_MISSION', mission.mission_id)
        self._change_state(drone, 'IN_FLIGHT', 'DRONE', drone.drone_id, mission.mission_id, 'Drone started the assigned mission')
        mission.transition(self.time_s, 'IN_PROGRESS')
        if self.config['weather']['mode'] != 'none':
            self._start_weather_flight(drone, mission)
            return
        work_finish = self.time_s + mission.outbound_time_s + mission.work_time_s
        arrival = work_finish + mission.return_time_s
        self._schedule(work_finish, 'mission_work_complete', {'drone_id': drone.drone_id, 'mission_id': mission.mission_id})
        self._schedule(arrival, 'return_arrival', {'drone_id': drone.drone_id, 'mission_id': mission.mission_id})
    def _on_mission_work_complete(self, payload: Dict[str, Any]) -> None:
        drone = self.drones[payload['drone_id']]
        mission = self.missions[payload['mission_id']]
        if drone.current_mission_id != mission.mission_id:
            return
        mission.work_finish_time_s = self.time_s
        drone.remaining_area_ha = max(0.0, drone.remaining_area_ha - mission.area_ha)
        self.controller.total_processed_area_ha += mission.area_ha
        self._change_state(drone, 'RETURNING', 'DRONE', drone.drone_id, mission.mission_id, 'Field work finished; drone is returning to the station', {'processed_area_ha': mission.area_ha})
        self._log('FLIGHT', 'RETURN_REQUESTED', 'DRONE', drone.drone_id, related_object_id=mission.mission_id, message='Drone requested a garage cell for return', details={'reason': mission.return_reason})
        drone.cell_request_since_s = self.time_s
        self._request_cell(drone)
        total_area = float(self.config['field']['area_ha'])
        if self.processing_completion_time_s is None and self.controller.total_processed_area_ha >= total_area - 1e-07:
            self.processing_completion_time_s = self.time_s
            self._log('MILESTONE', 'FIELD_PROCESSING_COMPLETE', 'FIELD', 'FIELD-01', message='All planned field area has been processed', details={'area_ha': self.controller.total_processed_area_ha})
    def _on_return_arrival(self, payload: Dict[str, Any]) -> None:
        drone = self.drones[payload['drone_id']]
        mission = self.missions[payload['mission_id']]
        mission.finish_time_s = self.time_s
        self._change_state(mission, 'COMPLETED', 'MISSION', mission.mission_id, drone.drone_id, 'Sortie completed and the drone reached the station', {'area_ha': mission.area_ha, 'energy_used_wh': mission.energy_used_wh})
        if mission.mission_id in self.controller.active_missions:
            self.controller.active_missions.remove(mission.mission_id)
        drone.missions_completed += 1
        drone.current_mission_id = None
        drone.flight_time_s += mission.outbound_time_s + mission.work_time_s + mission.return_time_s
        drone.remaining_liquid_l = max(0.0, drone.remaining_liquid_l - mission.liquid_used_l)
        battery = self.batteries[drone.battery_id or '']
        if mission.battery_id != battery.battery_id:
            raise SimulationError('Mission returned with a different battery')
        capacity_before_ageing_wh = self._effective_battery_capacity_wh(battery)
        battery.soc = max(0.0, battery.soc - mission.energy_used_wh / capacity_before_ageing_wh)
        mission.battery_soc_end_actual = battery.soc
        if battery.soc < float(self.config['battery_model']['reserve_soc']) - 1e-06:
            raise SimulationError('Actual return violated planning reserve')
        battery.cycle_count += 1
        battery.energy_throughput_wh += mission.energy_used_wh
        nominal_capacity_wh = float(self.config['battery_model']['nominal_capacity_wh'])
        battery.equivalent_full_cycles += mission.energy_used_wh / nominal_capacity_wh
        battery_cfg = self.config['battery_model']
        battery.soh = max(float(battery_cfg['minimum_soh']), float(battery_cfg['initial_soh']) - float(battery_cfg['soh_loss_per_equivalent_cycle']) * battery.equivalent_full_cycles)
        drone.remaining_energy_wh = battery.soc * self._effective_battery_capacity_wh(battery)
        ambient = float(battery_cfg['ambient_temperature_c'])
        thermal_fraction = min(1.0, mission.energy_used_wh / nominal_capacity_wh)
        battery.temperature_c = max(battery.temperature_c, ambient + float(battery_cfg['temperature_rise_at_full_discharge_c']) * thermal_fraction)
        drone.arrival_time_s = self.time_s
        drone.arrived_at_station = True
        drone.position = 'STATION_APPROACH'
        drone.ground_cycle_started_s = self.time_s
        self._log('FLIGHT', 'ARRIVED_AT_STATION', 'DRONE', drone.drone_id, related_object_id=mission.mission_id, message='Drone reached the station approach point', details={'remaining_energy_wh': drone.remaining_energy_wh, 'remaining_liquid_l': drone.remaining_liquid_l, 'battery_id': battery.battery_id, 'battery_soc': battery.soc, 'battery_soh': battery.soh, 'battery_temperature_c': battery.temperature_c})
        if drone.assigned_cell_id:
            door = self.doors[self.cells[drone.assigned_cell_id].door_id]
            if door.state == 'OPEN' and self.cells[drone.assigned_cell_id].platform_position == 'OUTSIDE':
                self._start_landing(drone)
            else:
                drone.waiting_cell_since_s = self.time_s
                self._change_state(drone, 'WAITING_FOR_CELL', 'DRONE', drone.drone_id, drone.assigned_cell_id, 'Drone is holding at the station approach while its reserved cell door opens')
        else:
            drone.waiting_cell_since_s = self.time_s
            self._change_state(drone, 'WAITING_FOR_CELL', 'DRONE', drone.drone_id, message='No cell is currently available')
    def _ordered_free_cells(self) -> List[GarageCell]:
        candidates = [cell for cell in self.cells.values() if cell.state == 'FREE' and (not cell.fault_state)]
        if not self.config['station']['third_cell_active']:
            primary_fault = any((c.fault_state for c in self.cells.values() if c.is_preferred))
            if not primary_fault:
                candidates = [c for c in candidates if c.is_preferred]
        policy = self.config['station']['reserve_cell_policy']
        if policy == 'preferred_two':
            return sorted(candidates, key=lambda cell: (not cell.is_preferred, cell.cell_id))
        return sorted(candidates, key=lambda cell: cell.cell_id)
    def _request_cell(self, drone: Drone) -> None:
        self._log('QUEUE', 'CELL_SEARCH_STARTED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', related_object_id=drone.drone_id, message='Controller is searching for a free garage cell', details={'queue_length': len(self.cell_queue)})
        free_cells = self._ordered_free_cells()
        self._log('QUEUE', 'AVAILABLE_CELLS_DETERMINED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', related_object_id=drone.drone_id, message='Controller evaluated current cell availability', details={'available_cell_ids': [cell.cell_id for cell in free_cells]})
        if free_cells and (not self.cell_queue):
            self._reserve_cell(drone, free_cells[0])
            return
        if drone.drone_id not in self._queued_for_cell:
            self._queue_sequence += 1
            heapq.heappush(self.cell_queue, (drone.cell_request_since_s if drone.cell_request_since_s is not None else self.time_s, self._queue_sequence, drone.drone_id))
            self._queued_for_cell.add(drone.drone_id)
            self.resource_conflicts += 1
            self._log('QUEUE', 'CELL_QUEUE_ENTERED', 'DRONE', drone.drone_id, related_object_id='CELL_QUEUE', message='No cell could be allocated; drone entered FIFO queue', details={'queue_length': len(self.cell_queue)})
    def _dispatch_cell_queue(self) -> None:
        while self.cell_queue:
            free_cells = self._ordered_free_cells()
            if not free_cells:
                return
            request_time, _, drone_id = heapq.heappop(self.cell_queue)
            self._queued_for_cell.discard(drone_id)
            drone = self.drones[drone_id]
            if drone.assigned_cell_id is not None:
                continue
            self._log('QUEUE', 'CELL_QUEUE_SELECTED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', related_object_id=drone_id, message='First FIFO request selected for allocation', details={'request_time_s': request_time})
            self._reserve_cell(drone, free_cells[0])
    def _reserve_cell(self, drone: Drone, cell: GarageCell) -> None:
        if drone.assigned_cell_id is not None or cell.state != 'FREE':
            raise SimulationError('Invalid cell reservation')
        cell.occupied_by = drone.drone_id
        cell.reservation_count += 1
        drone.assigned_cell_id = cell.cell_id
        self._change_state(cell, 'RESERVED', 'GARAGE_CELL', cell.cell_id, drone.drone_id, 'Cell reserved by the local controller')
        request_started = drone.cell_request_since_s if drone.cell_request_since_s is not None else self.time_s
        allocation_delay_s = max(0.0, self.time_s - request_started)
        drone.cell_allocation_delay_s += allocation_delay_s
        self.cell_allocation_delays_s.append(allocation_delay_s)
        drone.cell_request_since_s = None
        self._log('RESOURCE', 'CELL_RESERVED', 'GARAGE_CELL', cell.cell_id, related_object_id=drone.drone_id, message='A free cell was assigned to the returning drone', details={'allocation_delay_s': allocation_delay_s, 'reserve_cell_policy': self.config['station']['reserve_cell_policy'], 'preferred': cell.is_preferred})
        self._command('DOOR', cell.door_id, 'OPEN_FOR_ARRIVAL', drone.drone_id)
        self._start_door_operation(cell, 'OPEN', 'ARRIVAL')
    def _start_door_operation(self, cell: GarageCell, target_state: str, continuation: str) -> None:
        door = self.doors[cell.door_id]
        if door.fault_state:
            raise SimulationError(f'Door {door.door_id} is in fault state')
        if target_state == 'OPEN':
            if door.state == 'OPEN':
                self._after_door_open(cell, continuation)
                return
            duration = self._duration('door_open_s')
            intermediate = 'OPENING'
        else:
            if door.state == 'CLOSED':
                self._after_door_closed(cell, continuation)
                return
            duration = self._duration('door_close_s')
            intermediate = 'CLOSING'
        door.operation_token += 1
        token = door.operation_token
        door.operation_end_time_s = self.time_s + duration
        self._change_state(door, intermediate, 'DOOR', door.door_id, cell.occupied_by or '', f'Door started {intermediate.lower()}', {'duration_s': duration, 'continuation': continuation})
        self._schedule(self.time_s + duration, 'door_operation_complete', {'cell_id': cell.cell_id, 'target_state': target_state, 'continuation': continuation, 'token': token})
    def _on_door_operation_complete(self, payload: Dict[str, Any]) -> None:
        cell = self.cells[payload['cell_id']]
        door = self.doors[cell.door_id]
        if payload['token'] != door.operation_token:
            return
        target = payload['target_state']
        door.operation_end_time_s = None
        self._change_state(door, target, 'DOOR', door.door_id, cell.occupied_by or '', f'Door reached {target.lower()} position')
        if target == 'OPEN':
            door.open_count += 1
            self._after_door_open(cell, payload['continuation'])
        else:
            self._after_door_closed(cell, payload['continuation'])
    def _after_door_open(self, cell: GarageCell, continuation: str) -> None:
        if continuation in {'ARRIVAL', 'DEPARTURE'}:
            self._start_platform(cell, 'OUTSIDE', continuation)
    def _start_platform(self, cell: GarageCell, target: str, continuation: str) -> None:
        if cell.platform_position == target:
            self._after_platform(cell, continuation)
            return
        if cell.platform_position not in {'INSIDE', 'OUTSIDE'}:
            raise SimulationError('Platform is already moving')
        if self.doors[cell.door_id].state != 'OPEN':
            raise SimulationError('Platform cannot move through a closed door')
        if self.gantry.target_cell_id == cell.cell_id:
            raise SimulationError('Platform cannot move while manipulator occupies its cell')
        key = 'platform_out_s' if target == 'OUTSIDE' else 'platform_in_s'
        cell.platform_position = 'MOVING_OUT' if target == 'OUTSIDE' else 'MOVING_IN'
        if cell.occupied_by:
            drone = self.drones[cell.occupied_by]
            self._change_state(drone, cell.platform_position, 'DRONE', drone.drone_id, cell.cell_id, 'Independent platform travel started')
        self._log('PLATFORM', 'PLATFORM_MOVE_STARTED', 'GARAGE_CELL', cell.cell_id, related_object_id=cell.occupied_by or '', details={'target': target, 'continuation': continuation})
        self._schedule(self.time_s + self._duration(key), 'platform_complete', {'cell_id': cell.cell_id, 'target': target, 'continuation': continuation})
    def _after_platform(self, cell: GarageCell, continuation: str) -> None:
        drone = self.drones[cell.occupied_by] if cell.occupied_by else None
        if continuation == 'ARRIVAL':
            if drone and drone.arrived_at_station:
                self._start_landing(drone)
        elif continuation == 'INBOUND':
            self._start_door_operation(cell, 'CLOSED', 'INBOUND')
        elif continuation == 'DEPARTURE':
            self._start_final_check(drone)
        elif continuation == 'RELEASE':
            self._start_door_operation(cell, 'CLOSED', 'RELEASE')
    def _on_platform_complete(self, payload: Dict[str, Any]) -> None:
        cell = self.cells[payload['cell_id']]
        cell.platform_position = payload['target']
        self._log('PLATFORM', 'PLATFORM_MOVE_COMPLETE', 'GARAGE_CELL', cell.cell_id, related_object_id=cell.occupied_by or '', details=payload)
        self._after_platform(cell, payload['continuation'])
    def _after_door_closed(self, cell: GarageCell, continuation: str) -> None:
        if continuation == 'INBOUND':
            if not cell.occupied_by:
                raise SimulationError('Inbound cell has no drone')
            drone = self.drones[cell.occupied_by]
            self._on_inbound_complete(drone)
        elif continuation == 'RELEASE':
            self._change_state(cell, 'FAULT' if cell.fault_state else 'FREE', 'GARAGE_CELL', cell.cell_id, message='Cell released after drone departure and door closure')
            self._dispatch_cell_queue()
    def _start_landing(self, drone: Drone) -> None:
        cell = self.cells[drone.assigned_cell_id or '']
        if drone.waiting_cell_since_s is not None:
            wait_s = max(0.0, self.time_s - drone.waiting_cell_since_s)
            drone.waiting_cell_time_s += wait_s
            self.cell_waits_s.append(wait_s)
            drone.waiting_cell_since_s = None
            self._log('QUEUE', 'CELL_APPROACH_WAIT_ENDED', 'DRONE', drone.drone_id, related_object_id=cell.cell_id, message='Station-approach holding ended; landing may begin', details={'wait_s': wait_s})
        self._command('DRONE', drone.drone_id, 'LAND_IN_RESERVED_CELL', cell.cell_id)
        self._change_state(drone, 'LANDING', 'DRONE', drone.drone_id, cell.cell_id, 'Drone entered the reserved cell')
        self._schedule(self.time_s + self._duration('landing_s'), 'landing_complete', {'drone_id': drone.drone_id})
    def _on_landing_complete(self, drone: Drone) -> None:
        cell = self.cells[drone.assigned_cell_id or '']
        drone.position = cell.cell_id
        self._change_state(cell, 'OCCUPIED', 'GARAGE_CELL', cell.cell_id, drone.drone_id, 'Drone completed landing and is fixed in the cell')
        self._change_state(drone, 'IDENTIFICATION', 'DRONE', drone.drone_id, cell.cell_id, 'Cell started digital drone identification')
        self._command('GARAGE_CELL', cell.cell_id, 'IDENTIFY_DRONE', drone.drone_id)
        self._schedule(self.time_s + self._duration('identification_s'), 'identification_complete', {'drone_id': drone.drone_id})
    def _on_identification_complete(self, drone: Drone) -> None:
        cell = self.cells[drone.assigned_cell_id or '']
        self._log('SAFETY', 'IDENTIFICATION_CONFIRMED', 'DRONE', drone.drone_id, related_object_id=cell.cell_id, message='Drone identity matches the reservation')
        self._command('DOOR', cell.door_id, 'CLOSE_AFTER_ARRIVAL', drone.drone_id)
        self._change_state(drone, 'FIXING', 'DRONE', drone.drone_id, cell.cell_id, 'Rotors stopped; platform fixation started')
        self._schedule(self.time_s + self._duration('fixation_s'), 'fixation_complete', {'drone_id': drone.drone_id})
    def _on_inbound_complete(self, drone: Drone) -> None:
        cell = self.cells[drone.assigned_cell_id or '']
        self._log('OPERATION', 'INBOUND_COMPLETE', 'DRONE', drone.drone_id, related_object_id=cell.cell_id, message='Drone is fixed on its platform inside the cell')
        if drone.final_return and drone.remaining_area_ha <= EPS:
            self._change_state(drone, 'PARKED', 'DRONE', drone.drone_id, cell.cell_id, 'Final mission complete; drone remains parked without turnaround service')
            if drone.ground_cycle_started_s is not None:
                self.final_recovery_durations_s.append(self.time_s - drone.ground_cycle_started_s)
                drone.ground_cycle_started_s = None
            return
        drone.refill_complete = False
        drone.battery_service_complete = False
        self._change_state(cell, 'WAITING_SERVICE', 'GARAGE_CELL', cell.cell_id, drone.drone_id, 'Drone waits inside for the shared manipulator')
        self._request_gantry(drone)
    def _on_refill_complete(self, drone: Drone) -> None:
        drone.refill_complete = True
        drone.remaining_liquid_l = float(self.config['flight']['tank_capacity_l'])
        self._log('OPERATION', 'REFILL_COMPLETE', 'DRONE', drone.drone_id, related_object_id=drone.assigned_cell_id or '', message='Working-liquid tank was refilled')
    def _request_gantry(self, drone: Drone) -> None:
        drone.waiting_gantry_since_s = self.time_s
        self._change_state(drone, 'WAITING_FOR_GANTRY', 'DRONE', drone.drone_id, drone.assigned_cell_id or '', 'Drone requested the shared service gantry')
        self._log('QUEUE', 'GANTRY_SEARCH_STARTED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', related_object_id=drone.drone_id, message='Controller checked gantry and ready-battery availability', details={'gantry_state': self.gantry.state, 'ready_battery_ids': self._ready_battery_ids(), 'queue_length': len(self.gantry_queue)})
        ready_batteries = self._ready_battery_ids()
        if not ready_batteries:
            self.battery_shortage_events += 1
        if self.gantry.state != 'IDLE' or not ready_batteries:
            self.resource_conflicts += 1
            self._log('RESOURCE', 'RESOURCE_CONFLICT', 'SERVICE_GANTRY', self.gantry.gantry_id, old_state=self.gantry.state, new_state=self.gantry.state, related_object_id=drone.drone_id, message='Immediate gantry allocation was impossible; request remains in FIFO queue', details={'ready_battery_ids': self._ready_battery_ids(), 'queue_length_before': len(self.gantry_queue)})
        self._queue_sequence += 1
        heapq.heappush(self.gantry_queue, (drone.arrival_time_s, self._queue_sequence, drone.drone_id))
        self._queued_for_gantry.add(drone.drone_id)
        self._dispatch_gantry_queue()
    def _ready_battery_ids(self) -> List[str]:
        battery_cfg = self.config['battery_model']
        dispatch_soc = float(battery_cfg['dispatch_soc'])
        maximum_temperature = float(battery_cfg['max_install_temperature_c'])
        return sorted((battery_id for battery_id in self.rack.battery_ids if self.batteries[battery_id].state == 'READY' and (not self.batteries[battery_id].fault_state) and (self.batteries[battery_id].soc >= dispatch_soc - EPS) and (self.batteries[battery_id].temperature_c <= maximum_temperature + EPS)))
    def _dispatch_gantry_queue(self) -> None:
        if not self.gantry_queue:
            return
        if self.gantry.state != 'IDLE' or self.gantry.fault_state:
            return
        ready = self._ready_battery_ids()
        if not ready:
            first_drone_id = self.gantry_queue[0][2]
            self._log('QUEUE', 'READY_BATTERY_UNAVAILABLE', 'BATTERY_RACK', self.rack.rack_id, related_object_id=first_drone_id, message='Gantry queue is blocked until a battery becomes ready')
            return
        request_time, _, drone_id = heapq.heappop(self.gantry_queue)
        self._queued_for_gantry.discard(drone_id)
        drone = self.drones[drone_id]
        battery = self.batteries[ready[0]]
        battery.assigned_drone_id = drone_id
        battery.location = self.gantry.gantry_id
        self._change_state(battery, 'RESERVED', 'BATTERY', battery.battery_id, drone_id, 'Ready battery reserved for the first feasible gantry request', {'soc': battery.soc, 'temperature_c': battery.temperature_c})
        drone.pending_battery_id = battery.battery_id
        wait_s = max(0.0, self.time_s - drone.waiting_gantry_since_s)
        drone.waiting_gantry_time_s += wait_s
        self.gantry_waits_s.append(wait_s)
        drone.waiting_gantry_since_s = None
        self._log('QUEUE', 'GANTRY_QUEUE_SELECTED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', related_object_id=drone_id, message='First feasible FIFO gantry request selected', details={'wait_s': wait_s, 'battery_id': battery.battery_id})
        self.gantry.target_cell_id = drone.assigned_cell_id
        self.gantry.target_drone_id = drone_id
        cell = self.cells[drone.assigned_cell_id]
        if cell.platform_position != 'INSIDE':
            raise SimulationError('Service requires a stationary platform inside')
        self._change_state(cell, 'SERVICING', 'GARAGE_CELL', cell.cell_id, drone_id, 'Shared manipulator allocated')
        self._start_manipulator_phase(drone, 'MOVE', float(self.config['timing']['manipulator_move_s']))
    def _start_manipulator_phase(self, drone: Drone, phase: str, duration: float) -> None:
        duration = self._sample_duration(duration, phase.lower())
        if phase == 'SERVICE_CHECK' and self.config['randomness']['enabled'] and self.config['randomness']['service_delays']:
            rc = self.config['randomness']
            if self.delay_rng.random() < rc['delay_probability']:
                extra = self.delay_rng.uniform(rc['delay_min_s'], rc['delay_max_s'])
                duration += extra
                self.service_delay_events += 1
                self._log('DELAY', 'SERVICE_DELAY', 'DRONE', drone.drone_id, details={'additional_s': extra})
        self.gantry.current_operation = phase
        self.gantry.operation_token += 1
        self.gantry.operation_end_time_s = self.time_s + duration
        self._change_state(self.gantry, 'SERVICING', 'SERVICE_GANTRY', self.gantry.gantry_id, drone.drone_id, f'Manipulator phase: {phase}', {'duration_s': duration})
        self._change_state(drone, phase, 'DRONE', drone.drone_id, drone.assigned_cell_id or '', f'Manipulator phase: {phase}')
        self._log('OPERATION', 'MANIPULATOR_PHASE_STARTED', 'SERVICE_GANTRY', self.gantry.gantry_id, related_object_id=drone.drone_id, details={'phase': phase, 'duration_s': duration})
        self._schedule(self.time_s + duration, 'manipulator_phase_complete', {'drone_id': drone.drone_id, 'token': self.gantry.operation_token})
    def _on_manipulator_phase_complete(self, payload: Dict[str, Any]) -> None:
        if payload['token'] != self.gantry.operation_token:
            return
        drone = self.drones[payload['drone_id']]
        phase = self.gantry.current_operation
        timing = self.config['timing']
        if phase == 'MOVE':
            self._start_manipulator_phase(drone, 'BATTERY_SWAP', float(timing['battery_swap_s']))
        elif phase == 'BATTERY_SWAP':
            self._on_battery_swap_complete(payload)
            self._start_manipulator_phase(drone, 'REFILL', float(timing['refill_s']))
        elif phase == 'REFILL':
            self._on_refill_complete(drone)
            self._start_manipulator_phase(drone, 'SERVICE_CHECK', float(timing['service_check_s']))
        elif phase == 'SERVICE_CHECK':
            self._start_manipulator_phase(drone, 'CLEAR', float(timing['manipulator_clear_s']))
        elif phase == 'CLEAR':
            self.gantry.current_operation = None
            self.gantry.target_cell_id = None
            self.gantry.target_drone_id = None
            self.gantry.operation_end_time_s = None
            self._change_state(self.gantry, 'IDLE', 'SERVICE_GANTRY', self.gantry.gantry_id, drone.drone_id, 'Manipulator released after complete service')
            self._log('OPERATION', 'SERVICE_COMPLETE', 'DRONE', drone.drone_id, related_object_id=drone.assigned_cell_id or '')
            self._prepare_departure_or_wait(drone)
            self._dispatch_gantry_queue()
        else:
            raise SimulationError(f'Unknown manipulator phase: {phase}')
    def _on_battery_swap_complete(self, payload: Dict[str, Any]) -> None:
        if payload['token'] != self.gantry.operation_token:
            return
        drone = self.drones[payload['drone_id']]
        if not drone.pending_battery_id or not drone.battery_id:
            raise SimulationError('Battery swap lacks old or new battery')
        old_battery = self.batteries[drone.battery_id]
        new_battery = self.batteries[drone.pending_battery_id]
        old_battery.location = self.gantry.gantry_id
        self._change_state(old_battery, 'REMOVED', 'BATTERY', old_battery.battery_id, drone.drone_id, 'Discharged battery removed from the drone', {'soc': old_battery.soc, 'temperature_c': old_battery.temperature_c})
        old_battery.assigned_drone_id = None
        if new_battery.battery_id in self.rack.battery_ids:
            self.rack.battery_ids.remove(new_battery.battery_id)
        new_battery.assigned_drone_id = drone.drone_id
        new_battery.location = drone.drone_id
        new_battery.installation_count += 1
        self._change_state(new_battery, 'IN_DRONE', 'BATTERY', new_battery.battery_id, drone.drone_id, 'Prepared battery installed and locked', {'soc': new_battery.soc, 'soh': new_battery.soh})
        if old_battery.battery_id not in self.rack.battery_ids:
            self.rack.battery_ids.append(old_battery.battery_id)
        old_battery.location = self.rack.rack_id
        drone.battery_id = new_battery.battery_id
        drone.pending_battery_id = None
        drone.remaining_energy_wh = new_battery.soc * self._effective_battery_capacity_wh(new_battery)
        drone.battery_service_complete = True
        self._log('OPERATION', 'BATTERY_SENT_TO_RACK', 'BATTERY_RACK', self.rack.rack_id, related_object_id=old_battery.battery_id, message='Removed battery entered the explicit cooling/charging pipeline', details={'soc': old_battery.soc, 'temperature_c': old_battery.temperature_c})
        self._enqueue_battery_for_cooling(old_battery)
        self._log('OPERATION', 'BATTERY_SWAP_COMPLETE', 'DRONE', drone.drone_id, related_object_id=new_battery.battery_id, message='Battery replacement complete; manipulator continues refill')
    def _cooling_duration_s(self, battery: Battery) -> float:
        cfg = self.config['battery_model']
        target_c = float(cfg['charge_start_max_temperature_c'])
        excess_c = max(0.0, battery.temperature_c - target_c)
        if excess_c <= EPS:
            return 0.0
        reference_excess_c = max(1.0, float(cfg['hot_battery_temperature_c']) - target_c)
        duration_s = float(cfg['cooling_reference_s']) * (excess_c / reference_excess_c)
        duration_s = max(float(cfg['minimum_cooling_s']), duration_s)
        if float(cfg['ambient_temperature_c']) >= float(cfg['hot_ambient_threshold_c']):
            duration_s *= float(cfg['hot_cooling_time_multiplier'])
        return duration_s
    def _enqueue_battery_for_cooling(self, battery: Battery) -> None:
        if battery.battery_id in self._queued_for_cooling:
            return
        battery.cooling_wait_started_s = self.time_s
        battery.location = self.rack.rack_id
        self._change_state(battery, 'WAITING_FOR_COOLING', 'BATTERY', battery.battery_id, self.rack.rack_id, 'Battery queued for a finite cooling slot')
        self._queue_sequence += 1
        heapq.heappush(self.cooling_queue, (self.time_s, self._queue_sequence, battery.battery_id))
        self._queued_for_cooling.add(battery.battery_id)
        self._dispatch_cooling_queue()
    def _dispatch_cooling_queue(self) -> None:
        while self.cooling_queue:
            free_slots = sorted((slot for slot in self.cooling_slots.values() if slot.state == 'IDLE' and slot.battery_id is None), key=lambda item: item.slot_id)
            if not free_slots:
                return
            request_time, _, battery_id = heapq.heappop(self.cooling_queue)
            self._queued_for_cooling.discard(battery_id)
            battery = self.batteries[battery_id]
            if battery.state != 'WAITING_FOR_COOLING':
                continue
            slot = free_slots[0]
            wait_s = max(0.0, self.time_s - request_time)
            self.cooling_waits_s.append(wait_s)
            battery.cooling_wait_started_s = None
            duration_s = self._cooling_duration_s(battery)
            self.cooling_durations_s.append(duration_s)
            slot.battery_id = battery_id
            slot.operation_token += 1
            token = slot.operation_token
            slot.operation_end_time_s = self.time_s + duration_s
            battery.location = slot.slot_id
            self._change_state(slot, 'COOLING', 'COOLING_SLOT', slot.slot_id, battery_id, 'Cooling slot started active battery cooling', {'duration_s': duration_s, 'queue_wait_s': wait_s})
            self._change_state(battery, 'COOLING', 'BATTERY', battery_id, slot.slot_id, 'Battery cooling started', {'duration_s': duration_s, 'start_temperature_c': battery.temperature_c})
            self._log('OPERATION', 'BATTERY_COOLING_STARTED', 'BATTERY', battery_id, related_object_id=slot.slot_id, details={'duration_s': duration_s, 'queue_wait_s': wait_s})
            self._schedule(self.time_s + duration_s, 'battery_cooling_complete', {'slot_id': slot.slot_id, 'battery_id': battery_id, 'token': token})
    def _on_battery_cooling_complete(self, payload: Dict[str, Any]) -> None:
        slot = self.cooling_slots[payload['slot_id']]
        if payload['token'] != slot.operation_token:
            return
        if slot.battery_id != payload['battery_id']:
            raise SimulationError('Cooling slot/battery assignment is inconsistent')
        battery = self.batteries[payload['battery_id']]
        battery.temperature_c = min(battery.temperature_c, float(self.config['battery_model']['charge_start_max_temperature_c']))
        slot.battery_id = None
        slot.operation_end_time_s = None
        slot.completed_operations += 1
        self._change_state(slot, 'IDLE', 'COOLING_SLOT', slot.slot_id, battery.battery_id, 'Cooling slot released')
        self._log('OPERATION', 'BATTERY_COOLING_COMPLETE', 'BATTERY', battery.battery_id, related_object_id=slot.slot_id, details={'temperature_c': battery.temperature_c})
        self._enqueue_battery_for_charging(battery)
        self._dispatch_cooling_queue()
    def _enqueue_battery_for_charging(self, battery: Battery) -> None:
        if battery.battery_id in self._queued_for_charging:
            return
        battery.charge_wait_started_s = self.time_s
        battery.location = self.rack.rack_id
        self._change_state(battery, 'WAITING_FOR_CHARGER', 'BATTERY', battery.battery_id, self.rack.rack_id, 'Cooled battery queued for a charger channel', {'soc': battery.soc})
        self._queue_sequence += 1
        heapq.heappush(self.charger_queue, (self.time_s, self._queue_sequence, battery.battery_id))
        self._queued_for_charging.add(battery.battery_id)
        self._dispatch_charger_queue()
    def _charge_duration_s(self, battery: Battery) -> float:
        cfg = self.config['battery_model']
        target_soc = float(cfg['charge_target_soc'])
        delta_soc = max(0.0, target_soc - battery.soc)
        if delta_soc <= EPS:
            return 0.0
        reference_delta = float(cfg['charge_reference_to_soc']) - float(cfg['charge_reference_from_soc'])
        profile_time_s = float(cfg['charge_reference_time_s']) * (delta_soc / reference_delta)
        pack_energy_wh = delta_soc * self._effective_battery_capacity_wh(battery)
        power_time_s = pack_energy_wh / (float(cfg['charger_power_kw']) * 1000.0 * float(cfg['charge_efficiency'])) * 3600.0
        duration_s = max(profile_time_s, power_time_s)
        if float(cfg['ambient_temperature_c']) >= float(cfg['hot_ambient_threshold_c']):
            duration_s *= float(cfg['hot_charge_time_multiplier'])
        return duration_s
    def _dispatch_charger_queue(self) -> None:
        while self.charger_queue:
            free_chargers = sorted((charger for charger in self.chargers.values() if charger.state == 'IDLE' and charger.battery_id is None), key=lambda item: item.charger_id)
            if not free_chargers:
                return
            request_time, _, battery_id = heapq.heappop(self.charger_queue)
            self._queued_for_charging.discard(battery_id)
            battery = self.batteries[battery_id]
            if battery.state != 'WAITING_FOR_CHARGER':
                continue
            charger = free_chargers[0]
            wait_s = max(0.0, self.time_s - request_time)
            self.charger_waits_s.append(wait_s)
            battery.charge_wait_started_s = None
            battery.charge_started_soc = battery.soc
            duration_s = self._charge_duration_s(battery)
            self.charge_durations_s.append(duration_s)
            charger.battery_id = battery_id
            charger.operation_token += 1
            token = charger.operation_token
            charger.operation_end_time_s = self.time_s + duration_s
            battery.location = charger.charger_id
            self._change_state(charger, 'CHARGING', 'CHARGER', charger.charger_id, battery_id, 'Charger channel started charging', {'duration_s': duration_s, 'queue_wait_s': wait_s})
            self._change_state(battery, 'CHARGING', 'BATTERY', battery_id, charger.charger_id, 'Battery charging started', {'duration_s': duration_s, 'start_soc': battery.soc, 'target_soc': self.config['battery_model']['charge_target_soc']})
            self._log('OPERATION', 'BATTERY_CHARGING_STARTED', 'BATTERY', battery_id, related_object_id=charger.charger_id, details={'duration_s': duration_s, 'queue_wait_s': wait_s})
            self._schedule(self.time_s + duration_s, 'battery_charge_complete', {'charger_id': charger.charger_id, 'battery_id': battery_id, 'token': token})
    def _on_battery_charge_complete(self, payload: Dict[str, Any]) -> None:
        charger = self.chargers[payload['charger_id']]
        if payload['token'] != charger.operation_token:
            return
        if charger.battery_id != payload['battery_id']:
            raise SimulationError('Charger/battery assignment is inconsistent')
        battery = self.batteries[payload['battery_id']]
        cfg = self.config['battery_model']
        start_soc = battery.charge_started_soc if battery.charge_started_soc is not None else battery.soc
        target_soc = float(cfg['charge_target_soc'])
        delivered_wh = max(0.0, target_soc - start_soc) * self._effective_battery_capacity_wh(battery)
        grid_wh = delivered_wh / float(cfg['charge_efficiency'])
        battery.soc = target_soc
        battery.charged_energy_wh += delivered_wh
        battery.temperature_c = max(float(cfg['ambient_temperature_c']), float(cfg['charge_end_temperature_c']))
        battery.location = self.rack.rack_id
        battery.charge_started_soc = None
        battery.ready_at_s = self.time_s
        self._change_state(battery, 'READY', 'BATTERY', battery.battery_id, self.rack.rack_id, 'Battery reached dispatch SoC and is ready for reservation', {'soc': battery.soc, 'soh': battery.soh, 'temperature_c': battery.temperature_c, 'delivered_energy_wh': delivered_wh})
        charger.battery_id = None
        charger.operation_end_time_s = None
        charger.completed_operations += 1
        charger.delivered_energy_wh += delivered_wh
        charger.grid_energy_wh += grid_wh
        self._change_state(charger, 'IDLE', 'CHARGER', charger.charger_id, battery.battery_id, 'Charger channel released')
        self._log('OPERATION', 'BATTERY_READY', 'BATTERY', battery.battery_id, related_object_id=charger.charger_id, details={'soc': battery.soc, 'delivered_energy_wh': delivered_wh})
        self._dispatch_charger_queue()
        self._dispatch_gantry_queue()
    def _prepare_departure_or_wait(self, drone: Drone) -> None:
        if self.controller.communication_state == 'OFFLINE' and bool(self.config['communication']['server_confirmation_required_for_new_mission']):
            drone.waiting_for_new_mission = True
            self._change_state(drone, 'PARKED', 'DRONE', drone.drone_id, drone.assigned_cell_id or '', 'Turnaround complete; departure waits for server confirmation')
            return
        self._start_departure(drone)
    def _start_departure(self, drone: Drone) -> None:
        drone.waiting_for_new_mission = False
        cell = self.cells[drone.assigned_cell_id or '']
        self._start_door_operation(cell, 'OPEN', 'DEPARTURE')
    def _start_final_check(self, drone: Drone) -> None:
        self._change_state(drone, 'FINAL_CHECK', 'DRONE', drone.drone_id, drone.assigned_cell_id or '', 'Final readiness and interlock check started')
        self._command('LOCAL_CONTROLLER', 'CONTROLLER-01', 'VERIFY_DEPARTURE_READINESS', drone.drone_id, {'battery_id': drone.battery_id, 'cell_id': drone.assigned_cell_id, 'communication_state': self.controller.communication_state})
        self._schedule(self.time_s + self._duration('final_check_s'), 'final_check_complete', {'drone_id': drone.drone_id})
    def _on_final_check_complete(self, drone: Drone) -> None:
        if not drone.battery_id:
            raise SimulationError('Final check failed: no battery installed')
        battery = self.batteries[drone.battery_id]
        battery_cfg = self.config['battery_model']
        if battery.state != 'IN_DRONE' or battery.assigned_drone_id != drone.drone_id:
            raise SimulationError('Final check failed: battery assignment mismatch')
        if battery.soc < float(battery_cfg['dispatch_soc']) - EPS:
            raise SimulationError('Final check failed: battery SoC below dispatch threshold')
        if battery.temperature_c > float(battery_cfg['max_install_temperature_c']) + EPS:
            raise SimulationError('Final check failed: battery temperature is too high')
        self._log('SAFETY', 'FINAL_CHECK_PASSED', 'DRONE', drone.drone_id, related_object_id=drone.assigned_cell_id or '', message='Battery lock, liquid level and mission data are valid', details={'battery_id': battery.battery_id, 'battery_soc': battery.soc, 'battery_temperature_c': battery.temperature_c})
        self._command('DRONE', drone.drone_id, 'TAKE_OFF', drone.assigned_cell_id or '')
        self._change_state(drone, 'MISSION_ASSIGNED', 'DRONE', drone.drone_id, drone.assigned_cell_id or '', 'Drone received permission to leave the cell')
        self._schedule(self.time_s + self._duration('takeoff_s'), 'takeoff_complete', {'drone_id': drone.drone_id})
    def _on_takeoff_complete(self, drone: Drone) -> None:
        cell = self.cells[drone.assigned_cell_id or '']
        if drone.ground_cycle_started_s is not None:
            self.ground_cycle_durations_s.append(self.time_s - drone.ground_cycle_started_s)
            drone.ground_cycle_started_s = None
        cell.occupied_by = None
        drone.assigned_cell_id = None
        drone.position = 'FIELD'
        self._change_state(cell, 'BLOCKED', 'GARAGE_CELL', cell.cell_id, drone.drone_id, 'Drone departed; cell remains blocked until its door closes')
        self._command('DOOR', cell.door_id, 'CLOSE_AFTER_DEPARTURE', drone.drone_id)
        self._start_platform(cell, 'INSIDE', 'RELEASE')
        self._start_next_mission(drone)
    def _on_communication_lost(self) -> None:
        if self.controller.communication_state == 'OFFLINE':
            return
        self.controller.communication_state = 'OFFLINE'
        self.communication_loss_count += 1
        self._log('COMMUNICATION', 'SATELLITE_LINK_LOST', 'LOCAL_CONTROLLER', 'CONTROLLER-01', 'ONLINE', 'OFFLINE', message='Local controller continues flights, returns and ground service without server dependency')
    def _on_communication_restored(self) -> None:
        if self.controller.communication_state == 'ONLINE':
            return
        self.controller.communication_state = 'ONLINE'
        self._log('COMMUNICATION', 'SATELLITE_LINK_RESTORED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', 'OFFLINE', 'ONLINE', message='Buffered event journal is marked for synchronization', details={'buffered_events': len(self.records)})
        self._log('COMMUNICATION', 'EVENT_LOG_SYNCHRONIZED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', message='Local event journal synchronization completed')
        for drone in self.drones.values():
            if drone.waiting_for_new_mission:
                if drone.assigned_cell_id:
                    self._start_departure(drone)
                else:
                    self._start_next_mission(drone)
    def _on_failure_start(self, failure: Dict[str, Any]) -> None:
        kind = failure['object_type'].upper()
        object_id = failure['object_id']
        if kind == 'GARAGE_CELL':
            cell = self.cells[object_id]
            cell.fault_state = True
            if cell.state == 'FREE':
                self._change_state(cell, 'FAULT', 'GARAGE_CELL', object_id, message='Configured cell fault started')
            else:
                self._log('FAULT', 'FAULT_LATCHED', 'GARAGE_CELL', object_id, old_state=cell.state, new_state=cell.state, related_object_id=cell.occupied_by or '', message='Cell will finish the safe local cycle but accepts no new drone')
        elif kind == 'SERVICE_GANTRY':
            self.gantry.fault_state = True
            if self.gantry.state == 'SERVICING':
                self.gantry.paused_remaining_s = max(0.0, (self.gantry.operation_end_time_s or self.time_s) - self.time_s)
                self.gantry.operation_token += 1
            self._change_state(self.gantry, 'FAULT', 'SERVICE_GANTRY', self.gantry.gantry_id, self.gantry.target_drone_id or '', 'Configured gantry fault started; current operation is paused', {'remaining_operation_s': self.gantry.paused_remaining_s})
        else:
            raise ValueError(f'Unsupported failure object_type: {kind}')
    def _on_failure_end(self, failure: Dict[str, Any]) -> None:
        kind = failure['object_type'].upper()
        object_id = failure['object_id']
        if kind == 'GARAGE_CELL':
            cell = self.cells[object_id]
            cell.fault_state = False
            target = 'FREE' if cell.occupied_by is None else 'OCCUPIED'
            if cell.state == 'FAULT':
                self._change_state(cell, target, 'GARAGE_CELL', object_id, cell.occupied_by or '', 'Cell fault cleared')
            else:
                self._log('FAULT', 'FAULT_CLEARED', 'GARAGE_CELL', object_id, old_state=cell.state, new_state=cell.state, message='Latched cell fault cleared')
            self._dispatch_cell_queue()
        elif kind == 'SERVICE_GANTRY':
            self.gantry.fault_state = False
            if self.gantry.current_operation and self.gantry.target_drone_id:
                self.gantry.operation_token += 1
                token = self.gantry.operation_token
                remaining = self.gantry.paused_remaining_s
                self.gantry.operation_end_time_s = self.time_s + remaining
                self._change_state(self.gantry, 'SERVICING', 'SERVICE_GANTRY', self.gantry.gantry_id, self.gantry.target_drone_id, 'Gantry fault cleared; paused operation resumed', {'remaining_operation_s': remaining})
                self._schedule(self.time_s + remaining, 'manipulator_phase_complete', {'drone_id': self.gantry.target_drone_id, 'token': token})
                self.gantry.paused_remaining_s = 0.0
            else:
                self._change_state(self.gantry, 'IDLE', 'SERVICE_GANTRY', self.gantry.gantry_id, message='Gantry fault cleared')
                self._dispatch_gantry_queue()
    def _handle_event(self, event: ScheduledEvent) -> None:
        kind = event.kind
        payload = event.payload
        if kind == 'simulation_start':
            self._log('SYSTEM', 'SIMULATION_STARTED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', message='Droneport simulation started', details={'scenario': self.scenario_name})
            for drone_id in self.active_drone_ids:
                drone = self.drones[drone_id]
                self._change_state(drone, 'TAKING_OFF', 'DRONE', drone_id, message='First takeoff from pre-prepared external position')
                self._schedule(self.time_s + self._duration('takeoff_s'), 'initial_takeoff_complete', {'drone_id': drone_id})
        elif kind == 'weather_changed':
            self._log('WEATHER', 'WEATHER_CHANGED', 'WEATHER', 'COMMON', details=payload)
        elif kind == 'flight_step':
            self._on_flight_step(payload)
        elif kind == 'initial_takeoff_complete':
            self._start_next_mission(self.drones[payload['drone_id']])
        elif kind == 'mission_work_complete':
            self._on_mission_work_complete(payload)
        elif kind == 'return_arrival':
            self._on_return_arrival(payload)
        elif kind == 'platform_complete':
            self._on_platform_complete(payload)
        elif kind == 'fixation_complete':
            drone = self.drones[payload['drone_id']]
            self._start_platform(self.cells[drone.assigned_cell_id], 'INSIDE', 'INBOUND')
        elif kind == 'manipulator_phase_complete':
            self._on_manipulator_phase_complete(payload)
        elif kind == 'door_operation_complete':
            self._on_door_operation_complete(payload)
        elif kind == 'landing_complete':
            self._on_landing_complete(self.drones[payload['drone_id']])
        elif kind == 'identification_complete':
            self._on_identification_complete(self.drones[payload['drone_id']])
        elif kind == 'refill_complete':
            self._on_refill_complete(self.drones[payload['drone_id']])
        elif kind == 'battery_cooling_complete':
            self._on_battery_cooling_complete(payload)
        elif kind == 'battery_charge_complete':
            self._on_battery_charge_complete(payload)
        elif kind == 'final_check_complete':
            self._on_final_check_complete(self.drones[payload['drone_id']])
        elif kind == 'takeoff_complete':
            self._on_takeoff_complete(self.drones[payload['drone_id']])
        elif kind == 'communication_lost':
            self._on_communication_lost()
        elif kind == 'communication_restored':
            self._on_communication_restored()
        elif kind == 'failure_start':
            self._on_failure_start(payload['failure'])
        elif kind == 'failure_end':
            self._on_failure_end(payload['failure'])
        else:
            raise SimulationError(f'Unknown scheduled event: {kind}')
    def _check_invariants(self) -> None:
        self.invariant_checks += 1
        try:
            occupants = [cell.occupied_by for cell in self.cells.values() if cell.occupied_by is not None]
            if len(occupants) != len(set(occupants)):
                raise SimulationError('One drone occupies more than one cell')
            for cell in self.cells.values():
                if cell.occupied_by:
                    drone = self.drones[cell.occupied_by]
                    if drone.assigned_cell_id != cell.cell_id:
                        raise SimulationError('Drone/cell assignment is inconsistent')
            for drone in self.drones.values():
                if drone.assigned_cell_id:
                    cell = self.cells[drone.assigned_cell_id]
                    if cell.occupied_by != drone.drone_id:
                        raise SimulationError('Cell/drone assignment is inconsistent')
            in_drone = [battery.assigned_drone_id for battery in self.batteries.values() if battery.state == 'IN_DRONE']
            if any((drone_id is None for drone_id in in_drone)):
                raise SimulationError('Installed battery has no assigned drone')
            if len(in_drone) != len(set(in_drone)):
                raise SimulationError('A drone has multiple installed batteries')
            rack_ids = self.rack.battery_ids
            if len(rack_ids) != len(set(rack_ids)):
                raise SimulationError('Battery rack contains duplicate identifiers')
            if len(self.rack.battery_ids) > self.rack.capacity:
                raise SimulationError('Battery rack capacity exceeded')
            dispatch_soc = float(self.config['battery_model']['dispatch_soc'])
            minimum_soh = float(self.config['battery_model']['minimum_soh'])
            for battery in self.batteries.values():
                if not -EPS <= battery.soc <= 1.0 + EPS:
                    raise SimulationError(f'Invalid SoC for {battery.battery_id}')
                if not minimum_soh - EPS <= battery.soh <= 1.0 + EPS:
                    raise SimulationError(f'Invalid SoH for {battery.battery_id}')
                if battery.state == 'IN_DRONE':
                    if battery.battery_id in rack_ids:
                        raise SimulationError('Installed battery is also in the rack')
                    drone = self.drones[battery.assigned_drone_id or '']
                    if drone.battery_id != battery.battery_id:
                        raise SimulationError('Drone/battery assignment is inconsistent')
                    if battery.location != drone.drone_id:
                        raise SimulationError('Installed battery location is inconsistent')
                elif battery.battery_id not in rack_ids:
                    raise SimulationError('Non-installed battery is missing from rack')
                if battery.state == 'READY' and battery.soc < dispatch_soc - EPS:
                    raise SimulationError('READY battery is below dispatch SoC')
            cooling_batteries = [slot.battery_id for slot in self.cooling_slots.values() if slot.battery_id is not None]
            if len(cooling_batteries) != len(set(cooling_batteries)):
                raise SimulationError('A battery occupies multiple cooling slots')
            for slot in self.cooling_slots.values():
                if (slot.state == 'COOLING') != (slot.battery_id is not None):
                    raise SimulationError('Cooling slot state is inconsistent')
                if slot.battery_id:
                    battery = self.batteries[slot.battery_id]
                    if battery.state != 'COOLING' or battery.location != slot.slot_id:
                        raise SimulationError('Cooling slot/battery state is inconsistent')
            charging_batteries = [charger.battery_id for charger in self.chargers.values() if charger.battery_id is not None]
            if len(charging_batteries) != len(set(charging_batteries)):
                raise SimulationError('A battery occupies multiple charger channels')
            for charger in self.chargers.values():
                if (charger.state == 'CHARGING') != (charger.battery_id is not None):
                    raise SimulationError('Charger channel state is inconsistent')
                if charger.battery_id:
                    battery = self.batteries[charger.battery_id]
                    if battery.state != 'CHARGING' or battery.location != charger.charger_id:
                        raise SimulationError('Charger/battery state is inconsistent')
            if set(cooling_batteries) & set(charging_batteries):
                raise SimulationError('A battery is cooling and charging simultaneously')
            if self.gantry.target_cell_id:
                cell = self.cells[self.gantry.target_cell_id]
                if cell.platform_position != 'INSIDE' or cell.occupied_by != self.gantry.target_drone_id:
                    raise SimulationError('Manipulator target is not stationary inside its cell')
            if self.gantry.state == 'SERVICING' and (not self.gantry.target_drone_id):
                raise SimulationError('Busy gantry has no target drone')
        except SimulationError:
            self.invariant_violations += 1
            raise
    def _is_complete(self) -> bool:
        if self.processing_completion_time_s is None:
            return False
        return all((self.drones[drone_id].remaining_area_ha <= EPS and self.drones[drone_id].state == 'PARKED' for drone_id in self.active_drone_ids))
    def run(self) -> Dict[str, Any]:
        max_time = float(self.config['simulation']['max_time_s'])
        while self._scheduled:
            event = heapq.heappop(self._scheduled)
            if event.time_s > max_time + EPS:
                raise SimulationError(f'Simulation exceeded max_time_s={max_time:.1f}')
            delta = event.time_s - self.time_s
            for drone_id in self._queued_for_gantry:
                drone = self.drones[drone_id]
                if self.gantry.state != 'IDLE':
                    drone.queue_manipulator_wait_s += delta
                else:
                    drone.queue_battery_wait_s += delta
            self.time_s = event.time_s
            self._handle_event(event)
            self._capture_battery_inventory()
            self._check_invariants()
            if self._is_complete():
                self.return_completion_time_s = self.time_s
                self._log('MILESTONE', 'ALL_DRONES_RECOVERED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', message='All active drones completed their work and are safely parked')
                self._log('SYSTEM', 'SIMULATION_COMPLETED', 'LOCAL_CONTROLLER', 'CONTROLLER-01', message='Simulation completion criteria satisfied', details={'scenario': self.scenario_name})
                break
        if not self._is_complete():
            raise SimulationError('Event queue ended before the simulation completed')
        return self.summary()
    def summary(self) -> Dict[str, Any]:
        end_s = self.return_completion_time_s or self.time_s
        processing_s = self.processing_completion_time_s or end_s
        completed_missions = [mission for mission in self.missions.values() if mission.state == 'COMPLETED']
        incomplete = [mission for mission in self.missions.values() if mission.state != 'COMPLETED']
        third_cell = self.cells.get('CELL-03')
        total_area = self.controller.total_processed_area_ha
        limiting_reason_counts: Dict[str, int] = defaultdict(int)
        for mission in completed_missions:
            for reason in mission.return_reason.split('+'):
                if reason:
                    limiting_reason_counts[reason] += 1
        avg_sortie_area = mean_or_zero([m.area_ha for m in completed_missions])
        drone_metrics: Dict[str, Any] = {}
        for drone in self.drones.values():
            flight_states_s = drone.duration_in('IN_FLIGHT', end_s) + drone.duration_in('RETURNING', end_s)
            drone_metrics[drone.drone_id] = {'missions_completed': drone.missions_completed, 'processed_area_ha': round(drone.assigned_area_ha - drone.remaining_area_ha, 6), 'flight_time_s': round(drone.flight_time_s, 3), 'waiting_cell_time_s': round(drone.waiting_cell_time_s, 3), 'cell_allocation_delay_s': round(drone.cell_allocation_delay_s, 3), 'waiting_gantry_time_s': round(drone.waiting_gantry_time_s, 3), 'utilization': round(flight_states_s / end_s if end_s else 0.0, 6), 'final_state': drone.state}
        cell_metrics: Dict[str, Any] = {}
        for cell in self.cells.values():
            busy_s = sum((cell.duration_in(state, end_s) for state in ('RESERVED', 'OCCUPIED', 'SERVICING', 'BLOCKED')))
            cell_metrics[cell.cell_id] = {'utilization': round(busy_s / end_s if end_s else 0.0, 6), 'reservation_count': cell.reservation_count, 'final_state': cell.state, 'fault_state': cell.fault_state}
        gantry_busy_s = self.gantry.duration_in('SERVICING', end_s)
        charger_metrics: Dict[str, Any] = {}
        total_charger_busy_s = 0.0
        for charger in self.chargers.values():
            busy_s = charger.duration_in('CHARGING', end_s)
            total_charger_busy_s += busy_s
            charger_metrics[charger.charger_id] = {'utilization': round(busy_s / end_s if end_s else 0.0, 6), 'completed_operations': charger.completed_operations, 'delivered_energy_wh': round(charger.delivered_energy_wh, 3), 'grid_energy_wh': round(charger.grid_energy_wh, 3), 'final_state': charger.state, 'battery_id': charger.battery_id}
        cooling_metrics: Dict[str, Any] = {}
        total_cooling_busy_s = 0.0
        for slot in self.cooling_slots.values():
            busy_s = slot.duration_in('COOLING', end_s)
            total_cooling_busy_s += busy_s
            cooling_metrics[slot.slot_id] = {'utilization': round(busy_s / end_s if end_s else 0.0, 6), 'completed_operations': slot.completed_operations, 'final_state': slot.state, 'battery_id': slot.battery_id}
        battery_metrics: Dict[str, Any] = {}
        battery_state_counts: Dict[str, int] = defaultdict(int)
        for battery in self.batteries.values():
            battery_state_counts[battery.state] += 1
            battery_metrics[battery.battery_id] = {'final_state': battery.state, 'location': battery.location, 'assigned_drone_id': battery.assigned_drone_id, 'soc': round(battery.soc, 6), 'soh': round(battery.soh, 6), 'temperature_c': round(battery.temperature_c, 3), 'cycle_count': battery.cycle_count, 'installation_count': battery.installation_count, 'equivalent_full_cycles': round(battery.equivalent_full_cycles, 6), 'energy_throughput_wh': round(battery.energy_throughput_wh, 3), 'charged_energy_wh': round(battery.charged_energy_wh, 3)}
        charger_capacity_s = len(self.chargers) * end_s
        cooling_capacity_s = len(self.cooling_slots) * end_s
        return {'scenario': self.scenario_name, 'status': 'COMPLETED', 'field_area_ha': float(self.config['field']['area_ha']), 'processed_area_ha': round(total_area, 6), 'processing_completion_time_s': round(processing_s, 3), 'all_drones_recovered_time_s': round(end_s, 3), 'processing_completion_time_h': round(processing_s / 3600.0, 6), 'effective_productivity_ha_h': round(total_area / (processing_s / 3600.0) if processing_s else 0.0, 6), 'completed_missions': len(completed_missions), 'average_sortie_area_ha': round(avg_sortie_area, 6), 'limiting_reason_counts': dict(sorted(limiting_reason_counts.items())), 'unfinished_missions': len(incomplete), 'average_cell_allocation_delay_s': round(mean_or_zero(self.cell_allocation_delays_s), 3), 'maximum_cell_allocation_delay_s': round(max(self.cell_allocation_delays_s, default=0.0), 3), 'average_cell_wait_s': round(mean_or_zero(self.cell_waits_s), 3), 'maximum_cell_wait_s': round(max(self.cell_waits_s, default=0.0), 3), 'weather_mode': self.config['weather']['mode'], 'random_seed': self.config['randomness']['seed'], 'early_returns': sum((m.early_return for m in completed_missions)), 'nominal_plan_shortfall_ha': round(sum((max(0.0, m.planned_area_ha - m.area_ha) for m in completed_missions)), 6), 'early_returns_normal_conditions': sum((m.early_return and m.early_return_weather_level == 'normal' for m in completed_missions)), 'service_delay_events': self.service_delay_events, 'minimum_actual_return_soc': round(min((m.battery_soc_end_actual for m in completed_missions), default=0), 6), 'third_cell_active': self.config['station']['third_cell_active'], 'working_cells_count': 3 if self.config['station']['third_cell_active'] else 2, 'average_internal_wait_s': round(mean_or_zero(self.gantry_waits_s), 3), 'maximum_internal_wait_s': round(max(self.gantry_waits_s, default=0.0), 3), 'internal_wait_manipulator_total_s': round(sum((d.queue_manipulator_wait_s for d in self.drones.values())), 3), 'internal_wait_battery_total_s': round(sum((d.queue_battery_wait_s for d in self.drones.values())), 3), 'air_holding_total_s': round(sum((d.waiting_cell_time_s for d in self.drones.values())), 3), 'average_gantry_wait_s': round(mean_or_zero(self.gantry_waits_s), 3), 'maximum_gantry_wait_s': round(max(self.gantry_waits_s, default=0.0), 3), 'average_cooling_queue_wait_s': round(mean_or_zero(self.cooling_waits_s), 3), 'maximum_cooling_queue_wait_s': round(max(self.cooling_waits_s, default=0.0), 3), 'average_charger_queue_wait_s': round(mean_or_zero(self.charger_waits_s), 3), 'maximum_charger_queue_wait_s': round(max(self.charger_waits_s, default=0.0), 3), 'average_cooling_duration_s': round(mean_or_zero(self.cooling_durations_s), 3), 'average_charge_duration_s': round(mean_or_zero(self.charge_durations_s), 3), 'average_ground_cycle_s': round(mean_or_zero(self.ground_cycle_durations_s), 3), 'average_final_recovery_s': round(mean_or_zero(self.final_recovery_durations_s), 3), 'gantry_utilization': round(gantry_busy_s / end_s if end_s else 0.0, 6), 'charger_utilization': round(total_charger_busy_s / charger_capacity_s if charger_capacity_s else 0.0, 6), 'cooling_utilization': round(total_cooling_busy_s / cooling_capacity_s if cooling_capacity_s else 0.0, 6), 'battery_count': len(self.batteries), **self._battery_usage_metrics(), 'charger_channels': len(self.chargers), 'cooling_slots': len(self.cooling_slots), 'battery_shortage_events': self.battery_shortage_events, 'battery_states_at_end': dict(sorted(battery_state_counts.items())), 'battery_energy_discharged_wh': round(sum((item.energy_throughput_wh for item in self.batteries.values())), 3), 'battery_energy_charged_wh': round(sum((item.charged_energy_wh for item in self.batteries.values())), 3), 'charger_grid_energy_wh': round(sum((item.grid_energy_wh for item in self.chargers.values())), 3), 'minimum_final_battery_soh': round(min((item.soh for item in self.batteries.values()), default=0.0), 6), 'minimum_mission_end_soc': round(min((item.battery_soc_end_planned for item in completed_missions), default=0.0), 6), 'third_cell_uses': third_cell.reservation_count if third_cell else 0, 'door_open_operations': sum((door.open_count for door in self.doors.values())), 'resource_conflicts': self.resource_conflicts, 'communication_loss_events': self.communication_loss_count, 'event_log_records': len(self.records), 'invariant_checks': self.invariant_checks, 'invariant_violations': self.invariant_violations, 'drones': drone_metrics, 'cells': cell_metrics, 'batteries': battery_metrics, 'chargers': charger_metrics, 'cooling_resources': cooling_metrics}
    def write_results(self, output_dir: Path) -> Dict[str, Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        with (output_dir / 'battery_inventory.csv').open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.battery_inventory[0]))
            writer.writeheader()
            writer.writerows(self.battery_inventory)
        (output_dir / 'battery_usage.json').write_text(json.dumps(self._battery_usage_metrics(), ensure_ascii=False, indent=2), encoding='utf-8')
        write_parameter_audit(self.config, output_dir)
        paths = {'events_csv': output_dir / 'events.csv', 'events_jsonl': output_dir / 'events.jsonl', 'missions_csv': output_dir / 'missions.csv', 'batteries_csv': output_dir / 'batteries.csv', 'battery_resources_csv': output_dir / 'battery_resources.csv', 'summary_json': output_dir / 'summary.json', 'resolved_config_json': output_dir / 'resolved_config.json', 'summary_txt': output_dir / 'summary.txt'}
        (output_dir / 'weather_timeline.json').write_text(json.dumps(self.weather_timeline, ensure_ascii=False, indent=2), encoding='utf-8')
        (output_dir / 'sampled_operations.json').write_text(json.dumps(self.sampled_operations, ensure_ascii=False, indent=2), encoding='utf-8')
        event_fields = list(self.records[0].csv_row().keys()) if self.records else []
        with paths['events_csv'].open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=event_fields)
            writer.writeheader()
            writer.writerows((record.csv_row() for record in self.records))
        with paths['events_jsonl'].open('w', encoding='utf-8') as stream:
            for record in self.records:
                stream.write(json.dumps(asdict(record), ensure_ascii=False) + '\n')
        mission_fields = ['mission_id', 'assigned_drone_id', 'area_ha', 'state', 'start_time_s', 'work_finish_time_s', 'finish_time_s', 'return_reason', 'outbound_time_s', 'work_time_s', 'return_time_s', 'energy_used_wh', 'outbound_energy_wh', 'work_energy_wh', 'return_energy_wh', 'liquid_used_l', 'battery_id', 'battery_soc_start', 'battery_soc_end_planned', 'planned_area_ha', 'planned_energy_wh', 'battery_soc_end_actual', 'early_return', 'early_return_weather_level']
        with paths['missions_csv'].open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=mission_fields)
            writer.writeheader()
            for mission in sorted(self.missions.values(), key=lambda item: item.mission_id):
                writer.writerow({key: getattr(mission, key) for key in mission_fields})
        battery_fields = ['battery_id', 'state', 'location', 'assigned_drone_id', 'soc', 'soh', 'temperature_c', 'cycle_count', 'installation_count', 'equivalent_full_cycles', 'energy_throughput_wh', 'charged_energy_wh', 'ready_at_s']
        with paths['batteries_csv'].open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=battery_fields)
            writer.writeheader()
            for battery in sorted(self.batteries.values(), key=lambda item: item.battery_id):
                writer.writerow({key: getattr(battery, key) for key in battery_fields})
        resource_fields = ['resource_type', 'resource_id', 'state', 'battery_id', 'completed_operations', 'utilization', 'delivered_energy_wh', 'grid_energy_wh']
        end_s = self.return_completion_time_s or self.time_s
        resource_rows: List[Dict[str, Any]] = []
        for slot in sorted(self.cooling_slots.values(), key=lambda item: item.slot_id):
            busy_s = slot.duration_in('COOLING', end_s)
            resource_rows.append({'resource_type': 'COOLING_SLOT', 'resource_id': slot.slot_id, 'state': slot.state, 'battery_id': slot.battery_id or '', 'completed_operations': slot.completed_operations, 'utilization': busy_s / end_s if end_s else 0.0, 'delivered_energy_wh': '', 'grid_energy_wh': ''})
        for charger in sorted(self.chargers.values(), key=lambda item: item.charger_id):
            busy_s = charger.duration_in('CHARGING', end_s)
            resource_rows.append({'resource_type': 'CHARGER', 'resource_id': charger.charger_id, 'state': charger.state, 'battery_id': charger.battery_id or '', 'completed_operations': charger.completed_operations, 'utilization': busy_s / end_s if end_s else 0.0, 'delivered_energy_wh': charger.delivered_energy_wh, 'grid_energy_wh': charger.grid_energy_wh})
        with paths['battery_resources_csv'].open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=resource_fields)
            writer.writeheader()
            writer.writerows(resource_rows)
        paths['summary_json'].write_text(json.dumps(self.summary(), ensure_ascii=False, indent=2), encoding='utf-8')
        paths['resolved_config_json'].write_text(json.dumps(self.config, ensure_ascii=False, indent=2), encoding='utf-8')
        paths['summary_txt'].write_text(format_summary_text(self.summary()), encoding='utf-8')
        return paths
SOURCE_URLS = {'S4': 'https://map.onesoil.ai/2020/RUS', 'S1': 'https://ag.dji.com/t50/specs', 'S2': 'https://dl.djicdn.com/downloads/t50_t25/20240718/T50_T25_User_Manual_v1.0_EN.pdf', 'S3': 'https://www.dji.com/media-center/announcements/dji-agras-t50-and-t25-en'}
def write_parameter_audit(config: Dict[str, Any], directory: Path) -> None:
    resolved = DroneportSimulation(config).config
    physical = {'field.area_ha': ('Задание сценария; 27 га — округлённая оценка по набору OneSoil РФ/2020, не официальная статистика', 'S4'), 'flight.aircraft_mass_without_battery_kg': ('Паспорт', 'S1'), 'flight.tank_capacity_l': ('Паспорт', 'S1'), 'battery_model.capacity_ah': ('Паспорт', 'S1'), 'battery_model.nominal_voltage_v': ('Паспорт', 'S1'), 'battery_model.battery_mass_kg': ('Паспорт', 'S1'), 'battery_model.nominal_capacity_wh': ('Расчёт: А·ч × В', 'S1'), 'flight.max_flight_speed_m_s': ('Предел аппарата', 'S2, с. 31'), 'flight.max_takeoff_mass_kg': ('Паспорт, опрыскивание, уровень моря', 'S1'), 'flight.max_spray_flow_l_min': ('Паспорт, два распылителя', 'S1'), 'flight.transit_speed_m_s': ('Сценарий: выбрана в пределах аппарата', 'S2, с. 31'), 'flight.work_speed_m_s': ('Выбранный режим: скорость теста DJI', 'S3, прим. 1'), 'flight.swath_m': ('Сценарий: выбрана в паспортном диапазоне 4–11 м', 'S1'), 'battery_model.charger_power_kw': ('Паспорт: максимум одного C10000', 'S1'), 'battery_model.charge_reference_from_soc': ('Условия заявленного быстрого заряда', 'S3, прим. 5'), 'battery_model.charge_reference_to_soc': ('Условия заявленного быстрого заряда', 'S3, прим. 5'), 'battery_model.charge_reference_time_s': ('Заявленный эталон 30–95%, специальный режим', 'S3, прим. 5'), 'flight.reference_empty_mass_kg': ('Паспортная точка лабораторного испытания', 'S2, с. 31'), 'flight.reference_loaded_mass_kg': ('Паспортная точка лабораторного испытания', 'S2, с. 31'), 'flight.reference_empty_endurance_min': ('Лабораторная продолжительность', 'S2, с. 31'), 'flight.reference_loaded_endurance_min': ('Лабораторная продолжительность', 'S2, с. 31'), 'flight.reference_energy_wh': ('Расчёт энергии эталонной DB1560', 'S1'), 'flight.hover_power_coefficient_kw': ('Расчётная аппроксимация, не измеренная мощность', 'S1; S2'), 'flight.hover_power_exponent': ('Расчётная аппроксимация, не измеренная мощность', 'S2')}
    rows = []
    def walk(value: Any, path: str='') -> None:
        if isinstance(value, dict):
            for key, item in value.items():
                if key not in {'scenarios', 'parameter_notes'}:
                    walk(item, f'{path}.{key}'.strip('.'))
        else:
            origin, source = physical.get(path, ('Допущение / настройка сценария', '—'))
            unit = next((unit for suffix, unit in (('_m_s', 'м/с'), ('_l_ha', 'л/га'), ('_kg_l', 'кг/л'), ('_l_min', 'л/мин'), ('_ah', 'А·ч'), ('_wh', 'Вт·ч'), ('_kw', 'кВт'), ('_kg', 'кг'), ('_min', 'мин'), ('_s', 'с'), ('_m', 'м'), ('_l', 'л'), ('_c', '°C'), ('_v', 'В'), ('_ha', 'га')) if path.endswith(suffix)), '—')
            rows.append({'parameter': path, 'value': json.dumps(value, ensure_ascii=False), 'unit': unit, 'origin': origin, 'source': source})
    walk(resolved)
    with (directory / 'parameters.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    notes = ['# Параметры и границы модели T50 / DB1560', '', 'Паспортные данные проверены 16.10.2026. Значения конкретного расчёта приведены ниже; изменённые пользователем значения нужно повторно сверять с источником.', '', 'Скорости перелёта и обработки разделены. Ширина 10 м и норма 10 л/га — выбранный сценарий; это не универсальная агротехнологическая рекомендация.', '', 'Ёмкость 30 А·ч и энергия 1566,6 Вт·ч — разные величины. Энергия вычисляется по номинальному напряжению 52,22 В.', '', 'Эталон заряда — 540 с от 30 до 95% в указанных DJI условиях. Время для других SoC линейно экстраполируется с ограничением мощности. Оно не является паспортной зарядной кривой. КПД 0,92 и тепловые времена — допущения.', '', 'Два канала по 9 кВт означают два независимых источника/зарядных устройства с суммарной выходной мощностью 18 кВт, а не два разъёма одного устройства на 9 кВт. Общий лимит электроснабжения станции в этой версии не моделируется.', '', 'Зависимость P=a·m^b приближённо согласована с лабораторными точками T50: 52 кг / 16,5 мин и 92 кг / 7 мин. Использование номинальной энергии для восстановления мощности — допущение. Это не валидация энергии реального рабочего полёта.', '', 'Коэффициенты ветра не связаны с измеренными м/с. Паспортная ветроустойчивость не является разрешением на опрыскивание при таком ветре. Предварительное планирование не использует прогноз; оперативный контроль использует заданный консервативный диапазон условий.', '', 'Все времена механизмов станции, резерв 15%, запас жидкости 5%, расстояние до поля, норма внесения, количество АКБ и зарядников требуют задания или проверки для конкретного ТЗ. Энергия взлёта/посадки, развороты и геометрия поля отдельно не моделируются.', '', 'Счётчик разных АКБ показывает фактически задействованный парк, а не минимально необходимый запас. Для оценки запаса используйте --battery-sweep. Его вывод относится только к проверенным значениям и условиям, без резерва на отказы и старение.', '', '| Параметр | Значение | Ед. | Статус | Источник |', '|---|---|---|---|---|']
    notes += ['| ' + ' | '.join((str(r[k]).replace('|', '/') for k in ('parameter', 'value', 'unit', 'origin', 'source'))) + ' |' for r in rows]
    notes += ['', '## Источники', ''] + [f'- {key}: {url}' for key, url in SOURCE_URLS.items()]
    (directory / 'parameter_audit.md').write_text('\n'.join(notes) + '\n', encoding='utf-8')
def _plot_backend(config: Dict[str, Any]):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError('Для графиков установите matplotlib: python -m pip install matplotlib; либо --no-plots') from exc
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.labelsize': 9, 'axes.titlesize': 10, 'legend.fontsize': 7.8, 'xtick.labelsize': 8, 'ytick.labelsize': 8, 'axes.spines.top': False, 'axes.spines.right': False, 'svg.fonttype': 'none', 'axes.axisbelow': True})
    return plt
def _save_figure(fig, directory: Path, name: str, config: Dict[str, Any]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for extension in config['plots']['formats']:
        if extension not in {'png', 'svg'}:
            raise ValueError('plots.formats supports png and svg')
        fig.savefig(directory / f'{name}.{extension}', dpi=config['plots']['dpi'], facecolor='white')
def _figure_timeline(records: List[Dict[str, Any]], summary: Dict[str, Any], config: Dict[str, Any], directory: Path) -> None:
    plt = _plot_backend(config)
    from matplotlib.patches import Patch
    window = min(config['plots']['timeline_minutes'] * 60, summary['all_drones_recovered_time_s'])
    drone_ids = sorted({r['object_id'] for r in records if r['object_type'] == 'DRONE' and r['operation'] == 'STATE_TRANSITION'})
    rows = [(x, f'БПЛА {i + 1}') for i, x in enumerate(drone_ids)] + [('GANTRY-01', 'Манипулятор')]
    colors = {'Полёт': '#0072B2', 'Приём / выпуск': '#56B4E9', 'Ожидание внутри': '#E69F00', 'Обслуживание': '#009E73', 'Стоянка / свободен': '#E5E5E5'}
    def category(state: str, gantry: bool) -> str:
        if gantry:
            return 'Обслуживание' if state == 'SERVICING' else 'Стоянка / свободен'
        if state in {'IN_FLIGHT', 'RETURNING'}:
            return 'Полёт'
        if state == 'WAITING_FOR_GANTRY':
            return 'Ожидание внутри'
        if state in {'MOVE', 'BATTERY_SWAP', 'REFILL', 'SERVICE_CHECK', 'CLEAR'}:
            return 'Обслуживание'
        if state in {'PARKED', 'READY'}:
            return 'Стоянка / свободен'
        return 'Приём / выпуск'
    width = config['plots']['width_mm'] / 25.4
    fig, ax = plt.subplots(figsize=(width, 2.6), layout='constrained')
    for index, (object_id, label) in enumerate(rows):
        transitions = [r for r in records if r['operation'] == 'STATE_TRANSITION' and r['object_id'] == object_id]
        points = [(0.0, 'IDLE' if object_id == 'GANTRY-01' else 'READY')]
        points += [(r['simulation_time_s'], r['new_state']) for r in transitions]
        for (start, state), (end, _) in zip(points, points[1:] + [(window, '')]):
            if start >= window:
                break
            length = min(end, window) - start
            if length <= 0:
                continue
            color = colors[category(state, object_id == 'GANTRY-01')]
            ax.broken_barh([(start / 60, length / 60)], (index - 0.3, 0.6), facecolors=color, edgecolors='white', linewidth=0.2)
    ax.set(yticks=list(range(len(rows))), yticklabels=[r[1] for r in rows], xlabel='Время от начала работы, мин', xlim=(0, window / 60))
    ax.invert_yaxis()
    ax.grid(axis='x', alpha=0.2)
    ax.legend(handles=[Patch(facecolor=c, label=l) for l, c in colors.items()], ncol=3, loc='upper center', bbox_to_anchor=(0.5, -0.25), frameon=False)
    _save_figure(fig, directory, '02_service_timeline', config)
    plt.close(fig)
def _figure_inventory(inventory: List[Dict[str, Any]], config: Dict[str, Any], directory: Path) -> None:
    plt = _plot_backend(config)
    from matplotlib.ticker import MaxNLocator
    window = min(config['plots']['timeline_minutes'] * 60, inventory[-1]['time_s'])
    data = [dict(row) for row in inventory if row['time_s'] <= window]
    if data[-1]['time_s'] < window:
        data.append(dict(data[-1], time_s=window))
    names = [('in_drones', 'В дронах', '#0072B2'), ('ready', 'Готовы', '#009E73'), ('charging', 'Зарядка', '#E69F00'), ('cooling', 'Охлаждение', '#56B4E9'), ('waiting', 'Очереди / резервирование', '#CC79A7')]
    for row in data:
        row['waiting'] = row['reserved'] + row['waiting_cooling'] + row['waiting_charger'] + row['other_unavailable']
    fig, ax = plt.subplots(figsize=(config['plots']['width_mm'] / 25.4, 2.9), layout='constrained')
    ax.stackplot([r['time_s'] / 60 for r in data], *[[r[key] for r in data] for key, _, _ in names], labels=[label for _, label, _ in names], colors=[c for _, _, c in names], step='post', linewidth=0)
    ax.set(xlabel='Время от начала работы, мин', ylabel='Количество АКБ, шт.', xlim=(0, window / 60), ylim=(0, inventory[0]['total']))
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.legend(ncol=3, loc='upper center', bbox_to_anchor=(0.5, -0.24), frameon=False)
    _save_figure(fig, directory, '03_battery_inventory', config)
    plt.close(fig)
def make_run_figures(simulation: DroneportSimulation, directory: Path) -> None:
    plt = _plot_backend(simulation.config)
    missions = sorted(simulation.missions.values(), key=lambda m: m.work_finish_time_s or 0)
    xs, ys, area = ([0.0], [0.0], 0.0)
    for mission in missions:
        area += mission.area_ha
        xs.append(mission.work_finish_time_s / 3600)
        ys.append(area)
    fig, ax = plt.subplots(figsize=(simulation.config['plots']['width_mm'] / 25.4, 2.9), layout='constrained')
    ax.step(xs, ys, where='post', color='#0072B2', linewidth=1.4)
    ax.set(xlabel='Время от начала работы, ч', ylabel='Площадь завершённых участков, га', xlim=(0, max(xs)), ylim=(0, simulation.config['field']['area_ha'] * 1.03))
    ax.grid(alpha=0.2)
    _save_figure(fig, directory, '01_completed_area', simulation.config)
    plt.close(fig)
    _figure_timeline([asdict(r) for r in simulation.records], simulation.summary(), simulation.config, directory)
    _figure_inventory([dict(r) for r in simulation.battery_inventory], simulation.config, directory)
    (directory / 'captions.md').write_text(f"1. Накопленная площадь завершённых участков: ступени соответствуют окончанию обработки в отдельных вылетах, а не остановкам внесения.\n2. Циклограмма БПЛА и общего манипулятора; показан начальный интервал расчёта.\n3. Состояния всех физических АКБ во времени; сумма равна заданному запасу.\nСценарий: {simulation.scenario_name}; seed={simulation.config['randomness']['seed']}.\n", encoding='utf-8')
def make_suite_figures(groups: List[Dict[str, Any]], output_dir: Path, config: Dict[str, Any]) -> None:
    plt = _plot_backend(config)
    names = list(dict.fromkeys((g['scenario'] for g in groups)))
    fleets = sorted({g['active_drones'] for g in groups})
    labels = {'baseline': 'Штатный', 'operation_variation': 'Разброс\nопераций', 'steady_wind': 'Постоянный\nветер', 'wind_episodes': 'Усиления\nветра', 'service_delays': 'Задержки\nсервиса', 'combined': 'Совместный'}
    width = 0.72 / len(fleets)
    fig, ax = plt.subplots(figsize=(config['plots']['width_mm'] / 25.4, 3.3), layout='constrained')
    for index, count in enumerate(fleets):
        rows = [next((g for g in groups if g['scenario'] == name and g['active_drones'] == count)) for name in names]
        positions = [i - 0.36 + width / 2 + index * width for i in range(len(names))]
        ax.bar(positions, [r['time_mean_h'] for r in rows], width, yerr=[r['time_std_h'] for r in rows], label=f'{count} БПЛА', color=['#0072B2', '#E69F00', '#009E73'][index % 3], hatch='' if index == 0 else '//', capsize=2.5, edgecolor='#333333', linewidth=0.45)
    ax.set(xticks=list(range(len(names))), xticklabels=[labels.get(n, n) for n in names], ylabel='Время обработки поля, ч')
    ax.set_title(f"Площадь поля: {groups[0]['field_area_ha']:g} га")
    ax.grid(axis='y', alpha=0.2)
    ax.legend(frameon=False)
    directory = output_dir / 'figures'
    _save_figure(fig, directory, '01_scenario_comparison', config)
    plt.close(fig)
    selected = next((g for g in groups if g['scenario'] == 'combined' and g['active_drones'] == max(fleets)), groups[-1])
    case_dir = output_dir / f"{selected['scenario']}_{selected['active_drones']}drones" / 'rep_001'
    summary = json.loads((case_dir / 'summary.json').read_text(encoding='utf-8'))
    records = [json.loads(line) for line in (case_dir / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
    with (case_dir / 'battery_inventory.csv').open(encoding='utf-8-sig', newline='') as stream:
        inventory = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    _figure_timeline(records, summary, config, directory)
    _figure_inventory(inventory, config, directory)
    (directory / 'captions.md').write_text(f"1. Время обработки поля {groups[0]['field_area_ha']:g} га в различных сценариях. Столбцы — средние, интервалы — ± одно выборочное стандартное отклонение (не доверительный интервал). Детерминированные сценарии: один расчёт; число случайных повторов указано в scenario_statistics.csv.\n2–3. Циклограмма и состояния АКБ: отдельный пример {selected['scenario']}, {selected['active_drones']} БПЛА, повтор 1, seed={summary['random_seed']}; это не усреднённые траектории.\nКоэффициенты ветра и механические времена — модельные допущения; сведения о параметрах: parameter_audit.md.\n", encoding='utf-8')
def run_battery_sweep(config: Dict[str, Any], output_dir: Path) -> List[Dict[str, Any]]:
    config = deep_merge(DEFAULT_CONFIG, config)
    counts = sorted(set(config['battery_sweep']['counts']))
    count_drones = config['fleet']['total_drones']
    if not counts or any((type(n) is not int or n <= count_drones for n in counts)):
        raise ValueError('Every tested stock must exceed fleet.total_drones')
    rows = []
    stochastic = config['randomness']['enabled']
    reps = config['experiment']['replications'] if stochastic else 1
    for stock in counts:
        print(f'Запас АКБ: {stock}, повторов {reps}', flush=True)
        results = []
        for index in range(reps):
            case = deep_merge(config, {'battery_rack': {'battery_count': stock, 'capacity': stock - count_drones}, 'randomness': {'seed': config['randomness']['seed'] + index}})
            results.append(run_single(case, output_dir / f'{stock}_packs' / f'rep_{index + 1:03d}', f'stock_{stock}', render_plots=False))
        times = [r['processing_completion_time_h'] for r in results]
        rows.append({'battery_count': stock, 'replications': reps, 'time_mean_h': statistics.mean(times), 'time_p95_h': percentile(times, 0.95), 'battery_wait_max_total_s': max((r['internal_wait_battery_total_s'] for r in results)), 'unique_batteries_used_max': max((r['unique_batteries_used'] for r in results))})
    best_p95 = min((r['time_p95_h'] for r in rows))
    settings = config['battery_sweep']
    suitable = [r['battery_count'] for r in rows if r['time_p95_h'] <= best_p95 * (1 + settings['time_tolerance_fraction']) and r['battery_wait_max_total_s'] <= settings['maximum_battery_wait_s']]
    report = {'tested_results': rows, 'smallest_acceptable_tested_stock': min(suitable) if suitable else None, 'criteria': settings, 'note': 'Оценка только среди проверенных запасов и условий. Резерв на отказы и износ не включён.'}
    (output_dir / 'battery_sizing.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    with (output_dir / 'battery_sizing.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write_parameter_audit(config, output_dir)
    if suitable:
        print('Наименьший подходящий из проверенных запасов:', min(suitable))
    else:
        print('Среди проверенных запасов нет варианта, отвечающего заданным критериям; проверьте зарядную производительность и критерии.')
    if config['plots']['enabled']:
        plt = _plot_backend(config)
        fig, axes = plt.subplots(2, 1, figsize=(config['plots']['width_mm'] / 25.4, 4.0), sharex=True, layout='constrained')
        axes[0].plot(counts, [r['time_mean_h'] for r in rows], 'o-', color='#0072B2')
        axes[1].plot(counts, [r['battery_wait_max_total_s'] / 60 for r in rows], 's--', color='#D55E00')
        axes[0].set(ylabel='Время обработки, ч')
        axes[1].set(xlabel='Физический запас АКБ, шт.', ylabel='Ожидание АКБ, мин', xticks=counts)
        for ax in axes:
            ax.grid(alpha=0.2)
        _save_figure(fig, output_dir / 'figures', '01_battery_sizing', config)
        plt.close(fig)
    return rows
def load_config(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as stream:
        config = json.load(stream)
    if not isinstance(config, dict):
        raise ValueError('Configuration root must be a JSON object')
    if 'speed_m_s' in config.get('flight', {}):
        value = config['flight'].pop('speed_m_s')
        config['flight'].setdefault('transit_speed_m_s', value)
        config['flight'].setdefault('work_speed_m_s', value)
    return config
def run_single(config: Dict[str, Any], output_dir: Path, scenario_name: str='base', render_plots: bool=True) -> Dict[str, Any]:
    simulation = DroneportSimulation(config, scenario_name=scenario_name)
    summary = simulation.run()
    simulation.write_results(output_dir)
    if render_plots and simulation.config['plots']['enabled']:
        make_run_figures(simulation, output_dir / 'figures')
    return summary
def percentile(values: Sequence[float], q: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    lo, hi = (math.floor(position), math.ceil(position))
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (position - lo)
def run_scenario_suite(config: Dict[str, Any], output_dir: Path) -> List[Dict[str, Any]]:
    selected_scenarios = config.get('scenarios')
    config = deep_merge(DEFAULT_CONFIG, config)
    if selected_scenarios is not None:
        config['scenarios'] = copy.deepcopy(selected_scenarios)
    scenarios = config['scenarios']
    if not scenarios:
        raise ValueError('Configuration has no scenarios')
    base = {key: value for key, value in config.items() if key != 'scenarios'}
    raw, groups = ([], [])
    output_dir.mkdir(parents=True, exist_ok=True)
    base_seed = config['randomness']['seed']
    for name, override in scenarios.items():
        for count in config['experiment']['compare_fleets']:
            case = deep_merge(base, override)
            if count not in (1, 2, 3):
                raise ValueError('experiment.compare_fleets may contain only 1, 2, 3')
            case = deep_merge(case, {'fleet': {'total_drones': count, 'max_active_drones': count}, 'station': {'third_cell_active': override.get('station', {}).get('third_cell_active', int(count == 3))}})
            if count == 1:
                case['battery_rack']['capacity'] = max(case['battery_rack']['capacity'], case['battery_rack']['battery_count'] - count)
            rc = case['randomness']
            stochastic = rc['enabled'] and (rc['operation_variation'] > 0 or rc['service_delays'] or (case['weather']['mode'] == 'episodes' and case['weather']['random_intervals']))
            reps = case['experiment']['replications'] if stochastic else 1
            results = []
            case_name = f'{name}_{count}drones'
            print(f"Расчёт: {case_name}, площадь {case['field']['area_ha']:g} га, повторов {reps}", flush=True)
            for index in range(reps):
                settings = deep_merge(case, {'randomness': {'seed': base_seed + index}})
                result = run_single(settings, output_dir / case_name / f'rep_{index + 1:03d}', case_name, render_plots=False)
                result.update(replication=index + 1, active_drones=count)
                results.append(result)
                raw.append(result)
            times = [r['processing_completion_time_h'] for r in results]
            group = {'scenario': name, 'active_drones': count, 'field_area_ha': case['field']['area_ha'], 'replications': reps, 'time_mean_h': statistics.mean(times), 'time_std_h': statistics.stdev(times) if reps > 1 else 0.0, 'time_min_h': min(times), 'time_max_h': max(times), 'time_p95_h': percentile(times, 0.95), 'early_returns_mean': statistics.mean((r['early_returns'] for r in results)), 'internal_wait_mean_s': statistics.mean((r['average_internal_wait_s'] for r in results)), 'internal_wait_max_s': max((r['maximum_internal_wait_s'] for r in results)), 'air_holding_total_max_s': max((r['air_holding_total_s'] for r in results)), 'minimum_actual_return_soc': min((r['minimum_actual_return_soc'] for r in results)), 'gantry_utilization_mean': statistics.mean((r['gantry_utilization'] for r in results)), 'charger_utilization_mean': statistics.mean((r['charger_utilization'] for r in results)), 'battery_wait_total_mean_s': statistics.mean((r['internal_wait_battery_total_s'] for r in results)), 'unique_batteries_used_max': max((r['unique_batteries_used'] for r in results)), 'battery_swap_count_mean': statistics.mean((r['battery_swap_count'] for r in results)), 'minimum_ready_batteries': min((r['minimum_ready_batteries'] for r in results)), 'peak_charging_batteries': max((r['peak_charging_batteries'] for r in results)), 'peak_cooling_batteries': max((r['peak_cooling_batteries'] for r in results)), 'peak_waiting_for_charger': max((r['peak_waiting_for_charger'] for r in results)), 'invariant_violations': sum((r['invariant_violations'] for r in results))}
            groups.append(group)
    for filename, rows in (('scenario_summary', raw), ('scenario_statistics', groups)):
        (output_dir / f'{filename}.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf-8')
        keys = [key for key, value in rows[0].items() if not isinstance(value, (dict, list))]
        with (output_dir / f'{filename}.csv').open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=keys, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(rows)
    write_parameter_audit(config, output_dir)
    if config['plots']['enabled']:
        make_suite_figures(groups, output_dir, config)
    return groups
def self_test(config: Dict[str, Any]) -> None:
    base = {key: value for key, value in config.items() if key != 'scenarios'}
    base = deep_merge(base, {'plots': {'enabled': False}})
    with tempfile.TemporaryDirectory(prefix='droneport_v8_') as temp_dir:
        root = Path(temp_dir)
        first = run_single(base, root / 'first', 'self_test_first')
        second = run_single(base, root / 'second', 'self_test_second')
        keys = ['processed_area_ha', 'processing_completion_time_s', 'completed_missions', 'average_cell_wait_s', 'average_gantry_wait_s', 'door_open_operations', 'invariant_violations']
        if any((first[key] != second[key] for key in keys)):
            raise AssertionError('Identical inputs produced different results')
        if first['invariant_violations'] != 0:
            raise AssertionError('Resource invariant violation detected')
        if not math.isclose(first['processed_area_ha'], float(base['field']['area_ha']), abs_tol=1e-06):
            raise AssertionError('Processed area does not match field area')
        operations = {row['operation'] for row in csv.DictReader((root / 'first' / 'events.csv').open('r', encoding='utf-8-sig', newline=''))}
        required = {'CELL_SEARCH_STARTED', 'AVAILABLE_CELLS_DETERMINED', 'CELL_RESERVED', 'COMMAND_SENT', 'GANTRY_SEARCH_STARTED', 'BATTERY_SWAP_COMPLETE', 'BATTERY_COOLING_STARTED', 'BATTERY_CHARGING_STARTED', 'BATTERY_READY', 'SIMULATION_COMPLETED', 'FINAL_CHECK_PASSED'}
        missing = required - operations
        if missing:
            raise AssertionError(f'Event log is missing operations: {sorted(missing)}')
    for count, mode in ((1, 0), (2, 0), (3, 1)):
        model = DroneportSimulation({'field': {'area_ha': 12}, 'fleet': {'total_drones': count, 'max_active_drones': count}, 'station': {'third_cell_active': mode}, 'battery_rack': {'capacity': 6}})
        result = model.run()
        assert result['air_holding_total_s'] == 0
        assert result['invariant_violations'] == 0
        assert abs(result['processed_area_ha'] - 12) < 1e-06
        if not mode:
            assert result['third_cell_uses'] == 0
        if count == 3:
            assert result['third_cell_uses'] > 0
        phase_order = ['MOVE', 'BATTERY_SWAP', 'REFILL', 'SERVICE_CHECK', 'CLEAR']
        phases = [r.details['phase'] for r in model.records if r.operation == 'MANIPULATOR_PHASE_STARTED']
        assert phases and len(phases) % 5 == 0
        assert phases == phase_order * (len(phases) // 5)
        if count == 1:
            assert result['maximum_internal_wait_s'] == 0
        assert math.isclose(sum((d.waiting_gantry_time_s for d in model.drones.values())), sum((d.queue_battery_wait_s + d.queue_manipulator_wait_s for d in model.drones.values())), abs_tol=1e-06)
    for bad in (2, -1, 0.5, True):
        try:
            DroneportSimulation({'station': {'third_cell_active': bad}})
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid third cell mode accepted')
    try:
        DroneportSimulation({'fleet': {'total_drones': 3, 'max_active_drones': 3}, 'station': {'third_cell_active': 0}})
    except ValueError:
        pass
    else:
        raise AssertionError('Three active drones accepted with only two working cells')
    failover = DroneportSimulation({'field': {'area_ha': 8}, 'failures': [{'object_type': 'GARAGE_CELL', 'object_id': 'CELL-01', 'start_s': 0, 'duration_s': 10000}]})
    assert failover.run()['third_cell_uses'] > 0
    reference = DroneportSimulation({'field': {'area_ha': 8}})
    ref_result = reference.run()
    starts = [r for r in reference.records if r.operation == 'MANIPULATOR_PHASE_STARTED'][:5]
    for phase in starts:
        interrupted = DroneportSimulation({'field': {'area_ha': 8}, 'failures': [{'object_type': 'SERVICE_GANTRY', 'object_id': 'GANTRY-01', 'start_s': phase.simulation_time_s + 1, 'duration_s': 7}]})
        interrupted_result = interrupted.run()
        assert interrupted_result['invariant_violations'] == 0
        assert interrupted_result['all_drones_recovered_time_s'] >= ref_result['all_drones_recovered_time_s']
    for name in DEFAULT_CONFIG['scenarios']:
        cfg = deep_merge({'field': {'area_ha': 12}}, DEFAULT_CONFIG['scenarios'][name])
        first = DroneportSimulation(cfg)
        second = DroneportSimulation(cfg)
        a, b = (first.run(), second.run())
        for key in ('processing_completion_time_s', 'completed_missions', 'early_returns', 'minimum_actual_return_soc', 'air_holding_total_s'):
            assert a[key] == b[key], (name, key)
        assert a['minimum_actual_return_soc'] >= first.config['battery_model']['reserve_soc'] - 1e-06
        assert a['air_holding_total_s'] == 0
    cfg = deep_merge({'field': {'area_ha': 12}}, DEFAULT_CONFIG['scenarios']['combined'])
    pair2 = DroneportSimulation(cfg)
    pair3 = DroneportSimulation(deep_merge(cfg, {'fleet': {'total_drones': 3, 'max_active_drones': 3}, 'station': {'third_cell_active': 1}}))
    assert pair2.weather_timeline == pair3.weather_timeline
    assert pair3.run()['air_holding_total_s'] == 0
    changed = DroneportSimulation(deep_merge(cfg, {'randomness': {'seed': 43}}))
    assert changed.weather_timeline != pair2.weather_timeline
    no_service = DroneportSimulation(deep_merge(cfg, {'randomness': {'operation_variation': 0, 'service_delays': False}}))
    assert no_service.weather_timeline == pair2.weather_timeline
    fixed = deep_merge(cfg, {'randomness': {'enabled': False}})
    assert DroneportSimulation(fixed).weather_timeline == DroneportSimulation(deep_merge(fixed, {'randomness': {'seed': 99}})).weather_timeline
    coarse = DroneportSimulation(deep_merge(cfg, {'randomness': {'enabled': False}})).run()
    fine = DroneportSimulation(deep_merge(cfg, {'randomness': {'enabled': False}, 'weather': {'flight_update_s': 2}})).run()
    assert abs(coarse['processing_completion_time_s'] - fine['processing_completion_time_s']) < 0.1
    assert fine['early_returns'] > 0
    assert fine['minimum_actual_return_soc'] >= 0.15 - 1e-06
    neutral = {'mode': 'steady', 'steady_power_factor': 1.0, 'steady_transit_speed_factor': 1.0, 'steady_work_rate_factor': 1.0, 'steady_landing_factor': 1.0, 'return_margin_wh': 0.0}
    calm = DroneportSimulation({'field': {'area_ha': 12}}).run()
    neutral_result = DroneportSimulation({'field': {'area_ha': 12}, 'weather': neutral}).run()
    assert abs(calm['processing_completion_time_s'] - neutral_result['processing_completion_time_s']) < 0.01
    assert calm['completed_missions'] == neutral_result['completed_missions']
    assert neutral_result['early_returns'] == 0
    delayed = DroneportSimulation({'field': {'area_ha': 12}, 'randomness': {'enabled': True, 'operation_variation': 0.1, 'service_delays': True, 'delay_probability': 1.0, 'delay_min_s': 30.0, 'delay_max_s': 30.0}})
    delayed_result = delayed.run()
    assert delayed_result['service_delay_events'] > 0
    assert all((row['duration_s'] >= 0 for row in delayed.sampled_operations))
    rapid = DroneportSimulation({'field': {'area_ha': 12}, 'weather': {'mode': 'episodes', 'normal_min_s': 37.0, 'normal_max_s': 37.0, 'strong_min_s': 29.0, 'strong_max_s': 29.0}})
    rapid_result = rapid.run()
    assert rapid_result['minimum_actual_return_soc'] >= 0.15 - 1e-06
    assert abs(rapid_result['processed_area_ha'] - 12) < 1e-06
    audited = DroneportSimulation({'field': {'area_ha': 12}, 'plots': {'enabled': False}})
    result = audited.run()
    assert result['battery_flight_uses'] == result['completed_missions']
    assert result['battery_swap_count'] + len(audited.active_drone_ids) == result['battery_flight_uses']
    assert sum((b.installation_count for b in audited.batteries.values())) == 2 + result['battery_swap_count']
    assert result['unique_batteries_used'] <= result['battery_count']
    assert result['peak_charging_batteries'] <= result['charger_channels']
    assert result['peak_cooling_batteries'] <= result['cooling_slots']
    for row in audited.battery_inventory:
        assert row['total'] == sum((v for key, v in row.items() if key not in {'time_s', 'total'}))
    for mass, minutes in ((52, 16.5), (92, 7)):
        assert abs(1566.6 / 1000 / audited._flight_power_kw(mass, 'work') * 60 - minutes) < 1e-06
    larger = DroneportSimulation({'battery_model': {'capacity_ah': 32}})
    assert larger._flight_power_kw(92, 'work') == audited._flight_power_kw(92, 'work')
    assert larger.config['battery_model']['nominal_capacity_wh'] > 1566.6
def format_duration(seconds: float) -> str:
    total = int(round(seconds))
    hours, rem = divmod(total, 3600)
    minutes, secs = divmod(rem, 60)
    return f'{hours:02d}:{minutes:02d}:{secs:02d}'
def format_summary_text(summary: Dict[str, Any]) -> str:
    reasons = summary.get('limiting_reason_counts', {})
    reason_names = {'liquid': 'рабочая жидкость', 'battery': 'энергия аккумулятора', 'task_complete': 'завершение назначенной площади', 'wind_energy': 'оперативный возврат по энергии при погодном сценарии'}
    reason_text = ', '.join((f'{reason_names.get(k, k)}: {v}' for k, v in reasons.items())) or 'нет данных'
    lines = [f"СЦЕНАРИЙ: {summary['scenario']}", f"Статус: {summary['status']}", f"Погода: {summary['weather_mode']}; seed: {summary['random_seed']}", f"Досрочных возвратов: {summary['early_returns']}; задержек сервиса: {summary['service_delay_events']}", f"Из досрочных возвратов в обычных условиях (консервативный запас): {summary['early_returns_normal_conditions']}", f"Минимальный фактический SoC возврата: {summary['minimum_actual_return_soc'] * 100:.2f}%", '', 'ОСНОВНОЙ РЕЗУЛЬТАТ', f"  Площадь поля: {summary['field_area_ha']:.2f} га", f"  Обработано: {summary['processed_area_ha']:.2f} га", f"  Завершение обработки: {format_duration(summary['processing_completion_time_s'])} ({summary['processing_completion_time_h']:.3f} ч)", f"  Все БПЛА возвращены: {format_duration(summary['all_drones_recovered_time_s'])}", f"  Эффективная производительность комплекса: {summary['effective_productivity_ha_h']:.2f} га/ч", '', 'ВЫЛЕТЫ', f"  Завершённых вылетов: {summary['completed_missions']}", f"  Незавершённых вылетов: {summary['unfinished_missions']}", f"  Средняя площадь одного вылета: {summary['average_sortie_area_ha']:.2f} га", f'  Причины ограничения/возврата: {reason_text}', '', 'АККУМУЛЯТОРНАЯ ПОДСИСТЕМА', f"  Батарей / зарядных каналов / мест охлаждения: {summary['battery_count']} / {summary['charger_channels']} / {summary['cooling_slots']}", f"  Средняя длительность заряда: {summary['average_charge_duration_s']:.1f} с", f"  Среднее / максимальное ожидание зарядника: {summary['average_charger_queue_wait_s']:.1f} / {summary['maximum_charger_queue_wait_s']:.1f} с", f"  Среднее / максимальное ожидание охлаждения: {summary['average_cooling_queue_wait_s']:.1f} / {summary['maximum_cooling_queue_wait_s']:.1f} с", f"  Загрузка зарядников: {summary['charger_utilization'] * 100:.1f}%", f"  Загрузка мест охлаждения: {summary['cooling_utilization'] * 100:.1f}%", f"  Разных АКБ использовано / всего в системе: {summary['unique_batteries_used']} / {summary['battery_count']}", f"  Замен АКБ / вылетов на всех АКБ: {summary['battery_swap_count']} / {summary['battery_flight_uses']}", f"  Минимум готовых АКБ: {summary['minimum_ready_batteries']}", f"  Пик зарядки / охлаждения / очереди зарядки: {summary['peak_charging_batteries']} / {summary['peak_cooling_batteries']} / {summary['peak_waiting_for_charger']}", f"  Случаев ожидания готовой батареи: {summary['battery_shortage_events']}", f"  Минимальный расчётный SoC после вылета: {summary['minimum_mission_end_soc'] * 100:.1f}%", f"  Разряжено / возвращено в батареи: {summary['battery_energy_discharged_wh'] / 1000:.2f} / {summary['battery_energy_charged_wh'] / 1000:.2f} кВт·ч", '', 'ДРОНОПОРТ', f"  Средняя задержка назначения ячейки: {summary['average_cell_allocation_delay_s']:.1f} с", f"  Максимальная задержка назначения ячейки: {summary['maximum_cell_allocation_delay_s']:.1f} с", f"  Среднее фактическое ожидание у станции: {summary['average_cell_wait_s']:.1f} с", f"  Максимальное ожидание у станции: {summary['maximum_cell_wait_s']:.1f} с", f"  Среднее ожидание сервиса внутри (включая готовность АКБ): {summary['average_gantry_wait_s']:.1f} с", f"  Максимальное ожидание сервиса внутри: {summary['maximum_gantry_wait_s']:.1f} с", f"  Средний наземный цикл между вылетами: {summary['average_ground_cycle_s']:.1f} с", f"  Среднее финальное размещение после последнего вылета: {summary['average_final_recovery_s']:.1f} с", f"  Ожидание внутри из-за манипулятора / АКБ, суммарно по дронам: {summary['internal_wait_manipulator_total_s']:.1f} / {summary['internal_wait_battery_total_s']:.1f} с", f"  Загрузка сервисного портала: {summary['gantry_utilization'] * 100:.1f}%", f"  Третья ячейка: {('рабочая' if summary['third_cell_active'] else 'резервная')}", f"  Суммарное ожидание в воздухе: {summary['air_holding_total_s']:.1f} с", f"  Использований третьей ячейки: {summary['third_cell_uses']}", f"  Конфликтов ресурсов: {summary['resource_conflicts']}", '', 'КОНТРОЛЬ КОРРЕКТНОСТИ', f"  Записей журнала: {summary['event_log_records']}", f"  Проверок инвариантов: {summary['invariant_checks']}", f"  Нарушений инвариантов: {summary['invariant_violations']}"]
    return '\n'.join(lines) + '\n'
def print_summary(summary: Dict[str, Any]) -> None:
    print(format_summary_text(summary), end='')
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Discrete-event agricultural droneport simulation v8.0')
    parser.add_argument('--config', type=Path, default=None, help='Path to a JSON configuration file (built-in defaults are used if omitted)')
    parser.add_argument('--output-dir', type=Path, default=Path(DEFAULT_CONFIG['experiment']['output_dir']), help='Directory for logs and summaries')
    parser.add_argument('--dump-default-config', type=Path, default=None, metavar='PATH', help='Write the built-in JSON configuration to PATH and exit')
    parser.add_argument('--run-scenarios', action='store_true', help='Run every scenario from the configuration')
    parser.add_argument('--self-test', action='store_true', help='Run deterministic and audit-log checks before the simulation')
    parser.add_argument('--third-cell-active', type=int, choices=(0, 1), default=None, help='0: reserve cell 3; 1: working cell 3')
    parser.add_argument('--active-drones', type=int, choices=(1, 2, 3), default=None, help='Set both fleet size and number of working drones')
    parser.add_argument('--area-ha', type=float, default=None)
    parser.add_argument('--no-plots', action='store_true', help='Run without matplotlib or figure export')
    parser.add_argument('--battery-sweep', action='store_true', help='Compare configured battery inventories')
    parser.add_argument('--scenario', default=None, help='Named scenario from the configuration')
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--replications', type=int, default=None, help='Replications per stochastic scenario')
    parser.add_argument('--wind-mode', choices=('none', 'steady', 'episodes'), default=None)
    parser.add_argument('--random-enabled', type=int, choices=(0, 1), default=None)
    return parser
class ServiceArchitectureSimulation(DroneportSimulation):
    def __init__(self, config, scenario_name, independent=False):
        super().__init__(config, scenario_name)
        if self.config['failures'] or self.config['communication_outages']:
            raise ValueError('This controlled comparison excludes configured outages')
        self.independent = independent
        if independent:
            self.unit_for_cell = {key: ServiceGantry(state='IDLE', gantry_id='LOCAL-' + key) for key in self.cells}
            self.service_units = list(self.unit_for_cell.values())
        else:
            self.unit_for_cell = {key: self.gantry for key in self.cells}
            self.service_units = [self.gantry]
        self.config['comparison_service_architecture'] = {'type': 'independent_per_cell' if independent else 'shared', 'installed_service_mechanisms': len(self.service_units), 'same_phase_durations': True, 'shared_battery_and_charging_system': True}
        self._dispatching_units = False
    def _select_drone_unit(self, drone):
        self.gantry = self.unit_for_cell[drone.assigned_cell_id]
    def _request_gantry(self, drone):
        self._select_drone_unit(drone)
        super()._request_gantry(drone)
    def _dispatch_gantry_queue(self):
        if not hasattr(self, 'service_units'):
            return super()._dispatch_gantry_queue()
        if self._dispatching_units:
            return
        self._dispatching_units = True
        try:
            while self.gantry_queue and self._ready_battery_ids():
                eligible = None
                for request in sorted(self.gantry_queue):
                    drone = self.drones[request[2]]
                    unit = self.unit_for_cell[drone.assigned_cell_id]
                    if unit.state == 'IDLE' and (not unit.fault_state):
                        eligible = request
                        break
                if eligible is None:
                    break
                pending = list(self.gantry_queue)
                pending.remove(eligible)
                heapq.heapify(pending)
                self.gantry_queue = [eligible]
                self._select_drone_unit(self.drones[eligible[2]])
                super()._dispatch_gantry_queue()
                assert not self.gantry_queue
                self.gantry_queue = pending
        finally:
            self._dispatching_units = False
    def _on_manipulator_phase_complete(self, payload):
        self._select_drone_unit(self.drones[payload['drone_id']])
        return super()._on_manipulator_phase_complete(payload)
    def _on_battery_swap_complete(self, payload):
        self._select_drone_unit(self.drones[payload['drone_id']])
        return super()._on_battery_swap_complete(payload)
    def _start_platform(self, cell, target, continuation):
        if hasattr(self, 'unit_for_cell'):
            self.gantry = self.unit_for_cell[cell.cell_id]
        return super()._start_platform(cell, target, continuation)
    def _check_invariants(self):
        self.gantry = self.service_units[0]
        super()._check_invariants()
        targets = []
        for unit in self.service_units:
            if unit.target_drone_id:
                targets.append(unit.target_drone_id)
                cell = self.cells[unit.target_cell_id]
                if cell.platform_position != 'INSIDE' or cell.occupied_by != unit.target_drone_id:
                    raise SimulationError('A local service unit has a moving or inconsistent target')
            if unit.state == 'SERVICING' and (not unit.target_drone_id):
                raise SimulationError('An active local service unit has no target')
        if len(targets) != len(set(targets)):
            raise SimulationError('An aircraft is served by multiple service units')
        reserved = [d.pending_battery_id for d in self.drones.values() if d.pending_battery_id]
        if len(reserved) != len(set(reserved)):
            raise SimulationError('A replacement battery is assigned twice')
    def run(self):
        max_time = float(self.config['simulation']['max_time_s'])
        while self._scheduled:
            event = heapq.heappop(self._scheduled)
            if event.time_s > max_time + EPS:
                raise SimulationError('Controlled comparison exceeded maximum time')
            delta = event.time_s - self.time_s
            for drone_id in self._queued_for_gantry:
                drone = self.drones[drone_id]
                unit = self.unit_for_cell[drone.assigned_cell_id]
                if unit.state != 'IDLE':
                    drone.queue_manipulator_wait_s += delta
                else:
                    drone.queue_battery_wait_s += delta
            self.time_s = event.time_s
            self._handle_event(event)
            self._capture_battery_inventory()
            self._check_invariants()
            if self._is_complete():
                self.return_completion_time_s = self.time_s
                break
        if not self._is_complete():
            raise SimulationError('Controlled comparison ended before completion')
        return self.summary()
    def summary(self):
        self.gantry = self.service_units[0]
        result = super().summary()
        end = self.return_completion_time_s or self.time_s
        utilizations = {u.gantry_id: u.duration_in('SERVICING', end) / end for u in self.service_units}
        result['service_unit_utilizations'] = utilizations
        result['gantry_utilization'] = sum(utilizations.values()) / len(utilizations)
        result['independent_service'] = self.independent
        result['installed_service_mechanisms'] = len(self.service_units)
        return result
def refill_time_s(missing_l, setup_s, flow_l_min):
    if not all((math.isfinite(x) for x in (missing_l, setup_s, flow_l_min))):
        raise ValueError('Refill parameters must be finite')
    if missing_l < 0 or setup_s < 0 or flow_l_min <= 0:
        raise ValueError('Volume/setup must be nonnegative and flow must be positive')
    return setup_s + 60.0 * missing_l / flow_l_min if missing_l > EPS else 0.0
class RefinedTimingMixin:
    def __init__(self, config, scenario_name='refined', *args, **kwargs):
        config = deepcopy(config)
        self.timing_model = {'refill_mode': 'fixed', 'refill_setup_s': 20.0, 'refill_flow_l_min': 20.0, 'blade_fold_s': 0.0, 'blade_unfold_s': 0.0, 'blade_mechanism': 'local_per_cell'}
        self.timing_model.update(config.get('service_timing_model', {}))
        if self.timing_model['refill_mode'] not in {'fixed', 'volume'}:
            raise ValueError('Unknown refill mode')
        if self.timing_model['blade_mechanism'] != 'local_per_cell':
            raise ValueError('A shared blade mechanism requires a separate resource/geometry model')
        refill_time_s(1, float(self.timing_model['refill_setup_s']), float(self.timing_model['refill_flow_l_min']))
        for key in ('blade_fold_s', 'blade_unfold_s'):
            value = float(self.timing_model[key])
            if not math.isfinite(value) or value < 0:
                raise ValueError(key + ' must be finite and nonnegative')
        config['service_timing_model'] = deepcopy(self.timing_model)
        self.refills = []
        self.blade_operations = []
        self.blades_folded = {}
        self._refill_pending = {}
        super().__init__(config, scenario_name, *args, **kwargs)
        self.blades_folded = {key: False for key in self.drones}
    def _sample_duration(self, duration, operation):
        if operation == 'refill' and self.timing_model['refill_mode'] == 'volume':
            self.sampled_operations.append({'time_s': self.time_s, 'operation': operation, 'duration_s': duration})
            return duration
        return super()._sample_duration(duration, operation)
    def _start_manipulator_phase(self, drone, phase, duration):
        if phase == 'REFILL':
            missing = max(0.0, float(self.config['flight']['tank_capacity_l']) - drone.remaining_liquid_l)
            row = {'drone_id': drone.drone_id, 'start_s': self.time_s, 'volume_l': missing, 'mode': self.timing_model['refill_mode']}
            if row['mode'] == 'volume':
                setup = self._sample_duration(float(self.timing_model['refill_setup_s']), 'refill_setup')
                flow = self._sample_duration(float(self.timing_model['refill_flow_l_min']), 'refill_flow_l_min')
                duration = refill_time_s(missing, setup, flow)
                row.update(setup_s=setup, flow_l_min=flow, predicted_duration_s=duration)
            self._refill_pending[drone.drone_id] = row
        return super()._start_manipulator_phase(drone, phase, duration)
    def _on_refill_complete(self, drone):
        row = self._refill_pending.pop(drone.drone_id)
        row['end_s'] = self.time_s
        row['duration_s'] = self.time_s - row['start_s']
        self.refills.append(row)
        super()._on_refill_complete(drone)
    def _start_blade_operation(self, drone, kind):
        cell = self.cells[drone.assigned_cell_id]
        if cell.platform_position != 'OUTSIDE':
            raise SimulationError('Blade preparation requires a stationary external platform')
        key = 'blade_fold_s' if kind == 'FOLD' else 'blade_unfold_s'
        duration = self._sample_duration(float(self.timing_model[key]), key)
        row = {'drone_id': drone.drone_id, 'cell_id': cell.cell_id, 'kind': kind, 'start_s': self.time_s, 'duration_s': duration, 'end_s': self.time_s + duration, 'resource': 'LOCAL-' + cell.cell_id}
        self.blade_operations.append(row)
        self._change_state(drone, 'BLADE_' + kind, 'DRONE', drone.drone_id, cell.cell_id, 'Local blade preparation on stationary external platform')
        self._log('OPERATION', 'BLADE_' + kind + '_STARTED', 'DRONE', drone.drone_id, related_object_id=cell.cell_id, details=row)
        self._schedule(self.time_s + duration, 'local_blade_complete', {'drone_id': drone.drone_id, 'kind': kind})
    def _handle_event(self, event):
        if event.kind == 'fixation_complete' and self.timing_model['blade_fold_s'] > 0:
            self._start_blade_operation(self.drones[event.payload['drone_id']], 'FOLD')
        elif event.kind == 'local_blade_complete':
            drone = self.drones[event.payload['drone_id']]
            folding = event.payload['kind'] == 'FOLD'
            self.blades_folded[drone.drone_id] = folding
            self._log('OPERATION', 'BLADE_' + event.payload['kind'] + '_COMPLETE', 'DRONE', drone.drone_id, related_object_id=drone.assigned_cell_id)
            if folding:
                self._start_platform(self.cells[drone.assigned_cell_id], 'INSIDE', 'INBOUND')
            else:
                super()._start_final_check(drone)
        else:
            return super()._handle_event(event)
    def _start_final_check(self, drone):
        if self.blades_folded.get(drone.drone_id) and self.timing_model['blade_unfold_s'] > 0:
            self._start_blade_operation(drone, 'UNFOLD')
        else:
            self.blades_folded[drone.drone_id] = False
            return super()._start_final_check(drone)
    def _on_final_check_complete(self, drone):
        if self.blades_folded.get(drone.drone_id):
            raise SimulationError('Takeoff blocked while blades are folded')
        return super()._on_final_check_complete(drone)
    def summary(self):
        result = super().summary()
        for key in ('duration_s', 'volume_l'):
            values = [row[key] for row in self.refills]
            stem = 'refill_' + key
            result[stem + '_mean'] = statistics.mean(values) if values else 0.0
            result[stem + '_min'] = min(values, default=0.0)
            result[stem + '_max'] = max(values, default=0.0)
        result['total_refilled_l'] = sum((row['volume_l'] for row in self.refills))
        result['blade_operation_count'] = len(self.blade_operations)
        result['service_timing_model'] = deepcopy(self.timing_model)
        return result
class RefinedServiceSimulation(RefinedTimingMixin, ServiceArchitectureSimulation):
    pass
class RefinedFullSimulation(RefinedTimingMixin, DroneportSimulation):
    independent = False
def audit(model):
    active = {}
    intervals = []
    for row in model.records:
        if row.operation == 'MANIPULATOR_PHASE_STARTED':
            key = row.related_object_id
            if row.details['phase'] == 'MOVE':
                assert key not in active
                active[key] = [row.simulation_time_s, 0.0]
            active[key][1] += row.details['duration_s']
        elif row.operation == 'SERVICE_COMPLETE':
            start, duration = active.pop(row.object_id)
            assert math.isclose(row.simulation_time_s - start, duration, abs_tol=2e-06)
            intervals.append((start, row.simulation_time_s))
    assert not active and len(intervals) == len(model.refills)
    overlaps = sum((a[0] < b[1] - 1e-08 and b[0] < a[1] - 1e-08 for i, a in enumerate(intervals) for b in intervals[i + 1:]))
    if not model.independent:
        assert overlaps == 0
    final_liquid = sum((d.remaining_liquid_l for d in model.drones.values()))
    initial_liquid = len(model.active_drone_ids) * model.config['flight']['tank_capacity_l']
    used = sum((m.liquid_used_l for m in model.missions.values()))
    refilled = sum((row['volume_l'] for row in model.refills))
    residual = initial_liquid + refilled - used - final_liquid
    assert math.isclose(residual, 0, abs_tol=1e-06), residual
    for row in model.refills:
        if row['mode'] == 'volume':
            assert math.isclose(row['duration_s'], row['predicted_duration_s'], abs_tol=1e-07)
    for resource in {r['resource'] for r in model.blade_operations}:
        rows = sorted((r for r in model.blade_operations if r['resource'] == resource), key=lambda r: r['start_s'])
        assert all((a['end_s'] <= b['start_s'] + 1e-08 for a, b in zip(rows, rows[1:])))
    if model.timing_model['blade_fold_s'] > 0:
        folded = {key: False for key in model.drones}
        for row in model.records:
            if row.operation == 'BLADE_FOLD_COMPLETE':
                folded[row.object_id] = True
            elif row.operation == 'BLADE_UNFOLD_COMPLETE':
                folded[row.object_id] = False
            elif row.operation == 'PLATFORM_MOVE_STARTED' and row.details['target'] == 'INSIDE':
                if row.related_object_id:
                    assert folded[row.related_object_id], 'Retraction before blade folding'
            elif row.operation == 'FINAL_CHECK_PASSED':
                assert not folded[row.object_id], 'Departure before blade unfolding'
    assert model.summary()['invariant_violations'] == 0
    return {'service_cycles': len(intervals), 'service_overlap_pairs': overlaps, 'liquid_balance_residual_l': residual, 'blade_operations': len(model.blade_operations), 'local_blade_resources': 'one per cell; no gantry occupation', 'original_invariant_violations': 0}
BASE_PATCH = {'field': {'area_ha': 150}, 'timing': {'fixation_s': 25}, 'service_timing_model': {'refill_mode': 'volume', 'refill_setup_s': 20, 'refill_flow_l_min': 20, 'blade_fold_s': 20, 'blade_unfold_s': 20, 'blade_mechanism': 'local_per_cell'}, 'plots': {'enabled': False}}
def config(n, distance=1000, patch=None):
    cfg = deep_merge(BASE_PATCH, {'fleet': {'total_drones': n, 'max_active_drones': n}, 'station': {'third_cell_active': int(n == 3)}, 'flight': {'transit_distance_m': distance}})
    return deep_merge(cfg, patch or {})
def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
def write_csv(path, rows):
    keys = [k for k, v in rows[0].items() if not isinstance(v, (dict, list))]
    with path.open('w', encoding='utf-8-sig', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
def run(cfg, name, out, independent=None):
    sim = RefinedFullSimulation(cfg, name) if independent is None else RefinedServiceSimulation(cfg, name, independent)
    result = sim.run()
    checks = audit(sim)
    assert result['processed_area_ha'] == 150
    assert result['unfinished_missions'] == 0
    assert result['invariant_violations'] == 0
    result['weather_timeline_sha256'] = hashlib.sha256(json.dumps(sim.weather_timeline, sort_keys=True).encode()).hexdigest()
    data = {'config': sim.config, 'summary': result, 'audit': checks}
    write_json(out / 'cases' / (name + '.json'), data)
    return result
def compute(out):
    out.mkdir(parents=True, exist_ok=True)
    raw = []
    groups = []
    for scenario, override in DEFAULT_CONFIG['scenarios'].items():
        for n in (2, 3):
            count = 1 if scenario in {'baseline', 'steady_wind'} else 30
            results = []
            for rep in range(count):
                patch = deep_merge(override, {'randomness': {'seed': 42 + rep}})
                result = run(config(n, patch=patch), f'{scenario}_n{n}_rep{rep + 1:02}', out)
                result.update(scenario=scenario, n=n, replication=rep + 1)
                raw.append(result)
                results.append(result)
            times = [x['processing_completion_time_h'] for x in results]
            group = {'scenario': scenario, 'n': n, 'replications': count, 'time_mean_h': statistics.mean(times), 'time_std_h': statistics.stdev(times) if count > 1 else 0, 'time_min_h': min(times), 'time_max_h': max(times), 'time_p95_h': percentile(times, 0.95), 'early_returns_mean': statistics.mean((x['early_returns'] for x in results)), 'gantry_utilization_mean': statistics.mean((x['gantry_utilization'] for x in results)), 'charger_utilization_mean': statistics.mean((x['charger_utilization'] for x in results)), 'invariant_violations': 0}
            groups.append(group)
            print(scenario, n, f"{group['time_mean_h']:.6f}", f"sd={group['time_std_h']:.6f}", flush=True)
    assert len(raw) == 244
    for scenario in ('wind_episodes', 'combined'):
        for rep in range(1, 31):
            pair = [x for x in raw if x['scenario'] == scenario and x['replication'] == rep]
            assert len({x['weather_timeline_sha256'] for x in pair}) == 1
    write_json(out / 'scenario_summary.json', raw)
    write_json(out / 'scenario_statistics.json', groups)
    write_csv(out / 'scenario_summary.csv', raw)
    write_csv(out / 'scenario_statistics.csv', groups)
    architecture = []
    for n in (2, 3):
        for distance in (1000, 250):
            for local in (False, True):
                name = f'architecture_n{n}_L{distance}_local{int(local)}'
                result = run(config(n, distance), name, out, independent=local)
                result.update(n=n, distance_m=distance, independent=local)
                architecture.append(result)
                if not local and distance == 1000:
                    baseline = next((x for x in raw if x['scenario'] == 'baseline' and x['n'] == n))
                    for key in ('processing_completion_time_h', 'average_ground_cycle_s', 'internal_wait_battery_total_s', 'charger_utilization'):
                        assert result[key] == baseline[key], (n, key)
    sensitivity = []
    for case, power, charge in [('nominal', 1, 1), ('power_low', 0.9, 1), ('power_high', 1.1, 1), ('charge_fast', 1, 0.8), ('charge_slow', 1, 1.2)]:
        for n in (2, 3):
            for distance in (1000, 250):
                for local in (False, True):
                    cfg = config(n, distance, {'flight': {'power_calibration_factor': power}, 'battery_model': {'charge_reference_time_s': 540 * charge}})
                    name = f'sensitivity_{case}_n{n}_L{distance}_local{int(local)}'
                    result = run(cfg, name, out, independent=local)
                    sensitivity.append({'case': case, 'n': n, 'distance_m': distance, 'independent': local, 'time_h': result['processing_completion_time_h']})
    distances = []
    for n in (2, 3):
        for distance in (250, 500, 1000):
            result = run(config(n, distance), f'distance_n{n}_L{distance}', out)
            distances.append({'n': n, 'distance_m': distance, 'time_h': result['processing_completion_time_h'], 'refill_mean_s': result['refill_duration_s_mean'], 'sortie_area_ha': result['average_sortie_area_ha']})
    inventory = []
    for count in range(3, 11):
        cfg = config(2, patch={'battery_rack': {'battery_count': count, 'capacity': max(4, count - 2)}})
        result = run(cfg, f'battery_inventory_{count}', out)
        inventory.append({'count': count, 'time_h': result['processing_completion_time_h'], 'battery_wait_s': result['internal_wait_battery_total_s'], 'minimum_ready': result['minimum_ready_batteries']})
    reserve = []
    for cells, fault in [(2, False), (2, True), (3, True)]:
        patch = {'station': {'cells_count': cells, 'doors_count': cells, 'third_cell_active': 0}}
        if fault:
            patch['failures'] = [{'object_type': 'GARAGE_CELL', 'object_id': 'CELL-01', 'start_s': 32400, 'duration_s': 1800}]
        result = run(config(2, patch=patch), f'reserve_cells{cells}_fault{int(fault)}', out)
        reserve.append({'cells': cells, 'fault': fault, 'time_h': result['processing_completion_time_h'], 'air_holding_s': result['air_holding_total_s'], 'max_air_holding_s': result['maximum_cell_wait_s'], 'reserve_uses': result['cells'].get('CELL-03', {}).get('reservation_count', 0)})
    timing = []
    cases = [('nominal', {}), ('swap60', {'timing': {'battery_swap_s': 60}}), ('swap180', {'timing': {'battery_swap_s': 180}}), ('charge720', {'battery_model': {'charge_reference_time_s': 720}}), ('long_prep', {'timing': {'fixation_s': 40}, 'service_timing_model': {'blade_fold_s': 30, 'blade_unfold_s': 30}}), ('refill_slow', {'service_timing_model': {'refill_flow_l_min': 10, 'refill_setup_s': 40}}), ('refill_fast', {'service_timing_model': {'refill_flow_l_min': 30, 'refill_setup_s': 10}})]
    for case, patch in cases:
        for n in (2, 3):
            result = run(config(n, patch=patch), f'timing_{case}_n{n}', out)
            timing.append({'case': case, 'n': n, 'time_h': result['processing_completion_time_h'], 'refill_mean_s': result['refill_duration_s_mean']})
    result = {'description': __doc__, 'main_run_count': len(raw), 'additional_run_count': len(architecture) + len(sensitivity) + len(distances) + len(inventory) + len(reserve) + len(timing), 'baseline_patch': BASE_PATCH, 'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(), 'refined_model_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(), 'groups': groups, 'baseline': [x for x in raw if x['scenario'] == 'baseline'], 'architecture': architecture, 'sensitivity': sensitivity, 'distances': distances, 'inventory': inventory, 'reserve': reserve, 'timing': timing, 'checks': {'all_processed_area_ha': 150, 'resource_invariant_violations': 0, 'paired_weather_sequences_checked': 60, 'full_vs_adapter_baseline_equal': True}}
    write_json(out / 'article_results.json', result)
    for name in ('distances', 'inventory', 'reserve', 'timing', 'sensitivity'):
        write_csv(out / (name + '.csv'), result[name])
    print('Completed:', len(raw), 'main and', result['additional_run_count'], 'additional runs', flush=True)
    print('Inventory:', inventory, flush=True)
    print('Reserve:', reserve, flush=True)
    print('Distances:', distances, flush=True)
    return result
SOURCE = Path(__file__).resolve()
HERE = SOURCE.parent
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, default=Path('droneport_results'))
    parser.add_argument('--single', action='store_true')
    parser.add_argument('--drones', type=int, choices=(2, 3), default=2)
    parser.add_argument('--distance', type=float, default=1000.0)
    parser.add_argument('--scenario', choices=tuple(DEFAULT_CONFIG['scenarios']), default='baseline')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--independent', action='store_true')
    args = parser.parse_args()
    if args.single:
        patch = deep_merge(DEFAULT_CONFIG['scenarios'][args.scenario], {'randomness': {'seed': args.seed}})
        name = f'{args.scenario}_n{args.drones}_L{args.distance:g}_seed{args.seed}_local{int(args.independent)}'
        result = run(config(args.drones, args.distance, patch), name, args.output_dir, independent=True if args.independent else None)
        write_json(args.output_dir / 'summary.json', result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        if args.independent or args.drones != 2 or args.distance != 1000.0 or (args.scenario != 'baseline') or (args.seed != 42):
            parser.error('--drones, --distance, --scenario, --seed and --independent require --single')
        compute(args.output_dir)
    return 0
if __name__ == '__main__':
    raise SystemExit(main())
