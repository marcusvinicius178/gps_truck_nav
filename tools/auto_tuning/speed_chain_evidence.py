"""Describe effective speed envelopes without inferring physical vehicle speed."""

import math


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def speed_chain_evidence(observed):
    nodes = observed.get('plugin_parameters', {}).get('nodes', {})
    controller = nodes.get('/controller_server', {}).get('values', {})
    planner = nodes.get('/planner_server', {}).get('values', {})
    smooth_record = observed.get('parameters', {}).get('/velocity_smoother', {})
    smooth = smooth_record.get('values', {}) if smooth_record.get('status') == 'OK' else {}
    def value(values, name):
        record = values.get(name, {})
        return record.get('value') if record.get('type') in (2, 3, 4) else None
    def component(values, name, index):
        array = values.get(name)
        return array[index] if isinstance(array, list) and len(array) == 3 and finite(array[index]) else None
    mppi = {name: value(controller, 'FollowPath.' + name)
            for name in ('vx_max', 'vx_min', 'wz_max', 'model_dt', 'time_steps', 'motion_model')}
    smoother = {name: component(smooth, key, index) for name, key, index in (
        ('vx_max', 'max_velocity', 0), ('vx_min', 'min_velocity', 0),
        ('wz_max', 'max_velocity', 2), ('wz_min', 'min_velocity', 2))}
    required = [mppi.get(key) for key in ('vx_max', 'vx_min', 'wz_max')] + list(smoother.values())
    available = all(finite(item) for item in required)
    result = {'schema_version': 1, 'status': 'OBSERVED' if available else 'INCOMPLETE',
        'navigation_profile': observed.get('navigation_profile', {}).get('name', 'UNKNOWN'),
        'mppi_effective': mppi, 'smoother_effective': smoother,
        'planner_effective': {'minimum_turning_radius_m': value(planner, 'GridBased.minimum_turning_radius'),
                              'motion_model': value(planner, 'GridBased.motion_model_for_search')},
        'smoother_feedback': smooth.get('feedback'),
        'smoother_scale_velocities': smooth.get('scale_velocities'),
        'smoother_velocity_timeout_s': smooth.get('velocity_timeout'),
        'units': {'vx': 'm/s', 'wz': 'rad/s', 'plugin_angular_input': 'steering rad (source contract)'},
        'physical_speed_verified': False, 'actuator_conversion_connected': False,
        'motion_authorized': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED',
        'scope': 'Typed parameter readback and graph snapshot only; no command or motion was generated.'}
    publishers = observed.get('topic_publishers', {}).get('/cmd_vel', [])
    result['cmd_vel_publishers'] = publishers
    result['cmd_vel_subscribers'] = observed.get('topic_subscribers', {}).get('/cmd_vel', [])
    result['publishers_bypassing_smoother'] = [entry for entry in publishers if entry.get('node_name') != 'velocity_smoother']
    result['command_path_status'] = ('BYPASS_PRESENT' if result['publishers_bypassing_smoother'] else
                                     'NO_BYPASS_OBSERVED_IN_SNAPSHOT' if publishers else 'UNKNOWN')
    if available:
        ceiling = min(mppi['vx_max'], smoother['vx_max'])
        yaw_ceiling = min(mppi['wz_max'], smoother['wz_max'], -smoother['wz_min'])
        result['nominal_forward_ceiling_on_smoother_path_mps'] = ceiling
        result['nominal_forward_ceiling_on_smoother_path_kmh'] = ceiling * 3.6
        result['matching_declared_envelopes'] = all(math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
            for a, b in ((mppi['vx_max'], smoother['vx_max']), (mppi['vx_min'], smoother['vx_min']),
                         (mppi['wz_max'], smoother['wz_max']), (-mppi['wz_max'], smoother['wz_min'])))
        if ceiling > 0 and yaw_ceiling > 0:
            result['ideal_radius_at_forward_ceiling_due_to_yaw_cap_m'] = ceiling / yaw_ceiling
        result['ceiling_limitation'] = 'Only the path through the smoother; bypasses, steering semantics, acceleration, curvature and dynamics prevent a physical-speed claim.'
    if all(finite(mppi[key]) and mppi[key] > 0 for key in ('model_dt', 'time_steps')):
        horizon = mppi['model_dt'] * mppi['time_steps']
        result['prediction_horizon_s'] = horizon
        if finite(mppi['vx_max']) and mppi['vx_max'] > 0:
            result['nominal_straight_horizon_at_mppi_cap_m'] = horizon * mppi['vx_max']
    return result
