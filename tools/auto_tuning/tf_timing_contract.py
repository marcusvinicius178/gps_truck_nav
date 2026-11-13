"""Assess TF against /clock captured by the same observer, without ROS imports.

This is a diagnostic of received data. It neither proves publication cadence nor
zero transport loss. The older collector lacked a co-observed clock and cannot
be retrospectively promoted to PASS using a later Python snapshot.
"""

import math


REQUIRED_EDGES = (('map', 'odom'), ('odom', 'base_footprint'),
                  ('base_footprint', 'base_link'))
REFERENCE = 'OBSERVER_LOCAL_CLOCK_SAME_WINDOW'


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def stamp_seconds(value):
    if not isinstance(value, dict):
        return None
    seconds, nanoseconds = value.get('sec'), value.get('nanosec')
    if not finite(seconds) or not finite(nanoseconds) or not 0 <= nanoseconds < 1_000_000_000:
        return None
    return seconds + nanoseconds / 1_000_000_000


def count(value, minimum):
    return isinstance(value, int) and not isinstance(value, bool) and value >= minimum


def assess_tf_timing(authority, policy):
    """Return one readiness check; caller retains the independent authority check.

    Only authority-local timing is used: the function deliberately has no Python
    final clock argument. Required policy values are the existing 1 s simulated
    age, 0.25 s future tolerance and 2 s clock-silence diagnostic limits.
    """
    result = {'name': 'required_tf_numeric_and_fresh', 'status': 'UNKNOWN',
              'clock_reference': authority.get('clock_reference'), 'edges': [],
              'scope': 'Received TF in the C++ observer window; not mission readiness or proof of zero loss.'}
    if (authority.get('schema_version') != 2 or authority.get('clock_reference') != REFERENCE
            or authority.get('observation_complete') is not True):
        result['reason'] = 'A complete observation with a co-observed clock is required; cross-window extrapolation is forbidden.'
        return result
    clock = authority.get('clock', {})
    end = authority.get('observation_window_end_wall_offset')
    received = clock.get('last_received_wall_offset')
    advanced_at = clock.get('last_advance_wall_offset')
    first, last = stamp_seconds(clock.get('first_stamp')), stamp_seconds(clock.get('last_stamp'))
    if not all(finite(value) for value in (end, received, advanced_at, first, last)):
        result['reason'] = 'Observer clock timestamps or wall-time reception evidence are missing.'
        return result
    reception_age, advancement_age = end - received, end - advanced_at
    clock_live = (count(clock.get('count'), 3) and count(clock.get('forward_steps'), 2)
                  and clock.get('backward_steps') == 0 and last > first
                  and all(0 <= age <= policy['maximum_clock_silence_wall_seconds']
                          for age in (reception_age, advancement_age)))
    result['clock'] = {'first_sim_seconds': first, 'last_sim_seconds': last,
                       'last_reception_age_wall_seconds': reception_age,
                       'last_advance_age_wall_seconds': advancement_age,
                       'window_end_wall_offset': end, 'live': clock_live}
    failures, unknowns = [], []
    if not clock_live:
        failures.append('Observer-local clock did not remain live and monotonically advancing.')
    edges = authority.get('edges', [])
    for parent, child in REQUIRED_EDGES:
        matches = [edge for edge in edges if (edge.get('parent'), edge.get('child')) == (parent, child)]
        if not matches:
            unknowns.append(f'Missing required edge {parent}->{child}.')
            continue
        for edge in matches:
            record = {'parent': parent, 'child': child, 'topic': edge.get('topic'),
                      'publishers': [], 'status': 'PASS'}
            transform = edge.get('last_transform', {})
            position, rotation = transform.get('translation', {}), transform.get('rotation', {})
            numeric = (all(finite(position.get(axis)) for axis in ('x', 'y', 'z'))
                       and all(finite(rotation.get(axis)) for axis in ('x', 'y', 'z', 'w')))
            numeric = numeric and abs(math.sqrt(sum(rotation[axis] ** 2 for axis in ('x', 'y', 'z', 'w'))) - 1) <= 1e-3
            if not numeric or edge.get('nonfinite_transform_count', 0):
                record['status'] = 'FAIL'
                failures.append(f'Invalid numeric transform {parent}->{child}.')
            if (parent, child) == ('base_footprint', 'base_link'):
                record['static_timestamp_exempt'] = True
                result['edges'].append(record)
                continue
            if edge.get('topic') != '/tf':
                record['status'] = 'FAIL'
                failures.append(f'Dynamic edge {parent}->{child} is not on /tf.')
            sequences = edge.get('publisher_observations', [])
            gids = set(edge.get('publisher_gids', []))
            observed_gids = {item.get('gid') for item in sequences}
            if (not sequences or not gids or '' in observed_gids or gids != observed_gids
                    or edge.get('publisher_gid_unknown_count', 0)):
                if record['status'] != 'FAIL':
                    record['status'] = 'UNKNOWN'
                unknowns.append(f'Missing per-publisher timing identity {parent}->{child}.')
            for sequence in sequences:
                timing = sequence.get('timing', {})
                source_first = stamp_seconds(timing.get('first_stamp'))
                source_last = stamp_seconds(timing.get('last_stamp'))
                arrival = timing.get('last_received_wall_offset')
                details = {'gid': sequence.get('gid'), 'status': 'UNKNOWN',
                           'count': timing.get('count'), 'source_first': source_first,
                           'source_last': source_last, 'last_received_wall_offset': arrival,
                           'source_interval_sim_seconds': timing.get('source_interval_sim_seconds'),
                           'reception_interval_wall_seconds': timing.get('reception_interval_wall_seconds')}
                if not all(finite(value) for value in (source_first, source_last, arrival)):
                    unknowns.append(f'Incomplete reception/source timing {parent}->{child}.')
                    if record['status'] != 'FAIL':
                        record['status'] = 'UNKNOWN'
                else:
                    age = last - source_last
                    details.update(age_sim_seconds=age, reception_age_wall_seconds=end - arrival)
                    progressing = (count(timing.get('count'), 2) and count(timing.get('forward_steps'), 1)
                                   and timing.get('backward_steps') == 0 and source_last > source_first)
                    healthy = (clock_live and progressing and 0 <= arrival <= end
                               and -policy['maximum_future_sim_seconds'] <= age <= policy['maximum_age_sim_seconds'])
                    details['status'] = 'PASS' if healthy else 'FAIL'
                    if not healthy:
                        record['status'] = 'FAIL'
                        failures.append(f'Stale, nonadvancing or invalid timing for {parent}->{child} GID {sequence.get("gid")}.')
                record['publishers'].append(details)
            result['edges'].append(record)
    result.update(status='FAIL' if failures else 'UNKNOWN' if unknowns else 'PASS',
                  failures=failures, unknowns=unknowns,
                  note='Static zero stamps are valid. Dynamic ages use only the same observer clock; reception jitter and middleware loss reports remain separate evidence.')
    return result
