"""Experiment 04's frozen comparison and independent position requirement."""
import copy
import math

CONDITIONS = ('baseline', 'relaxed')
TOLERANCES = {'baseline': 0.25, 'relaxed': 0.75}
START = (-2.0, -0.5)
GOAL = (1.0, 0.5)
POSITION_LIMIT = 0.30
PARAMETER = 'general_goal_checker.xy_goal_tolerance'


def configuration(base, condition):
    result = copy.deepcopy(base)
    result['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'] = TOLERANCES[condition]
    return result


def differences(a, b, prefix=''):
    if isinstance(a, dict) and isinstance(b, dict):
        result = []
        for key in sorted(set(a) | set(b)):
            name = f'{prefix}.{key}' if prefix else key
            if key not in a or key not in b:
                result.append((name, a.get(key), b.get(key)))
            else:
                result.extend(differences(a[key], b[key], name))
        return result
    return [] if a == b else [(prefix, a, b)]


def distance(row):
    return math.hypot(row['x'] - GOAL[0], row['y'] - GOAL[1])


def endpoint_metrics(truth, localized, start, end, outcome):
    # Truth stamp is simulation acquisition time, not ROS receipt time.
    def terminal(rows):
        eligible = [r for r in rows if start <= r['stamp'] <= end]
        if not eligible or end - eligible[-1]['stamp'] > 0.15:
            raise ValueError('Missing/far-stale terminal pose')
        return eligible[-1]
    tr = terminal(truth)
    loc = terminal(localized)
    settled = [r for r in truth if end + 1.0 <= r['stamp'] <= end + 1.5]
    if len(settled) < 8 or settled[0]['stamp'] > end + 1.1 or settled[-1]['stamp'] < end + 1.4:
        raise ValueError('Incomplete settled-position window')
    if max(b['stamp'] - a['stamp'] for a, b in zip(settled, settled[1:])) > 0.15:
        raise ValueError('Sparse settled-position window')
    route = [r for r in truth if start <= r['stamp'] <= end]
    peak = max(distance(r) for r in settled)
    return dict(terminal_world_error_m=distance(tr), terminal_localized_error_m=distance(loc),
                terminal_world_stamp=tr['stamp'], terminal_localized_stamp=loc['stamp'],
                terminal_world_x=tr['x'], terminal_world_y=tr['y'],
                terminal_world_yaw_error_rad=abs(math.atan2(math.sin(tr['yaw']), math.cos(tr['yaw']))),
                settled_world_error_max_m=peak,
                settled_world_error_mean_m=sum(distance(r) for r in settled) / len(settled),
                settled_position_pass=bool(outcome == 'succeeded' and peak <= POSITION_LIMIT),
                world_path_length_m=sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(route,route[1:])),
                settled_samples=len(settled))
