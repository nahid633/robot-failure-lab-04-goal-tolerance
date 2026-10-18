"""Recompute endpoint quality and validate recorded configuration/telemetry."""
import csv
import json
from core import PARAMETER,TOLERANCES,START,endpoint_metrics


def rows(path):
    with path.open() as stream:
        return [{k:(v if k in ('frame','child') else float(v)) for k,v in row.items()}
                for row in csv.DictReader(stream)]


def validate_trial(root,record):
    folder=root/record['trial'];start=record['start_sim_s'];end=record['end_sim_s']
    before=json.loads((folder/'effective-parameters-before.json').read_text())
    after=json.loads((folder/'effective-parameters-after.json').read_text())
    assert before==after
    assert before['/controller_server'][PARAMETER]==TOLERANCES[record['condition']]
    assert before['/collision_monitor']['source_timeout']==1.0
    truth=rows(folder/'world-pose.csv');localized=rows(folder/'localized-pose.csv')
    assert record['metrics']==endpoint_metrics(truth,localized,start,end,record['outcome'])
    assert record['goal_accepted'] and record['terminal_action_status'] in (4,5,6)
    initial=record['initial_world_pose']
    assert abs(initial['x']-START[0])<.05 and abs(initial['y']-START[1])<.05
    assert set(record['graph']['/rfl04/truth']['subscribers'])=={'failure_lab_04_observer'}
    assert len(record['graph']['/rfl04/truth']['publishers'])==1
    assert {'amcl','local_costmap','global_costmap','collision_monitor'} <= set(record['graph']['/scan']['subscribers'])
    summary={}
    for name,data,gap in [('truth',truth,.15),('odom',rows(folder/'odom.csv'),.15),('scans',rows(folder/'scans.csv'),.5)]:
        group=[r for r in data if start<=r['stamp']<=end+1.5]
        assert len(group)>5
        assert group[0]['stamp']-start<gap and end+1.5-group[-1]['stamp']<gap
        gaps=[b['stamp']-a['stamp'] for a,b in zip(group,group[1:])]
        assert min(gaps)>=0 and max(gaps)<gap,(name,max(gaps))
        assert all(-.02<=r['t']-r['stamp']<gap for r in group),(name,'stale receipts')
        summary[name]=dict(samples=len(group),max_gap_s=max(gaps))
    assert len({r['frame'] for r in truth})==1 and truth[0]['frame'] in ('default','world')
    assert all(r['child'].split('::')[-1]=='turtlebot3_waffle' for r in truth)
    assert rows(folder/'commands.csv') and rows(folder/'feedback.csv')
    assert json.loads((folder/'TASK_TERMINAL.json').read_text())==dict(t=end,outcome=record['outcome'])
    return dict(passed=True,telemetry=summary,parameters_unchanged=True,
                scope='Independent Gazebo world pose; map/world alignment inherited from supplied sandbox and spawn coordinates. Position acceptance only.')
