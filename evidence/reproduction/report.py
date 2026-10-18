#!/usr/bin/env python3
"""All-trial report, two figures and predeclared first-pair measured animation."""
import argparse,json,hashlib,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from matplotlib.patches import Circle
from PIL import Image
import yaml
from core import CONDITIONS,GOAL,POSITION_LIMIT,differences
from validate import rows,validate_trial

COLORS={'baseline':'#216c9e','relaxed':'#d16a32'}
LABELS={'baseline':'25 cm tolerance','relaxed':'75 cm tolerance'}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    args=parser.parse_args();root=args.root.resolve()
    records=json.loads((root/'results.json').read_text())
    protocol=json.loads((root/'PROTOCOL.json').read_text())
    expected=2 if protocol['verification'] else 6
    assert len(records)==expected
    for name,digest in protocol['source_sha256'].items():
        assert hashlib.sha256((root/'source'/name).read_bytes()).hexdigest()==digest
    checks={r['trial']:validate_trial(root,r) for r in records}
    configs={c:yaml.safe_load((root/f'{c}-params.yaml').read_text()) for c in CONDITIONS}
    assert differences(configs['baseline'],configs['relaxed'])==[
        ('controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance',.25,.75)]
    first_before={c:json.loads((root/next(r['trial'] for r in records if r['condition']==c)/'effective-parameters-before.json').read_text()) for c in CONDITIONS}
    runtime_delta=differences(first_before['baseline'],first_before['relaxed'])
    assert runtime_delta==[('/controller_server.general_goal_checker.xy_goal_tolerance',.25,.75)],runtime_delta
    for r in records:
        actual=json.loads((root/r['trial']/'effective-parameters-before.json').read_text())
        assert actual==first_before[r['condition']],r['trial']
    (root/'EVIDENCE-VALIDATION.json').write_text(json.dumps(dict(passed=True,trials=checks,
        only_runtime_difference=runtime_delta,all_source_hashes_match=True),indent=2))
    table=['| Block | Configuration | Nav2 result | Time (sim s) | World error at result (m) | Settled max error (m) | Fixed 30 cm position gate | Recoveries |',
           '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in records:
        m=r['metrics'];table.append(f'| {r["block"]} | {LABELS[r["condition"]]} | {r["outcome"]} | {r["elapsed_sim_s"]:.3f} | {m["terminal_world_error_m"]:.3f} | {m["settled_world_error_max_m"]:.3f} | {"pass" if m["settled_position_pass"] else "fail"} | {r["recoveries"]} |')
    summary=[]
    for c in CONDITIONS:
        group=[r for r in records if r['condition']==c]
        summary.append(f'- {LABELS[c]}: Nav2 success {sum(r["outcome"]=="succeeded" for r in group)}/{len(group)}; fixed position gate {sum(r["metrics"]["settled_position_pass"] for r in group)}/{len(group)}; goal-to-result times '+', '.join(f'{r["elapsed_sim_s"]:.3f}' for r in group)+' sim s.')
    pairs=[]
    for block in sorted({r['block'] for r in records}):
        a=next(r for r in records if r['block']==block and r['condition']=='baseline')
        b=next(r for r in records if r['block']==block and r['condition']=='relaxed')
        pairs.append(f'- Block {block}: relaxed minus baseline time {b["elapsed_sim_s"]-a["elapsed_sim_s"]:+.3f} sim s; settled error {b["metrics"]["settled_world_error_max_m"]-a["metrics"]["settled_world_error_max_m"]:+.3f} m. Timing is secondary; failed actions are not completed journeys.')
    report='\n'.join(['# Robot Failure Lab 04: one configuration change, two definitions of success','',
        'Measured Gazebo/Nav2 simulation. One route, three main pairs (or two excluded verification runs if this folder is verification). Standalone runner.','',
        '## Every trial','',*table,'','## Summary','',*summary,'','## Within-seed comparisons','',*pairs,'',
        '## Fixed method and limits','',
        '- Only navigation YAML difference: general_goal_checker.xy_goal_tolerance, 0.25 to 0.75 m. Live parameter snapshots before/after and across all trials verify exactly that difference.',
        '- Original route (-2,-0.5) to (1,0.5), yaw zero; fresh processes and equal 1.5-sim-second pre-goal wait. No sensor faults, replacement goals, disabled monitor or changed speed setting.',
        '- Position requirement was fixed before main runs: Nav2 success AND every world-pose sample 1.0–1.5 sim seconds after the result within 0.30m of target. This is an illustrative XY requirement, not a customer specification or docking test.',
        '- World pose comes from a common observer-only Gazebo PosePublisher plugin. It is not derived from AMCL or wheel odometry and is never fed into navigation TF. The supplied sandbox map/world correspondence is assumed; spawn XY is checked within 0.05m. Model/base frames have equal XY origins.',
        '- Terminal error uses the last acquired world pose at/before the action result, at most 0.15s old. Settled error uses a separately checked post-result window. Latest feedback is not substituted for ground truth.',
        '- Seeds 101/102/103 are matched within pairs; AMCL/MPPI/scheduling remain uncontrolled. Three pairs on one route do not prove a population speedup, general reliability or physical performance. Null/mixed timing remains visible.',
        '- This deliberately relaxes a documented acceptance policy. A mismatch against our fixed requirement is not a Nav2 bug or a production incident. Task success alone cannot establish unchanged task quality.',
        '- Fresh-machine reproduction, Vektor integration, external use and public posting remain unverified. Preserve all excluded checks and startup/process anomalies in the archive.',''])
    (root/'REPORT.md').write_text(report)
    fig,axes=plt.subplots(1,2,figsize=(11,4.8),layout='constrained')
    for c in CONDITIONS:
        group=[r for r in records if r['condition']==c];xs=[r['block'] for r in group]
        axes[0].plot(xs,[r['elapsed_sim_s'] for r in group],'o-',color=COLORS[c],label=LABELS[c])
        axes[1].plot(xs,[r['metrics']['settled_world_error_max_m'] for r in group],'o-',color=COLORS[c],label=LABELS[c])
    axes[0].set_ylabel('Goal to action result (sim seconds)');axes[0].set_title('Reported task time (all outcomes shown)')
    axes[1].axhline(POSITION_LIMIT,color='#a12939',ls='--',label='Fixed position limit: 30 cm')
    axes[1].set_ylabel('Max world-position error, result +1.0..1.5s (m)');axes[1].set_title('Arrival quality against an unchanged requirement')
    for ax in axes:
        ax.set_xlabel('Block / matched Gazebo seed');ax.set_xticks(sorted({r['block'] for r in records}));ax.set_ylim(bottom=0);ax.grid(alpha=.2);ax.legend(frameon=False,fontsize=8)
    fig.suptitle('One YAML change: goal tolerance 25 cm → 75 cm\nMeasured simulation · same target and navigation configuration',fontsize=13)
    fig.savefig(root/'comparison.png',dpi=170);plt.close(fig)
    first=min(r['block'] for r in records)
    chosen={c:next(r for r in records if r['block']==first and r['condition']==c) for c in CONDITIONS}
    trajectories={c:rows(root/r['trial']/'world-pose.csv') for c,r in chosen.items()}
    fig,ax=plt.subplots(figsize=(8,6),layout='constrained')
    for c,r in chosen.items():
        data=[v for v in trajectories[c] if r['start_sim_s']<=v['stamp']<=r['end_sim_s']+1.5]
        ax.plot([v['x'] for v in data],[v['y'] for v in data],color=COLORS[c],label=LABELS[c])
        ax.plot(data[-1]['x'],data[-1]['y'],'o',color=COLORS[c]);ax.text(data[-1]['x'],data[-1]['y']+.10,f'{LABELS[c]}\nerror {r["metrics"]["settled_world_error_max_m"]:.2f}m',fontsize=8)
    ax.add_patch(Circle(GOAL,POSITION_LIMIT,fill=False,color='#a12939',ls='--',label='Fixed 30 cm position region'))
    ax.plot(*GOAL,'*',color='#222',ms=12,label='Commanded target');ax.set_aspect('equal');ax.grid(alpha=.15)
    ax.set_xlabel('Gazebo world x (m)');ax.set_ylabel('Gazebo world y (m)');ax.legend(loc='lower right',fontsize=8)
    ax.set_title('Predeclared first pair · actual simulated world trajectories\nEndpoints shown after action termination; all pairs in report')
    fig.savefig(root/'first-pair-paths.png',dpi=170);plt.close(fig)
    # Top-down animation of recorded poses, not synthetic simulator footage.
    fig,axes=plt.subplots(1,2,figsize=(10,5.2),layout='constrained')
    map_data=yaml.safe_load((root/'tb3_sandbox.yaml').read_text());image=np.asarray(Image.open(root/'tb3_sandbox.pgm'))
    ox,oy,_=map_data['origin'];res=map_data['resolution'];height,width=image.shape
    artists={}
    for ax,c in zip(axes,CONDITIONS):
        ax.imshow(image,cmap='gray',vmin=0,vmax=255,extent=(ox,ox+width*res,oy,oy+height*res),origin='upper',alpha=.45)
        ax.add_patch(Circle(GOAL,POSITION_LIMIT,fill=False,color='#a12939',ls='--'))
        ax.plot(*GOAL,'*',color='#222',ms=10)
        trail,=ax.plot([],[],color=COLORS[c]);dot,=ax.plot([],[],'o',color=COLORS[c],ms=9)
        text=ax.text(.02,.02,'',transform=ax.transAxes,fontsize=9,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
        artists[c]=(trail,dot,text);ax.set_xlim(-2.6,1.7);ax.set_ylim(-1.6,1.6);ax.set_aspect('equal');ax.set_title(LABELS[c]);ax.set_xlabel('world x (m)')
    axes[0].set_ylabel('world y (m)')
    max_time=max(r['elapsed_sim_s']+1.5 for r in chosen.values())
    times=np.linspace(0,max_time,min(240,int(max_time*5)+1))
    def animate(t):
        output=[]
        for c,r in chosen.items():
            selected=[v for v in trajectories[c] if r['start_sim_s']<=v['stamp']<=min(r['start_sim_s']+t,r['end_sim_s']+1.5)]
            trail,dot,text=artists[c]
            if selected:
                trail.set_data([v['x'] for v in selected],[v['y'] for v in selected]);dot.set_data([selected[-1]['x']],[selected[-1]['y']])
                error=math.hypot(selected[-1]['x']-GOAL[0],selected[-1]['y']-GOAL[1])
                state=r['outcome'] if t>=r['elapsed_sim_s'] else 'navigating'
                text.set_text(f't = {t:.1f} sim s · {state}\nWorld position error: {error:.2f} m\nDashed circle: fixed 30 cm requirement')
            output.extend((trail,dot,text))
        return output
    fig.suptitle('Recorded simulation · predeclared first pair\nOne changed goal tolerance; original target unchanged',fontsize=12)
    animation=FuncAnimation(fig,animate,frames=times,interval=100,blit=False)
    animation.save(root/'first-pair-recorded.gif',writer=PillowWriter(fps=10),dpi=85);plt.close(fig)
    print(f'Validated {len(records)} trials; report, two figures and recorded-position animation saved.')


if __name__=='__main__':main()
