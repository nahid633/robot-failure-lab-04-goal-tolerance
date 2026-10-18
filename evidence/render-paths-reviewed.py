"""Post-run label-layout improvement; no measurement or trial code changes."""
import csv,json,shutil
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

root=Path('/tmp/robot-failure-lab-04-main-20261018')
records=json.loads((root/'results.json').read_text())
fig,ax=plt.subplots(figsize=(9,5.3),layout='constrained')
for condition,color,where in [('baseline','#216c9e',(1.02,1.02)),('relaxed','#d16a32',(-.15,.93))]:
    r=next(r for r in records if r['block']==1 and r['condition']==condition)
    with (root/r['trial']/'world-pose.csv').open() as stream:
        data=[{k:float(v) for k,v in row.items() if k in ('stamp','x','y')} for row in csv.DictReader(stream)]
    data=[v for v in data if r['start_sim_s']<=v['stamp']<=r['end_sim_s']+1.5]
    label='25 cm tolerance' if condition=='baseline' else '75 cm tolerance'
    ax.plot([v['x'] for v in data],[v['y'] for v in data],color=color,label=label)
    endpoint=data[-1]['x'],data[-1]['y'];ax.plot(*endpoint,'o',color=color)
    ax.annotate(label+'\nSettled error {:.2f} m'.format(r['metrics']['settled_world_error_max_m']),
                xy=endpoint,xytext=where,ha='center',fontsize=9,color=color,
                arrowprops=dict(arrowstyle='->',color=color),
                bbox=dict(boxstyle='round,pad=.3',facecolor='white',edgecolor=color,alpha=.95))
ax.add_patch(Circle((1,.5),.30,fill=False,color='#a12939',ls='--',label='Fixed 30 cm position region'))
ax.plot(1,.5,'*',color='#222',ms=12,label='Commanded target')
ax.set_xlim(-2.2,1.65);ax.set_ylim(-.65,1.23);ax.set_aspect('equal');ax.grid(alpha=.15)
ax.set_xlabel('Gazebo world x (m)');ax.set_ylabel('Gazebo world y (m)')
ax.legend(loc='lower right',fontsize=8)
ax.set_title('Predeclared first pair · measured world trajectories\nSame target; changed completion tolerance',fontsize=12)
shutil.copy2(root/'first-pair-paths.png',root/'first-pair-paths-initial-layout.png')
fig.savefig(root/'first-pair-paths.png',dpi=170);plt.close(fig)
shutil.copy2(__file__,root/'render-paths-reviewed.py')
from PIL import Image
with Image.open(root/'first-pair-recorded.gif') as image:
    print('Animation:',image.size,'frames:',image.n_frames)
    for index,label in [(image.n_frames//2,'mid'),(image.n_frames-1,'last')]:
        image.seek(index);image.convert('RGB').save('/tmp/rfl04-animation-'+label+'.png')
