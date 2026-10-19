"""Render report figures from retained measurements; never run a simulator."""
from pathlib import Path
import json
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch

HERE = Path(__file__).resolve().parent
DATA = HERE.parent
OUT = HERE / 'figures'
OUT.mkdir(exist_ok=True)
plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'DejaVu Sans'],
    'font.size': 8, 'axes.titlesize': 9, 'axes.labelsize': 8,
    'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': .7,
    'legend.frameon': False, 'svg.fonttype': 'none', 'pdf.fonttype': 42,
    'lines.linewidth': 1.1})
BLUE, ORANGE, GRAY = '#286C91', '#C57A30', '#777777'

def save(fig, name):
    fig.savefig(OUT / f'{name}.pdf', bbox_inches='tight', facecolor='white')
    fig.savefig(OUT / f'{name}.svg', bbox_inches='tight', facecolor='white')
    fig.savefig(OUT / f'{name}.png', dpi=600, bbox_inches='tight', facecolor='white')
    plt.close(fig)

# All observations appear in the time-domain panels. The steady mask implements
# the pre-existing two-period exclusion for summary metrics, not a fitted cutoff.
d = np.genfromtxt(DATA / 'unity_flight.csv', delimiter=',', names=True)
t = d['trajectory_time']
p = np.column_stack([d[k] for k in ('x', 'y', 'z')])
pd = np.column_stack([d[k] for k in ('xd', 'yd', 'zd')])
e = p-pd
err = np.linalg.norm(e, axis=1)
s = t >= 8*np.pi
assert np.isfinite(np.column_stack((t,p,pd))).all() and np.all(np.diff(t)>=0) and np.all(np.diff(d['t'])>0)
phase = np.unwrap(np.arctan2(p[s,0],p[s,1]))
m = {'samples':len(t), 'repeated_reference_times':int(np.sum(np.diff(t)==0)), 'steady_samples':int(s.sum()), 'excluded_from_steady_metrics':int((~s).sum()),
    'steady_first_sample_s':float(t[s][0]), 'steady_last_sample_s':float(t[s][-1]),
    'steady_position_rms_m':float(np.sqrt(np.mean(err[s]**2))),
    'steady_position_max_m':float(err[s].max()),
    'steady_radial_rms_m':float(np.sqrt(np.mean((np.linalg.norm(p[s,:2],axis=1)-5)**2))),
    'observed_steady_laps':float((phase[-1]-phase[0])/(2*np.pi)),
    'component_rms_m':np.sqrt(np.mean(e[s]**2,axis=0)).tolist(),
    'mean_height_error_m':float(e[s,2].mean()),
    'height_share_of_squared_position_error':float(np.mean(e[s,2]**2)/np.mean(err[s]**2))}
existing = json.loads((DATA/'unity_metrics.json').read_text())
for k in ('steady_position_rms_m','steady_position_max_m','steady_radial_rms_m','observed_steady_laps'):
    assert np.isclose(m[k],existing[k],rtol=1e-10), (k,m[k],existing[k])
(HERE/'qa/recomputed_metrics.json').write_text(json.dumps(m,indent=2)+'\n')
fig = plt.figure(figsize=(160/25.4,133/25.4), layout='constrained')
gs=fig.add_gridspec(3,2,width_ratios=[1.08,1])
a=fig.add_subplot(gs[:2,0])
a.plot(pd[:,0],pd[:,1],color=GRAY,ls='--',label='Reference')
a.plot(p[~s,0],p[~s,1],color=ORANGE,alpha=.85,label='Initial two periods')
a.plot(p[s,0],p[s,1],color=BLUE,label='Steady window')
a.scatter(p[0,0],p[0,1],s=18,color=ORANGE,zorder=5)
a.set(xlabel='x (m)',ylabel='y (m)',aspect='equal',xlim=(-6,6),ylim=(-6,6))
a.set_title('a  Circular path',loc='left',fontweight='bold'); a.legend(loc='upper center',bbox_to_anchor=(.5,-.18),fontsize=7)
b=fig.add_subplot(gs[0,1]); b.plot(t,p[:,2],color=BLUE); b.plot(t,pd[:,2],color=GRAY,ls='--')
b.set(ylabel='Height (m)',ylim=(.7,2.3)); b.set_title('b  Vertical tracking',loc='left',fontweight='bold')
b.text(.03,.94,'Dashed: 2 m reference',transform=b.transAxes,va='top',fontsize=7)
c=fig.add_subplot(gs[1,1]); c.plot(t,err,color=BLUE); c.set(ylabel='3D error (m)')
c.set_title('c  Position error',loc='left',fontweight='bold')
c.text(.98,.95,'Steady RMS: 0.306 m',ha='right',va='top',transform=c.transAxes,fontsize=7)
for ax in (b,c):
    ax.axvspan(0,8*np.pi,color=ORANGE,alpha=.1); ax.axvline(8*np.pi,color=GRAY,ls=':',lw=.8)
    ax.set(xlim=(0,t[-1]),xlabel='Trajectory time (s)'); ax.grid(alpha=.15)
f=fig.add_subplot(gs[2,:]); f.plot(t[s],(phase-phase[0])/(2*np.pi),color=BLUE)
f.set(xlabel='Trajectory time (s)',ylabel='Observed turns',xlim=(8*np.pi,t[-1]),ylim=(0,2.6))
f.axhline(1,color=GRAY,ls=':',lw=.8); f.set_title('d  Actual path completes 2.376 turns in the steady window',loc='left',fontweight='bold')
f.grid(alpha=.15); save(fig,'official_tracking')

# Exercise numbering, checked against the official top-view diagram.
Fa=np.array([[1,1,1,1],[1,-1,-1,1],[1,1,-1,-1],[-1,1,-1,1]])
Fb=Fa.copy(); Fb[3]=[-1,1,1,-1]
assert np.linalg.matrix_rank(Fa)==4 and np.linalg.matrix_rank(Fb)==3
assert np.all(Fb[3]==-Fb[1]) and np.all(Fb@np.array([1,-1,1,-1])==0)
fig,axs=plt.subplots(1,2,figsize=(160/25.4,70/25.4),layout='constrained')
positions=[(-1,1),(-1,-1),(1,-1),(1,1)]
for ax,signs,label in zip(axs,[[-1,1,-1,1],[-1,1,1,-1]],['a  Original layout: rank 4','b  Reversed rotors 3 and 4: rank 3']):
    for i,((x,y),sign) in enumerate(zip(positions,signs),1):
        ax.plot([0,x],[0,y],color=GRAY,lw=2,zorder=0)
        col=BLUE if sign<0 else ORANGE
        ax.add_patch(Circle((x,y),.34,facecolor=col,edgecolor='white'))
        ax.text(x,y,str(i),ha='center',va='center',color='white',fontsize=11)
        ax.text(x,y-.55,('CCW rotor' if sign<0 else 'CW rotor')+'\n'+('$M_z<0$' if sign<0 else '$M_z>0$'),ha='center',va='center',fontsize=7)
    ax.annotate('',xy=(.6,0),xytext=(0,0),arrowprops={'arrowstyle':'->','lw':.8})
    ax.annotate('',xy=(0,.6),xytext=(0,0),arrowprops={'arrowstyle':'->','lw':.8})
    ax.text(.66,0,'x',va='center'); ax.text(0,.66,'y',ha='center')
    ax.set(xlim=(-1.65,1.65),ylim=(-1.85,1.65),aspect='equal')
    ax.axis('off'); ax.set_title(label,loc='left',fontweight='bold')
save(fig,'rotor_layout')

fig,ax=plt.subplots(figsize=(160/25.4,63/25.4)); ax.set(xlim=(0,10),ylim=(0,4)); ax.axis('off')
boxes=[(.1,2.4,2.15,.85,'Trajectory publisher\nPosition, velocity, acceleration'),
       (3.1,2.4,2.25,.85,'Geometric controller\nForce, attitude and allocation'),
       (6.3,2.4,3.55,.85,'Control interface + TESSE\nMotor commands and simulated motion'),
       (3.2,.15,2.25,.85,'TESSE ROS bridge\nOdometry feedback')]
for x,y,w,h,txt in boxes:
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.05,rounding_size=0.06',facecolor='#EDF3F7',edgecolor=BLUE,lw=.8))
    ax.text(x+w/2,y+h/2,txt,ha='center',va='center',fontsize=7)
for start,end in [((2.3,2.82),(3.02,2.82)),((5.4,2.82),(6.22,2.82)),((8.05,2.35),(5.5,.65)),((4.3,1.04),(4.3,2.34))]:
    ax.add_patch(FancyArrowPatch(start,end,arrowstyle='-|>',mutation_scale=10,lw=.9,color=GRAY))
ax.text(2.62,3.55,'/desired_state',ha='center',fontsize=7)
ax.text(5.8,3.55,'/rotor_speed_cmds',ha='center',fontsize=7)
ax.text(4.45,1.65,'/current_state\n'+r'$\Omega = R^T\Omega_{world}$',va='center',fontsize=7)
ax.text(7.8,1.0,'Simulator state\nvia UDP',ha='center',fontsize=7)
fig.tight_layout(pad=.2); save(fig,'control_loop')

log=(DATA/'numerical_validation.txt').read_text()
blocks=re.findall(r'mode=(.*?)\nsteady_rms_position_error_m=([\deE.+-]+)',log)
cases={cfg:float(value) for cfg,value in blocks}
vals=[cases['circle dt=0.001 drag=1 compensation=0'],cases['circle dt=0.001 drag=1 compensation=1']]
assert np.all(np.isfinite(vals)) and vals[0]>0 and vals[1]>0
fig,ax=plt.subplots(figsize=(135/25.4,54/25.4),layout='constrained')
ax.barh([1,0],vals,height=.46,color=[GRAY,BLUE])
ax.set(yticks=[1,0],yticklabels=['Drag, no compensation','Drag, compensation enabled'],xlabel='Steady 3D position RMS error (m)',xlim=(0,.105),ylim=(-.55,1.65))
for y,v in zip([1,0],vals): ax.text(v+.002,y,f'{v:.5f} m',va='center',fontsize=8)
ax.set_title('Independent rigid-body model; one run per condition',loc='left',fontsize=9)
ax.spines['left'].set_visible(False); ax.tick_params(axis='y',length=0); ax.grid(axis='x',alpha=.15); ax.set_axisbelow(True)
save(fig,'drag_compensation')
print(json.dumps(m,indent=2))

# Spatial geometry complements the existing plan view; retain every sample.
fig = plt.figure(figsize=(160/25.4,84/25.4),layout='constrained')
gs=fig.add_gridspec(1,2,width_ratios=[1.35,1],wspace=.3)
a=fig.add_subplot(gs[0,0],projection='3d')
a.plot(pd[:,0],pd[:,1],pd[:,2],color=GRAY,ls='--',label='Reference')
a.plot(p[~s,0],p[~s,1],p[~s,2],color=ORANGE,label='Initial two periods')
a.plot(p[s,0],p[s,1],p[s,2],color=BLUE,label='Steady window')
a.set(xlabel='x (m)',ylabel='y (m)',zlabel='',zlim=(0,2.3))
a.set_box_aspect((1,1,.55)); a.view_init(elev=24,azim=-58)
a.set_title('a  Spatial trajectory',loc='left',fontweight='bold')
a.tick_params(labelsize=7,pad=0)
a.text2D(.95,.72,'z (m)',transform=a.transAxes,rotation=90,fontsize=7)
b=fig.add_subplot(gs[0,1]); b.plot(pd[:,0],pd[:,2],color=GRAY,ls='--')
b.plot(p[~s,0],p[~s,2],color=ORANGE); b.plot(p[s,0],p[s,2],color=BLUE)
b.set(xlabel='x (m)',ylabel='z (m)',ylim=(0,2.3)); b.grid(alpha=.15)
b.set_title('b  Side projection',loc='left',fontweight='bold')
b.text(.05,.97,'Dashed: 2 m target',transform=b.transAxes,va='top',fontsize=7)
fig.legend(*a.get_legend_handles_labels(),loc='outside lower center',ncol=3,fontsize=7)
save(fig,'spatial_tracking')

colors=[BLUE,ORANGE,'#685687']
fig,axs=plt.subplots(2,2,figsize=(160/25.4,105/25.4),layout='constrained')
for j,ax in enumerate(axs.flat[:3]):
    ax.plot(t[s],e[s,j],color=colors[j]); ax.axhline(0,color=GRAY,ls=':',lw=.8)
    ax.set(xlabel='Trajectory time (s)',ylabel=f'{"xyz"[j]} error (m)',xlim=(t[s][0],t[s][-1]),ylim=(min(-.5,float(e[s].min())-.03),max(.3,float(e[s].max())+.03)))
    ax.set_title(f'{"abc"[j]}  {"xyz"[j]} component',loc='left',fontweight='bold'); ax.grid(alpha=.15)
ax=axs[1,1]; values=np.array(m['component_rms_m']); ax.bar(['x','y','z'],values,color=colors,width=.55)
for j,value in enumerate(values): ax.text(j,value+.008,f'{value:.3f}',ha='center',fontsize=8)
ax.set(ylabel='Component RMS error (m)',ylim=(0,.33)); ax.set_title('d  Steady-window RMS',loc='left',fontweight='bold')
ax.text(.5,.95,'z: 72.1% of squared 3D error',transform=ax.transAxes,ha='center',va='top',fontsize=7)
assert np.isclose(np.sum(values**2),m['steady_position_rms_m']**2)
save(fig,'component_errors')

fig,axs=plt.subplots(1,2,figsize=(160/25.4,62/25.4),layout='constrained',gridspec_kw={'width_ratios':[1.5,1]})
trial_counts={}; trial_rms=[]
for name,label,color in [('unity_circle_initial','Selected gains',BLUE),('unity_circle_tuned','Higher gains',ORANGE)]:
    trial=np.genfromtxt(DATA/'trials'/f'{name}.csv',delimiter=',',names=True)
    tt=trial['trajectory_time']; ee=np.linalg.norm(np.column_stack([trial[k]-trial[k+'d'] for k in ('x','y','z')]),axis=1)
    assert np.isfinite(ee).all() and np.all(np.diff(tt)>=0)
    mask=tt>=20; rms=float(np.sqrt(np.mean(ee[mask]**2)))
    assert np.isclose(rms,json.loads((DATA/'trials'/f'{name}.json').read_text())['steady_position_rms_m'])
    trial_rms.append(rms); trial_counts[name]={'total':len(tt),'selected':int(mask.sum()),'excluded':int((~mask).sum())}
    axs[0].plot(tt,ee,color=color,label=label)
axs[0].axvline(20,color=GRAY,ls=':',lw=.8); axs[0].axvspan(0,20,color=GRAY,alpha=.07)
axs[0].set(xlabel='Trajectory time (s)',ylabel='3D error (m)',xlim=(0,40.2)); axs[0].legend(fontsize=7)
axs[0].set_title('a  Complete trial records',loc='left',fontweight='bold'); axs[0].grid(alpha=.15)
axs[1].bar([0,1],trial_rms,color=[BLUE,ORANGE],width=.55)
for j,value in enumerate(trial_rms): axs[1].text(j,value+.02,f'{value:.3f} m',ha='center',fontsize=8)
axs[1].set(xticks=[0,1],xticklabels=['Selected\ngains','Higher\ngains'],ylabel='Position RMS error (m)',ylim=(0,1.13))
axs[1].set_title('b  Reference time ≥ 20 s',loc='left',fontweight='bold')
save(fig,'gain_comparison')

rotors=np.column_stack([d[f'rotor{i}'] for i in range(1,5)])
assert np.isfinite(rotors).all()
fig,axs=plt.subplots(2,1,figsize=(160/25.4,85/25.4),layout='constrained')
for j,color in enumerate([BLUE,ORANGE,'#685687','#397C64']):
    axs[0].plot(t,rotors[:,j],color=color,lw=.7,label=f'Rotor {j+1}')
    axs[1].plot(t[s],rotors[s,j],color=color,lw=.7)
axs[0].axvline(8*np.pi,color=GRAY,ls=':',lw=.8)
axs[0].set_title('a  Complete command record',loc='left',fontweight='bold'); axs[0].legend(ncol=4,fontsize=7,loc='upper right')
axs[1].set_title('b  Steady-window commands',loc='left',fontweight='bold')
for ax in axs:
    ax.set(xlabel='Trajectory time (s)',ylabel='Command (rad/s)'); ax.grid(alpha=.15)
axs[0].set_xlim(t[0],t[-1]); axs[1].set_xlim(t[s][0],t[s][-1])
save(fig,'rotor_commands')
(HERE/'qa/expanded_figure_data.json').write_text(json.dumps({'gain_trials':trial_counts,'official_full_rows':len(t),'official_steady_rows':int(s.sum()),'rotor_command_range_rad_s':[float(rotors.min()),float(rotors.max())]},indent=2)+'\n')
