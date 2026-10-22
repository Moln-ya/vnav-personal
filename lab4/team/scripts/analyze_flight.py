#!/usr/bin/env python3
"""Summarize and visualize an independent ROS rigid-body flight, not TESSE."""
import argparse
import csv
import json
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D  # Register projection on Ubuntu 20.04 matplotlib.


def gates_from_csv(path):
    gates = []
    with open(path) as f:
        for row in csv.reader(f):
            if row and row[0].startswith('red_square_drone_door'):
                index = int(row[0].split('(')[1].rstrip(')')) if '(' in row[0] else 0
                gates.append((index, [float(row[1]), float(row[3]), float(row[2])]))
    return np.array([p for _,p in sorted(gates)])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', action='store_true')
    args = parser.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(root,'report')
    d = np.genfromtxt(os.path.join(out,'rigid_body_flight.csv'), delimiter=',', names=True)
    t = d['t']
    actual = np.column_stack([d[k] for k in ['x','y','z']])
    desired = np.column_stack([d[k] for k in ['xd','yd','zd']])
    gates = gates_from_csv(os.path.join(root,'simulator_data/StreamingAssets/vnav-2020-lab4_static_tfs.csv'))
    error = np.linalg.norm(actual-desired,axis=1)
    active = (d['trajectory_time']>0.1) & (d['trajectory_time']<d['trajectory_time'].max()-0.01)
    passages = []
    previous = -1
    for i,g in enumerate(gates):
        distance = np.linalg.norm(actual-g,axis=1)
        j = int(np.argmin(distance))
        assert j > previous, 'Gate centers must be visited in order'
        previous = j
        passages.append(dict(gate=i,time_s=float(t[j]),min_center_distance_m=float(distance[j])))
    summary = dict(source='Independent 6-DoF rigid-body ROS model, NOT Unity/TESSE',
                   gate_count=len(gates), trajectory_duration_s=float(d['trajectory_time'].max()),
                   rms_tracking_error_m=float(np.sqrt(np.mean(error[active]**2))),
                   max_tracking_error_m=float(error[active].max()),
                   final_position_error_m=float(np.linalg.norm(actual[-1]-gates[-1])),
                   minimum_height_m=float(actual[:,2].min()),
                   gate_center_visits=passages,
                   limitation='Gate center proximity does not certify collision-free Unity gate passage.')
    with open(os.path.join(out,'rigid_body_summary.json'),'w') as f: json.dump(summary,f,indent=2)
    print(json.dumps(summary,indent=2))
    assert max(p['min_center_distance_m'] for p in passages) < 0.6
    assert summary['final_position_error_m'] < 0.1
    assert summary['minimum_height_m'] > 0.5
    fig, ax = plt.subplots(1,2,figsize=(11,4.5))
    valid = d['trajectory_time'] > 0
    ax[0].plot(desired[valid,0],desired[valid,1],'--',label='Desired')
    ax[0].plot(actual[:,0],actual[:,1],label='Actual')
    ax[0].scatter(gates[:,0],gates[:,1],marker='s',s=35,label='Gate centers')
    for i,g in enumerate(gates): ax[0].annotate(str(i),g[:2],xytext=(4,4),textcoords='offset points')
    ax[0].set(xlabel='World x [m]',ylabel='World y [m]',title='All 12 official gate positions')
    ax[0].axis('equal');ax[0].grid(alpha=.25);ax[0].legend()
    ax[1].plot(t[valid],error[valid]);ax[1].set(xlabel='Simulation time [s]',ylabel='Position error [m]',title='Production ROS controller and follower')
    ax[1].grid(alpha=.25)
    fig.suptitle('Independent rigid-body validation (not Unity/TESSE)')
    fig.tight_layout(rect=(0,0,1,.92));fig.savefig(os.path.join(out,'rigid_body_tracking.png'),dpi=160);plt.close(fig)
    if args.video:
        fig = plt.figure(figsize=(10,6))
        ax = fig.add_subplot(111,projection='3d')
        ax.plot(desired[valid,0],desired[valid,1],desired[valid,2],'--',color='0.7',label='Desired')
        ax.scatter(*gates.T,marker='s',s=40,c='#df7045',label='Gate centers')
        for i,g in enumerate(gates):ax.text(*g,str(i))
        trail, = ax.plot([],[],[],color='#1b82bd',lw=2,label='Actual')
        drone, = ax.plot([],[],[],marker='o',color='#1b82bd',ms=7)
        ax.set(xlim=(-50,3),ylim=(-12,18),zlim=(0,5),xlabel='x [m]',ylabel='y [m]',zlabel='z [m]')
        if hasattr(ax, 'set_box_aspect'): ax.set_box_aspect((3,2,0.7))
        ax.view_init(elev=35,azim=-65);ax.legend(loc='upper left')
        label = fig.text(.5,.08,'',ha='center')
        fig.suptitle('Lab4 independent rigid-body ROS validation\nNot Unity/TESSE; playback at 2x',fontsize=13)
        writer = FFMpegWriter(fps=15,bitrate=1400)
        with writer.saving(fig,os.path.join(out,'rigid_body_validation.mp4'),100):
            for current in np.arange(t[0],t[-1],2/15):
                j = min(np.searchsorted(t,current),len(t)-1)
                trail.set_data(actual[:j+1,0],actual[:j+1,1]);trail.set_3d_properties(actual[:j+1,2])
                drone.set_data([actual[j,0]],[actual[j,1]]);drone.set_3d_properties([actual[j,2]])
                label.set_text('t = {:.1f} s   |   position error = {:.3f} m'.format(t[j],error[j]))
                writer.grab_frame()
        plt.close(fig)

if __name__=='__main__':main()
