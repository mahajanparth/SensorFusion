"""Evaluate EKF runs against GPS ground truth on the Linkou bag.

Run with the anaconda python (needs rosbags, numpy, matplotlib):
    python eval/evaluate.py

Inputs:  the Linkou bag (ground truth + baselines, cached to results/inputs.npz)
         results/run_<A|B|C>/  (recorded /ekf/odom from run_variants.sh)
Outputs: results/metrics.csv, results/metrics.md, results/*.png
"""
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from _bagio import BAG, Reader, typestore  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, 'results')
SEGMENTS = [(0, 60), (60, 120), (120, 180), (180, 240)]  # seconds from bag start
RPE_DIST = 10.0  # metres
GOOD_GPS_VAR = 2.0  # m^2; /fix samples with larger horizontal variance are flagged as poor

# Fixed categorical order (reference palette slots 1-4); GT and reference stay neutral.
STYLE = {
    'wheel_odom': dict(color='#2a78d6', label='Wheel odom (/wheel_odom)'),
    'A': dict(color='#eb6834', label='EKF-A original model (overlaps wheel odom)'),
    'B': dict(color='#1baf7a', label='EKF-B fixed model + odom pose'),
    'C': dict(color='#eda100', label='EKF-C fixed model, odom velocity only'),
}
INK, MUTED, GRID = '#0b0b0b', '#8a8984', '#e6e5e0'


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read_odom_bag(path, topics):
    ts = typestore()
    out = {t: [] for t in topics}
    with Reader(path) as r:
        conns = [c for c in r.connections if c.topic in topics]
        for c, _, raw in r.messages(connections=conns):
            m = ts.deserialize_cdr(raw, c.msgtype)
            if c.topic == '/fix':
                out[c.topic].append((stamp(m), m.latitude, m.longitude, m.position_covariance[0]))
            else:
                p = m.pose.pose
                cov = np.asarray(m.pose.covariance).reshape(6, 6)
                out[c.topic].append((stamp(m), p.position.x, p.position.y, yaw_of(p.orientation),
                                     cov[0, 0], cov[1, 1], cov[5, 5]))
    return {k: np.array(v) for k, v in out.items()}


def load_inputs():
    cache = os.path.join(RES, 'inputs.npz')
    if os.path.exists(cache):
        return dict(np.load(cache))
    d = read_odom_bag(BAG, ['/fix', '/odom', '/wheel_odom'])
    fix = d['/fix']
    lat0, lon0 = fix[0, 1], fix[0, 2]
    # Local tangent plane (equirectangular) - error < 1 mm over this ~200 m drive.
    east = (fix[:, 2] - lon0) * np.cos(np.radians(lat0)) * 111320.0
    north = (fix[:, 1] - lat0) * 110574.0
    out = dict(gps=np.c_[fix[:, 0], east, north, fix[:, 3]], ref=d['/odom'], wheel_odom=d['/wheel_odom'])
    np.savez(cache, **out)
    return out


def umeyama_se2(src, dst):
    """Rotation R and translation t minimizing |R src + t - dst| (no scale)."""
    ms, md = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((dst - md).T @ (src - ms))
    D = np.diag([1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    return R, md - R @ ms


def interp_traj(traj, t):
    """Interpolate x, y and (unwrapped) yaw of traj at times t."""
    yaw = np.unwrap(traj[:, 3])
    return np.c_[np.interp(t, traj[:, 0], traj[:, 1]), np.interp(t, traj[:, 0], traj[:, 2]),
                 np.interp(t, traj[:, 0], yaw)]


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def evaluate_window(est, gps, ref, t0, t1):
    """ATE vs GPS, ATE vs reference, heading RMSE and RPE inside [t0, t1)."""
    lo, hi = max(t0, est[0, 0]), min(t1, est[-1, 0])
    g = gps[(gps[:, 0] >= lo) & (gps[:, 0] < hi)]
    r = ref[(ref[:, 0] >= lo) & (ref[:, 0] < hi)]

    e_g = interp_traj(est, g[:, 0])
    R, t = umeyama_se2(e_g[:, :2], g[:, 1:3])
    err_g = np.linalg.norm(e_g[:, :2] @ R.T + t - g[:, 1:3], axis=1)
    good = g[:, 3] < GOOD_GPS_VAR

    e_r = interp_traj(est, r[:, 0])
    Rr, tr = umeyama_se2(e_r[:, :2], r[:, 1:3])
    err_r = np.linalg.norm(e_r[:, :2] @ Rr.T + tr - r[:, 1:3], axis=1)
    # Heading error after removing the constant frame offset (circular mean of the difference).
    diff = e_r[:, 2] - np.unwrap(r[:, 3])
    offset = np.arctan2(np.sin(diff).mean(), np.cos(diff).mean())
    dyaw = wrap(diff - offset)

    # RPE: relative SE(2) error over RPE_DIST of reference path length.
    path = np.r_[0, np.cumsum(np.linalg.norm(np.diff(r[:, 1:3], axis=0), axis=1))]
    rpe = []
    for i in range(len(r)):
        j = np.searchsorted(path, path[i] + RPE_DIST)
        if j >= len(r):
            break
        def rel(p):
            c, s = np.cos(p[i, 2]), np.sin(p[i, 2])
            d = p[j, :2] - p[i, :2]
            return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])
        rpe.append(np.linalg.norm(rel(e_r) - rel(np.c_[r[:, 1:3], np.unwrap(r[:, 3])])))
    rpe = np.array(rpe)

    rms = lambda x: float(np.sqrt(np.mean(np.square(x)))) if len(x) else float('nan')
    return dict(
        n_gps=len(g),
        ate_gps=rms(err_g), ate_gps_good=rms(err_g[good]), ate_gps_max=float(err_g.max()),
        ate_ref=rms(err_r), yaw_rmse_deg=float(np.degrees(rms(dyaw))),
        rpe10=rms(rpe),
        _R=R, _t=t, _Rr=Rr, _tr=tr, _err_g=err_g, _tg=g[:, 0], _err_r=err_r, _tr_t=r[:, 0], _dyaw=dyaw,
    )


def setup_ax(ax, title):
    ax.set_title(title, loc='left', fontsize=11, color=INK)
    ax.grid(True, color=GRID, linewidth=0.8)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors='#52514e', labelsize=9)


def plot_xy(ax, trajs, gps, ref, res, t0, t1, title):
    setup_ax(ax, title)
    g = gps[(gps[:, 0] >= t0) & (gps[:, 0] < t1)]
    good = g[:, 3] < GOOD_GPS_VAR
    rr = ref[(ref[:, 0] >= t0) & (ref[:, 0] < t1)]
    rg = interp_traj(rr, g[:, 0])
    Rref, tref = umeyama_se2(rg[:, :2], g[:, 1:3])
    rxy = rr[:, 1:3] @ Rref.T + tref
    ax.scatter(g[good, 1], g[good, 2], s=10, color=INK, zorder=2, label='GPS /fix (ground truth)', linewidths=0)
    ax.scatter(g[~good, 1], g[~good, 2], s=10, facecolors='none', edgecolors=MUTED, zorder=2,
               linewidths=0.8, label=f'GPS, var > {GOOD_GPS_VAR:g} m²')
    ax.plot(rxy[:, 0], rxy[:, 1], color=MUTED, linestyle='--', linewidth=1.5, zorder=3, label='/odom reference')
    for k, tr in trajs.items():
        sel = tr[(tr[:, 0] >= t0) & (tr[:, 0] < t1)]
        R, t = res[k]['_R'], res[k]['_t']
        xy = sel[:, 1:3] @ R.T + t
        ax.plot(xy[:, 0], xy[:, 1], linewidth=2, color=STYLE[k]['color'], zorder=4,
                label=f"{STYLE[k]['label']}  ATE {res[k]['ate_gps']:.2f} m")
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')


def main():
    os.makedirs(RES, exist_ok=True)
    inp = load_inputs()
    gps, ref = inp['gps'], inp['ref']
    trajs = {'wheel_odom': inp['wheel_odom']}
    for v in ('A', 'B', 'C'):
        p = os.path.join(RES, f'run_{v}')
        if os.path.isdir(p):
            trajs[v] = read_odom_bag(p, ['/ekf/odom'])['/ekf/odom']
    tstart = gps[0, 0]

    windows = [('full', 0, 240)] + [(f'seg{i + 1} ({a}-{b}s)', a, b) for i, (a, b) in enumerate(SEGMENTS)]
    results = {}
    rows = []
    for name, a, b in windows:
        results[name] = {}
        for k, tr in trajs.items():
            m = evaluate_window(tr, gps, ref, tstart + a, tstart + b)
            results[name][k] = m
            rows.append(dict(window=name, method=k, **{x: y for x, y in m.items() if not x.startswith('_')}))

    keys = ['window', 'method', 'n_gps', 'ate_gps', 'ate_gps_good', 'ate_gps_max', 'ate_ref', 'yaw_rmse_deg', 'rpe10']
    with open(os.path.join(RES, 'metrics.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{r[k]:.3f}' if isinstance(r[k], float) else r[k]) for k in keys})
    with open(os.path.join(RES, 'metrics.md'), 'w') as f:
        f.write('| window | method | ATE vs GPS [m] | ATE vs GPS, good fixes [m] | ATE vs /odom ref [m] | heading RMSE [deg] | RPE@10 m [m] |\n')
        f.write('|---|---|---:|---:|---:|---:|---:|\n')
        for r in rows:
            f.write(f"| {r['window']} | {r['method']} | {r['ate_gps']:.2f} | {r['ate_gps_good']:.2f} | "
                    f"{r['ate_ref']:.2f} | {r['yaw_rmse_deg']:.1f} | {r['rpe10']:.2f} |\n")
    print(open(os.path.join(RES, 'metrics.md')).read())

    # Full-run trajectory overlay.
    fig, ax = plt.subplots(figsize=(10, 7))
    plot_xy(ax, trajs, gps, ref, results['full'], tstart, tstart + 240,
            'Full run (240 s): trajectories aligned to GPS with SE(2) Umeyama')
    ax.legend(fontsize=8, frameon=False, loc='best')
    fig.tight_layout()
    fig.savefig(os.path.join(RES, 'traj_full.png'), dpi=150)
    plt.close(fig)

    # Per-segment overlays (each segment re-aligned independently).
    fig, axs = plt.subplots(2, 2, figsize=(13, 10))
    for ax, (name, a, b) in zip(axs.ravel(), windows[1:]):
        plot_xy(ax, trajs, gps, ref, results[name], tstart + a, tstart + b, name)
        ax.legend(fontsize=7, frameon=False, loc='best')
    fig.suptitle('60 s segments, each aligned to GPS independently', x=0.01, ha='left', fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(RES, 'traj_segments.png'), dpi=150)
    plt.close(fig)

    # Error over time (full run): position error vs GPS and heading error vs reference.
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    setup_ax(a1, 'Position error vs GPS (full-run alignment)')
    setup_ax(a2, 'Heading error vs /odom reference')
    for k in trajs:
        m = results['full'][k]
        a1.plot(m['_tg'] - tstart, m['_err_g'], color=STYLE[k]['color'], linewidth=2, label=STYLE[k]['label'])
        a2.plot(m['_tr_t'] - tstart, np.degrees(m['_dyaw']), color=STYLE[k]['color'], linewidth=2, label=STYLE[k]['label'])
    a1.set_ylabel('error [m]')
    a2.set_ylabel('error [deg]')
    a2.set_xlabel('time since start [s]')
    a1.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(RES, 'error_vs_time.png'), dpi=150)
    plt.close(fig)

    # Consistency: position error vs the filter's own 3-sigma bound (EKF variants only).
    ekfs = [k for k in ('A', 'B', 'C') if k in trajs]
    fig, axs = plt.subplots(len(ekfs), 1, figsize=(10, 2.6 * len(ekfs)), sharex=True, squeeze=False)
    for ax, k in zip(axs[:, 0], ekfs):
        m = results['full'][k]
        tr = trajs[k]
        sig = np.sqrt(np.interp(m['_tg'], tr[:, 0], tr[:, 4] + tr[:, 5]))
        setup_ax(ax, f"{STYLE[k]['label']}: error vs GPS and filter 3σ")
        ax.fill_between(m['_tg'] - tstart, 0, 3 * sig, color=STYLE[k]['color'], alpha=0.18, linewidth=0, label='3σ (from EKF covariance)')
        ax.plot(m['_tg'] - tstart, m['_err_g'], color=STYLE[k]['color'], linewidth=2, label='position error')
        ax.set_ylabel('[m]')
        ax.legend(fontsize=8, frameon=False, loc='upper left')
    axs[-1, 0].set_xlabel('time since start [s]')
    fig.tight_layout()
    fig.savefig(os.path.join(RES, 'consistency_3sigma.png'), dpi=150)
    plt.close(fig)

    # One figure per EKF variant: trajectory vs GPS (wheel odom for context), position error with
    # the filter's 3-sigma band, and heading error.
    full = results['full']
    for k in ekfs:
        m, tr = full[k], trajs[k]
        fig, (ax_xy, ax_e, ax_h) = plt.subplots(1, 3, figsize=(16, 5), gridspec_kw=dict(width_ratios=[1.3, 1, 1]))
        setup_ax(ax_xy, f"{STYLE[k]['label']}: trajectory")
        g = gps
        good = g[:, 3] < GOOD_GPS_VAR
        ax_xy.scatter(g[good, 1], g[good, 2], s=10, color=INK, zorder=2, linewidths=0, label='GPS /fix (ground truth)')
        ax_xy.scatter(g[~good, 1], g[~good, 2], s=10, facecolors='none', edgecolors=MUTED, zorder=2,
                      linewidths=0.8, label=f'GPS, var > {GOOD_GPS_VAR:g} m²')
        w = trajs['wheel_odom']
        wxy = w[:, 1:3] @ full['wheel_odom']['_R'].T + full['wheel_odom']['_t']
        ax_xy.plot(wxy[:, 0], wxy[:, 1], color=MUTED, linestyle='--', linewidth=1.5, zorder=3,
                   label=f"Wheel odom  ATE {full['wheel_odom']['ate_gps']:.2f} m")
        xy = tr[:, 1:3] @ m['_R'].T + m['_t']
        ax_xy.plot(xy[:, 0], xy[:, 1], color=STYLE[k]['color'], linewidth=2.2, zorder=4,
                   label=f"EKF-{k}  ATE {m['ate_gps']:.2f} m")
        ax_xy.set_aspect('equal', adjustable='datalim')
        ax_xy.set_xlabel('East [m]')
        ax_xy.set_ylabel('North [m]')
        ax_xy.legend(fontsize=8, frameon=False, loc='lower left')

        setup_ax(ax_e, 'Position error vs GPS')
        sig = np.sqrt(np.interp(m['_tg'], tr[:, 0], tr[:, 4] + tr[:, 5]))
        ax_e.fill_between(m['_tg'] - tstart, 0, 3 * sig, color=STYLE[k]['color'], alpha=0.18, linewidth=0,
                          label='filter 3σ')
        ax_e.plot(full['wheel_odom']['_tg'] - tstart, full['wheel_odom']['_err_g'], color=MUTED, linestyle='--',
                  linewidth=1.5, label='wheel odom')
        ax_e.plot(m['_tg'] - tstart, m['_err_g'], color=STYLE[k]['color'], linewidth=2, label=f'EKF-{k}')
        ax_e.set_xlabel('time since start [s]')
        ax_e.set_ylabel('error [m]')
        ax_e.legend(fontsize=8, frameon=False, loc='upper right')

        setup_ax(ax_h, 'Heading error vs /odom reference')
        ax_h.plot(full['wheel_odom']['_tr_t'] - tstart, np.degrees(full['wheel_odom']['_dyaw']), color=MUTED,
                  linestyle='--', linewidth=1.5, label='wheel odom')
        ax_h.plot(m['_tr_t'] - tstart, np.degrees(m['_dyaw']), color=STYLE[k]['color'], linewidth=2,
                  label=f"EKF-{k}  RMSE {m['yaw_rmse_deg']:.1f}°")
        ax_h.set_xlabel('time since start [s]')
        ax_h.set_ylabel('error [deg]')
        ax_h.legend(fontsize=8, frameon=False, loc='upper left')

        fig.tight_layout()
        fig.savefig(os.path.join(RES, f'variant_{k}.png'), dpi=130)
        plt.close(fig)

    # Bar chart of ATE per window.
    fig, ax = plt.subplots(figsize=(10, 4.5))
    setup_ax(ax, 'ATE RMSE vs GPS per window')
    names = [w[0] for w in windows]
    width = 0.8 / len(trajs)
    for i, k in enumerate(trajs):
        vals = [results[n][k]['ate_gps'] for n in names]
        x = np.arange(len(names)) + (i - (len(trajs) - 1) / 2) * width
        ax.bar(x, vals, width * 0.92, color=STYLE[k]['color'], label=STYLE[k]['label'])
    ax.set_xticks(np.arange(len(names)))
    ax.set_xticklabels(names)
    ax.set_ylabel('ATE RMSE [m]')
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(RES, 'ate_bars.png'), dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
