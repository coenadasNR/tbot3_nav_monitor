#!/usr/bin/env python3
"""
Adaptive vs Baseline performance comparison.

Usage:
    python3 scripts/analyse.py [--data-dir data/csv] [--out-dir data/plots]
"""
import argparse
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd


# ── colour palette ────────────────────────────────────────────────────────────
COLOURS = {'adaptive': '#2196F3', 'baseline': '#FF5722'}
ALPHA_LINE = 0.85
ALPHA_FILL = 0.15
FIG_DPI = 150


# ── helpers ───────────────────────────────────────────────────────────────────

def load_data(data_dir: Path) -> pd.DataFrame:
    files = sorted(data_dir.glob('nav_metrics_*.csv'))
    if not files:
        sys.exit(f'No nav_metrics_*.csv files found in {data_dir}')

    frames = []
    for f in files:
        df = pd.read_csv(f)
        if 'config' not in df.columns:
            print(f'  skip {f.name} — no config column')
            continue
        frames.append(df)

    if not frames:
        sys.exit('No CSV files with a config column found.')

    df = pd.concat(frames, ignore_index=True)
    df.sort_values('timestamp', inplace=True)
    df.reset_index(drop=True, inplace=True)
    return df


def completed_goal_rows(df: pd.DataFrame) -> pd.DataFrame:
    """Return only rows that correspond to a newly completed goal."""
    out = []
    for cfg, gdf in df.groupby('config'):
        gdf = gdf.copy().reset_index(drop=True)
        gdf['_delta'] = gdf['goals_completed'].diff().fillna(0)
        out.append(gdf[gdf['_delta'] > 0])
    return pd.concat(out, ignore_index=True) if out else pd.DataFrame()


def save(fig: plt.Figure, path: Path) -> None:
    fig.savefig(path, dpi=FIG_DPI, bbox_inches='tight')
    plt.close(fig)
    print(f'  saved {path}')


# ── plots ─────────────────────────────────────────────────────────────────────

def _world_label(df: pd.DataFrame) -> str:
    worlds = df['world'].unique()
    return ' + '.join(sorted(str(w).title() for w in worlds))


def plot_summary_bar(df: pd.DataFrame, out_dir: Path) -> None:
    """Bar chart comparing key aggregate metrics per config."""
    world = _world_label(df)
    configs = sorted(df['config'].unique(), key=lambda c: (c != 'adaptive'))
    metrics = {
        'Avg Recovery Count\n(per sample)': lambda d: d['recovery_count'].mean(),
        'Avg Path Efficiency': lambda d: d.loc[d['path_efficiency'] > 0, 'path_efficiency'].mean(),
        'Goal Success Rate': lambda d: (
            d['goals_completed'].max() /
            max(d['goals_completed'].max() + d['goals_failed'].max(), 1)
        ) if 'goals_failed' in d.columns else float('nan'),
        'Avg Execution Time (s)\n(active goals)': lambda d: d.loc[d['execution_time_s'] > 0, 'execution_time_s'].mean(),
    }

    n_metrics = len(metrics)
    fig, axes = plt.subplots(1, n_metrics, figsize=(4 * n_metrics, 4.5))
    fig.suptitle(f'Adaptive vs Baseline — Aggregate Comparison ({world} World)', fontsize=13, fontweight='bold', y=1.02)

    x = np.arange(len(configs))
    for ax, (label, fn) in zip(axes, metrics.items()):
        values = []
        colours = []
        for cfg in configs:
            v = fn(df[df['config'] == cfg])
            values.append(v if not np.isnan(v) else 0)
            colours.append(COLOURS.get(cfg, '#888'))

        bars = ax.bar(x, values, color=colours, width=0.5, edgecolor='white', linewidth=0.8)
        for bar, val in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + max(values) * 0.02,
                    f'{val:.2f}', ha='center', va='bottom', fontsize=9)

        ax.set_xticks(x)
        ax.set_xticklabels([c.title() for c in configs], fontsize=10)
        ax.set_title(label, fontsize=9)
        ax.set_ylim(0, max(values) * 1.25 + 1e-6)
        ax.spines[['top', 'right']].set_visible(False)

    handles = [mpatches.Patch(color=COLOURS.get(c, '#888'), label=c.title()) for c in configs]
    fig.legend(handles=handles, loc='lower center', ncol=len(configs),
               bbox_to_anchor=(0.5, -0.04), fontsize=10)
    save(fig, out_dir / 'summary_bar.png')


def plot_recovery_over_time(df: pd.DataFrame, out_dir: Path) -> None:
    """Recovery count over simulation time, one line per config."""
    world = _world_label(df)
    fig, ax = plt.subplots(figsize=(9, 4))
    for cfg, gdf in df.groupby('config'):
        t = gdf['timestamp'] - gdf['timestamp'].min()
        colour = COLOURS.get(cfg, '#888')
        ax.plot(t, gdf['recovery_count'], label=cfg.title(), color=colour, alpha=ALPHA_LINE, linewidth=1.4)

    ax.set_xlabel('Elapsed Time (s)')
    ax.set_ylabel('Cumulative Recovery Count')
    ax.set_title(f'Recovery Count Over Time — {world} World', fontsize=12, fontweight='bold')
    ax.legend()
    ax.spines[['top', 'right']].set_visible(False)
    save(fig, out_dir / 'recovery_over_time.png')


def plot_efficiency_over_time(df: pd.DataFrame, out_dir: Path) -> None:
    """Path efficiency of completed goals over time."""
    world = _world_label(df)
    goal_rows = completed_goal_rows(df)
    if goal_rows.empty:
        print('  skip efficiency_over_time — no completed goals found')
        return

    fig, ax = plt.subplots(figsize=(9, 4))
    for cfg, gdf in goal_rows.groupby('config'):
        colour = COLOURS.get(cfg, '#888')
        t = gdf['timestamp'] - df[df['config'] == cfg]['timestamp'].min()
        ax.plot(t, gdf['path_efficiency'], 'o-', label=cfg.title(),
                color=colour, alpha=ALPHA_LINE, linewidth=1.4, markersize=5)

    ax.axhline(1.0, color='grey', linestyle='--', linewidth=0.8, label='Ideal (1.0)')
    ax.set_xlabel('Elapsed Time (s)')
    ax.set_ylabel('Path Efficiency (actual/straight-line)')
    ax.set_title(f'Path Efficiency per Completed Goal — {world} World', fontsize=12, fontweight='bold')
    ax.legend()
    ax.spines[['top', 'right']].set_visible(False)
    save(fig, out_dir / 'efficiency_over_time.png')


def plot_execution_time_dist(df: pd.DataFrame, out_dir: Path) -> None:
    """Box plot of per-goal execution time distribution."""
    world = _world_label(df)
    goal_rows = completed_goal_rows(df)
    if goal_rows.empty:
        print('  skip execution_time_dist — no completed goals found')
        return

    configs = sorted(goal_rows['config'].unique(), key=lambda c: (c != 'adaptive'))
    data = [goal_rows.loc[goal_rows['config'] == c, 'execution_time_s'].values for c in configs]

    fig, ax = plt.subplots(figsize=(6, 5))
    bp = ax.boxplot(data, tick_labels=[c.title() for c in configs], patch_artist=True, notch=False,
                    medianprops=dict(color='white', linewidth=2))
    for patch, cfg in zip(bp['boxes'], configs):
        patch.set_facecolor(COLOURS.get(cfg, '#888'))
        patch.set_alpha(0.75)

    ax.set_ylabel('Execution Time (s)')
    ax.set_title(f'Goal Execution Time Distribution — {world} World', fontsize=12, fontweight='bold')
    ax.spines[['top', 'right']].set_visible(False)
    save(fig, out_dir / 'execution_time_dist.png')


def plot_battery_over_time(df: pd.DataFrame, out_dir: Path) -> None:
    """Battery drain over time."""
    world = _world_label(df)
    fig, ax = plt.subplots(figsize=(9, 4))
    for cfg, gdf in df.groupby('config'):
        t = gdf['timestamp'] - gdf['timestamp'].min()
        colour = COLOURS.get(cfg, '#888')
        ax.plot(t, gdf['battery_pct'], label=cfg.title(), color=colour, alpha=ALPHA_LINE, linewidth=1.4)

    ax.set_xlabel('Elapsed Time (s)')
    ax.set_ylabel('Battery (%)')
    ax.set_title(f'Battery Level Over Time — {world} World', fontsize=12, fontweight='bold')
    ax.legend()
    ax.spines[['top', 'right']].set_visible(False)
    save(fig, out_dir / 'battery_over_time.png')


def plot_goal_outcomes(df: pd.DataFrame, out_dir: Path) -> None:
    """Stacked bar of goal outcomes (completed / failed / canceled) per config."""
    world = _world_label(df)
    if 'goals_failed' not in df.columns:
        print('  skip goal_outcomes — goals_failed column missing')
        return

    configs = sorted(df['config'].unique(), key=lambda c: (c != 'adaptive'))
    completed = [df[df['config'] == c]['goals_completed'].max() for c in configs]
    failed    = [df[df['config'] == c]['goals_failed'].max() for c in configs]
    canceled  = [df[df['config'] == c]['goals_canceled'].max() if 'goals_canceled' in df.columns else 0
                 for c in configs]

    x = np.arange(len(configs))
    width = 0.5

    fig, ax = plt.subplots(figsize=(6, 5))
    b1 = ax.bar(x, completed, width, label='Succeeded', color='#4CAF50', edgecolor='white')
    b2 = ax.bar(x, failed,    width, label='Failed',    color='#F44336', edgecolor='white', bottom=completed)
    b3 = ax.bar(x, canceled,  width, label='Canceled',  color='#FF9800', edgecolor='white',
                bottom=[c + f for c, f in zip(completed, failed)])

    for bar_group in (b1, b2, b3):
        for bar in bar_group:
            h = bar.get_height()
            if h > 0:
                ax.text(bar.get_x() + bar.get_width() / 2,
                        bar.get_y() + h / 2, str(int(h)),
                        ha='center', va='center', fontsize=9, color='white', fontweight='bold')

    ax.set_xticks(x)
    ax.set_xticklabels([c.title() for c in configs], fontsize=11)
    ax.set_ylabel('Number of Goals')
    ax.set_title(f'Goal Outcomes — {world} World', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right')
    ax.spines[['top', 'right']].set_visible(False)
    save(fig, out_dir / 'goal_outcomes.png')


def print_summary_table(df: pd.DataFrame) -> None:
    configs = sorted(df['config'].unique(), key=lambda c: (c != 'adaptive'))
    print('\n' + '=' * 62)
    print(f'{"Metric":<32} {"Adaptive":>13} {"Baseline":>13}')
    print('=' * 62)

    def row(label, fn, fmt='.2f'):
        vals = []
        for cfg in configs:
            try:
                v = fn(df[df['config'] == cfg])
                vals.append(f'{v:{fmt}}')
            except Exception:
                vals.append('  n/a')
        pad = [''] * (2 - len(vals))
        all_vals = vals + pad
        print(f'{label:<32} {all_vals[0]:>13} {all_vals[1]:>13}')

    active = lambda d: d[d['execution_time_s'] > 0]
    eff    = lambda d: completed_goal_rows(d)['path_efficiency']

    row('Duration (s)', lambda d: d['timestamp'].max() - d['timestamp'].min())
    row('Total samples', lambda d: len(d), fmt='d')
    row('Goals succeeded', lambda d: int(d['goals_completed'].max()), fmt='d')
    row('Goals failed',    lambda d: int(d['goals_failed'].max()) if 'goals_failed' in d.columns else 0, fmt='d')
    row('Goals canceled',  lambda d: int(d['goals_canceled'].max()) if 'goals_canceled' in d.columns else 0, fmt='d')
    row('Success rate (%)',
        lambda d: 100 * d['goals_completed'].max() /
                  max(d['goals_completed'].max() + (d['goals_failed'].max() if 'goals_failed' in d.columns else 0), 1))
    row('Avg recovery count', lambda d: d['recovery_count'].mean())
    row('Peak recovery count', lambda d: d['recovery_count'].max())
    row('Avg path efficiency', lambda d: eff(d).mean() if len(eff(d)) > 0 else float('nan'))
    row('Avg exec time (s)', lambda d: active(d)['execution_time_s'].mean() if len(active(d)) > 0 else float('nan'))
    row('Battery remaining (%)', lambda d: d['battery_pct'].min())
    row('Total distance (m)', lambda d: d['total_distance_m'].max())
    print('=' * 62)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='Analyse nav_metrics CSVs')
    ap.add_argument('--data-dir', default='data/csv',  help='Directory containing CSV files')
    ap.add_argument('--out-dir',  default='data/plots', help='Output directory for plots')
    ap.add_argument('--world',    default=None,         help='Filter to a specific world (e.g. house, narrow)')
    args = ap.parse_args()

    data_dir = Path(args.data_dir)
    out_dir  = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f'Loading CSVs from {data_dir} ...')
    df = load_data(data_dir)

    if args.world:
        df = df[df['world'] == args.world].reset_index(drop=True)
        if df.empty:
            sys.exit(f'No rows found for world={args.world}')

    configs = df['config'].unique()
    worlds  = df['world'].unique()
    print(f'  {len(df)} rows | configs: {list(configs)} | worlds: {list(worlds)}')

    print_summary_table(df)

    print('\nGenerating plots ...')
    plot_summary_bar(df, out_dir)
    plot_recovery_over_time(df, out_dir)
    plot_efficiency_over_time(df, out_dir)
    plot_execution_time_dist(df, out_dir)
    plot_battery_over_time(df, out_dir)
    plot_goal_outcomes(df, out_dir)

    print(f'\nDone. Plots saved to {out_dir}/')


if __name__ == '__main__':
    main()
