"""Plot HOCBF-QP grasp simulation results from CSV."""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_results(csv_path, save_path=None):
    df = pd.read_csv(csv_path)

    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)

    # Phase boundaries
    phase_changes = df['phase'].diff().fillna(0).abs() > 0
    for ax in axes:
        for t in df.loc[phase_changes, 't']:
            ax.axvline(x=t, color='gray', linestyle='--', alpha=0.5, linewidth=0.8)

    # Panel 1: State + safety set
    ax = axes[0]
    ax.plot(df['t'], df['x1'], 'b-', linewidth=1.5, label='x1 (theta_rel)')
    ax.plot(df['t'], df['theta_max'], 'r--', linewidth=1.5, label='theta_max')
    tau_min_over_K = np.where(df['K_obj_hat'] > 0.01, df['tau_min'] / df['K_obj_hat'], 0)
    ax.plot(df['t'], tau_min_over_K, 'g--', linewidth=1.5, label='tau_min / K_hat')
    ax.fill_between(df['t'], tau_min_over_K, df['theta_max'], alpha=0.1, color='green')
    ax.set_ylabel('angle [rad]')
    ax.set_title('State & Safety Set')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 2: Control input
    ax = axes[1]
    ax.plot(df['t'], df['u_nom'], 'k-', linewidth=0.8, alpha=0.5, label='u_nom')
    ax.plot(df['t'], df['u_star'], 'b-', linewidth=1.5, label='u* (CBF-QP)')
    ax.plot(df['t'], df['u_lb'], 'g--', linewidth=1.0, label='u_lb (slip)')
    ax.plot(df['t'], df['u_ub'], 'r--', linewidth=1.0, label='u_ub (deform)')
    ax.set_ylabel('torque [Nm]')
    ax.set_title('Control Input')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 3: Estimation
    ax = axes[2]
    ax.plot(df['t'], df['K_obj_true'], 'k-', linewidth=1.5, label='K_obj (true)')
    ax.plot(df['t'], df['K_obj_hat'], 'r--', linewidth=1.5, label='K_obj_hat (RFOB)')
    ax.set_ylabel('stiffness [Nm/rad]')
    ax.set_title('Stiffness Estimation')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 4: Safety
    ax = axes[3]
    ax.plot(df['t'], df['h_lb'], 'b-', linewidth=1.5, label='h_lb (slip barrier)')
    ax.plot(df['t'], df['h_ub'], 'r-', linewidth=1.5, label='h_ub (deform barrier)')
    ax.plot(df['t'], df['G'], 'k-', linewidth=1.5, label='G (graspability)')
    ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
    ax.set_ylabel('value')
    ax.set_xlabel('time [s]')
    ax.set_title('Safety & Graspability')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")
    else:
        plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot.py <csv_path> [--save <output.png>]")
        sys.exit(1)
    csv_path = sys.argv[1]
    save_path = None
    if '--save' in sys.argv:
        idx = sys.argv.index('--save')
        if idx + 1 < len(sys.argv):
            save_path = sys.argv[idx + 1]
    plot_results(csv_path, save_path)
