"""Plot HOCBF-QP grasp simulation results from CSV (no pandas/numpy)."""

import sys
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_csv(path):
    """Read CSV into dict of lists."""
    data = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        keys = reader.fieldnames
        for key in keys:
            data[key] = []
        for row in reader:
            for key in keys:
                try:
                    data[key].append(float(row[key]))
                except (ValueError, KeyError):
                    data[key].append(0.0)
    return data


def find_phase_boundaries(phase_list):
    """Return list of (index, new_phase) where phase changes."""
    boundaries = []
    for i in range(1, len(phase_list)):
        if phase_list[i] != phase_list[i - 1]:
            boundaries.append((i, int(phase_list[i])))
    return boundaries


def add_phase_shading(ax, t, phase):
    """Add background color per phase."""
    colors = {0: '#e3f2fd', 1: '#fff3e0', 2: '#e8f5e9', 3: '#fce4ec'}
    bounds = find_phase_boundaries(phase)
    # first segment
    prev_t = t[0]
    prev_ph = int(phase[0])
    for idx, ph in bounds:
        ax.axvspan(prev_t, t[idx], alpha=0.15, color=colors.get(prev_ph, 'white'))
        prev_t = t[idx]
        prev_ph = ph
    ax.axvspan(prev_t, t[-1], alpha=0.15, color=colors.get(prev_ph, 'white'))


def plot_results(csv_path, save_path=None):
    df = read_csv(csv_path)
    t = df['t']
    phase = df['phase']

    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)

    for ax in axes:
        add_phase_shading(ax, t, phase)

    # Panel 1: Position
    ax = axes[0]
    ax.plot(t, df['theta'], 'b-', linewidth=1.0, label='theta (absolute)')
    # estimate theta_contact from data
    bounds = find_phase_boundaries(phase)
    for idx, ph in bounds:
        if ph >= 2 and df['x1'][idx] > 0:
            tc = df['theta'][idx] - df['x1'][idx]
            ax.axhline(y=tc, color='orange', linestyle='--', linewidth=1, label=f'theta_contact={tc:.3f}')
            break
    ax.set_ylabel('angle [rad]')
    ax.set_title('Position')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 2: State + safety set
    ax = axes[1]
    ax.plot(t, df['x1'], 'b-', linewidth=1.5, label='x1 (deformation)')
    ax.plot(t, df['theta_max'], 'r--', linewidth=1.5, label='theta_max')
    tau_K = [df['tau_min'][i] / df['K_obj_hat'][i] if df['K_obj_hat'][i] > 0.01 else 0.0
             for i in range(len(t))]
    ax.plot(t, tau_K, 'g--', linewidth=1.5, label='tau_min / K_hat')
    ax.fill_between(t, tau_K, df['theta_max'], alpha=0.1, color='green', label='safe set')
    ax.set_ylabel('angle [rad]')
    ax.set_title('State & Safety Set')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 3: Control input
    ax = axes[2]
    ax.plot(t, df['u_nom'], 'k-', linewidth=0.8, alpha=0.5, label='u_nom')
    ax.plot(t, df['u_star'], 'b-', linewidth=1.5, label='u* (CBF-QP)')
    ax.plot(t, df['u_lb'], 'g--', linewidth=1.0, label='u_lb (slip)')
    ax.plot(t, df['u_ub'], 'r--', linewidth=1.0, label='u_ub (deform)')
    ax.set_ylabel('torque [Nm]')
    ax.set_title('Control Input')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 4: Estimation
    ax = axes[3]
    ax.plot(t, df['K_obj_true'], 'k-', linewidth=1.5, label='K_obj (true)')
    ax.plot(t, df['K_obj_hat'], 'r--', linewidth=1.5, label='K_obj_hat (RFOB)')
    ax2 = ax.twinx()
    ax2.plot(t, df['f_dis_hat'], 'b-', linewidth=0.8, alpha=0.5, label='f_dis_hat (DOB)')
    ax2.set_ylabel('force [Nm]', color='blue')
    ax.set_ylabel('stiffness [Nm/rad]')
    ax.set_title('Estimation (K_obj & DOB)')
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 5: Safety
    ax = axes[4]
    ax.plot(t, df['h_lb'], 'b-', linewidth=1.5, label='h_lb (slip barrier)')
    ax.plot(t, df['h_ub'], 'r-', linewidth=1.5, label='h_ub (deform barrier)')
    ax.plot(t, df['G'], 'k-', linewidth=1.5, label='G (graspability)')
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
