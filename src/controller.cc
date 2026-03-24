#include <cstdio>
#include <cmath>
#include "controller.h"
#include "control_timer.h"
#include "thread_definition.h"
#include "signal_processing.h"
#include "hocbf_qp.h"
#include "rfob.h"
#include "contact_detector.h"
#include "adaptive_theta_max.h"
#include "object_model.h"

#define x_res(n) robot.joints[(n)].data[mc::response][mc::x]
#define dx_res(n) robot.joints[(n)].data[mc::response][mc::dx]
#define ddx_res(n) robot.joints[(n)].data[mc::response][mc::ddx]
#define f_dis(n) robot.joints[(n)].data[mc::response][mc::f_dis]
#define f_ref(n) robot.joints[(n)].data[mc::reference][mc::f]
#define f_out(n) robot.joints[(n)].data[mc::output][mc::f]
#define f_vol(n) robot.joints[(n)].data[mc::output][mc::f_dis]
#define k_p(n) robot.joints[(n)].parameter[mc::k_p]
#define k_v(n) robot.joints[(n)].parameter[mc::k_v]
#define k_f(n) robot.joints[(n)].parameter[mc::k_f]
#define M(n) robot.joints[(n)].parameter[mc::mass]
#define D(n) robot.joints[(n)].parameter[mc::damper]
#define K(n) robot.joints[(n)].parameter[mc::spring]
#define M_v(n) robot.joints[(n)].parameter[mc::mass_virtual]
#define D_v(n) robot.joints[(n)].parameter[mc::damper_virtual]
#define K_v(n) robot.joints[(n)].parameter[mc::spring_virtual]
#define param(n) robot.joints[(n)].parameter
#define PI 3.141592653589

static double dt = 0.0001;

// Finger joint index (joint 0 = active finger)
static const int FIN = 0;

void mc::control::register_controller()
{
	controller[mc::idle] = [](robot_system &robot)
	{
		for (size_t i = 0; i < robot.joints.size(); ++i)
			f_out(i) = 0.0;
	};

	controller[mc::DA_check] = [](robot_system &robot)
	{
		for (size_t i = 0; i < robot.joints.size(); ++i)
			f_out(i) = 1.0;
	};

	controller[mc::pos_con] = [](robot_system &robot)
	{
		for (size_t i = 0; i < robot.joints.size(); ++i)
		{
			f_ref(i) = (k_p(i) * (0.0 - x_res(i)) + k_v(i) * (0.0 - dx_res(i))) * M(i) + f_dis(i);
			f_out(i) = f_ref(i);
		}
	};

	controller[mc::initialposi_control] = [](robot_system &robot)
	{
		for (size_t i = 0; i < robot.joints.size(); ++i)
		{
			f_ref(i) = (k_p(i) * (0.0 - x_res(i)) + k_v(i) * (0.0 - dx_res(i))) * M(i) + f_dis(i);
			f_out(i) = f_ref(i);
		}
	};

	// ================================================================
	// HOCBF_grasp: HOCBF-QP safe grasping control
	// ================================================================
	controller[mc::HOCBF_grasp] = [](robot_system &robot)
	{
		// --- static state (persists across calls) ---
		static int phase = 0;  // 0:approach, 1:deepen, 2:grasp (no Phase 3)
		static long long step = 0;
		static long long phase2_step = 0;  // Phase 2 内のステップ数
		static double theta_hold = 0.0;
		static double theta_des = 0.0;

		// HOCBF parameters (ADR-005)
		// gamma が大きいほど安全集合への収束が速いが、G ∝ M*gamma^2 なので
		// M が小さい系では gamma を下げないと G < 0 (infeasible) になる
		static double gamma_pos   = 5.0;
		static double gamma_force = 5.0;
		static double tau_min     = 0.025;  // from config or tau_min_estimation

		// Object model (for simulation verification)
		static object_model obj = {3.0, 0.05, 30.0, 0.5, 0.025, 0.35};

		// RFOB
		static rfob rfob_est;
		static double g_rfob = 100.0;
		static double g_dk   = 50.0;   // pseudo-diff cutoff for dK/dt

		// Contact detection (ADR-008)
		static contact_detector cd(1000, 3.0, 0.05);

		// Adaptive theta_max (ADR-005)
		// theta_max_min must be > tau_min / K_0 (= 0.025/3.0 = 0.00833) for G > 0
		// G = M * gamma^2 * (theta_max - tau_min/K) なので theta_max_min で下限を保証
		static adaptive_theta_max adapt(0.01, 0.013, 0.10, 0.1, 0.5, 0.1, 0.5);

		// High-frequency injection (ADR-007)
		// A_inject は M が小さい系では速度振動 ≈ A/(M*2*pi*f) を考慮して設定
		static double A_inject = 0.001;
		static double f_inject = 50.0;

		// CSV recording
		static FILE *csv_fp = nullptr;
		static bool csv_header_written = false;

		// Reset on mode change
		if (robot.is_control_mode_changed())
		{
			phase = 0;
			step = 0;
			phase2_step = 0;
			theta_hold = 0.0;
			rfob_est.reset();
			cd.reset();
			adapt.reset(0.01);
			if (csv_fp) { fclose(csv_fp); csv_fp = nullptr; }
			csv_header_written = false;
			robot.update_control_mode();
		}

		double t = step * dt;
		double Mn = M(FIN);
		double theta = x_res(FIN);
		double theta_dot = dx_res(FIN);
		double f_dis_hat = f_dis(FIN);

		// RFOB update (always, but only meaningful after contact)
		double theta_rel = obj.get_theta_rel(theta);
		rfob_est.update(f_dis_hat, theta_rel, dt, g_rfob, g_dk);
		double K_hat = rfob_est.K_obj_hat;
		double dK_dt = rfob_est.dK_obj_hat_dt;

		// Ground truth (for CSV logging, not used in control)
		double K_true = obj.compute_K_obj(theta);
		double tau_ext_true = obj.compute_tau_ext(theta);

		// Variables for logging
		double u_nom = 0.0, u_star = 0.0, u_lb = 0.0, u_ub = 0.0;
		double h_lb = 0.0, h_ub = 0.0, G = 0.0;
		double x1 = 0.0, x2 = 0.0;

		if (phase == 0)
		{
			// Phase 0: Approach — PD position control toward object
			theta_des = obj.theta_contact + 0.02;  // target slightly past contact
			u_nom = (k_p(FIN) * (theta_des - theta) + k_v(FIN) * (0.0 - theta_dot)) * Mn + f_dis_hat;
			u_star = u_nom;
			f_out(FIN) = u_star;

			// Contact detection
			cd.update(f_dis_hat, K_hat, theta);
			if (cd.detected)
			{
				// theta_holdは物体表面位置を使う。cd.theta_holdは検出時点の位置で
			// 既に変形が進んでおり、HOCBF が反力を過小評価する原因になる。
			// 実機ではDOB信号の立ち上がりから theta_contact を推定する。
			theta_hold = obj.theta_contact;
				phase = 1;  // Phase 1: deepen contact with PD
				tau_min = obj.tau_min;  // from config/estimation
			}
		}
		else if (phase == 1)
		{
			// Phase 1: Continue PD to deepen contact until x1 enters safe set
			// Safe set requires x1 >= tau_min / K_hat for h_lb >= 0
			x1 = theta - theta_hold;
			if (x1 < 0.0) x1 = 0.0;
			x2 = theta_dot;

			double x1_safe = (K_hat > 1e-6) ? (tau_min / K_hat) * 1.2 : 0.01;  // 20% margin
			theta_des = theta_hold + x1_safe;
			u_nom = (k_p(FIN) * (theta_des - theta) + k_v(FIN) * (0.0 - theta_dot)) * Mn + f_dis_hat;
			u_star = u_nom;
			f_out(FIN) = u_star;

			if (x1 >= x1_safe && std::fabs(x2) < 0.1)
			{
				// theta_max は現在の x1 + マージン（h_ub > 0 を保証）
				double theta_max_init = x1 * 1.3 + 0.002;
				adapt.reset(theta_max_init);
				phase2_step = 0;
				phase = 2;  // x1 is inside safe set, activate HOCBF
			}
		}
		else // phase 2 or 3
		{
			x1 = theta - theta_hold;
			if (x1 < 0.0) x1 = 0.0;
			x2 = theta_dot;

			// Adaptive theta_max update (Phase 2 開始直後は凍結)
			static const long long adapt_freeze_steps = 10000;  // 1.0s 凍結
			double u_ub_static = hocbf::compute_u_ub(x1, x2, f_dis_hat, adapt.theta_max, 0.0, Mn, gamma_pos);
			u_lb = hocbf::compute_u_lb(x1, x2, f_dis_hat, K_hat, tau_min, Mn, gamma_force);
			if (phase2_step > adapt_freeze_steps)
				adapt.update(dK_dt, u_ub_static, u_lb, Mn, gamma_pos, dt);
			phase2_step++;

			// HOCBF-QP (DOBベース: K_hat*x1 の代わりに f_dis_hat を使用)
			u_ub = hocbf::compute_u_ub(x1, x2, f_dis_hat, adapt.theta_max, adapt.theta_max_dot, Mn, gamma_pos);
			u_lb = hocbf::compute_u_lb(x1, x2, f_dis_hat, K_hat, tau_min, Mn, gamma_force);

			// Nominal controller: PD toward safe set center
			// x1_target = (tau_min/K_hat + theta_max) / 2  (midpoint of safe set)
			double x1_target = (K_hat > 1e-6)
				? (tau_min / K_hat + adapt.theta_max) * 0.5
				: adapt.theta_max * 0.5;
			theta_des = theta_hold + x1_target;
			u_nom = (k_p(FIN) * (theta_des - theta) + k_v(FIN) * (0.0 - theta_dot)) * Mn + f_dis_hat;

			// Safety filter
			if (u_lb > u_ub)
			{
				// G < 0: infeasible — prioritize deformation prevention
				u_star = u_ub;
			}
			else
			{
				u_star = hocbf::compute_u_star(u_nom, u_lb, u_ub);
			}

			// High-frequency injection for RFOB
			double u_inject = A_inject * std::sin(2.0 * PI * f_inject * t);
			f_out(FIN) = u_star + u_inject;

			// Barrier values
			h_lb = hocbf::compute_h_lb(K_hat, x1, tau_min);
			h_ub = hocbf::compute_h_ub(adapt.theta_max, x1);
			G = hocbf::compute_G(u_ub, u_lb);

			// Phase 2 のまま運用（Phase 3 は廃止）
		}

		// DOBへの力指令入力（f_volはDOBが参照する）
		f_vol(FIN) = f_out(FIN);

		// Other joints: zero output
		for (size_t i = 1; i < robot.joints.size(); ++i)
			f_out(i) = 0.0;

		// --- CSV recording (auto-start on mode entry) ---
		if (!csv_fp)
		{
			csv_fp = fopen("../data/result.csv", "w");
			csv_header_written = false;
		}
		if (csv_fp && !csv_header_written)
		{
			fprintf(csv_fp, "t,phase,theta,x1,x2,u_nom,u_star,u_lb,u_ub,"
			        "tau_ext,f_dis_hat,K_obj_true,K_obj_hat,"
			        "theta_max,tau_min,h_lb,h_ub,G\n");
			csv_header_written = true;
		}
		if (csv_fp && step % 10 == 0)  // 10kHz sampling (every 10 steps)
		{
			fprintf(csv_fp, "%.6f,%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,"
			        "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
			        t, phase, theta, x1, x2, u_nom, u_star, u_lb, u_ub,
			        tau_ext_true, f_dis_hat, K_true, K_hat,
			        adapt.theta_max, tau_min, h_lb, h_ub, G);
		}

		step++;
	};
}

void mc::control::default_controller(robot_system &robot)
{
	for (size_t i = 0; i < robot.joints.size(); ++i)
		f_out(i) = 0.0;
}
