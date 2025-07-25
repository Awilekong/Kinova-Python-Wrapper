import numpy as np
import time

from juliacall import Main as jl, convert as jlconvert
jl.seval("using MeshCat")
jl.seval("using Rotations")
jl.seval("using TORA")


class TORA:
    def __init__(self, trajectory_discretisation=5, trajectory_duration=1.0):
        """
        Constructs a TORA object with specified parameters.

        Parameters
        ----------
            trajectory_discretisation : int
                frequency (in Hz) at which to discretise the trajectory
            trajectory_duration : float
                total duration of the trajectory (in seconds)
        """
        jl.println("Hello from Julia!")

        self.jl_robot = self.init()

        self.trajectory_discretisation = trajectory_discretisation
        self.trajectory_duration = trajectory_duration

        self.trajectory_num_knots = 5 #int(trajectory_discretisation * trajectory_duration) + 1  # in units (number of knots)
        self.trajectory_dt = 1 / trajectory_discretisation  # in seconds

        # Options for the Ipopt numerical solver
        ipopt_options = jl.Dict()
        ipopt_options["hessian_approximation"] = "limited-memory"  # L-BFGS
        ipopt_options["max_cpu_time"] = 2.0  # in seconds
        ipopt_options["mu_strategy"] = "adaptive"  # monotone, adaptive, quality-function
        # ipopt_options["acceptable_compl_inf_tol"] = 0.1  # default: 0.01
        ipopt_options["acceptable_constr_viol_tol"] = 0.1  # default: 0.01
        # ipopt_options["acceptable_dual_inf_tol"] = 1e5  # default: 1e10
        ipopt_options["acceptable_iter"] = 1  # default: 15
        ipopt_options["acceptable_tol"] = 1e-1  # default: 1e-6
        # ipopt_options["print_level"] = 3  # default: 5

        self.ipopt_options = ipopt_options

    def slerp(self, q1, q2, t):
        """Spherical Linear Interpolation (SLERP) for quaternions."""
        dot = np.dot(q1, q2)

        # Ensure shortest path
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        dot = np.clip(dot, -1.0, 1.0)
        theta = np.arccos(dot)

        if abs(theta) < 1e-6:
            return q2  # If very close, return q1

        sin_theta = np.sin(theta)
        w1 = np.sin((1 - t) * theta) / sin_theta
        w2 = np.sin(t * theta) / sin_theta
        return w1 * q1 + w2 * q2

    def plan(self,
             current_joint_pos, current_joint_vel,
             current_ee_trans, current_ee_quat,
             target_ee_trans, target_ee_quat):

        # Create a problem instance
        jl_problem = jl.TORA.Problem(self.jl_robot, self.trajectory_num_knots, self.trajectory_dt)

        # Ensure the problem is initialized with the correct parameters
        assert jl_problem.num_knots == self.trajectory_num_knots
        assert jl_problem.dt == self.trajectory_dt

        # Get the current robot state (pos and vel)
        py_q = current_joint_pos  # current joint positions
        py_v = current_joint_vel  # current joint velocities

        # Constrain the initial joint positions and velocities to the current state of the robot
        jl.TORA.fix_joint_positions_b(jl_problem, self.jl_robot, 1, py_q)
        jl.TORA.fix_joint_velocities_b(jl_problem, self.jl_robot, 1, py_v)

        # Constrain the final joint velocities to zero
        # jl.TORA.fix_joint_velocities_b(jl_problem, self.jl_robot, jl_problem.num_knots, np.zeros(self.jl_robot.n_v))

        body_name = "panda_link7"  # this is the fixed parent link of "panda_hand_tcp"

        # adding constrain to each knot
        for k in range(2, jl_problem.num_knots):
            t = (k - 1) / (jl_problem.num_knots - 1)
            # print(f"{k=}, {t=}")

            # Linear interpolation between the current and target positions
            py_eff_pos_k = (1 - t) * current_ee_trans + t * target_ee_trans
            jl.TORA.constrain_ee_position_b(jl_problem, k, py_eff_pos_k)

            # Quaternion interpolation between the current and target orientations
            py_eff_quat_k = self.slerp(current_ee_quat, target_ee_quat, t)
            jl_quat_k = jl.QuatRotation(py_eff_quat_k[3], py_eff_quat_k[0], py_eff_quat_k[1], py_eff_quat_k[2])  # change quat order to w,x,y,z 
            jl.TORA.add_constraint_body_orientation_b(jl_problem, self.jl_robot, body_name, k, jl_quat_k)

            # If this loop iteration is setting the constraints for the last knot of the trajectory, check that
            # the interpolated values match the target position and orientation for the end of the trajectory.
            if k == jl_problem.num_knots:
                assert np.allclose(py_eff_pos_k, target_ee_trans)
                assert np.allclose(py_eff_quat_k, target_ee_quat)


        jl.TORA.constrain_ee_position_b(jl_problem, jl_problem.num_knots, target_ee_trans)
        jl_quaternion = jl.QuatRotation(target_ee_quat[3], target_ee_quat[0], target_ee_quat[1], target_ee_quat[2]) 
        jl.TORA.add_constraint_body_orientation_b(jl_problem, self.jl_robot, body_name, jl_problem.num_knots, jl_quaternion)

        print(f"{target_ee_trans=} {target_ee_quat=}")

        # Show a summary of the problem instance (this can be commented out)
        jl.TORA.show_problem_info(jl_problem)

        # Prepare the initial guess for the solver
        jl_initial_guess = self.prepare_initial_guess(jl_problem, self.jl_robot, py_q)

        jl_cpu_time, jl_x, jl_solver_log = jl.TORA.solve_with_ipopt(
            jl_problem,
            self.jl_robot,
            initial_guess=jl_initial_guess,
            user_options=self.ipopt_options,
            use_inv_dyn=True,
            minimise_velocities=False,
            minimise_torques=True,
        )

        # Unpack the solution `x` into joint positions, velocities, and torques
        jl_qs, jl_vs, jl_τs = jl.TORA.unpack_x(jl_x, self.jl_robot, jl_problem)

        # Convert the Julia arrays to numpy arrays
        py_qs = np.array(jl_qs).T
        py_vs = np.array(jl_vs).T
        py_τs = np.array(jl_τs).T
        return py_qs[1:], py_vs[1:], py_τs[1:]

    def init(self):
        """
        Initialize the Julia environment, load the TORA.jl package, and create a robot model.
        """
        # Load package dependencies
        # jl.seval("using MeshCat")
        # jl.seval("using Rotations")
        # jl.seval("using TORA")

        # Call the greet() function from the TORA.jl package
        jl.TORA.greet()

        jl_vis = jl.MeshCat.Visualizer()
        # jl.open(jl_vis)  # NOT WORKING

        jl_robot = jl.TORA.create_robot_franka("panda_arm", jl_vis)

        return jl_robot

    def prepare_initial_guess(self, jl_problem, jl_robot, py_q):
        # Start by assuming zero positions, velocities, and torques for all the knots of the trajectory
        py_initial_qs = np.zeros((jl_problem.num_knots, jl_robot.n_q))
        py_initial_vs = np.zeros((jl_problem.num_knots, jl_robot.n_v))
        py_initial_τs = np.zeros((jl_problem.num_knots, jl_robot.n_τ))

        # To improve the initial guess for the robot joint positions, we can take
        # the current robot configuration and simply repeat it for all the knots
        py_initial_qs = np.tile(py_q, (jl_problem.num_knots, 1))

        # Concatenate the initial guess
        py_initial_guess = np.concatenate((py_initial_qs, py_initial_vs, py_initial_τs), axis=1)

        # Flatten the numpy array
        py_initial_guess = py_initial_guess.flatten()

        # Truncate the torques at the last knot of the trajectory from the initial guess
        py_initial_guess = py_initial_guess[: -jl_robot.n_τ]

        # Convert the numpy array to a Julia array
        jl_initial_guess = jlconvert(jl.Vector[jl.Float64], py_initial_guess)

        return jl_initial_guess