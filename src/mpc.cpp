#include <ros/ros.h>
#include <mpc_path_tracking.h>

void FG_eval::operator()(ADvector &fg, const ADvector &vars)
{
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.

    fg[0] = 0;
    
    nh.param<double>("cost_weight/cte_cost_weight", cte_cost_weight, 1000);
    nh.param<double>("cost_weight/epsi_cost_weight", epsi_cost_weight, 1000);
    nh.param<double>("cost_weight/v_start_weight", v_weight, 1);
    nh.param<double>("cost_weight/delta_cost_weight", delta_cost_weight, 500);
    nh.param<double>("cost_weight/v_target_cost_weight", v_target_cost_weight, 1);
    nh.param<double>("cost_weight/delta_change_cost_weight", delta_change_cost_weight, 10000);
    nh.param<double>("cost_weight/v_target_change_cost_weight", v_target_change_cost_weight, 1);

    // Cost for CTE, psi error and velocity
    for (int t = 0; t < N; t++)
    {
        fg[0] += cte_cost_weight * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
        fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
        fg[0] += v_weight * CppAD::pow(vars[v_start + t] - vars[v_target_start + t - 1], 2);
    }

    // Costs for steering (delta) and acceleration (a)
    for (int t = 0; t < N - 1; t++)
    {
        fg[0] += delta_cost_weight * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += v_target_cost_weight * CppAD::pow(vars[v_target_start + t] - ref_v, 2);
    }

    // Costs related to the change in steering and acceleration (makes the ride smoother)
    for (int t = 0; t < N - 2; t++)
    {
        fg[0] += delta_change_cost_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += v_target_change_cost_weight * CppAD::pow(vars[v_target_start + t + 1] - vars[v_target_start + t], 2);
    }

    // Setup Model Constraints

    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (int t = 1; t < N; t++)
    {
        // State at time t + 1
        AD<double> x1 = vars[x_start + t];
        AD<double> y1 = vars[y_start + t];
        AD<double> psi1 = vars[psi_start + t];
        AD<double> v1 = vars[v_start + t];
        AD<double> cte1 = vars[cte_start + t];
        AD<double> epsi1 = vars[epsi_start + t];

        // The state at time t.
        AD<double> x0 = vars[x_start + t - 1];
        AD<double> y0 = vars[y_start + t - 1];
        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> v0 = vars[v_start + t - 1];
        AD<double> cte0 = vars[cte_start + t - 1];
        AD<double> epsi0 = vars[epsi_start + t - 1];

        // Actuator constraints at time t only
        AD<double> delta0 = vars[delta_start + t - 1];
        AD<double> v_target0 = vars[v_target_start + t - 1];

        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
        AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

        // Setting up the rest of the model constraints
        
        
        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
        fg[1 + v_start + t] = v1 - (v0 + k_v * (v_target0 - v0) * dt);
        fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 / Lf * delta0 * dt);
    }
}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    bool ok = true;
    //size_t i;

    typedef CPPAD_TESTVECTOR(double) Dvector;

    const double x = state[0];
    const double y = state[1];
    const double psi = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double epsi = state[5];

    size_t n_vars = state.size() * N + (N-1) * 2;
    size_t n_constraints = N * state.size();

    Dvector vars(n_vars);

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // TODO: Set lower and upper limits for variables.
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < delta_start; i++) 
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // For delta. [-40, 40] in radians
    for (int i = delta_start; i < v_target_start; i++) 
    {
        vars_lowerbound[i] = steer_lowerBound;
        vars_upperbound[i] = steer_upperBound;
    }

    for (int i = v_target_start; i < n_vars; i++ ) {
        vars_lowerbound[i] = speed_lowerBound;
        vars_upperbound[i] = speed_upperBound;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; i++) 
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // Start lower and upper limits at current values
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // options for IPOPT solver
    string options;

    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    cost = solution.obj_value;
    

    // Returns the first actuator values, along with predicted x and y values to plot in the simulator
    std::vector<double> result;
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[v_target_start]);

    for (int i = 0; i < N; ++i)
    {
        result.push_back(solution.x[x_start + i]);
        result.push_back(solution.x[y_start + i]);
    }

    return result;
}

Eigen::VectorXd MPC::predict_FutureState(float velocity_actual, float delta, float cte, float epsi, double target_velocity, double dt)
{
    // pred_px = 0;
    // pred_py = 0;
    // pred_psi = 0;
    // pred_v = velocity_actual + k_v * (target_velocity - velocity_actual) * dt;
    // pred_cte = cte + velocity_actual * sin(epsi) * dt;
    // pred_epsi = epsi + pred_psi;

    pred_px = velocity_actual * dt;
    pred_py = 0;
    pred_psi = velocity_actual * (-delta) * dt / Lf;
    pred_v = velocity_actual + k_v * (target_velocity - velocity_actual) * dt;
    pred_cte = cte + velocity_actual * sin(epsi) * dt;
    pred_epsi = epsi + pred_psi;

    Eigen::VectorXd state(6);
    state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

    return state;
}
