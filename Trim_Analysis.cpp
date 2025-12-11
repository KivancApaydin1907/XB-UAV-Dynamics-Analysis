// Tool: Longitudinal Static Stability & Trim Analyzer
// Method: Newton-Raphson Solver with V-Tail Physics from "XB Dynamics Doc"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>

using namespace std;

// --- Constants ---
const double PI = 3.141592653589793;
const double DEG_TO_RAD = PI/180.0;

// --- XB Aircraft Physical Parameters  ---

// 1. Aerodynamic Coefficients
const double Cm_ac_wing = -0.17413;
const double Cm_prop = -0.0012; 

// 2. V-Tail Geometry & Coefficients
// Gamma = 20.6 degrees (Dihedral) 
const double COS_DIHEDRAL = 0.93606; // cos(20.6)
const double SIN_DIHEDRAL = 0.352;   // sin(20.6)

// 3. Tail Volume Coefficients (Moment Arms)
// Term: (lt*St)/(c*S) -> Longitudinal Arm Ratio 
const double VOL_COEFF_LONGITUDINAL = 0.355;

// Term: (zt*St)/(c*S) -> Vertical Arm Ratio
const double VOL_COEFF_VERTICAL = 0.0266;

// 4. Special 3D Correction Factor
const double AT_3D_CONST = 0.0781;

// --- Robust Data Manager (Class Structure) ---
class TailDataManager  
{
private:
	struct DataPoint // Structure for alpha and Cm
    {
        double alpha;
        double cm;
    };
	vector<DataPoint> data; // Vector to hold data points

public:
    // Load datat.txt file
	bool loadData(const string& filename)
    {
		ifstream file(filename); // Open the file
        if (!file.is_open())
        {
			// Error Handling
            cerr << "ERROR: Could not open '" << filename << "'!" << endl;
            cerr << "Please ensure datat.txt is in the same directory." << endl;
            return false;
        }
        double a, c;
        while (file >> a >> c) 
        {
			data.push_back({ a, c }); // Store alpha and Cm
        }
		file.close(); // Close the file
        cout << "Database: Loaded " << data.size() << " aerodynamic data points.\n";
        return true;
    }

    // Get Cm with Clamping (Prevents crash if alpha is out of bounds)
    double getCm(double target_alpha) 
    {
		if (data.empty()) return 0.0; // No data loaded

        // Safety Clamps
		if (target_alpha <= data.front().alpha) return data.front().cm; // Lower Bound
		if (target_alpha >= data.back().alpha) return data.back().cm; // Upper Bound

        // Linear Interpolation
        for (size_t i = 0; i < data.size() - 1; ++i) 
        {
            if (target_alpha >= data[i].alpha && target_alpha < data[i + 1].alpha) 
            {
                double slope = (data[i + 1].cm - data[i].cm) / (data[i + 1].alpha - data[i].alpha);
				return data[i].cm + (target_alpha - data[i].alpha) * slope; // Interpolated Cm
            }
        }
		return data.back().cm; // Fallback (should not reach here)
    }
};

TailDataManager tailDB;

// --- Physics Engine: V-Tail Moment Calculation ---
double calculate_total_moment(double alpha_tail_deg, double i_plane) 
{

    // Total angle seen by the tail
    double total_angle_deg = alpha_tail_deg + i_plane;
    double total_angle_rad = total_angle_deg*DEG_TO_RAD;

    // Get Aerodynamic Center Moment of Tail from Data (Cm,act)
    double cm_act = tailDB.getCm(total_angle_deg);

    // --- Term A: Tail Aerodynamic Center Contribution ---
    // Formula: (Cm,act)*sin(Gamma) 
    double term_ac = cm_act*SIN_DIHEDRAL;

    // --- Common Geometric Terms ---
	double geom_factor = AT_3D_CONST*total_angle_deg; // (at * alpha) 

    // The quadratic drag/lift term: 0.0046 + 0.1050*(at * alpha)^2 
    double aero_quadratic = 0.0046 + 0.1050*pow(geom_factor, 2);

    // --- Term B: Longitudinal Contribution (Lift-based) ---
    double lift_component = (geom_factor*cos(total_angle_rad)*COS_DIHEDRAL)
        + (aero_quadratic*sin(total_angle_rad));

    double term_longitudinal = lift_component*VOL_COEFF_LONGITUDINAL;

    // --- Term C: Vertical Contribution (Drag/Tilt-based) ---
    double drag_component = (geom_factor*sin(total_angle_rad)*COS_DIHEDRAL)
        - (aero_quadratic * cos(total_angle_rad));

    double term_vertical = drag_component * VOL_COEFF_VERTICAL;

    // Total Tail Moment Coefficient 
    double Cm_tail = term_ac - term_longitudinal + term_vertical;

    // Total Aircraft Moment Equation 
    // Cm_total = Cm_w + Cm_t + Cm_p
    return Cm_ac_wing + Cm_tail + Cm_prop; 
}

int main()  
{
    cout << "==============================================\n";
    cout << "              STABILITY SOLVER                \n";
    cout << "       Physics Model: V-Tail w/ Dihedral      \n";
    cout << "==============================================\n";

	if (!tailDB.loadData("datat.txt")) return 1; // Load tail data

	double i_plane = 0.0; // Aircraft Incidence Angle (deg)
    double alpha_tail = -2.0; // Initial Guess

    // Solver Settings
    double tolerance = 1e-6;
    int max_iter = 100;
    int iter = 0;

    cout << "\n[1] TRIMMING AIRCRAFT (Newton-Raphson Solver)...\n";

    // --- Newton-Raphson Loop ---
    for (iter = 0; iter < max_iter; iter++) 
    {
        double Cm_current = calculate_total_moment(alpha_tail, i_plane);

        if (abs(Cm_current) < tolerance) break; // Solution found

        // Numerical Derivative (Slope)
        double delta = 0.001;
        double Cm_plus = calculate_total_moment(alpha_tail + delta, i_plane);
        double gradient = (Cm_plus - Cm_current)/delta;

        if (abs(gradient) < 1e-9) 
        {
            alpha_tail += 0.1; // Avoid division by zero
            continue;
        }

        // Newton Step: x_new = x_old - f(x)/f'(x)
        alpha_tail = alpha_tail - (Cm_current/gradient);
    }

    double final_moment = calculate_total_moment(alpha_tail, i_plane);

    cout << "   -> Iterations: " << iter << "\n";
    cout << "   -> Trimmed Tail Angle: " << fixed << setprecision(5) << alpha_tail << " deg\n";
    cout << "   -> Residual Moment:    " << scientific << final_moment << "\n";

    // --- Stability Check ---
    cout << "\n[2] CHECKING STATIC STABILITY...\n";

	double alpha_perturb = 1.0; // Small Perturbation (deg)
	double Cm_disturbed = calculate_total_moment(alpha_tail, i_plane + alpha_perturb); // Perturb incidence
	double Cma = (Cm_disturbed - final_moment)/alpha_perturb; // Stability Derivative

    cout << "   -> Stability Derivative (Cma): " << fixed << setprecision(5) << Cma << " /deg\n";

    if (Cma < 0) 
    {
        cout << ">>> RESULT: STABLE configuration.\n";
    }
    else 
    {
        cout << ">>> RESULT: UNSTABLE configuration.\n";
    }

    cout << "\nPress Enter to exit...";
	cin.ignore(); 
	cin.get(); 

    return 0; 
}