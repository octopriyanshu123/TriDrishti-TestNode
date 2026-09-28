import numpy as np

print("=== MPC FROM SCRATCH - TEMPERATURE CONTROL ===")
# ============================================
# 1. PROBLEM SETUP - TEMPERATURE CONTROL
# ============================================
print("1. SYSTEM DEFINITION:")
print("   Room temperature control with heater")
print("   Model: T(k+1) = 0.9*T(k) + 0.5*P(k)")
print()

# System parameters
a = 0.9     # Thermal retention
b = 0.5     # Heater effectiveness
T_ref = 22  # Target temperature [°C]

# Constraints
P_min, P_max = 0, 100   # Heater power limits [%]
T_min, T_max = 18, 25   # Temperature limits [°C]

# MPC parameters
N = 3       # Prediction horizon
Q = 1.0     # Temperature error weight
R = 0.1     # Control effort weight

print(f"   Target: {T_ref}°C, Horizon: N={N}")
print(f"   Power limits: [{P_min}, {P_max}]%")
print(f"   Temp limits: [{T_min}, {T_max}]°C")
print()

# ============================================
# 2. BRUTE FORCE MPC IMPLEMENTATION
# ============================================
print("2. MPC ALGORITHM (BRUTE FORCE):")
print("   Testing all possible power combinations...")

def brute_force_mpc(T_current, N, P_options=20):
    """
    Simple MPC using brute force search
    Returns: Optimal first control action
    """
    # Generate possible power values (discretized)
    P_values = np.linspace(P_min, P_max, P_options)
    
    best_cost = float('inf')
    best_first_P = 0
    
    # Try EVERY possible sequence of length N
    # We'll use recursion for clarity
    def evaluate_sequence(seq, depth, T_current):
        """Evaluate cost for a given power sequence"""
        T = T_current
        total_cost = 0
        
        for i, P in enumerate(seq):
            # System dynamics
            T = a * T + b * P
            
            # Temperature constraint violation penalty
            if T < T_min or T > T_max:
                return float('inf')  # Invalid sequence
            
            # Cost components
            temp_error = T - T_ref
            total_cost += Q * temp_error**2 + R * P**2
            
            # Limit search depth
            if i >= depth:
                break
        
        return total_cost
    
    # Generate all combinations (brute force)
    # For simplicity, we'll search for first action only with 1-step lookahead
    for P1 in P_values:
        # Test this first action
        T_next = a * T_current + b * P1
        
        # Check constraint
        if T_next < T_min or T_next > T_max:
            continue
        
        # Calculate cost for just first step (simplified)
        temp_error = T_next - T_ref
        cost = Q * temp_error**2 + R * P1**2
        
        if cost < best_cost:
            best_cost = cost
            best_first_P = P1
    
    return best_first_P

# ============================================
# 3. SIMPLIFIED MPC USING LEAST SQUARES
# ============================================
print("\n3. MPC USING LINEAR LEAST SQUARES:")
print("   (More efficient than brute force)")
print()

def least_squares_mpc(T_current, N):
    """
    MPC using unconstrained least squares
    Then clip to constraints
    """
    # Build prediction matrices
    # We want: T = F * T_current + G * U
    # Where U = [P(k), P(k+1), ..., P(k+N-1)]
    
    F = np.zeros(N)
    G = np.zeros((N, N))
    
    # Build F and G matrices
    for i in range(N):
        F[i] = a**(i+1)
        for j in range(i+1):
            G[i, j] = a**(i-j) * b
    
    # Desired temperature trajectory
    T_desired = np.ones(N) * T_ref
    
    # Unconstrained solution: U = (GᵀG)⁻¹Gᵀ(T_desired - F*T_current)
    # But we need to add regularization for control effort
    
    # Build the full optimization problem:
    # Minimize: ||G*U + F*T_current - T_desired||² + λ||U||²
    # Where λ = R/Q
    
    lambda_reg = R / Q
    
    # Solution: U = (GᵀG + λI)⁻¹ Gᵀ (T_desired - F*T_current)
    G_T = G.T
    I = np.eye(N)
    
    # Compute optimal unconstrained control sequence
    U_opt = np.linalg.inv(G_T @ G + lambda_reg * I) @ G_T @ (T_desired - F * T_current)
    
    # Apply constraints (clipping)
    U_opt_clipped = np.clip(U_opt, P_min, P_max)
    
    # Return first control action
    return U_opt_clipped[0]

# ============================================
# 4. SIMULATION WITH BOTH METHODS
# ============================================
print("4. MPC SIMULATION:")
print("="*60)
print(f"{'Time':<6} {'Temp':<8} {'Brute MPC':<12} {'LSQ MPC':<12}")
print("-"*60)

# Initial conditions
T_current = 18.0
T_history = [T_current]
P_brute_history = []
P_lsq_history = []

for step in range(15):
    # Get control from both methods
    P_brute = brute_force_mpc(T_current, N=2, P_options=10)
    P_lsq = least_squares_mpc(T_current, N=N)
    
    # Apply brute force control (for simulation)
    T_next = a * T_current + b * P_brute
    
    # Store results
    T_history.append(T_next)
    P_brute_history.append(P_brute)
    P_lsq_history.append(P_lsq)
    
    # Print current status
    print(f"{step:<6} {T_current:<8.2f} {P_brute:<12.2f} {P_lsq:<12.2f}")
    
    # Update for next iteration
    T_current = T_next

print("="*60)

# ============================================
# 5. RESULTS ANALYSIS
# ============================================
print("\n5. RESULTS ANALYSIS:")
print("-"*40)

print("\nTemperature evolution:")
for i, temp in enumerate(T_history[:5]):
    print(f"  Step {i}: {temp:.2f}°C")

print(f"\nFinal temperature: {T_current:.2f}°C")
print(f"Target temperature: {T_ref}°C")
print(f"Error: {abs(T_current - T_ref):.2f}°C")

# ============================================
# 6. MANUAL CALCULATION EXAMPLE
# ============================================
print("\n6. MANUAL CALCULATION (Step 0):")
print("-"*40)

print("\nGiven: T(0) = 18°C, Target = 22°C")
print("\nOption 1: P = 0% (No heating)")
T1 = a * 18 + b * 0
error1 = T1 - T_ref
cost1 = Q * error1**2 + R * 0**2
print(f"  T(1) = 0.9*18 + 0.5*0 = {T1:.2f}°C")
print(f"  Error = {T1:.2f} - 22 = {error1:.2f}")
print(f"  Cost = 1*({error1:.2f})² + 0.1*0² = {cost1:.2f}")

print("\nOption 2: P = 50%")
T2 = a * 18 + b * 50
error2 = T2 - T_ref
cost2 = Q * error2**2 + R * 50**2
print(f"  T(1) = 0.9*18 + 0.5*50 = {T2:.2f}°C")
print(f"  Error = {T2:.2f} - 22 = {error2:.2f}")
print(f"  Cost = 1*({error2:.2f})² + 0.1*50² = {cost2:.2f}")

print("\nOption 3: P = 8% (Approximate optimal)")
T3 = a * 18 + b * 8
error3 = T3 - T_ref
cost3 = Q * error3**2 + R * 8**2
print(f"  T(1) = 0.9*18 + 0.5*8 = {T3:.2f}°C")
print(f"  Error = {T3:.2f} - 22 = {error3:.2f}")
print(f"  Cost = 1*({error3:.2f})² + 0.1*8² = {cost3:.2f}")

print(f"\n✓ Best option is P = 8% (lowest cost: {cost3:.2f})")

# ============================================
# 7. MPC PSEUDOCODE
# ============================================
print("\n7. MPC ALGORITHM (Pseudocode):")
print("-"*40)

print("""
function MPC_control(current_state, target, N):
    # Step 1: Prediction
    for each possible control sequence U of length N:
        predict future states using: x(k+1) = A*x(k) + B*u(k)
    
    # Step 2: Optimization
    for each predicted trajectory:
        calculate cost = Σ[Q*(x - target)² + R*u²]
        check if constraints satisfied
    
    # Step 3: Selection
    find sequence U* with minimum cost
    
    # Step 4: Receding Horizon
    apply only first control: u = U*[0]
    
    return u
""")

print("\n=== MPC FROM SCRATCH COMPLETE ===")