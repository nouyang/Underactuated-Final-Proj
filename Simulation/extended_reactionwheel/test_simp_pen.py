from sympy.physics.mechanics import (dynamicsymbols, ReferenceFrame, Point,
                                     RigidBody, LagrangesMethod, Particle,
                                     inertia, Lagrangian)
from sympy import symbols, pi, sin, cos, tan, simplify, Function, \
    Derivative, Matrix


def test_simp_pen():
    # This tests that the equations generated by LagrangesMethod are identical
    # to those obtained by hand calculations. The system under consideration is
    # the simple pendulum.
    # We begin by creating the generalized coordinates as per the requirements
    # of LagrangesMethod. Also we created the associate symbols
    # that characterize the system: 'm' is the mass of the bob, l is the length
    # of the massless rigid rod connecting the bob to a point O fixed in the
    # inertial frame.
    q, u = dynamicsymbols('q u')
    qd, ud = dynamicsymbols('q u ', 1)
    l, m, g = symbols('l m g')

    # We then create the inertial frame and a frame attached to the massless
    # string following which we define the inertial angular velocity of the
    # string.
    N = ReferenceFrame('N')
    A = N.orientnew('A', 'Axis', [q, N.z])
    A.set_ang_vel(N, qd * N.z)

    # Next, we create the point O and fix it in the inertial frame. We then
    # locate the point P to which the bob is attached. Its corresponding
    # velocity is then determined by the 'two point formula'.
    O = Point('O')
    O.set_vel(N, 0)
    P = O.locatenew('P', l * A.x)
    P.v2pt_theory(O, N, A)

    # The 'Particle' which represents the bob is then created and its
    # Lagrangian generated.
    Pa = Particle('Pa', P, m)
    Pa.potential_energy = - m * g * l * cos(q)
    print('bob knietic', Pa.kinetic_energy(N))
    L = Lagrangian(N, Pa)

    # The 'LagrangesMethod' class is invoked to obtain equations of motion.
    lm = LagrangesMethod(L, [q])
    lm.form_lagranges_equations()
    RHS = lm.rhs()
    print('lm', lm)
    print('RHS', RHS)
    assert RHS[1] == -g*sin(q)/l


def test_nonminimal_pendulum():
    q1, q2 = dynamicsymbols('q1:3') # x, y ?
    q1d, q2d = dynamicsymbols('q1:3', level=1)
    L, m, t = symbols('L, m, t')
    g = 9.8
    # Compose World Frame
    N = ReferenceFrame('N')
    pN = Point('N*')
    pN.set_vel(N, 0)
    # Create point P, the pendulum mass
    P = pN.locatenew('P1', q1*N.x + q2*N.y)
    P.set_vel(N, P.pos_from(pN).dt(N))
    pP = Particle('pP', P, m)
    # Constraint Equations
    f_c = Matrix([q1**2 + q2**2 - L**2])
    # Calculate the lagrangian, and form the equations of motion
    Lag = Lagrangian(N, pP)
    LM = LagrangesMethod(Lag, [q1, q2], hol_coneqs=f_c,
                         forcelist=[(P, m*g*N.x)], frame=N)
    LM.form_lagranges_equations()
    print('LM mass matrix', LM.mass_matrix)
    # Check solution
    lam1 = LM.lam_vec[0, 0]
    eom_sol = Matrix([[m*Derivative(q1, t, t) - 9.8*m + 2*lam1*q1],
                      [m*Derivative(q2, t, t) + 2*lam1*q2]])
    print(eom_sol)
    assert LM.eom == eom_sol
    # Check multiplier solution
    lam_sol = Matrix([(19.6*q1 + 2*q1d**2 + 2*q2d**2)/(4*q1**2/m + 4*q2**2/m)])
    print(LM.solve_multipliers(sol_type='Matrix'))
    assert LM.solve_multipliers(sol_type='Matrix') == lam_sol


test_simp_pen()
test_nonminimal_pendulum()


def test_inertia_pendulum():
    xsym, ysym = dynamicsymbols('xsym1:3') # x, y ?
    xsymd, ysymd = dynamicsymbols('xsym1:3', level=1)
    L, m, t = symbols('L, m, t')
    g = 9.8
    # Compose World Frame
    N = ReferenceFrame('N')
    pN = Point('N*')
    pN.set_vel(N, 0)
    # Create point P, the pendulum mass
    P = pN.locatenew('P1', xsym*N.x + ysym*N.y)
    P.set_vel(N, P.pos_from(pN).dt(N))
    pP = Particle('pP', P, m)
    # Constraint Equations
    f_c = Matrix([xsym**2 + ysym**2 - L**2])
    # Calculate the lagrangian, and form the equations of motion
    Lag = Lagrangian(N, pP)
    LM = LagrangesMethod(Lag, [xsym, ysym], hol_coneqs=f_c,
                         forcelist=[(P, m*g*N.x)], frame=N)
    LM.form_lagranges_equations()
    # Check solution
    lam1 = LM.lam_vec[0, 0]
    eom_sol = Matrix([[m*Derivative(xsym, t, t) - 9.8*m + 2*lam1*xsym],
                      [m*Derivative(ysym, t, t) + 2*lam1*ysym]])
    print('inertial')
    print('lm', LM.eom)
    assert LM.eom == eom_sol
    # Check multiplier solution
    lam_sol = Matrix([(19.6*xsym + 2*xsymd**2 + 2*ysymd**2)/(4*xsym**2/m + 4*ysym**2/m)])
    print(LM.solve_multipliers(sol_type='Matrix'))
    assert LM.solve_multipliers(sol_type='Matrix') == lam_sol

test_inertia_pendulum()
