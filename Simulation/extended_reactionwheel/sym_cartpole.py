# sanity check -- adapt for reaction wheel inverted pendulum

# You are free to use, modify, copy, distribute the code.
# Please give a clap on medium, star on github, or share the article if you
# like.
# Created by Jonas, github.com/jkoendev

# Double pendulum on a cart (dpc) simulation code
#
# This file generates the model from symbolic expressions and generates a
# file with the name `dpc_dynamics_generated.py`

import sympy
from sympy import sin, cos, simplify, Derivative, diff
from sympy import symbols as syms
from sympy.matrices import Matrix
from sympy.utilities.lambdify import lambdastr

import time

# https://docs.sympy.org/0.7.6/modules/physics/mechanics/examples/rollingdisc_example_lagrange.html
# state, state derivative, and control variables
# we only have theta for the simple pendulum
# the force input -- there is none
#q, qdot, qddot = syms('q qdot qddot')
x, theta, xdot, tdot, xddot, tddot, u = syms('x theta xdot tdot xddot tddot u')

# parameters
#m, l, g = syms('m l g')
M, m, g, len = syms('M m g len')

p = Matrix([M, m, g, len])      # parameter vector
q = Matrix([x, theta])

qdot = Matrix([xdot, tdot])  # time derivative of q
qddot = Matrix([xddot, tddot])  # time derivative of qdot

#qdot = diff(q, t)
#qddot = diff(qdot, t)

# To calculate time derivatives of a function f(q), we use:
# df(q)/dt = df(q)/dq * dq/dt = df(q)/dq * qdot

# Write as a matrix as fxn of the q terms
# one for t1, one for t2
#x = m1 * l1**2
#x = m2 * l2**2

#l2 = 2 * l1


#pendulumCoM = Matrix([l1 * sin(t1), l1 * cos(t1)])
#wheelCoM = Matrix([l2 * sin(t1), l2 * cos(t1)])
# v_pend = pendulumCoM.jacobian(Matrix([t1])) * Matrix([t1dot])
# v_wheel = wheelCoM.jacobian(Matrix([t2])) * Matrix([t2dot])
# K_translat = 0.5 * m1 * v_pend.T * v_pend + \
    # 0.5 * m1 * v_pend.T * v_pend 


K = Matrix([0.5 * (M + m) * xdot**2 + \
    m * len * xdot * tdot * cos(theta) + \
    0.5 * m * len**2 * tdot**2])

# K_translat = Matrix([0.5 * m1 * (l1 * t1dot)**2 + \
    # 0.5 * m2 * (l2 * t2dot)**2])
# K_inertial = 0.5 * Matrix([m1 * (l1 * t1dot)**2, 
                           # m2 * (l2 * t2dot)**2])

# K_inertial = Matrix([0.5 * I1 * t1dot**2 + \
    # 0.5 * I2 * t1dot**2 + \
    # 0.5 * I2 * t2dot**2])

# P = Matrix([m1 * g * (1 - l1 * cos(t1)) + m2 * g * (1 - l2 * cos(t1))])
P = Matrix([-1 * m * g * len * (1 - cos(theta))])

# P = Matrix([m1 * g * (1 - l1 * cos(t1)), 
            # m2 * g * (1 - l2 * cos(t1))])

# K_translat = 0.5 * Matrix([I1 * t1dot**2 + I2 * t1dot**2, 
                           # I2 * t2dot**2])

# dynamics:
# P = m * g * l * (1 - cos(q)) 
# P = Matrix([m * g * l * (1 - cos(q)) ])
# K = 0.5 * m * l**2 * (qdot.T * qdot) 

# Lagrangian L=sum(K)-sum(P)
# L =  K_translat + K_inertial - P
L =  K - P

print(L)
print('L', L.shape)
print('q', q.shape)

# first term in the Euler-Lagrange equation
partial_L_by_partial_q = L.jacobian(Matrix([q])).T
print('\ndL dq', partial_L_by_partial_q)
print('dL dq', partial_L_by_partial_q.shape)

# inner term of the second part of the Euler-Lagrange equation
partial_L_by_partial_qdot = L.jacobian(Matrix([qdot]))
print('\ndL dqdot', partial_L_by_partial_qdot)
print('dL dqdot', partial_L_by_partial_qdot.shape)

# second term (overall, time derivative) in the Euler-Lagrange equation
# applies the chain rule
d_inner_by_dt = partial_L_by_partial_qdot.jacobian(Matrix([q])) * qdot + \
    partial_L_by_partial_qdot.jacobian(Matrix([qdot])) * qddot
print('\nd/dt dL/d_qdot', d_inner_by_dt)

# Euler-Lagrange equation
lagrange_eq = partial_L_by_partial_q - d_inner_by_dt + Matrix([0, 0])
print('\nlagrange eq', lagrange_eq)

# solve the lagrange equation for qddot and simplify
prevTime = time.time()
print('START: ', prevTime)
print("Calculations take a while...")
r = sympy.solvers.solve(simplify(lagrange_eq), Matrix([qddot]))
print('Elapsed: ', time.time() - prevTime)
prevTime = time.time()

print("Simplifying...")
#qddot = simplify(r)
xddot = simplify(r[xddot])
tddot = simplify(r[tddot])

print('xddot= {}\n'.format(xddot));
print('tddot= {}\n'.format(tddot));

print('Elapsed: ', time.time() - prevTime)
prevTime = time.time()
# generate python function
print(x, theta, xdot, tdot, xddot, tddot, u)
print(M, m, g, len)


f_gen = open("reaction_pendulum_dynamics_generated.py", 'w')

f_gen.close()
