#  Now for the whole shebang...

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

# The states: x, t1, t2
x, t1, t2, xdot, t1dot, t2dot, xddot, t1ddot, t2ddot, tau = \
    syms('x t1 t2 xdot t1dot t2dot xddot t1ddot t2ddot tau')

# parameters
#m, l, g = syms('m l g')
# M of cart, m1 pendulum, m2 flywheel
M, m1, l1, I1, m2, l2, I2, g = syms('M m1 l1 I1 m2 l2 I2 g')

#p = Matrix([m1, l1, I1, m2, l2, I2, g])      # parameter vector
q = Matrix([x, t1, t2])

qdot = Matrix([xdot, t1dot, t2dot]) # time derivative of q
qddot = Matrix([xddot, t1ddot, t2ddot]) # time derivative of qdot

#qdot = diff(q, t)
#qddot = diff(qdot, t)

l1 = 0.5 * l2


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



# https://ocw.mit.edu/courses/mechanical-engineering/2-003sc-engineering-dynamics-fall-2011/lagrange-equations/MIT2_003SCF11_rec8notes1.pdf

#K_translat = Matrix([0.5 * m1 * (l1 * t1dot)**2 + \
#    0.5 * m2 * (l2 * t2dot)**2])
# K_inertial = 0.5 * Matrix([m1 * (l1 * t1dot)**2, 
                           # m2 * (l2 * t2dot)**2])
state_cart = Matrix([x, 0])
state_pend = Matrix([ x + l1 * sin(t1), \
                     l1 * cos(t1)])
state_wheel = Matrix([ l2 * sin(t1),  \
                      l2 * cos(t1) ])
v_cart = state_cart.jacobian(Matrix([x])) * Matrix([xdot])
v_pend = state_pend.jacobian(Matrix([x, t1])) * Matrix([xdot, t1dot])
v_wheel = state_wheel.jacobian(Matrix([t1])) * Matrix([t1dot])

print('v_cart', v_cart.shape)

K_cart = M * v_cart.T * v_cart / 2
K_pend = m1 * v_pend.T * v_pend / 2
K_wheel = m2 * v_wheel.T * v_wheel / 2

K_translat = K_cart + K_pend + K_wheel

# angular things
K_inertial = Matrix([0.5 * I1 * t1dot**2 + \
    0.5 * I2 * t1dot**2 + \
    0.5 * I2 * t2dot**2])

P = Matrix([m1 * g * (1 - l1 * cos(t1)) + m2 * g * (1 - l2 * cos(t1))])

# P = Matrix([m1 * g * (1 - l1 * cos(t1)), 
            # m2 * g * (1 - l2 * cos(t1))])

# K_translat = 0.5 * Matrix([I1 * t1dot**2 + I2 * t1dot**2, 
                           # I2 * t2dot**2])

# dynamics:
# P = m * g * l * (1 - cos(q)) 
# P = Matrix([m * g * l * (1 - cos(q)) ])
# K = 0.5 * m * l**2 * (qdot.T * qdot) 

# Lagrangian L=sum(K)-sum(P)
L =  K_translat + K_inertial - P

print(L)
print('L', L.shape)
print('q', q.shape)

# first term in the Euler-Lagrange equation
partial_L_by_partial_q = L.jacobian(Matrix([q])).T
print('dL dq', partial_L_by_partial_q.shape)
print('dL dq', partial_L_by_partial_q)

print('qdot', qdot)
# inner term of the second part of the Euler-Lagrange equation
partial_L_by_partial_qdot = L.jacobian(Matrix([qdot]))
print(partial_L_by_partial_qdot)
print('dL dqdot', partial_L_by_partial_qdot.shape)

# second term (overall, time derivative) in the Euler-Lagrange equation
# applies the chain rule
d_inner_by_dt = partial_L_by_partial_qdot.jacobian(Matrix([q])) * qdot + \
    partial_L_by_partial_qdot.jacobian(Matrix([qdot])) * qddot


# Euler-Lagrange equation
# no force on cart, no force on pendulum, just tau on reaction wheel
lagrange_eq = partial_L_by_partial_q - d_inner_by_dt + Matrix([0, 0, tau])

# solve the lagrange equation for qddot and simplify
prevTime = time.time()
print('START: ', prevTime)
print("Calculations take a while...")
r = sympy.solvers.solve(simplify(lagrange_eq), Matrix([qddot]))
print('Elapsed: ', time.time() - prevTime)
prevTime = time.time()

print("Simplifying...")
#qddot = simplify(r)
t1ddot = simplify(r[t1ddot])
t2ddot = simplify(r[t2ddot])
#tau2 = simplify(r[tau])
# t1dot = simplify(r[t1dot])
# t2dot = simplify(r[t2dot])

print('t1ddot= {}\n'.format(t1ddot));
print('t2ddot= {}\n'.format(t2ddot));
#print('tau= {}\n'.format(tau2));
# print('t1dot= {}\n'.format(t1dot));
# print('t2dot= {}\n'.format(t2dot));

print('Elapsed: ', time.time() - prevTime)
prevTime = time.time()
# generate python function
print(t1, t2, t1dot, t2dot, t1ddot, t2ddot)
# print(t1, t2, t1dot, t2dot, t1ddot, t2ddot)
print(m1, l1, I1, m2, l2, I2, g)

f_gen = open("reaction_pendulum_dynamics_generated.py", 'w')

f_gen.close()
## RESULTS
# t1ddot= l2*(2.0*g*(M + m1)*(m1 + 2.0*m2) + l2*m1**2*t1dot**2*cos(t1))*sin(t1)/(l2**2*m1**2*cos(t1)**2 - (M + m1)*(4.0*I1 + 4.0*I2 + l2**2*m1 + 4.0*l2**2*m2))

# t2ddot= tau/I2


## LINEARIZATION
# linearization should be straightforward from here
# just plug in the fixed points theta values, instead of the cos(theta) and
# sin(theta) above
