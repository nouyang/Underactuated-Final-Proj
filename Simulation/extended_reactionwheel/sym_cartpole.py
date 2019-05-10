# sanity check -- adapt for cartpole
# for double pendulum on cart

# You are free to use, modify, copy, distribute the code.
# Please give a clap on medium, star on github, or share the article if you
# like.
# Created by Jonas, github.com/jkoendev

# Double pendulum on a cart (dpc) simulation code
#
# This file generates the model from symbolic expressions and generates a
# file with the name `dpc_dynamics_generated.py`

import sympy
from sympy import sin, cos, simplify
from sympy import symbols as syms
from sympy.matrices import Matrix
from sympy.utilities.lambdify import lambdastr

import time

# state, state derivative, and control variables
# we only have theta for the simple pendulum
# the force input -- there is none
q, qdot, qddot = syms('q qdot qddot')

# parameters
m, l, g = syms('m l g')

p = Matrix([m, l, g])      # parameter vector

#q = Matrix([q])                   # generalized positions
qdot = Matrix([qdot])       # time derivative of q
qddot = Matrix([qddot])   # time derivative of qdot

# To calculate time derivatives of a function f(q), we use:
# df(q)/dt = df(q)/dq * dq/dt = df(q)/dq * qdot

# kinematics:

# dynamics:
#P = m * g * l * (1 - cos(q)) 
P = Matrix([m * g * l * (1 - cos(q)) ])
# df(q)/dt = df(q)/dq * dq/dt = df(q)/dq * qdot
# v_c = p_c.jacobian(Matrix([q_0])) * Matrix([qdot_0])
K = 0.5 * m * l**2 * (qdot.T * qdot) 

# v_c = p_c.jacobian(Matrix([q_0])) * Matrix([qdot_0])
 
# Lagrangian L=sum(K)-sum(P)
L =  K - P

# first term in the Euler-Lagrange equation
partial_L_by_partial_q = L.jacobian(Matrix([q])).T

# inner term of the second part of the Euler-Lagrange equation
partial_L_by_partial_qdot = L.jacobian(Matrix([qdot]))

# second term (overall, time derivative) in the Euler-Lagrange equation
# applies the chain rule
d_inner_by_dt = partial_L_by_partial_qdot.jacobian(Matrix([q])) * qdot + partial_L_by_partial_qdot.jacobian(Matrix([qdot])) * qddot

# Euler-Lagrange equation
lagrange_eq = partial_L_by_partial_q - d_inner_by_dt + Matrix([0])

# solve the lagrange equation for qddot and simplify
prevTime = time.time()
print('START: ', prevTime)
print("Calculations take a while...")
r = sympy.solvers.solve(simplify(lagrange_eq), Matrix([qddot]))
print('Elapsed: ', time.time() - prevTime)
prevTime = time.time()

print("Simplifying...")
#qddot = simplify(r)
qddot = r#[qddot]

print('qddot = {}\n'.format(qddot));

print('Elapsed: ', time.time() - prevTime)
prevTime = time.time()
# generate python function
print((q, qdot, qddot, l, m, g))
#print(s)

f_gen = open("pendulum_dynamics_generated.py", 'w')
#f_gen.write("import math\ndef dpc_dynamics_generated(q_0, q_1, q_2, qdot_0, qdot_1, qdot_2, f, r_1, r_2, m_c, m_1, m_2, g):\n\tfun="+s+"\n\treturn fun(q_0, q_1, q_2, qdot_0, qdot_1, qdot_2, f, r_1, r_2, m_c, m_1, m_2, g)")
f_gen.close()

