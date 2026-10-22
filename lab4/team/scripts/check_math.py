#!/usr/bin/env python3
"""Exact rational feasibility and KKT checks, without symbolic dependencies."""
from fractions import Fraction as F


def multiply(a, b):
    return [sum(x*y for x, y in zip(row, b)) for row in a]


def check(p, cost, terminal_velocity=False):
    p = [F(x) for x in p]
    n = len(p)
    q = [[F((i+1)*(j+1),i+j+1) for j in range(n)] for i in range(n)]
    qp = multiply(q,p)
    assert sum(p) == 1
    assert sum(x*y for x,y in zip(p,qp)) == F(cost)
    if terminal_velocity:
        assert sum((i+1)*p[i] for i in range(n)) == -2
        # At the optimum Qp lies in the row space of the two constraints.
        if n > 2:
            delta = qp[1]-qp[0]
            assert all(qp[i] == qp[0]+i*delta for i in range(n))
    else:
        assert len(set(qp)) == 1
    print('PASS p={} cost={}'.format(p, F(cost)))


for p in ([1], [1,0], [1,0,0]): check(p,1)
check([4,-3],4,True)
check([F(1,4),F(9,2),F(-15,4)],F(17,8),True)
for r in range(1,8):
    for k in range(1,10): assert 2*k+(2*r-2)*(k-1)+2*(r-1) == 2*r*k
print('PASS general constraint count, including k=1 and r=1')
