"""LQR/LQG del brazo de 3 GDL con graficas de convergencia.
Requiere: numpy, scipy, matplotlib.  Ejecutar: python lqg_3gdl_graficas.py
Parametros marcados [PROV] son provisionales (faltan datos del CAD)."""
import numpy as np
from scipy.linalg import solve_continuous_are as care
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

rad = np.pi/180; g = 9.81
# ---------------- Parametros ----------------
m2, m3, m1 = 1.070, 0.300, 0.050            # m1 [PROV]
L2, d2, d3 = 0.180, 0.038, 0.032
a2, b2, c2 = L2/2, 0.0, 0.0                 # [PROV] CM conjunto 2
a3, b3, c3 = 0.120/2, 0.0, 0.0              # [PROV] CM conjunto 3
a1 = b1 = 0.0                               # [PROV]
I1 = 1e-4*np.array([[4.5592015,-0.0035457,0.0167289],[-0.0035457,4.5323812,0],[0.0167289,0,3.8840580]])
I2 = 1e-3*np.array([[6.80582900,-2.94565886,2.54217153],[-2.94565886,9.24674771,-2.25256127],[2.54217153,-2.25256127,8.16941144]])
I3 = 1e-4*np.array([[1.0546731,-0.5095049,-0.4979885],[-0.5095049,1.9958820,0.2155950],[-0.4979885,0.2155950,1.6805178]])
J1 = I1[2,2] + m1*(a1**2 + b1**2)

# ---------------- Dinamica ----------------
def Mfun(q):
    q2_, q3_ = q[1], q[2]; th = q2_+q3_
    rho2 = a2*np.cos(q2_)-c2*np.sin(q2_); eta2 = a2*np.sin(q2_)+c2*np.cos(q2_)
    rho3 = a3*np.cos(th)-c3*np.sin(th);   eta3 = a3*np.sin(th)+c3*np.cos(th)
    be2, be3 = d2+b2, d2+d3+b3
    x = L2*np.cos(q2_)+rho3; z = L2*np.sin(q2_)+eta3
    k = lambda I,t: I[0,0]*np.sin(t)**2 + 2*I[0,2]*np.sin(t)*np.cos(t) + I[2,2]*np.cos(t)**2
    l = lambda I,t: -I[0,1]*np.sin(t) - I[1,2]*np.cos(t)
    s2, s3 = a2**2+c2**2, a3**2+c3**2
    f3 = a3*np.cos(q3_)-c3*np.sin(q3_)
    M = np.zeros((3,3))
    M[0,0] = J1+m2*(rho2**2+be2**2)+m3*(x**2+be3**2)+k(I2,q2_)+k(I3,th)
    M[0,1] = m2*be2*eta2+m3*be3*z+l(I2,q2_)+l(I3,th)
    M[0,2] = m3*be3*eta3+l(I3,th)
    M[1,1] = m2*s2+I2[1,1]+m3*(L2**2+s3+2*L2*f3)+I3[1,1]
    M[1,2] = m3*(s3+L2*f3)+I3[1,1]
    M[2,2] = m3*s3+I3[1,1]
    return M + np.triu(M,1).T

def Gfun(q):
    q2_ = q[1]; th = q[1]+q[2]
    rho2 = a2*np.cos(q2_)-c2*np.sin(q2_); rho3 = a3*np.cos(th)-c3*np.sin(th)
    return np.array([0, g*(m2*rho2+m3*(L2*np.cos(q2_)+rho3)), g*m3*rho3])

def Vfun(q, qd):  # Coriolis/centrifugos por simbolos de Christoffel
    e = 1e-6; dM = np.zeros((3,3,3))
    for k in range(3):
        d = np.zeros(3); d[k] = e
        dM[:,:,k] = (Mfun(q+d)-Mfun(q-d))/(2*e)      # dM[i,j,k] = dMij/dqk
    V = np.zeros(3)
    for i in range(3):
        for j in range(3):
            for k in range(3):
                V[i] += 0.5*(dM[i,j,k]+dM[i,k,j]-dM[j,k,i])*qd[j]*qd[k]
    return V

def f_planta(x, tau):
    q, qd = x[:3], x[3:]
    qdd = np.linalg.solve(Mfun(q), tau - Vfun(q,qd) - Gfun(q))
    return np.concatenate([qd, qdd])

# ---------------- Equilibrio y linealizacion ----------------
qe = np.array([30,30,-30])*rad                      # [PROV]
xe = np.concatenate([qe, np.zeros(3)]); tau_e = Gfun(qe)
assert np.abs(f_planta(xe, tau_e)).max() < 1e-10
Me = Mfun(qe); Mi = np.linalg.inv(Me); e = 1e-6
JG = np.array([(Gfun(qe+e*np.eye(3)[i])-Gfun(qe-e*np.eye(3)[i]))/(2*e) for i in range(3)]).T
A = np.block([[np.zeros((3,3)), np.eye(3)], [-Mi@JG, np.zeros((3,3))]])
B = np.vstack([np.zeros((3,3)), Mi]); C = np.hstack([np.eye(3), np.zeros((3,3))])

# ---------------- LQR ----------------
x_esc = np.array([10,10,10,30,30,30])*rad
u_esc = np.array([0.5,0.5,0.1])                     # [PROV] N*m
Q = np.diag(1/x_esc**2); R = np.diag(1/u_esc**2)
P = care(A,B,Q,R); F = np.linalg.solve(R, B.T@P)
# ---------------- Kalman (dual) ----------------
Qw = np.diag([0.05]*3)**2; Rv = np.diag([0.2*rad]*3)**2
Pk = care(A.T, C.T, B@Qw@B.T, Rv); L = np.linalg.solve(Rv, C@Pk).T
Ak = A-B@F-L@C
Alqg = np.block([[A, -B@F],[L@C, Ak]])
print("Polos planta   :", np.round(np.sort_complex(np.linalg.eigvals(A)),3))
print("Polos A-BF     :", np.round(np.sort_complex(np.linalg.eigvals(A-B@F)),3))
print("Polos A-LC     :", np.round(np.sort_complex(np.linalg.eigvals(A-L@C)),3))
print("Polos LQG (max Re):", np.linalg.eigvals(Alqg).real.max())
print("F =\n", np.round(F,4)); print("L =\n", np.round(L,3))

# ---------------- Simulacion de regulacion ----------------
x0 = np.array([4,-4,3,0,0,0])*rad                   # desviacion inicial real
xh0 = np.zeros(6)                                   # el observador arranca en el equilibrio
T = 4.0; t = np.linspace(0, T, 2000)

def lqg_nl(_, z):
    dx, dxh = z[:6], z[6:]
    du = -F@dxh
    y = C@dx
    xdot = f_planta(xe+dx, tau_e+du)
    xhdot = A@dxh + B@du + L@(y - C@dxh)
    return np.concatenate([xdot, xhdot])

sol = solve_ivp(lqg_nl, [0,T], np.concatenate([x0,xh0]), t_eval=t, rtol=1e-8, atol=1e-10, max_step=0.005)
x_nl, xh_nl = sol.y[:6].T, sol.y[6:].T
# LQG lineal
from scipy.signal import StateSpace, lsim
sys_lqg = StateSpace(Alqg, np.zeros((12,1)), np.eye(12), np.zeros((12,1)))
_, z_lin, _ = lsim(sys_lqg, np.zeros_like(t), t, X0=np.concatenate([x0,xh0]))
# LQR lineal ideal
sys_lqr = StateSpace(A-B@F, np.zeros((6,1)), np.eye(6), np.zeros((6,1)))
_, x_lqr, _ = lsim(sys_lqr, np.zeros_like(t), t, X0=x0)

q_nl  = (x_nl[:,:3]+qe)/rad; q_lin = (z_lin[:,:3]+qe)/rad; q_lqr = (x_lqr[:,:3]+qe)/rad
tau_nl = tau_e - xh_nl@F.T;  tau_lqr = tau_e - x_lqr@F.T
err_est = x_nl - xh_nl

print("\nError final de posicion [deg]:", np.round(x_nl[-1,:3]/rad,6))
print("Pico de torque total [N m]  :", np.round(np.abs(tau_nl).max(0),3))
print("Torque final [N m]          :", np.round(tau_nl[-1],4), " (tau_e =", np.round(tau_e,4), ")")
# tiempo de asentamiento 2 % (sobre la desviacion inicial) por articulacion
for i in range(3):
    dev = np.abs(x_nl[:,i]/rad); lim = 0.02*abs(x0[i]/rad)
    idx = np.where(dev > lim)[0]; ts = t[idx[-1]+1] if len(idx) else 0
    print(f"t_asentamiento 2% q{i+1}: {ts:.2f} s")

# ---------------- Graficas ----------------
col = ['tab:blue','tab:red','tab:green']
fig, ax = plt.subplots(3,2, figsize=(13,11)); fig.suptitle("Convergencia LQG - brazo 3 GDL (parametros provisionales)")
for i in range(3):
    ax[0,0].plot(t, q_nl[:,i]-qe[i]/rad, color=col[i], label=f"q{i+1} LQG no lineal")
    ax[0,0].plot(t, q_lqr[:,i]-qe[i]/rad, '--', color=col[i], alpha=.6, label=f"q{i+1} LQR ideal")
ax[0,0].axhline(0,color='k',lw=.8); ax[0,0].set(title="Error de posicion respecto al equilibrio", xlabel="t [s]", ylabel="[grados]")
ax[0,0].legend(fontsize=7, ncol=2)
for i in range(3): ax[0,1].plot(t, q_nl[:,i], color=col[i], label=f"q{i+1}"); ax[0,1].axhline(qe[i]/rad, color=col[i], ls=':')
ax[0,1].set(title="Posiciones absolutas (punteado = equilibrio)", xlabel="t [s]", ylabel="[grados]"); ax[0,1].legend()
for i in range(3):
    ax[1,0].plot(t, x_nl[:,3+i]/rad, color=col[i], label=f"dq{i+1} real")
    ax[1,0].plot(t, xh_nl[:,3+i]/rad, '--', color=col[i], label=f"dq{i+1} estimada")
ax[1,0].set(title="Velocidades reales y estimadas", xlabel="t [s]", ylabel="[grados/s]"); ax[1,0].legend(fontsize=7, ncol=2)
for i in range(3): ax[1,1].semilogy(t, np.abs(err_est[:,i])/rad+1e-12, color=col[i], label=f"|e q{i+1}|")
for i in range(3): ax[1,1].semilogy(t, np.abs(err_est[:,3+i])/rad+1e-12, '--', color=col[i], label=f"|e dq{i+1}|")
ax[1,1].set(title="Error de estimacion (escala log)", xlabel="t [s]", ylabel="[grados, grados/s]"); ax[1,1].legend(fontsize=7, ncol=2)
for i in range(3):
    ax[2,0].plot(t, tau_nl[:,i], color=col[i], label=f"tau{i+1} LQG")
    ax[2,0].axhline(tau_e[i], color=col[i], ls=':', lw=.8)
ax[2,0].set(title="Torque total (punteado = torque de sosten)", xlabel="t [s]", ylabel="[N m]"); ax[2,0].legend()
for i in range(3): ax[2,1].semilogy(t, np.abs(x_nl[:,i]/rad)+1e-12, color=col[i], label=f"|dq{i+1}|")
ax[2,1].set(title="Convergencia a cero (escala log)", xlabel="t [s]", ylabel="|desviacion| [grados]"); ax[2,1].legend()
for a in ax.ravel(): a.grid(True, alpha=.3)
plt.tight_layout(); plt.savefig("convergencia_lqg_3gdl.png", dpi=150); plt.show()