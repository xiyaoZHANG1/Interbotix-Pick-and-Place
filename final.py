
import numpy as np
import time
import sys


sys.path.insert(0, "/home/b/interbotix_ws/install/interbotix_xs_modules/lib/python3.8/site-packages")

try:
    from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
    print("Succès : Module Interbotix importé !")
except ImportError:
    from interbotix_xs_modules.arm import InterbotixManipulatorXS
    print("Succès : Module Interbotix importé (chemin alternatif) !")

#en metres
L1 = 89.45
L2 = 100.0
Lm = 35.0
Lr = 105.95
L3 = 100.0
L4 = 109.0

#question 1
#configuration articulaire qa correspondant à la configuration de repos qdh = [0,0,0,0]
offsets = np.array([0, -np.pi/2+np.arccos(L2/Lr), np.pi/2-np.arccos(L2/Lr), 0])


#question 2
def qdh_to_qa(qdh):
    qa = np.array(qdh) - offsets
    qa = (qa + np.pi) % (2 * np.pi) - np.pi  # normalisation entre -pi et pi
    return qa

def qa_to_qdh(qa):
    qdh = np.array(qa) + offsets
    qdh = (qdh + np.pi) % (2 * np.pi) - np.pi  # normalisation entre -pi et pi
    return qdh

#question 3
def matrice_Transform_DH(qdh, L1, Lr, L3, L4):
    """
    Calcule la matrice de transformation homogène 0T4
    à partir des angles articulaires q = [q1, q2, q3, q4]
    
    Angles en radians
    Longueurs en mètres
    """
    q1, q2, q3, q4 = qdh

    # Trigonométrie
    C1 = np.cos(q1)
    S1 = np.sin(q1)

    C2 = np.cos(q2)
    S2 = np.sin(q2)

    C23 = np.cos(q2 + q3)
    S23 = np.sin(q2 + q3)

    C234 = np.cos(q2 + q3 + q4)
    S234 = np.sin(q2 + q3 + q4)

    # Terme commun
    R = Lr * C2 + L3 * C23 + L4 * C234

    # Matrice homogène
    T = np.array([
        [ C1*C234, -C1*S234, -S1,  C1*R ],
        [ S1*C234, -S1*S234,  C1,  S1*R ],
        [  -S234,    -C234,   0.0, - (Lr*S2 + L3*S23 + L4*S234) + L1 ],
        [   0.0,       0.0,   0.0,  1.0 ]
    ])

    return T




#question 4
def inverse_kinematics(px, py, pz, psi = np.pi/2,
                       L1=L1, Lr=Lr, L3=L3, L4=L4):    
    """
    Modèle Géométrique Inverse du Pincher X100
    Retourne toutes les solutions q = [q1, q2, q3, q4]
    """

    solutions = []

    # q1
    q1 = np.arctan2(py, px)

    # Variables intermédiaires
    U = px*np.cos(q1) + py*np.sin(q1) - L4*np.cos(psi)
    V = L1 - pz - L4*np.sin(psi)

    A = 2 * Lr * U
    B = 2 * Lr * V
    W = U**2 + V**2 + Lr**2 - L3**2

    D = A**2 + B**2 - W**2

    # Cas inatteignable
    if D < 0:
        return []  # aucune solution

    sqrtD = np.sqrt(D)

    eps = 1  # +1 ou -1 pour les deux configurations du coude
    q2 = np.arctan2(B*W - eps*A*sqrtD,
                    A*W + eps*B*sqrtD)

    # q3
    q3 = np.arctan2(
        -U*np.sin(q2) + V*np.cos(q2),
        U*np.cos(q2) + V*np.sin(q2) - Lr
    )

    # q4
    q4 = psi - q2 - q3

    solutions=[q1, q2, q3, q4]

    return solutions


# question 5
def init_pose(bot):
    print("Mise en position Home...")
    bot.arm.go_to_sleep_pose()
    bot.gripper.grasp()


def go_to_pose(bot, px, py, pz, psi, temp = 2.0):
    qd = inverse_kinematics(px, py, pz, psi)
    if not qd:
        print("Pose inatteignable")
        return False
    qa = qdh_to_qa(qd)
    bot.arm.set_joint_positions(qa, temp)
    time.sleep(temp+0.2)
    return True






def pick(bot, px=135, py=0, pz=10.3, psi=np.pi/2, temp=2.0):
    print("Mise en position Home...")
    init_pose(bot)

    print("Aller à la position de prise...")
    bot.gripper.release()
    go_to_pose(bot,px,py,pz+50,psi,temp)

    print("Descendre pour saisir l'objet...")
    go_to_pose(bot,px,py,pz,psi,temp)

    print("Saisir l'objet...")
    bot.gripper.grasp()
    time.sleep(1.0)

    print("Remonter avec l'objet...")
    go_to_pose(bot,px,py,pz+50,psi,temp)

    print("Déplacement côté")
    go_to_pose(bot,px,50.0, pz+50, psi, temp)

    print("Descendre.")
    go_to_pose(bot,px,50.0, pz, psi, temp)

    print("Lacher l'objet...")
    bot.gripper.release()
    time.sleep(1.0)

    print("retoure à la position Home...")
    init_pose(bot)





#question 7
def jacobian(q, Lr=Lr, L3=L3, L4=L4):
    """
    Jacobienne géométrique J (3x4)
    q = [q1, q2, q3, q4] (angles DH)
    """

    q1, q2, q3, q4 = q

    # Trigonométrie
    C1 = np.cos(q1)
    S1 = np.sin(q1)

    C2 = np.cos(q2)
    S2 = np.sin(q2)

    C23 = np.cos(q2 + q3)
    S23 = np.sin(q2 + q3)

    C234 = np.cos(q2 + q3 + q4)
    S234 = np.sin(q2 + q3 + q4)

    # Termes communs
    A = Lr*C2 + L3*C23 + L4*C234
    B = -Lr*S2 - L3*S23 - L4*S234
    C = -L3*S23 - L4*S234

    J = np.array([
        [ -A*S1,  B*C1,  C*C1, -L4*S234*C1 ],
        [  A*C1,  B*S1,  C*S1, -L4*S234*S1 ],
        [   0.0, -A,    -L3*C23 - L4*C234, -L4*C234 ]
    ])

    return J


#question 8
def fk_position(q, L1=L1, Lr=Lr, L3=L3, L4=L4):
    q1, q2, q3, q4 = q
    c1, s1 = np.cos(q1), np.sin(q1)

    c2,  s2  = np.cos(q2), np.sin(q2)
    c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)
    c234, s234 = np.cos(q2 + q3 + q4), np.sin(q2 + q3 + q4)

    r = Lr*c2 + L3*c23 + L4*c234

    px = r * c1
    py = r * s1
    pz = L1 - (Lr*s2 + L3*s23 + L4*s234)

    return np.array([px, py, pz])

def vary_configuration_without_moving_ee(
    q0,
    qnom=np.zeros(4),
    alpha=0.2,
    beta=0.05,
    tol=1e-5,
    max_iter=200,
    L1=L1, Lr=Lr, L3=L3, L4=L4
):
    q = np.array(q0, dtype=float)

    # La position cible est celle atteinte au départ (pose "verrouillée")
    x_target = fk_position(q, L1, Lr, L3, L4)

    for _ in range(max_iter):
        x_cur = fk_position(q, L1, Lr, L3, L4)
        e = x_target - x_cur  # erreur cartésienne (doit rester ~0)

        J = jacobian(q, Lr, L3, L4)
        J_pinv = np.linalg.pinv(J)

        # Projecteur dans le noyau : P = I - J#J
        P = np.eye(4) - J_pinv @ J

        # Equation (5)
        dq = alpha * (J_pinv @ e) + beta * (P @ (qnom - q))

        q_next = q + dq

        # Critère d'arrêt : position tenue + pas articulaire petit
        if np.linalg.norm(e) < tol and np.linalg.norm(dq) < 1e-6:
            q = q_next
            break

        q = q_next

    return q, x_target



#question 9
def go_to_best_pose(bot, px, py, pz, psi=np.pi/2, temp=2.0):
    q0 = inverse_kinematics(px, py, pz, psi)
    if not q0:
        print("Pose inatteignable")
        return
    q0 = np.array(q0, dtype=float)

    qnom = np.zeros(4)

    q_new, x_locked = vary_configuration_without_moving_ee(
        q0,
        qnom=qnom,
        alpha=0.2,
        beta=0.05,
        tol=1e-6,
        max_iter=300,
        L1=L1, Lr=Lr, L3=L3, L4=L4
    )
    
    qa_new = qdh_to_qa(q_new)
    bot.arm.set_joint_positions(qa_new, temp)
    time.sleep(temp + 0.2)
    






def pick_and_place(
    bot,
    pick_pos = (135.0, 0.0, 10.3),     # (px, py, pz)
    place_pos = (135.0, 50.0, 10.3),    # (px, py, pz)
    psi=np.pi/2,
    lift_height=50.0,  
    move_time=2.0
):
    px, py, pz = pick_pos
    px2, py2, pz2 = place_pos

    print("Début de la séquence de pick and place...")
    init_pose(bot)

    print("1.overture de la pince")
    bot.gripper.release()
    time.sleep(0.5)

    print("2. aller au-dessus de l'objet")
    go_to_best_pose(bot, px, py, pz + lift_height, psi, move_time)

    print("3. descendre vers l'objet")
    go_to_best_pose(bot, px, py, pz, psi, move_time)
   
    print("4. saisir l'objet")
    bot.gripper.grasp()
    time.sleep(1.0)

    print("5. remonter avec l'objet")
    go_to_best_pose(bot, px, py, pz + lift_height, psi, move_time)

    print("6. déplacer vers la position de dépôt")
    go_to_best_pose(bot, px2, py2, pz2 + lift_height, psi, move_time)

    print("7. descendre vers la position de dépôt")
    go_to_best_pose(bot, px2, py2, pz2, psi, move_time)

    print("8. lâcher l'objet")
    bot.gripper.release()
    time.sleep(1.0)

    print("9. remonter après la dépose")
    go_to_best_pose(bot, px2, py2, pz2 + lift_height, psi, move_time)

    print("Retour à la position Home...")
    init_pose(bot)









def main1():
    pick(
    bot=InterbotixManipulatorXS("px100", "arm", "gripper")
    )



def main2():
    # 1) 
    px, py, pz, psi = 135.0, 0.0, 10.3, np.pi/2

    # 2) q0 initial
    q0 = inverse_kinematics(px, py, pz, psi)
    if not q0:
        print("Pose inatteignable")
        return
    q0 = np.array(q0, dtype=float)

    # 3) 
    x0 = fk_position(q0, L1, Lr, L3, L4)

    # 4) 
    qnom = np.zeros(4)

    # 5) 
    q_new, x_locked = vary_configuration_without_moving_ee(
        q0,
        qnom=qnom,
        alpha=0.2,
        beta=0.05,
        tol=1e-6,
        max_iter=300,
        L1=L1, Lr=Lr, L3=L3, L4=L4
    )

    # 6) meme position mais avec differents q
    x_new = fk_position(q_new, L1, Lr, L3, L4)

    print("x_locked (mm) =", x_locked)
    print("x_new    (mm) =", x_new)
    print("||dx|| (mm)   =", np.linalg.norm(x_new - x_locked))

    print("q0   (rad) =", q0)
    print("q_new(rad) =", q_new)
    print("delta q    =", q_new - q0)


def main3():
    bot = InterbotixManipulatorXS("px100", "arm", "gripper")
    pick_and_place(
        bot,
        pick_pos = (135.0, 0.0, 10.3),     # (px, py, pz)
        place_pos = (135.0, 50.0, 10.3),    # (px, py, pz)
        psi=np.pi/2,
        lift_height=50.0,  
        move_time=2.0
    )

if __name__=='__main__':
    main3()
