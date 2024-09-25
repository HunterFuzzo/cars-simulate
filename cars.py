import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class Voiture:
    def __init__(self, l, lar, h, Cx, Cz, masse, rho, Mu, am, alpha, g, r):
        self.l = l
        self.lar = lar
        self.h = h
        self.Cx = Cx
        self.Cz = Cz
        self.rho = rho
        self.masse = masse
        self.Mu = Mu
        self.am = am
        self.alpha = alpha
        self.g = g
        self.r = r

        self.surface = l * lar
        self.boost_enabled_pente = False
        self.boost_enabled_looping = False
        self.boost_enabled_finish = False
        self.aileron_enabled = False
        self.jupe_enabled = False

    def initialization(self):
        boost_location = input("Where do you want to enable boost? (pente/looping/finish, enter for nothing): ")
        if boost_location == 'pente':
            self.boost_enabled_pente = True
        elif boost_location == 'looping':
            self.boost_enabled_looping = True
        elif boost_location == 'finish':
            self.boost_enabled_finish = True
        else:
            pass

        ask_for_aileron = input("Do you want to enable aileron? (y/n, enter for nothing): ")
        if ask_for_aileron == 'y':
            self.update_params_for_aileron()

        ask_for_jupe = input("Do you want to enable jupe? (y/n, enter for nothing): ")
        if ask_for_jupe == 'y':
            self.update_params_for_jupe()

    def update_params_for_aileron(self):
        self.surface += 0.8
        self.masse += 30
        self.Cz *= 1.1
        self.aileron_enabled = True

    def update_params_for_jupe(self):
        self.masse += 15
        self.Cx *= 0.95
        self.jupe_enabled = True

    def system_pente(self, v, t):
        # Équation différentielle pour la pente
        if self.boost_enabled_pente:
            return (self.masse * self.g * np.sin(self.alpha) - self.Mu * self.g * np.cos(self.alpha) - 0.5 * self.rho * self.surface * self.Cx * v**2 + self.masse * self.am*1.3) / self.masse
        return (self.masse * self.g * np.sin(self.alpha) - self.Mu * self.g * np.cos(self.alpha) - 0.5 * self.rho * self.surface * self.Cx * v**2 + self.masse * self.am) / self.masse
    
    def system_looping(self, y, t):
        # Équation différentielle pour le looping
        v, theta = y
        if self.boost_enabled_looping:
            a_total = (-(0.5 * self.Cx * self.surface * self.rho * v**2) - (self.Mu * (self.masse * self.g * np.cos(theta) + self.masse * v**2 / self.r)) - (self.masse * self.g * np.sin(theta)) + self.masse * self.am*1.3) / self.masse
        a_total = (-(0.5 * self.Cx * self.surface * self.rho * v**2) - (self.Mu * (self.masse * self.g * np.cos(theta) + self.masse * v**2 / self.r)) - (self.masse * self.g * np.sin(theta)) + self.masse * self.am) / self.masse
        dtheta = (v / self.r)

        if self.aileron_enabled or self.jupe_enabled:
            a_total -= 0.5 * self.rho * self.surface * self.Cz * v**2 / self.masse  # Effet de l'aileron ou de la jupe

        return [a_total, dtheta]
    
    def systeme_ravin(self, x, t):
        # x contient [position_x, position_y, vitesse_x, vitesse_y]
        position_x, position_y, vitesse_x, vitesse_y = x
        
        v = np.sqrt(vitesse_x**2 + vitesse_y**2)
        
        dvx_dt = -0.5 * self.rho * self.surface * self.Cx * v * vitesse_x / self.masse
        dvy_dt = (self.masse * self.g - 0.5 * self.rho * self.surface * self.Cz * v * vitesse_y) / self.masse

        # Les équations différentielles pour la position sont simplement les composantes de vitesse
        dx_dt = vitesse_x
        dy_dt = vitesse_y

        return [dx_dt, dy_dt, dvx_dt, dvy_dt]

    def systeme_fin(self, v, t):
        if self.boost_enabled_finish:
            return (-self.Mu * self.g * self.g - 1/2 * self.rho * self.surface * self.Cx * v**2 + self.masse * self.am*1.3) / self.masse
        return (-self.Mu * self.g * self.g - 1/2 * self.rho * self.surface * self.Cx * v**2 + self.masse * self.am) / self.masse
    
    def simulate_looping(self):
        # Conditions initiales pour le looping
        v0_looping = velocity_at_31m_pente
        theta0_looping = 0
        initial_conditions_looping = [v0_looping, theta0_looping]

        # Pas de temps pour la simulation du looping
        times_looping = np.arange(0, 2.5, 0.001)

        # Résolution de l'équation différentielle pour le looping
        result_looping = odeint(self.system_looping, initial_conditions_looping, times_looping)

        # Vérification si le looping est complété
        looping_completed = False
        for step in range(len(times_looping)):
            if result_looping[step, 1] >= 2 * np.pi:
                time_to_complete_loop = times_looping[step]
                exit_speed = result_looping[step, 0]
                looping_completed = True
                break

        if looping_completed:
            print(f"Looping complété avec succès en {time_to_complete_loop:.2f} secondes.")
            print(f"Vitesse de sortie du looping : {exit_speed:.2f} m/s")

            # Vérification de la vitesse minimale requise
            # Vitesse minimale requise pour le looping
            vitesse_minimale_requise = np.sqrt(self.g * self.r)

            # Vérification de la vitesse minimale requise
            if exit_speed >= vitesse_minimale_requise:
                print(f"Vitesse minimale requise pour le looping : {vitesse_minimale_requise:.2f} m/s (réussi)")
            else:
                print(f"Vitesse minimale requise pour le looping : {vitesse_minimale_requise:.2f} m/s (échec)")


    def __repr__(self):
        return f"Voiture : {self.l} m, {self.lar} m, {self.h} m, {self.Cx} m, {self.Cz} m, {self.rho} kg/m3, {self.masse} kg, {self.Mu} Nm, {self.am} Nm, {self.alpha} rad, {self.g} m/s², {self.r} m"

# Création de l'objet Voiture
dodge = Voiture(5.28, 1.95, 1.35, 0.38, 0.30, 1760, 1.20, 0.1, 5.1, np.sin(0.0645609693), 9.81, 6)
toyota = Voiture(4.51, 1.81, 1.27, 0.29, 0.30, 1615, 1.20, 0.1, 5, np.sin(0.0645609693), 9.81, 6)
chevrolet = Voiture(4.72, 1.88, 1.30, 0.35, 0.30, 1498, 1.20, 0.1, 5.3, np.sin(0.0645609693), 9.81, 6)
mazda = Voiture(4.3, 1.75, 1.23, 0.28, 0.3, 1385, 1.20, 0.1, 5.2, np.sin(0.0645609693), 9.81, 6)
nissan = Voiture(4.6, 1.79, 1.36, 0.34, 0.3, 1540, 1.20, 0.1, 5.8, np.sin(0.0645609693), 9.81, 6)
mitsubishi = Voiture(4.51, 1.81, 1.48, 0.28, 0.3, 1600, 1.20, 0.1, 5, np.sin(0.0645609693), 9.81, 6)

choosed_car = input("Veuillez choisir une voiture : ")
# Initialisation de la Dodge avec boost, aileron et jupe

car = globals()[choosed_car]
car.initialization()

times_pente = np.linspace(0, 3.35, 1000)
initial_conditions_pente = [0]

# Résolution de l'équation différentielle pour la pente
result_pente = odeint(car.system_pente, initial_conditions_pente, times_pente)

# Trouver le temps nécessaire pour atteindre 31 mètres
time_to_31m_pente = np.interp(31, np.cumsum(result_pente[:, 0]) * (times_pente[1] - times_pente[0]), times_pente)
velocity_at_31m_pente = np.interp(time_to_31m_pente, times_pente, result_pente[:, 0])

# Affichage des résultats pour la pente
plt.plot(times_pente, result_pente[:, 0], label="Vitesse - Pente")
plt.xlabel("Temps")
plt.ylabel("Vitesse")
plt.legend()

print(f'31 mètres en {time_to_31m_pente:.2f}s pour une vitesse: {velocity_at_31m_pente:.2f}m/s')
plt.show()

# Conditions initiales pour le looping
v0_looping = velocity_at_31m_pente
theta0_looping = 0
initial_conditions_looping = [v0_looping, theta0_looping]

# Pas de temps pour la simulation du looping
times_looping = np.arange(0, 2.5, 0.001)

# Résolution de l'équation différentielle pour le looping
result_looping = odeint(car.system_looping, initial_conditions_looping, times_looping)

# Affichage des résultats pour le looping
plt.plot(times_looping, result_looping[:, 0], label="Vitesse - Looping")
plt.xlabel("Temps")
plt.ylabel("Vitesse")
plt.legend()

# Trouver la vitesse au bout de 2.5 secondes pour le looping
velocity_at_2_5s_looping = np.interp(2.5, times_looping, result_looping[:, 0])
print(f"Vitesse après 2.5 secondes pour le looping: {velocity_at_2_5s_looping:.2f} m/s")

plt.show()

# Calcule de la vitesse minimale pour le looping
car.simulate_looping()

# Conditions initiales pour la chute du ravin
initial_conditions_ravin = [0, -1, velocity_at_2_5s_looping, 0]
# Pas de temps pour la simulation de la chute du ravin
times_ravin = np.linspace(0, 0.5, 5000)  # Chute sur 9 mètres

# Résolution de l'équation différentielle pour la chute du ravin
result_ravin = odeint(car.systeme_ravin, initial_conditions_ravin, times_ravin)

# Obtenez les résultats pour la position
position_x = np.linspace(0, 10, len(times_ravin))  # Position en x sur 9 mètres
position_y = -result_ravin[:, 1]  # Composante y de la position (multiplication par -1)

# Trouver le temps nécessaire pour atteindre 9 mètres
time_to_9m_ravin = np.interp(9, np.cumsum(result_ravin[:, 0]) * (times_ravin[1] - times_ravin[0]), times_ravin)
velocity_at_9m_ravin = np.interp(time_to_9m_ravin, times_ravin, result_ravin[:, 2])

# Affichage des résultats pour la chute du ravin (position)
plt.figure(figsize=(8, 6))
plt.plot(position_x, position_y, label="Position - Ravin")
plt.axvline(x=position_x[np.argmax(position_y <= 0)], color='red', linestyle='--', label='Barre à 9m')  # Barre à 9m au bon emplacement
plt.xlabel("Position X (m)")
plt.ylabel("Position Y (m)")
plt.legend()
plt.title("Trajectoire de la voiture dans le ravin")
plt.grid(True)
plt.show()



# Conditions initiales pour la fin de piste
initial_conditions_fin = [velocity_at_9m_ravin]

# Pas de temps pour la simulation de la fin de piste
times_fin = np.linspace(0, 10, 1000)  # Ajustez la durée si nécessaire

# Résolution de l'équation différentielle pour la fin de piste
result_fin = odeint(car.systeme_fin, initial_conditions_fin, times_fin)

# Trouver le temps nécessaire pour atteindre 10 mètres
time_to_10m_fin = np.interp(10, np.cumsum(result_fin[:, 0]) * (times_fin[1] - times_fin[0]), times_fin)
velocity_at_10m_fin = np.interp(time_to_10m_fin, times_fin, result_fin[:, 0])

# Affichage des résultats pour la fin de piste
plt.plot(times_fin, result_fin[:, 0], label="Vitesse - Fin de piste")
plt.xlabel("Temps")
plt.ylabel("Vitesse")
plt.legend()

print(f'10 mètres en {time_to_10m_fin:.2f}s pour une vitesse: {velocity_at_10m_fin:.2f}m/s')
plt.show()
