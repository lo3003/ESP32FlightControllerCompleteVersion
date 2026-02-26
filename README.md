# Contrôleur de Vol ESP32

**Système de vol haute fréquence basé sur ESP32 et FreeRTOS**

Ce projet est un programme de contrôle de vol pour drone (quadricoptère). Il utilise les deux cœurs du processeur ESP32 pour séparer la lecture des capteurs et le contrôle des moteurs, ce qui permet d'obtenir un drone beaucoup plus réactif et stable.

---

## Architecture Matérielle (Hardware)

Le système repose sur la configuration matérielle spécifique suivante, centralisée autour de la puissance de calcul de l'ESP32. 

*[Insérer la photo de l'architecture matérielle ou du drone ici]*

* **Microcontrôleur (Le Cerveau) :** ESP32 FireBeetle (choisi pour son format compact, son processeur double cœur et sa connectivité Wi-Fi intégrée).
* **Centrale Inertielle (IMU) :** Module MPU6050 connecté en I2C, pour mesurer l'inclinaison (Gyroscope) et la gravité (Accéléromètre).
* **Magnétomètre / Altimètre :** Module Pololu AltIMU-10 connecté en I2C, utilisé pour le cap magnétique (boussole).
* **Radiocommande :** Radiolink AT9S.
* **Récepteur Radio :** Radiolink R6DS connecté via le protocole série ultra-rapide SBUS pour recevoir les ordres du pilote sans latence.
* **Moteurs :** Hawks Motor A2212 920KV (Moteurs Brushless).
* **Contrôleurs de Moteurs (ESC) :** ESC 30A standards recevant un signal PWM mis à jour à 250 Hz.
* **Capteurs de positionnement (Optionnels selon le mode) :**
  * LiDAR TF-Luna (Maintien d'altitude).
  * Capteur Optical Flow (Maintien de position horizontale).

## Fonctionnement Asynchrone (1000 Hz / 250 Hz)

Le code sépare le calcul de la stabilisation et l'envoi des commandes aux moteurs :

* **Le Cerveau (1000 Hz) :** La lecture des capteurs de mouvement (IMU) et les calculs de correction (PID) tournent très rapidement, à 1000 fois par seconde (1 ms).
* **Les Moteurs (250 Hz) :** Le signal envoyé aux contrôleurs des moteurs (ESC) est mis à jour 250 fois par seconde (4 ms). Le code utilise un diviseur pour ne pas surcharger les moteurs tout en gardant un calcul ultra-rapide en amont.

## Fonctionnalités Principales

* **FreeRTOS :** Utilisation d'un système d'exploitation temps réel pour répartir les tâches sans qu'elles se bloquent entre elles.
* **Filtre Complémentaire :** Un algorithme simple et très rapide qui mélange les données du gyroscope et de l'accéléromètre pour connaître l'inclinaison exacte du drone, sans dérive.
* **Calibration Automatique :** Le drone règle son point zéro (l'horizontale) tout seul au démarrage.

## Réglages et Sécurité

### Compensation de la Batterie
Quand la batterie se vide, elle donne moins de puissance. Le code détecte cette baisse de tension et augmente automatiquement la nervosité des commandes (les PID) pour que le drone garde le même comportement du début à la fin du vol.

### Adaptation des PID selon la fréquence
Si vous modifiez la vitesse de la boucle principale (actuellement 1000 Hz), vous devez ajuster les réglages PID :
* **P (Proportionnel) :** Ne change pas.
* **I (Intégral) :** À multiplier par le ratio de changement de fréquence.
* **D (Dérivé) :** À diviser par le ratio de changement de fréquence.

## Avancement du Projet

* [x] Utilisation des deux cœurs avec FreeRTOS
* [x] Boucle rapide à 1000 Hz et moteurs à 250 Hz
* [x] Nouveau filtre de stabilisation
* [x] Page web de télémétrie en direct
* [x] Mode Maintien d'altitude (LiDAR)
* [x] Mode Maintien de position (Optical Flow)
* [ ] Navigation automatique par points (Waypoints)