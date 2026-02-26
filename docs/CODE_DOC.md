# Documentation du Code Source et Dépannage

Ce document est un guide de référence pour comprendre le rôle de chaque fichier source (`.cpp`) du projet. Il est particulièrement utile si vous devez modifier le code pour adapter le drone à de nouveaux composants, ou si vous rencontrez des problèmes en vol (vibrations, dérive, moteurs qui ne tournent pas, etc.).

---

## 1. Le Cœur du Système

### `main.cpp`
C'est le chef d'orchestre. Il ne fait pas de calculs lui-même, mais il organise le travail de tous les autres fichiers en utilisant les deux cœurs de l'ESP32 via FreeRTOS.
* **Fonction importante :** `setup()` (initialise tout) et les déclarations des tâches `imu_task` et `pid_task`.
* **Variables en cas de soucis :**
  * `TASK_IMU_FREQ` / `TASK_PID_FREQ` : Si l'ESP32 plante ou surchauffe, vérifiez que ces fréquences sont bien à 1000 Hz.
  * Les priorités des tâches FreeRTOS (ex: `configMAX_PRIORITIES - 1`). Ne les baissez pas, sinon le drone va s'écraser car il ratera des calculs.

### `pid.cpp`
C'est le "cerveau" de l'équilibre. Ce fichier compare ce que le pilote demande avec ce que les capteurs lisent, et calcule la force de redressement nécessaire.
* **Fonction importante :** `compute_pid()`. Elle calcule l'erreur et applique les mathématiques (Proportionnel, Intégral, Dérivé).
* **Variables en cas de soucis (LE PLUS IMPORTANT) :**
  * **Si le drone tremble/vibre très vite :** Le `Kp` (Proportionnel) ou le `Kd` (Dérivé) est trop élevé. Baissez-les.
  * **Si le drone est mou et met du temps à réagir :** Le `Kp` est trop faible.
  * **Si le drone dérive lentement d'un côté au fil du temps :** Le `Ki` (Intégral) est trop faible.
  * *Rappel :* Si vous changez la vitesse de la boucle (ex: passage de 1000 Hz à 500 Hz), vous **devez** multiplier le `Ki` par 0.5 et diviser le `Kd` par 0.5.

### `motors.cpp`
Ce fichier transforme les calculs de correction du `pid.cpp` en signaux électriques réels (PWM) envoyés aux ESC (contrôleurs de moteurs). Il gère aussi le mixage (quel moteur accélérer pour aller en avant).
* **Fonction importante :** `mix_motors()` (répartit la puissance sur les 4 hélices) et `update_motors_pwm()` (envoie le signal physique).
* **Variables en cas de soucis :**
  * `MIN_PULSE` (souvent 1000 µs) et `MAX_PULSE` (souvent 2000 µs) : Si vos moteurs ne démarrent pas ou n'atteignent pas leur puissance maximale, ces valeurs ne correspondent pas à vos ESC.
  * `IDLE_SPEED` : Vitesse de rotation des hélices quand le drone est armé mais avec les gaz à zéro. Si les moteurs broutent ou s'arrêtent en vol lors d'une chute, augmentez cette valeur.

### `radio.cpp`
Ce fichier écoute les signaux venant de la radiocommande du pilote.
* **Fonction importante :** `read_radio()` et les interruptions matérielles (`ISR`) qui mesurent la longueur des impulsions envoyées par le récepteur radio.
* **Variables en cas de soucis :**
  * `DEADBAND` (Zone morte) : Si le drone dérive tout seul alors que vous ne touchez pas aux manches, augmentez la zone morte (ex: passez de 3 à 5 µs) car les manches de votre radio ne reviennent pas parfaitement au centre.
  * L'ordre des canaux (`AETR`, `TAER`, etc.). Si le manche des gaz fait tourner le drone, modifiez l'affectation des variables de canaux.

---

## 2. Les Capteurs (Acquisition)

### `imu.cpp` (et `alt_imu.cpp`)
Ils gèrent la puce MPU6050 (ou équivalent). Ce fichier lit l'accéléromètre et le gyroscope pour savoir comment le drone est penché. C'est ici que se trouve le **Filtre Complémentaire**.
* **Fonction importante :** `read_imu()` et `calibrate_imu()`.
* **Variables en cas de soucis :**
  * `GYRO_WEIGHT` (0.9999) et `ACCEL_WEIGHT` (0.0001) : Si le drone est instable lors de gros coups de gaz, le poids de l'accéléromètre est peut-être trop fort (sensible aux vibrations).
  * Le temps de calibration au démarrage. S'il est trop court, le "zéro" sera faux.

### `lidar.cpp`
Gère le capteur laser (TF-Luna) qui regarde le sol pour le mode *Alt Hold* (Maintien d'altitude).
* **Fonction importante :** `read_lidar()`. Elle inclut une compensation trigonométrique (si le drone penche, la distance lue est plus longue que la vraie hauteur, la fonction corrige ça).
* **Variables en cas de soucis :**
  * L'adresse I2C du Lidar. S'il n'est pas détecté, vérifiez cette adresse.
  * `MAX_RELIABLE_DISTANCE` : Si le drone monte brutalement sans raison en mode Alt Hold, le lidar lit peut-être des données fausses hors de sa portée.

### `optical_flow.cpp`
Gère la petite caméra sous le drone qui regarde les motifs du sol pour empêcher le drone de dériver avec le vent (mode *Pos Hold*).
* **Fonction importante :** `read_flow()`.
* **Variables en cas de soucis :**
  * `FLOW_SCALING_FACTOR` : Si le drone surcompense et "danse" de gauche à droite, ce multiplicateur est trop élevé. S'il dérive avec le vent sans réagir, il est trop bas.
  * L'orientation du capteur (X et Y inversés selon comment vous avez collé la puce sur le drone).

---

## 3. Mathématiques et Fusions Avancées

### `yaw_fusion.cpp`
Gère un problème mathématique complexe : quand le drone tourne sur lui-même (Lacet/Yaw) alors qu'il est penché, ses axes Roulis et Tangage s'inversent virtuellement. Ce fichier applique une matrice de rotation pour corriger ça.
* **Fonction importante :** `transfer_reference_frame()`.
* **Variables en cas de soucis :** Si le drone fait une "cuillère" (perd l'équilibre) quand vous tournez violemment sur vous-même, vérifiez les signes (+ ou -) dans les équations de sinus/cosinus de ce fichier.

### `position_ekf.cpp` / `kalman.cpp`
*Note : Le filtre de Kalman classique a été désactivé pour l'assiette, mais ces fichiers peuvent contenir les mathématiques résiduelles ou être utilisés pour fusionner la position X/Y (Optical Flow).*
* **Fonction importante :** Les matrices de prédiction. Ne pas modifier sans de solides bases en algèbre linéaire.

---

## 4. Utilitaires et Télémétrie

### `telemetry.cpp`
Fait tourner le serveur Web Asynchrone sur le réseau Wi-Fi de l'ESP32 pour envoyer les données au tableau de bord.
* **Fonction importante :** `handle_data()` qui crée le fichier JSON.
* **Variables en cas de soucis :**
  * Le nom du Wi-Fi (SSID) et le mot de passe.
  * Si la télémétrie fige, vérifiez que vous ne demandez pas trop de variables à la fois (ce qui sature la mémoire `String` de l'ESP32).

### `esc_calibrate.cpp`
Fichier de maintenance. Il sert uniquement la toute première fois pour dire aux moteurs : "Ceci est le minimum, ceci est le maximum".
* **Fonction importante :** `calibrate_escs()`.
* **En cas de soucis :** Si un moteur démarre avant les autres, décommentez l'appel à ce fichier dans `main.cpp`, allumez avec les gaz à fond, attendez les bips, baissez les gaz, attendez les bips, puis redémarrez et re-commentez le fichier.

### `setup_wizard.cpp`
Gère l'assistant de première configuration (souvent via le port Série ou une page web dédiée) pour éviter de devoir recompiler le code à chaque fois qu'on veut changer un petit paramètre.