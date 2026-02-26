# Interface de Télémétrie et Tableau de Bord

Ce document explique comment utiliser la page web embarquée dans le drone. Cette page agit comme un tableau de bord (Dashboard) qui vous permet de surveiller la santé et le comportement du drone en temps réel depuis votre téléphone ou ordinateur.

---

## 1. Guide du Tableau de Bord (Ce que vous voyez à l'écran)

Une fois connecté à l'adresse IP du drone (ex: `http://192.168.4.1`), vous arrivez sur une interface visuelle divisée en plusieurs zones. Voici comment lire et utiliser ces informations :

### 🛩️ Zone "Attitude et Inclinaison" (Horizon Artificiel)
* **Ce qu'on y voit :** L'inclinaison exacte du drone calculée par le cerveau (ESP32). Vous y trouverez les valeurs de Roulis (gauche/droite), Tangage (avant/arrière) et Lacet (boussole).
* **Comment l'utiliser :** C'est indispensable pour vérifier si la calibration s'est bien passée. Si le drone est posé à plat sur une table, les valeurs de Roulis et Tangage doivent être très proches de `0.0°`. Si ce n'est pas le cas, redémarrez le drone sans le bouger.

### 🔋 Zone "Santé du Système" (Batterie et Vitesse)
* **Ce qu'on y voit :** La tension de la batterie (`vBat`) et le temps de cycle du code (`Loop Time`).
* **Comment l'utiliser :** * Surveillez la **Batterie** : Si elle chute en dessous d'un certain seuil (ex: 10.5V pour une Lipo 3S), il faut atterrir immédiatement.
  * Surveillez le **Loop Time** : Il doit rester stable autour de 1000 µs (ce qui correspond à 1000 Hz). S'il y a de gros pics, cela signifie que le processeur galère ou qu'une tâche bloque le système.

### 🎮 Zone "Radiocommande" (Canaux RC)
* **Ce qu'on y voit :** La position en direct des manches de votre radiocommande (Radiolink AT9S) sous forme de valeurs allant généralement de 1000 à 2000.
* **Comment l'utiliser :** Très utile pour vérifier que votre récepteur R6DS communique bien en SBUS avec le drone. Bougez vos manches : si les valeurs changent sur l'écran, la connexion est bonne. Vérifiez aussi que lorsque vos manches sont au centre (sans les toucher), les valeurs sont bien stabilisées autour de `1500`.

### 📡 Zone "Capteurs Avancés" (LiDAR & Caméra)
* **Ce qu'on y voit :** La distance lue par le laser pointé vers le sol (en centimètres) et les vitesses de dérive calculées par la caméra (Optical Flow).
* **Comment l'utiliser :** Soulevez le drone à la main : la valeur du LiDAR doit augmenter de manière fluide. Bougez le drone latéralement au-dessus d'un tapis : vous devriez voir les valeurs `Flow X` et `Flow Y` réagir pour indiquer le mouvement.

### ⚙️ Zone "Statut et Mode de Vol"
* **Ce qu'on y voit :** Si le drone est ARMED (hélices tournantes) ou SAFE (sécurité), et quel est le mode de vol actif choisi avec vos switchs (FLYING, ALT_HOLD, POS_HOLD, etc.).
* **Comment l'utiliser :** Avant de décoller, basculez vos switchs AUX1 et AUX2 et vérifiez sur l'écran que le mode de vol change correctement.

---

## 2. Architecture du Serveur (Pour les développeurs)

L'infrastructure réseau repose sur un serveur HTTP asynchrone déployé spécifiquement sur le Cœur 1 de l'ESP32. L'utilisation d'une pile réseau asynchrone garantit que le traitement des requêtes HTTP n'introduit aucun blocage sur la boucle de stabilisation cadencée à 1000 Hz.

## 3. Données Brutes : Point d'Accès `GET /data`

Pour ceux qui veulent créer leur propre application mobile ou récupérer les données dans un logiciel externe, le drone génère un fichier JSON à l'adresse `/data`.

### Dictionnaire des Variables JSON

Pour que la transmission radio soit ultra-rapide (pas de lag), les noms des variables ont été raccourcis. Voici leur signification pour le décodage :

| Nom court | Type | Unité | Correspondance dans le code |
|-----------|------|-------|-----------------------------|
| `ar` | Décimal | Degrés | Angle Roulis (Roll) |
| `ap` | Décimal | Degrés | Angle Tangage (Pitch) |
| `ay` | Décimal | Degrés | Angle Lacet (Yaw) |
| `r1` | Entier | µs | RC Canal 1 (Aileron/Roulis) |
| `r2` | Entier | µs | RC Canal 2 (Profondeur/Tangage) |
| `r3` | Entier | µs | RC Canal 3 (Gaz/Throttle) |
| `r4` | Entier | µs | RC Canal 4 (Direction/Yaw) |
| `vb` | Décimal | Volts | Tension de la batterie (V-Bat) |
| `lt` | Entier | µs | Vitesse d'exécution (Loop Time) |
| `lidar_dist`| Décimal| cm | Hauteur sol lue par le LiDAR |
| `flow_vx` | Décimal| cm/s | Vitesse de dérive latérale (Caméra) |
| `flow_vy` | Décimal| cm/s | Vitesse de dérive longitudinale (Caméra) |
| `pos_x` | Décimal| cm | Position X estimée |
| `pos_y` | Décimal| cm | Position Y estimée |
| `mode` | Entier | Code | Index du mode de vol (0 à 5) |