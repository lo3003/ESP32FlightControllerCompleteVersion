# Manuel d'Utilisation

Ce document explique comment démarrer, piloter, configurer votre radiocommande (Radiolink AT9S) et accéder aux données en direct du drone.

---

## Démarrage et Calibration (TRÈS IMPORTANT)

**ATTENTION : DÈS QUE VOUS BRANCHEZ LA BATTERIE, LE DRONE SE CALIBRE. IL DOIT ÊTRE POSÉ SUR UNE SURFACE BIEN PLATE ET NE DOIT PAS ÊTRE TOUCHÉ PENDANT LES 5 PREMIÈRES SECONDES.**

Le drone ne possède plus de réglages manuels pour l'horizontale. Il utilise ces 5 premières secondes pour comprendre où est la gravité et compenser les petits défauts de fabrication de la carte électronique. Si vous le bougez pendant ce temps, le drone pensera qu'il est penché et partira sur le côté au décollage.

---

## Accès à la Télémétrie en Direct (Serveur Web)

Le contrôleur ESP32 FireBeetle génère son propre réseau sans fil pour vous permettre de surveiller l'état du drone (inclinaison, batterie, capteurs) depuis votre téléphone ou votre ordinateur.

**Procédure de connexion :**
1. Allumez le drone en branchant la batterie.
2. Sur votre smartphone ou ordinateur, ouvrez la recherche de réseaux Wi-Fi.
3. Connectez-vous au réseau Wi-Fi émis par le drone (par défaut : `ESP32_FlightController` ou le nom configuré dans le code). Le mot de passe par défaut est souvent `12345678` (à vérifier dans votre configuration).
4. Ouvrez votre navigateur web (Chrome, Safari, Firefox).
5. Tapez l'adresse IP du drone dans la barre d'adresse : `http://192.168.4.1`
6. La page de télémétrie s'affiche et se met à jour en temps réel. Cette interface ne ralentit pas le vol grâce à l'architecture asynchrone du code.

---

## Les Modes de Vol et Configuration des Switchs

Vous pouvez changer le comportement du drone en vol à l'aide de deux interrupteurs (switchs) situés sur votre radiocommande Radiolink AT9S, affectés aux canaux AUX1 et AUX2.

*[Insérer photo de la radiocommande Radiolink AT9S avec les switchs AUX1 et AUX2 entourés ici]*

Voici comment positionner physiquement vos interrupteurs pour activer chaque mode :

### 1. Mode Manuel (MODE_FLYING)
* **Position :** AUX1 en bas / AUX2 en bas
* **Fonctionnement :** C'est le mode classique. Le drone se remet à plat quand vous lâchez le manche de droite. C'est vous qui gérez la hauteur manuellement avec le manche des gaz (à gauche).

### 2. Maintien d'Altitude (MODE_ALT_HOLD)
* **Position :** AUX1 en haut / AUX2 en bas
* **Fonctionnement :** Le capteur LiDAR prend le relais sur la hauteur. Le manche des gaz ne contrôle plus la puissance directe des moteurs : si vous le mettez au milieu, le drone garde sa hauteur actuelle. Si vous le montez, il grimpe, si vous le baissez, il descend.

### 3. Maintien sur Place (MODE_POS_HOLD)
* **Position :** AUX1 en bas / AUX2 en haut
* **Fonctionnement :** La caméra optique sous le drone regarde le sol. Si le vent pousse le drone, il s'inclinera tout seul pour contrer le vent et rester au même endroit. Vous devez toujours gérer la hauteur manuellement avec les gaz.

### 4. Pilote Automatique (MODE_AUTO_POS)
* **Position :** AUX1 en haut / AUX2 en haut
* **Fonctionnement :** Combinaison de l'altitude et de la position. Vous pouvez lâcher la radiocommande, le drone restera totalement immobile en l'air (hauteur et position fixes).

---

## Comment Armer et Couper les Moteurs

Par mesure de sécurité, les hélices ne tournent pas au branchement.

### Allumer les moteurs (Armement)
1. Baissez le manche des gaz au minimum.
2. Poussez le manche de direction (Yaw / lacet) complètement à **gauche**.
3. Maintenez cette position pendant **1 seconde**. Les moteurs vont commencer à tourner au ralenti.

### Éteindre les moteurs (Désarmement / Atterrissage)
1. Baissez le manche des gaz au minimum.
2. Poussez le manche de direction complètement à **droite**.
3. Maintenez pendant **1 seconde**. Les hélices s'arrêtent.

### Procédure d'Urgence
Si le drone a un problème en vol ou va percuter un obstacle, faites la procédure pour éteindre les moteurs (Gaz en bas, direction à droite). Cela coupe instantanément le courant reçu par le récepteur R6DS, le drone tombera, mais cela évite des dommages plus graves ou des blessures.

Si la radiocommande AT9S est éteinte ou perd le signal pendant plus de 60 secondes, le drone coupera ses moteurs automatiquement par sécurité logicielle.