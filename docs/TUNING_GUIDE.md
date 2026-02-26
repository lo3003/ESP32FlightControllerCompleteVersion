# Guide de Réglage des PID (Tuning Guide)

Ce document explique comment ajuster les boucles de régulation PID (Proportionnel, Intégral, Dérivé) de votre contrôleur de vol. Un mauvais réglage peut rendre le drone incontrôlable, tandis qu'un bon réglage garantit un vol fluide, précis et verrouillé.

---

## Principes de Base du PID

Avant de modifier le code, il est crucial de comprendre l'action de chaque coefficient :

* **P (Proportionnel) - "Le Muscle" :** C'est la force de réaction immédiate. Plus l'erreur est grande (différence entre ce que vous voulez et la réalité), plus la force appliquée pour corriger est forte.
* **I (Intégral) - "La Mémoire" :** Il accumule les petites erreurs au fil du temps. Son rôle est de contrer les forces constantes comme le vent ou un léger déséquilibre du poids.
* **D (Dérivé) - "Le Frein" :** Il observe la vitesse à laquelle le drone se corrige. Son rôle est de freiner le mouvement juste avant d'atteindre la cible pour éviter de la dépasser (empêche le rebond).

---

## 1. PID Principal (Stabilisation / Autolevel)

C'est la boucle la plus critique, tournant à 1000 Hz. Elle contrôle l'assiette du drone (Roulis et Tangage) pour le maintenir à l'horizontale ou à l'angle demandé par le pilote.

### Comment régler selon le comportement en vol :

**1. Le coefficient P (Proportionnel)**
* **Si le P est trop HAUT :** Le drone vibre ou oscille très rapidement de gauche à droite ou d'avant en arrière, avec un bruit de moteur saccadé et aigu.
* **Si le P est trop BAS :** Le drone est "mou", il met du temps à réagir à vos commandes et semble glisser sur une plaque de verglas.
* *Méthode :* Augmentez le P progressivement jusqu'à voir de légères vibrations rapides, puis baissez-le de 10 à 20%.

**2. Le coefficient D (Dérivé)**
* **Si le D est trop HAUT :** Les moteurs deviennent brûlants très rapidement. Le drone présente des micro-tremblements nerveux et un son "granuleux".
* **Si le D est trop BAS :** Lorsque vous donnez un coup de manche brusque puis que vous relâchez, le drone dépasse l'horizontale, rebondit et oscille 2 ou 3 fois avant de se stabiliser.
* *Méthode :* Augmentez le D pour supprimer les rebonds après un mouvement sec. Gardez cette valeur la plus basse possible pour préserver les moteurs.

**3. Le coefficient I (Intégral)**
* **Si le I est trop BAS :** Le drone dérive lentement mais constamment dans une direction, surtout s'il y a une légère brise ou si la batterie est décalée de son centre.
* **Si le I est trop HAUT :** Le drone oscille de manière lente et ample (comme un bateau sur des vagues) et met du temps à revenir parfaitement à plat.

---

## 2. PID d'Altitude (Alt Hold via LiDAR)

Cette boucle fonctionne à une fréquence plus basse (ex: 50 Hz). Elle prend la hauteur mesurée par le LiDAR, la compare avec la hauteur voulue, et ajoute ou retire de la puissance globale (Gaz/Throttle) aux moteurs.

### Comment régler selon le comportement en vol :

**1. Le coefficient P (Poussée verticale)**
* **Si le P est trop HAUT :** Le drone donne des coups de gaz violents. Il fait des bonds saccadés de haut en bas très rapidement.
* **Si le P est trop BAS :** Le drone perd trop d'altitude quand vous avancez ou réagit trop lentement quand la consigne de hauteur change.

**2. Le coefficient D (Amortissement vertical)**
* **Si le D est trop BAS :** Le fameux "Effet Yoyo". Le drone monte vers sa cible, la dépasse (monte trop haut), coupe les gaz, descend trop bas, remet les gaz, etc. Il n'arrive pas à se stabiliser à la hauteur exacte.
* **Si le D est trop HAUT :** Les moteurs varient brutalement de régime à la moindre variation de la mesure du LiDAR (sensibilité au bruit du capteur).
* *Méthode :* Le D est crucial ici pour freiner le drone au moment où il s'approche de l'altitude cible.

**3. Le coefficient I (Compensation du poids)**
* **Son rôle :** Au fur et à mesure que la batterie se vide, ou si vous ajoutez une caméra, le drone a besoin de plus de puissance de base pour tenir en l'air. Le I va chercher automatiquement ce point d'équilibre. S'il est bien réglé, le drone maintient son altitude même sans vent.

---

## 3. PID de Position (Optical Flow)

Cette boucle utilise les données de vélocité de la caméra (Optical Flow) pour maintenir le drone sur un point fixe. Si le capteur détecte que le drone dérive vers la droite à 10 cm/s, la boucle PID va ordonner au PID Principal de pencher le drone vers la gauche pour freiner.

### Comment régler selon le comportement en vol :

**1. Le coefficient P (Angle de correction)**
* **Si le P est trop HAUT :** Le drone panique. S'il dérive légèrement, il va se pencher violemment pour compenser, ce qui va le faire partir trop vite dans l'autre sens. Résultat : de grandes oscillations agressives de gauche à droite.
* **Si le P est trop BAS :** Le drone dérive avec le vent et met beaucoup trop de temps à se pencher pour freiner sa course.

**2. Le coefficient I (Résistance au vent constant)**
* **Symptôme d'un mauvais réglage (Toilet Bowl Effect) :** Si le I ou le P est mal réglé par rapport au compas/lacet, le drone se met à tourner en rond autour de son point cible, en faisant des cercles de plus en plus grands (comme l'eau dans une cuvette).
* **Si le I est trop BAS :** S'il y a un vent constant d'est en ouest, le drone finira par dériver très lentement vers l'ouest car le P seul ne suffit pas à maintenir une inclinaison constante face au vent.

**3. Le coefficient D (Lissage de la consigne)**
* **Rôle :** L'Optical Flow peut envoyer des données un peu bruitées (selon la texture du sol). Le D permet de lisser la réaction du drone pour éviter que les moteurs ne donnent des à-coups à chaque micro-changement de pixel détecté par la caméra.

---

## Rappel Important : Règle de Fréquence

Comme mentionné dans la documentation de l'architecture, le temps d'exécution de la boucle (dt) est directement inclus dans les mathématiques du PID. 

**Si vous modifiez la fréquence de la tâche principale (ex: de 1000 Hz à 500 Hz), vos réglages PID actuels deviendront faux.**
Vous devrez appliquer cette règle :
* Gain Proportionnel (P) : Reste identique.
* Gain Intégral (I) : À multiplier par le ratio de la fréquence.
* Gain Dérivé (D) : À diviser par le ratio de la fréquence.