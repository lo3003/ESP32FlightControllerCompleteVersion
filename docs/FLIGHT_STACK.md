# Architecture Logicielle (Flight Stack)

Ce document explique comment le programme est découpé à l'intérieur de l'ESP32 pour garantir qu'aucun calcul ne soit mis en pause accidentellement.

---

## Séparation sur les Deux Cœurs

L'ESP32 possède deux processeurs (cœurs). Nous les utilisons pour séparer le travail critique du travail secondaire.

### Schéma de Fonctionnement

    ┌─────────────────────────────────────────────────────────────────────────┐
    │                          CŒUR 0 (Lecture des Capteurs)                  │
    │  Priorité : Très Haute                                                  │
    │                                                                         │
    │   ┌────────────────┐      ┌─────────────────┐      ┌────────────────┐   │
    │   │ Lecture IMU    │      │ Lecture LiDAR   │      │ Lecture Caméra │   │
    │   │ (1000 Hz)      │      │ (50 Hz)         │      │ (100 Hz)       │   │
    │   └──────┬─────────┘      └──────┬──────────┘      └──────┬─────────┘   │
    │          │                       │                        │             │
    │          └─────────── Verrouillage I2C (Mutex) ───────────┘             │
    └───────────────────────────────┬─────────────────────────────────────────┘
                                    │ (Copie rapide des données)
    ┌───────────────────────────────▼─────────────────────────────────────────┐
    │                          CŒUR 1 (Calculs et Moteurs)                    │
    │                                                                         │
    │   ┌─────────────────────────────────────────────────────────────────┐   │
    │   │                        BOUCLE PID (1000 Hz)                     │   │
    │   │  Compare la position voulue avec la position réelle             │   │
    │   └───────────────────────────┬─────────────────────────────────────┘   │
    │                               │                                         │
    │                               ▼                                         │
    │   ┌─────────────────────────────────────────────────────────────────┐   │
    │   │                      ENVOI AUX MOTEURS                          │   │
    │   │  Envoi du signal PWM aux ESC une fois sur quatre (250 Hz)       │   │
    │   └───────────────────────────┬─────────────────────────────────────┘   │
    │                               │                                         │
    │   ┌───────────────────────────▼─────────────────────────────────────┐   │
    │   │                      SERVEUR WEB (Télémétrie)                   │   │
    │   │  Priorité basse : envoie les données au PC ou smartphone        │   │
    │   └─────────────────────────────────────────────────────────────────┘   │
    └─────────────────────────────────────────────────────────────────────────┘

## Sécurité des Données (Thread Safety)

Comme plusieurs tâches s'exécutent en même temps, il faut éviter qu'elles n'essaient de lire ou d'écrire au même endroit en même temps :

1. **Verrou I2C (Mutex) :** Les trois capteurs (IMU, LiDAR, Caméra) utilisent le même câblage physique (bus I2C). Le code utilise un système de "jeton" (Mutex). Un capteur doit avoir le jeton pour parler, ce qui évite les collisions de données.
2. **Copie Rapide :** Quand le Cœur 0 envoie les angles au Cœur 1, il bloque temporairement le système pendant une fraction de milliseconde pour faire une copie propre. Cela garantit que le Cœur 1 ne lira jamais une donnée à moitié modifiée.
3. **Isolement du Réseau :** Le serveur web est sur le Cœur 1 mais avec une priorité très faible. Si le réseau capte mal et ralentit, cela ne ralentira jamais les hélices du drone.

## Calcul de l'Inclinaison

Le filtre de Kalman, trop complexe, a été remplacé par un Filtre Complémentaire.

1. **Le Gyroscope (Rapide) :** Il détecte les mouvements instantanés. On l'utilise à 99.99% car il est très réactif, mais il a tendance à dériver avec le temps.
2. **L'Accéléromètre (Lent) :** Il sait toujours où est le bas (la gravité). On l'utilise à 0.01% pour corriger doucement la dérive du gyroscope.

**Calcul exact dans le code :**
Inclinaison = (Mesure Gyroscope * 0.9999) + (Mesure Accéléromètre * 0.0001)