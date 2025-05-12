# Détection de plans 3D avec RANSAC

## Description
Ce projet implémente une version optimisée de l’algorithme RANSAC en C++ pour détecter plusieurs plans dans une scène 3D représentée au format `.obj`. Il permet d’identifier les `m` plans dominants d’un nuage de points et de colorier chaque plan détecté avec une couleur aléatoire.

## Fonctionnalités
- Chargement et sauvegarde de fichiers `.obj`
- Détection de plans par RANSAC
- Élimination des inliers proches du plan à chaque itération
- Génération de couleurs aléatoires pour visualisation
- Parallélisation avec OpenMP pour améliorer les performances

## Dépendances
- Eigen (bibliothèque linéaire)
- OpenMP (multi-threading)
- CMake

## Compilation
```bash
mkdir build
cd build
cmake ..
make
````

## Utilisation

```bash
./ransac fichier.obj
```

Un fichier `ransac.obj` sera généré avec les points colorés selon leur plan d’appartenance.

## Paramètres

Les principaux paramètres (modifiables dans `ransac.cpp`) :

* `m` : nombre d'itérations RANSAC
* `delta` : seuil de distance pour les inliers
* `numPlanes` : nombre de plans à détecter
* `angleThreshold` : angle maximal entre les normales
