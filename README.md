# Pathfinding en Julia

Une implémentation en Julia d'algorithmes classiques de recherche de chemin sur des cartes représentées sous forme de grilles.

Le projet lit des fichiers de cartes, modélise les obstacles et les coûts de terrain, puis compare plusieurs stratégies de recherche, de la recherche non informée à la recherche heuristique.

## Algorithmes implémentés

- Breadth-First Search
- Dijkstra
- A*
- Recherche gloutonne
- A* pondéré

Le projet contient aussi les fonctions nécessaires pour lire une carte, calculer les coûts de déplacement, récupérer les voisins valides, reconstruire un chemin et afficher la route obtenue.

## Modèle de carte

Les cartes sont chargées depuis le dossier `dat/`. Le lecteur ignore l'en-tête du fichier jusqu'à trouver la ligne `map`, puis construit une matrice de caractères.

Comportement des terrains :

- `@` : obstacle non traversable
- `.` : case traversable normale, coût `1`
- `S` : terrain plus lent, coût `5`
- `W` : terrain de type eau, coût `8`

Les déplacements se font dans 4 directions : haut, bas, gauche et droite.

## Structure du dépôt

```text
.
├── dat/
│   ├── Paris_2_1024.map
│   ├── Sydney_2_512.map
│   └── maze512-1-0.map
└── src/
    └── Path.jl
```

## Prérequis

- Julia
- `DataStructures.jl`

Installer le package nécessaire depuis le REPL Julia :

```julia
using Pkg
Pkg.add("DataStructures")
```

## Utilisation

Cloner le dépôt :

```bash
git clone https://github.com/Bocoum1/Pathfinding.git
cd Pathfinding
```

Ouvrir Julia et charger le fichier source :

```julia
include("src/Path.jl")
```

Lancer un algorithme sur l'une des cartes fournies :

```julia
start = (333, 14)
goal = (312, 15)
map_file = "dat/maze512-1-0.map"

algobfs(map_file, start, goal)
algoDijkstra(map_file, start, goal)
algoAstar(map_file, start, goal)
algoGlouton(map_file, start, goal)
```

Lancer A* pondéré :

```julia
algo_wAstar(map_file, start, goal; mode=2, w=1.5)
```

## Modes de A* pondéré

La fonction `algo_wAstar` accepte plusieurs stratégies de pondération via `mode` :

- `mode=1` : mélange pondéré entre le coût du chemin et l'heuristique
- `mode=2` : `f(n) = g(n) + w * h(n)`
- `mode=3` : poids dynamique qui diminue lorsque le nombre d'états évalués augmente

## Sortie attendue

Chaque algorithme affiche :

- l'algorithme utilisé
- la distance ou le coût du chemin
- le nombre d'états évalués
- le chemin reconstruit sous forme de coordonnées

## Notes

Ce projet met surtout l'accent sur la clarté algorithmique. Il ne contient pas encore d'interface graphique ni de visualisation des chemins.

## Prochaines améliorations

- Ajouter des tableaux de benchmark comparant les états évalués et les coûts obtenus.
- Ajouter une visualisation des chemins sur les cartes.
- Ajouter des tests pour le parsing des cartes et la reconstruction des chemins.
- Transformer le code en petit module Julia.
