****PROJET DE Pathfinding en Julia*****


Ce projet implémente quatre algorithmes de recherche de chemin sur une grille(carte) lue depuis des fichiers texte :

.)Breadth-First Search(BFS)

.)Dijkstra

.)A*

.)Glouton

La carte est représentée par une matrice de caractères. Les obstacles sont indiqués par '@' et les cases traversables par d'autres caractères (par exemple, '-',S ou W).

***********************Structure du depot:*******************************************

**src/**::

Contient le code source avec:
*)lire_map(fname::String) qui lit le fichier de carte en ignorant l'en-ête et construit la matrice.

*)cout_deplacement(cell::Char) qui definit le cout de passage selon chaque caractere

*)get_neigbhors(M::Matrix{Char}, position::Tuple{Int,Int}) Qui renvoie la liste des voisins accessibles

*)Les implémentations des algorithmes : algobfs, algoDijkstra, algoAstar, algoGlouton.

*)La fonction reconstruire_path qui remonte le chemin à partir du dictionnaire des parents

  ***dat/**

Contient les fichiers de données (cartes) au format texte.

****Packages Utilisés : **
Datastructures(Pour la file de priorité(PriorityQueue)**

****************************************Installation et Execution*****************************************


Cloner le dépôt

Dans un terminal, exécuter :

    git clone https://github.com/Bocoum1/Pathfinding.git
    
    cd Pathfinding

Dans le REPL de Julia, executer 

 using Pkg

 Pkg.instantiate()

Puis executer les algorithmes

 include("src/Path.jl")
 
 Puis, pour tester chaque algorithme
# Exemple avec un fichier de carte dans le dossier dat/

 algobfs("dat/maze512-1-0.map",(333,14),(312,15))
 
 algoDijkstra("dat/maze512-1-0.map",(333,14),(312,15))
 
 algoAstar("dat/maze512-1-0.map",(333,14),(312,15))
 
 algoGlouton("dat/maze512-1-0.map",(333,14),(312,15))

*********Gestion des Obstacles*******

Les obstacles sont marqués par '@' et ne sont pas accessibles.

La fonction cout_deplacement attribue des coûts spécifiques aux cases contenant 'S' (coût 5) et 'W' (coût 8), et un coût de 1 pour les autres cases traversables.



    


