#BOCOUM AMADOU

function lire_map(fname::String)
    
    lines=open(fname, "r") do file
        readlines(file)
    end

    ind=findfirst(line->lowercase(strip(line))=="map",lines)
     if ind==nothing
        println("Erreur sur le fichier il est sans en-tete")
     end
    carte=lines[(ind+1):end]

    n=length(carte)
    #Trouver la longueur maximale parmi toutes les lignes
    m=maximum(length.(carte))
    M=Matrix{Char}(undef, n,m)
    for i in 1:n
        line=carte[i]
        for j in 1:m
            if j<=length(line)
                M[i,j]=line[j]
            else
                #Si la ligne est plus courte , on complete avec des espaces accessibles
                M[i,j]= '-'
            end
        end
    end
    return M
end
#Fonction qui calcul le cout de deplacement
# Les couts de S et W sont 5 et 8 et pour le . ona 1 tout autre caractere est consideré comme étant un obstacle
function cout_deplacement(cell::Char)
    if cell == 'S'
        return 5
    elseif cell == 'W'
        return 8
    else
        return 1 
    end
end

function get_neighbors(M::Matrix{Char}, position::Tuple{Int,Int})
    l,c=position
    voisins=Tuple{Int,Int}[]
    directions=[(0,1),(1,0),(0,-1),(-1,0)] #Les directions d'evolution
    for (dl,dc) in directions
      nl,nc=l+dl, c+dc
      if nl >= 1 && nl <= size(M,1) && nc >= 1 && nc <= size(M,2)
      #On verifie que la case est accessible(Donc on ne va ajouter que les cases accessibles dans la liste des voisins)
         if M[nl,nc] =='.' || M[nl,nc]=='S' || M[nl,nc]=='W'
            push!(voisins,(nl,nc))
         end
     end
    end
    return voisins
end
function reconstruire_path(parent::Dict,D::Tuple{Int,Int},A::Tuple{Int,Int})
    path=[A]
    current=A
    while current !=D
        current = parent[current]
        pushfirst!(path,current)
    end
    return path
end

#Une fonction qui permet d'afficher le chemin:
function afficher_chemin(path:: Vector{Tuple{Int,Int}})
    chemin= map(x->" ($(x[1]), $(x[2]))", path)
    pont= join(chemin, "->")
     return pont
end

#******************************************************************************************
function algobfs(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    println("Breadth-First-Search")
    grid=lire_map(fname)
    queue=[D]
    visited=Set([D])
    parent = Dict{Tuple{Int,Int},Tuple{Int,Int}}()#Pour pouvoir reconstruire le chemin parcouru(On associera donc à chaque case son parent)
    nb_evaluated=0 #Pour evaluer le nombre d'états
    
    while !isempty(queue)
        current=popfirst!(queue)
        nb_evaluated+=1
        if current==A
          path =reconstruire_path(parent,D,A)
          println("Distance D->A ", length(path)-1)
          println("Number of states evaluated:",nb_evaluated )
          print("chemin :")
          return afficher_chemin(path)
        end
        for nb in get_neighbors(grid,current)
            if !(nb in visited)
                push!(visited,nb)
                parent[nb]=current
                push!(queue,nb)
            end
        end
    end
    println("aucun chemin trouvé")
end

# Algorithme de Dijkstra::************************************

using DataStructures
 function algoDijkstra(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    println(" Dijkstra ")
    grid=lire_map(fname)
 
    #Initialisation des distances pour chaque position dans la grille
    dist=Dict{Tuple{Int,Int},Float64}()
for i in 1:size(grid,1)
    for j in 1:size(grid,2)
        dist[(i,j)]=Inf                  #Inf pour l'infini
 end
end
dist[D] = 0.0 # le cout pour atteindre la position de depart est de 0
 
#Dictionnaire pour pouvoir suivre les parents des differentes positions
parent = Dict{Tuple{Int,Int},Tuple{Int,Int}}()

#Initialisation de la dile de priorité avec la position de depart
pq=PriorityQueue{Tuple{Int,Int},Float64}()
enqueue!(pq,D,0.0)

visited = Set{Tuple{Int,Int}}()
nb_evaluated=0
while !isempty(pq)
    current= dequeue!(pq) 
    nb_evaluated +=1
    
    if current == A
        path=reconstruire_path(parent,D,A)
        println("Distance D->A ", dist[A])
        println("Number of states evaluated:",nb_evaluated )
        print("chemin : ")
        return afficher_chemin(path)
    end
    push!(visited,current)

    #Parcours des voisins
    for voisin in get_neighbors(grid,current)
        #Calcule du cout pour aller sur le voisin immediat
        cost = cout_deplacement(grid[voisin[1], voisin[2]])
        new_dist= dist[current]+cost # le cout pour atteindre current + le cout du mouvement pour aller sur son voisin
        if new_dist < dist[voisin]
            dist[voisin]=new_dist
            parent[voisin]=current
            enqueue!(pq, voisin, new_dist)
        end
    end
end
println("Aucun chemin trouvé avec Dijkstra")
    return []
end

#La heuristique de manhattan*************************************************
function heuristique(a::Tuple{Int,Int},b::Tuple{Int,Int})
    return abs(a[1]-b[1])+abs(a[2]-b[2])
end

#L'Algorithme A*****************************************************************
# 'g'  Le cout pour atteindre un sommet S  depuis le sommet de depart
# 'h' Une estimation du cout pour aller du sommet ou l'on est jusqu'à l'arrivée
# f=g+h  f le cout total pour atteundre l'arrivée et g(Du depart)=0 
# f(D)=g(D)+h(D)=0+h(D)=h(D)
 
function algoAstar(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    grid=lire_map(fname)

    #Initialisation des g et f pour chaque case

    g=Dict{Tuple{Int,Int}, Float64}()
    f=Dict{Tuple{Int,Int}, Float64}()
    for i in 1:size(grid,1)
        for j in 1:size(grid,2)
            g[(i,j)]=Inf # Tous les g (sauf celui du depart) sonnt initialisés à l'infin(Inf)
            f[(i,j)]=Inf #Tous les f (sauf celui du depart)
        end
    end
    g[D]=0.0
    # f(D)=g(D)+h(D)=0+h(D)=h(D)
    f[D]=heuristique(D,A)

    #Dictionnaire pour suivre chaque parent
    parent=Dict{Tuple{Int,Int}, Tuple{Int,Int}}()

    #PriorityQueue qui va utiliser f comme priorité
    pq = PriorityQueue{Tuple{Int,Int}, Float64}()
    enqueue!(pq,D,f[D])
    
    nb_evaluated=0
    visited=Set{Tuple{Int,Int}}()

    while !isempty(pq)
        current=dequeue!(pq)
        nb_evaluated +=1
        if current == A
         path=reconstruire_path(parent,D,A)
         println("Distance D->A ", g[A])
         println("Number of states evaluated:",nb_evaluated )
         print("chemin : ")
         return afficher_chemin(path)
        end
        push!(visited,current)
    
        for voisin in get_neighbors(grid,current)
            if voisin in visited
                continue
            end
            cost=cout_deplacement(grid[voisin[1],voisin[2]])
            calcul_g=g[current]+cost
            if calcul_g < g[voisin]
                parent[voisin] = current
                g[voisin] = calcul_g
                f[voisin] = calcul_g+heuristique(voisin,A) 
                if haskey(pq,voisin)
                    delete!(pq, voisin)
                end
                enqueue!(pq,voisin,f[voisin])
            end
        end
    end
    println("Aucun chemin trouvé avec A* ")
        return []
end

#Algorithme Glouton ************************************************************
function algoGlouton(fname::String, D::Tuple{Int,Int}, A::Tuple{Int,Int})
    grid=lire_map(fname)
    
    #PriorityQueue avec pour priorité la valeur Heuristique(heuristique de Manhattan)
   pq=PriorityQueue{Tuple{Int,Int}, Float64}()
   enqueue!(pq,D,heuristique(D,A))

   parent=Dict{Tuple{Int,Int},Tuple{Int,Int}}()
   visited=Set{Tuple{Int,Int}}()
   nb_eval=0
   while !isempty(pq)
    current=dequeue!(pq)
    nb_eval +=1
    if current == A
        path=reconstruire_path(parent,D,A)
        println("Distance D->A ", length(path)-1)
        println("Number of states evaluated:",nb_eval )
        print("chemin : ")
        return afficher_chemin(path)
    end
    push!(visited,current)

    for voisin in get_neighbors(grid,current)
        if voisin in visited
            continue
        end
        #On se basera uniquement sur l'estimation heuristique du cout restant jusqu'à l'arrivée
        h_val=heuristique(voisin,A)
        if haskey(pq,voisin)
            delete!(pq, voisin)
        end
        enqueue!(pq,voisin,h_val)
    parent[voisin]=current
    end
   end
   println("Aucun chemin trouvé avec le Glouton")
   return []
end

#Calcul de f pour les 3 cas de  l'algorthme wA*

function calcul_f(pos::Tuple{Int,Int}, gpos::Float64, A::Tuple{Int,Int}; mode::Int,w::Float64)
    hpos = heuristique(pos, A)
    if mode == 1 
     #f(n) = w*g(n) + (1-w)*h(n), 0 <= w <=1
     return w*gpos + (1 - w)*hpos
    else
     # mode=2 ou 3 : f(n) = g(n) + w*h(n), w >= 1
     return gpos + w*hpos
    end
end

#Une fonction w_dynamique pour avoir w dynamique pour le 3eme cas 
# Ici je choisi de diminuer w en fonction du nombre d'états évaluées

function w_dynamique(voisin::Tuple{Int,Int}, nb_eval::Int, w_init::Float64)
    # Plus le nombre d'états augmente plus w diminue(et w >= 1 )
    new_w= 1+(w_ini-1)*exp(-0.0001*nb_eval)
end

#L'algorithme A* Ponderé
function algo_wAstar(fname::String,D::Tuple{Int,Int},A::Tuple{Int,Int};mode::Int, w::Float64)
  
    #Lecture de la carte  
    grid = lire_map(fname)

    #  g et f 
    g = Dict{Tuple{Int,Int}, Float64}()
    f = Dict{Tuple{Int,Int}, Float64}()

    for i in 1:size(grid,1), j in 1:size(grid,2)
        g[(i,j)] = Inf
        f[(i,j)] = Inf
    end

    # Initialisation
    g[D] = 0.0
    # Calcul initial de f(D) selon la variante
    f[D] = calcul_f(D, g[D], A; mode=mode, w=w)

    # Dictionnaire pour suivre le parent de chaque position
    parent = Dict{Tuple{Int,Int}, Tuple{Int,Int}}()

    # PriorityQueue pour gérer l'ordre d'exploration (basé sur f)
    pq = PriorityQueue{Tuple{Int,Int}, Float64}()
    enqueue!(pq, D, f[D])

    visited = Set{Tuple{Int,Int}}()
    nb_evaluated = 0

    # Boucle principale
    while !isempty(pq)
        current = dequeue!(pq)
        nb_evaluated += 1

        # Si on a atteint l'arrivée
        if current == A
            path = reconstruire_path(parent, D, A)
            println("mode=$mode, w=$w (initial)")
            println("Distance D->A ", g[A])
            println("Number of states evaluated:",nb_evaluated )
            print("Chemin : ")
           return afficher_chemin(path)
        end

        push!(visited, current)

        for voisin in get_neighbors(grid, current)
            if voisin in visited
                continue
            end

            # Si mode=3, on recalcule w dynamiquement
            current_w = w
            if mode == 3
                current_w = w_dynamique(voisin, nb_evaluated, w)
            end

            # Calcul du coût pour atteindre 'voisin'
            cost = cout_deplacement(grid[voisin[1], voisin[2]])
            tentative_g = g[current] + cost

            if tentative_g < g[voisin]
                parent[voisin] = current
                g[voisin] = tentative_g
                f[voisin] = calcul_f(voisin, g[voisin], A; mode=mode, w=current_w)

                # Mise à jour dans la PQ
                if haskey(pq, voisin)
                    delete!(pq, voisin)
                end
                enqueue!(pq, voisin, f[voisin])
            end
        end
    end

    println("Aucun chemin trouvé (mode=$mode, w=$w).")
    return []
end
