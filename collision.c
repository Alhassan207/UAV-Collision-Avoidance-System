/*
 
  SYSTÈME DE DÉTECTION DE COLLISION — ESSAIM UAV
  École des Sciences de l'Information
 Programmation Avancée en C — Projet Industriel
 
 # ARCHITECTURE GLOBALE :
   1. Allocation dynamique d'un bloc contigu (malloc) pour 10 000 drones
   2. Remplissage via arithmétique pure des pointeurs (ZÉRO crochet d'indexation)
  3. Tri par l'axe X avec qsort (O(n log n))
  4. Scan linéaire avec fenêtre glissante pour trouver la paire minimale (O(n))
 → Complexité globale : O(n log n)
 
 #CONTRAINTE STRICTE :
  L'opérateur [] est INTERDIT. Tout accès mémoire passe par *(ptr + offset).
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>   /* Pour FLT_MAX : la valeur flottante maximale */
#include <time.h>    /* Pour srand(time(NULL)) — graine aléatoire   */

/* 
#  SECTION 1 — STRUCTURE DE DONNÉE
 
  Un drone est un objet hétérogène : entier + trois flottants.
  Sa taille en mémoire est sizeof(struct Drone) = 16 octets
   1 int = 4 octets + 3 float = 12 octets).

 # En mémoire, un tableau de N drones ressemble à :
[id0|x0|y0|z0][id1|x1|y1|z1]...[idN-1|xN-1|yN-1|zN-1] ^-- drone 0 --^^-- drone 1 --^  ^------ drone N-1 ---^
 
 Chaque bloc fait sizeof(Drone) octets. L'arithmétique de pointeurs
 repose sur ce fait : (essaim + i) avance de i * sizeof(Drone) octets.
 */
typedef struct {
    int   id;   /* Identifiant unique du drone (0 à N-1)  */
    float x;    /* Coordonnée spatiale X (longitude simulée) */
    float y;    /* Coordonnée spatiale Y (latitude simulée)  */
    float z;    /* Coordonnée spatiale Z (altitude simulée)*/
} Drone;

/* 
 # SECTION 2 — CONSTANTES
 */
#define N          10000   /* Nombre de drones dans l'essaim  */
#define ESPACE_MAX 1000.0f /* Cube de simulation : [0, 1000]^3 mètres */
#define FENETRE    20      /* Taille de la fenêtre glissante (heuristique) */

/* 
 # SECTION 3 — FONCTIONS UTILITAIRES
  */

/*
 # distance_euclidienne_3d()
Calcule la distance entre deux drones dans l'espace 3D.
Formule : d = sqrt((x2-x1)² + (y2-y1)² + (z2-z1)²)
OPTIMISATION : on pourrait travailler avec le carré de la distance
pour éviter le sqrt() coûteux. Ici on le garde pour la clarté.
Paramètres : deux pointeurs vers des Drone (const = lecture seule)
Retour : flottant représentant la distance en mètres
 */
float distance_euclidienne_3d(const Drone *a, const Drone*b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/*
 comparateur_axe_x()
 # Fonction de comparaison pour qsort().
 
 qsort() attend une fonction de la forme :
  int cmp(const void *a, const void *b)
 qui retourne :
 < 0  si a doit venir AVANT b
 = 0  si a et b sont équivalents
 > 0  si a doit venir APRÈS b
 
 On caste les void* en Drone* pour accéder aux champs.
 Le tri se fait sur la coordonnée X : après le tri, les drones sont ordonnés du plus à gauche au plus à droite dans l'espace.
 
 @ POURQUOI TRIER SUR X ?
 Si la distance minimale est d, alors les deux drones candidats ne peuvent pas être séparés de plus de d sur l'axe X.
 Après tri, ils sont donc forcément proches dans le tableau.
 */
int comparateur_axe_x(const void *a, const void *b) {
    /* Cast void* → Drone* pour accéder au champ x */
    const Drone *da = (const Drone *)a;
    const Drone *db = (const Drone *)b;

    /* Comparaison robuste pour les flottants (évite le cast entier) */
    if (da->x < db->x) return -1;
    if (da->x > db->x) return  1;
    return 0;
}

/* 
 # SECTION 4 — GÉNÉRATION DES DRONES (via pointeurs) */

/*
 # generer_essaim()
 Remplit le tableau de drones avec des coordonnées aléatoires
 RÈGLE D'OR : ZÉRO crochet []. Tout accès via *(essaim + i).
 *(essaim + i) est STRICTEMENT ÉQUIVALENT à essaim[i] en C, mais nous utilisons uniquement la notation pointeur.
 La navigation fonctionne ainsi :
 - essaim           → adresse du 1er Drone
 - essaim + 1       → adresse du 2ème Drone (décalage de sizeof(Drone) octets)
 - essaim + i       → adresse du (i+1)ème Drone
 - *(essaim + i)    → valeur (le Drone lui-même)
 - (essaim + i)->x  → champ x du (i+1)ème Drone
 */
void generer_essaim(Drone *essaim, int n) {
    Drone *pointeur_courant = essaim; /* Pointeur qui va parcourir le bloc */

    for (int i = 0; i < n; i++) {
        /*
         (pointeur_courant)->id   : accès au champ id du drone courant
         Équivalent interdit : essaim[i].id
         rand() / RAND_MAX donne un float entre 0.0 et 1.0
         Multiplié par ESPACE_MAX → coordonnée dans [0, 1000]
         */
        pointeur_courant->id = i;
        pointeur_courant->x  = ((float)rand() / RAND_MAX) * ESPACE_MAX;
        pointeur_courant->y  = ((float)rand() / RAND_MAX) * ESPACE_MAX;
        pointeur_courant->z  = ((float)rand() / RAND_MAX) * ESPACE_MAX;

        pointeur_courant++; /* Avance d'un Drone en mémoire (sizeof(Drone) octets) */
    }
}

/* 
 # SECTION 5 — ALGORITHME PRINCIPAL : SCAN LINÉAIRE
 IDÉE CLÉE — Pourquoi le tri rend le problème linéaire ?
 Après tri par X, si on cherche la paire la plus proche,
 les deux drones les plus proches doivent avoir des coordonnées X
 proches. On peut donc ignorer les drones trop éloignés en X.
 On utilise une FENÊTRE GLISSANTE : pour chaque drone i, on ne compare qu'avec les drones [i+1 ... i+FENETRE].
 En pratique, FENETRE = 20 suffit largement pour 10 000 drones
 répartis uniformément.
 Complexité de cette phase : O(n × FENETRE) = O(n) car FENETRE est constant.

 find_pair_proche()
Paramètres :
essaim  : tableau trié de N drones
n : nombre de drones
idx_a, idx_b : (sortie) indices des deux drones les plus proches
 Retour : distance minimale trouvée
 */
float find_pair_proche(const Drone *essaim, int n, int *idx_a, int *idx_b) {
    float dist_min = FLT_MAX; /* Initialisation à +∞ (float maximum) */
    *idx_a = 0;
    *idx_b = 1;

    /*
     Boucle externe : drone de référence i
     Boucle interne : voisins j dans la fenêtre [i+1, i+FENETRE]
      Accès mémoire via pointeurs :
     (essaim + i)  →  pointeur vers le drone i
     (essaim + j)  →  pointeur vers le drone j
     */
    for (int i = 0; i < n - 1; i++) {
        /* Limite de la fenêtre : min(i + FENETRE, n-1) */
        int limite = i + FENETRE;
        if (limite >= n) limite = n - 1;

        for (int j = i + 1; j <= limite; j++) {
            /*
             Appel avec adresses des drones i et j
             Pas de crochet : on passe (essaim + i) au lieu de &essaim[i]
             */
            float d = distance_euclidienne_3d(essaim + i, essaim + j);

            if (d < dist_min) {
                dist_min = d;
                *idx_a   = i;
                *idx_b   = j;
            }
        }
    }
    return dist_min;
}

/*
 # SECTION 6 — PROGRAMME PRINCIPAL */
int main(void) {
    printf("=== Système de détection de collision UAV ===\n");
    printf("Nombre de drones : %d\n\n", N);

    /*  #ÉTAPE 1 : Allocation dynamique
     malloc(N * sizeof(Drone)) réserve un bloc CONTIGU de mémoire sur le tas (heap) pour N drones.
     Retourne un void* qu'on caste en Drone*.
     Si l'allocation échoue (RAM insuffisante), malloc retourne NULL.
     */
    Drone *essaim = (Drone *)malloc(N * sizeof(Drone));

    if (essaim == NULL) {
        fprintf(stderr, "ERREUR CRITIQUE : Allocation mémoire échouée.\n");
        return EXIT_FAILURE;
    }
printf("[OK] Allocation mémoire : %lu octets\n",
           (unsigned long)(N * sizeof(Drone)));

    /* 
     # ÉTAPE 2 : Génération des positions aléatoires
     srand() initialise le générateur pseudo-aléatoire avec
     l'heure courante → résultats différents à chaque exécution.
     */
    srand((unsigned int)time(NULL));
    generer_essaim(essaim, N);
    printf("[OK] %d drones générés aléatoirement\n", N);

    /* Aperçu des 3 premiers drones pour vérification (via pointeurs) */
    printf("\nAperçu (3 premiers drones avant tri) :\n");
    for (int i = 0; i < 3; i++) {
        printf("  Drone #%d : x=%.2f, y=%.2f, z=%.2f\n",
               (essaim + i)->id,
               (essaim + i)->x,
               (essaim + i)->y,
               (essaim + i)->z);
    }

    /* 
      # ÉTAPE 3 : Tri par coordonnée X
    
     qsort() est la fonction de tri standard de la bibliothèque C.
     Signature : qsort(base, nmemb, size, compar) base   : pointeur vers le début du tableau nmemb  : nombre d'éléments
     size   : taille d'un élément (sizeof(Drone))
     compar : pointeur vers la fonction de comparaison
     Algorithme interne : Quicksort → O(n log n) en moyenne.
     Après ce tri, essaim[0].x ≤ essaim[1].x ≤ ... ≤ essaim[N-1].x
     */
    printf("\n[..] Tri en cours.\n");
    qsort(essaim, N, sizeof(Drone), comparateur_axe_x);
    printf("[OK] Tri terminé (O(n log n))\n");

    /* Vérification du tri (3 premiers après tri) */
    printf("\nAperçu (3 premiers drones après tri par X) :\n");
    for (int i = 0; i < 3; i++) {
        printf("  Drone #%d : x=%.4f\n",
               (essaim + i)->id,
               (essaim + i)->x);
    }

    /* # ÉTAPE 4 : Détection de la paire critique
     Le scan linéaire avec fenêtre glissante cherche les deux drones les plus proches en O(n) grâce au pré-tri.
     */
    printf("\n[...] Recherche de la paire critique...\n");
    int idx_a, idx_b;
    float dist_min = find_pair_proche(essaim, N, &idx_a, &idx_b);

    /* 3 ÉTAPE 5 : Affichage du résultat et déclenchement d'alerte
     On accède aux drones par arithmétique de pointeurs :
     (essaim + idx_a)  →  adresse du drone A
     (essaim + idx_b)  →  adresse du drone B
     */
    const Drone *drone_a = essaim + idx_a;
    const Drone *drone_b = essaim + idx_b;

    printf("ALERTE DE COLLISION DÉTECTÉE\n"); 
    printf("Drone A  : ID=%-5d  x=%-8.2f\n", drone_a->id, drone_a->x);
    printf("y=%-8.2f  z=%-8.2f \n", drone_a->y, drone_a->z);
    printf(" Drone B : ID=%-5d  x=%-8.2f \n", drone_b->id, drone_b->x);
    printf("y=%-8.2f  z=%-8.2f\n", drone_b->y, drone_b->z);
    printf(" Distance : %-8.4f mètres \n", dist_min);
    printf("\n→ Manœuvre d'évitement déclenchée !\n");

    /* 
     # ÉTAPE 6 : Libération de la mémoire
     OBLIGATOIRE : free() restitue le bloc au système.
     Sans free(), on a une fuite mémoire (memory leak).
     Après free(), le pointeur est invalide → on le met à NULL.
     */
    free(essaim);
    essaim = NULL;
    printf("\n[OK] Mémoire libérée. Système prêt.\n");
    return EXIT_SUCCESS;
}
