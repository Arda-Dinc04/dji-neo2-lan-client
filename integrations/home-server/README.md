# Home Server : contrôle du Neo 2 depuis le cloud

Cette intégration ajoute le contrôle PC à la console Home Server existante.
L'ancienne page HLS est conservée sous `/home-server/drone/video` ; la page
`/home-server/drone` affiche l'interface de contrôle partagée avec le PC.

Cette copie publique fournit les modèles d'une intégration antérieure.
`example.com`, les adresses de compte et les chemins opérateur sont des
exemples anonymisés ; aucun service de démonstration n'est annoncé à cette
adresse. L'outil d'application des sources n'installe pas les services PC.
Les empreintes de `baseline.json` concernent les sources Home Server
d'origine, qui ne sont pas distribuées ici : ce paquet ne constitue pas une
application Home Server autonome.

Le panneau de parcours JSON charge le programme avant le vol, exige le
maintien d'Espace pendant l'interaction et affiche les étapes. Le code de
référence est dans [l'interface](../../src/neo2_control/web/flight.js).
Les limites des essais et les vérifications de la copie publique sont dans
[le bilan](../../docs/VALIDATION.md) et
[les vérifications de publication](../../docs/RELEASE-VALIDATION.md).

## Appliquer aux sources

Depuis neo2-control, avec une copie des sources Home Server :

```sh
python tools/apply_home_server_control.py /chemin/home-server --check
python tools/apply_home_server_control.py /chemin/home-server
```

`baseline.json` fixe les empreintes des fichiers de départ. Toute modification
inconnue, collision avec un nouveau fichier ou chemin symbolique est refusé
avant les écritures. L'état `.neo2-control-integration.json` permet ensuite
les mises à jour du paquet. Les secrets `.env` ne sont ni copiés ni modifiés.
Les quatre ressources web et le modèle HTML proviennent du même checkout
neo2-control que le contrôleur PC.

Dans les sources obtenues, Node 24.15.0 et le verrou npm existant permettent :

```sh
npm ci --ignore-scripts --no-audit --no-fund
npm test
npm run check
npm run build --workspace @manaty/home-server-web
```

## Transport et authentification

Le PC conserve son serveur HTTP sur `127.0.0.1:8794`. Un tunnel SSH sortant
doit exposer ce port **uniquement** par le socket Unix privé
`/run/neo2-control/flight.sock` sur le cloud. Le répertoire doit appartenir au
compte qui crée le tunnel et ne pas être accessible aux autres comptes ; le
socket doit être lisible/connectable uniquement par le service autorisé.
La configuration compose fournie suppose le déploiement actuel dont l'API
s'exécute comme root, et monte ce répertoire en lecture seule.

`NEO2_CONTROL_ENABLED` vaut `false` par défaut. Le passer à `true` exige un
contrôleur PC opérationnel et un tunnel vérifié. Le paquet ne crée pas ces
services, ne touche pas au Wi-Fi et ne démarre pas de connexion drone.
Une activation et un déploiement doivent être accompagnés d'une sauvegarde
des sources/images existantes et d'une vérification de la configuration
réelle du serveur ; le paquet n'embarque aucun secret de production.

Le navigateur présente son JWT Home Server. L'API vérifie le même
administrateur que la console existante, puis relaie la requête vers le PC.
Le jeton local du PC reste côté serveur. Le cache de principal vérifié dure
au plus cinq secondes, respecte l'expiration JWT, actualise sa vérification
en arrière-plan et borne les validations en cours.

L'API transmet les octets POST d'origine, sans réécriture, redirection ni
réessai. Le PC garde la validation JSON stricte, les tickets et la décision
finale. Une seule mutation normale peut être en cours ; LAND peut la
dépasser, ce qui n'annule pas une commande déjà reçue. Les corps, réponses,
requêtes simultanées et délais sont bornés.

## Latence

La passerelle cible `/api/remote-flight/*`, distinct du protocole local
`/api/flight/*` à échéance 250 ms. Un ticket WAN autorise une cible désirée
pendant `min(2 s, max(0,5 s, 2 × RTT HTTP + 0,4 s))`. Sans mesure initiale,
la durée vaut deux secondes. Le navigateur retient le maximum des huit
derniers RTT HTTP. ICMP seul ne mesure pas les relais de la requête complète.

Le PC garde une seule cible, remplacée par la suivante, et produit localement
les commandes à validité de 200 ms au maximum. Chaque commande est aussi
bornée par l'échéance d'origine de sa cible. La réponse fournit le ticket
suivant pour éviter un GET entre deux POST ; ce ticket ne renouvelle aucun
bail et ne permet pas de rejouer la mutation précédente.

Le watchdog, les limites de vol, la télémétrie PC et les confirmations
existantes restent actifs. Une cible expirée ou une perte du parent provoque
une demande LAND locale. Une demande LAND vide le tampon. Le maintien
réseau peut donc retarder la détection d'une perte d'opérateur jusqu'à deux
secondes ; la latence et cette limite ne garantissent pas l'arrêt physique.

L'interface conserve les âges mesurés et leur ajoute le temps de requête.
Son seuil d'interaction WAN ajoute au plus le RTT mesuré, avec une borne
de deux secondes ; le PC continue d'exiger ses données locales à 500 ms.
L'image doit toujours respecter les limites existantes. Une confirmation
explicite de fin de descente ou de sol ne peut pas être créée par le tampon.

## Preuves et limites

Les tests couvrent le vrai HTTP Nest avec son guard, le socket Unix, les
octets JSON, les tickets locaux/WAN distincts, le processus propriétaire
indépendant, la chaîne de commandes UDP sur loopback, les échéances et les
priorités LAND. Le 18 octobre, un cycle simulé a aussi traversé le vrai
serveur et le tunnel SSH avec des POST de 582 à 615 ms, sans défaut final.
Les journaux privés distinguent explicitement le backend synthétique.

Cela valide le transport et l'exécution logicielle. Les contrôles de
production confirment aussi le refus des accès non authentifiés, la version
des ressources web et la lecture de l'état PC depuis le conteneur API. Le
parcours depuis une session utilisateur connectée, les translations réelles,
un tour de 360° et le suivi physique d'un parcours restent à valider
ultérieurement avec l'opérateur et le drone.
