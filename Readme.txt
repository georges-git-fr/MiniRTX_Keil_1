
Le projet "MiniRTX" permet de mettre en pratique les différentes fonctions proposées par le RTOS RTX de Keil.

Ressources utilisées :
---------------------------

Hard :
* Carte d'évaluation STM32F4Discovery.

Logiciels :
* CubeMX V5.6.1
* Keil µVision V5.28.0.0 + dernier pack en cours

D'après plusieurs documents comparatifs, RTX serait plus performant que FreeRTos bien que celui-ci soit largement plus utilisé dans les systèmes embarqués.
Pour plus de détails, lire les deux documents suivants :
* Software vs Hardware Implementations for Real-Time Operating Systems de Nicoleta Cristina GAITAN et Ioan Ungurean
* Symmetry - Timing Comparison of the Real-Time Operating Systems for Small Microcontrollers.pdf de Ioan Ungurean

Création du projet :
-------------------------

La première étape est de partir d'un projet MDK-ARM généré par cubeMX. Ce dernier permet de configurer facilement le système et de construire un projet prêt à l'emploi.

Pour ce projet, on utilise en particulier le USART2 qui permet de transmettre et recevoir des messages via le terminal de votre choix.
L'utilisation de USART2 est toutefois facultative car on peut utiliser directement la fonction printf en debug (ce que l'on fait également dans le projet).
L'utilisateur peut choisir le UART/USART de son choix et même sélectionner tout périphérique avec lequel il pourra faire évoluer le projet (i2C, spi, etc..).

Pour plus de détails sur le processus de génération du projet complet, lire le document intitulé "Comment implémenter CMSIS RTX dans Keil µVision depuis un projet cubeMX.txt"

On peut également partir directement de Keil (from scratch) sans passer par CubeMX.
Pour plus de détails sur ce processus de génération du projet, lire le document intitulé "Comment implémenter CMSIS RTX dans Keil µVision from scratch.txt"

Le second processus (from scratch) est un peu plus simple à mettre en oeuvre mais si l'on désire ajouter des périphériques utilisant la librairie HAL,
ce sera plus délicat sans cubeMX mais plus formateur peut être.

Les fonctions mises en pratique dans ce projet :
-----------------------------------------------------------------

* Création de user Templates pour l'éditeur Keil (édition)

* Création et utilisation des fichiers scvd personnalisées (debug)
* printf (debug)
* ITM_SendChar (debug)
* Event Recorder (debug)
* Registre DWT->CYCCNT (mesure précise en debug)
* Stack Usage (debug)

* Sémaphore
* Mutex
* Signal
* Algorithme de "Rendez-vous" avec sémaphore
* Algorithme de "Turnstile / Barrier" avec sémaphore + mutex
* Synchronisation de threads avec sémaphore (Moi d'abord, Toi ensuite)
* Multiplex (plusieurs instances d'un thread)
* Passage de paramètre à un thread (simple entier ou structure plus complexe)
* Memory pool
* MessageQ
* MailQ
* Virtual Timer + callback + passage de parametre

* Programmation d'un USART de façon complète (HAL+DMA+Callback) ou simplifiée (registres)
* Génération de nombres aléatoires avec librairie tm_stm32f4_rng de Tilen Majerle

N'hésitez pas à aller sur le web pour rechercher des infos, on y trouve absolument toutes les réponses. J'ai récupéré un nombre considérable de docs sur les différents sujets :
- La programmation C
- Keil (configuration + édition + debug)
- CubeMX
- CMSIS
- HAL
- RTX
- la carte STM32F4Discovery (hard + soft)
- les algos liés au sémaphore + mutex
- etc…



