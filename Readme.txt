
Le projet "MiniRTX" permet de mettre en pratique les diff�rentes fonctions propos�es par le RTOS RTX de Keil.

Ressources utilis�es :
---------------------------

Hard :
* Carte d'�valuation STM32F4Discovery.

Logiciels :
* CubeMX V5.6.1
* Keil �Vision V5.28.0.0 + dernier pack en cours

D'apr�s plusieurs documents comparatifs, RTX serait plus performant que FreeRTos bien que celui-ci soit largement plus utilis� dans les syst�mes embarqu�s.
Pour plus de d�tails, lire les deux documents suivants :
* Software vs Hardware Implementations for Real-Time Operating Systems de Nicoleta Cristina GAITAN et Ioan Ungurean
* Symmetry - Timing Comparison of the Real-Time Operating Systems for Small Microcontrollers.pdf de Ioan Ungurean

Cr�ation du projet :
-------------------------

La premi�re �tape est de partir d'un projet MDK-ARM g�n�r� par cubeMX. Ce dernier permet de configurer facilement le syst�me et de construire un projet pr�t � l'emploi.

Pour ce projet, on utilise en particulier le USART2 qui permet de transmettre et recevoir des messages via le terminal de votre choix.
L'utilisation de USART2 est toutefois facultative car on peut utiliser directement la fonction printf en debug (ce que l'on fait �galement dans le projet).
L'utilisateur peut choisir le UART/USART de son choix et m�me s�lectionner tout p�riph�rique avec lequel il pourra faire �voluer le projet (i2C, spi, etc..).

Pour plus de d�tails sur le processus de g�n�ration du projet complet, lire le document intitul� "Comment impl�menter CMSIS RTX dans Keil �Vision depuis un projet cubeMX.txt"

On peut �galement partir directement de Keil (from scratch) sans passer par CubeMX.
Pour plus de d�tails sur ce processus de g�n�ration du projet, lire le document intitul� "Comment impl�menter CMSIS RTX dans Keil �Vision from scratch.txt"

Le second processus (from scratch) est un peu plus simple � mettre en oeuvre mais si l'on d�sire ajouter des p�riph�riques utilisant la librairie HAL,
ce sera plus d�licat sans cubeMX mais plus formateur peut �tre.

Les fonctions mises en pratique dans ce projet :
-----------------------------------------------------------------

* Cr�ation de user Templates pour l'�diteur Keil (�dition)

* Cr�ation et utilisation des fichiers scvd personnalis�es (debug)
* printf (debug)
* ITM_SendChar (debug)
* Event Recorder (debug)
* Registre DWT->CYCCNT (mesure pr�cise en debug)
* Stack Usage (debug)

* S�maphore
* Mutex
* Signal
* Algorithme de "Rendez-vous" avec s�maphore
* Algorithme de "Turnstile / Barrier" avec s�maphore + mutex
* Synchronisation de threads avec s�maphore (Moi d'abord, Toi ensuite)
* Multiplex (plusieurs instances d'un thread)
* Passage de param�tre � un thread (simple entier ou structure plus complexe)
* Memory pool
* MessageQ
* MailQ
* Virtual Timer + callback + passage de parametre

* Programmation d'un USART de fa�on compl�te (HAL+DMA+Callback) ou simplifi�e (registres)
* G�n�ration de nombres al�atoires avec librairie tm_stm32f4_rng de Tilen Majerle

N'h�sitez pas � aller sur le web pour rechercher des infos, on y trouve absolument toutes les r�ponses. J'ai r�cup�r� un nombre consid�rable de docs sur les diff�rents sujets :
- La programmation C
- Keil (configuration + �dition + debug)
- CubeMX
- CMSIS
- HAL
- RTX
- la carte STM32F4Discovery (hard + soft)
- les algos li�s au s�maphore + mutex
- etc�

Le projet contient un sous-r�pertoire "Doc" dans lequel vous trouverez l'essentiel concernant les fonctionnalit�s du pr�sent projet.

Utilisation du projet :
---------------------------
Ouvrir le projet avec l'�diteur Keil.
A la fin du fichier main, on trouve toutes les taches en commentaire, sauf les 4 derni�res qui permettent de faire clignoter les 4 leds de la carte d'�valuation avec des tempos fixes ou al�atoires.
Il suffit de commenter/d�commenter les lignes de code pour "jongler" avec les diff�rentes fonctions � tester.

Amusez vous bien !




