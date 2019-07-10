# glaciere_chocolat
Code pour la glacière à chocolats


Le but c'est de conserver le (bon) chocolat le mieux possible. Le chocolat n'aime pas les fortes températures (ce qu'on à eu il ya quelques jours) ni non plus les basses: le conserver au frigo, même dans le bac à légumes le dénature. Trop chaud ou trop froid il reste tout à fait comestible mais perd son gout, ce qui est très dommage.

La température idéale est environ 15°.

J'ai donc acheté une glacière électrique portable, mais il n’y à aucun thermostat sur ces glacières, il faut donc en faire un. J'ai utilisé un Arduino, un capteur onwire DS18B20 pour la température, un afficheur 2 lignes et un émetteur 433Mhz pour envoyer les données sur Domoticz (format thermomètre Oregon).

La glacière originale fonctionne avec une alim 12V réglable par un potentiomètre (via un classique TL431) qui fait varier la tension de l'alim entre 12V et 0 (en réalité 3V). Le ventilateur est pris en parallèle sur le module, ce qui fait qu'il tourne plus ou moins vite en fonction du refroidissement voulu.

Une module Peltier se commande en tension, le PWM n'est pas adapté, d'ou je pense ce système.

J'ai donc remplacé le potentiomètre mécanique par un potentiomètre électronique commandé par l'Arduino. J'alimente le ventilateur en 9V ce qui fait moins de bruit, une meilleure durée de vie et je pense un meilleure refroidissement car ça ventile en permanence. Le ventilateur peut être arrêté par l'Arduino si besoin. J'aurais pu le commander en pwm mais je n'ai pas vu l’intérêt de faire varier sa vitesse, une ventilation "moyenne" fonctionne très bien.

Étant assez réfractaire aux pid's et à leurs réglages délicats, j'ai fait une régulation en "presque" tout ou rien. (C'est d'ailleurs en tout ou rien que sont aussi commandés les Peltier des tireuses de bière Beertender). La glacière refroidit à fond (12v) jusqu'à arriver presque à la consigne (15,1° pour moi) et arrivée à 2/10° de degré de part et d'autre de la consigne elle baisse légèrement (en gros 7V). Elle est coupée quand on est sous la valeur de la consigne. Ça ajoute un petit hystérésis (réglable) et évite de fluctuer trop rapidement. La température oscille d'environ 3/10° de degrés, c'est tout à fait correct et peu important en fait pour le chocolat. On pourrait je pense affiner encore mais c'est bien comme ça pour moi.

Au cas ou la température descendrait de quelques degrés sous les 15° (par ex la glacière dans une ambiance plus froide que 15°) je coupe le refroidissement ET le ventilateur.

On pourrait même inverser le Peltier pour chauffer mais dans mon cas je n'ai pas vu l’intérêt de rajouter (encore) une complication supplémentaire.

On pourrait je pense aussi simplifier en zapant le potentiomètre électronique et en commandant le Peltier en simple tout ou rien comme les Beertender mais moi j'aime bien pouvoir commander plus finement si je veux et régler la tension logiciellement.

Bien sur tous les paramètres de température sont réglables, la température, consigne, hystérésis sont affichés et la température est envoyée toutes les 2 minutes via 433Mhz.
