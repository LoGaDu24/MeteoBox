## MétéoBox

La MétéoBox est une petite station d’alerte météorologique par SMS qui fonctionne sans carte SIM grâce au wifi, pour intérieur ou extérieur. [Voir le code]([url](https://github.com/LoGaDu24/MeteoBox/blob/main/projet_innovez.ino)).

Elle dispose des fonctionnalité suivantes : alerte pluie (se basant sur la pression atmosphérique, avec une exactitude allant de 70% à 90%), alerte gel, alerte HI (Indice de chaleur, correspond à la canicule)[1] de trois paliers (canicule potentiellement dangereuse, dangereuse et très dangereuse), et une alerte qualité de l’air (CO2 principalement, qui peut donc potentiellement servir de détecteur de fumée ou encore éviter une atmosphère irrespirable dans un endroit clos).

La MétéoBox envoie les alertes par SMS. Pas besoin de carte SIM pour l’utiliser, il vous suffit d’un réseau wifi : elle utilise ainsi votre numéro de téléphone pour vous envoyer les SMS grâce à un lien[2] que vous pourrez trouver sur votre compte Free (je ne sais pas si les autres opérateurs donnent ce lien).

La MétéoBox se compose d’un ESP32 (WROOM-32), un capteur de  pression atmosphérique, de température et d’humidité de l’air BME280 et d’un capteur de qualité de l’air MQ-135[3]. 

![Organigrammes](https://github.com/LoGaDu24/MeteoBox/assets/161145804/76b358a3-65c5-4e88-b741-7dd54c618bc7)

Tout ceci à faible coût, pour un total d’environ 25 €, voire beaucoup moins si vous achetez sur AliExpress (moins de 10 €) !

[1] Calculé : c₁ + c₂T + c₃φ + c₄Tφ + c₅T² + c₆φ² + c₇T²φ + c₈Tφ² + c₉T²φ², où T exprime la température en °C et φ l'humidité relative de l'air en %

[2] De type :  `` https://smsapi.free-mobile.fr/sendmsg?user=xxxxxxxx&pass=xxxxxxxxxxxxxx&msg= ``

[3] Gaz détectés : le benzène (C6H6), l'ammoniaque (NH3), le sulfure, la fumée et «contaminants aéroportés»
