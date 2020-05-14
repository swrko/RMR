#	SLOVENSKÁ TECHNICKÁ UNIVERZITA V BRATISLAVE FAKULTA ELEKTROTECHNIKY A INFORMATIKY
	
## Dokcumentacia 

## Riadenie Mobilných Robotov

<br>


**Letný semester 2019/2020**<br>
**Vypracoval: 		Filip Zúbek, Adam Gavula**

<br>
<br>
<br>

### Úloha č.1. Lokalizácia  a polohovanie robota v prostredí

Lokalizáciu mobilného robota typu Kobuki sme vykonali pomocou odometrie. Za týmto účelom bolo potrebné správne spracovať získané dáta z enkodérov a teda ošetriť pretekanie premenných. Z týchto dát  a informácii o robote sme vypočítali prejdenú vzdialenosť a uhol natočenia robota voči počiatočnej polohe robota v priestore (x,y  → 0,0). Pomocou vzťahov uvedených v zadaní pre odometriu sme z prejdenej vzdialenosti a uhla natočenia určili polohu robota X a Y v prostredí v ktorom sa robot nachádza. Pre lokalizáciu robota sme uvažovali dva prípady výpočtu polohy robota: 
**1.** -	Rýchlosť kolies  sa rovná 
**2.** -	Rýchlosť kolies  je rozdielna.
Pre každý z týchto prípadov sme počítali polohu pomocou iného vzťahu (viď zadanie). Následne sme zostrojili združený regulátor pre rýchlosť pohybu a uhol natočenia robota. Pre zostrojenie takéhoto regulátora bolo potrebné zaviesť možnosť zadávať cieľový bod (x,y súradnice bodu v priestore robota)  pomocou objektov „line edit“ v GUI. Ďalej implementovať funkciu pre výpočet uhla ktorý zviera aktuálna poloha robota a cieľový bod v priestore a funkciu pre výpočet euklidovskej vzdialenosti. Zostrojený regulátor sa natočí na cieľ tak aby bol zvieraný uhol s cieľom menší ako 45 stupňov. Po dosiahnutí tejto podmienky sa robot pohybuje do cieľa po oblúkoch a tak sa odchýlka uhla  doreguluje počas cesty. Rýchlosť akou sa robot pohybuje je úmerná vzdialenosti od cieľa a polomer oblúku po akom sa robot pohybuje je nepriamo úmerný uhlovej odchýlke.

### Úloha č.2. Navigácia

Navigáciou sa rozumie bezkolízny prechod prostredia bez toho aby prostredie poznal (mapa). Na takúto navigáciu využíva aktuálne údaje zo snímačov v našom prípade laserový diaľkomer (ďalej len lidar). Po zadaní cieľového bodu sa robot virtuálne natočí na tento bod a skontroluje dostupnosť cesty. Pokiaľ  v ceste nie je prekážka ide rovno na cieľ pričom kontroluje či sa prekážka nevyskytne počas cesty. V prípade, že cesta rovno do cieľa nie je možná ( v ceste je prekážka), hľadá krajné body  prekážky v nejakom rozsahu( vľavo a vprav od prekážky). Vypočíta  euklidovskú vzdialenosť do cieľa skrz tieto body a rozhodne sa pre menšiu. Pre zvolený krajný bod vypočíta  nové súradnice v bezpečnej vzdialenosti od kraja prekážky a zvolí ich ako medzi cieľ. Zapamätá si vzdialenosť do cieľa a pohybuje sa na medzi cieľ pomocou polohovania z úlohy 1. Ak je tento bod dosiahnutý celý algoritmus sa zopakuje. V prípade, že sa vzdiali od cieľa, robot sa prepne do režimu sledovania steny kde vyhľadá najbližší bod (stenu)  hľadá jej krajný bod  ktorý sa nastaví na cieľ po jeho dosiahnutí sa algoritmus zopakuje. Pokiaľ je vzdialenosť do cieľa kratšia ako posledná zapamätaná prepne sa opäť do režimu vyhýbania sa prekážok.

### Úloha č.3. Mapovanie

Úlohou mapovania je vytvoriť reprezentáciu prostredia, v ktorom sa robot nachádza. Táto mapa je vytvorená pomocou snímačov a informácií o polohe robota. V našom prípade je to laserový diaľkomer a využitie odometrie z úlohy č.1. Robot pri inicializácii vytvorí mriežkovú mapu o veľkosti 400x400  s hodnotou 0 (voľný priestor) pre každý jeden bod. Následne pri prechádzaní robota prostredím sa toto pole napĺňa na základe prepočtu dát z lidaru na svetové súradnice prostredia robota a následnou úpravou do mierky mapy. K získaným súradniciam je pridaný ofset ktorý je meniteľný a určuje bod 0,0 v mape. Body získané z lidaru sú prekážky prostredia a v mape sú zapísané hodnotou 1 (obsadený priestor). Získané údaje o prostredí sú z dôvodu indexovania zaokrúhlené a teda vzniká nepresnosť úmerná voči mierke s akou prevádzame súradnice z mapy do súradníc prostredia a naopak. S voliteľnou periódou sa získaná mapa zapisuje do textového súboru. Je implementovaná funkcia ktorá môže byť použitá a namiesto zápisu do textového súboru zapíše mapu vo formáte CSV. Implementovaná je funkcia čítania mapy z textového súboru ktorej vstupom je cesta k súboru. 

### Úloha č.4. Plánovanie trajektórie

Plánovanie trajektórie má za úlohu nájsť najkratšiu cestu v známom prostredí tak aby sa robot vyhol prekážkam. V prvom kroku je načítaná známa mapa prostredia z textového súboru pomocou implementovanej funkcie (argumentom je cesta k mape). V tomto momente sa mapa zabezpečí tým že sa umelo rozšíria prekážky v mape o bezpečnú vzdialenosť ( v niektorých prípadoch môžeme prísť o možné cesty, avšak priechod prostredím je bezpečnejší). Ďalej sa v  mape vyznačí bod aktuálnej pozície robota (123456) a zadaného cieľu (2). Následne sa aplikuje záplavový algoritmus pre 4-susednosť (princíp algoritmu je v zadaní). Následne sa hľadá cesta k cieľu podľa ohodnotených buniek pričom sa hľadajú zlomové body (tzv. uzly). Hľadá sa najkratšia cesta k cieľu s ohľadom na počet uzlov ( aby sa robot točil čo najmenej. Následne sú nájdené uzly vložené do FIFO zásobníka spolu s cieľom. Zásobník sa vyprázdňuje pokiaľ robot neprejdem všetkými bodmi a nedosiahne cieľ.

### Používateľská príručka

Pre všetky úlohy je potrebné spustiť simulátor robota ktorý sme obdržali so zadaním. Po spustení simulátora je nutné spustiť súbor DemoRMR.

- **1.**-Užívateľ si v GUI zadá cieľové súradnice v políčkach: „target x“ a „target y“. Následne je potrebné kliknúť na tlačidlo „go on!“. Keď robot dosiahne cieľovú polohu v prostredí automaticky zastaví. V prípade že chcem proces z nejakého dôvodu zastaviť môžeme kliknúť na tlačidlo „GENERAL STOP“. Robot nie je schopný sa vyhýbať prekážkam, preto treba cieľové súradnice  tomuto prispôsobiť.

- **2.** Užívateľ si zadá cieľovú súradnicu rovnako ako v predchádzajúcom prípade a klikne na tlačidlo „navigate“. Ak robot dosiahne svoj cieľ automaticky zastaví. V prípade že chcem proces z nejakého dôvodu zastaviť môžeme kliknúť na tlačidlo „GENERAL STOP“.

- **3.** Keď užívateľ klikne na tlačidlo „mapping“, robot začne zaznamenávať prostredie a priebežne ukladať do súboru. Robot nezačne sám prehľadávať prostredie je na užívateľovi aby robotovi dával povely a prehľadal mapu či už manuálne alebo pomocou úlohy č.1, č.2 . Treba brať v úvahu že časté natáčanie robota spôsobuje nepresnosť ktorá sa odrazí na mape.  Zapisovaná vzdialenosť bodov je do 1.5 (metra) od robota.

- **4.** Užívateľ si zadá cieľovú súradnicu rovnako ako v úlohe 1. a klikne na tlačidlo „map navigate“. Robot si automaticky nájde vhodnú trasu a poputuje do cieľa. Keď robot dosiahne cieľovú polohu v prostredí automaticky zastaví. V prípade že chcem proces z nejakého dôvodu zastaviť môžeme kliknúť na tlačidlo „GENERAL STOP“. 

