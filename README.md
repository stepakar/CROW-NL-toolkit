Multilinguistic (English and Czech as base) NL toolkit for language processing created within CROW project (CIIRC CTU in Prague)

Balíček je *jazykově agnostický* (jednotlivé funkce mají zvolený jazyk jako parametr a tudíž lze balíček spustit pod jakýmkoli jazykem). Specifikace jednotlivých postupů a šablon pro daný jazyk se připravuje v ````JSON```` souborech. V rámci publikovaného balíčku jsou implementována vzorová data pro angličtinu a češtinu. Tvorba balíčku nástrojů vycházela z předchozího průzkumu jednotlivých nástrojů pro zpracování anglického a českého jazyka.

Balíček obsahuje nejrůznější sady funkcí implementovaných vždy jako samostatné třídy (více viz [https://github.com/stepakar/CROW-NL-toolkit/README.md](https://github.com/stepakar/CROW-NL-toolkit/README.md)). Jedná se například o přípravu a kalibraci mikrofonu, zpracování jazykového vstupu, značkování slov pomocí slovních druhů a mluvnických kategorií, zpracování vět a souvětí, rozdělení souvětí, detekce jednotlivých obecných příkazů (jako například Polož, Ukliď apod.), detekce parametrů jednotlivých příkazů (barva, geometrie, typ objektu apod.), komunikace s uživatelem pomocí funkcí say a listen, výběr konkrétního objektu z dostupných objektů na pracovišti přímo před vykonáním daného příkazu (tzv. ukotvení).

Mnohé z dostupných funkcí lze *využít také nezávisle na zbývajících modulech*.

## Hlavní využití balíčku v rámci tohoto projektu
Hlavní využití balíčku v rámci projektu je: Informování uživatele o procesech na pracovišti; Rozpoznání příkazu uživatele; Generování zadání pro uživatele a porovnání odpovědi s danou sadou variant.

1. Informování uživatele o procesech na pracovišti pomocí jazykového či textového výstupu (````TextToSpeech````/````TextToScreen````) (
Příklad: *"Detekoval jsem příkaz POLOŽ a objekt ŠROUBOVÁK. Na pracovišti jsem nalezl dva šroubováky. Který z nich chcete podat?"*)

2. Rozpoznání příkazu uživatele včetně parametrů příkazu z obecně dané sady příkazů (např. *"Polož šroubovák na krabici."*). Uživatel může přímo referencovat objekty nacházející se na pracovišti. 

Pro účely testování obsahuje balíček sadu obecných předpřipravených příkazů (sadu lze dále rozšiřovat). Celá sada je dostupná na [https://github.com/stepakar/CROW-NL-toolkit](https://github.com/stepakar/CROW-NL-toolkit), příklad některých z těchto příkazů je v následující Tabulce. 




| Funkce | Šablona | Popis akce
|-----------|---------|----------
| GlueTask | <NALEP> <TYP> <POZICE> | robot aplikuje lepidlo na danou pozici
| FetchTask | <UCHOP> <OBJEKT> | robot uchopí zvolený objekt 
|     PutTask | <POLOŽ> <OBJEKT> <POZICE> | robot položí daný objekt na danou pozici
| FetchToHandTask | <PODEJ> <MI> <OBJEKT> | robot uchopí objekt a uvolní nad dlaní uživatele 
StoreTask | <ULOŽ> <OBJEKT> |  robot dá objekt do úložiště robotu (dostupné jen robotu)  
TidyingTask | <TIDY UP> | robot uklidí z pracoviště objekty mající úložiště (např. nástroje)
RemoveTask | <ODSTRAŇ> <NEPOTŘEBNÉ> <OBJEKT> | odstranění objektů nepotřebných pro aktuální úlohu 
InsertTask | <VLOŽ> <OBJEKT> <DO> <OBJEKT> | vsunutí objektu (př. kolíku) do otvoru 
UplugTask | <VYNDEJ> <OBJEKT> <Z> <OBJEKT> | vysunutí objektu z otvoru 
StackTask | <POLOŽ> <OBJEKT> <NA> <OBJEKT> | vrstvení objektů 
... | ... | ... 

3. Umožnění rozhodování uživatele o jednotlivých krocích procesu. Balíček umožňuje pro daný stav procesu *generování zadání* pro uživatele a *porovnání odpovědi uživatele* s danou sadou variant. Samotný proces postupu pro danou úlohu je definovaný uživatelem ve formě ````YAML```` souboru, který je následně zpracován pomocí sémantické reprezentace úloh (WP~\ref{sec:WP23}) a uložen ve znalostní bázi (WP~\ref{sec:znalostni_baze}). Pro interakci s uživatelem v daných stavech procesu lze zavolat funkci \texttt{query\_state}:

````
selected_variant = query_state(state_description, query_type, query_variants, mode)
````


Funkce ````query_state```` umožňuje interakci s uživatelem v několika módech: ````Select````, ````Yes_or_no```` a ````Inform````. 

\begin{description}
 * **Select** Uživateli je nabídnuto několik variant, ze kterých si může vybrat. Generované zadání: "*Jsme ve stavu ````<state_description>````. Možnosti výběru jsou
````<query_variants>````. Jakou možnost si přejete?"*.  Příklad: 

````
selected_variant = query_state("Výběr hlavy",select,{"medvěd", "kůň", "vlk"})
````
 Vygenerované zadání uživateli: *"Jsme ve stavu výběr hlavy. Možnosti jsou medvěd, kůň, vlk. Jakou možnost si přejete?"*
 
 Detekovaná odpověď uživatele: *"Chci medvěda"*. Odpověď je porovnána vůči zadaným možnostem a detekovaná možnost ````selected_variant = 1```` (Medvěd) vrácena zpět.


* **Yes_or_no** Uživateli je položena otázka a očekává se odpověď ve tvaru ano nebo ne (či jejich obdoba). Generované zadání uživateli: *"Jsme ve stavu ````<state_description>````. Chcete ````<query_variants>````. Ano nebo ne?"*. Příklad: 
````
selected_variant = query_state("Připojení příčníků", yes_or_no, "podat příčník")
````

Vygenerované zadání uživateli:*"Jsme ve stavu připojení příčníků. Chcete podat příčník? Ano nebo ne?"*

Detekovaná odpověď uživatele: *"Jo, chci."*. Odpověď je porovnána vůči sadě synonym a variant výběru a výsledná ````selected_variant = Yes```` vrácena zpět.

* **Inform** Uživatel je pouze informován o daném stavu a neočekává se žádná odpověď. Generované zadání uživateli: *"Jsme ve stavu ````<state_description>````. ````<OPTIONAL: query_variants>````."*. Příklad: 
    
````
selected_variant = query_state("Dokončování a úklidu", inform, "Nyní provedu úklid")
````

Vygenerovaný text uživateli: *"Jsme ve stavu dokončování a úklidu. Nyní provedu úklid."*

Mezi módy 2 (rozpoznání příkazu) a 3 (rozhodování o jednotlivých krocích procesu) lze přepínat. Díky tomu může uživatel kdykoli zadat vlastní příkaz a odložit odpověď na otázku systému.

Balíček nástrojů je úzce *propojený se znalostní bází (ontologií)*, kde jsou uloženy informace o daném stavu světa. Díky tomu lze využít *omezený slovník*, který je závislý na \emph{zpracovávané úloze}. Omezením slovníku se výrazně vylepší schopnosti zpracování vstupního jazykového příkazu (např. Při detekci objektu, který je podáván, porovnáváme detekované slovo vůči všem objektům, které se dle znalostní báze nacházejí v daném světě. Tudíž rozpoznáváme větu: "Podej mi kostku" a ne "Podej mi kobku."). V rámci open source balíčku je přiložena také vzorová ontologie, která obsahuje několik základních typů objektů dostupných na pracovišti a umožňuje tak testování.

Balíček je doplněn sadou *unit testů*. 

