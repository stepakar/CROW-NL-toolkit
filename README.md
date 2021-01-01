Multilinguistic (English and Czech as base) NL toolkit for language processing created within CROW project (CIIRC CTU in Prague)

Balíček je \emph{jazykově agnostický} (jednotlivé funkce mají zvolený jazyk jako parametr a tudíž lze balíček spustit pod jakýmkoli jazykem). Specifikace jednotlivých postupů a šablon pro daný jazyk se připravuje v \texttt{JSON} souborech. V rámci publikovaného balíčku jsou implementována vzorová data pro angličtinu a češtinu. Tvorba balíčku nástrojů vycházela z předchozího průzkumu jednotlivých nástrojů pro zpracování anglického a českého jazyka.

Balíček obsahuje nejrůznější sady funkcí implementovaných vždy jako samostatné třídy (více viz \url{https://github.com/stepakar/CROW-NL-toolkit/README.md}). Jedná se například o přípravu a kalibraci mikrofonu, zpracování jazykového vstupu, značkování slov pomocí slovních druhů a mluvnických kategorií, zpracování vět a souvětí, rozdělení souvětí, detekce jednotlivých obecných příkazů (jako například Polož, Ukliď apod.), detekce parametrů jednotlivých příkazů (barva, geometrie, typ objektu apod.), komunikace s uživatelem pomocí funkcí say a listen, výběr konkrétního objektu z dostupných objektů na pracovišti přímo před vykonáním daného příkazu (tzv. ukotvení).

Mnohé z dostupných funkcí lze \emph{využít také nezávisle na zbývajících modulech}.

\subsection{Hlavní využití balíčku v rámci tohoto projektu} 
Hlavní využití balíčku v rámci projektu je: Informování uživatele o procesech na pracovišti; Rozpoznání příkazu uživatele; Generování zadání pro uživatele a porovnání odpovědi s danou sadou variant.
\begin{enumerate}
\item Informování uživatele o procesech na pracovišti pomocí jazykového či textového výstupu (\texttt{TextToSpeech}/\texttt{TextToScreen}) (
Příklad: \textit{\uv{Detekoval jsem příkaz POLOŽ a objekt ŠROUBOVÁK. Na pracovišti jsem nalezl dva šroubováky. Který z nich chcete podat?}})

\item Rozpoznání příkazu uživatele včetně parametrů příkazu z obecně dané sady příkazů (např. \textit{\uv{Polož šroubovák na krabici.}}). Uživatel může přímo referencovat objekty nacházející se na pracovišti. 

Pro účely testování obsahuje balíček sadu obecných předpřipravených příkazů (sadu lze dále rozšiřovat). Celá sada je dostupná na \url{https://github.com/stepakar/CROW-NL-toolkit}, příklad některých z těchto příkazů je v Tabulce~\ref{tab:nl_toolkit_prikazy}. 

\begin{table}[!h]
\scriptsize
\caption{Ukázka vybraných šablon předpřipravených příkazů.}
\begin{tabular}{l|l l}
Funkce & Šablona & Popis akce \\
\hline
GlueTask & <NALEP><TYP><POZICE> & robot aplikuje lepidlo na danou pozici\\
FetchTask & <UCHOP> <OBJEKT> & robot uchopí zvolený objekt \\
     PutTask & <POLOŽ><OBJEKT><POZICE> & robot položí daný objekt na danou pozici\\
FetchToHandTask & <PODEJ> <MI> <OBJEKT> & robot uchopí objekt a uvolní nad dlaní uživatele \\
StoreTask & <ULOŽ> <OBJEKT> &  robot dá objekt do úložiště robotu (dostupné jen robotu)  \\
TidyingTask & <TIDY UP> & robot uklidí z pracoviště objekty mající úložiště (např. nástroje)
 \\
RemoveTask & <ODSTRAŇ> <NEPOTŘEBNÉ> <OBJEKT> & odstranění objektů nepotřebných pro aktuální úlohu \\
InsertTask & <VLOŽ> <OBJEKT> <DO> <OBJEKT> & vsunutí objektu (př. kolíku) do otvoru \\
UplugTask & <VYNDEJ> <OBJEKT> <Z> <OBJEKT> & vysunutí objektu z otvoru \\
StackTask & <POLOŽ> <OBJEKT> <NA> <OBJEKT> & vrstvení objektů \\
... & ... & ... \\
\end{tabular}
\label{tab:nl_toolkit_prikazy}
\end{table}

\item Umožnění rozhodování uživatele o jednotlivých krocích procesu. Balíček umožňuje pro daný stav procesu \emph{generování zadání} pro uživatele a \emph{porovnání odpovědi uživatele} s danou sadou variant. Samotný proces postupu pro danou úlohu je definovaný uživatelem ve formě \texttt{YAML} souboru, který je následně zpracován pomocí sémantické reprezentace úloh (WP~\ref{sec:WP23}) a uložen ve znalostní bázi (WP~\ref{sec:znalostni_baze}). Pro interakci s uživatelem v daných stavech procesu lze zavolat funkci \texttt{query\_state}:

\begin{minted}[fontsize=\small]{python}
selected_variant = query_state(state_description, query_type, query_variants, mode)
\end{minted}

%\begin{tcolorbox}
%\begin{verbatim}
%selected_variant = query_state(state_description, query_type,
%query_variants, mode)
%\end{verbatim}
%\end{tcolorbox}
%Tento způsob komunikace a funkci \texttt{query\_state} lze využít nezávisle na ontologii a daném pracovišti při komunikaci s uživatelem například na základě předpřipraveného stavového automatu. 

Funkce \texttt{query\_state} umožňuje interakci s uživatelem v několika módech: \texttt{Select}, \texttt{Yes\_or\_no} a \texttt{Inform}. 

\begin{description}
\item [{Select}] Uživateli je nabídnuto několik variant, ze kterých si může vybrat. Generované zadání: \textit{\uv{Jsme ve stavu \texttt{<state\_description>}. Možnosti výběru jsou
\texttt{<query\_variants>}. Jakou možnost si přejete?}}.  Příklad: 

%\begin{tcolorbox}
\begin{minted}[fontsize=\small]{python}
selected_variant = query_state("Výběr hlavy",select,{"medvěd", "kůň", "vlk"})
\end{minted}
\begin{tcolorbox}
 Vygenerované zadání uživateli: \textit{\uv{Jsme ve stavu výběr hlavy. Možnosti jsou medvěd, kůň, vlk. Jakou možnost si přejete?} }
 
 Detekovaná odpověď uživatele: \textit{\uv{Chci medvěda}}. Odpověď je porovnána vůči zadaným možnostem a detekovaná možnost \texttt{selected\_variant = 1} (Medvěd) vrácena zpět.
  \end{tcolorbox}

%\texttt{selected\_variant = query\_state(\uv{Výběr hlavy},select,\{\uv{medvěd}, \uv{kůň}, \uv{vlk}\})}

%\end{tcolorbox}

\item [{Yes\_or\_no}] Uživateli je položena otázka a očekává se odpověď ve tvaru ano nebo ne (či jejich obdoba). Generované zadání uživateli: \textit{\uv{Jsme ve stavu \texttt{<state\_description>}. Chcete \\ \texttt{<query\_variants>}. Ano nebo ne?}}. Příklad: 
\begin{minted}[fontsize=\small]{python}
selected_variant = query_state("Připojení příčníků", yes_or_no, "podat příčník")
\end{minted}

\begin{tcolorbox}
%Vstup funkce: \texttt{(\uv{Připojení příčníků}, yes\_or\_no, {\uv{podat příčník}} )}

Vygenerované zadání uživateli:\textit{ \uv{Jsme ve stavu připojení příčníků. Chcete podat příčník? Ano nebo ne?} }

Detekovaná odpověď uživatele: \textit{\uv{Jo, chci.}}. Odpověď je porovnána vůči sadě synonym a variant výběru a výsledná \texttt{selected\_variant = Yes} vrácena zpět.
\end{tcolorbox}

\item [{Inform}] Uživatel je pouze informován o daném stavu a neočekává se žádná odpověď. Generované zadání uživateli: \textit{\uv{Jsme ve stavu \texttt{<state\_description>}. \texttt{<OPTIONAL: \\ query\_variants>}.}}. Příklad: 
    
\begin{minted}[fontsize=\small]{python}
selected_variant = query_state("Dokončování a úklidu", inform, "Nyní provedu úklid")
\end{minted}

\begin{tcolorbox}
Vygenerovaný text uživateli: \textit{\uv{Jsme ve stavu dokončování a úklidu. Nyní provedu úklid.} }
\end{tcolorbox}
\end{description}
\end{enumerate}

Mezi módy 2 (rozpoznání příkazu) a 3 (rozhodování o jednotlivých krocích procesu) lze přepínat. Díky tomu může uživatel kdykoli zadat vlastní příkaz a odložit odpověď na otázku systému.

Balíček nástrojů je úzce \emph{propojený se znalostní bází (ontologií)}, kde jsou uloženy informace o daném stavu světa. Díky tomu lze využít \emph{omezený slovník}, který je závislý na \emph{zpracovávané úloze}. Omezením slovníku se výrazně vylepší schopnosti zpracování vstupního jazykového příkazu (např. Při detekci objektu, který je podáván, porovnáváme detekované slovo vůči všem objektům, které se dle znalostní báze nacházejí v daném světě. Tudíž rozpoznáváme větu: \uv{Podej mi kostku} a ne \uv{Podej mi kobku.}). V rámci open source balíčku je přiložena také vzorová ontologie, která obsahuje několik základních typů objektů dostupných na pracovišti a umožňuje tak testování.

Balíček je doplněn sadou \emph{unit testů}. 

