# Obsługa dokumentacji

Strony źródłowe są w **/src**. Jest to czysty markdown, ale można używać też HTML.
 
Ważne - nazwa pliku .md powinna być spójna z głównym nagłówkiem (\# nazwa pliku \#), czyli np. jeżeli główny nagłówek to: 
 
\# jakaś tam podstrona \#
 
to plik powinien mieć nazwę jakas-tam-podstrona.md
 
Numery kolejnych nagłówków (\#\# Nagłówek 2 poziomu \#\#,  \#\#\# Nagłówek 3 poziomu \#\#\#) są generowane automatycznie zarówno w spisie treści jak i w samej treści.
 
Ważne aby w każdym pliku *.md były uzupełnione metadane. Na początku każdego pliku:
``` 
---
title: 'Tytuł strony - mało ważny'
platform_title: 'Wyświetlany tytuł platformy'
nazwa_platformy: true        // na tej wyświetlane jest odpowiednie menu górne itd
autotoc: true                // czy generować spis treści
layout: layout.hbs           // użyty layout
order: 2                     //kolejność w sekcji
---
```

 
Konfiguracja dostępnych sekcji jest w config.js.
 
W razie potrzeby kolejne sekcje dodajemy zgodnie z widoczną konwencją:
 
    'platforma/sekcja_dokumentacji/podstrony' : {
       pattern: 'platforma/sekcja_dokumentacji/podstrony/*.md',
       sortBy: 'order',
       metadata: {
           description: 'wyświetlany tytuł sekcji w menu bocznym'
       }
    }
 
Dokumentacja przebudowuje się automatycznie po puszczeniu pusha do brancha master. Potem jest widoczna tu: https://test-docs.husarion.com/

## Obrazki
 
Obrazki wrzucamy do /assets/img. Wyświetlanie - standardowy markdown.
 
Tworzenie galerii zdjęć:
 ```
<div class="gallery gallery-6">
    ![To jest podpis pod obrazkiem](/assets/img/zrodlo_obrazka.png "a ten opis wyświetla się przy powiększeniu obrazka")
    ![To jest podpis pod obrazkiem 2](/assets/img/zrodlo_obrazka_2.png "a ten opis wyświetla się przy powiększeniu obrazka 2")
</div>
 ```
`Gallery-6` - oznacza do 6 obrazków obok siebie. Docelowo będzie jeszcze `gallery-2`, `gallery-3` i inne w miarę potrzeby. Jak widać wewnątrz `<div class=”gallery”>` dodajemy standardowy markdown z obrazkami.
 
Standardowy markdown opisany jest np. tutaj:
https://daringfireball.net/projects/markdown/syntax
 
## Narzędzie ułatwiające konwersje plików do rozszerzenia .md

Wchodzimy na stronę z linku poniżej:
https://convertfiles.online/convert/tex/md
Wybieramy jakie rozszerzenie ma nasz oryginalny plik np: .tex.
Konwertujemy do .md, ale musimy pamiętać, że konwerter nie zrobi za nas wszystkiego!!! Po wygenerowaniu pliku .md sprawdźmy dokładnie czy znajduję się w nim wszystko co znaleść się powinno. 
