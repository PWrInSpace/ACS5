# Test serw na stole (build bench)

Cel: potwierdzić mapowanie gniazd J4 na płetwy, kierunki wychyleń i działanie blokady. Wyniki są wejściem do miksera regulatora 3-osiowego.

## Pierwsze uruchomienie płytki

Zegar systemowy pochodzi z kwarcu HSE 50 MHz (Y1). Jeśli kwarc nie wystartuje, firmware zatrzymuje się w `Error_Handler` zaraz po włączeniu: brak enumeracji USB, brak komunikatów, LED-y nie migają. Wtedy sprawdzić montaż Y1 oraz C34/C35 (6 pF).

1. Flash buildu lotnego, USB do komputera.
2. Port CDC pojawia się w systemie, a w terminalu `--- ACS5 Aggregator Task Starting ---`.
3. Telemetria raportuje pętlę ~500 Hz (`[STANDBY] 500 Hz`).

## Przygotowanie

1. Serwa SRT T75 w gniazdach J4 CH2–CH5, serwo blokady w CH6. CH1 nie jest podłączone do MCU.
2. Zasilanie VIN dla serw; USB do komputera, terminal na porcie CDC.
3. Build bench (README, sekcja „Servo bench build”), flash.

## Procedura

1. Po starcie: blokada otwiera się na 3 s i zamyka; w terminalu `[OK] Servos armed (neutral), canards locked`, potem `[BENCH] Servo bench build - flight loop disabled`.
2. Dla każdego gniazda CH2–CH5 terminal drukuje `[BENCH] J4 CHx +10 deg`, `-10 deg`, `+0 deg` co 1 s. W tym czasie rusza się wyłącznie serwo w gnieździe CHx. Zapisz w tabeli, która to płetwa na kadłubie i w którą stronę wychyla się krawędź spływu przy `+10`.
3. `[BENCH] LOCK OPEN` → blokada otwarta; po 3 s `[BENCH] LOCK CLOSED` → zamknięta.
4. Cykl powtarza się co 18 s.

## Build lotny

1. Flash buildu lotnego.
2. Po starcie: sekwencja blokady (otwarta 3 s → zamknięta), canardy w neutrum, brak ruchu serw w STANDBY.
3. LED_1 (PA8) miga w rytmie telemetrii; serwa nie drgają przy mruganiu.

## Wyniki

Data: — Płytka: — Wykonał: —

| Gniazdo J4 | Fizyczna płetwa (pozycja na kadłubie, widok od nosa) | Kierunek krawędzi spływu przy +10° | Neutrum [µs] |
|---|---|---|---|
| CH2 (fin 0) | | | |
| CH3 (fin 1) | | | |
| CH4 (fin 2) | | | |
| CH5 (fin 3) | | | |

| Blokada (CH6) | Wynik |
|---|---|
| 1050 µs | otwarta / zamknięta |
| 1900 µs | otwarta / zamknięta |
