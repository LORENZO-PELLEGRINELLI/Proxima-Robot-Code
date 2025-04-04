# Proxima

Proxima è un progetto di robotica basato su Arduino che integra modalità di controllo manuale e autonoma. Il robot utilizza vari sensori per navigare nell'ambiente, evitare ostacoli e fornire un monitoraggio in tempo reale tramite una dashboard web.

## Indice

- [Panoramica](#panoramica)
- [Caratteristiche](#caratteristiche)
- [Requisiti Hardware](#requisiti-hardware)
- [Requisiti Software](#requisiti-software)
- [Schema di Collegamento e Configurazione dei Pin](#schema-di-collegamento-e-configurazione-dei-pin)
- [Installazione](#installazione)
- [Panoramica del Codice](#panoramica-del-codice)
  - [Funzione setup()](#funzione-setup)
  - [Funzione loop()](#funzione-loop)
  - [Funzioni Chiave](#funzioni-chiave)
- [Utilizzo](#utilizzo)
  - [Modalità Autonoma](#modalità-autonoma)
  - [Modalità Manuale](#modalità-manuale)
  - [Dashboard Web](#dashboard-web)
- [Contributi](#contributi)
- [Licenza](#licenza)
- [Riconoscimenti](#riconoscimenti)

## Panoramica

Il progetto Proxima implementa un robot che può navigare in autonomia o essere controllato da remoto in modalità manuale. Il robot si avvale di sensori (ultrasuoni, infrarossi e DHT11 per temperatura e umidità) e di un servomotore per eseguire scansioni ambientali. Inoltre, ospita un server web WiFi che mostra in tempo reale i dati dei sensori e lo stato del sistema tramite una dashboard interattiva.

## Caratteristiche

- **Navigazione Autonoma:**  
  - Rilevamento degli ostacoli tramite sensori ad ultrasuoni e infrarossi.
  - Navigazione adattiva mediante un controllore PID per regolare la velocità dei motori.
  - Esecuzione di manovre di fuga in situazioni di blocco.

- **Controllo Manuale:**  
  - Possibilità di inviare comandi da remoto tramite l'interfaccia web.
  - Comandi disponibili: avanti, indietro, sinistra, destra e stop.

- **Monitoraggio in Tempo Reale:**  
  - Una dashboard web che visualizza lo stato del movimento, la distanza misurata, lo stato dei sensori IR, la velocità attuale e la potenza del segnale WiFi.
  - Aggiornamenti continui dei dati tramite richieste AJAX.

## Requisiti Hardware

- **Scheda di Controllo:**  
  Una scheda Arduino compatibile con sufficienti pin di I/O.

- **Sensori e Attuatori:**
  - **Sensore ad Ultrasuoni:** Per la misurazione della distanza (Trigger su A1, Echo su A0).
  - **Sensori Infrarossi:** Due sensori per il rilevamento degli ostacoli (su pin 9 e 10).
  - **Sensore DHT11:** Per la misurazione di temperatura e umidità (su pin 1).
  - **Servomotore:** Per la scansione dell'ambiente (connesso al pin A2).

- **Controllo dei Motori:**
  - Driver per motori collegato ai seguenti pin:
    - **Motore Sinistro:** Pin direzionali su 2 e 4, PWM su 5.
    - **Motore Destro:** Pin direzionali su 7 e 8, PWM su 6.

- **Modulo WiFi:**  
  Compatibile con la libreria [WiFiS3](https://github.com/arduino-libraries/WiFi) per la connessione in rete.

## Requisiti Software

- **Arduino IDE** o un ambiente di sviluppo compatibile.
- **Librerie Richieste:**
  - [SPI](https://www.arduino.cc/en/Reference/SPI)
  - [WiFiS3](https://github.com/arduino-libraries/WiFi)
  - [Servo](https://www.arduino.cc/en/Reference/Servo)
  - [PID_v1](https://playground.arduino.cc/Code/PIDLibrary/)
  - [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library)

## Schema di Collegamento e Configurazione dei Pin

| Componente              | Descrizione Funzionale                                    | Assegnazione Pin         |
|-------------------------|-----------------------------------------------------------|--------------------------|
| **Motori**              |                                                           |                          |
| PWM Motore Sinistro     | Controllo velocità                                        | Pin 5                    |
| PWM Motore Destro       | Controllo velocità                                        | Pin 6                    |
| Direzione Motore Sinistro| IN1 e IN2 per marcia avanti e retromarcia                | Pin 2 (LB) e 4 (LF)      |
| Direzione Motore Destro  | IN3 e IN4 per marcia avanti e retromarcia                | Pin 7 (RB) e 8 (RF)      |
| **Sensore ad Ultrasuoni**|                                                           |                          |
| Trigger                 | Avvio della misurazione della distanza                    | Pin A1                   |
| Echo                    | Ricezione del segnale riflesso                            | Pin A0                   |
| **Sensori Infrarossi**  | Rilevamento ostacoli                                      | LEFT_IR: Pin 9, RIGHT_IR: Pin 10 |
| **Servomotore**         | Meccanismo di scansione (ruota verso angoli predefiniti)    | Pin A2                   |
| **Sensore DHT11**       | Sensore di temperatura e umidità                          | Pin 1                    |

## Installazione

1. **Clonare il Repository:**

   ```bash
   git clone https://github.com/tuo-username/proxima.git
   cd proxima
