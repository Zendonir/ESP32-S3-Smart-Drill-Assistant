# Kalibrierung

Die Kalibrierung hat zwei getrennte Ebenen:

1. **Sensorgrundkalibrierung** (gilt für das Gerät, NVS-Namespace `sdacal`): Gyro-Offset,
   Offset und Skala des Beschleunigungssensors, optional eine Kreuzkopplungsmatrix und
   Temperaturtabellen.
2. **Montagekalibrierung** (gilt für ein Werkzeug, im Profil in LittleFS): die Lage der
   Bohrachse im Gerätekoordinatensystem und eine Winkelkorrektur.

Ein Werksreset löscht Einstellungen und Profile. Die Sensorkalibrierung bleibt erhalten,
sie wird separat unter Kalibrierung › „Kalibr. löschen“ gelöscht.

## 0. Achsenzuordnung (Menü › Kalibrierung › Achsen erkennen)

Dieser Schritt kommt vor allen anderen.
Er bestimmt, wie die Sensorachsen des QMI8658 auf die Geräteachsen fallen (X rechts, Y oben, Z aus dem Display).
Dafür misst die Firmware zwei Ruhelagen, jeweils etwa 1 s.
Jede Messung beginnt erst nach **Start** bzw. **Weiter**, damit man in Ruhe umlagern kann.
Schritt 2 wird zusätzlich nur gemessen, wenn das Gerät mindestens 45° anders liegt als in Schritt 1:

1. Display flach nach oben: Die Schwerkraftachse ergibt Z.
2. Aufrecht, Text lesbar: Die Schwerkraftachse ergibt Y.

X folgt daraus als Y × Z, die Zuordnung ist also immer eine echte Drehung.
Jede Lage muss innerhalb von 25° an einer Sensorachse liegen.
Der angezeigte „Lagefehler“ ist die Abweichung der beiden Messungen von 90°.
Beim Speichern rechnet die Firmware Gyro-Bias und Beschleunigungskalibrierung exakt in den neuen Rahmen um (M' = R·M·Rᵀ, c' = R·c, b' = R·b).

## 1. Gyroskop (Menü › Kalibrierung › Gyroskop)

Ablauf:

1. Gerät auf eine feste Unterlage legen und nicht berühren.
2. Es werden 600 Samples (ca. 2,7 s) aufgenommen.
3. Bewegungsprüfung je Sample: Weicht die Drehrate mehr als 2 °/s vom laufenden
   Mittelwert ab oder ändert sich der Schwerkraftvektor um mehr als 0,02 g, beginnt die
   Messung neu. Nach 20 Neustarts bricht sie ab.
4. Ergebnis: Mittelwert (Bias), Standardabweichung (Rauschen) und Temperatur.
5. Plausibilität: Rauschen ≤ 0,6 °/s je Achse und |Bias| ≤ 20 °/s, sonst wird das
   Ergebnis abgelehnt.
6. **Speichern** übernimmt den Punkt in die Temperaturtabelle und schreibt ihn in den
   Flash. **Verwerfen** stellt den vorherigen Zustand wieder her.

Beim Start (Einstellung „Gyro-Kal. Start“):

* *Wenn ruhig* (Standard): Die Kalibrierung läuft automatisch, wird aber verworfen,
  sobald das Gerät bewegt wird. Das Ergebnis gilt nur im RAM. So entsteht kein
  Flash-Schreibvorgang bei jedem Start. Ausnahme: Existiert noch keine gespeicherte
  Gyro-Kalibrierung, wird das erste erfolgreiche Ergebnis gespeichert.
* *Immer*: Die Kalibrierung wird beim Start geführt auf dem Display angezeigt.
* *Nie*: Es gelten nur die gespeicherten Werte.

Im Betrieb wird der Bias im **bestätigten Stillstand** zusätzlich nachgeführt (EMA mit
τ = 8 s, nur ohne Vibration, Begrenzung ±1,5 °/s). Die Nachführung erfasst alle drei
Achsen, auch die Achse um die Lotrichtung.

## 2. Beschleunigungssensor, 6 Lagen (Menü › Kalibrierung › Sensor 6 Lagen)

Die sechs Lagen beziehen sich auf das Gerätekoordinatensystem (X rechts, Y oben,
Z aus dem Display):

| Lage | Haltung |
|---|---|
| +Z | Display nach oben, flach liegend |
| −Z | Display nach unten |
| +Y | aufrecht auf der unteren Kante (oben = oben) |
| −Y | Kopfstand (auf der oberen Kante) |
| +X | auf der linken Kante, rechte Kante oben |
| −X | auf der rechten Kante |

Nach jeder fertigen Lage zeigt der Assistent „✓ Lage … erledigt (n/6)“ und wartet auf **Weiter**. Erst dann wird die nächste Lage erkannt und gemessen. Die erste Messung startet mit **Start**.

Weil das Gerät rund ist, empfiehlt sich ein **rechtwinkliges Gehäuse oder ein Halter**
mit ebenen Flächen. Die Reihenfolge ist beliebig: Die Firmware erkennt die aktuelle Lage
(Toleranz 20°) und misst eine noch fehlende Lage automatisch, sobald das Gerät ruhig
liegt. Pro Lage werden 300 Samples gemittelt. Bei mehr als 15 mg Abweichung vom
laufenden Mittel oder falscher Lage beginnt die Messung dieser Lage neu.

**Standardmodell (Offset und Skala je Achse):**

```
o_i = (m_i(+) + m_i(−)) / 2        s_i = (m_i(+) − m_i(−)) / 2
a_kal,i = (a_roh,i − o_i) / s_i
```

Kleine Platzierungsfehler gehen nur in zweiter Ordnung ein: 2° Schiefstand ergibt
cos 2° = 0,9994, also 0,06 % Skalenfehler. Bei der Mittelung von +/− heben sie sich im
Offset auf.

**Erweitertes Modell (3×3-Matrix und Offset)** löst `a_kal = M·a_roh + c` je
Ausgangszeile per linearer Ausgleichsrechnung über die 6 Lagen (6 Gleichungen,
4 Unbekannte). Es korrigiert auch Achsfehlstellungen und Kreuzkopplung, **setzt aber
exakt orthogonale Auflageflächen voraus**, etwa einen Präzisionswinkel. Ungenaue
Flächen werden sonst als Kreuzkopplung „wegkalibriert“.

**Qualität:** Der RMS-Wert von |a_kal| − 1 g über alle 6 Lagen wird als „Restfehler“
in mg angezeigt und gespeichert. Typisch gut sind < 3 mg. Plausibilitätsgrenzen:
Skala 0,8…1,25, Nebendiagonale ≤ 0,1, Offset ≤ 0,5 g.

## 3. Temperaturkompensation

Jede Kalibrierung speichert ihre mittlere Sensortemperatur. Pro Größe entsteht eine
Tabelle mit bis zu 6 Gyro- und 4 Accel-Punkten. Ein Punkt innerhalb von ±1,5 K (Gyro)
bzw. ±3 K (Accel) zu einem vorhandenen ersetzt diesen.

* Zwischen den Punkten wird linear interpoliert.
* Außerhalb des Bereichs wird **nicht extrapoliert**. Es gilt der nächstgelegene Punkt.
* Als „aktiv“ gilt die Kompensation erst ab **2 Punkten mit ≥ 3 K Abstand**. Mit nur einem
  Punkt meldet die Firmware ausdrücklich „TK inaktiv“.
* Ungültige Temperaturen (außerhalb −40…105 °C, NaN) gelten als ungültig und werden nicht
  zur Interpolation verwendet.
* Liegt die aktuelle Temperatur mehr als 10 K vom kalibrierten Bereich entfernt, wird das
  Qualitätsflag `TempOutOfCalRange` gesetzt.

Vorgehen: Gyro und 6 Lagen bei Raumtemperatur kalibrieren, dann z. B. nach 20 min
Betrieb in warmer Umgebung oder im kalten Keller erneut kalibrieren. Die neuen Punkte
werden ergänzt.

Das Unsicherheitsmodell nimmt bis zur Kompensation eine Bias-Temperaturdrift von
0,02 °/s/K an. Dieser Wert ist eine **Annahme**, kein verifizierter Datenblattwert, und
im Menü bzw. in der API einstellbar.

## 4. Montagekalibrierung

Die Leiterplatte sitzt in der Regel nicht exakt parallel zur Bohrachse. Gesucht ist der
Einheitsvektor der Bohrachse **d** im Gerätekoordinatensystem. Er hat 2 Freiheitsgrade.
Die Rolllage um die Achse ist für die Bohrrichtung bedeutungslos und wird aus der
Displaygeometrie abgeleitet.

**Kein Verfahren leitet die Montage aus einer einzigen beliebigen Lage ab.**

### a) V-Nut-/Rollverfahren (empfohlen, Menü › Montage V-Nut)

1. Gerät fest an der Bohrmaschine montieren.
2. Den Bohrer, idealerweise einen langen, geraden Bohrer oder Dorn, mit dem Schaft in
   eine V-Nut oder auf zwei Prismen legen. Die Achse sollte möglichst waagerecht sein,
   eine Neigung bis 60° ist erlaubt.
3. Die Maschine **um die eigene Bohrachse drehen** und an 4–8 Positionen (je 45–90°)
   jeweils „Erfassen“ drücken. Die Erfassung wartet auf Stillstand und mittelt dann
   250 Samples.
4. Bei fester Achse liegen alle Schwerkraftvektoren auf einem Kegel um **d**. Ihre
   Spitzen liegen in einer Ebene, deren Normale **d** ist (Eigenvektor zum kleinsten
   Eigenwert der Streumatrix).

Qualitätskriterien:

* Abdeckung ≥ 90° (größte Winkellücke < 270°)
* Kegel-Restfehler ≤ 1° RMS
* Positionen untereinander ≥ 15° verschieden

Angezeigt werden Achse, Restfehler und Abdeckung.

### b) Bohrständer (Menü › Montage Ständer)

Die Bohrachse wird **exakt senkrecht** gestellt, z. B. in einem Bohrständer, der mit
einer Präzisionswasserwaage geprüft ist. Eine Ruhemessung liefert dann **d** = −(oben).
Die Genauigkeit entspricht der Genauigkeit der mechanischen Referenz.

### c) Grobe Voreinstellung (Menü › Montage grob)

Die Achse ist ohne Messung eine der Hauptachsen des Geräts, z. B. „Display zum Nutzer“,
also d = −Z. Das ist nur für grobe Arbeiten gedacht.

### Winkelkorrektur (Profil)

Eine zusätzliche Elevationskorrektur (±10°, Schritt 0,05°) für bekannte systematische
Abweichungen, z. B. aus einer Vergleichsmessung mit einer Referenz.

## 5. Profile

Ein Profil enthält Montageachse, Methode, Restfehler, Winkelkorrektur, Sollwinkel,
Toleranz, Skala und Anzeigeoptionen. Standardprofile: Bohrmaschine 1, Bohrmaschine 2,
Akkuschrauber, Freie Wasserwaage (Messachse = Geräte-X, Toleranz 0,2°),
Benutzerdefiniert. Es gibt bis zu 8 Profile. Freie Namen lassen sich über die
Weboberfläche vergeben.

Profilspezifische Sensor-Offsets sind **nicht** nötig: Der Sensor wird unabhängig vom
Werkzeug kalibriert, nur Achse und Korrektur hängen vom Werkzeug ab.

## Typische Fehlerquellen

* Bewegung oder Vibration (Tisch, Kompressor) während der Kalibrierung → Neustarts oder
  höheres Rauschen
* Wärmequelle in der Nähe, Gerät gerade eingeschaltet → Temperaturdrift während der
  Messung. Nach dem Einschalten etwa 5 min warten.
* Unebene Auflageflächen bei der erweiterten 6-Lagen-Kalibrierung
* V-Nut-Verfahren mit krummem Bohrer oder Spiel im Bohrfutter → Kegel-Restfehler steigt
* Montage, die sich nach der Kalibrierung bewegt (Klettband, Gummihalter)

## Prüfung der Ergebnisse

1. Nach der 6-Lagen-Kalibrierung in jeder der 6 Lagen |a| prüfen (Diagnose oder
   `/api/imu`): Abweichung von 1,000 g ≤ 3 mg.
2. **Umschlagmessung:** Gerät auf eine Fläche legen, Neigung X ablesen, um 180° drehen
   (gleiche Auflagepunkte) und erneut ablesen. Die halbe Summe ist der Nullpunktfehler
   des Geräts, die halbe Differenz die Neigung der Fläche.
3. Montage: Bohrer im geprüften senkrechten Ständer → Bohrmodus 90° muss im Ziel liegen.
4. Gyro: im Stillstand Diagnose › „Online-Bias“. Nach einer frischen Kalibrierung sollte
   er < 0,05 °/s bleiben.
