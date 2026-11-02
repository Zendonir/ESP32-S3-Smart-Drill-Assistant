# Mathematik und Messprinzip

Die gesamte Mathematik steht in `lib/DrillCore` und ist hardwareunabhängig,
unit-getestet und frei von Heap-Allokationen im Messpfad.

## 1. Koordinatensysteme

| System | Definition |
|---|---|
| **S** Sensor | Achsen des QMI8658-Chips, wie er auf der Leiterplatte sitzt |
| **D** Gerät | X = Display rechts, Y = Display oben, Z = aus dem Display heraus (rechtshändig) |
| **T** Werkzeug | X_T = Bohrachse (zur Spitze), Z_T = Display-oben senkrecht zur Achse, Y_T = Z_T × X_T |
| **W** Welt | Z_W = oben (gegen die Schwerkraft), X_W/Y_W horizontal. **Azimut beliebig und vom Gyro gehalten** |
| **R** Referenz | gespeicherte Weltorientierung (Nullpunkt oder Fläche) |

Transformationen:

* **S→D**: feste vorzeichenbehaftete Achspermutation `AxisMap` (24 echte Drehungen,
  Determinante +1, Spiegelungen ausgeschlossen)
* **D→W**: Quaternion `q_WD` aus der Sensorfusion. Es gilt `v_W = q ⊗ v_D ⊗ q*`
* **D→T**: Montagemodell `MountModel`. Die Achse **d** (D-Koordinaten) stammt aus der
  Kalibrierung. Die Anzeige-Basis ist up_s = normalize(Y_D − (Y_D·d)d) und
  right_s = up_s × (−d). Liegt Y_D fast parallel zu **d**, wird Z_D verwendet.
* Referenzen speichern je nach Art einen Schwerkraftvektor (D) oder ein Quaternion (W).

Konventionen: Hamilton-Quaternionen (w, x, y, z), `q_AB` dreht Vektoren von B nach A,
`q_AC = q_AB ⊗ q_BC`. Rotationsmatrizen sind zeilenweise gespeichert.

## 2. Sensordatenverarbeitung

1. Der **IMU-Task** liest datengetrieben den Block 0x30..0x40 (17 Byte: Zähler,
   Temperatur, Accel, Gyro). Ein neues Sample erkennt er am 24-Bit-Sample-Zähler. Nach
   einem Sample schläft er 3 ms (Periode 4,46 ms) und pollt dann jede Millisekunde.
   Damit bleibt der Bus größtenteils frei für den Touch-Controller.
2. **Zeitschritt:** `dt = Δcounter · T̂`. T̂ wird über ≥ 2 s gegen die Quarzzeit des Hosts
   geschätzt und alle 30 s neu verankert, denn der interne Oszillator des Sensors hat
   einige Prozent Toleranz. Δcounter > 1 bedeutet verlorene Samples. Lücken über 100 ms
   gelten als Diskontinuität: Es wird nicht integriert, und die Fusion wird
   neu initialisiert. Verhält sich der Zähler in den ersten 64 Ereignissen nicht wie ein
   Sample-Zähler, schaltet die Firmware auf Host-Zeitstempel um.
3. **Plausibilität:** NaN oder |a| > 1,8·FS gelten als unplausibel. |raw| ≥ 32700 ist
   Clipping. Mehr als 150 identische Samples gelten als festhängend (echtes
   Sensorrauschen macht das praktisch unmöglich).
4. **Kalibrierung:** `g = g_roh − b(T) − b_online`, `a = M(T)·a_roh + c(T)`.
5. Danach folgen Bewegungs- und Vibrationsanalyse, Vertrauensgewicht, Fusion,
   Qualität und Snapshot.

## 3. Sensorfusion

### Filter

* **Mahony** (Standard): `ω' = ω + (k/dt)·(â × v̂)` mit `v̂ = q*⊗ẑ⊗q` (geschätztes „oben“
  im Gerät). Integration exakt: `q ← q ⊗ exp(ω'·dt/2)`.
* **Komplementär:** Gyro-Prädiktion, danach eine exakte Teilrotation um den Anteil k,
  die v̂ auf â dreht. Das ist auch bei großen Fehlern stabil.
* **Madgwick (IMU):** Gradientenabstieg mit β = k·|Fehler|/(2·dt), dadurch gleich schnelle
  Korrektur wie die anderen Filter.

Die Korrekturachse â × v̂ steht senkrecht auf „oben“. **Keiner der Filter korrigiert
den Azimut aus Beschleunigungsdaten**, dafür gibt es einen Unit-Test.

### Adaptive Verstärkung (Unsicherheitsmodell)

Ein skalarer, Kalman-artiger Tiltschätzer liefert k pro Schritt:

```
Prädiktion: σ ← sqrt(σ² + (σ_ω·dt)²) + (σ_b + ε_s·|ω|)·dt      (Bias, Skalenfehler linear)
Messung:    R = σ_a² / w,   k = σ² / (σ² + R),   k ∈ [k_min·w, k_max]
            σ² ← (1−k)²σ² + k²R                                   (Joseph-Form)
```

* σ_a ist das **gemessene** Kipprauschen des Beschleunigungssensors im Stillstand.
* σ_b ist die Bias-Unsicherheit: Rauschen/√N aus der Kalibrierung, bei Nachführung
  der Online-Wert, zuzüglich der angenommenen Temperaturdrift · ΔT.
* ε_s ist der Skalen- bzw. Kreuzachsenfehler (0,5 %).
* **w** ist das Vertrauen in den Beschleunigungssensor (0…1), das Produkt aus drei Rampen:
  |‖a‖−1 g| (1,5…12 % → 1…0), Vibrations-RMS (15…150 mg) und Drehrate (20…250 °/s).
  Bei Clipping ist w = 0.
* Im Stillstand begrenzt die Filterstärke k auf dt/τ_min (τ_min = 0,05…1 s).
* Nach einem Reset wird 1 s lang mit k ≥ 0,05 schnell eingeschwungen.

Folge: Bei langer Vibration ist w ≈ 0, der Filter arbeitet nur mit dem Gyro, **σ wächst
sichtbar**, und ab σ > 0,5° geht der Zustand auf „Orientierung eingeschränkt“. Die
Firmware verlässt sich also nicht dauerhaft unbemerkt nur auf das Gyroskop. Für ein
grünes Signal muss außerdem 2σ ≤ Toleranz gelten.

### Azimut (nicht beobachtbar)

Die Azimut-Unsicherheit wächst linear: `σ_ψ += (σ_b + ε_s·|ω|)·dt`. Sie wird nie aus
Messungen verringert, nur beim Erfassen einer Referenz auf 0 gesetzt. Referenzen mit
Gyro-Anteil zeigen deshalb ihr Alter und σ_ψ. Ab σ_ψ > 1° oder einem Alter über 10 min
meldet die Firmware „Referenz evtl. gedriftet“.

## 4. Stillstand, Bewegung, Vibration

* **Stillstand:** Alle Bedingungen müssen 0,6 s lang gelten:
  * |ω| < max(0,8 °/s; 5·σ_Gyro)
  * Kurzzeit-Std von a (EMA, τ = 0,25 s) < max(4 mg; 4·σ_a)
  * |‖a‖−1| < 0,03 g
  * Drift der Schwerkraftrichtung im Haltefenster < 0,25°. Das fängt langsames Kriechen
    ab, das unter der Gyroschwelle liegt.

  Der Stillstand endet sofort, wenn eine Bedingung verletzt wird.
* **Bewegung:** langsam bei ≤ 25 °/s, sonst schnell.
* **Vibration:** RMS des hochpassgefilterten Beschleunigungsvektors (2 Hz, τ = 0,2 s).
  Leicht ab 20 mg, stark ab 80 mg. Das Eskalieren geschieht sofort, die Rückstufung erst
  nach 0,5 s Haltezeit. Bandbegrenzung siehe README.
* **Automatischer Bohrmodus:** Aus Bewegungszustand und Vibrationsstufe ergibt sich
  „ruhig gehalten“, „wird ausgerichtet“, „vibriert stark“ oder „steht wieder“. Der
  Motorzustand wird **nicht** behauptet.

## 5. Winkelberechnung

**Wasserwaage:** Aus u = „oben“ in D folgt

* tiltX = asin(u_x), d. h. die rechte Kante liegt höher, wenn tiltX > 0
* tiltY = asin(u_y)
* Flachlage, wenn |u_z| ≥ cos 45°
* sonst Kantenlibelle: roll = atan2(u_x, u_y), auf ±45° um das nächste Vielfache von 90°
  reduziert

**Bohrachse:**

* d_W = q·d
* Elevation ε = asin(d_W·ẑ) (+ = Spitze nach oben)
* Neigung gegen die Horizontale β = |ε|
* Rolle der Maschine = Winkel zwischen ĥ und der Display-oben-Richtung um d

Die Winkelkorrektur dreht d in seiner Vertikalebene: `d' = d·cos c + ĥ·sin c`.

**Abweichung zum Ziel.** Am Ort d wird eine lokale Tangentialbasis gebildet:
`ĥ = normalize(ẑ − (ẑ·d)d)` (oben senkrecht zu d) und `ŝ = d × ĥ` (rechts).
Das Ziel t wird darin ausgedrückt: `t_x = t·ŝ`, `t_y = t·ĥ`, `t_z = t·d`.
Der exakte Winkel ist `θ = atan2(√(t_x²+t_y²), t_z)`. Der Marker (Lage der Achse relativ
zum Ziel) liegt bei `−θ·(t_x, t_y)/‖·‖` und wird über die Anzeige-Basis in
Bildschirmkoordinaten umgerechnet. Das ist eine 2D-Drehung, der Betrag bleibt erhalten.

Zielarten:

* **Neigungsziel** (Bohrassistent, Standard): t hat die Soll-Elevation ±β_soll **und
  denselben Azimut wie d**. Der seitliche Anteil ist damit konstruktionsbedingt 0, die
  Anzeige lautet „Seite frei“. Für β_soll = 90° ist t = ±ẑ, und die Abweichung ist ein
  vollständiger, rein gravitationsbasierter 2D-Kippvektor. Beide Fälle laufen über
  dieselbe Formel.
* **Richtungsziel** (3D-Referenz): t ist ein fester Weltvektor. Für nicht senkrechte
  Ziele hängt der seitliche Anteil vom Gyro-Azimut ab und ist so gekennzeichnet.
* **Hemisphäre** (bohren nach unten oder oben): automatisch mit Hysterese
  ±(β/2 + 2°) um die Horizontale, oder fest eingestellt.

**Zielerkennung** (`TargetTracker`):

* Innerhalb der Toleranz bei Eintritt ≤ tol, Austritt erst bei > tol·(1 + 0,25).
* „Im Ziel“ (grün) erst nach der Haltezeit (Standard 0,4 s).
* Nah, wenn ≤ max(3·tol, tol + 0,5°), mit 10 % Hysterese.
* Ungültige Messung (Qualität oder Unsicherheit) wird immer grau.

**Skala:** fest ±20/10/5/2° oder automatisch. Die automatische Skala vergrößert sofort
bei > 90 % und verkleinert erst nach 0,8 s unter 35 % der kleineren Stufe. Die
Abbildung ist linear und proportional. Außerhalb der Skala erscheint ein Randpfeil.

**Anzeigeglättung:** One-Euro-Filter (Grenzfrequenz steigt mit der Signalgeschwindigkeit)
und eine Zahlen-Hysterese von 0,6 Stellen gegen flackernde letzte Ziffern.

## 6. Referenzberechnung

| Art | gespeichert | Messgröße | Drift |
|---|---|---|---|
| Neigungs-Nullpunkt | u_ref (D) | Minimaldrehung u_ref → ẑ angewendet auf u → (relX, relY) | **driftfrei** |
| 3D-Nullpunkt | q_ref | Neigungsanteil wie oben und Azimutänderung | Azimut **driftet** (σ_ψ) |
| Fläche (nur Neigung) | Normale n_W (Gerät flach auf der Fläche, Rückseite aufliegend: n = q·ẑ_D) | Neigungsziel mit β = Elevation von −n | driftfrei. Bei schräger Fläche bleibt der **Azimut frei** (Anzeige) und muss vom Nutzer, z. B. entlang der Falllinie, ausgerichtet werden |
| Fläche 3D | n_W | Richtungsziel t = −n | Azimut driftet, inklusive Skalenfehler bei der Ummontage (∫\|ω\|) |

Für horizontale Flächen ist der Flächenmodus vollständig gravitationsbasiert. Für eine
beliebig orientierte Fläche ist eine dauerhaft driftfreie Vollreferenz mit einer
einzelnen 6-Achsen-IMU **physikalisch nicht möglich**. Die Firmware bietet daher den
eingeschränkten Modus als Standard.

## 7. Präzisionsmodus

* Die Messung wartet auf Stillstand ohne Vibration.
* Im Zeitfenster (0,5–10 s) werden Einzelwerte direkt aus dem kalibrierten
  Beschleunigungssensor gesammelt, ohne Gyro-Integration.
* Ausgewertet werden Median (robuster Wert), Mittelwert, Standardabweichung (Rauschen),
  MAD, Standardfehler des Mittelwerts und linearer Trend.
* „Stabil“ heißt: Std ≤ 0,05° und |Trend| ≤ 0,02 °/s.
* Bei Bewegung beginnt das Fenster neu.
* Über mehrere Messungen wird die **Wiederholbarkeit** (Std der Ergebnisse) angegeben.

Der Standardfehler ist eine rein statistische Größe. Er sagt nichts über die absolute
Genauigkeit (Kalibrierung, Montage, Referenz).

## 8. Grenzen der 6-Achsen-IMU, kurz gefasst

* Absolut messbar sind 2 Freiheitsgrade: die Richtung der Schwerkraft im Gerät.
* Nicht absolut messbar ist der Azimut. Er ist relativ über die Gyro-Integration
  verfügbar und driftet mit Bias-Rest und Skalenfehler.
* Lineare Beschleunigungen und Vibrationen verfälschen die Schwerkraftmessung. Die
  Firmware erkennt das, gewichtet herunter, lässt die Unsicherheit wachsen und warnt.
* Die Genauigkeit ist begrenzt durch die Kalibrierqualität (Restfehler in mg ↔ Winkel:
  1 mg ≈ 0,057°), die Temperaturdrift und die Montage-Wiederholbarkeit.
