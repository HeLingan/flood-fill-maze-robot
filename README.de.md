[English](README.md) | [中文](README.zh.md) | [Deutsch](README.de.md)

# Flood Fill Maze Robot

Ein Python-Robotersimulator erkundet ein unbekanntes 8×8-Gitterlabyrinth mit lokaler Wanderkennung und Flood-Fill-Pfadplanung, umgesetzt als rückwärts vom Ziel ausgehende Breitensuche (BFS).

Maintainer: Lingan He. Mit KI-Unterstützung entwickelt.

## Funktionsweise

- `Maze` enthält den vollständigen Aufbau; `KnownMaze` speichert beobachtete Kanten. Anfangs kennt der Roboter Abmessungen, Start, Ziel und Außenbegrenzung. Er erfasst ausschließlich die vier Kanten seiner aktuellen Zelle.
- Die Planung behandelt unbekannte Kanten als passierbar. Nach jeder Beobachtung berechnet die rückwärts gerichtete BFS das gesamte Distanzfeld auf dieser optimistischen Karte neu. Ungerichtete Kanten und Kosten von eins je Schritt erlauben die Suche vom Ziel aus.
- Jeder Schritt führt über eine bestätigte `OPEN`-Kante in eine Nachbarzelle mit einer um eins kleineren Distanz. Bei Gleichstand gilt die Reihenfolge Ost, Süd, West, Nord.
- Die Erkundung endet am Ziel oder sobald von der aktuellen Zelle selbst auf der optimistischen Karte kein Weg zum Ziel existiert. Unter den Modellannahmen ist das Ziel dann nachweislich unerreichbar.

## Projektstruktur

| Datei | Aufgabe |
| --- | --- |
| `main.py` | Programmeinstieg und Diagnose von Tk-Startproblemen. |
| `maze.py` | Tatsächliche und bekannte Karte, lokale Wanderkennung und drei vordefinierte Labyrinthe. |
| `planner.py` | Distanzberechnung durch rückwärts gerichtete BFS. |
| `simulator.py` | Roboterzustand und Zyklus aus Wahrnehmung, Planung und Bewegung. |
| `ui.py` | Tkinter-Visualisierung, Bedienelemente und Zeitsteuerung. |
| `test_robot.py` | Verhaltenstests und unabhängige Dijkstra-Referenz. |
| `README.md`, `README.zh.md`, `README.de.md` | Englische, chinesische und deutsche Dokumentation. |
| `LICENSE` | MIT-Lizenzbedingungen. |
| `.gitignore` | Ausschluss lokaler Dateien und Python-Caches. |

## Voraussetzungen & Nutzung

Python 3.9+; ausschließlich Standardbibliothek. Die grafische Oberfläche benötigt Tkinter, eine funktionsfähige Tcl/Tk-Installation und eine grafische Desktop-Sitzung. Start im Repository-Verzeichnis:

```sh
python3 main.py
```

Unter Windows kann `py` anstelle von `python3` verwendet werden. Bedienelemente: **Step**, **Run/Pause**, **Reset** und Auswahl des Labyrinths.

- `open`: Navigation ohne Hindernisse.
- `detour`: Entdeckung einer verborgenen Sackgasse, Rückweg und Neuplanung.
- `blocked`: Erkennung eines unerreichbaren, vollständig eingeschlossenen Ziels.

## Tests

```sh
python3 -m unittest -v
```

Zehn Testmethoden prüfen die Ergebnisse der vordefinierten Labyrinthe, die Beschränkung auf lokal erfasste Informationen, Kantensymmetrie, widersprüchliche Beobachtungen, identische Start- und Zielpositionen, einen eingeschlossenen Start, ungültige Ziele sowie das Verhalten nach Abschluss der Erkundung. Prüfungen nach jedem Schritt kontrollieren sichere Bewegungen und einen konsistenten Zustand. Zwei Methoden enthalten 24 Teilfälle zum Vergleich vollständig bekannter Distanzen mit unabhängigen Dijkstra-Ergebnissen und 30 Teilfälle zum Vergleich der Erkundungsergebnisse mit der tatsächlichen Erreichbarkeit; beide verwenden feste Zufallsstartwerte. Die Teilfälle entsprechen nicht zwingend unterschiedlichen Labyrinthen. Das Verhalten der grafischen Oberfläche wird von dieser Testsuite nicht geprüft.

## Einschränkungen

Diskrete Gittersimulation mit statischen Wänden und idealen, rauschfreien Sensoren. Keine physische Hardware, Motoren oder kinematisches Modell. Die erste Erkundungsroute ist nicht zwingend ein global kürzester Weg; das Erreichen des Ziels bedeutet keine vollständige Kartierung.

## Lizenz

[MIT](LICENSE) — Copyright © 2026 HeLingan.
