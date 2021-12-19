# Softwareentwicklungsprojekt: <Punktwolkensegmentierung mittels Machine Learning>

## Punktwolkensegmentierung mittels Machine Learning

- Teammitglieder:
  1.  Vinh Thong Trinh
  2.  Marouan Lahouimel
  3.  Mohamed Ali Naffeti
- Team: 8
- Semester: WS21/22

Eine Software um Punktwolken zu visualisieren, analysieren und anschließend einzufärben mithilfe Machine Learning.

---

### Technologiestack:

- Qt 5.15.2
- PointCloudLibrary(PCL) 1.12.0
- Visualization Toolkit(VTK) 9.1.0
- C++17-Standard
- Python 3.8.x

---

### Voraussetzungen + Installation:

Voraussetzungen:

1. Visual Studio 2019 oder neuer
2. Qt 5.15.2
3. Git
4. vcpkg
5. PCL 1.12.0 + VTK 9.0.1
6. OpenNI2: Windows 10 SDK (10.0.18362.0)
7. Python 3.8.x


**Einbindung Qt**

Installieren Sie [Qt 5.15.2 mit der msvc2019_64](https://www.qt.io/download) Erweiterung.

Installieren in [Qt Visual Studio Tools](https://marketplace.visualstudio.com/items?itemName=TheQtCompany.QtVisualStudioTools-19123)

**Projekt klonen**

Klonen Sie das Projekt in ihr Verzeichnis. Das Projekt noch nicht starten.

```cmd
> git clone https://gitlab.rz.htw-berlin.de/softwareentwicklungsprojekt/wise2021-22/team8.git
```

**vcpkg + packages**

Nachdem das Projekt geklont wuerd, muss vcpkg geklont+installiert werden.

- vcpkg: [Siehe Dokumentation für die Installation](https://github.com/microsoft/vcpkg).
  <!-- - **Link:** https://github.com/microsoft/vcpkg -->

Nachdem vcpkg geklont wurde, folgende packages installieren:

```cmd
> .\vcpkg\vcpkg install boost-accumulators:x64-windows
> .\vcpkg\vcpkg install boost-timer:x64-windows
> .\vcpkg\vcpkg install pcl[qt,opengl,vtk]:x64-windows
```

Damit vcpkg mit Visual Studio funktioniert, führe den folgenden Kommando aus:

```cmd
> .\vcpkg\vcpkg integrate install
```

**Python Installation**

1. Installieren Sie [Python3.8](https://www.python.org/downloads/release/python-380/) den Windows x86-64 executable installer

2. Nach der Installation im Pfad "team8\PunktewolkenSegmentierung\QtWidgetsApplicationtest" eine eigene virtuelle Umgebung anlegen. Öffne anschließend die Kommandozeile als Adminstrator und navigiere zu oben genannten Pfad.

```cmd
>  cd team8\PunktewolkenSegmentierung\QtWidgetsApplicationtest\
```

3. Geben Sie anschließend folgendes ein:

```cmd
>  {path to python.exe} -m venv venv}
```
Es muss der komplette Pfad der Python.exe ohne Klammern angegeben werden!

4. Virtuelle Umgebung aktivieren und pip upgraden/installieren

```cmd
> .\venv\Scripts\activate
(venv) > python -m pip install --upgrade pip
```

5. Alle nötigen packages installieren

```cmd
(venv) > pip install -r requirements.txt
```


**Start**

Falls Sie alle Voraussetzungen erfüllt haben. Starten Sie das Projekt und folgen Sie folgende Schritte:

1. PunktwolkeSegmentierung.sln ~ Datei öffnen
2. Gehe zu: Erweiterung > Qt VS Tools > Qt Versions: Qt Version auswählen/hinzufügen.
   Falls noch keine hinzugefügt wurde:
   1. Drücke auf: Neue Qt Version hinzufügen
   2. Unter Version: Namen eingeben
   3. Host: Windows
   4. Pfad hinzufügen, wo sich Qt5.15 msvc2019_64 befindet. "C:/Qt/5.15.2/msvc2019_64/"
   5. Drücke auf: Ok
3. Gehe zu: Project > Eigenschaften

   > Qt Project Settings:

   - Qt Installation: Qt Version auswählen
   - Qt Modules: core;gui;widgets
   - Build Config: Debug

   > VC++ Directories:

   - pcl include pfad einfügen
   - vtk include pfad einfügen
   - qt include pfad einfügen

   > C/C++

   - pcl include pfad einfügen
   - vtk include pfad einfügen
   - qt include pfad einfügen

4. Projekt bauen/starten

**Testdaten**

In der HTW-Cloud sind die Testdaten zu finden.
-> Team8
