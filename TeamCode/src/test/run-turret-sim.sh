#!/usr/bin/env bash
# Off-robot check of the turret AprilTag lock (TurretSimCheck). No robot needed.
# Run from Git Bash:  bash TeamCode/src/test/run-turret-sim.sh
set -e
cd "$(dirname "$0")/../../.."
./gradlew :TeamCode:compileDebugJavaWithJavac --offline -q

# SDK + Pedro jars that Gradle already downloaded
lib() { find ~/.gradle/caches -path "$1" | head -1; }
SEP=":"; case "$(uname -s)" in MINGW*|MSYS*|CYGWIN*) SEP=";";; esac
CP="TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes"
for j in "$(lib '*/RobotCore-11.0.0/jars/classes.jar')" "$(lib '*/Hardware-11.0.0/jars/classes.jar')" \
         "$(lib '*/com.pedropathing/core/*/core-*[0-9].jar')" \
         "TeamCode/build/intermediates/assets/debug/mergeDebugAssets/java/android.jar"; do
  [ "$SEP" = ";" ] && j="$(cygpath -w "$j")"
  CP="$CP$SEP$j"
done

OUT=build/turret-sim
mkdir -p "$OUT"
javac -nowarn -encoding UTF-8 -cp "$CP" -d "$OUT" TeamCode/src/test/java/org/firstinspires/ftc/teamcode/biobuzz/TurretSimCheck.java
java -Dstdout.encoding=UTF-8 -cp "$OUT$SEP$CP" org.firstinspires.ftc.teamcode.biobuzz.TurretSimCheck
