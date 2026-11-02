#!/bin/bash
# Ejecuta las simulaciones de un archivo de órdenes, cada una en su carpeta.
#
# Uso: experimentos/correr.sh <árbol de ns-3> <archivo de órdenes> [carpeta de salida] [procesos]
#
# Cada línea del archivo es el identificador de la simulación seguido de los
# argumentos de dv-cl-campaign-example. Las líneas que empiezan con # se omiten.
# La simulación escribe sus resultados (CSV y resumen JSON) en <salida>/<identificador>.
set -eu

NS3=$(cd "$1" && pwd)
LISTA=$2
SALIDA=${3:-salida}
PROCESOS=${4:-$(nproc)}

BIN="$NS3/build/contrib/dv-cl/examples/ns3-dev-dv-cl-campaign-example-default"
[ -x "$BIN" ] || { echo "No se encuentra $BIN. Compile primero (ver README.md)."; exit 1; }
export BIN LD_LIBRARY_PATH="$NS3/build/lib"

mkdir -p "$SALIDA"
SALIDA=$(cd "$SALIDA" && pwd)
export SALIDA

grep -v '^#' "$LISTA" | grep -v '^$' | xargs -P "$PROCESOS" -L 1 bash -c '
  id=$0
  mkdir -p "$SALIDA/$id"
  cd "$SALIDA/$id" && "$BIN" "$@" > salida.log 2>&1
  echo "$id rc=$?"
'
