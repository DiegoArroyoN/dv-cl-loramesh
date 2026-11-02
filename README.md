# DV-CL para LoRa Mesh

Código de la tesis *Protocolo DV cross-layer con métrica compuesta y MAC
consciente de duty cycle para redes LoRa Mesh* (Diego Arroyo Navarrete,
Magíster en Ciencias de la Ingeniería con mención en Ingeniería Eléctrica,
Universidad de Concepción, 2026, profesor guía Gabriel Saavedra).

DV-CL es un protocolo de vector distancia para redes LoRa Mesh. Elige las rutas
con una métrica compuesta, que suma el tiempo en aire de cada enlace, un costo
fijo por salto y una penalización por la batería del siguiente salto, y accede
al medio con CSMA/CAD bajo el límite de duty cycle de EU868.

## Contenido

- `contrib/dv-cl/`: el módulo de ns-3 con el protocolo, el modelo de energía,
  los ejemplos y doce conjuntos de pruebas.
- `patches/lorawan.patch`: los cambios al módulo `lorawan` de ns-3 que usa el
  módulo (recepción en varios SF, sensibilidad del SX1276 y modelos de
  interferencia).
- `experimentos/`: las órdenes de las 6120 simulaciones de la tesis, un archivo
  por experimento, y `correr.sh`, que las ejecuta.

## Compilación

Se necesita ns-3.46 (commit `948aa03`) y el módulo `lorawan` de
`signetlabdei/lorawan` (commit `1809ba2`) con el parche de este repositorio.
Desde la carpeta en que se quiera dejar ns-3:

```bash
git clone https://gitlab.com/nsnam/ns-3-dev.git
cd ns-3-dev
git checkout 948aa03
git clone https://github.com/signetlabdei/lorawan.git src/lorawan
git -C src/lorawan checkout 1809ba2
git -C src/lorawan apply <este repositorio>/patches/lorawan.patch
cp -r <este repositorio>/contrib/dv-cl contrib/
./ns3 configure --enable-tests --enable-examples
./ns3 build dv-cl dv-cl-campaign-example
```

Las pruebas se ejecutan con:

```bash
for s in dv-cl dv-cl-aloha dv-cl-conservation dv-cl-convergence dv-cl-energy dv-cl-lorawan-equiv dv-cl-mac dv-cl-region dv-cl-routing dv-cl-shadowing dv-cl-shutdown dv-cl-wire; do ./test.py -s $s; done
```

## Experimentos

Cada archivo de `experimentos/` corresponde a un experimento de la tesis
(tabla 6.2) y tiene una línea por simulación, con su identificador y los
argumentos de `dv-cl-campaign-example`. Por ejemplo, desde este repositorio:

```bash
experimentos/correr.sh <ruta a ns-3-dev> experimentos/cadencia_beacons.txt salida/cadencia
```

Cada simulación escribe en su propia carpeta los registros del recolector, con
cada transmisión, recepción, ruta y retardo, y un resumen en JSON con la
entrega, la latencia, la energía, la vida útil (FND y T50), el plano de control
y los descartes según su motivo.

Dos opciones del ejemplo controlan la energía:

- `--shutdownOnDepletion=true` apaga los nodos que agotan su batería. Lo usan
  todas las simulaciones de larga duración de la tesis.
- `--idleCurrentMa` fija la corriente con que se cobra la escucha del canal.
  La tesis usa 10.3 mA, la corriente de recepción del SX1276.

Las simulaciones de carga liviana usan ambas opciones en su valor por defecto.
Los experimentos de condiciones de los capítulos 5 y 8 provienen del artículo
para *Ad Hoc Networks* y no se incluyen aquí.

## Licencia

GPL-2.0-only, como ns-3. Ver `LICENSE`.
