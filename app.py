from flask import Flask, request, jsonify, render_template, send_from_directory
import requests
import openrouteservice
from openrouteservice import convert
import folium
import math
import os
import heapq

app = Flask(__name__)

ORS_API_KEY = "5b3ce3597851110001cf6248f555579aacd34d039838693d5989d654"
client = openrouteservice.Client(key=ORS_API_KEY)

def geocode_ciudad(ciudad):
    url = f"https://nominatim.openstreetmap.org/search?format=json&q={ciudad}, Mexico"
    response = requests.get(url, headers={"User-Agent": "CarlosEduardoApp"})
    data = response.json()
    if data:
        lat = float(data[0]['lat'])
        lon = float(data[0]['lon'])
        return (lon, lat)  # orden correcto para ORS
    return None

def obtener_ruta(partida_coords, destino_coords):
    try:
        print(f"Coordenadas para ORS: partida={partida_coords}, destino={destino_coords}")
        coords = [partida_coords, destino_coords]  # ya están en [lon, lat]
        route = client.directions(coords, radiuses=[1000, 1000])
        distancia_valor = route['routes'][0]['summary']['distance'] / 1000
        distancia = f"{distancia_valor:.2f} km"
        pasos = route['routes'][0]['segments'][0]['steps']
        ruta_nombres = [step['instruction'] for step in pasos]
        ruta = " → ".join(ruta_nombres)
        coordenadas = convert.decode_polyline(route['routes'][0]['geometry'])['coordinates']
        coordenadas = [(latlng[1], latlng[0]) for latlng in coordenadas]
        return ruta, distancia, distancia_valor, coordenadas
    except Exception as e:
        print("Error en obtener_ruta:", e)
        return None, None, None, None

def dibujar_ruta_en_mapa(coordenadas):
    mapa = folium.Map(location=coordenadas[0], zoom_start=10)
    folium.Marker(coordenadas[0], popup="Inicio", icon=folium.Icon(color='red')).add_to(mapa)
    folium.Marker(coordenadas[-1], popup="Destino", icon=folium.Icon(color='green')).add_to(mapa)
    folium.PolyLine(locations=coordenadas, color='blue').add_to(mapa)
    ruta_html_path = os.path.join("static", "ruta_interactiva.html")
    mapa.save(ruta_html_path)

def distancia_euclidiana(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def distancia_manhattan(p1, p2):
    return abs(p2[0] - p1[0]) + abs(p2[1] - p1[1])

def construir_grafo(coordenadas, costos, tiempos, pesos):
    grafo = {}
    n = len(coordenadas)
    for i in range(n):
        grafo[i] = {}
        for j in range(n):
            if i != j:
                # Distancias base (euclidiana para Dijkstra, Manhattan para Manhattan)
                dist = distancia_euclidiana(coordenadas[i], coordenadas[j])
                manh = distancia_manhattan(coordenadas[i], coordenadas[j])
                # Calculamos el peso ponderado con costo, tiempo
                peso = (costos[i][j] * pesos['costo'] +
                        tiempos[i][j] * pesos['tiempo'])
                grafo[i][j] = peso
    return grafo

def dijkstra(grafo, inicio, fin):
    distancias = {n: float("inf") for n in grafo}
    distancias[inicio] = 0
    anterior = {n: None for n in grafo}
    cola = [(0, inicio)]

    while cola:
        actual_dist, actual_nodo = heapq.heappop(cola)
        if actual_nodo == fin:
            break
        if actual_dist > distancias[actual_nodo]:
            continue
        for vecino, peso in grafo[actual_nodo].items():
            nueva_dist = actual_dist + peso
            if nueva_dist < distancias[vecino]:
                distancias[vecino] = nueva_dist
                anterior[vecino] = actual_nodo
                heapq.heappush(cola, (nueva_dist, vecino))

    # Reconstruir ruta
    camino = []
    actual = fin
    while actual is not None:
        camino.append(actual)
        actual = anterior[actual]
    camino.reverse()
    return camino, distancias[fin]
def calcular_distancias_matrices(coordenadas):
    n = len(coordenadas)
    costos = [[0]*n for _ in range(n)]
    tiempos = [[0]*n for _ in range(n)]

    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            # Aquí ejemplo simple:
            # costo = distancia euclidiana * 10 (peso monetario)
            costos[i][j] = distancia_euclidiana(coordenadas[i], coordenadas[j]) * 10
            # tiempo = distancia euclidiana * 2 (minutos)
            tiempos[i][j] = distancia_euclidiana(coordenadas[i], coordenadas[j]) * 2
    return costos, tiempos

def calcular_distancia_ruta(coordenadas, ruta):
    total = 0
    for i in range(len(ruta) - 1):
        total += distancia_euclidiana(coordenadas[ruta[i]], coordenadas[ruta[i+1]])
    return total

@app.route('/')
def index():
    return render_template("index.html")

@app.route('/ruta', methods=['POST'])
def calcular_ruta():
    data = request.json
    partida = data['partida']
    destino = data['destino']
    pesos = data.get('pesos', {'costo':1, 'tiempo':1})

    coords_partida = geocode_ciudad(partida)
    coords_destino = geocode_ciudad(destino)
    if not coords_partida or not coords_destino:
        return jsonify({'error': 'No se encontraron coordenadas'}), 400

    ruta, distancia, distancia_valor, coordenadas = obtener_ruta(coords_partida, coords_destino)
    if not ruta:
        return jsonify({'error': 'No se encontró una ruta válida'}), 400

    dibujar_ruta_en_mapa(coordenadas)

    costos, tiempos = calcular_distancias_matrices(coordenadas)

    # Construimos grafos para Dijkstra y Manhattan con pesos ponderados
    grafo = construir_grafo(coordenadas, costos, tiempos, pesos)

    # Dijkstra optimiza con pesos personalizados
    ruta_dijkstra, costo_dijkstra = dijkstra(grafo, 0, len(coordenadas)-1)

    # Para Manhattan, hacemos lo mismo pero grafo con distancia Manhattan y mismo peso
    grafo_manhattan = {}
    n = len(coordenadas)
    for i in range(n):
        grafo_manhattan[i] = {}
        for j in range(n):
            if i != j:
                peso = (distancia_manhattan(coordenadas[i], coordenadas[j]) * 
                        (pesos['costo'] + pesos['tiempo']) / 2)
                grafo_manhattan[i][j] = peso
    ruta_manhattan, costo_manhattan = dijkstra(grafo_manhattan, 0, n-1)

    return jsonify({
        'ruta_or': ruta,
        'distancia': distancia,
        'dijkstra': {
            'ruta': ruta_dijkstra,
            'costo': costo_dijkstra
        },
        'manhattan': {
            'ruta': ruta_manhattan,
            'costo': costo_manhattan
        }
    })

@app.route('/static/<path:filename>')
def static_files(filename):
    return send_from_directory("static", filename)

if __name__ == "__main__":
    app.run(debug=True)
