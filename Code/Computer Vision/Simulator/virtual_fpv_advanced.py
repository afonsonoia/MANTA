import numpy as np
import requests
import math
import time
from PIL import Image
import io

try:
    import pyvista as pv
except ImportError:
    print("Erro: A biblioteca 'pyvista' não está instalada.")
    print("A instalação deve estar a terminar em background. Tenta correr novamente daqui a uns segundos.")
    exit(1)

def deg2num(lat_deg, lon_deg, zoom):
    """Converte coordenadas GPS para Tiles"""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def fetch_elevation(lat, lon, zoom=14):
    """Faz download do modelo de elevação real via Mapzen/AWS"""
    print(f"[{lat}, {lon}] A transferir o mapa topográfico de Chão das Feiteiras...")
    x, y = deg2num(lat, lon, zoom)
    url = f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{zoom}/{x}/{y}.png"
    
    resp = requests.get(url)
    if resp.status_code != 200:
        raise Exception(f"HTTP Error {resp.status_code}")
    
    img = Image.open(io.BytesIO(resp.content))
    img_arr = np.array(img)
    
    # Descodificação Terrarium -> Metros MSL
    R = img_arr[:,:,0].astype(np.float64)
    G = img_arr[:,:,1].astype(np.float64)
    B = img_arr[:,:,2].astype(np.float64)
    elevation = (R * 256.0 + G + B / 256.0) - 32768.0
    return elevation

def main():
    lat = 32.723
    lon = -16.892
    
    elev_data = fetch_elevation(lat, lon, zoom=14)
    
    # Vamos usar a resolução máxima para o PyVista, pois é um motor 3D poderoso
    Z = elev_data
    x_len, y_len = Z.shape
    
    # Um tile zoom 14 na Madeira tem ~3km (3000 metros) de aresta
    size_m = 3000 
    
    # Criar a grelha de coordenadas do terreno
    x = np.linspace(0, size_m, x_len)
    y = np.linspace(0, size_m, y_len)
    x, y = np.meshgrid(x, y)

    # Inicializar a malha do PyVista
    print("A inicializar o motor 3D PyVista...")
    grid = pv.StructuredGrid(x, y, Z)
    grid.point_data['Elevation'] = Z.flatten() # Para dar cores

    plotter = pv.Plotter(title="MANTA - Simulador Completo FPV (PyVista)")
    plotter.add_mesh(grid, scalars='Elevation', cmap='terrain', show_edges=False, lighting=True)
    
    # Mudar o fundo para simular um céu e disfarçar o fim do mapa
    plotter.set_background('lightblue')

    # ==========================================
    # LÓGICA DE SIMULAÇÃO DO AVIÃO E DA CÂMARA
    # ==========================================
    
    # Estado inicial do Avião
    plane = {
        'x': 800.0,       # Começar mais dentro do mapa
        'y': 800.0,       # Começar mais dentro do mapa
        'alt': 1800.0,    
        'speed': 2.0,     # Reduzido de 8.0 para 2.0 (voo muito mais suave e realista, ~216 km/h)
        'heading': 45.0   
    }

    print("Simulação Pronta! O avião está a descolar virtualmente.")
    print("Vai seguir o terreno e patrulhar o vale a 120m de altura do solo.")

    def update_flight(step):
        # 1. Movimento X e Y do avião (Cinemática baseada no Heading)
        rad = math.radians(plane['heading'])
        plane['x'] += plane['speed'] * math.cos(rad)
        plane['y'] += plane['speed'] * math.sin(rad)
        
        # Obter índices no array correspondentes ao X/Y atual para ler o chão
        ix = int(plane['x'] / size_m * (x_len - 1))
        iy = int(plane['y'] / size_m * (y_len - 1))
        
        # 2. Prevenir saída dos limites do mapa (Geo-fence virtual)
        margin = 60
        if ix <= margin or ix >= x_len - margin or iy <= margin or iy >= y_len - margin:
            # Em vez de um "salto" instantâneo de 150 graus, o avião curva de forma fluída (3 graus por frame)
            plane['heading'] += 3.0 
            plane['heading'] %= 360
            
        # 3. Lógica de "Terrain Following" (Prevenção de Colisão)
        ground_alt = Z[iy, ix]
        target_alt = ground_alt + 120 # O avião quer voar sempre 120 metros acima do chão
        
        # Simular o tempo de resposta do elevador (PID muito básico)
        plane['alt'] += (target_alt - plane['alt']) * 0.03
        
        # 4. Atualizar a câmara PyVista para a posição exata do "nariz" do avião
        cam_pos = [plane['x'], plane['y'], plane['alt']]
        
        # O Ponto Focal (para onde estamos a olhar)
        look_dist = 500 # A câmara olha 500 metros para a frente
        focal_point = [
            plane['x'] + look_dist * math.cos(rad),
            plane['y'] + look_dist * math.sin(rad),
            plane['alt'] - 20 # Nariz ligeiramente inclinado para baixo
        ]
        
        plotter.camera.position = cam_pos
        plotter.camera.focal_point = focal_point
        plotter.camera.up = (0, 0, 1)

    print("A iniciar o loop de voo...")
    
    # Método mais fiável para animar a câmara em todas as versões do PyVista
    plotter.show(auto_close=False, interactive_update=True)
    
    step = 0
    try:
        while True:
            update_flight(step)
            plotter.update()
            step += 1
            time.sleep(0.03) # 30 FPS
    except Exception:
        # A janela foi fechada pelo utilizador
        pass

if __name__ == '__main__':
    main()
