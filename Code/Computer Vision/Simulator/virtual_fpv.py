import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import requests
import math
from PIL import Image
import io

def deg2num(lat_deg, lon_deg, zoom):
    """Converte latitude e longitude para coordenadas de Tiles (Mapzen/OSM)"""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def fetch_elevation(lat, lon, zoom=14):
    """Descarrega dados reais de elevação (DEM) da Ilha da Madeira via AWS Open Data"""
    print(f"A descarregar dados de topografia para as coordenadas [{lat}, {lon}]...")
    x, y = deg2num(lat, lon, zoom)
    # Mapzen Terrain Tiles na Amazon S3 (Acesso Público Gratuito)
    url = f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{zoom}/{x}/{y}.png"
    
    resp = requests.get(url)
    if resp.status_code != 200:
        raise Exception(f"Falha ao descarregar o mapa: HTTP {resp.status_code}")
    
    img = Image.open(io.BytesIO(resp.content))
    img_arr = np.array(img)
    
    # Descodificar o formato Terrarium para altitude real em metros
    # Fórmula: (Red * 256 + Green + Blue / 256) - 32768
    R = img_arr[:,:,0].astype(np.float64)
    G = img_arr[:,:,1].astype(np.float64)
    B = img_arr[:,:,2].astype(np.float64)
    elevation = (R * 256.0 + G + B / 256.0) - 32768.0
    return elevation

def main():
    # Coordenadas: Chão das Feiteiras, Ilha da Madeira (Alta Montanha)
    lat = 32.723
    lon = -16.892
    
    try:
        elev_data = fetch_elevation(lat, lon, zoom=14)
    except Exception as e:
        print(f"Erro: {e}")
        return

    # Reduzir a resolução da malha para o matplotlib não arrastar o PC
    # Vamos usar apenas 1 em cada 3 pontos (escala a grelha de 256 para ~85)
    skip = 3
    Z = elev_data[::skip, ::skip]
    x_len, y_len = Z.shape
    X, Y = np.meshgrid(np.arange(x_len), np.arange(y_len))

    print(f"Mapa carregado. Altitude mínima: {np.min(Z):.0f}m, máxima: {np.max(Z):.0f}m.")
    print("A gerar o Virtual FPV 3D...")

    # Configurar o gráfico 3D
    plt.style.use('dark_background') # Fica com aspeto mais "Ground Station"
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    
    # Renderizar as montanhas com paleta de cores 'terrain'
    surf = ax.plot_surface(X, Y, Z, cmap='terrain', edgecolor='none', alpha=0.9)
    
    # Estilizar a janela
    ax.set_title("MANTA - Virtual FPV Ground Station\nLocal: Chão das Feiteiras (Madeira)", fontsize=14, color='white')
    ax.set_zlim(np.min(Z) - 100, np.max(Z) + 500) # Espaço no ar para "voar"
    ax.axis('off') # Esconder os eixos normais X Y Z para maior imersão
    
    # Função responsável por animar a câmara
    def update_camera(frame):
        # Simulamos a atitude do avião:
        # azim = Rotação do avião no vale (Yaw)
        # elev = Inclinação (Pitch)
        azim = 30 + frame * 0.4 
        pitch = 30 + 15 * math.sin(frame * 0.05) # Oscila o nariz do avião suavemente
        
        ax.view_init(elev=pitch, azim=azim)
        return fig,

    # Criar a animação (simulação de voo a ~20 FPS)
    print("A iniciar simulação de voo. Podes fechar a janela para parar.")
    ani = animation.FuncAnimation(fig, update_camera, frames=500, interval=50, blit=False)
    
    plt.show()

if __name__ == "__main__":
    main()
