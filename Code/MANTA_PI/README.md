# MANTA_PI — Companion Computer Software Stack

Esta pasta contém o software de bordo concebido para correr no **Raspberry Pi 3 A+** com câmara **Sony IMX378-79** no UAV MANTA.

---

## 🛡️ 1. Proteção Contra Quebras de Energia (Sem Corrupção)
Em voos com UAVs ou aeromodelos, a bateria LiPo pode desencaixar-se ou a alimentação cortar bruscamente no pouso/impacto.
- **Problema do MP4 normal**: O índice (`moov` atom) só é gravado no fecho limpo do ficheiro. Se a energia for cortada, o ficheiro fica corrompido e ilegível.
- **Solução implementada (`.mkv` + `--flush`)**:
  - Utiliza o contentor **Matroska (`.mkv`)**, que grava cada bloco de fotogramas de forma autónoma e contínua.
  - A flag `--flush` força o sistema operativo a descarregar imediatamente os dados da RAM para a memória flash do cartão SD.
  - **Resultado**: Mesmo com corte súbito de alimentação a meio do voo, **o vídeo permanece 100% íntegro e reproduzível até ao último segundo antes do corte**.

---

## ✈️ 2. Otimização Anti-Vibração & Anti-Jello (Motor UAV)
Os motores elétricos de alta rotação provocam vibrações de alta frequência na fuselagem que causam o efeito ondulado (*jello effect*) em sensores CMOS Rolling Shutter.
Os scripts implementam as seguintes mitigações óticas e de leitura do sensor IMX378:

1. **Modo de Leitura Rápido (2x2 Pixel Binning 2028x1080)**:
   - O sensor IMX378 lê as linhas muito mais depressa neste modo do que na resolução total (4056x3040), cortando o desvio temporal de varrimento para metade e reduzindo drasticamente o *jello*.
2. **Prioridade de Obturador Rápido (`--exposure sport`)**:
   - Força o algoritmo de exposição automática a manter o tempo de abertura do obturador (*shutter speed*) no mínimo absoluto possível (aumentando ganho analógico em vez de arrastar o tempo de exposição).
   - Para dias claros com muito sol, pode ser passado um shutter fixo ainda mais rápido (ex: `SHUTTER_US=2000` -> 1/500s ou `1000` -> 1/1000s) para congelar cada fotograma.
3. **Denoise Temporal Desligado (`--denoise cdn_off`)**:
   - Evita que filtros de suavização misturem ruído de vibração entre frames, eliminando o efeito "fantasma" ou borrão de contornos.
4. **Codificação H.264 por Hardware (`bcm2835-codec`)**:
   - Codificação acelerada pelo hardware do processador BCM2837, sem sobrecarregar a CPU nem perder fotogramas.

---

## 🚀 3. Ficheiros Incluídos

| Ficheiro | Descrição |
|---|---|
| [`record_flight.sh`](file:///c:/Users/Afonso%20Noia/PycharmProjects/BlueSky/Code/MANTA_PI/record_flight.sh) | Script Bash de gravação de voo com data/hora automática e proteção de disco. |
| [`record_flight.py`](file:///c:/Users/Afonso%20Noia/PycharmProjects/BlueSky/Code/MANTA_PI/record_flight.py) | Controlador Python com argumentos CLI (`--fps`, `--shutter`, `--duration`). |
| [`stream_flight.sh`](file:///c:/Users/Afonso%20Noia/PycharmProjects/BlueSky/Code/MANTA_PI/stream_flight.sh) | Script para emissão de vídeo em direto de latência mínima para FPV via SSH. |
| [`manta-record.service`](file:///c:/Users/Afonso%20Noia/PycharmProjects/BlueSky/Code/MANTA_PI/manta-record.service) | Ficheiro de serviço systemd para iniciar gravação automática ao ligar a bateria. |

---

## 📥 4. Como Sincronizar Apenas Esta Pasta no Raspberry Pi (Sparse-Checkout)

No terminal do Raspberry Pi:
```bash
# 1. Clonar apenas os metadados do repositório (sem descarregar ficheiros das outras pastas)
git clone --filter=blob:none --no-checkout https://github.com/afonsonoia/MANTA.git
cd MANTA

# 2. Configurar para extrair exclusivamente a pasta Code/MANTA_PI
git sparse-checkout init --cone
git sparse-checkout set "Code/MANTA_PI"

# 3. Extrair os ficheiros do ramo principal
git checkout main
```

Para atualizar no futuro:
```bash
git pull
```
*(Apenas esta pasta será atualizada; todo o restante repositório é ignorado).*

---

## 🎬 5. Como Utilizar

### Iniciar Gravação Manual de Voo:
```bash
# No Raspberry Pi:
cd ~/MANTA/Code/MANTA_PI
chmod +x *.sh
./record_flight.sh
```
Os vídeos serão guardados com data e hora em `/home/pc/flight_videos/manta_flight_YYYYMMDD_HHMMSS.mkv`.

Para gravar com shutter fixo ultra-rápido (ex: voo em dia com sol muito brilhante, congelando vibrações):
```bash
SHUTTER_US=1500 ./record_flight.sh  # Shutter a 1/666s
```

### Iniciar Gravação Automática no Arranque (Opcional):
Para que o Raspberry Pi comece a gravar mal ligas a bateria sem precisares de computador nem SSH:
```bash
sudo cp manta-record.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable manta-record.service
```
*(Para parar ou ver estado: `sudo systemctl status manta-record`)*.
