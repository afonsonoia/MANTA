import socket

# Config
IP_PC = "0.0.0.0"
PORTA = 5005

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((IP_PC, PORTA))
server_socket.listen(1)

print(f"TCP server started at {IP_PC}:{PORTA}")
print("Waiting for ESP32 connection...")

try:
    while True:
        client_socket, addr = server_socket.accept()
        print(f"\n[NEW CONNECTION] Client: {addr}")

        try:
            while True:
                dados_brutos = client_socket.recv(1024)

                if not dados_brutos:
                    break  # Connection closed by client

                mensagem = dados_brutos.decode('utf-8').strip()

                if mensagem:
                    print(f"[RECEIVED] {mensagem}")
                    resposta = "OK\n"
                    client_socket.send(resposta.encode())

        except ConnectionResetError:
            print("[ERROR] ESP32 connection lost abruptly.")
        finally:
            client_socket.close()
            print("[INFO] Connection closed. Waiting for new cycle...")

except KeyboardInterrupt:
    print("\n[STOP] Server stopped by user.")
finally:
    server_socket.close()