import socket

# Configurações
IP_PC = "0.0.0.0"
PORTA = 5005

# Criar o socket TCP
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Permite reiniciar o servidor sem erro de "Porta em uso"
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((IP_PC, PORTA))
server_socket.listen(1)

print(f"Servidor TCP iniciado em {IP_PC}:{PORTA}")
print("À espera de ligação do ESP32...")

try:
    while True:
        client_socket, addr = server_socket.accept()
        print(f"\n[NOVA CONEXÃO] Cliente: {addr}")

        try:
            while True:
                # Recebe os bytes
                dados_brutos = client_socket.recv(1024)

                if not dados_brutos:
                    break  # Conexão fechada pelo cliente

                # Decodifica e limpa espaços/quebras de linha (\r \n)
                mensagem = dados_brutos.decode('utf-8').strip()

                # Só processa se a mensagem não estiver vazia
                if mensagem:
                    print(f"[RECEBIDO] {mensagem}")

                    # Envia confirmação de volta
                    resposta = "OK\n"
                    client_socket.send(resposta.encode())

        except ConnectionResetError:
            print("[ERRO] O ESP32 perdeu a ligação abruptamente.")
        finally:
            client_socket.close()
            print("[INFO] Ligação encerrada. Aguardando novo ciclo...")

except KeyboardInterrupt:
    print("\n[STOP] Servidor desligado pelo utilizador.")
finally:
    server_socket.close()