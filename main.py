import socket

CONNECTION_STRING_ERROR = "E00"
CHANGE_SETTINGS_ERROR   = "E01"
OK_STATUS               = "OK"


def connectSocket(hostName: str = "localhost", port: int = 21833) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((hostName, port))
    return s


def closeSocket(s: socket.socket):
    s.close()
    return None


def send_recv(s: socket.socket, msg: str = "Holla"):

    s.sendall(bytes(msg, encoding='utf-8'))
    # receive +
    while (str(s.recv(1024), encoding='utf-8') != "+"):
        pass
    result = str(s.recv(1024), encoding='utf-8') # receive data
    s.sendall(bytes("+", encoding='utf-8')) # send +
    return result

def playload2msg(payload:str):
    result = '$'
    cksum = 0;

    for char in payload:
        cksum = (ord(char) + cksum) % 256;
        result += char

    result += '#'
    result += hex(cksum)[2:].lower()
    return result


if __name__ == '__main__':

    s = connectSocket("localhost", 21833)
    #s = connectSocket("localhost", 4242)

    error_connect_payload = "u2:COM5;012232"
    right_connect_payload = "u2:COM4;000400"
    comport_payload       = "u1:"
    # a = playload2msg(CONNECTION_STRING_ERROR)
    # b = playload2msg(CHANGE_SETTINGS_ERROR)
    # c = playload2msg(OK_STATUS)
    ans = send_recv(s, playload2msg(comport_payload))
    print(f"recv: {ans}")

    ans = send_recv(s, playload2msg(error_connect_payload))
    print(ans)
    if(ans == playload2msg(CONNECTION_STRING_ERROR)):
        ans = send_recv(s, msg=playload2msg(right_connect_payload))

    print(ans)
    closeSocket(s)

